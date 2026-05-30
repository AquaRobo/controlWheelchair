import time
import os as _os
import ctypes as _ctypes
import numpy as np
import sounddevice as sd

# Suppress the verbose ALSA/PortAudio error messages that spam stderr
# even for routine probe failures.  The errors are still caught in Python;
# this only silences the C-level fprintf calls.
try:
    _asound = _ctypes.CDLL('libasound.so.2')
    _asound.snd_lib_error_set_handler(_ctypes.cast(None, _ctypes.c_void_p))
except Exception:
    pass

import torch, torchaudio
from faster_whisper import WhisperModel
from collections import deque

from cv.helper.AudioBuffer import AudioBuffer
from cv.helper.WakeWordModel import WakeWordModel
from cv.helper.AudioProcessor import AudioProcessor
from cv.helper.AudioCleaner import AudioCleaner

from typing import Generator


# =====================================================
# VOTE BUFFER
# =====================================================
class VoteBuffer:
    def __init__(self, window_size: int = 5, required_votes: int = 4):
        self.window_size    = window_size
        self.required_votes = required_votes
        self.votes          = deque(maxlen=window_size)

    def push(self, label: str, confidence: float, margin: float):
        self.votes.append((label, confidence, margin))

    def get_consensus(self, min_confidence: float, min_margin: float):
        if len(self.votes) < self.window_size:
            return None
        valid = [
            (label, conf, margin)
            for label, conf, margin in self.votes
            if label != "Other"
            and conf   >= min_confidence
            and margin >= min_margin
        ]
        if len(valid) < self.required_votes:
            return None
        labels = [v[0] for v in valid]
        if len(set(labels)) != 1:
            return None
        avg_conf   = sum(v[1] for v in valid) / len(valid)
        avg_margin = sum(v[2] for v in valid) / len(valid)
        return labels[0], avg_conf, avg_margin

    def clear(self):
        self.votes.clear()


# =====================================================
# SPEECH RECOGNIZER
# =====================================================
class SpeechRecognizer:
    def __init__(
        self,
        config: dict,
        model_path: str,
        device_index: int | str | None = None,
        device_channels: int = 1,
        device_rate: int | None = None,
    ):
        self.config          = config
        self.model_path      = model_path
        self.device_index    = device_index   # int, str pulse-source, or None
        self.device_channels = device_channels

        self.device = config.get(
            "device",
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        # Use the probed hardware rate if the device needs it, otherwise
        # fall back to what the config requests.
        config_rate = config.get("sample_rate", 44100)
        self.sample_rate        = device_rate if device_rate else config_rate
        self.target_sample_rate = config.get("target_sample_rate", 16000)
        self.duration           = config.get("listen_duration", 2)

        # ── Audio cleaner ─────────────────────────────────────────────────
        self.audio_cleaner = AudioCleaner(
            sample_rate  = self.sample_rate,
            hum_freq     = config.get("hum_freq", "50hz"),
            hp_cutoff    = config.get("hp_cutoff_freq", 80.0),
            hp_order     = config.get("hp_order", 4),
            notch_q      = config.get("notch_q", 35.0),
            n_harmonics  = config.get("n_harmonics", 4),
            ss_alpha     = config.get("ss_alpha", 2.0),
            ss_beta      = config.get("ss_beta", 0.01),
            noise_frames = config.get("noise_frames", 40),
        )

        # ── Buffers ───────────────────────────────────────────────────────
        self.audio_buffer     = AudioBuffer(self.sample_rate, self.duration)
        self.command_duration = config.get("command_duration", 5.0)
        self.command_buffer   = AudioBuffer(self.sample_rate, self.command_duration)

        # ── Audio processor ───────────────────────────────────────────────
        self.audio_processor = AudioProcessor(
            input_sample_rate=self.sample_rate,
            target_sample_rate=self.target_sample_rate,
            duration=self.duration,
            device=self.device
        )

        # ── Wake word model ───────────────────────────────────────────────
        self.wake_word_model = WakeWordModel(num_classes=3).to(self.device)
        self.class_map       = {0: "Milo", 1: "Jarvis", 2: "Other"}

        # ── Thresholds ────────────────────────────────────────────────────
        self.min_confidence        = config.get("min_confidence", 0.92)
        self.min_margin            = config.get("min_margin", 0.75)
        self.cooldown_seconds      = config.get("cooldown_seconds", 3.0)
        self.silence_frames_needed = config.get("silence_frames_needed", 3)
        self.energy_threshold      = config.get("energy_threshold", 0.01)

        # ── Echo / reverb tail suppression ────────────────────────────────
        # After a detection, the room keeps ringing.  We wait until the
        # cleaned RMS stays below this multiplier × energy_threshold for
        # `reverb_settle_frames` consecutive frames before re-arming the
        # vote buffer.  Increase reverb_settle_frames in very live rooms.
        self.reverb_settle_frames   = config.get("reverb_settle_frames", 4)
        self.reverb_rms_multiplier  = config.get("reverb_rms_multiplier", 1.5)
        self._reverb_settle_counter = 0

        # ── Vote buffer ───────────────────────────────────────────────────
        self.vote_buffer = VoteBuffer(
            window_size    = config.get("vote_window", 5),
            required_votes = config.get("required_votes", 4)
        )

        # ── State ─────────────────────────────────────────────────────────
        self._last_detection_time = 0.0
        self._armed               = True
        self._silence_streak      = 0
        self._recording_command   = False

        # ── Whisper ───────────────────────────────────────────────────────
        self.whisper_model      = None
        self.whisper_model_size = config.get("whisper_model", "tiny.en")

        self.command_resampler = torchaudio.transforms.Resample(
            orig_freq=self.sample_rate,
            new_freq=self.target_sample_rate
        )

        self._loadWakeWordModel()
        self._loadWhisperModel()

        # ── Stream — use the user-selected device and probed config ─────────
        self.stream = sd.InputStream(
            device=self.device_index,
            samplerate=self.sample_rate,
            channels=self.device_channels,
            dtype="float32",
            callback=self._audio_callback,
        )
        self.stream.start()
        # Temporarily add this right after self.stream.start() in __init__
        print(f"Stream opened: device={self.device_index}, rate={self.sample_rate}, ch={self.device_channels}")

    # =========================================================
    # MIC SELECTION  (call this before constructing the object)
    # =========================================================

    @staticmethod
    def _alsa_capture_devices() -> list[dict]:
        """
        Run `arecord -l` and parse its output to find real ALSA capture
        devices.  Returns a list of dicts:
            {"card": int, "device": int, "name": str, "card_name": str}

        This is the only reliable way to discover USB mics on Linux — ALSA
        hw: entries in sounddevice are output-only descriptors that PortAudio
        cannot open for capture because the channel-count metadata is wrong.
        """
        import subprocess, re
        try:
            out = subprocess.check_output(
                ["arecord", "-l"], stderr=subprocess.DEVNULL, text=True
            )
        except (FileNotFoundError, subprocess.CalledProcessError):
            return []

        devices = []
        # Lines look like:
        #   card 2: Device [USB Audio Device], device 0: USB Audio [USB Audio]
        pattern = re.compile(
            r"card\s+(\d+):\s+\S+\s+\[([^\]]+)\],\s+device\s+(\d+):\s+[^\[]+\[([^\]]+)\]"
        )
        for line in out.splitlines():
            m = pattern.search(line)
            if m:
                devices.append({
                    "card":      int(m.group(1)),
                    "device":    int(m.group(3)),
                    "card_name": m.group(2).strip(),
                    "name":      m.group(4).strip(),
                })
        return devices

    @staticmethod
    def _probe_pulse_for_alsa(card: int, device: int, preferred_rate: int) -> dict | None:
        """
        Given an ALSA card/device, find the matching PulseAudio source name
        via `pactl list sources short` and probe it through sounddevice.

        PulseAudio wraps hw: devices and handles sample-rate / channel-count
        negotiation, so the probe always succeeds for a real capture device.
        Returns {"sd_name": str, "channels": int, "rate": int} or None.
        """
        import subprocess, re
        # Build the expected ALSA sink name fragment, e.g. "hw:2,0" or "alsa_input.*2.*0"
        hw_tag = f"hw:{card},{device}"
        alt_tag = f"alsa_input"   # PulseAudio source names start with this

        try:
            out = subprocess.check_output(
                ["pactl", "list", "sources", "short"],
                stderr=subprocess.DEVNULL, text=True
            )
        except (FileNotFoundError, subprocess.CalledProcessError):
            out = ""

        # pactl short format: index  name  driver  rate  state
        # e.g.:  2  alsa_input.usb-..._hw_2_0.analog-stereo  ...
        pulse_name = None
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 2:
                name = parts[1]
                # Match by card+device number embedded in the source name
                # PulseAudio replaces ":" and "," with "_", so hw:2,0 → _2_0
                tag = f"_{card}_{device}"
                if tag in name and "input" in name:
                    pulse_name = name
                    break

        # Try sounddevice by pulse source name, then by "pulse" device, then default
        candidate_sd_devices: list[str | None] = []
        if pulse_name:
            candidate_sd_devices.append(pulse_name)
        candidate_sd_devices.extend(["pulse", None])

        candidate_rates    = sorted({preferred_rate, 48000, 44100, 16000}, reverse=True)
        candidate_channels = [1, 2]

        for sd_dev in candidate_sd_devices:
            for ch in candidate_channels:
                for rate in candidate_rates:
                    try:
                        s = sd.InputStream(
                            device=sd_dev,
                            samplerate=rate,
                            channels=ch,
                            dtype="float32",
                        )
                        s.start(); s.stop(); s.close()
                        return {"sd_name": sd_dev, "channels": ch, "rate": rate,
                                "pulse_name": pulse_name}
                    except Exception:
                        continue
        return None

    @staticmethod
    def _probe_device(index: int, preferred_rate: int) -> dict | None:
        """
        Probe a sounddevice index directly.  Works for pulse/default.
        For raw ALSA hw: entries this will usually fail — use
        _probe_pulse_for_alsa() instead via select_mic_device().
        """
        candidate_rates    = sorted({preferred_rate, 48000, 44100, 16000}, reverse=True)
        candidate_channels = [1, 2]
        for ch in candidate_channels:
            for rate in candidate_rates:
                try:
                    s = sd.InputStream(
                        device=index,
                        samplerate=rate,
                        channels=ch,
                        dtype="float32",
                    )
                    s.start(); s.stop(); s.close()
                    return {"channels": ch, "rate": rate, "sd_name": index, "pulse_name": None}
                except Exception:
                    continue
        return None

    @staticmethod
    def select_mic_device(sample_rate: int = 44100) -> tuple[int | None, int, int]:
        """
        Build a unified list of input devices from two sources:

        1. ALSA (`arecord -l`) — finds real capture devices including USB mics
           that PortAudio/sounddevice incorrectly marks as output-only.
           These are opened via PulseAudio to avoid hw: strictness.

        2. sounddevice — pulse and default, which work as flexible catch-alls.

        Deduplicates so pulse/default only appear once.
        Returns (sd_device_index_or_name, channels, rate).
        sd_device is None for the system default, or a string PulseAudio
        source name, or an int sounddevice index.
        """
        import os
        default_idx = sd.default.device[0]
        sd_devices  = sd.query_devices()

        # ── 1. ALSA capture devices ───────────────────────────────────────
        alsa_devs = SpeechRecognizer._alsa_capture_devices()

        # ── 2. sounddevice pulse/default entries ──────────────────────────
        sd_flexible = []   # (sd_index, name) for pulse/default only
        for i, dev in enumerate(sd_devices):
            if dev["name"].lower() in ("pulse", "default"):
                sd_flexible.append((i, dev["name"]))

        # ── Build menu ────────────────────────────────────────────────────
        # Each entry: {"label": str, "probe_fn": callable → dict|None}
        menu: list[dict] = []

        seen_pulse = False
        for ad in alsa_devs:
            label = f"{ad['card_name']} — {ad['name']}  (hw:{ad['card']},{ad['device']})"
            card, device = ad["card"], ad["device"]
            menu.append({
                "label":    label,
                "probe_fn": lambda c=card, d=device: SpeechRecognizer._probe_pulse_for_alsa(
                    c, d, sample_rate
                ),
                "is_alsa":  True,
                "card":     card,
                "device":   device,
            })

        for sd_idx, sd_name in sd_flexible:
            menu.append({
                "label":    sd_name,
                "probe_fn": lambda i=sd_idx: SpeechRecognizer._probe_device(i, sample_rate),
                "is_alsa":  False,
                "sd_idx":   sd_idx,
            })

        # ── Probe and print ───────────────────────────────────────────────
        import sys
        # Suppress the ALSA stderr noise during probing
        devnull = open(os.devnull, "w")
        old_stderr_fd = os.dup(2)
        os.dup2(devnull.fileno(), 2)

        print("\n─── Available audio input devices ───────────────────────────")
        working: dict[int, dict] = {}   # menu index → probe result

        for mi, entry in enumerate(menu):
            result = entry["probe_fn"]()
            if result:
                status = f"OK  ({result['channels']}ch @ {result['rate']} Hz)"
                working[mi] = result
            else:
                status = "✗"
            print(f"  {mi:>2}  {entry['label']:<50}  {status}")

        # Restore stderr
        os.dup2(old_stderr_fd, 2)
        os.close(old_stderr_fd)
        devnull.close()

        print("─────────────────────────────────────────────────────────────")

        if not working:
            print("   No working input devices found — using system default.\n")
            return None, 1, sample_rate

        # Default suggestion: first working ALSA device if any, else menu[0]
        default_menu_idx = next(
            (mi for mi in working if menu[mi].get("is_alsa")),
            next(iter(working))
        )

        while True:
            raw = input(
                f"   Select number [Enter = {default_menu_idx} "
                f"({menu[default_menu_idx]['label'].split('—')[0].strip()})]: "
            ).strip()

            if raw == "":
                choice = default_menu_idx
            else:
                try:
                    choice = int(raw)
                except ValueError:
                    print("   Please enter a number.")
                    continue

            if choice not in working:
                if 0 <= choice < len(menu):
                    print(f"   Device {choice} failed the probe — pick one marked OK.")
                else:
                    print(f"   '{choice}' is out of range — try again.")
                continue

            cfg   = working[choice]
            entry = menu[choice]
            sd_dev = cfg.get("sd_name")   # str pulse name, int index, or None
            print(
                f"   Selected: {entry['label']} — "
                f"{cfg['channels']}ch @ {cfg['rate']} Hz\n"
            )
            # Convert to the int/None form __init__ expects for sd.InputStream
            # A string pulse-source name is passed directly; int or None as-is.
            return sd_dev, cfg["channels"], cfg["rate"]

    # =========================================================
    # SETUP
    # =========================================================

    def _audio_callback(self, indata, frames, time, status):
        if status:
            return
        # Mix down to mono — works for both 1-ch and 2-ch devices
        samples = indata.mean(axis=1)
        if self._recording_command:
            self.command_buffer.add(samples)
        else:
            self.audio_buffer.add(samples)

    def _loadWakeWordModel(self):
        state_dict = torch.load(self.model_path, map_location=self.device)
        self.wake_word_model.load_state_dict(state_dict)
        self.wake_word_model.eval()

    def _loadWhisperModel(self):
        self.whisper_model = WhisperModel(
            self.whisper_model_size,
            device="cpu",
            compute_type="int8"
        )

    # =========================================================
    # CALIBRATION
    # =========================================================

    def calibrate_noise_floor(self, seconds: float = 10.0):
        """
        Calibrate both the energy VAD threshold and the spectral subtractor.

        Echo fix: instead of mean × 4 (which is inflated by reverb tails
        landing in "silent" frames), we use the 25th-percentile of cleaned
        RMS values × 3.  The bottom quartile contains frames where the room
        was actually quiet; reverb-contaminated frames sit in the upper half
        and are excluded from the floor estimate.

        Increase `seconds` if you are in a very reverberant room — each
        AudioBuffer frame is ~2 s, so 10 s ≈ 5 frames minimum.
        """
        print(f"Calibrating noise floor for {seconds}s — please stay quiet...")
        deadline      = time.time() + seconds
        clean_samples = []

        while time.time() < deadline:
            if self.audio_buffer.ready():
                raw = self.audio_buffer.get()

                # Feed raw to spectral subtractor
                tensor = torch.tensor(raw, dtype=torch.float32).unsqueeze(0)
                self.audio_cleaner.calibrate(tensor)

                # Measure RMS on CLEANED audio
                cleaned   = self.audio_cleaner.process_numpy(raw)
                clean_rms = float(np.sqrt(np.mean(cleaned ** 2)))
                clean_samples.append(clean_rms)

            time.sleep(0.05)

        if not clean_samples:
            print("   Calibration failed — keeping default threshold.\n")
            return

        # ── Echo-robust floor: 25th-percentile × 3 ────────────────────────
        # In a reverberant room the mean is pulled up by echo tails.
        # The 25th-percentile catches the genuinely quiet moments.
        noise_p25 = float(np.percentile(clean_samples, 25))
        noise_mean = float(np.mean(clean_samples))
        self.energy_threshold = noise_p25 * 3.0

        status = "ready" if self.audio_cleaner.calibrated else "needs more frames (increase calibration seconds)"
        print(f"   Cleaned RMS mean     : {noise_mean:.5f}")
        print(f"   Cleaned RMS p25      : {noise_p25:.5f}  (echo-robust floor)")
        print(f"   Energy threshold     : {self.energy_threshold:.5f}  (p25 × 3)")
        print(f"   Spectral cleaner     : {status}\n")

    # =========================================================
    # HELPERS
    # =========================================================

    def _clean(self, audio: np.ndarray) -> np.ndarray:
        return self.audio_cleaner.process_numpy(audio)

    def _runWakeWordDetection(self, mel_spec: torch.Tensor) -> tuple[str, float, float]:
        prediction, confidence, margin = self.wake_word_model.predict(mel_spec)
        return self.class_map[prediction], confidence, margin

    # =========================================================
    # WAKE WORD DETECTION
    # =========================================================

    def _detect_wake_word(self) -> str:
        while True:
            if not self.audio_buffer.ready():
                continue

            raw     = self.audio_buffer.get()
            cleaned = self._clean(raw)
            rms     = float(np.sqrt(np.mean(cleaned ** 2)))

            # ── Gate 1: energy VAD (on cleaned audio) ─────────────────────
            if rms <= self.energy_threshold:
                self.vote_buffer.clear()
                self._silence_streak += 1

                # ── Reverb tail suppression ────────────────────────────────
                # After a detection the room keeps ringing.  We require the
                # cleaned RMS to stay below reverb_rms_multiplier × threshold
                # for reverb_settle_frames consecutive silent frames before
                # we allow re-arming.  This prevents the decaying echo from
                # immediately triggering another false detection.
                if not self._armed:
                    reverb_ceil = self.energy_threshold * self.reverb_rms_multiplier
                    if rms <= reverb_ceil:
                        self._reverb_settle_counter += 1
                    else:
                        # Echo tail is still too loud — reset the counter
                        self._reverb_settle_counter = 0

                    if self._reverb_settle_counter >= self.reverb_settle_frames:
                        self._armed = True
                        self._reverb_settle_counter = 0

                    # Also honour the original silence-streak re-arming
                    if self._silence_streak >= self.silence_frames_needed:
                        self._armed = True
                        self._reverb_settle_counter = 0

                # ── Adaptive noise update during silence ───────────────────
                # Gently update the spectral subtractor's noise profile
                # whenever the room is quiet.  This lets the model adapt to
                # the new room's steady-state noise without a full restart.
                tensor = torch.tensor(raw, dtype=torch.float32).unsqueeze(0)
                self.audio_cleaner.update_noise_adaptive(tensor)

                continue

            self._silence_streak      = 0
            self._reverb_settle_counter = 0

            # ── Gate 2: disarmed guard ────────────────────────────────────
            if not self._armed:
                continue

            # ── Gate 3: model inference on cleaned audio ──────────────────
            mel_spec = self.audio_processor.preprocess(cleaned)
            mel_spec = mel_spec.unsqueeze(0).to(self.device)

            label, confidence, margin = self._runWakeWordDetection(mel_spec)
            print(f"  {label:6s}  conf={confidence:.3f}  margin={margin:.3f}  rms={rms:.5f}")            

            # ── Gate 4: per-frame thresholds ──────────────────────────────
            effective_label = (
                label
                if confidence >= self.min_confidence and margin >= self.min_margin
                else "Other"
            )
            self.vote_buffer.push(effective_label, confidence, margin)

            # ── Gate 5: consensus voting ──────────────────────────────────
            consensus = self.vote_buffer.get_consensus(
                self.min_confidence, self.min_margin
            )
            if consensus is None:
                continue

            detected_label, avg_conf, avg_margin = consensus

            # ── Gate 6: cooldown ──────────────────────────────────────────
            now = time.time()
            if (now - self._last_detection_time) < self.cooldown_seconds:
                continue

            # ══ All gates passed ══════════════════════════════════════════
            self._last_detection_time   = now
            self._armed                 = False
            self._silence_streak        = 0
            self._reverb_settle_counter = 0
            self.vote_buffer.clear()

            self.audio_buffer.clear()
            self.command_buffer.clear()
            self._recording_command = True
            time.sleep(0.3)

            return detected_label

    # =========================================================
    # COMMAND RECORDING
    # =========================================================

    def _record_command(self) -> np.ndarray:
        print("Listening for command...")

        while not self.command_buffer.ready():
            time.sleep(0.05)

        audio = self.command_buffer.get()
        self.command_buffer.clear()
        self._recording_command = False

        audio = self._clean(audio)

        if self.sample_rate != self.target_sample_rate:
            tensor = torch.tensor(audio, dtype=torch.float32).unsqueeze(0)
            tensor = self.command_resampler(tensor)
            audio  = tensor.squeeze(0).numpy()

        print(f"Command captured — {len(audio) / self.target_sample_rate:.1f}s")
        return audio

    # =========================================================
    # WHISPER + COMMAND PARSING
    # =========================================================

    def _runWhisper(self, audio: np.ndarray) -> str:
        if self.whisper_model is None:
            return ""
        segments, _ = self.whisper_model.transcribe(audio, language="en")
        return " ".join(segment.text for segment in segments).strip()

    def _getCommandedRoom(self, transcription: str) -> str:
        for room in self.config.get("rooms", []):
            if room.lower() in transcription.lower():
                return room
        return ""

    def _getCommandedObject(self, transcription: str) -> str:
        for obj in self.config.get("objects", []):
            if obj.lower() in transcription.lower():
                return obj
        return ""

    def _getCommandedAction(self, transcription: str) -> str:
        for action in self.config.get("actions", []):
            if action.lower() in transcription.lower():
                return action
        return ""

    def recognizeSpeech(self) -> Generator[tuple[str, str, str], None, None]:
        wake_word = self._detect_wake_word()

        if wake_word == "Milo":
            print("Running Whisper for Milo...")
            while True:
                self.command_buffer.clear()
                self._recording_command = True
                audio         = self._record_command()
                transcription = self._runWhisper(audio)
                print(f"Transcription: {transcription}")
                room   = self._getCommandedRoom(transcription)
                action = self._getCommandedAction(transcription)
                yield room, None, action
                if action == "exit command mode":
                    break

        elif wake_word == "Jarvis":
            print("Running Whisper for Jarvis...")
            while True:
                self.command_buffer.clear()
                self._recording_command = True
                audio         = self._record_command()
                transcription = self._runWhisper(audio)
                print(f"Transcription: {transcription}")
                obj    = self._getCommandedObject(transcription)
                action = self._getCommandedAction(transcription)
                yield None, obj, action
                if action == "exit command mode":
                    break

    def end_stream(self):
        self.stream.stop()
        self.stream.close()