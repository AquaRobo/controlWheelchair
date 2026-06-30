import os
import re
import subprocess
import time
import threading
import numpy as np
import sounddevice as sd
import torch
import torchaudio
from faster_whisper import WhisperModel
from typing import Generator


class SpeechRecognizer:

    # ==================================================================
    # MIC SELECTION
    # ==================================================================

    @staticmethod
    def _alsa_capture_devices() -> list[dict]:
        """Parse `arecord -l` to find real ALSA capture devices."""
        try:
            out = subprocess.check_output(["arecord", "-l"], stderr=subprocess.DEVNULL, text=True)
        except (FileNotFoundError, subprocess.CalledProcessError):
            return []

        devices = []
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
    def _probe_alsa_via_pulse(card: int, device: int, preferred_rate: int) -> dict | None:
        """Find the PulseAudio source for an ALSA card/device and probe it."""
        try:
            out = subprocess.check_output(
                ["pactl", "list", "sources", "short"], stderr=subprocess.DEVNULL, text=True
            )
        except (FileNotFoundError, subprocess.CalledProcessError):
            out = ""

        pulse_name = None
        tag = f"_{card}_{device}"
        for line in out.splitlines():
            parts = line.split()
            if len(parts) >= 2 and tag in parts[1] and "input" in parts[1]:
                pulse_name = parts[1]
                break

        candidates = ([pulse_name] if pulse_name else []) + ["pulse", None]
        for sd_dev in candidates:
            for ch in [1, 2]:
                for rate in sorted({preferred_rate, 48000, 44100, 16000}, reverse=True):
                    try:
                        s = sd.InputStream(device=sd_dev, samplerate=rate, channels=ch, dtype="float32")
                        s.start(); s.stop(); s.close()
                        return {"sd_name": sd_dev, "channels": ch, "rate": rate}
                    except Exception:
                        continue
        return None

    @staticmethod
    def _probe_sd_device(index: int, preferred_rate: int) -> dict | None:
        """Probe a plain sounddevice index (pulse / default)."""
        for ch in [1, 2]:
            for rate in sorted({preferred_rate, 48000, 44100, 16000}, reverse=True):
                try:
                    s = sd.InputStream(device=index, samplerate=rate, channels=ch, dtype="float32")
                    s.start(); s.stop(); s.close()
                    return {"sd_name": index, "channels": ch, "rate": rate}
                except Exception:
                    continue
        return None

    @staticmethod
    def select_mic_device(preferred_rate: int = 16000) -> tuple[int | str | None, int, int]:
        """
        Print a numbered list of available input devices, let the user pick
        one, and return (sd_device, channels, rate).

        Combines ALSA capture devices (via PulseAudio) and sounddevice
        pulse/default entries so USB mics always appear regardless of how
        PortAudio enumerates them.
        """
        alsa_devs = SpeechRecognizer._alsa_capture_devices()

        sd_flexible = [
            (i, dev["name"])
            for i, dev in enumerate(sd.query_devices())
            if dev["name"].lower() in ("pulse", "default") and dev["max_input_channels"] > 0
        ]

        # Build unified menu
        menu: list[dict] = []
        for ad in alsa_devs:
            menu.append({
                "label":    f"{ad['card_name']} — {ad['name']}  (hw:{ad['card']},{ad['device']})",
                "probe_fn": lambda c=ad["card"], d=ad["device"]: SpeechRecognizer._probe_alsa_via_pulse(c, d, preferred_rate),
                "is_alsa":  True,
            })
        for sd_idx, sd_name in sd_flexible:
            menu.append({
                "label":    sd_name,
                "probe_fn": lambda i=sd_idx: SpeechRecognizer._probe_sd_device(i, preferred_rate),
                "is_alsa":  False,
            })

        # Probe each device (suppress ALSA stderr noise while doing so)
        devnull      = open(os.devnull, "w")
        old_stderr   = os.dup(2)
        os.dup2(devnull.fileno(), 2)

        working: dict[int, dict] = {}
        for mi, entry in enumerate(menu):
            result = entry["probe_fn"]()
            if result:
                working[mi] = result

        os.dup2(old_stderr, 2)
        os.close(old_stderr)
        devnull.close()

        # Print menu
        print("\n─── Available audio input devices ───────────────────────────")
        for mi, entry in enumerate(menu):
            if mi in working:
                r = working[mi]
                status = f"✓  {r['channels']}ch @ {r['rate']} Hz"
            else:
                status = "✗  unavailable"
            print(f"  {mi:>2}  {entry['label']:<52}  {status}")
        print("─────────────────────────────────────────────────────────────")

        if not working:
            print("   No working input devices found — using system default.\n")
            return None, 1, preferred_rate

        default_idx = next(
            (mi for mi in working if menu[mi].get("is_alsa")),
            next(iter(working))
        )

        while True:
            raw = input(f"   Select [Enter = {default_idx}]: ").strip()
            if raw == "":
                choice = default_idx
            else:
                try:
                    choice = int(raw)
                except ValueError:
                    print("   Please enter a number.")
                    continue

            if choice not in working:
                print(f"   {choice} is not available — choose a device marked ✓.")
                continue

            cfg = working[choice]
            print(f"   → {menu[choice]['label']}  ({cfg['channels']}ch @ {cfg['rate']} Hz)\n")
            return cfg["sd_name"], cfg["channels"], cfg["rate"]

    # ==================================================================
    # INIT
    # ==================================================================

    def __init__(
        self,
        config: dict,
        device_index: int | str | None = None,
        device_channels: int = 1,
        device_rate: int | None = None,
    ):
        self.config          = config
        self.device_index    = device_index
        self.device_channels = device_channels

        config_rate             = config.get("sample_rate", 16000)
        self.sample_rate        = device_rate if device_rate else config_rate
        self.target_sample_rate = 16000  # Whisper always wants 16 kHz

        self.wake_duration    = config.get("wake_duration", 2.0)
        self.command_duration = config.get("command_duration", 5.0)
        self.energy_threshold = config.get("energy_threshold", 0.01)
        self.cooldown_seconds = config.get("cooldown_seconds", 2.0)
        self.wake_words: dict[str, str] = config.get("wake_words", {"milo": "Milo", "jarvis": "Jarvis"})

        self._resampler = (
            torchaudio.transforms.Resample(orig_freq=self.sample_rate, new_freq=self.target_sample_rate)
            if self.sample_rate != self.target_sample_rate else None
        )

        self._wake_frames:    list[np.ndarray] = []
        self._command_frames: list[np.ndarray] = []
        self._recording_command = False
        self._last_detection_time = 0.0

        whisper_size       = config.get("whisper_model", "tiny.en")
        self.whisper_model = WhisperModel(whisper_size, device="cpu", compute_type="int8")

        self.stream = sd.InputStream(
            device=self.device_index,
            samplerate=self.sample_rate,
            channels=self.device_channels,
            dtype="float32",
            blocksize=int(self.sample_rate * 0.1),
            callback=self._audio_callback,
        )
        self.stream.start()
        print(f"[SpeechRecognizer] stream open — device={self.device_index}, rate={self.sample_rate}, ch={self.device_channels}")

    # ==================================================================
    # AUDIO CALLBACK
    # ==================================================================

    def _audio_callback(self, indata, frames, time_info, status):
        mono = indata.mean(axis=1).copy()
        if self._recording_command:
            self._command_frames.append(mono)
        else:
            self._wake_frames.append(mono)

    # ==================================================================
    # HELPERS
    # ==================================================================

    def _collect_audio(self, frames: list[np.ndarray], duration: float) -> np.ndarray:
        needed = int(self.sample_rate * duration)
        while sum(f.shape[0] for f in frames) < needed:
            time.sleep(0.02)
        audio = np.concatenate(frames)[:needed]
        frames.clear()
        return audio

    def _to_whisper(self, audio: np.ndarray) -> np.ndarray:
        if self._resampler is None:
            return audio
        t = torch.tensor(audio, dtype=torch.float32).unsqueeze(0)
        return self._resampler(t).squeeze(0).numpy()

    def _transcribe(self, audio: np.ndarray) -> str:
        segments, _ = self.whisper_model.transcribe(audio, language="en")
        return " ".join(seg.text for seg in segments).strip()

    def _match_wake_word(self, transcript: str) -> str | None:
        lower = transcript.lower()
        for phrase, label in self.wake_words.items():
            if phrase.lower() in lower:
                return label
        return None

    # ==================================================================
    # WAKE-WORD DETECTION
    # ==================================================================

    def _detect_wake_word(self) -> str:
        print("[SpeechRecognizer] listening for wake word...")
        while True:
            self._wake_frames.clear()
            audio = self._collect_audio(self._wake_frames, self.wake_duration)

            rms = float(np.sqrt(np.mean(audio ** 2)))
            if rms <= self.energy_threshold:
                continue

            if (time.time() - self._last_detection_time) < self.cooldown_seconds:
                continue

            transcript = self._transcribe(self._to_whisper(audio))
            print(f"  [wake] rms={rms:.4f}  heard: {transcript!r}")

            label = self._match_wake_word(transcript)
            if label is None:
                continue

            print(f"  [wake] detected → {label!r}")
            self._last_detection_time = time.time()
            return label

    # ==================================================================
    # COMMAND RECORDING
    # ==================================================================

    def _record_command(self) -> np.ndarray:
        print("  [command] recording...")
        self._command_frames.clear()
        self._recording_command = True
        audio = self._collect_audio(self._command_frames, self.command_duration)
        self._recording_command = False
        return self._to_whisper(audio)

    def _runWhisper(self, audio: np.ndarray) -> str:
        return self._transcribe(audio)

    # ==================================================================
    # COMMAND PARSING
    # ==================================================================

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

    # ==================================================================
    # PUBLIC API
    # ==================================================================

    def recognizeSpeech(self) -> Generator[tuple[str, str, str], None, None]:
        wake_word = self._detect_wake_word()

        if wake_word == "Milo":
            print("Running Whisper for Milo…")
            while True:
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
            print("Running Whisper for Jarvis…")
            while True:
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
        self._shutdown = True
        with self._stream_lock:
            self.stream.stop()
            self.stream.close()