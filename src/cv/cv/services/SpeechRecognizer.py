import time
import numpy as np
import sounddevice as sd
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
    def __init__(self, config: dict, model_path: str):
        self.config     = config
        self.model_path = model_path

        self.device = config.get(
            "device",
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        self.sample_rate        = config.get("sample_rate", 16000)
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
            noise_frames = config.get("noise_frames", 40),   # bumped default
        )

        # ── Buffers ───────────────────────────────────────────────────────
        self.audio_buffer     = AudioBuffer(self.sample_rate, self.duration)
        self.command_duration = config.get("command_duration", 3)
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

        self.stream = sd.InputStream(
            device=10,
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32",
            callback=self._audio_callback,
        )
        self.stream.start()

    # =========================================================
    # SETUP
    # =========================================================

    def _audio_callback(self, indata, frames, time, status):
        if status:
            return
        samples = indata[:, 0]
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

    def calibrate_noise_floor(self, seconds: float = 6.0):
        """
        Calibrates both the energy VAD threshold and the spectral subtractor.

        Key fix: the energy threshold is measured on CLEANED audio (post
        notch + spectral subtraction), not raw.  This means USB mic hum
        no longer inflates the threshold and blocks real speech from passing
        Gate 1.

        Increase `seconds` if spectral cleaner still reports 'needs more
        frames' — each AudioBuffer frame is ~2s, so 6s ≈ 3 frames minimum.
        """
        print(f"Calibrating noise floor for {seconds}s — stay quiet...")
        deadline     = time.time() + seconds
        raw_samples  = []   # raw RMS  (for display only)
        clean_samples = []  # cleaned RMS  (used for threshold)

        while time.time() < deadline:
            if self.audio_buffer.ready():
                raw = self.audio_buffer.get()

                # ── Step 1: feed raw to spectral subtractor ────────────────
                tensor = torch.tensor(raw, dtype=torch.float32).unsqueeze(0)
                self.audio_cleaner.calibrate(tensor)

                # ── Step 2: measure RMS on CLEANED audio ───────────────────
                cleaned = self.audio_cleaner.process_numpy(raw)
                raw_rms   = float(np.sqrt(np.mean(raw     ** 2)))
                clean_rms = float(np.sqrt(np.mean(cleaned ** 2)))
                raw_samples.append(raw_rms)
                clean_samples.append(clean_rms)

            time.sleep(0.05)

        if not clean_samples:
            print("   Calibration failed — keeping default threshold.\n")
            return

        raw_noise   = float(np.mean(raw_samples))
        clean_noise = float(np.mean(clean_samples))

        # Threshold is on cleaned signal — 4× its noise floor
        self.energy_threshold = clean_noise * 4.0

        status = "ready" if self.audio_cleaner.calibrated else "needs more frames (increase calibration seconds)"
        print(f"   Raw mic RMS      : {raw_noise:.5f}")
        print(f"   Cleaned RMS      : {clean_noise:.5f}  ({100*(1-clean_noise/max(raw_noise,1e-9)):.0f}% noise removed)")
        print(f"   Energy threshold : {self.energy_threshold:.5f}  (on cleaned signal)")
        print(f"   Spectral cleaner : {status}\n")

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

            # ── Gate 1: energy VAD (on cleaned audio) ─────────────────────
            raw     = self.audio_buffer.get()
            cleaned = self._clean(raw)                        # ← clean FIRST
            rms     = float(np.sqrt(np.mean(cleaned ** 2)))  # RMS on clean

            if rms <= self.energy_threshold:
                self.vote_buffer.clear()
                self._silence_streak += 1
                if not self._armed and self._silence_streak >= self.silence_frames_needed:
                    self._armed = True
                continue

            self._silence_streak = 0

            # ── Gate 2: disarmed guard ────────────────────────────────────
            if not self._armed:
                continue

            # ── Gate 3: model inference on cleaned audio ──────────────────
            mel_spec = self.audio_processor.preprocess(cleaned)
            mel_spec = mel_spec.unsqueeze(0).to(self.device)

            label, confidence, margin = self._runWakeWordDetection(mel_spec)

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
            self._last_detection_time = now
            self._armed               = False
            self._silence_streak      = 0
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