import time
import numpy as np
import sounddevice as sd
import torch, torchaudio
from faster_whisper import WhisperModel
from collections import deque

from cv.helper.AudioBuffer import AudioBuffer
from cv.helper.WakeWordModel import WakeWordModel
from cv.helper.AudioProcessor import AudioProcessor

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

        # ── Wake word buffer (sliding deque — never manually cleared) ─────
        self.audio_buffer = AudioBuffer(self.sample_rate, self.duration)

        # ── Command buffer (separate fixed-duration buffer for commands) ───
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

        # ── Detection thresholds ──────────────────────────────────────────
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

        # ── Detection state ───────────────────────────────────────────────
        self._last_detection_time = 0.0
        self._armed               = True
        self._silence_streak      = 0
        self._recording_command   = False

        # ── Whisper ───────────────────────────────────────────────────────
        self.whisper_model      = None
        self.whisper_model_size = config.get("whisper_model", "tiny.en")



        # ── Command resampler (for Whisper input) ─────────────────────────────
        self.command_resampler = torchaudio.transforms.Resample(
            orig_freq=self.sample_rate,
            new_freq=self.target_sample_rate
        )  # keep on CPU — Whisper runs on CPU
        self._loadWakeWordModel()
        self._loadWhisperModel()

        self.stream = sd.InputStream(
            device=None,
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
            device="cpu",        # Pi has no CUDA
            compute_type="int8"  # quantized — much faster and lower RAM on Pi
        )

    # =========================================================
    # NOISE CALIBRATION
    # =========================================================

    def calibrate_noise_floor(self, seconds: float = 3.0):
        print(f"Calibrating noise floor for {seconds}s — stay quiet...")
        deadline = time.time() + seconds
        samples  = []

        while time.time() < deadline:
            if self.audio_buffer.ready():
                samples.append(self.audio_buffer.rms())
            time.sleep(0.05)

        if not samples:
            print("   Calibration failed — keeping default threshold.\n")
            return

        noise_rms             = float(np.mean(samples))
        self.energy_threshold = noise_rms * 4.0

    # =========================================================
    # WAKE WORD DETECTION
    # =========================================================

    def _runWakeWordDetection(self, mel_spec: torch.Tensor) -> tuple[str, float, float]:
        prediction, confidence, margin = self.wake_word_model.predict(mel_spec)
        label = self.class_map[prediction]
        return label, confidence, margin

    def _detect_wake_word(self) -> str:
        while True:
            if not self.audio_buffer.ready():
                continue

            rms = self.audio_buffer.rms()

            # ── Gate 1: energy VAD ────────────────────────────────────────
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

            # ── Gate 3: model inference ───────────────────────────────────
            audio    = self.audio_buffer.get()
            mel_spec = self.audio_processor.preprocess(audio)
            mel_spec = mel_spec.unsqueeze(0).to(self.device)

            label, confidence, margin = self._runWakeWordDetection(mel_spec)

            # ── Gate 4: per-frame hard thresholds ────────────────────────
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

        audio = self.command_buffer.get()               # np.ndarray (float32)
        self.command_buffer.clear()
        self._recording_command = False

        # Resample for Whisper if needed
        if self.sample_rate != self.target_sample_rate:
            tensor = torch.tensor(audio, dtype=torch.float32).unsqueeze(0)  # (1, T)
            tensor = self.command_resampler(tensor)                          # (1, T')
            audio  = tensor.squeeze(0).numpy()                               # (T',)

        print(f"Command captured — {len(audio) / self.target_sample_rate:.1f}s")
        return audio
    
    # =========================================================
    # WHISPER + COMMAND PARSING
    # =========================================================

    def _runWhisper(self, audio: np.ndarray) -> str:
        if self.whisper_model is None:
            return ""

        segments, _ = self.whisper_model.transcribe(
            audio,
            language="en"
        )
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
                self._recording_command = True          # ← re-arm before recording
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
                self._recording_command = True          # ← re-arm before recording
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