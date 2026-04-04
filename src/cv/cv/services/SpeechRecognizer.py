
import time
import numpy as np
import sounddevice as sd
import whisper
import torch
from collections import deque

from cv.helper.AudioBuffer import AudioBuffer
from cv.helper.WakeWordModel import WakeWordModel
from cv.helper.AudioProcessor import AudioProcessor


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

        self.sample_rate = config.get("sample_rate", 16000)
        self.duration    = config.get("listen_duration", 2)

        # ── Wake word buffer (sliding deque — never manually cleared) ─────
        self.audio_buffer = AudioBuffer(self.sample_rate, self.duration)

        # ── Command buffer (separate fixed-duration buffer for commands) ───
        self.command_duration = config.get("command_duration", 3)
        self.command_buffer   = AudioBuffer(self.sample_rate, self.command_duration)



        # ── Audio processor ───────────────────────────────────────────────
        self.audio_processor = AudioProcessor(
            target_sample_rate=self.sample_rate,
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
        self._recording_command   = False   # gates which buffer gets audio

        # ── Whisper ───────────────────────────────────────────────────────
        self.whisper_model      = None
        self.whisper_model_size = config.get("whisper_model", "tiny.en")

        self.room   = None
        self.obj    = None
        self.action = None

        self._loadWakeWordModel()
        self._loadWhisperModel()

        self.stream = sd.InputStream(
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
        # Route audio to the correct buffer depending on current state
        if self._recording_command:
            self.command_buffer.add(samples)
        else:
            self.audio_buffer.add(samples)

    def _loadWakeWordModel(self):
        state_dict = torch.load(self.model_path, map_location=self.device)
        self.wake_word_model.load_state_dict(state_dict)
        self.wake_word_model.eval()

    def _loadWhisperModel(self):
        self.whisper_model = whisper.load_model(
            self.whisper_model_size,
            device=self.device,
        )

    # =========================================================
    # NOISE CALIBRATION
    # =========================================================

    def calibrate_noise_floor(self, seconds: float = 3.0):
        """
        Listen silently for seconds and auto-set energy_threshold
        to 4× the measured noise floor. Call once at startup.
        """
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
        # print(f"   Noise floor RMS : {noise_rms:.5f}")
        # print(f"   Auto threshold  : {self.energy_threshold:.5f}\n")

    # =========================================================
    # WAKE WORD DETECTION
    # =========================================================

    def _runWakeWordDetection(self, mel_spec: torch.Tensor) -> tuple[str, float, float]:
        prediction, confidence, margin = self.wake_word_model.predict(mel_spec)
        label = self.class_map[prediction]
        # print(
        #     f"Wake Word Detection — "
        #     f"Predicted: {label}  "
        #     f"Conf: {confidence:.4f}  "
        #     f"Margin: {margin:.4f}"
        # )
        return label, confidence, margin

    def _detect_wake_word(self) -> str:
        """
        Blocking loop — runs the full 6-gate pipeline and returns
        the confirmed wake word label once detected.
        """
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
                    # print("\n[armed — ready for next word]")

                # print(f"\r[silence] RMS={rms:.5f}  armed={self._armed}   ", end="")
                continue

            self._silence_streak = 0

            # ── Gate 2: disarmed guard ────────────────────────────────────
            if not self._armed:
                # print(f"\r[disarmed — waiting for silence]   ", end="")
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

            vote_count = sum(
                1 for v in self.vote_buffer.votes
                if v[0] == effective_label
            )
            # print(
            #     f"\r[{effective_label:6s}] "
            #     f"conf={confidence:.3f}  margin={margin:.3f}  "
            #     f"RMS={rms:.4f}  votes={vote_count}/{self.vote_buffer.window_size}   ",
            #     end=""
            # )

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
            # print(
            #     f"\n CONFIRMED: {detected_label} "
            #     f"| avg_conf={avg_conf:.3f} "
            #     f"| avg_margin={avg_margin:.3f}"
            # )
            self._last_detection_time = now
            self._armed               = False
            self._silence_streak      = 0
            self.vote_buffer.clear()

            # Flush wake word audio and switch routing to command buffer
            self.audio_buffer.clear()
            self.command_buffer.clear()
            self._recording_command = True
            time.sleep(0.3)   # brief pause so user knows detection happened

            return detected_label

    # =========================================================
    # COMMAND RECORDING
    # =========================================================

    def _record_command(self) -> np.ndarray:
        """
        Wait for the dedicated command buffer to fill up with
        exactly command_duration seconds of fresh audio, then
        return it — identical behaviour to the original code.
        """
        print("Listening for command...")

        # Block until the command buffer is full
        while not self.command_buffer.ready():
            time.sleep(0.05)

        audio                   = self.command_buffer.get()
        self.command_buffer.clear()
        self._recording_command = False   # switch routing back to wake word buffer

        print(f"Command captured — {len(audio) / self.sample_rate:.1f}s")
        return audio

    # =========================================================
    # WHISPER + COMMAND PARSING
    # =========================================================

    def _runWhisper(self, audio: np.ndarray) -> str:
        if self.whisper_model is None:
            return ""
        result = self.whisper_model.transcribe(
            audio,
            language="en",
            fp16=(self.device == "cuda"),
        )
        return result["text"].strip()

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

    # =========================================================
    # PUBLIC API
    # =========================================================

    def recognizeSpeech(self) -> tuple[str, str, str]:
        """
        Blocks until a wake word is confirmed, records the full
        command into a dedicated buffer, runs Whisper, then
        returns (room, obj, action).
        """
        wake_word = self._detect_wake_word()
        audio     = self._record_command()

        if wake_word == "Milo":
            print("Running Whisper for Milo...")
            transcription = self._runWhisper(audio)
            print(f"Transcription: {transcription}")
            self.room   = self._getCommandedRoom(transcription)
            self.action = self._getCommandedAction(transcription)

        elif wake_word == "Jarvis":
            print("Running Whisper for Jarvis...")
            transcription = self._runWhisper(audio)
            print(f"Transcription: {transcription}")
            self.obj    = self._getCommandedObject(transcription)
            self.action = self._getCommandedAction(transcription)

        return self.room, self.obj, self.action

    def end_stream(self):
        self.stream.stop()
        self.stream.close()
