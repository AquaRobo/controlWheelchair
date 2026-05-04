import time
import threading
import numpy as np
import sounddevice as sd
import whisper
import torch

from cv.helper.AudioBuffer import AudioBuffer


class SpeechRecognizer:
    def __init__(self, config: dict, model_path: str):
        self.config = config
        self.model_path = model_path

        self.device = config.get(
            "device",
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        self.sample_rate = config.get("sample_rate", 16000)
        self.duration = config.get("listen_duration", 3)

        self.audio_buffer = AudioBuffer(self.sample_rate, self.duration)

        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32",
            callback=self._audio_callback,
        )
        self.stream.start()

        self.wake_word_threshold = config.get("wake_word_threshold", 0.5)
        self.whisper_model = None
        self.whisper_model_size = config.get("whisper_model", "tiny.en")

        # Background Whisper state (thread-safe)
        self._lock = threading.Lock()
        self._whisper_running = False
        self._last_result = (None, None, None)

        self._loadWhisperModel()

    # ------------------------------------------------------------------
    # Audio capture
    # ------------------------------------------------------------------

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            return
        self.audio_buffer.add(indata[:, 0])

    # ------------------------------------------------------------------
    # Model loading
    # ------------------------------------------------------------------

    def _loadWhisperModel(self):
        self.whisper_model = whisper.load_model(
            self.whisper_model_size,
            device=self.device,
        )

    # ------------------------------------------------------------------
    # Whisper inference (background thread — never blocks ROS timer)
    # ------------------------------------------------------------------

    def _whisper_worker(self, audio: np.ndarray):
        try:
            result = self.whisper_model.transcribe(
                audio,
                language="en",
                fp16=(self.device == "cuda"),
            )
            transcription = result["text"].strip()

            room   = self._getCommandedRoom(transcription)
            obj    = self._getCommandedObject(transcription)
            action = self._getCommandedAction(transcription)

            with self._lock:
                self._last_result = (room, obj, action)
        finally:
            with self._lock:
                self._whisper_running = False

    # ------------------------------------------------------------------
    # Command extraction
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Public API — called by SpeechRecognizerNode timer (10 Hz)
    # ------------------------------------------------------------------

    def recognizeSpeech(self):
        """Non-blocking. Returns (room, obj, action) when ready, else Nones."""

        # Kick off background Whisper when buffer is full and nothing is running
        if self.audio_buffer.ready():
            with self._lock:
                if not self._whisper_running:
                    audio = self.audio_buffer.get()
                    self.audio_buffer.clear()
                    self._whisper_running = True
                    threading.Thread(
                        target=self._whisper_worker,
                        args=(audio,),
                        daemon=True
                    ).start()

        # Return any result that arrived since the last call
        with self._lock:
            result = self._last_result
            if any(result):
                self._last_result = (None, None, None)
                return result

        return None, None, None