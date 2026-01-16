import time
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
import librosa
import torch
from cv.helper.AudioBuffer import AudioBuffer


class SpeechRecognizer:
    def __init__(self, config: dict, model_path: str):
        # ---------------- Configuration ----------------
        self.sample_rate = config["SAMPLE_RATE"]
        self.wake_duration = config["WAKE_DURATION"]
        self.device = config["DEVICE"]
        self.target_words = config["TARGET_WORDS"]
        self.threshold = config["THRESHOLD"]
        self.whisper_model_size = config["WHISPER_MODEL_SIZE"]

        # ---------------- Audio ----------------
        self.command_duration = 3.0
        self.audio_buffer = AudioBuffer(
            self.sample_rate,
            self.command_duration
        )

        # ---------------- Models ----------------
        self.wake_model = self.__loadWakeModel(model_path)

        self.whisper_model = WhisperModel(
            self.whisper_model_size,
            device=self.device,
            compute_type="int8" if self.device == "cpu" else "float16"
        )

        # ---------------- Runtime State ----------------
        self.recognized_room = None
        self.recognized_object = None
        self.__transcription = ""
        self.__wake_prob = 0.0


    # ==================================================
    # Wake Word Model Loader
    # ==================================================

    def __loadWakeModel(self, model_path: str):
        """
        Load your TorchScript wake-word model.
        """
        import torch
        model = torch.jit.load(model_path, map_location=self.device)
        model.eval()
        return model


    # ==================================================
    # Audio Callback
    # ==================================================

    def __audioCallback(self, indata, frames, time_info, status):
        if status:
            print(status)
        self.audio_buffer.add(indata[:, 0])


    # ==================================================
    # Wake Word Detection
    # ==================================================

    def __waitForWakeWord(self) -> bool:
        """
        Blocking wake-word detection window.
        """

        data = sd.rec(
            int(self.wake_duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32"
        )
        sd.wait()

        y = data.flatten()
        if np.max(np.abs(y)) > 0:
            y = y / np.max(np.abs(y))

        mel = librosa.feature.melspectrogram(
            y=y,
            sr=self.sample_rate,
            n_fft=1024,
            hop_length=512,
            n_mels=64
        )
        mel_db = librosa.power_to_db(mel, ref=np.max)
        mel_db = (mel_db - mel_db.mean()) / (mel_db.std() + 1e-6)

        x = torch.tensor(mel_db, dtype=torch.float32)\
                .unsqueeze(0).unsqueeze(0)

        with torch.inference_mode():
            logit = self.wake_model(x).item()
            self.__wake_prob = torch.sigmoid(
                torch.tensor(logit)
            ).item()

        return self.__wake_prob >= self.threshold


    # ==================================================
    # Streaming Command Recording
    # ==================================================

    def __recordCommand(self) -> np.ndarray:
        self.audio_buffer.clear()

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            callback=self.__audioCallback,
            blocksize=int(self.sample_rate * 0.2),
        ):
            while not self.audio_buffer.ready():
                time.sleep(0.01)

        return self.audio_buffer.get()


    # ==================================================
    # Whisper Transcription + Keyword Extraction
    # ==================================================

    def __transcribeAndCheck(self, audio: np.ndarray) -> bool:
        segments, _ = self.whisper_model.transcribe(
            audio,
            language="en",
            beam_size=1,
            vad_filter=True
        )

        self.__transcription = " ".join(
            seg.text for seg in segments
        ).lower().strip()

        for word in self.target_words:
            if word in self.__transcription:
                if word in [
                    "kitchen", "bed room", "living room",
                    "bath room", "office"
                ]:
                    self.recognized_room = word.replace(" ", "_")
                else:
                    self.recognized_object = word
                return True

        return False


    # ==================================================
    # Public API (ROS Node Calls This)
    # ==================================================

    def recognizeSpeech(self) -> bool:
        self.recognized_room = None
        self.recognized_object = None
        self.__transcription = ""

        if not self.__waitForWakeWord():
            return False

        audio = self.__recordCommand()
        return self.__transcribeAndCheck(audio)


    # ==================================================
    # Getters
    # ==================================================

    def getRecognizedRoom(self):
        return self.recognized_room

    def getRecognizedObject(self):
        return self.recognized_object

    def getTranscription(self):
        return self.__transcription

    def getWakeWordProbability(self):
        return self.__wake_prob
