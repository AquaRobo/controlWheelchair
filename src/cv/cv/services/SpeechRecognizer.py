import time
import numpy as np
import sounddevice as sd
import whisper
import librosa
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


        self.audio_buffer = AudioBuffer(
            self.sample_rate,
            self.duration
        )

        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32",
            callback=self._audio_callback,
        )
        self.stream.start()
        self.wake_word_threshold = config.get("wake_word_threshold", 0.5)
        self.wake_word_model = None

        self.whisper_model = None
        self.whisper_model_size = config.get("whisper_model", "tiny.en")

        

        # self._loadWakeWordModel()
        self._loadWhisperModel()



    def _audio_callback(self, indata, frames, time, status):
        if status:
            return
        self.audio_buffer.add(indata[:, 0])



    # def _listen(self) -> np.ndarray:
    #     audio = sd.rec(
    #         int(self.sample_rate * self.duration),
    #         samplerate=self.sample_rate,
    #         channels=1,
    #         dtype="float32",
    #     )
    #     sd.wait()
    #     return audio.flatten()

    def _loadWakeWordModel(self):
        # model = torch.load(self.model_path, map_location=self.device, weights_only=False)

        # if not isinstance(model, torch.jit.ScriptModule):
        #     model = model.to(self.device)

        # model.eval()
        # self.wake_word_model = 
        pass

    def _runWakeWordDetection(self) -> float:
        # if self.wake_word_model is None:
        #     return 0.0

        # audio = self._listen()

        # if np.max(np.abs(audio)) > 0:
        #     audio = audio / np.max(np.abs(audio))

        # mel = librosa.feature.melspectrogram(
        #     y=audio,
        #     sr=self.sample_rate,
        #     n_fft=1024,
        #     hop_length=512,
        #     n_mels=64,
        # )
        # mel_db = librosa.power_to_db(mel, ref=np.max)
        # mel_db = (mel_db - mel_db.mean()) / (mel_db.std() + 1e-6)

        # x = (
        #     torch.tensor(mel_db, dtype=torch.float32)
        #     .unsqueeze(0)
        #     .unsqueeze(0)
        #     .to(self.device)
        # )

        # with torch.inference_mode():
        #     logit = self.wake_word_model(x).item()
        #     prob = torch.sigmoid(torch.tensor(logit, device=self.device)).item()

        # return prob
        pass


    def _getWakeWord(self) -> bool:
        return  True #self._runWakeWordDetection() >= self.wake_word_threshold

    def _loadWhisperModel(self):
        self.whisper_model = whisper.load_model(
            self.whisper_model_size,
            device=self.device,
        )

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
        
    
    def recognizeSpeech(self):

        if not self.audio_buffer.ready():
            return None, None, None
        audio = self.audio_buffer.get()
        self.audio_buffer.clear()

        transcription = self._runWhisper(audio)

        room = self._getCommandedRoom(transcription)
        obj = self._getCommandedObject(transcription)
        action = self._getCommandedAction(transcription)

        return room, obj, action

    # def end_stream(self):
    #     self.stream.stop()
    #     self.stream.close()