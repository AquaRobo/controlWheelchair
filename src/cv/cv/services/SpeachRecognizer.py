import torch
import librosa
import numpy as np
import sounddevice as sd
import whisper
import soundfile as sf

class SpeachRecognizer:
    def __init__(self, config: dict, model_path: str):
        self.wake_model_path = model_path
        self.whisper_model_size = config['WHISPER_MODEL_SIZE']
        self.sample_rate = config['SAMPLE_RATE']
        self.wake_duration = config['WAKE_DURATION']
        self.step = config['STEP']
        self.threshold = config['THRESHOLD']
        self.device = torch.device(config['DEVICE'])  # ✅ enforce torch.device
        self.target_words = config['TARGET_WORDS']

        self.wake_model = self.__loadWakeModel()
        self.whisper_model = whisper.load_model(
            self.whisper_model_size,
            device=str(self.device)  # whisper expects string
        )

        self.recognized_room = None
        self.recognized_object = None
        self.__transcribe = None
        self.__prob = 0.0

    def __loadWakeModel(self):
        model = torch.jit.load(
            self.wake_model_path,
            map_location=self.device
        ).to(self.device)
        model.eval()
        return model

    def recognizeSpeech(self) -> bool:
        if self.__waitForWakeWord():
            cmd_file = self.__recordForWhisper()
            return self.__transcribeAndCheck(self.whisper_model, cmd_file)
        return False

    def __waitForWakeWord(self):
        data = sd.rec(
            int(self.wake_duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32"
        )
        sd.wait()
        y = data.flatten()
        self.__prob = self.__predictWake(y)
        return self.__prob >= self.threshold

    def __predictWake(self, y):
        x = self.__extractMel(y, self.sample_rate)  # already on device
        with torch.inference_mode():
            logit = self.wake_model(x)               # tensor on device
            prob = torch.sigmoid(logit).item()       # no CPU tensor creation
        return prob

    def __extractMel(self, y, sr):
        if np.max(np.abs(y)) > 0:
            y = y / np.max(np.abs(y))

        y, _ = librosa.effects.trim(y, top_db=30)
        y = self.__padOrTrim(y, sr, self.wake_duration)

        mel = librosa.feature.melspectrogram(
            y=y,
            sr=sr,
            n_fft=1024,
            hop_length=512,
            n_mels=64,
            fmin=20,
            fmax=sr // 2
        )

        mel_db = librosa.power_to_db(mel, ref=np.max)
        mel_db = (mel_db - mel_db.mean()) / (mel_db.std() + 1e-6)

        # ✅ MOVE TO DEVICE HERE
        return (
            torch.from_numpy(mel_db)
            .float()
            .unsqueeze(0)
            .unsqueeze(0)
            .to(self.device)
        )

    def __padOrTrim(self, y, sr, duration):
        target = int(sr * duration)
        if len(y) > target:
            return y[-target:]
        return np.pad(y, (target - len(y), 0))

    def __recordForWhisper(self, filename="command.wav", duration=3.0):
        data = sd.rec(
            int(duration * self.sample_rate),
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32"
        )
        sd.wait()
        sf.write(filename, data, self.sample_rate)
        return filename

    def __transcribeAndCheck(self, model, filename):
        result = model.transcribe(filename)
        self.__transcribe = result["text"].strip().lower()

        for word in self.target_words:
            if word in self.__transcribe:
                if word in ['kitchen', 'bed room', 'living room', 'bath room', 'office']:
                    self.recognized_room = word.replace(" ", "_")
                else:
                    self.recognized_object = word
                return True
        return False

    def getTranscription(self):
        return self.__transcribe

    def getWakeWordProbability(self):
        return self.__prob
