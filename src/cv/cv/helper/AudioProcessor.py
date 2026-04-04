import torch
import torchaudio


class AudioProcessor:
    def __init__(self,
                 target_sample_rate: int = 16000,
                 duration: float         = 2.0,
                 device: str             = "cpu"):

        self.target_sample_rate = target_sample_rate
        self.duration           = duration
        self.device             = device

        self.transformation = torch.nn.Sequential(
            torchaudio.transforms.MelSpectrogram(
                sample_rate=target_sample_rate,
                n_fft=1024,
                hop_length=512,
                n_mels=64
            ),
            torchaudio.transforms.AmplitudeToDB()
        ).to(device)   # keep the transform on the same device as the model

    def preprocess(self, audio) -> torch.Tensor:
        signal = torch.tensor(audio, dtype=torch.float32).unsqueeze(0).to(self.device)

        # Normalize
        peak = signal.abs().max()
        if peak > 0:
            signal = signal / peak

        # Pad or trim to exact expected length
        num_samples = int(self.target_sample_rate * self.duration)
        if signal.shape[1] < num_samples:
            signal = torch.nn.functional.pad(signal, (0, num_samples - signal.shape[1]))
        elif signal.shape[1] > num_samples:
            signal = signal[:, :num_samples]

        return self.transformation(signal)