from collections import deque
import numpy as np


class AudioBuffer:
    def __init__(self, sample_rate: int, duration: float):
        self.max_samples = int(sample_rate * duration)
        self.buffer      = deque(maxlen=self.max_samples)

    def add(self, samples):
        self.buffer.extend(samples)

    def ready(self) -> bool:
        return len(self.buffer) == self.max_samples

    def get(self) -> np.ndarray:
        return np.array(self.buffer, dtype=np.float32)

    def rms(self) -> float:
        """RMS energy of the current buffer — used for VAD gating."""
        if len(self.buffer) == 0:
            return 0.0
        arr = np.array(self.buffer, dtype=np.float32)
        return float(np.sqrt(np.mean(arr ** 2)))

    def clear(self):
        self.buffer.clear()