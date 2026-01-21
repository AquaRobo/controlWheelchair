from collections import deque
import numpy as np

class AudioBuffer:
    def __init__(self, sample_rate: int, duration: float):
        self.max_samples = int(sample_rate * duration)
        self.buffer = deque(maxlen=self.max_samples)

    def add(self, samples):
        self.buffer.extend(samples)

    def ready(self) -> bool:
        return len(self.buffer) == self.max_samples

    def get(self) -> np.ndarray:
        return np.array(self.buffer, dtype=np.float32)

    def clear(self):
        self.buffer.clear()
