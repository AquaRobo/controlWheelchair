import torch
import torch.nn as nn


class WakeWordModel(nn.Module):
    def __init__(self, num_classes: int = 3):
        super().__init__()

        self.features = nn.Sequential(
            # Block 1
            nn.Conv2d(1, 16, kernel_size=3, padding=1),
            nn.BatchNorm2d(16),
            nn.ReLU(inplace=True),
            nn.MaxPool2d((2, 1)),
            nn.Dropout(p=0.2),

            # Block 2
            nn.Conv2d(16, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d((2, 1)),
            nn.Dropout(p=0.3),

            # Block 3 (no pooling — preserve time)
            nn.Conv2d(32, 64, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.4),

            # Time-frequency aggregation
            nn.AdaptiveAvgPool2d((1, 1))
        )

        self.classifier = nn.Sequential(
            nn.Linear(64, 32),
            nn.ReLU(inplace=True),
            nn.Dropout(p=0.4),
            nn.Linear(32, num_classes)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: [B, 1, 64, T]
        x = self.features(x)       # [B, 64, 1, 1]
        x = torch.flatten(x, 1)    # [B, 64]
        return self.classifier(x)

    @torch.inference_mode()
    def predict(self, mel_spec: torch.Tensor) -> tuple[int, float, float]:
        """
        Run a single inference pass.

        Args:
            mel_spec: tensor of shape [1, 1, n_mels, T] already on the correct device

        Returns:
            (prediction, confidence, margin)
            - prediction : argmax class index
            - confidence : softmax probability of the winning class
            - margin     : gap between top-1 and top-2 probabilities
        """
        probs      = torch.softmax(self(mel_spec), dim=1)
        prediction = torch.argmax(probs, dim=1).item()
        confidence = probs[0][prediction].item()
        top2       = torch.topk(probs, 2)
        margin     = (top2.values[0][0] - top2.values[0][1]).item()

        return prediction, confidence, margin