import torch
import torchaudio
import sounddevice as sd
import numpy as np
from cv.helper.WakeWordModel import WakeWordModel
from cv.helper.AudioProcessor import AudioProcessor

SAMPLE_RATE = 44100
DURATION    = 2

model = WakeWordModel(num_classes=3)
model.load_state_dict(torch.load("/home/abdelrahman/Desktop/grad_project/controlWheelchair/src/cv/models/best_wakeword_model3.pt", map_location="cpu"))
model.eval()

processor = AudioProcessor(
    input_sample_rate  = SAMPLE_RATE,
    target_sample_rate = 16000,
    duration           = DURATION,
    device             = "cpu"
)

class_map = {0: "Milo", 1: "Jarvis", 2: "Other"}

print("Say a wake word in 3 seconds...")
import time; time.sleep(1)
print("Recording NOW")

audio = sd.rec(int(SAMPLE_RATE * DURATION), samplerate=SAMPLE_RATE,
               channels=1, dtype="float32")
sd.wait()
audio = audio[:, 0]

mel = processor.preprocess(audio).unsqueeze(0)
with torch.no_grad():
    probs = torch.softmax(model(mel), dim=1)[0]

print(f"Milo={probs[0]:.3f}  Jarvis={probs[1]:.3f}  Other={probs[2]:.3f}")
print(f"Predicted: {class_map[probs.argmax().item()]}")