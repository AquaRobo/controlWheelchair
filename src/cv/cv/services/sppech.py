import os
import sys
import time
import torch
import librosa
import numpy as np
import sounddevice as sd
import whisper
import termios, tty, queue
import soundfile as sf

# ===== CONFIG =====
WAKE_MODEL_PATH = "model_output/best_model_scripted.pt"
WHISPER_MODEL_PATH = "tiny"
SAMPLE_RATE = 16000
WAKE_DURATION = 1.0
STEP = 0.2
THRESHOLD = 0.2
TARGET_WORDS = ['water', 'bottle', 'cup', 'kitchen', 'bed room', 'bed',
                'living room', 'bath room', 'office', 'book']
# ==================

# --- Non-blocking key press ---
def get_keypress_nonblock():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if select := __import__("select"):
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if dr:
                return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None

# --- Audio preprocessing ---
def pad_or_trim(y, sr=SAMPLE_RATE, duration=WAKE_DURATION):
    target = int(sr * duration)
    if len(y) > target:
        return y[-target:]
    return np.pad(y, (target - len(y), 0))

def extract_mel(y, sr=SAMPLE_RATE):
    if np.max(np.abs(y)) > 0:
        y = y / np.max(np.abs(y))
    y, _ = librosa.effects.trim(y, top_db=30)
    y = pad_or_trim(y, sr)
    mel = librosa.feature.melspectrogram(
        y=y, sr=sr, n_fft=1024, hop_length=512, n_mels=64, fmin=20, fmax=sr // 2)
    mel_db = librosa.power_to_db(mel, ref=np.max)
    mel_db = (mel_db - mel_db.mean()) / (mel_db.std() + 1e-6)
    return torch.tensor(mel_db, dtype=torch.float32).unsqueeze(0).unsqueeze(0)

# --- Load models ---
def load_wake_model(path):
    model = torch.jit.load(path, map_location="cpu")
    model.eval()
    return model

def load_whisper_model(path):
    print(f"Loading Whisper model from: {path}")
    model = whisper.load_model(path)
    return model

# --- Predict wake word ---
def predict_wake(y, model):
    x = extract_mel(y)
    with torch.inference_mode():
        logit = model(x).item()
        prob = torch.sigmoid(torch.tensor(logit)).item()
    return prob

# --- Whisper transcription ---
def record_for_whisper(filename="command.wav", duration=3.0):
    print(f"Recording command for {duration}s...")
    data = sd.rec(int(duration * SAMPLE_RATE), samplerate=SAMPLE_RATE,
                  channels=1, dtype="float32")
    sd.wait()
    sf.write(filename, data, SAMPLE_RATE)
    return filename

def transcribe_and_check(model, filename):
    print("Transcribing using Whisper (offline)...")
    result = model.transcribe(filename)
    text = result["text"].strip().lower()
    print(f"Transcribed text: {text}")

    for word in TARGET_WORDS:
        if word in text:
            print(f"Detected command word: '{word}'")
            return word
    print("No target words detected.")
    return None

# --- Main ---
def main():
    print("Wake-word + Whisper combo running (offline). Press 'q' to quit.")
    wake_model = load_wake_model(WAKE_MODEL_PATH)
    whisper_model = load_whisper_model(WHISPER_MODEL_PATH)

    while True:
        # quit check
        key = get_keypress_nonblock()
        if key == "q":
            print("Exiting program.")
            break

        # record small window for wake word
        data = sd.rec(int(WAKE_DURATION * SAMPLE_RATE), samplerate=SAMPLE_RATE,
                      channels=1, dtype="float32")
        sd.wait()
        y = data.flatten()

        prob = predict_wake(y, wake_model)
        if prob >= THRESHOLD:
            print(f"Wake word detected (prob={prob:.2f})")
            cmd_file = record_for_whisper()
            transcribe_and_check(whisper_model, cmd_file)
            print("Returning to wake-word listening...")
        else:
            print(f"Background / no wake word (prob={prob:.2f})")

        time.sleep(STEP)

if __name__ == "__main__":
    main()
