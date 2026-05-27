"""
AudioCleaner
============
Removes electrical hum and steady-state background noise from a USB mic signal.

Pipeline (in order):
  1. High-pass biquad        — kills DC offset and sub-80 Hz rumble
  2. Notch filter stack      — surgical removal of 50/60 Hz + harmonics
  3. Spectral subtraction    — subtracts the estimated noise floor (estimated
                               during the first `noise_frames` of silence)

All operations stay in torch — no scipy, no numpy round-trips, runs on CPU or CUDA.
"""

from __future__ import annotations

import torch
import torchaudio.functional as F
from typing import Literal


# ──────────────────────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────────────────────

def _highpass(waveform: torch.Tensor, cutoff: float, sr: int, order: int = 4) -> torch.Tensor:
    """Cascaded biquad high-pass (order/2 stages)."""
    for _ in range(max(1, order // 2)):
        waveform = F.highpass_biquad(waveform, sample_rate=sr, cutoff_freq=cutoff)
    return waveform


def _notch_stack(
    waveform: torch.Tensor,
    fundamental: float,
    sr: int,
    n_harmonics: int = 4,
    q: float = 35.0,
) -> torch.Tensor:
    """
    Apply a notch (band-reject) biquad at `fundamental` and its first
    `n_harmonics` harmonics.

    Q=35 is narrow enough to be inaudible but wide enough to catch
    slight frequency drift in cheap USB audio chips.

    Harmonics targeted for 50 Hz: 50, 100, 150, 200, 250 Hz
    Harmonics targeted for 60 Hz: 60, 120, 180, 240, 300 Hz
    """
    for k in range(1, n_harmonics + 2):          # k=1 → fundamental, k=2..5 → harmonics
        freq = fundamental * k
        if freq >= sr / 2:                        # don't exceed Nyquist
            break
        waveform = F.bandreject_biquad(waveform, sample_rate=sr, central_freq=freq, Q=q)
    return waveform


# ──────────────────────────────────────────────────────────────────────────────
# SPECTRAL SUBTRACTION
# ──────────────────────────────────────────────────────────────────────────────

class SpectralSubtractor:
    """
    Estimates a steady-state noise profile from the first `noise_frames` frames
    of silence, then subtracts it from every subsequent frame using
    over-subtraction to suppress residual musical noise.

    Call `update(frame)` during the calibration window (silence).
    Call `apply(frame)`  during live audio processing.
    `reset()` re-starts calibration (e.g. after a long pause).
    """

    def __init__(
        self,
        n_fft: int = 512,
        hop_length: int = 256,
        noise_frames: int = 20,
        alpha: float = 2.0,     # over-subtraction factor  (1.0–3.0; higher → more aggressive)
        beta: float = 0.01,     # spectral floor            (prevents total silence artifacts)
    ):
        self.n_fft        = n_fft
        self.hop_length   = hop_length
        self.noise_frames = noise_frames
        self.alpha        = alpha
        self.beta         = beta

        self._noise_accum: torch.Tensor | None = None
        self._frame_count = 0
        self._noise_profile: torch.Tensor | None = None

    # ── calibration ───────────────────────────────────────────────────────────

    def update(self, waveform: torch.Tensor) -> None:
        """Feed a silent frame to build the noise profile."""
        mag = self._magnitude(waveform)           # (freq_bins,)
        if self._noise_accum is None:
            self._noise_accum = mag
        else:
            self._noise_accum = self._noise_accum + mag
        self._frame_count += 1

        if self._frame_count >= self.noise_frames:
            self._noise_profile = self._noise_accum / self._frame_count

    @property
    def ready(self) -> bool:
        return self._noise_profile is not None

    def reset(self) -> None:
        self._noise_accum   = None
        self._frame_count   = 0
        self._noise_profile = None

    # ── processing ────────────────────────────────────────────────────────────

    def apply(self, waveform: torch.Tensor) -> torch.Tensor:
        """
        Subtract the noise profile from `waveform`.
        Returns waveform unchanged if calibration is not complete.
        """
        if not self.ready:
            return waveform

        # STFT
        original_length = waveform.shape[-1]
        spec   = torch.stft(
            waveform.squeeze(0),
            n_fft=self.n_fft,
            hop_length=self.hop_length,
            return_complex=True,
        )                                         # (freq_bins, time_frames)

        mag   = spec.abs()                        # amplitude spectrum
        phase = spec.angle()                      # phase (preserved)

        # Over-subtraction: subtract α × noise_profile, floor at β × mag
        noise = self._noise_profile.unsqueeze(1).to(waveform.device)
        mag_clean = torch.clamp(mag - self.alpha * noise, min=self.beta * mag)

        # Reconstruct complex spectrum and invert
        spec_clean = torch.polar(mag_clean, phase)
        wav_clean  = torch.istft(
            spec_clean,
            n_fft=self.n_fft,
            hop_length=self.hop_length,
            length=original_length,
        )
        return wav_clean.unsqueeze(0)

    # ── internal ──────────────────────────────────────────────────────────────

    def _magnitude(self, waveform: torch.Tensor) -> torch.Tensor:
        spec = torch.stft(
            waveform.squeeze(0),
            n_fft=self.n_fft,
            hop_length=self.hop_length,
            return_complex=True,
        )
        return spec.abs().mean(dim=1)             # average across time → (freq_bins,)


# ──────────────────────────────────────────────────────────────────────────────
# MAIN CLASS
# ──────────────────────────────────────────────────────────────────────────────

class AudioCleaner:
    """
    Drop-in audio cleaning pipeline for USB mic input.

    Usage
    -----
    cleaner = AudioCleaner(sample_rate=16000, hum_freq="50hz")

    # During silence at startup (feeds SpectralSubtractor):
    cleaner.calibrate(silent_audio_tensor)

    # Every audio frame thereafter:
    clean = cleaner.process(raw_audio_tensor)

    Parameters
    ----------
    sample_rate  : mic sample rate in Hz
    hum_freq     : "50hz" (Europe/Asia) or "60hz" (Americas/Japan)
    hp_cutoff    : high-pass cutoff in Hz          (default 80)
    hp_order     : high-pass order (even)          (default 4)
    notch_q      : notch filter Q factor           (default 35)
    n_harmonics  : how many harmonics to notch     (default 4 → fund + 4 harmonics)
    ss_alpha     : spectral subtraction strength   (default 2.0)
    ss_beta      : spectral floor ratio            (default 0.01)
    noise_frames : silent frames needed to calibrate (default 20)
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        hum_freq: Literal["50hz", "60hz"] = "50hz",
        hp_cutoff: float = 80.0,
        hp_order: int = 4,
        notch_q: float = 35.0,
        n_harmonics: int = 4,
        ss_alpha: float = 2.0,
        ss_beta: float = 0.01,
        noise_frames: int = 20,
    ):
        self.sample_rate  = sample_rate
        self.fundamental  = 50.0 if hum_freq == "50hz" else 60.0
        self.hp_cutoff    = hp_cutoff
        self.hp_order     = hp_order
        self.notch_q      = notch_q
        self.n_harmonics  = n_harmonics

        self.subtractor = SpectralSubtractor(
            noise_frames=noise_frames,
            alpha=ss_alpha,
            beta=ss_beta,
        )

    # ── public API ────────────────────────────────────────────────────────────

    def calibrate(self, silent_audio: torch.Tensor) -> None:
        """
        Feed a frame of silence to build the spectral noise profile.
        Call repeatedly (or once with a long silent clip) until
        `self.subtractor.ready` is True.
        After `noise_frames` calls it locks in automatically.
        """
        # Run through deterministic filters first so the profile
        # matches what the subtractor will see in process()
        filtered = self._deterministic_filters(silent_audio)
        self.subtractor.update(filtered)

    def process(self, raw_audio: torch.Tensor) -> torch.Tensor:
        """
        Clean a raw audio tensor (1, T) or (T,).
        Returns a tensor of the same shape.
        """
        squeezed = raw_audio.dim() == 1
        if squeezed:
            raw_audio = raw_audio.unsqueeze(0)

        out = self._deterministic_filters(raw_audio)  # 1. HP + notch
        out = self.subtractor.apply(out)               # 2. spectral subtraction

        return out.squeeze(0) if squeezed else out

    def process_numpy(self, raw_audio) -> "np.ndarray":  # noqa: F821
        """Convenience wrapper: accepts np.ndarray, returns np.ndarray."""
        import numpy as np
        t = torch.tensor(raw_audio, dtype=torch.float32)
        return self.process(t).numpy()

    @property
    def calibrated(self) -> bool:
        return self.subtractor.ready

    def reset_calibration(self) -> None:
        self.subtractor.reset()

    # ── internal ──────────────────────────────────────────────────────────────

    def _deterministic_filters(self, waveform: torch.Tensor) -> torch.Tensor:
        """High-pass + notch stack — deterministic, no state."""
        out = _highpass(waveform, self.hp_cutoff, self.sample_rate, self.hp_order)
        out = _notch_stack(out, self.fundamental, self.sample_rate, self.n_harmonics, self.notch_q)
        return out