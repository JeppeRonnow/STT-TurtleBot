from scipy.signal import butter, sosfiltfilt
import numpy as np

class DSP:
    filter = ""

    # Create filter on objct init
    def __init__(self, sample_rate, highpass_hz, lowpass_hz) -> None:
        self.filter = self.design_bandpass_filter(sample_rate, highpass_hz, lowpass_hz)
        pass


    # Create band-pass filter
    def design_bandpass_filter(self, sr: int, hp: float, lp: float, order: int = 4):
        self.filter = butter(order, [hp / (0.5 * sr), lp / (0.5 * sr)], btype='bandpass', output='sos')
        return self.filter


    # Apply band-pass filter to audio signal
    def apply_filters(self, audio: np.ndarray):
        if audio.size == 0:
            return audio
        audio = sosfiltfilt(self.filter, audio).astype(np.float32)   # apply band-pass filter
        return audio


    # Normalize audio signal after filtering
    def normalize(self, audio: np.ndarray):
        peak = np.max(np.abs(audio)) if audio.size else 1.0
        if peak > 0:
            audio = (audio / peak * 0.95).astype(np.float32)
        return audio

