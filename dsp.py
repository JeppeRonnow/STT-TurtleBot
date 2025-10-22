from scipy.signal import butter, sosfiltfilt
import numpy as np

import config

def design_bandpass_filter(sr: int, hp: float, lp: float, order: int = 4):
    filter = butter(order, [hp / (0.5 * sr), lp / (0.5 * sr)], btype='bandpass', output='sos')
    return filter

def apply_filters(audio: np.ndarray):
    global filter
    if audio.size == 0:
        return audio
    audio = sosfiltfilt(filter, audio).astype(np.float32)   # apply band-pass filter
    return audio

def normalize(audio: np.ndarray):
    peak = np.max(np.abs(audio)) if audio.size else 1.0
    if peak > 0:
        audio = (audio / peak * 0.95).astype(np.float32)
    return audio

filter = design_bandpass_filter(config.SAMPLE_RATE, config.HIGHPASS_HZ, config.LOWPASS_HZ)