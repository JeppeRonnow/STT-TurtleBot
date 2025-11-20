from scipy.signal import butter, lfilter
import numpy as np


class DSP:
    n, d = [None, None]
    sample_rate = None
    high_cut_hz = None
    low_cut_hz = None
    order = None

    def __init__(self, sample_rate, low_cut_hz, high_cut_hz, order, DEBUG):
        self.sample_rate = sample_rate
        self.low_cut_hz = low_cut_hz
        self.high_cut_hz = high_cut_hz
        self.order = order
        self.DEBUG = DEBUG

        self.n, self.d =self.create_filter() # Create the filter

        if self.DEBUG: print("[DSP class initalized]")


    # Create filter
    def create_filter(self):
        nquist = 0.5 * self.sample_rate
        low_cut = self.low_cut_hz / nquist
        high_cut = self.high_cut_hz / nquist
        n, d = butter(self.order, [low_cut, high_cut], btype='bandpass')
        return n, d


    # Apply bandpass filter to adio
    def apply_filters(self, audio):
        return lfilter(self.n, self.d, audio)


    # Normalize audio
    def normalize(self, audio):
        max_val = np.max(np.abs(audio))
        if max_val > 0:
            audio = audio / max_val
        audio = np.clip(audio, -1.0, 1.0)

        return audio.astype(np.float32)


if __name__ == '__main__':
    from record import Record
    from plot import Plot

    fs = 16000
    lp = 120.0
    hp = 5000.0
    order = 4

    filter = DSP(fs, lp, hp, order, False)

    # Read audio file
    record = Record(fs, False)
    file_name = "Raw.wav"
    audio, time = record.get_audio(file_name)

    # Filter and normalize audio
    audio_filter = filter.apply_filters(audio)
    audio_norm = filter.normalize(audio_filter)

    # Plot results
    Plot.plot_audio(file_name, audio, audio_filter, audio_norm, time)
