import soundfile as sf
import numpy as np

class Record:    
    # Set sample rate on object creation
    def __init__(self, SAMPLE_RATE, DEBUG) -> None:
        self.SAMPLE_RATE = SAMPLE_RATE
        self.DEBUG = DEBUG
        if self.DEBUG: print(f"[Record class initialized]")

    
    # Record audio and save
    def save_audio(self, audio, name):
        with sf.SoundFile(name, mode='w', samplerate=self.SAMPLE_RATE, channels=1) as file:
            file.write(audio)
        if self.DEBUG: print(f"[Audio] Saved audio as {name}")


    @staticmethod
    def get_audio(name):
        audio, sample_rate = sf.read(name, dtype='float32')
        if audio.ndim > 1:
            audio = audio[:, 0]
        time = np.arange(audio.shape[0]) / sample_rate
        return audio, sample_rate, time