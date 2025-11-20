from pathlib import Path
import soundfile as sf
import numpy as np

class Record:
    parrent = Path(__file__).resolve().parents[1]
    folder = parrent / "recordings"


    def __init__(self, SAMPLE_RATE, DEBUG) -> None:
        self.SAMPLE_RATE = SAMPLE_RATE
        self.DEBUG = DEBUG
        if self.DEBUG: print(f"[Record class initialized]")

    
    # Record audio and save
    def save_audio(self, audio, name):
        filepath = self.folder / name
        with sf.SoundFile(filepath, mode='w', samplerate=self.SAMPLE_RATE, channels=1) as file:
            file.write(audio)
        if self.DEBUG: print(f"[Audio] Saved audio as {name}")


    # Load audio from file
    def get_audio(self, name):
        filepath = self.folder / name
        audio, sample_rate = sf.read(filepath, dtype='float32')
        if audio.ndim > 1:
            audio = audio[:, 0]
        time = np.arange(audio.shape[0]) / sample_rate
        return audio, sample_rate, time