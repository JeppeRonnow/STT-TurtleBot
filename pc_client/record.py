from pathlib import Path
import sounddevice as sd
import soundfile as sf
import numpy as np

class Record:
    parrent = Path(__file__).resolve().parents[1]
    folder = parrent / "recordings"


    def __init__(self, SAMPLE_RATE, DEBUG) -> None:
        self.SAMPLE_RATE = SAMPLE_RATE
        self.DEBUG = DEBUG
        if self.DEBUG: print(f"[Record class initialized]")


    # Record audio from microphone
    def record_audio(self, duration, device) -> np.ndarray:
        if self.DEBUG: print(f"Recording {duration}s of audio...")
        audio = sd.rec(
            int(duration * self.SAMPLE_RATE),
            samplerate=self.SAMPLE_RATE,
            channels=1,
            dtype='float32',
            device=device
        )
        sd.wait() # Wait until recording is finished
        return audio.flatten()
    
    
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
        return audio, time