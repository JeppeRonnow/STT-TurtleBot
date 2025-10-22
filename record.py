import soundfile as sf
import numpy as np
import time

class Record:
    sample_rate = ""
    recorded_audio_raw = []
    recorded_audio_filtered = []
    recording_start_time = None
    recording_done = False
    
    # Set sample rate on object creation
    def __init__(self, SAMPLE_RATE) -> None:
        self.sample_rate = SAMPLE_RATE
        pass

    
    # Record audio and save
    def save_audio(self, raw, filtered, interval = 20):
        if not self.recording_done:
            if self.recording_start_time is None:
                self.recording_start_time = time.time()
            self.recorded_audio_raw.append(raw)
            self.recorded_audio_filtered.append(filtered)
            if time.time() - self.recording_start_time >= interval:
                # Concatenate and save as WAV
                audio_to_save_raw = np.concatenate(self.recorded_audio_raw)
                audio_to_save_filtered = np.concatenate(self.recorded_audio_filtered)
                sf.write("Raw.wav", audio_to_save_raw, self.sample_rate)
                sf.write("Filtered.wav", audio_to_save_filtered, self.sample_rate)
                print("Saved first " + str(interval) + " seconds of filtered audio.")
                self.recording_done = True
