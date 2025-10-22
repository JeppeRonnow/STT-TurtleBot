import soundfile as sf
import numpy as np
import config
import time

recorded_audio_raw = []
recorded_audio_filtered = []
recording_start_time = None
recording_done = False

def save_audio(raw, filtered, interval = 20):
    global recorded_audio_raw, recorded_audio_filtered, recording_start_time, recording_done

    if not recording_done:
        if recording_start_time is None:
            recording_start_time = time.time()
        recorded_audio_raw.append(raw)
        recorded_audio_filtered.append(filtered)
        if time.time() - recording_start_time >= interval:
            # Concatenate and save as WAV
            audio_to_save_raw = np.concatenate(recorded_audio_raw)
            audio_to_save_filtered = np.concatenate(recorded_audio_filtered)
            sf.write("Raw.wav", audio_to_save_raw, config.SAMPLE_RATE)
            sf.write("Filtered.wav", audio_to_save_filtered, config.SAMPLE_RATE)
            print("Saved first " + str(interval) + " seconds of filtered audio.")
            recording_done = True