import sounddevice as sd
import numpy as np
import queue
import time

from config import *
from record import Record
from logic import Logic
from stt import STT
from dsp import DSP
from mqtt_transmitter import MQTT_Transmitter

def main():
    # Init objects
    filter = DSP(SAMPLE_RATE, HIGHPASS_HZ, LOWPASS_HZ)  # Bandpass filter
    logic = Logic() # Logic control
    audio = Record(SAMPLE_RATE) # Audio recorder
    s_t_t = STT(MODEL_NAME, MODEL_DEVICE, BUFFER_SECONDS, SAMPLE_RATE, ERROR_WORDS, MAX_TOKENS) # Speach to Text
    turtle = MQTT_Transmitter(SERVER)   # MQTT_Transmitter

    # Load Whisper STT model
    model = stt.load_model()

    next_transcribe_time = 0.0
    blocksize = int(CHUNK_SECONDS * SAMPLE_RATE)

    try:
        with sd.InputStream(channels=1, samplerate=SAMPLE_RATE, blocksize=blocksize, dtype='float32', callback=stt.audio_callback, device=DEVICE):
            while True:
                try:    # Wait for next chunk signal
                    tick_q.get(timeout=1.0)
                except queue.Empty:
                    continue

                now = time.time()
                if now < next_transcribe_time:
                    continue
                next_transcribe_time = now + CHUNK_SECONDS * 0.9  # slight overlap

                with buffer_lock:
                    if not audio_buffer:
                        continue
                    raw = np.array(audio_buffer, dtype=np.float32)

                # Apply filters
                filtered = filter.apply_filters(raw)

                # Normalize lightly to (-1,1)
                filtered = filter.normalize(filtered)

                # Save audio for reference/debugging
                audio.save_audio(raw, filtered)

                # Run STT and update token buffer
                s_t_t.transcribe(model, filtered)

                # Print current transcription
                s_t_t.print_transcription()

                # Handle commands from tokens
                words = s_t_t.get_transcription()
                payload, consumed = logic.handle_transcription(words)
                if payload:
                    print("[Payload]:", payload)

                    turtle.publish_command()
                    #mqtt.send_packet("robot/command", payload)
                if consumed:
                    s_t_t.strip_transcription(consumed)
    finally:
        pass
        #mqtt.disconnect()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript stopped succesfully...")
