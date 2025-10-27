import sounddevice as sd
import numpy as np
import queue
import time

from mqtt import MQTT_Transmitter
from config import Config
from record import Record
from logic import Logic
from stt import STT
from dsp import DSP

def main():
    # Configuration parameters
    config = Config() # Load config variables from YAML file

    # Init objects
    filter = DSP(config.SAMPLE_RATE, config.HIGHPASS_HZ, config.LOWPASS_HZ, config.DEBUG)                                                     # Bandpass filter
    logic = Logic(config.PAUSE_ITTERATIONS, config.DEFAULT_TURN_DEG, config.DEFAULT_DISTANCE_CM, config.DEBUG)                                # Logic control
    audio = Record(config.SAMPLE_RATE, config.DEBUG)                                                                                          # Audio recorder
    whisper = STT(config.MODEL_NAME, config.MODEL_DEVICE, config.BUFFER_SECONDS, config.SAMPLE_RATE, config.MAX_BUFFER_LENGTH, config.DEBUG)  # Speach to Text
    turtle = MQTT_Transmitter(config.SERVER, config.DEBUG)                                                                                    # MQTT_Transmitter

    # Load Whisper STT model
    model = whisper.load_model()

    next_transcribe_time = 0.0
    blocksize = int(config.CHUNK_SECONDS * config.SAMPLE_RATE)

    try:
        with sd.InputStream(channels=1, samplerate=config.SAMPLE_RATE, blocksize=blocksize, dtype='float32', callback=whisper.audio_callback, device=config.AUDIO_DEVICE):
            while True:
                try:    # Wait for next chunk signal
                    whisper.get_tick_q(timeout=1.0)
                except queue.Empty:
                    continue

                now = time.time()
                if now < next_transcribe_time:
                    continue
                next_transcribe_time = now + config.CHUNK_SECONDS * 0.9  # slight overlap

                with whisper.get_buffer_lock():
                    if not whisper.get_audio_buffer():
                        continue
                    raw = np.array(whisper.get_audio_buffer(), dtype=np.float32)
                                   # Apply filters
                filtered = filter.apply_filters(raw)
                
                # Normalize lightly to (-1,1)
                filtered = filter.normalize(filtered)

                # Save audio for reference/debugging
                audio.save_audio(raw, filtered)

                # Run STT and update token buffer
                whisper.transcribe(model, filtered)

                # Print current transcription
                whisper.print_transcription()

                # Handle commands from tokens
                words = whisper.get_transcription()
                payload, consumed = logic.handle_transcription(words)
                if payload:
                    print("[Payload]:", payload)
                    
                    velocities = logic.payload_to_velocities(payload)
                    #turtle.publish_command(velocities[0], velocities[1])
                    
                    whisper.clear_transcribe()

                if consumed:
                    whisper.strip_transcription(consumed)

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Sending stop command (linear.x=0, angular.z=0) and disconnecting.")

        # Send stop command with zero velocities
        turtle.publish_command(0.0, 0.0)

    finally:
        turtle.close_connectio()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript stopped succesfully...")
