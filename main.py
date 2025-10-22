import sounddevice as sd
import numpy as np
import queue
import time

from mqtt import MQTT_Client
import config
import record
import logic
import stt
import dsp

def main():
    # Load Whisper STT model
    model = stt.load_model()

    #mqtt = MQTT_Client("localhost", 1883)
    
    next_transcribe_time = 0.0
    blocksize = int(config.CHUNK_SECONDS * config.SAMPLE_RATE)

    try:
        with sd.InputStream(channels=1, samplerate=config.SAMPLE_RATE, blocksize=blocksize, dtype='float32', callback=stt.audio_callback, device=config.DEVICE):
            while True:
                try:    # Wait for next chunk signal
                    config.tick_q.get(timeout=1.0)
                except queue.Empty:
                    continue

                now = time.time()
                if now < next_transcribe_time:
                    continue
                next_transcribe_time = now + config.CHUNK_SECONDS * 0.9  # slight overlap

                with config.buffer_lock:
                    if not config.audio_buffer:
                        continue
                    raw = np.array(config.audio_buffer, dtype=np.float32)

                # Apply filters
                filtered = dsp.apply_filters(raw)

                # Normalize lightly to (-1,1)
                filtered = dsp.normalize(filtered)

                # Save audio for reference/debugging
                record.save_audio(raw, filtered)

                # Run STT and update token buffer
                stt.transcribe(model, filtered)

                # Print current transcription
                stt.print_transcription()

                # Handle commands from tokens
                words = stt.get_transcription()
                payload, consumed = logic.handle_transcription(words)
                if payload:
                    print("[Payload]:", payload)
                    #mqtt.send_packet("robot/command", payload)
                if consumed:
                    stt.strip_transcription(consumed)
    finally:
        pass
        #mqtt.disconnect()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nScript stopped succesfully...")