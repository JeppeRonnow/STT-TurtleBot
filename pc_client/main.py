import threading
from config import Config
from Dashboard import Dashboard
from dsp import DSP
from logic import Logic
from move_timer import Move_Timer
from mqtt import MQTT_Transmitter
from record import Record
from stt import STT, WakeWord
from SQLiteDB import SQLiteDB

from mqtt_receiver import MQTT_Receiver
import sounddevice as sd
import numpy as np
import queue
import time

# robot control thread:wav
def robot_loop(running_flag, config, mqtt, move_timer, logic, filter, audio, whisper, wakeWord, model, dashboard, sqlitedb):
    print("[Thread] Robot controller started on thread:", threading.current_thread().name)

    while running_flag[0]:
        try:
            # Wait for wake word
            wakeWord.await_wake_word()

            # Start timer for benchmarking
            start_time = time.perf_counter()

            # Record audio and save
            raw = audio.record_audio(config.CHUNK_SECONDS, config.AUDIO_DEVICE)
            audio.save_audio(raw, "Raw.wav")

            # Apply filters and save
            filtered = filter.apply_filters(raw)
            audio.save_audio(filtered, "Filtered.wav")

            # Normalize lightly to (-1,1)
            normalized = filter.normalize(filtered)
            audio.save_audio(normalized, "Normalized.wav")

            # Update dashboard with audio recordings
            raw_time = np.arange(len(raw)) / config.SAMPLE_RATE
            filtered_time = np.arange(len(filtered)) / config.SAMPLE_RATE
            dashboard.update_audio_recordings(raw, raw_time, filtered, filtered_time)

            # Run STT and update token buffer
            whisper.transcribe(model, normalized)

            # Handle commands from tokens
            while running_flag[0]:
                whisper.print_transcription()
                words = whisper.get_transcription()
                transcription_text = " ".join(words)  # Save before stripping
                whisper.strip_transcription()
                payload = logic.handle_transcription(words)
                
                end_time = time.perf_counter()
                elapsed_ms = (end_time - start_time) * 1000

                # Save data in sqlite database
                sqlitedb.add_recording(
                    str(audio.folder / "Raw.wav"),
                    str(audio.folder / "Filtered.wav"),
                    str(audio.folder / "Normalized.wav"),
                    transcription_text,
                    elapsed_ms
                )
                
                if payload:
                    if config.DEBUG:
                        print("[Payload]:", payload)
                    velocities = logic.payload_to_velocities(payload)
                    mqtt.publish_command(velocities[0], velocities[1])
                else:
                    break

        except KeyboardInterrupt:
            print("\n[Thread] Ctrl+C detected in thread")
            running_flag[0] = False
            break

        except Exception as e:
            print(f"[Thread] Error in control loop: {e}")
            if config.DEBUG:
                import traceback
                traceback.print_exc()

    print("[Thread] Robot controller stopped")


# main thread
def main():
    print("Starting on main thread:", threading.current_thread().name)

    # Configuration and init objects
    config = Config()
    mqtt = MQTT_Transmitter(config.SERVER, config.DEBUG)
    move_timer = Move_Timer(mqtt, config.MOVE_VELOCITY, config.TURN_VELOCITY, config.DEBUG)
    logic = Logic(move_timer, config.DEFAULT_TURN_DEG, config.DEFAULT_DISTANCE, config.MOVE_VELOCITY, config.TURN_VELOCITY, config.DEBUG)
    filter = DSP(config.SAMPLE_RATE, config.HIGHPASS_HZ, config.LOWPASS_HZ, config.FILTER_ORDER, config.DEBUG)
    audio = Record(config.SAMPLE_RATE, config.DEBUG)
    whisper = STT(config.MODEL_NAME, config.MODEL_DEVICE, config.MAX_BUFFER_LENGTH, config.DEBUG)
    wakeWord = WakeWord(mqtt, config.MODEL_PATH, config.SAMPLE_RATE, config.WAKE_WORD_BLOCK_SIZE, config.WAKE_WORD_THRESHOLD, config.WAKE_WORD_COOLDOWN, config.DEBUG)
    sqlitedb = SQLiteDB(config.DB_PATH)

    # Load Whisper model
    model = whisper.load_model()
    
    # Create Dashboard first with MQTT transmitter reference
    app = Dashboard(mqtt_transmitter=mqtt)
    
    # Initialize MQTT Receiver with dashboard reference
    reciever = MQTT_Receiver(config.SERVER, config.DEBUG, app)
    
    # Start MQTT Receiver thread
    reciever.start()

    # Shared flag for thread control (mutable so thread can see changes)
    running = [True]

    # Start robot control thread
    thread = threading.Thread(
        target=robot_loop,
        args=(running, config, mqtt, move_timer, logic, filter, audio, whisper, wakeWord, model, app, sqlitedb),
        name="RobotControllerThread",
        daemon=True
    )
    thread.start()
    print("Controller thread started")

    # Start GUI main loop
    try:
        app.mainloop()
    except KeyboardInterrupt:
        print("\nCtrl+C pressed in main thread")
    finally:
        print("Stopping controller...")

        running[0] = False
        mqtt.publish_command(0.0, 0.0)
        thread.join(timeout=5.0)

        if thread.is_alive():
            print("Warning: Thread did not stop gracefully")

        mqtt.close_connection()
        print("Script stopped successfully")


if __name__ == "__main__":
    main()
