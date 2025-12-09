from mqtt import MQTT_Transmitter
from Dashboard import Dashboard
from stt import STT, WakeWord
from config import Config
from record import Record
from logic import Logic
from dsp import DSP
import threading

from mqtt_receiver import MQTT_Receiver
import numpy as np


# robot control thread
def robot_loop(config, mqtt, logic, filter, audio, whisper, wakeWord, dashboard, stop_event):
    print("[Thread] Robot controller started on thread:", threading.current_thread().name)

    # Load Whisper model
    model = whisper.load_model()

    while not stop_event.is_set():
        try:
            # Wait for wake word
            wakeWord.await_wake_word()

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
            while True:
                whisper.print_transcription()
                words = whisper.get_transcription()
                whisper.strip_transcription()
                payload = logic.handle_transcription(words)
                if payload:
                    if config.DEBUG:
                        print("[Payload]:", payload)
                    velocities = logic.payload_to_velocities(payload)
                    mqtt.publish_command(velocities[0], velocities[1])
                    #dashboard.update_last_command(velocities[0], velocities[1])
                else:
                    break

        except KeyboardInterrupt:
            print("\n[Thread] Ctrl+C detected in thread")
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
    logic = Logic(mqtt, config.DEFAULT_TURN_DEG, config.DEFAULT_DISTANCE, config.MOVE_VELOCITY, config.TURN_VELOCITY, config.DEBUG)
    filter = DSP(config.SAMPLE_RATE, config.HIGHPASS_HZ, config.LOWPASS_HZ, config.FILTER_ORDER, config.DEBUG)
    audio = Record(config.SAMPLE_RATE, config.DEBUG)
    whisper = STT(config.MODEL_NAME, config.MODEL_DEVICE, config.DEBUG)
    wakeWord = WakeWord(mqtt, config.MODEL_PATH, config.SAMPLE_RATE, config.WAKE_WORD_BLOCK_SIZE, config.WAKE_WORD_THRESHOLD, config.WAKE_WORD_COOLDOWN, config.DEBUG)
    
    # Create Dashboard first with MQTT transmitter reference
    app = Dashboard(mqtt)
    
    # Initialize MQTT Receiver with dashboard reference
    reciever = MQTT_Receiver(config.SERVER, config.DEBUG, app)
    reciever.start()

    # Event to signal thread to stop
    stop_event = threading.Event()

    # Start robot control thread
    thread = threading.Thread(
        target=robot_loop,
        args=(config, mqtt, logic, filter, audio, whisper, wakeWord, app, stop_event),
        name="RobotControllerThread",
        daemon=True
    )
    thread.start()
    if config.DEBUG: print("Controller thread started")

    # Start GUI main loop
    try:
        app.mainloop()

    except KeyboardInterrupt:
        print("\nCtrl+C pressed in main thread")

    finally:
        print("Stopping controller...")

        # Wait for the robot control thread to finish
        stop_event.set()
        thread.join(timeout=1.0)

        if thread.is_alive() and config.DEBUG:
            print("Warning: Thread did not stop gracefully")

        # Stop the robot and close MQTT connection
        mqtt.publish_command(0.0, 0.0)
        mqtt.close_connection()

        print("Script stopped successfully")


if __name__ == "__main__":
    main()
