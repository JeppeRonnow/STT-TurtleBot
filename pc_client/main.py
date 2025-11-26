import threading

from config import Config
from Dashboard import Dashboard
from dsp import DSP
from logic import Logic
from move_timer import Move_Timer
from mqtt import MQTT_Transmitter
from record import Record
from stt import STT, WakeWord


class RobotController:
    def __init__(self):
        self.running = False
        self.thread = None

        # Configuration parameters
        self.config = Config()  # Load config variables from YAML file

        # Init objects
        self.mqtt = MQTT_Transmitter(self.config.SERVER, self.config.DEBUG)
        self.move_timer = Move_Timer(
            self.mqtt,
            self.config.MOVE_VELOCITY,
            self.config.TURN_VELOCITY,
            self.config.DEBUG,
        )
        self.logic = Logic(
            self.move_timer,
            self.config.PAUSE_ITTERATIONS,
            self.config.DEFAULT_TURN_DEG,
            self.config.DEFAULT_DISTANCE,
            self.config.MOVE_VELOCITY,
            self.config.TURN_VELOCITY,
            self.config.DEBUG,
        )
        self.filter = DSP(
            self.config.SAMPLE_RATE,
            self.config.HIGHPASS_HZ,
            self.config.LOWPASS_HZ,
            self.config.FILTER_ORDER,
            self.config.DEBUG,
        )
        self.audio = Record(self.config.SAMPLE_RATE, self.config.DEBUG)
        self.whisper = STT(
            self.config.MODEL_NAME,
            self.config.MODEL_DEVICE,
            self.config.MAX_BUFFER_LENGTH,
            self.config.DEBUG,
        )
        self.wakeWord = WakeWord(
            self.config.MODEL_PATH,
            self.config.SAMPLE_RATE,
            self.config.WAKE_WORD_BLOCK_SIZE,
            self.config.WAKE_WORD_THRESHOLD,
            self.config.DEBUG,
        )

        # Load Whisper STT model
        self.model = self.whisper.load_model()

    def run(self):
        """Main control loop - runs on separate thread"""
        self.running = True
        print(
            "[Thread] Robot controller started on thread:",
            threading.current_thread().name,
        )

        while self.running:
            try:
                # Wait for wake word
                self.wakeWord.await_wake_word()

                # Record audio and save
                raw = self.audio.record_audio(
                    self.config.CHUNK_SECONDS, self.config.AUDIO_DEVICE
                )
                self.audio.save_audio(raw, "Raw.wav")

                # Apply filters and save
                filtered = self.filter.apply_filters(raw)
                self.audio.save_audio(filtered, "Filtered.wav")

                # Normalize lightly to (-1,1)
                normalized = self.filter.normalize(filtered)
                self.audio.save_audio(normalized, "Normalized.wav")

                # Run STT and update token buffer
                self.whisper.transcribe(self.model, normalized)

                # Handle commands from tokens
                while self.running:
                    # Print current transcription
                    self.whisper.print_transcription()

                    # Get current words
                    words = self.whisper.get_transcription()
                    self.whisper.strip_transcription()
                    payload = self.logic.handle_transcription(words)
                    if payload:
                        if self.config.DEBUG:
                            print("[Payload]:", payload)
                        velocities = self.logic.payload_to_velocities(payload)
                        self.mqtt.publish_command(velocities[0], velocities[1])
                    else:
                        break

            except KeyboardInterrupt:
                print("\n[Thread] Ctrl+C detected in thread")
                self.stop()
                break
            except Exception as e:
                print(f"[Thread] Error in control loop: {e}")
                if self.config.DEBUG:
                    import traceback

                    traceback.print_exc()

        print("[Thread] Robot controller stopped")

    def start(self):
        """Start the controller on a separate thread"""
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(
                target=self.run, name="RobotControllerThread", daemon=True
            )
            self.thread.start()
            print("[Main] Controller thread started")
        else:
            print("[Main] Controller thread is already running")

    def stop(self):
        """Stop the controller thread"""
        print("[Main] Stopping controller...")
        self.running = False
        if self.thread and self.thread.is_alive():
            self.mqtt.publish_command(0.0, 0.0)
            self.thread.join(timeout=5.0)
            if self.thread.is_alive():
                print("[Main] Warning: Thread did not stop gracefully")
        self.mqtt.close_connectio()
        print("[Main] Controller stopped successfully")

    def is_running(self):
        """Check if controller thread is running"""
        return self.running and self.thread and self.thread.is_alive()


def main():
    """Main entry point - runs GUI on main thread"""
    print("[Main] Starting on main thread:", threading.current_thread().name)

    # Create controller instance
    controller = RobotController()

    # Start controller on separate thread
    controller.start()

    # Create and run Dashboard on main thread
    try:
        print("[Main] Starting Dashboard GUI on main thread")
        app = Dashboard()

        # TODO: Connect dashboard to controller for live updates
        # This will require adding methods to update dashboard with:
        # - Robot position/velocity
        # - Audio/LDR sensor data
        # - Transcription results

        app.mainloop()  # This blocks until window is closed

    except KeyboardInterrupt:
        print("\n[Main] Ctrl+C pressed in main thread")
    finally:
        controller.stop()
        print("[Main] Script stopped successfully")


if __name__ == "__main__":
    main()
