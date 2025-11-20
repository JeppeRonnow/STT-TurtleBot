from mqtt import MQTT_Transmitter
from move_timer import Move_Timer
from config import Config
from record import Record
from stt import WakeWord
from logic import Logic
from stt import STT
from dsp import DSP

def main():
    # Configuration parameters
    config = Config() # Load config variables from YAML file

    # Init objects
    mqtt = MQTT_Transmitter(config.SERVER, config.DEBUG) # MQTT_Transmitter
    move_timer = Move_Timer(mqtt, config.MOVE_VELOCITY, config.TURN_VELOCITY, config.DEBUG)
    logic = Logic(move_timer, config.PAUSE_ITTERATIONS, config.DEFAULT_TURN_DEG, config.DEFAULT_DISTANCE, config.MOVE_VELOCITY, config.TURN_VELOCITY, config.DEBUG) # Logic control
    filter = DSP(config.SAMPLE_RATE, config.HIGHPASS_HZ, config.LOWPASS_HZ, config.FILTER_ORDER, config.DEBUG) # Bandpass filter
    audio = Record(config.SAMPLE_RATE, config.DEBUG) # Audio recorder
    whisper = STT(config.MODEL_NAME, config.MODEL_DEVICE, config.BUFFER_SECONDS, config.SAMPLE_RATE, config.MAX_BUFFER_LENGTH, config.DEBUG)  # Speach to Text
    wakeWord = WakeWord(config.MODEL_PATH, config.SAMPLE_RATE, config.WAKE_WORD_BLOCK_SIZE, config.WAKE_WORD_THRESHOLD, config.DEBUG) # Wake word detection

    # Load Whisper STT model
    model = whisper.load_model()

    while True:
        try:
            # Wait for wake word
            wakeWord.await_wake_word()
        
            # Record audio and save
            raw = audio.record_audio(config.CHUNK_SECONDS, config.AUDIO_DEVICE)  # Wait until recording is finished
            audio.save_audio(raw, "Raw.wav")

            # Apply filters and save
            filtered = filter.apply_filters(raw)
            audio.save_audio(filtered, "Filtered.wav")
            
            # Normalize lightly to (-1,1)
            normalized = filter.normalize(filtered)
            audio.save_audio(normalized, "Normalized.wav")

            # Run STT and update token buffer
            whisper.transcribe(model, normalized)

            # Handle commands from tokens
            while True:
                # Print current transcription
                whisper.print_transcription()
                
                # Get current words
                words = whisper.get_transcription()
                payload = logic.handle_transcription(words)
                if payload:
                    if config.DEBUG: print("[Payload]:", payload)
                    velocities = logic.payload_to_velocities(payload)
                    mqtt.publish_command(velocities[0], velocities[1])
                    whisper.strip_transcription()
                else: 
                    whisper.strip_transcription()
                    break                    

        except KeyboardInterrupt:
            print("\nCtrl+C pressed. Sending stop command (linear.x=0, angular.z=0) and disconnecting.")
            mqtt.publish_command(0.0, 0.0)
            break

    mqtt.close_connectio()
    print("\nScript stopped succesfully...")

if __name__ == "__main__":
    main()
        
