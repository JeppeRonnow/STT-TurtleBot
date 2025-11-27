from openwakeword import Model
from pathlib import Path
import sounddevice as sd
import numpy as np
import threading
import whisper
import time

class STT:
    model_name = ""
    model_device = ""
    max_buffer_length = None
    transcription = []
    DEBUG = False
    

    # Object init
    def __init__(self, MODEL_NAME, MODEL_DEVICE, MAX_BUFFER_LENGTH, DEBUG) -> None:
        self.model_name = MODEL_NAME
        self.model_device = MODEL_DEVICE
        self.max_buffer_length = MAX_BUFFER_LENGTH
        self.DEBUG = DEBUG
        if self.DEBUG: print(f"[STT class initialized]")


    # Load whisper model
    def load_model(self) -> object:
        if self.DEBUG: print("Loading Whisper model ...")
        model = whisper.load_model(self.model_name, device=self.model_device)
        if self.DEBUG: print("Model loaded. Starting stream. Press Ctrl+C to stop.")
        return model


    def transcribe(self, model, audio) -> None:
        try:
            audio = audio.astype(np.float32, copy=False)
            result = model.transcribe(audio, fp16=False, language='en', verbose=None, no_speech_threshold=0.5)
            segments = result.get("segments", [])
            for segment in segments:
                text = segment.get("text", "").strip()
                if self.DEBUG: print(f"[Transcribe] Raw text: {text}")
                if text:
                    if self.DEBUG: print(text)
                    for word in text.split():
                        self.add_transcription(word)
        except Exception as e:
            if self.DEBUG: print(f"[Transcribe error] {e}", flush=True)


    def add_transcription(self, word: str):
        w = self.clean_word(word)
        self.transcription.append(w)
        if len(self.transcription) > self.max_buffer_length:
            del self.transcription[:-self.max_buffer_length]


    def get_transcription(self) -> list:
        # return a shallow copy to avoid race with strip
        return list(self.transcription)


    def print_transcription(self) -> None:
        print("\n[Current transcription]:", " ".join(self.transcription))


    def strip_transcription(self) -> None:
        # Clear entire transcription
        self.transcription = []


    def clean_word(self, word: str) -> str:
        w = word.lower().strip()
        w = w.strip(".,!?;:()[]{}\"“”‘’'`…")
        return w


class WakeWord:
    def __init__(self, mqtt_class, MODEL_PATH: str, SAMPLE_RATE: float, BLOCK_SIZE: float, THRESHOLD: float, WAKE_WORD_COOLDOWN: float, DEBUG: bool):
        self.mqtt = mqtt_class
        self.MODEL_PATH = MODEL_PATH
        self.THRESHOLD = THRESHOLD
        self.SAMPLE_RATE = SAMPLE_RATE
        self.BLOCK_SIZE = BLOCK_SIZE
        self.COOLDOWN = WAKE_WORD_COOLDOWN
        self.DEBUG = DEBUG
        
        self.last_detection = 0.0
        self.event = threading.Event()  

        self.initialize_model()

        if self.DEBUG: print("[WakeWord] Model loaded successfully.")


    def initialize_model(self) -> None:
        if self.DEBUG: print(f"[WakeWord] Loading model from: {str(self.MODEL_PATH)}")

        parrent = Path(__file__).resolve().parents[1]

        paths = [str(parrent / path) for path in self.MODEL_PATH]

        self.wakeWord = Model(
                wakeword_model_paths = paths,
                enable_speex_noise_suppression = True
            )


    def flag_is_set(self) -> bool:
        return self.event.is_set()


    def callback(self, indata, frames, time_info, status) -> None:
        # Thread-safe exit check
        if self.flag_is_set():
            return
    
        # Convert float32 [-1, 1] to int16 [-32768, 32767]
        audio_int16 = np.int16(indata[:, 0] * 32767)
        
        # Get predictions
        scores = self.wakeWord.predict(audio_int16)

        # Check for match with cooldown
        now = time.time()
        for name, score in scores.items():
            if self.DEBUG: print(f"Model: {name}, Score: {score:.6f}")
            if score >= self.THRESHOLD and (now - self.last_detection) > self.COOLDOWN:
                if name == "Turtlebot-2":
                    self.event.set()
                    self.last_detection = now
                    if self.DEBUG: print("Wake word detected")
                elif name == "Stop":
                    self.mqtt.publish_command(0, 0)
                    self.last_detection = now
                    if self.DEBUG: print("Stop word detected")
                else:
                    if self.DEBUG: print(f"Unrecognized wake word: {name}")


    def await_wake_word(self) -> None:
        self.event.clear()
        self.last_detection = time.time()

        with sd.InputStream(
                channels=1,
                samplerate=self.SAMPLE_RATE,
                blocksize=self.BLOCK_SIZE,
                dtype="float32",
                callback=self.callback,
            ):
            if self.DEBUG: print("Listening for wake word... Press Ctrl+C to stop.")

            self.event.wait() # Wait until wake word is detected


if __name__ == "__main__":
    import time
    import numpy as np
    import sounddevice as sd
    from pathlib import Path
    from openwakeword import Model

    SAMPLE_RATE = 16000
    BLOCK_SIZE = 1280          # 80ms chunks (optimal for openWakeWord)
    THRESHOLD = 0.002          # adjust per model
    COOLDOWN = 2.0           # debounce detections
    
    last_detection = 0.0

    # Resolve model path using pathlib
    parrent = Path(__file__).resolve().parents[1]
    model_path = parrent / "wake_word/Turtlebot-2.onnx"

    print(f"Loading wake-word model: {model_path}")
    wakeWord = Model(
        wakeword_model_paths=[str(model_path)],
        enable_speex_noise_suppression=True,
    )

    def callback(indata, frames, time_info, status):
        global last_detection

        # Convert float32 [-1, 1] to int16 [-32768, 32767]
        audio_int16 = (indata[:, 0] * 32767).astype(np.int16)
        
        # Get predictions
        scores = wakeWord.predict(audio_int16)

        now = time.time()
        for name, score in scores.items():
            print(f"Model: {name}, Score: {score:.6f}")
            if score >= THRESHOLD:
                if (now - last_detection) > COOLDOWN:
                    last_detection = now
                    print("Wake word detected")

    with sd.InputStream(
            channels=1,
            samplerate=SAMPLE_RATE,
            blocksize=BLOCK_SIZE,
            dtype="float32",
            callback=callback,
        ):
        print("Listening for wake word... Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
