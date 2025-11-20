from openwakeword import Model
from collections import deque
from pathlib import Path
import sounddevice as sd
import numpy as np
import threading
import whisper
import queue
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
    def load_model(self):
        if self.DEBUG: print("Loading Whisper model ...")
        model = whisper.load_model(self.model_name, device=self.model_device)
        if self.DEBUG: print("Model loaded. Starting stream. Press Ctrl+C to stop.")
        return model


    def transcribe(self, model, audio):
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


    def get_transcription(self):
        # return a shallow copy to avoid race with strip
        return list(self.transcription)


    def print_transcription(self):
        print("\n[Current transcription]:", " ".join(self.transcription))


    def strip_transcription(self) -> None:
        # Clear entire transcription
        self.transcription = []


    def clean_word(self, word: str) -> str:
        w = word.lower().strip()
        w = w.strip(".,!?;:()[]{}\"“”‘’'`…")
        return w


class WakeWord:
    def __init__(self, MODEL_PATH: str, SAMPLE_RATE: float, BLOCK_SIZE: float, THRESHOLD: float, DEBUG: bool):
        self.MODEL_PATH = MODEL_PATH
        self.THRESHOLD = THRESHOLD
        self.SAMPLE_RATE = SAMPLE_RATE
        self.BLOCK_SIZE = BLOCK_SIZE
        self.DEBUG = DEBUG
        
        self.wake_word_detected = False

        if self.DEBUG:
            print(f"[WakeWord] Loading model from: {str(self.MODEL_PATH)}")

        parrent = Path(__file__).resolve().parents[1]
        path = parrent / self.MODEL_PATH
        self.wakeWord = Model(
                wakeword_model_paths=[str(path)],
                enable_speex_noise_suppression=True
            )

        if self.DEBUG:
            print("[WakeWord] Model loaded successfully.")


    def callback(self, indata, frames, time_info, status):
        # Exit if already detected
        if self.wake_word_detected:
            return
    
        # Convert float32 [-1, 1] to int16 [-32768, 32767]
        audio_int16 = np.int16(indata[:, 0] * 32767)
        
        # Get predictions
        scores = self.wakeWord.predict(audio_int16)

        # Check for match
        for name, score in scores.items():
            if self.DEBUG: print(f"Model: {name}, Score: {score:.6f}")
            if score >= self.THRESHOLD and not self.wake_word_detected:
                self.wake_word_detected = True
                if self.DEBUG: print("Wake word detected")
                break


    def await_wake_word(self) -> None:
        self.wake_word_detected = False

        with sd.InputStream(
                channels=1,
                samplerate=self.SAMPLE_RATE,
                blocksize=self.BLOCK_SIZE,
                dtype="float32",
                callback=self.callback,
            ):
            if self.DEBUG: print("Listening for wake word... Press Ctrl+C to stop.")

            while not self.wake_word_detected:
                time.sleep(0.1)


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
