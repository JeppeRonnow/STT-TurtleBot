import numpy as np
import whisper
import time
from collections import deque
import threading
import queue

import sounddevice as sd # for simple wake

class STT:
    model_name = ""
    model_device = ""
    max_buffer_samples = 0
    buffer_lock = threading.Lock()
    audio_buffer = deque()
    tick_q = queue.Queue()
    error_words = ""
    max_buffer_length = 0
    transcription = []
    DEBUG = False
    
    # Object init
    def __init__(self, MODEL_NAME, MODEL_DEVICE, BUFFER_SECONDS, SAMPLERATE, MAX_BUFFER_LENGTH, DEBUG) -> None:
        self.model_name = MODEL_NAME
        self.model_device = MODEL_DEVICE
        self.max_buffer_samples = BUFFER_SECONDS * SAMPLERATE
        self.max_buffer_length = MAX_BUFFER_LENGTH
        self.DEBUG = DEBUG
        if self.DEBUG: print(f"[STT class initialized]")

    # Load whisper model
    def load_model(self):
        if self.DEBUG: print("Loading Whisper model ...")
        model = whisper.load_model(self.model_name, device=self.model_device)
        if self.DEBUG: print("Model loaded. Starting stream. Press Ctrl+C to stop.")
        return model


    def audio_callback(self, indata, frames, time_info, status):
        mono = indata.mean(axis=1).astype(np.float32)  # ensure mono
        with self.buffer_lock:
            self.audio_buffer.extend(mono)
            # Trim overflow by removing oldest samples
            while len(self.audio_buffer) > self.max_buffer_samples:
                self.audio_buffer.popleft()
        # Signal once per chunk
        self.tick_q.put(time.time())


    def transcribe(self, model, audio):
        try:
            audio = audio.astype(np.float32, copy=False)
            result = model.transcribe(audio, fp16=False, language='en', verbose=None, no_speech_threshold=0.5)
            segments = result.get("segments", [])
            for segment in segments:
                text = segment.get("text", "").strip()
                if self.DEBUG: print(f"Raw text: {text}")
                if text:
                    if self.DEBUG: print(text)
                    for word in text.split():
                        self.add_transcription(word)
            # Clear buffer after transcription to avoid repeats
            with self.buffer_lock:
                self.audio_buffer.clear()
        except Exception as e:
            if self.DEBUG: print(f"[Transcribe error] {e}", flush=True)


    def add_transcription(self, word: str):
        w = self.clean_word(word)
        self.transcription.append(w)
        if len(self.transcription) > self.max_buffer_length:
            del self.transcription[:-self.max_buffer_length]

    # wake word
    def listen_for_wake_word(self, wake_words=None, window_size=5):
        if wake_words is None:
            wake_words = ["turtle"]
        
        # clean ord
        wake_words = [self.clean_word(w) for w in wake_words]
        
        # get recent transcription
        recent = self.transcription[-window_size:] if len(self.transcription) > 0 else []
        
        if self.DEBUG:
            print(f"[Wake word] Checking: {recent}")
        
        # takes transcription and checks if one of wake words are init
        for i, word in enumerate(recent):
            if word in wake_words:
                actual_position = len(self.transcription) - len(recent) + i
                if self.DEBUG:
                    print(f"[Wake word] Detected '{word}' at position {actual_position}")
                return (True, word, actual_position)
        
        return (False, None, -1)


    # even simpler wake
    def simple_wake_word(self, wake_word="turtle", duration=4):
        print(f"Listening for wake word: '{wake_word}'")
        
        samplerate = 16000
        chunk_samples = int(samplerate * duration)
        
        while True:
            # Record audio chunk
            audio = sd.rec(chunk_samples, samplerate=samplerate, channels=1, dtype='float32')
            sd.wait()  # Wait until recording is finished
            audio = audio.flatten()
            
            # Transcribe
            result = self.model.transcribe(audio, fp16=False, language="en")
            text = result["text"].lower().strip()
            
            print(f"You said: {text}")
            
            # Check for wake word
            if wake_word in text:
                print(f"Wake word '{wake_word}' detected!")
                return True


    def get_transcription(self):
        # return a shallow copy to avoid race with strip
        return list(self.transcription)


    def print_transcription(self):
        print("\n[Current transcription]:", " ".join(self.transcription))


    def strip_transcription(self, x: int):
        if x <= 0:
            return
        del self.transcription[:x]


    def clean_word(self, word: str) -> str:
        w = word.lower().strip()
        w = w.strip(".,!?;:()[]{}\"“”‘’'`…")
        return w


    def get_tick_q(self, timeout):
        return self.tick_q.get(timeout)


    def get_buffer_lock(self):
        return self.buffer_lock


    def get_audio_buffer(self):
        return self.audio_buffer

    
    def clear_transcribe(self):
        self.transcription = []


    def test_wake_word(model_path: str = None, model_name: str = None,
                       threshold: float = 0.6, cooldown_sec: float = 2.0,
                       smooth_n: int = 5, rate: int = 16000, chunk_ms: int = 80):
        """Simple test listener using openwakeword. Pass either a trained model name
        (legacy) or a full path to an ONNX file via model_path. When model_path is
        provided the function will use the ONNX inference framework and derive
        the model key from the filename if model_name is not given.
        """
        # Lazy import to avoid requiring openwakeword at module import-time
        from openwakeword import Model
        from collections import deque
        from pathlib import Path

        # If a model file is provided, use ONNX and derive model name from filename
        if model_path:
            model_file = str(Path(model_path))
            inferred_name = Path(model_file).stem
            model_key = model_name or inferred_name
            oww = Model(wakeword_models=[model_file], inference_framework="onnx")
        else:
            # Fallback to legacy name-based usage (keeps backward compatibility)
            model_key = model_name or WAKEWORD_NAME
            oww = Model(wakeword_models=[model_key], inference_framework="tflite")

        recent_scores = deque(maxlen=smooth_n)
        last_trigger = -1e9

        def test_record():
            print("record() called – implement your post-wake logic here.")

        CHUNK_SAMPLES = int(rate * chunk_ms / 1000.0)

        def callback(indata, frames, time_info, status):
            nonlocal last_trigger
            if status:
                return
            audio = (indata[:, 0] * 32767).astype(np.int16)
            scores = oww.predict(audio)
            score = scores.get(model_key, 0.0)
            recent_scores.append(score)
            smooth = float(np.mean(recent_scores)) if len(recent_scores) > 0 else 0.0
            print(f"{model_key} prob (smoothed): {smooth:.3f}", end="\r")
            if smooth >= threshold and (time.time() - last_trigger) >= cooldown_sec:
                last_trigger = time.time()
                print(f"\nWake word detected (score={smooth:.3f} ≥ {threshold:.3f})")
                test_record()

        print(f"Listening for wake word: {model_key}")
        with sd.InputStream(channels=1, samplerate=rate, blocksize=CHUNK_SAMPLES, dtype="float32", callback=callback):
            try:
                while True:
                    time.sleep(0.25)
            except KeyboardInterrupt:
                print("\nStopping...")


if __name__ == "__main__":
    from pathlib import Path
    import time

    # Constants (kept for reference / backward compatibility)
    THRESHOLD = 0.6
    COOLDOWN_SEC = 2.0
    SMOOTH_N = 5
    RATE = 16000
    CHUNK_MS = 80
    CHUNK_SAMPLES = int(RATE * CHUNK_MS / 1000.0)
    WAKEWORD_NAME = "Turtlebot"

    # Use the ONNX model file located in the repository's wake_word folder.
    # This resolves relative to the repo layout: pc_client/.. -> repo root
    model_file = Path(__file__).parent.parent / "wake_word" / "turtlebot.onnx"
    print(f"Using ONNX wake word model: {model_file}")

    # Call the test helper with the model file path (uses ONNX inference)
    stt = STT.test_wake_word(model_path=str(model_file), model_name=None,
                             threshold=THRESHOLD, cooldown_sec=COOLDOWN_SEC,
                             smooth_n=SMOOTH_N, rate=RATE, chunk_ms=CHUNK_MS)