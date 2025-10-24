import numpy as np
import whisper
import time
from collections import deque
import threading
import queue


class STT:
    model_name = ""
    model_device = ""
    max_buffer_size = ""
    buffer_lock = threading.Lock()
    audio_buffer = deque()
    tick_q = queue.Queue()
    error_words = ""
    max_tokens = ""
    transcription = []
    
    # Object init
    def __init__(self, MODEL_NAME, MODEL_DEVICE, BUFFER_SECONDS, SAMPLERATE, ERROR_WORDS, MAX_TOKENS) -> None:
        self.model_name = MODEL_NAME
        self.model_device = MODEL_DEVICE
        self.max_buffer_size = BUFFER_SECONDS * SAMPLERATE
        self.error_words = ERROR_WORDS
        self.max_tokens = MAX_TOKENS
        pass


    # Load whisper model
    def load_model(self):
        print("Loading Whisper model ...")
        model = whisper.load_model(self.model_name, self.model_device)
        print("Model loaded. Starting stream. Press Ctrl+C to stop.")
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
            result = model.transcribe(audio, fp16=False, language='en', verbose=False, no_speech_threshold=0.5)
            segments = result.get("segments", [])
            for segment in segments:
                text = segment.get("text", "").strip()
                if text and text not in self.error_words:
                    print(text)
                    for word in text.split():
                        add_transcription(word)
            # Clear buffer after transcription to avoid repeats
            with self.buffer_lock:
                self.audio_buffer.clear()
        except Exception as e:
            print(f"[Transcribe error] {e}", flush=True)


    def add_transcription(self, word: str):
        w = _clean_word(word)
        self.transcription.append(w)
        if len(self.transcription) > self.max_tokens:
            del self.transcription[:-self.max_tokens]


    def get_transcription(self):
        # return a shallow copy to avoid race with strip
        return list(self.transcription)


    def print_transcription(self):
        print("[Current transcription]:", " ".join(self.transcription))


    def strip_transcription(self, x: int):
        if x <= 0:
            return
        del self.transcription[:x]


    def _clean_word(self, word: str) -> str:
        w = word.lower().strip()
        w = w.strip(".,!?;:()[]{}\"“”‘’'`…")
        return w
