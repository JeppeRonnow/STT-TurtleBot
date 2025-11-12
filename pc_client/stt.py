import numpy as np
import whisper
import time
from collections import deque
import threading
import queue


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

if __name__ == "__main__":
    import sounddevice as sd
    from config import Config
    from dsp import DSP
    
    config = Config()
    filter = DSP(config.SAMPLE_RATE, config.HIGHPASS_HZ, config.LOWPASS_HZ, config.DEBUG)
    stt = STT(config.MODEL_NAME, config.MODEL_DEVICE, config.BUFFER_SECONDS, config.SAMPLE_RATE, config.MAX_BUFFER_LENGTH, config.DEBUG)
    model = stt.load_model()
    
    print("Say 'turtle' to test wake word detection. Ctrl+C to stop.\n")
    
    blocksize = int(config.CHUNK_SECONDS * config.SAMPLE_RATE)
    
    try:
        with sd.InputStream(channels=1, samplerate=config.SAMPLE_RATE, blocksize=blocksize, dtype='float32', callback=stt.audio_callback, device=config.AUDIO_DEVICE):
            while True:
                try:
                    stt.get_tick_q(timeout=1.0)
                except queue.Empty:
                    continue
                
                with stt.get_buffer_lock():
                    if not stt.get_audio_buffer():
                        continue
                    raw = np.array(stt.get_audio_buffer(), dtype=np.float32)
                
                filtered = filter.normalize(filter.apply_filters(raw))
                stt.transcribe(model, filtered)
                stt.print_transcription()
                
                detected, wake_word, position = stt.listen_for_wake_word()
                if detected:
                    print(f"\nWAKE WORD '{wake_word}' DETECTED!\n")
                    stt.strip_transcription(position + 1)
                    
    except KeyboardInterrupt:
        print("\nStopped.")
