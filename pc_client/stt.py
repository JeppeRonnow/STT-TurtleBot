import queue
import threading
import time
from collections import deque

import numpy as np
import sounddevice as sd  # for simple wake
import whisper


class STT:
    def __init__(
        self,
        MODEL_NAME,
        MODEL_DEVICE,
        BUFFER_SECONDS,
        SAMPLERATE,
        MAX_BUFFER_LENGTH,
        DEBUG,
    ) -> None:
        self.model_name = MODEL_NAME
        self.model_device = MODEL_DEVICE
        self.max_buffer_samples = BUFFER_SECONDS * SAMPLERATE
        self.max_buffer_length = MAX_BUFFER_LENGTH
        self.DEBUG = DEBUG
        self.buffer_lock = threading.Lock()
        self.audio_buffer = deque()
        self.tick_q = queue.Queue()
        self.transcription = []
        if self.DEBUG:
            print(f"[STT class initialized]")

    # Load whisper model
    def load_model(self):
        if self.DEBUG:
            print("Loading Whisper model ...")
        self.model = whisper.load_model(self.model_name, device=self.model_device)
        if self.DEBUG:
            print("Model loaded. Starting stream. Press Ctrl+C to stop.")

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
            result = model.transcribe(
                audio, fp16=False, language="en", verbose=None, no_speech_threshold=0.5
            )
            segments = result.get("segments", [])
            for segment in segments:
                text = segment.get("text", "").strip()
                if self.DEBUG:
                    print(f"Raw text: {text}")
                if text:
                    if self.DEBUG:
                        print(text)
                    for word in text.split():
                        self.add_transcription(word)
            # Clear buffer after transcription to avoid repeats
            with self.buffer_lock:
                self.audio_buffer.clear()
        except Exception as e:
            if self.DEBUG:
                print(f"[Transcribe error] {e}", flush=True)

    def add_transcription(self, word: str):
        w = self.clean_word(word)
        self.transcription.append(w)
        if len(self.transcription) > self.max_buffer_length:
            del self.transcription[: -self.max_buffer_length]

    # wake word
    def listen_for_wake_word(self, wake_words=None, window_size=5):
        if wake_words is None:
            wake_words = ["hey turtle", "ok turtle"]

        # get recent transcription as string
        recent = (
            self.transcription[-window_size:] if len(self.transcription) > 0 else []
        )
        recent_text = " ".join(recent).lower()

        if self.DEBUG:
            print(f"[Wake word] Checking: {recent_text}")

        # check if any wake phrase is in the recent text
        for wake_phrase in wake_words:
            wake_phrase_lower = wake_phrase.lower()
            if wake_phrase_lower in recent_text:
                # find approximate position
                words_in_phrase = wake_phrase.split()
                phrase_len = len(words_in_phrase)

                # try to find the phrase in recent words
                for i in range(len(recent) - phrase_len + 1):
                    window = " ".join(recent[i : i + phrase_len]).lower()
                    if window == wake_phrase_lower:
                        actual_position = len(self.transcription) - len(recent) + i
                        if self.DEBUG:
                            print(
                                f"[Wake word] Detected '{wake_phrase}' at position {actual_position}"
                            )
                        return (True, wake_phrase, actual_position)

        return (False, None, -1)

    # even simpler wake
    def simple_wake_word(self, trigger_words=None, duration=4):
        if trigger_words is None:
            trigger_words = ["ok", "okay", "hello", "hey"]

        samplerate = 16000
        chunk_samples = int(samplerate * duration)

        # Record audio chunk
        audio = sd.rec(
            chunk_samples, samplerate=samplerate, channels=1, dtype="float32"
        )
        sd.wait()
        audio = audio.flatten()

        # Transcribe
        result = self.model.transcribe(audio, fp16=False, language="en")
        raw_text = result["text"]
        text = self.clean_word(raw_text)

        print(f"You said: {raw_text} -> cleaned: {text}")

        # Check if "turtle" is in the text AND any trigger word
        if "turtle" in text:
            for trigger in trigger_words:
                if trigger in text:
                    print(f"Wake word detected: '{trigger}' + 'turtle'!")
                    return True

        # No wake word detected
        return False

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
        punct_to_remove = str.maketrans("", "", ".,!?;:()[]{}\"'''`…")
        w = w.translate(punct_to_remove)
        w = " ".join(w.split())
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
    from config import Config

    config = Config()
    stt = STT(
        config.MODEL_NAME,
        config.MODEL_DEVICE,
        config.BUFFER_SECONDS,
        config.SAMPLE_RATE,
        config.MAX_BUFFER_LENGTH,
        config.DEBUG,
    )

    stt.load_model()

    print("Simple wake word test\n")
    print(
        "Listening for: any trigger word (ok/okay/hello/hey) + 'turtle' anywhere in sentence\n"
    )

    # Wait for wake word
    while True:
        detected = stt.simple_wake_word(duration=2)
        if detected:
            print(f"\nWake word detected! Exiting...")
            break
