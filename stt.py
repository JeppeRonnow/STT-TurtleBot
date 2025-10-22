import numpy as np
import whisper
import time

import config

def load_model():
    print("Loading Whisper model ...")
    model = whisper.load_model(config.MODEL_NAME, device="cpu")
    print("Model loaded. Starting stream. Press Ctrl+C to stop.")
    return model

def audio_callback(indata, frames, time_info, status):
    mono = indata.mean(axis=1).astype(np.float32)  # ensure mono
    with config.buffer_lock:
        config.audio_buffer.extend(mono)
        # Trim overflow by removing oldest samples
        while len(config.audio_buffer) > config.max_buffer_samples:
            config.audio_buffer.popleft()
    # Signal once per chunk
    config.tick_q.put(time.time())

def transcribe(model, audio):
    global transcription
    try:
        audio = audio.astype(np.float32, copy=False)
        result = model.transcribe(audio, fp16=False, language='en', verbose=False, no_speech_threshold=0.5)
        segments = result.get("segments", [])
        for segment in segments:
            text = segment.get("text", "").strip()
            if text and text not in config.ERROR_WORDS:
                print(text)
                for word in text.split():
                    add_transcription(word)
        # Clear buffer after transcription to avoid repeats
        with config.buffer_lock:
            config.audio_buffer.clear()
    except Exception as e:
        print(f"[Transcribe error] {e}", flush=True)

def add_transcription(word: str):
    global transcription
    w = _clean_word(word)
    transcription.append(w)
    if len(transcription) > config.MAX_TOKENS:
        del transcription[:-config.MAX_TOKENS]

def get_transcription():
    global transcription
    # return a shallow copy to avoid race with strip
    return list(transcription)

def print_transcription():
    global transcription
    print("[Current transcription]:", " ".join(transcription))

def strip_transcription(x: int):
    global transcription
    if x <= 0:
        return
    del transcription[:x]

def _clean_word(word: str) -> str:
    w = word.lower().strip()
    w = w.strip(".,!?;:()[]{}\"“”‘’'`…")
    return w

transcription = []