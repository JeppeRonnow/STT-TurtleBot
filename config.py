# MQTT Config
SERVER = "10.67.156.254"

# Whisper config
SAMPLE_RATE = 16000           # Whisper expects 16 kHz
HIGHPASS_HZ = 120.0           # High-pass filter cutoff
LOWPASS_HZ = 5000.0           # Low-pass filter cutoff
CHUNK_SECONDS = 2             # how often we trigger a partial transcription
BUFFER_SECONDS = 4            # rolling context length

MODEL_NAME = "tiny"       # change to "base" for lighter, "medium"/"large" for better accuracy (slower)
MODEL_DEVICE = "cpu"      # set input device index or leave None for default
AUDIO_DEVICE = None

ERROR_WORDS = {"Thank you.", 
               "Thanks for watching!"}  # Words to filter out from transcription

DEFAULT_DISTANCE_CM = 100     # 100 cm if distance not given
DEFAULT_TURN_DEG = 90         # 90 deg if angle not given
MAX_TOKENS = 10               # cap transcription token buffer to at most 10 tokens

# Deferral counters used by logic.handle_transcription
PAUSE_ITTERATIONS = 2
CURRENT_PAUSE = 0

