# ===== Keys, Constants =====
# openai API Key
API_KEY = ''
# REST API for Llama docker container
OLLAMA_RES_ID =  "http://172.17.0.2:11434/api/chat"

# models
LLAMA_MODEL = "llama3"
WHISPER_LANG = "en"

#system message
SYS_MSG = [{"role": "system", "content": "You are Pepper the robot from Aldebaran Robotics. Answer only briefly and in less than three sentences."}]

# ===== Flags =====
LLAMA_RESPONSE = True
CGPT_RESPONSE = False


# ==== Audio stuff ====
RATE = 16000
CHUNK_DURATION_MS = 30
CHUNK = int(RATE * CHUNK_DURATION_MS / 1000)  # Number of frames per buffer
ACCUMULATE_DURATION = 10  # Duration to accumulate audio before transcription in seconds


# ===== Flags for deprecated V1=====
CLEAR_TO_PROCESS = False
CLEAR_TO_PUBLISH = False
