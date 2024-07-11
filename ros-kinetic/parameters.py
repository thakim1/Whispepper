# ===== Port, IP etc. =====
NAO_PORT = 9559
NAO_IP = "169.254.218.22"
T2S_MODULE = "ALTextToSpeech"

# ===== Flags =====
PUBLISHING = True

# ===== Audio Recording =====
RECORDING_DURATION = 10     # seconds, maximum recording time, also default value for startRecording(), Google Speech API only accepts up to about 10-15 seconds
LOOKAHEAD_DURATION = 1.0    # seconds, for auto-detect mode: amount of seconds before the threshold trigger that will be included in the request
IDLE_RELEASE_TIME = 2.0     # seconds, for auto-detect mode: idle time (RMS below threshold) after which we stop recording and recognize
HOLD_TIME = 3.0             # seconds, minimum recording time after we started recording (autodetection)
SAMPLE_RATE = 16000         # Hz, be careful changing this, both google and Naoqi have requirements!

CALIBRATION_DURATION = 4    # seconds, timespan during which calibration is performed (summing up RMS values and calculating mean)
CALIBRATION_THRESHOLD_FACTOR = 1.5  # factor the calculated mean RMS gets multiplied by to determine the auto detection threshold (after calibration)

DEFAULT_LANGUAGE = "en-us"  # RFC5646 language tag, e.g. "en-us", "de-de", "fr-fr",... <http://stackoverflow.com/a/14302134>

WRITE_WAV_FILE = True      # write the recorded audio to "out.wav" before sending it to google. intended for debugging purposes
PRINT_RMS = False           # prints the calculated RMS value to the console, useful for setting the threshold

PREBUFFER_WHEN_STOP = False # Fills pre-buffer with last samples when stopping recording. WARNING: has performance issues!


# ===== Flags for deprecated V1 =====
CLEAR_TO_RECORD = True
CLEAR_TO_VOICE = False
CLEAR_TO_PUBLISH = False
