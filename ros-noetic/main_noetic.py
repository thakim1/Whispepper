"""
main python3.8 Module - Whispepper
"""
import rospy
from audio_common_msgs.msg import AudioData
import whisper
import numpy as np
import threading
import queue
import webrtcvad
from std_msgs.msg import String
from parameters import RATE, CHUNK_DURATION_MS, WHISPER_LANG
from parameters import OLLAMA_RES_ID, LLAMA_MODEL, SYS_MSG
import json
import requests
import warnings

# Suppress specific warning
warnings.filterwarnings("ignore", message="FP16 is not supported on CPU; using FP32 instead")


# Load the Whisper model
whisper_model = whisper.load_model("tiny")

# Initialize VAD
vad = webrtcvad.Vad()
vad.set_mode(3)  # 0: very sensitive, 3: least sensitive

# Messages for Llama context 
conversation_messages = []

# Queue to hold audio data for transcription
audio_queue = queue.Queue()

# ROS publisher for transcribed text
transcription_pub = rospy.Publisher('/audio/transcription', String, queue_size=10)


def query_llama(messages, is_user=True):
    try:
        r = requests.post(
            OLLAMA_RES_ID,
            json={"model": LLAMA_MODEL, "messages": messages, "stream": True},
            stream=True
        )
        r.raise_for_status()

        if is_user:
            print('Response: ')

        output = ""
        message_buffer = ""
        for line in r.iter_lines():
            if line:
                body = json.loads(line)

                if "error" in body:
                    raise Exception(body["error"])

                if body.get("done") is False:
                    message = body.get("message", {})
                    content = message.get("content", "")
                    message_buffer += content
                    output += content

                    # Check if we have a complete sentence
                    if output.strip() and output.strip()[-1] in ".!?" and is_user:
                        sentence = output.strip()
                        print(sentence)
                        transcription_pub.publish(sentence)
                        output = ""
                      
                if body.get("done", False):
                    message["content"] = message_buffer
                    return message

    except requests.RequestException as e:
        print(f"Request to LLama API failed: {e}")

    except Exception as e:
        print(f"Error processing LLama API response: {e}")

    return None

def is_speech(data, sample_rate):

    # Ensure the data is the correct size for WebRTC VAD
    frame_duration = 30  # in milliseconds
    frame_size = int(sample_rate * frame_duration / 1000) * 2  # number of bytes

    if len(data) < frame_size:
        # Pad the data if it's not long enough
        data = data.ljust(frame_size, b'\0')
    elif len(data) > frame_size:
        # Trim the data if it's too long
        data = data[:frame_size]

    return vad.is_speech(data, sample_rate)

def transcribe_audio():
    accumulated_data = []
    accumulated_duration = 0
    silent_chunks = 0
    #initial_warmup_chunks = 10  # Number of chunks to discard at start
    #warmup_counter = 0
    max_silent_chunks = int(0.5 / (CHUNK_DURATION_MS / 1000))  # 0.5 seconds of silence
    noise_threshold = 500  # Adjust this threshold based on your noise level

    while True:
        
        print("silent chunks: ", silent_chunks, " speech chunks: ", len(accumulated_data), "listening...", end='\r')

        # Get audio data from the queue
        audio_data = audio_queue.get()
        if audio_data is None:
            break

        # Discard initial warmup chunks
        #if warmup_counter < initial_warmup_chunks:
        #    warmup_counter += 1
        #    continue

        if np.max(np.frombuffer(audio_data, np.int16)) < noise_threshold:
            # Ignore chunks with low amplitude
            continue

        if is_speech(audio_data, RATE):
            silent_chunks = 0
            accumulated_data.append(audio_data)
            accumulated_duration += len(audio_data) / (RATE * 2)
        else:
            silent_chunks += 1

        if silent_chunks > max_silent_chunks and accumulated_data and len(accumulated_data) > 3: # One bug remains^^ 
            print("Reached max. silent chunks: ", max_silent_chunks ," after speech. Start transcribing...")


            audio_data_combined = b''.join(accumulated_data)
            audio_np = np.frombuffer(audio_data_combined, np.int16).astype(np.float32) / 32768.0
            result = whisper_model.transcribe(audio_np, language=WHISPER_LANG)
            transcription = result['text']

            if transcription.strip():  # Publish only if there's meaningful transcription
                print("Transcription: ", transcription)
                
                # LLama response
                conversation_messages.append({"role": "user", "content": transcription})
                llm_response = query_llama(conversation_messages)
                conversation_messages.append(llm_response)            
            
            accumulated_data = []
            accumulated_duration = 0
            silent_chunks = 0

def audio_callback(msg):
    # Put the audio data into the queue for transcription
    audio_queue.put(msg.data)

def main():

    # Give Llama context
    conversation_messages.append(query_llama(SYS_MSG, is_user=False))

    # Wait for user 
    input("Press Enter to start.")

    # Create and start the transcription thread
    transcription_thread = threading.Thread(target=transcribe_audio)
    transcription_thread.start()

    # Rop node
    rospy.init_node('audio_transcriber', anonymous=True)
    
    # Ros subsriber to pepper microphone stream
    rospy.Subscriber('/audio/audio', AudioData, audio_callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stopping...")

    # Stop the transcription thread
    audio_queue.put(None)
    transcription_thread.join()

    print("Stopped.")

if __name__ == "__main__":
    main()
