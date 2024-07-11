#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Get Signal from Front Microphone & Calculate its rms Power"""

from naoqi import ALProxy
import qi
import argparse
import sys
import time
import numpy as np
import threading
import rospy
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String
import parameters as par



class SoundProcessingModule(object):
    """
    A simple get signal from the front microphone of Nao & calculate its rms power.
    It requires numpy.
    """

    def __init__(self, app, session):
        """
        Initialise services and variables.
        """
        super(SoundProcessingModule, self).__init__()
        app.start()
        session = app.session

        # Get the service ALAudioDevice.
        self.audio_service = session.service("ALAudioDevice")
        self.isProcessingDone = threading.Event()
        #self.nbOfFramesToProcess = 20
        #self.framesCount = 0
        self.module_name = "SoundProcessingModule"

        # Text to speech object for voicing answers 
        self.textToSpeech = ALProxy(par.T2S_MODULE, par.NAO_IP, par.NAO_PORT)

        # ROS node and publisher
        rospy.init_node('audio_pub_node', anonymous=True)

        # /audio/audio topic important for audio play
        self.audio_pub = rospy.Publisher('/audio/audio', AudioData, queue_size=10)
        self.inputBuffer = []

        # ROS subscriber 
        self.sub = rospy.Subscriber('/audio/transcription', String, self.custom_callback)

        # ===== Motion setup =====
        # Get the services ALMotion & ALRobotPosture.
        self.motion_service = session.service("ALMotion")
        self.posture_service = session.service("ALRobotPosture")
        # Wake up robot
        self.motion_service.wakeUp()
        # Send robot to Stand Init
        self.posture_service.goToPosture("StandInit", 0.5)
        # Go to rest position
        # motion_service.rest()    

        self.asr_service = session.service("ALAnimatedSpeech")
        # set the local configuration, options: random, contextual, disabled 
        self.configuration = {"bodyLanguageMode":"contextual"}
        # say the text with the local configuration
        # self.asr_service.say("Hello, I am a robot !", self.configuration)

    def custom_callback(self, msg):

        # Perform custom actions with the received message here
        rospy.loginfo("Received: {}".format(msg.data))

        # stop recording while speaking 
        print("Pausing audio stream briefly..")
        
        #self.audio_service.unsubscribe(self.module_name)
        #self.audio_service.subscribe(self.module_name)

        # Stop publishing for the time pepper talks
        par.PUBLISHING = False
        #self.textToSpeech.say(msg.data)
        self.asr_service.say(msg.data, self.configuration)
        print(msg.data)
        par.PUBLISHING = True

        print("Resuming audio stream...")
        

    def subscribe(self):

        while not rospy.is_shutdown():
            # Perform subscription-related tasks here
            rospy.sleep(1)  # Adjust the sleep duration as needed


    def startProcessing(self):
        """
        Start processing
        """
        # Front microphone signal sampled at 16kHz
        self.audio_service.setClientPreferences(self.module_name, par.SAMPLE_RATE, 3, 0)
        self.audio_service.subscribe(self.module_name)

        # Start the voice rec thread
        threading.Thread(target=self.subscribe).start()

        while not rospy.is_shutdown() and not self.isProcessingDone.is_set():
            time.sleep(1)

        self.audio_service.unsubscribe(self.module_name)

    def processRemote(self, nbOfChannels, nbOfSamplesByChannel, timeStamp, inputBuffer):
        """
        Compute RMS from mic.

        Although arguments are unused, signature is important for functionality 
        """
        # self.framesCount += 1
        self.inputBuffer = inputBuffer

        # Convert inputBuffer to signed integer as it is interpreted as a string by Python
        micFront = self.convertStr2SignedInt(self.inputBuffer)
        # Compute the RMS level on front mic
        rmsMicFront = self.calcRMSLevel(micFront)

        sys.stdout.write("rms-level:" + str(rmsMicFront) + "\r")
        sys.stdout.flush()

        if par.PUBLISHING:
            self.publishAudioData(inputBuffer)

    def publishAudioData(self, data):
        """
        Publish audio data to the ROS topic.
        """
        audio_data = np.frombuffer(data, dtype=np.uint8)
        audio_msg = AudioData(data=audio_data.tobytes())
        
        # print("len:"+str(len(audio_data))+" data:" + str(audio_data))
        self.audio_pub.publish(audio_msg)

    def calcRMSLevel(self, data):
        """
        Calculate RMS level.
        """
        rms = 20 * np.log10(np.sqrt(np.sum(np.power(data, 2) / len(data))))
        return rms
    

    def convertStr2SignedInt(self, data):
        """
        Convert string data to signed integer values.
        """    
        signedData = []
        ind = 0
        for i in range(0, len(data) // 2):
            signedData.append(data[ind] + data[ind + 1] * 256)
            ind += 2
    
        for i in range(len(signedData)):
            if signedData[i] >= 32768:
                signedData[i] = signedData[i] - 65536
    
        for i in range(len(signedData)):
            signedData[i] = signedData[i] / 32768.0
        
        return signedData


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=par.NAO_IP,
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    
    try:
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["SoundProcessingModule", "--qi-url=" + connection_url])
        session = qi.Session().connect(connection_url)
        MySoundProcessingModule = SoundProcessingModule(app, session)
        app.session.registerService("SoundProcessingModule", MySoundProcessingModule)
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    

    MySoundProcessingModule.startProcessing()


if __name__ == "__main__":
    main()
