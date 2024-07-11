"""
Deprecated main python2.7 Module - Whispepper
"""
#!/usr/bin/env python
import rospy
import threading
import parameters
import time
from std_msgs.msg import String
from vam import SpeechRecognitionModule
import naoqi
from naoqi import ALProxy

tts = ALProxy(parameters.T2S_MODULE, parameters.NAO_IP, parameters.NAO_PORT)


CGPT_response = ""

def publish_message(pub, msgs):
    rate = rospy.Rate(1)  # Adjust the publishing rate as needed

    while not rospy.is_shutdown():
        
        if parameters.CLEAR_TO_PUBLISH: 
            pub.publish(msgs[0])        
            rospy.loginfo("Published: {}".format(msgs[0]))
            
            # Set state machine to initial state
            parameters.CLEAR_TO_PUBLISH = False
        
        rate.sleep()

def subscribe():
    while not rospy.is_shutdown():
        # Perform subscription-related tasks here
        rospy.sleep(1)  # Adjust the sleep duration as needed

def make_recording():

    #
    # TO DO: Implement THIS!
    #

    print("Recording audio...")
    # Allow to publish response when GPT processing and transcription is done
    print("Writing to file...")
    parameters.CLEAR_TO_RECORD = False
    
    # Recording is done so we signal that publishing can be done
    parameters.CLEAR_TO_PUBLISH = True
    return 

def voice_response():

    global CGPT_response
    response_string = CGPT_response
    #
    # TO DO: Implement THIS!
    #
    tts.say(response_string)

    # use global CGPT_reponse for t2speech module
    print("Doing voice thingy.")

    parameters.CLEAR_TO_VOICE = False
    # After voicing is done we can record again
    parameters.CLEAR_TO_RECORD = True

    return 

def write_msg(msg, str):
    msg[0]=str

def custom_callback(msg):
    # Perform custom actions with the received message here
    rospy.loginfo("Received: {}".format(msg.data))

    i = msg.data.find(':')
    flag = msg.data[0:i]
    global CGPT_response
    CGPT_response = msg.data[i+1:]

    print(flag)

    if (flag == 'PLAY_TEXT'):
        # Start the voicing process when recording is done
        parameters.CLEAR_TO_VOICE = True

def main():
    # Initialize the ROS node
    rospy.init_node('msg_pub_sub_P27', anonymous=True)
    
    # Create publisher and subscriber to python 3.8 module
    pub = rospy.Publisher('downlink', String, queue_size=10)
    sub = rospy.Subscriber('uplink', String, custom_callback)
    

    # Create threads for publishing 
    msg = ["flag:payload"] # Updating element in array works with threads
    publish_thread = threading.Thread(target=publish_message, args=(pub,msg))
    publish_thread.start()
    # and subscribing
    subscribe_thread = threading.Thread(target=subscribe, args=())
    subscribe_thread.start()
    
    # Create a VoiceDetection instance
    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = naoqi.ALBroker("myBroker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       parameters.NAO_IP,         # parent broker IP
       parameters.NAO_PORT)       # parent broker port

    try:
        p = ALProxy("SpeechRecognition")
        p.exit()  # kill previous instance, useful for developing ;)
    except:
        pass

    # Reinstantiate module

    # Warning: SpeechRecognition must be a global variable
    # The name given to the constructor must be the name of the
    # variable
    global SpeechRecognition
    SpeechRecognition = SpeechRecognitionModule("SpeechRecognition", parameters.NAO_IP)

    # uncomment for debug purposes
    # usually a subscribing client will call start() from ALProxy
    SpeechRecognition.start()
    SpeechRecognition.startRecording()
    SpeechRecognition.calibrate()
    SpeechRecognition.enableAutoDetection()
    SpeechRecognition.startRecording()


    time.sleep(10) # enough time to start the other python module manually

    # Perform other tasks concurrently
    while not rospy.is_shutdown():

        print("clear to pub:", parameters.CLEAR_TO_PUBLISH); print("clear to rec:", parameters.CLEAR_TO_RECORD); print("clear to voc:", parameters.CLEAR_TO_VOICE)

        if parameters.CLEAR_TO_RECORD == True:

            # this has to loop until 
            # voice has been detected and recording is 
            # done only then continue to write_msg
            # to be implemented... scroll up
            make_recording() 
            
            
            
            write_msg(msg, "REC_DONE:blank")
            parameters.CLEAR_TO_RECORD = False  


        if parameters.CLEAR_TO_VOICE == True:
            voice_response()
            parameters.CLEAR_TO_VOICE = False

        rospy.sleep(1)  # Adjust the sleep duration as needed



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
