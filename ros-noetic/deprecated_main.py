"""
Main Python3.8 Module - Whispepper
"""
#!/usr/bin/env python
import rospy
import threading
import parameters
from std_msgs.msg import String

# transcription
import whitr

# might have to change depending on your system
audio_file_path = "/var/lib/docker/volumes/rv2024_pepper_volume/_data/out.wav"

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

def make_transcription():
    
    print("Transcribing .wav-file...")
    return whitr.transcribe(audio_file_path)

def make_GPT_Request():

    #
    # TO DO: Implement THIS!
    #

    print("Querying GPT...")
    # Allow to publish response when GPT processing and transcription is done
    print("Setting proper flags...")
    parameters.CLEAR_TO_PUBLISH = True
    return 

def write_msg(msg, str):
    msg[0]=str

def custom_callback(msg):
    # Perform custom actions with the received message here
    rospy.loginfo("Received: {}".format(msg.data))

    if (msg.data == 'REC_DONE:blank'):
        # Start the reply process when recordind is done
        parameters.CLEAR_TO_PROCESS = True

def main():
    # Initialize the ROS node
    rospy.init_node('msg_pub_sub_P38', anonymous=True)
    
    # Create publisher and subscriber to python 2.7 module
    pub = rospy.Publisher('uplink', String, queue_size=10)
    sub = rospy.Subscriber('downlink', String, custom_callback)
    

    # Create threads for publishing 
    msg = ["status:payload"]
    publish_thread = threading.Thread(target=publish_message, args=(pub,msg))
    publish_thread.start()
    # and subscribing
    subscribe_thread = threading.Thread(target=subscribe, args=())
    subscribe_thread.start()

    # Perform other tasks concurrently
    while not rospy.is_shutdown():


        print(f"clear to pro: {parameters.CLEAR_TO_PROCESS}, clear to pub: {parameters.CLEAR_TO_PUBLISH}")
       

        if parameters.CLEAR_TO_PROCESS == True:
            transcription = make_transcription() #todo
            make_GPT_Request() #todo
            write_msg(msg, "PLAY_TEXT:"+ transcription)
            parameters.CLEAR_TO_PROCESS = False            

        rospy.sleep(1)  # Adjust the sleep duration as needed



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
