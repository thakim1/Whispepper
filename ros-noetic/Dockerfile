FROM ros:noetic 
# NETWORKING:
# For networking use parameters 
# --network=host
# --ipc=host 
# when using the "docker run" command 
# in terminal to hand over the networking 
# configuration from the host. 
# UPDATES AND PACKAGES:
RUN apt-get update \
    && apt-get install -y \
        nano \
        curl \
        lsb-release \
        python3-pip \
        wget \
        ros-noetic-audio-common \
        python3-rospy \
        python3-pyaudio \
        python3-websockets \
    && pip3 install numpy==1.22.4 \
    && pip3 install openai openai-whisper webrtcvad \
    # delete package list to save size
    && rm -rf /var/lib/apt/lists/*

COPY download_models.py .
RUN python3 download_models.py
#ADD source dest
#ADD . /home/
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# Apparently src directory and Dockerfile has to be . does not work otherwise.
# ENVIRONMENT VARIABLE FOR ROS AUDIO STREAM
ENV PYTHONPATH "${PYTHONPATH}:/opt/ros/noetic/share/audio_common/"
# ENVIRONMENT VARIABLE FOR ROS AUDIO STREAM LISTENER 
ENV PYTHONPATH "${PYTHONPATH}:/opt/ros/noetic/share/audio_play/"
# Start roscore either in the kinetic or noetic container
# It must not be started by both containers!
# CMD ["roscore"]
