FROM ros:kinetic 

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
    && apt-get install -y nano \
    && apt-get -y install curl \
    && apt-get install -y lsb-release \
    && apt-get install -y python-pip \
    && apt-get install -y wget \
    && apt-get install -y ros-kinetic-audio-common \
    # delete package list to save size.
    && rm -rf /var/lib/apt/lists/* \\

ADD pynaoqi-python2.7-2.5.5.5-linux64 /home/naoqi 
ADD . /home/
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
# Apparently src directory and Dockerfile has to be . does not work otherwise.

# ENVIRONMENT VARIABLE FOR NAOQI LIB in Python2.7
ENV PYTHONPATH "${PYTHONPATH}/home/naoqi/lib/python2.7/site-packages/"
# ENVIRONMENT VARIABLE FOR ROS AUDIO STREAM
ENV PYTHONPATH "${PYTHONPATH}:/opt/ros/kinetic/share/audio_common/"
# ENVIRONMENT VARIABLE FOR ROS AUDIO STREAM LISTENER 
ENV PYTHONPATH "${PYTHONPATH}:/opt/ros/kinetic/share/audio_play/"
# Start ros core for communication 
CMD ["roscore"]
