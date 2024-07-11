# !/bin/bash

# Delete all previously exited containers if needed
# docker rm $(docker ps -a -f status=exited -q)

# create volume for saving wav files
docker volume create rv2024_pepper_volume
# print info to get file path
docker volume inspect rv2024_pepper_volume
# remove built image
docker rmi -f rv2024_kinetic_pepper_image
# build newly
docker build -t rv2024_kinetic_pepper_image .

# -v references the volume to access written audio files, this was needed in V1 (deprecated main)
# because it didn't use the ROS audio-message stream but .wav files instead. 
# run it
docker run -it --rm --name rv2024_kinetic_pepper_container \
    -v rv2024_pepper_volume:/home/volume_save/ \ 
    --ipc=host \
    --network=host \
    rv2024_kinetic_pepper_image

