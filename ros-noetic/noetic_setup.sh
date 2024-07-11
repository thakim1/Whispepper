# !/bin/bash

# Delete all previously exited containers
# docker rm $(docker ps -a -f status=exited -q)

# removing image 
docker rmi -f rv2024_noetic_pepper_image
# building
docker build -t rv2024_noetic_pepper_image .
# running the image
docker run -it --rm --name rv2024_noetic_pepper_container --network=host --ipc=host -v .:/home rv2024_noetic_pepper_image
