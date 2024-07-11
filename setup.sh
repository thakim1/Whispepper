#!/bin/bash

# Make sure you have Ubuntu 20.04 installed 
# Be aware that the docker images kinetic and noetic are for x86 !not arm architectures
# if you want to run this on apple silicon or other arm based devices you have some setup todo yourself
# Make sure you have at least 40GBs of available space on your ubuntu system for: docker, images, whisper model etc.

# download and install docker according to https://docs.docker.com/engine/install/ubuntu/

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the Docker packages.
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verify that the Docker Engine installation is successful by running the hello-world image.
sudo docker run hello-world

# Pull ros noetic and ros kinetic docker images 
docker pull ros:noetic 
docker pull ros:kinetic

# Enter API-key for ChatGPT queries 
