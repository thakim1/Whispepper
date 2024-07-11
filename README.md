# Whispepper - Speech Recognition for the Pepper Robot using ROS
*Whispepper* is a project enabling voice interaction on the Aldebaran 'Pepper' robot using OpenAI's whisper model and Meta's LlaMA3 model with [Ollama](https://ollama.com/). This project brings whisper-speech recognition and Ollama to the Aldebaran Pepper robot. 

- [Whispepper](#whispepper)
  * [Results](#results)
  * [Technologies](#technologies)
  * [Mockup](#mockup)
  * [Setup and Usage](#setup-and-configuration)
  * [Authors](#authors)
  * [License](#license)
  * [Acknowledgments](#acknowledgments)

## Results

Results for Word Error Rate (WER) evaluation: 

![graph](/eval/wer_graph.svg)

Average response time: 5.4s - Pepper is quite quick to response, so the conversations are somewhat fluent. There are, however, some improvements I haven't followed up on yet:
- Being able to interrupt Pepper mid-sentence
- Improved VAD
- Improved response time
- Unresolved Bug: Upon startup 3 speech chunks are recognized
  

## Technologies
* Python2.7 for ROS Kinetic and 3.8 for ROS Noetic (they ship with these python versions)
* Latest numpy and ROSpy for both python versions
* [NaoQi Pepper SDK for Python2.7](https://www.aldebaran.com/en/support/pepper-naoqi-2-9/downloads-softwares)
* [OpenAI's Whisper model](https://github.com/openai/whisper)
* [ROS Kinetic Docker Image](https://hub.docker.com/_/ros/tags?page=1&name=kinetic)
* [ROS Noetic Docker Image](https://hub.docker.com/_/ros/tags?page=1&name=noetic)
* [Ollama](https://ollama.com/)
* [Fix for 'possible' Pepper SDK library complaint](https://stackoverflow.com/questions/48306849/lib-x86-64-linux-gnu-libz-so-1-version-zlib-1-2-9-not-found/50097275#50097275)

If you want to use ChatGPT instad of LlaMA you need an OpenAI account with API-key -> [chat.openai.com](chat.openai.com)


## Mockup

We implement two docker containers which communicate using ROS publish/subscribe messaging and a REST API for LlaMA3.

![grafik](/eval/RV_setup_sketch.jpg)


## Setup and Usage

There is a bit of setup to do before you can run anything. You have to configure Pepper's IP, Ollama (Docker) REST API, and other parameters in the paramters.py files. 


### Pythonpath configuration for correct module resolving 
In order to use the ros and naoqi libraries the pythonpath environment variable has to be set accordingly. On the host, it should look like this depending on your system. Although the naoqi library is not needed in the python3.8 code since it only waits for the ros messages from the python2.7 module (which controls the Pepper robot) it is shown here and might be useful for
testing the SDK. 
```
echo $PYTHONPATH
/opt/ros/noetic/lib/python3/dist-packages:/home/hakim/Downloads/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages:/opt/ros/noetic/share/audio_common/src
```
Inside the docker container it should look similar to this also depending on your docker setup.
```
echo $PYTHONPATH
/opt/ros/kinetic/lib/python2.7/dist-packages:/home/naoqi/lib/python2.7/site-packages/:/opt/ros/kinetic/share/audio_common/src
```
 
If you use the Noetic Docker container instead of running the main_noetic.py python3.8 file on the host system natively, no additional pythonpath setup is needed. 

### Usage

Super user priviliges may be necessary when starting main_noetic.py on the host instead of using the Noetic Docker container. 

0. Configure Pepper's IP, Ollama (Docker) REST API etc. in the parameters.py files. And make sure you have the NaoQi SDK folder inside the ros-kinetic directory. 

1. If you have neither docker installed nor ROS kinetic and noetic images downloaded execute setup.sh
```sh
./setup.sh
```
2. Navigate to the ros-kinetic directory
```sh
cd ros-kinetic
```

3. Run the Kinetic setup script
```sh
./kinetic_setup.sh
```
Roscore should now be running in that terminal (don't close it)

4. Open a new terminal and execute the following command in the same path to access the Kinetic Docker container
```sh
docker exec -it rv2024_kinetic_pepper_image bash
```
Inside the kinetic docker container, run
```sh
python2 /home/main_p27.py
```
5. Open another terminal, navigate to the ros-noetic directory
```sh
cd ros-noetic
```
6. Run the Noetic setup script
```sh
./noetic_setup.sh
```
Inside the noetic docker container run. 
```sh
python3 /home/main_p38.py
```
You will be prompte to press the Enter-Key. After pressing it you will be able to converse with Pepper. If you exit the Kinetic container (with Ctrl+C or by just typing 'exit' etc.), to avoid building again just use:
```sh
docker run -it --rm --name rv2024_kinetic_pepper_container \
    -v rv2024_pepper_volume:/home/volume_save/ \
    --ipc=host \
    --network=host \
    rv2024_kinetic_pepper_image
```
instead of kinetic_setup.sh. If you exit the Kinetic container after building, to avoid building again just use instead of noetic_setup.sh:
```sh
docker run -it --rm --name rv2024_noetic_pepper_container \
    --network=host \
    --ipc=host \
    -v .:/home \
    rv2024_noetic_pepper_image
```

## Authors
* **Hakim Tayari** @ Vienna University of Technology - *Initial work*

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Special Thanks to [Matthias](https://github.com/hirschmanner) for his guidance and support
* [PepperChat](https://github.com/ilabsweden/pepperchat)
* [Pepper Speech Recognition](https://github.com/JBramauer/pepperspeechrecognition)
