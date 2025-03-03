FROM osrf/ros:humble-ros-base

RUN apt-get update
RUN apt-get install -y python3-opencv python3-smbus python3-pygame python3-pip
RUN python3 -m pip install mediapipe Adafruit_GPIO

RUN wget https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js -O roslib.js