FROM ros:humble-ros-core

RUN apt-get update
RUN apt-get install python3-opencv -y
RUN apt-get install python3-flask -y
RUN apt-get install python3-yaml -y
RUN apt-get install python3-smbus -y
RUN apt-get install python3-pygame -y
RUN apt-get install python3-pip -y

RUN pip install mediapipe
RUN pip install Adafruit_GPIO