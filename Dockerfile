FROM ros:humble-ros-base

RUN apt-get update
RUN apt-get install -y ffmpeg
RUN apt-get install -y ros-humble-rosbridge-server ros-humble-vision-opencv
RUN apt-get install -y python3-numpy python3-opencv python3-pygame python3-pip
RUN python3 -m pip install mediapipe ffmpeg-python adafruit-circuitpython-pca9685 adafruit-circuitpython-ads1x15 adafruit-circuitpython-st7789

# RUN wget https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js -O roslib.js

RUN colcon build --packages-select wall_e_msg_srv
RUN colcon build --packages-select wall_e_core

RUN echo "install/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

RUN ros2 launch wall_e_core wall_e_launch.py