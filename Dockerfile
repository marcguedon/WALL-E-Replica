FROM ros:humble-ros-base

RUN apt-get update
RUN apt-get install -y ffmpeg
RUN apt-get install -y ros-humble-rosbridge-server ros-humble-vision-opencv
RUN apt-get install -y python3-colcon-common-extensions python3-numpy python3-opencv python3-pygame python3-pip python3-rpi.gpio
RUN python3 -m pip install mediapipe ffmpeg-python adafruit-circuitpython-display-text adafruit-circuitpython-display-shapes adafruit-circuitpython-pca9685 adafruit-circuitpython-ads1x15 adafruit-circuitpython-st7789

COPY . /wall-e
WORKDIR /wall-e

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select wall_e_msg_srv && \
    colcon build --packages-select wall_e_core"

RUN echo 'source /wall-e/install/setup.bash' >> ~/.bashrc