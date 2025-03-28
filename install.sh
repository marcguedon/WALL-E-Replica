#!/bin/bash

sudo apt-get update

# Installing the dependencies
sudo apt-get install -y libraspberrypi-bin v4l-utils raspi-config
sudo apt-get install -y ros-humble-rosbridge-server ros-humble-vision-opencv ros-humble-v4l2-camera
sudo apt-get install -y python3-pip python3-colcon-common-extensions

python3 -m pip install -r requirements.txt

# Downloading roslib.js file
curl -o /home/$USER/WALL-E-Replica/web_server/js/roslib.js https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js

# Configuring the Raspberry
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_legacy 0

# Building the packages
source /opt/ros/humble/setup.bash

colcon build --packages-select wall_e_msg_srv
colcon build --packages-select wall_e_core

# Starting the service
USER=$(whoami)
SERVICE_FILE="wall-e.service"

if [ -f "$SERVICE_FILE" ]; then # Check if the .service file exists
    sudo sed -i "s/%u/$USER/g" "$SERVICE_FILE" # Replace %u by the current user in the wall-e.service file
    echo "wall-e.service file updated with user $USER."
else
    echo "wall-e.service file not found."
    exit 1
fi

sudo cp wall-e.service /etc/systemd/system/

sudo systemctl daemon-reload
sudo systemctl enable wall-e.service
sudo systemctl start wall-e.service
