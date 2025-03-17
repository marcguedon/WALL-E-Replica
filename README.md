# WALL-E Replica

Here is the [link to my project webpage](http://thedraill.e-monsite.com/pages/projects/wall-e-replica.html). You will find information about the nomenclature, 3D printing and electronics.

## Installation of the environment

### Installation of the OS

First, you have to install an OS on the Raspberry. For the proper functioning of the AI ​​used (which we will see later), it is essential to have a 64-bit OS. In addition, to prevent the Raspberry from using its resources unnecessarily, you must install an OS without a desktop. That's why you need to install **Ubuntu server 22.04.X LTS (64bits)**. To do this, just use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and install the OS on a micro SD card. You can enable SSH if you want to work on the Raspberry remotely, by adding a `ssh` file inside the boot directory of the SD card.

The first thing to do after installing the OS is to update it.
```console
sudo apt-get update
```

### Installation of the libraries

You must also activate the **I2C**, **SPI** and **Camera** interfaces in ```raspi-config```'s **Interfacing Options**:

```
sudo raspi-config
```

## Setting up the web server

We can now download the ```Code``` and ```Tests``` folders and put them in a folder on the Raspberry. Here is an example:

```
sudo cp -r Code /home/[your_username]/WALL-E/
sudo cp -r Tests /home/[your_username]/WALL-E/
```

The files present in the ```Tests``` folder allow you to test the robot modules one by one, independently of each other.

## Setting up a service

After downloading the ```wall-e.service``` file, you must modify it to adapt it to your environment. Here is an example:

```
[Service]
ExecStart=/usr/bin/python /home/[your_username]/WALL-E/Code/webServer/webServer.py
WorkingDirectory=/home/[your_username]/WALL-E/Code/webServer
Restart=always
User=[your_username]
```

Copy it to the ```/etc/systemd/system/``` folder:

```
sudo cp wall-e.service /etc/systemd/system/
```

Now you need to activate and start the service:

```
sudo systemctl daemon-reload
sudo systemctl enable wall-e.service
sudo systemctl start wall-e.service
```

In order to make sure that everything is working correctly, it is possible to look at the logs of the code that has been executed:

```
sudo journalctl -u wall-e.service
```

The web server will now start automatically as soon as the Raspberry is turned on.

## Installation and configuration of RaspAP

[RaspAP](https://raspap.com/) will be used to generate its own wifi network on the Raspberry.

### Installation

You must first update the raspberry if this is not the case. You must also set the wiFi country in ```raspi-config```'s **Localisation Options**:

```
sudo raspi-config
```

We can now install RaspAP. It is advisable to accept the recommended options, but you can refuse the non-recommended ones:

```
curl -sL https://install.raspap.com | bash
```

Before restarting the Raspberry, you must open port 22 (SSH) with ```iptables``` in order to be able to use it after restarting the Raspberry:

```
sudo iptables -A INPUT -p tcp --dport 22 -j ACCEPT
sudo iptables-save | sudo tee /etc/iptables/rules.v4
sudo service iptables save
sudo systemctl restart iptables
```

It is possible to check iptables rules before and after opening port 22:

```
sudo iptables -L
```

After installing RaspAP and opening port 22, restart the Raspberry and configure RaspAP. To do this, you must connect to the wifi network that the Raspberry has created and connect to the RaspAP web interface:

- **SSID**: raspi-webgui
- **Password**: ChangeMe
- **IP address**: 10.3.141.1
- **Username**: admin
- **Password**: secret

You can change the wifi SSID and password as well as the credentials to access the RaspAP web interface. However, you will now need to **SSH** in with ```10.3.141.1``` and access the **web interface** with ```10.3.141.1:5000```.






https://docs.docker.com/engine/install/debian/#install-using-the-repository

**Installer rosbridge_server**
sudo apt install -y ros-humble-rosbridge-server ros-humble-vision-opencv ros-humble-rosidl-generator-py python3-rosdep2

**Build le package**
colcon build --packages-select wall_e_msg_srv
colcon build --packages-select wall_e_core
source install/setup.bash

echo "install/setup.bash" >> ~/.bashrc
source ~/.bashrc

ros2 launch wall_e_core wall_e_launch.py


docker run -it --privileged --name WALL_E -v /tmp:/tmp -v $PWD:/wall_e --device /dev/gpiomem -v /sys:/sys --device /dev/video0 --device /dev/mem --device /dev/gpiomem --device /dev/i2c-1 --device /dev/spidev0.0 --device /dev/vchiq -p 80:80 -p 5000:5000 -p 9090:9090 wallereplica:latest bash

docker run -it --privileged --name WALL_E -v /tmp:/tmp -v $PWD:/wall_e --device /dev/gpiomem -v /sys:/sys --device /dev/video0 --device /dev/mem --device /dev/gpiomem --device /dev/i2c-1 --device /dev/spidev0.0 --device /dev/vchiq --env LIBGL_ALWAYS_SOFTWARE=1 -p 80:80 -p 5000:5000 -p 9090:9090 wallereplica:latest bash

sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0



Sur le Raspberry, lancer la commande suivante:
```console
libcamera-vid -t 0 -o /tmp/video_stream.h264
libcamera-still -t 0 -o /tmp/video_stream.jpg
```

Dans le docker, installer la lib suivante:
```console
apt-get install -y ffmpeg
python3 -m pip install ffmpeg-python
```

```py
import cv2
import ffmpeg
import numpy as np
from flask import Flask, render_template, Response

app = Flask(__name__)


def generate_frames():
    process = (
        ffmpeg.input("/tmp/video_stream.jpg")
        .output("pipe:1", format="rawvideo", pix_fmt="bgr24")
        .run_async(pipe_stdout=True, pipe_stderr=True)
    )

    while True:
        in_bytes = process.stdout.read(640 * 480 * 3)

        if len(in_bytes) < 640 * 480 * 3:
            break

        frame = np.frombuffer(in_bytes, np.uint8).reshape([480, 640, 3])
        ret, buffer = cv2.imencode(".jpg", frame)

        # in_bytes shape: 921600, frame shape: (480, 640, 3), buffer shape: (47269,) but buffer shape always change
        print(
            f"in_bytes shape: {len(in_bytes)}, frame shape: {frame.shape}, buffer shape: {buffer.shape}"
        )

        if not ret:
            print("Failed to encode frame")
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
        )


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
```


docker run -it --privileged --name WALL_E -v /tmp:/tmp --device /dev/gpiomem -v /sys:/sys --device /dev/video0 --device /dev/mem --device /dev/gpiomem --device /dev/i2c-1 --device /dev/spidev0.0 --device /dev/vchiq --env LIBGL_ALWAYS_SOFTWARE=1 -p 80:80 -p 5000:5000 -p 9090:9090 wallereplica:latest bash



docker run -it --privileged --name WALL_E -v /usr/bin/libcamera-still:/usr/bin/libcamera-still -v /run/libcamera:/run/libcamera -v /usr/bin/libcamera-vid:/usr/bin/libcamera-vid -v /lib:/lib -v /usr/lib:/usr/lib -v /tmp:/tmp -v /dev:/dev -v /run/udev:/run/udev -v /sys:/sys --env LIBGL_ALWAYS_SOFTWARE=1 -p 80:80 -p 5000:5000 -p 9090:9090 wallereplica:latest bash

docker run -it --privileged --name WALL_E -v /usr:/usr -v /tmp:/tmp -v /dev:/dev -v /run/udev:/run/udev -v /sys:/sys --env LIBGL_ALWAYS_SOFTWARE=1 -p 80:80 -p 5000:5000 -p 9090:9090 wallereplica:latest bash





# Sur le raspberry

Installation de Ubuntu server 22.04.X LTS (64bits)
Installation de [ROS2 Humle](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) en choisissant `ros-humble-ros-base`

```console
sudo apt-get install -y libraspberrypi-bin v4l-utils ros-humble-v4l2-camera raspi-config

sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_camera 0

echo "start_x=1
gpu_mem=128
camera_auto_detect=0
dtoverlay=imx708
dtoverlay=vc4-kms-v3d" >> /boot/firmware/config.txt

sudo apt-get install -y ros-humble-rosbridge-server ros-humble-vision-opencv
sudo apt-get install -y python3-colcon-common-extensions python3-numpy python3-opencv python3-pygame python3-pip python3-rpi.gpio
python3 -m pip install mediapipe adafruit-circuitpython-display-text adafruit-circuitpython-display-shapes adafruit-circuitpython-pca9685 adafruit-circuitpython-ads1x15 adafruit-circuitpython-st7789

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

colcon build --packages-select wall_e_msg_srv
colcon build --packages-select wall_e_core

echo "source /home/$USER/WALL-E-Replica/install/setup.bash" >> ~/.bashrc

ros2 launch wall_e_core wall_e_launch.py
```

amixer cset numid=3 1
sudo apt install pulseaudio pulseaudio-utils