# WALL-E Replica

Here is the [link to my project webpage](http://thedraill.e-monsite.com/pages/projects/wall-e-replica.html). You will find information about the nomenclature, 3D printing and electronics.

## Installation of the environment

### Installation of the OS

First, you have to install an OS on the Raspberry. To operate the ```mediapipe``` library (which we will see later), it is essential to have a 64-bit OS. In addition, to prevent the Raspberry from using its resources unnecessarily, you must install an OS without a desktop. That's why you need to install **Raspberry Pi OS Lite (64-bit)**. You can enable SSH if you want to work on the Raspberry remotely. To do this, just use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and install the OS on a micro SD card.

The first thing to do after installing the OS is to update it:

```
sudo apt-get update
sudo apt-get upgrade
```

### Installation of the libraries

First, check that the correct version of Python is installed:

```
python --version
```

I personally used version 2.9.2 because it was the one that was installed on my OS originally. Higher versions should work, but I'm **not sure if 2.10 and higher will work** because of the ```mediapipe``` library.

You must also activate the **I2C**, **SPI** and **Camera** interfaces in ```raspi-config```'s **Interfacing Options**:

```
sudo raspi-config
```

Now you need to install the following libraries:

- **flask**: used for the web server
- **pyyaml**: used for the web server
- **pygame**: used to produce the sounds
- **smbus**: used by the HAT library
- **smbus2**: used by the ADC library
- **opencv-python**: used for the auto mode and the camera overlay
- **opencv-contrib-python**: used for the auto mode and the camera overlay
- **mediapipe**: used for the auto mode and the camera overlay
- **Adafruit_GPIO.SPI**: used by the screen libraries

```
pip install flask
pip install pyyaml
pip install pygame
pip install smbus
pip install smbus2
pip install opencv-python
pip install opencv-contrib-python
pip install mediapipe
pip install Adafruit_GPIO
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