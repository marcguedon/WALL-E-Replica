# WALL-E Replica

Here is the [link to my project webpage](http://thedraill.e-monsite.com/pages/projects/wall-e-replica.html). You will find information about the nomenclature, 3D printing and electronics.

## Installation of the environment

### Installation of the OS

First, you have to install an OS on the Raspberry. For the proper functioning of AI, it is essential to have a 64-bit OS. In addition, to prevent the Raspberry from using its resources unnecessarily, you must install an OS without a desktop. That's why you need to install **Ubuntu server 22.04.X LTS (64bits)**. To do this, just use [Raspberry Pi Imager](https://www.raspberrypi.com/software/) and install the OS on a micro SD card. You must enable SSH to be able to work on the Raspberry remotely, by adding a `ssh` file inside the boot directory of the SD card.

After putting the SD card in the slot of the Raspberry, you can turn it on.

### Retrieval and installation of the project

> **Important**:
> Your Raspberry needs internet access.

Before continuing, you have to install ROS (ros-humble-ros-base) by following this [link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

Now, you will need to retrieve the project from the github repository. On the Raspberry, run the following command lines.

```console
cd ~
git clone https://github.com/marcguedon/WALL-E-Replica.git
```

To install all the dependencies and prepare the project, you just need to execute the `install.sh` script.

```console
cd WALL-E-Replica
./install.sh
```

Now that everything is installed, you must reboot the raspberry.

```console
sudo reboot
```

## Installation and configuration of RaspAP

[RaspAP](https://raspap.com/) will be used to generate its own wifi network on the Raspberry.

### Installation

You must first update the raspberry if this is not the case. You must also set the WLAN country.

```console
sudo raspi-config noint do_wifi_country <your_country>
```

We can now install RaspAP. It is advisable to accept the recommended options, but you can refuse the non-recommended ones.

```console
curl -sL https://install.raspap.com | bash
```

Before restarting the Raspberry, you must open port 22 (SSH) with `iptables` in order to be able to use it after restarting the Raspberry.

```console
sudo iptables -A INPUT -p tcp --dport 22 -j ACCEPT
sudo iptables-save | sudo tee /etc/iptables/rules.v4
sudo service iptables save
sudo systemctl restart iptables
```

It is possible to check iptables rules before and after opening port 22.

```console
sudo iptables -L
```

### Configuration

After installing RaspAP and opening port 22, restart the Raspberry and configure RaspAP. To do this, you must connect to the wifi network that the Raspberry has created and connect to the RaspAP web interface:

- **SSID**: raspi-webgui
- **Password**: ChangeMe
- **IP address**: 10.3.141.1
- **Username**: admin
- **Password**: secret

You can change the wifi SSID and password as well as the credentials to access the RaspAP web interface. However, you will now need to **SSH** in with `10.3.141.1` and access the **web interface** with `10.3.141.1:5000`.
