# Raspberry Pi Headless Installation

## Requirements

* Raspberry Pi 3 B/B+
* 8 Gb SD Card
* Workstation PC (OS: Ubuntu 16.04/ Ubuntu 16.04 WSL)
* Wifi Network


## Prerequisites
* Download Raspbian Stretch Lite (No GUI) from [http://www.raspberrypi.org/downloads/](http://www.raspberrypi.org/downloads/)
* Burn the OS Image to the SD Card.

## Installation
This is a headless installation of ROS Kinetic on the raspberry pi hence it must be connected to a network with internet access.

### Connecting to the Pi

1. Create a new file named `ssh` with no extensions on the SD card boot partition.
```
touch ssh
```
or if you are on Windows 10, create a new text file and delete the `.txt` extension.

2. Create a new file called `wpa_supplicant.conf` on the SD card boot partiion. With the following contents:
```
update_config=1
ctrl_interface=/var/run/wpa_supplicant

network={
    scan_ssid=1
    ssid="YOUR_NETWORK_NAME"
    psk="YOUR_PASSWORD"
    key_mgmt=WPA-PSK
}
``` 
Reboot the Pi and it should automatically connect to the network.

3. Find the raspberry Pi's IP address.

    This can be done using any of the following:
    
    a. Go to your routers IP address and look up the IP address of the raspberry pi.
    
    b. Using a LAN Ethernet Cable.
    
    c. Using an external application. [Fing](https://www.fing.com/)

If you have access to your router the first option would be the easiest, however using the third option would also work especially for a shared network with numerous devices connected.

4. SSH into your raspberry Pi.
```
$ ssh pi@IP.ADDRESS

```
The default password is `raspberry`