# ROSberry Pi- Installing ROS Kinetic on the Raspberry Pi 3

The following installs a lite version of ROS Kinetic (No GUI) taken from the [ROS Installation Instructions](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi).

## Requirements

* Raspberry Pi 3 B/B+
* 8 Gb SD Card
* Workstation PC (OS: Ubuntu 16.04/ Ubuntu 16.04 WSL)
* Wifi Network


## Prerequisites
* Follow the instrutions on `setup-pi` to connect to your raspberry pi (headless).

## Setup ROS repositories
This is a headless installation of ROS Kinetic on the raspberry pi hence it must be connected to a network with internet access.

1. Install `dirmngr`
```
$ sudo apt install dirmngr
```
2. Setup keys
```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```
3. Update packages
```
$ sudo apt update
$ sudo apt upgrade
```
4. Install dependencies
```
$ sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```
5. Initialize rosdep
```
$ sudo rosdep init
$ rosdep update
```
## Installation
1. Create a workspace
```
$ mkdir -p ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```
2. Fetch required ROS Packages. 
This installation requires three packages `ros_comm common_msgs and rosserial`. Other packages can be found at [https://github.com/ros-gbp](https://github.com/ros-gbp)
```
$ rosinstall_generator ros_comm common_msgs rosserial --rosdistro kinetic --deps --wet-only --tar > kinetic-ros_comm-wet.rosinstall
$ wstool init src kinetic-ros_comm-wet.rosinstall
```
3. Resolve dependencies
```
$ cd ~/ros_catkin_ws
$ rosdep install -y --from-paths src --ignore-src --rosdistro kinetic -r --os=debian:stretch
```
4. Build the workspace
```
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic -j2
```
If an error occurs, try [adding more swap space](https://wpitchoune.net/tricks/raspberry_pi3_increase_swap_size.html).

5. Source the new installation
```
$ source /opt/ros/kinetic/setup.bash
$ cd ~/ros_catkin_ws
$ source devel/setup.bash
```

ROS Kinetic should now be installed

## License
[MIT](https://choosealicense.com/licenses/mit/)