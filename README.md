# Devastator PI

An autonavigation robot using ROS, RPI and an arduino powered robot.

## Minimum System Requirements
* CPU: Intel Skylake i5/Ryzen 5 or higher 
* CPU SPEED: 2.5 Ghz
* OS: Ubuntu 16.04/ Ubuntu 16.04 WSL (Windows 10) with ROS Kinetic
* RAM: 6 GB
* Raspberry PI - 3 with ROS Kinetic

## Installation & Usage
Prior to proceeding to the following steps, `devastator_arduino` code must already be loaded to the arduino control board. See instructions to [setup the arduino control board.](https://github.com/DOST-ASTI/devastator-pi/tree/dev/devastator_arduino)

### A. Configure the Devastator-PI Package
1. Clone the repository into your ROS workspace.
```
$ cd ~/<path_to_workspace>
$ git clone git@github.com:DOST-ASTI/devastator-pi.git
``` 
2. Source the workspace `setup.bash`
```
$ cd ~/<path_to_workspace>
$ catkin_make
$ source devel/setup.bash
```
Note: Instead of entering the command for every shell launched, you can optionally append it to the `~/.bashrc`.
```
$ echo "source ~/<path_to_workspace>/devel/setup.bash" >> ~/.bashrc
```

3. Source the ROS `setup.bash`. Normally this should be added to your `~/.bashrc` file.
```
$ source /opt/ros/kinetic/setup.bash
```
### B. Setup the Network
Set the Master and Slave IPs
```
$ export ROS_IP=<IP.ADDRESS.OF.THE.DEVICE>
$ export ROS_MASTER_IP=http://<IP.ADDRESS.OF.THE.HOST>:11311
```
Do steps A.2 - B for the host PC and the raspberry-pi for each shell/terminal launched prior to entering any ros commands. See [ROS Network Setup](http://wiki.ros.org/ROS/NetworkSetup) for more details.

### C. Run the Devastator PI nodes
The following commands assumes you have done steps A.2 - B beforehand.

1. In a terminal start the `roscore`.
```
$ roscore
```
2. Open a second terminal to run the `devastator-pi` nodes. This will be your control terminal.
```
$ roslaunch devastator devastator.launch
```
3. Open a separate terminal and `ssh` into the raspberry pi
```
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
```
The serial cable from the arduino control board must be connected to one of the Raspberry-PI usb ports. In this case it is connected as /ttyACM0.

4. Activate and follow the onscreen instructions on the control terminal.

Sample Control terminal
```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
       i    
   j   k   l
       ,    


f : Enable auto navigation mode
r : Reset to manual navigation mode

CTRL-C to quit
```

Note: You may have to change the permission for the `devastator_key_teleop.py` file in order to use it.
```
$ cd ~/path_to_workspace/src/devastator-pi/src/
$ sudo chmod +x devastator_key_teleop.py
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.


## License
[3-Clause BSD](https://opensource.org/licenses/BSD-3-Clause)
