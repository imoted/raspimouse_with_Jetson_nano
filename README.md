[![Build Status](https://travis-ci.org/ryuichiueda/raspimouse_ros_2.svg?branch=master)](https://travis-ci.org/ryuichiueda/raspimouse_ros_2)

# raspimouse_ros_2

The current version of the ROS base package for Raspberry Pi Mouse. This package is derived from "pimouse_ros" package, which is coded for the book from Nikkei BP.
* old versions: [ryuichiueda/raspimouse_ros](https://github.com/ryuichiueda/raspimouse_ros)

## Requirements

This package requires the following to run:

* Ubuntu
  * Ubuntu 16.04
  * Ubuntu MATE 16.04
* ROS 
  * ROS Kinetic Kame
* Device Driver
  * [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)

## Installation

### 1. Install the latest stable version of ROS.  

Please refer to [ROS WiKi](http://wiki.ros.org/kinetic/Installation) for installation, or run the following ros setup scripts.
* [ryuichiueda/ros_setup_scripts_Ubuntu16.04_server](https://github.com/ryuichiueda/ros_setup_scripts_Ubuntu16.04_server)

### 2. Download and install device driver.  

Please refer to [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse) for download and installation.

### 3. Make a workspace.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_worksapce
cd ~/catkin_ws
catkin_make
```

Delete the following line from `~/.bashrc`:

```
source /opt/ros/kinetic/setup.bash
```

Add the follwing line to `~/.bashrc`:

```
source ~/catkin_ws/devel/setup.bash
```

### 4. Download this repository into `~/catkin_ws/src`.

```
cd ~/catkin_ws/src
git clone https://github.com/ryuichiueda/raspimouse_ros_2.git
```

### 5. Resolve the system dependencies.

Beforehand, please check the value of `$ROS_PACKAGE_PATH`. (The path in which raspimouse_ros_2 exists should be included.)

```
rosdep install raspimouse_ros_2
```

### 6. Build this repository.

```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```


### 7. Test with the buzzer node

```
roslaunch raspimouse_ros_2 raspimouse.launch
rostopic pub /buzzer std_msgs/UInt16 1000
```
## Support of MPU6050

At first you need to setup I2C permissions for non-root users.

1) Create new user group called i2c:

```
user@machine:~ sudo groupadd i2c
```
2) Change the group ownership of /dev/i2c-1 to i2c:
```
user@machine:~ sudo chown :i2c /dev/i2c-1
```
3) Change the file permissions of the device /dev/i2c-1 so users of the i2c group can read and write to the device:
```
user@machine:~ sudo chmod g+rw /dev/i2c-1
```
4) Add your user to the group i2c:
```
user@machine:~ sudo usermod -aG i2c user
```
5) After you logout and login again you should be able to run i2cdetect -y 1.

The listing blow shows an example output of the command. As one can see, I connected three I2C devices to my Raspberry Pi 2.
```
user@machine:~ i2cdetect -y 1
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- 04 -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- 39 -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- 77
```

## Support of RT-USB-9axisIMU2

If you have an RT-USB-9axisIMU2 9-axis sensor, its z angular velocity can be reflected to the `/odom` topic. 


### requirement

* hardware: RT-USB-9axisIMU2 (text mode)
* software: https://github.com/AtsushiSaito/rt_usb_9axis_sensor

### configuration

Please set `1` to the argument `imu`. When you want to use this feature permanently, please rewrite the arg element of raspimouse.launch as follows. 

```raspimouse.launch 
<launch>
  <arg name="imu" default="1" />   <!-- change from 0 to 1 -->
    <include if="$(arg imu)" file="$(find rt_usb_9axis_sensor)/launch/rt_usb_9axis_sensor.launch" />
...
```
