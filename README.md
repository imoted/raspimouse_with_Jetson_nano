# raspimouse_with_Jetson_nano

* The package is for raspimouse [http://products.rt-net.jp/micromouse/raspberry-pi-mouse] with Jetson Nano hardware.
* This package is derived from "pimouse_ros_2" package[https://github.com/ryuichiueda/raspimouse_ros_2]
* Currecntly only work with the motor. without buzzar, light sensor, switch, LED

## Requirements

This package requires the following to run:

* Ubuntu
  * Ubuntu 18.04 
    * follow the instruction 

    https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit

* ROS 
  * ROS Melodic
* Electrical wirering
  * raspimouse P2 pin31 - Jetson nano J41 pin35
  * raspimouse P2 pin33 - Jetson nano J41 pin33 
  * raspimouse P2 pin32 - Jetson nano J41 pin32
  * raspimouse P2 pin36 - Jetson nano J41 pin31
  * raspimouse P2 pin29 - raspimouse 3V

The Jetson Nano carrier board uses TXB0108 bidirectional voltage-level translators on the GPIOs to go from the module’s 1.8 V levels to the headers 3.3 V levels. The data sheet for the TXB0108 states “With regard to capacitive loads, TXB translators are designed to drive up to 70 pF without issue.” 

http://www.tij.co.jp/jp/lit/ds/symlink/txb0108.pdf
http://akizukidenshi.com/download/ds/ir/irlml6402.pdf

FET irlml6402 has 633pF input capasitance on Raspimouse.
So Jetson nano GPIO can't drive directly raspimouse Enable pin(FET irlml6402).

https://github.com/rt-net/RaspberryPiMouse_Hardware/blob/master/supplement/RaspberryPiMouseV2_circuit.pdf

https://www.jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/

* Connect power cable 5V to Jetson nano
  * 5V connectin from raspi mouse P2 to Jetson nano J25
  * don't forget to connect jumper J48
* Mechanical setup
  * 3D print jetson3.stl file and setup the Jetson nano

* Do not need raspberry pi mouse device driver

## Installation

### 1. Setup the Jetson nano 

* PWM pin assign on Jetson nano
  * Connect PWM pwm0 function to pin32 and pwm2 function to pin33  by following the instruction 

https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%2520Linux%2520Driver%2520Package%2520Development%2520Guide%2Fhw_setup_jetson_io.html%23wwpID0E0JE0HA

### 2. Install the ROS1 Melodic

Please refer to [ROS WiKi](http://wiki.ros.org/melodic/Installation) for installation, or run the following ros setup scripts.

### 3. clone and build

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_worksapce
```

Add the follwing line to `~/.bashrc`:

```
source ~/catkin_ws/devel/setup.bash
```

```
cd ~/catkin_ws/src
git clone https://github.com/imoted/raspimouse_with_Jetson_nano.git
rosdep install raspimouse_with_Jetson_nano
catkin build
```

### remaining issue

* implement LED, light sensor, buzzar driver
* software and circuit fix to drive the motot enable
* implement the RT-USB-9axisIMU2
