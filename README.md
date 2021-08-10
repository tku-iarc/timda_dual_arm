# TKU TIMDA Dual-Arm

## Requirements

### ROS Packages:
```bash
$ sudo apt-get install ros-$ROS_DISTRO-pcl-ros
$ sudo apt-get install ros-$ROS_DISTRO-camera-info-manager
$ sudo apt-get install ros-$ROS_DISTRO-position-controllers
$ sudo apt-get install ros-$ROS_DISTRO-velocity-controllers
$ sudo apt-get install ros-$ROS_DISTRO-effort-controllers
$ sudo apt-get install ros-$ROS_DISTRO-joint-state-controller
$ sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher
$ sudo apt-get install ros-$ROS_DISTRO-aruco-detect
$ sudo apt-get install ros-$ROS_DISTRO-visp-hand2eye-calibration
$ sudo apt-get install ros-$ROS_DISTRO-soem
$ sudo apt-get install ros-$ROS_DISTRO-socketcan-interface
$ sudo apt-get install ros-$ROS_DISTRO-flexbe-behavior-engine
```
### Install modbus library `libmodbus`:
**See [linear_motion/README.md](linear_motion/README.md)**

### Python packages:
```bash
$ cd src/
$ cd pip install -r requirements.txt
```
