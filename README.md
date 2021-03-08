# TKU TIMDA Dual-Arm

## Requirements

### ROS Packages:
```bash
$ sudo apt-get install ros-<distro>-pcl-ros
$ sudo apt-get install ros-<distro>-camera-info-manager
$ sudo apt-get install ros-<distro>-position-controllers
$ sudo apt-get install ros-<distro>-velocity-controllers
$ sudo apt-get install ros-<distro>-effort-controllers
$ sudo apt-get install ros-<distro>-joint-state-controller
$ sudo apt-get install ros-<distro>-joint-state-publisher
$ sudo apt-get install ros-<distro>-aruco-detect
$ sudo apt-get install ros-<distro>-visp-hand2eye-calibration
```
ros-melodic-position-controllers ros-melodic-velocity-controllers ros-melodic-effort-controllers ros-melodic-joint-state-controller ros-melodic-joint-state-publisher
### Install modbus library `libmodbus`:
**See [linear_motion/README.md](linear_motion/README.md)**

### Python packages:
```bash
$ cd src/
$ cd pip install -r requirements.txt
```
