# ME5524 Bayesian Robotics - Fall 2018

Workspace for the MBZIRC 2020 Challenge 3. The objective of this package is to locate a fire using the Clearpath Jackal UGV and navigate towards it.

![alt text][jackal]

[jackal]: https://github.com/gavincangan/bayesian-robotics/master/jackal.jpg "The Jackal"

## Videos
1. [Detection]()
2. [Kalman Filter](https://www.youtube.com/watch?v=IeikM8otWBU)
3. [Sensor fusion](https://www.youtube.com/watch?v=pEAvC2AhT7s)
4. [SLAM]()
5. [Final]()

## Table of Contents
1. [Installation](#1-installation)
2. [Operation](#2-operation)
3. [Timeline](#3-timeline)
4. [TODO](#4-todo)
5. [Contributiors](#5-contributors)

## 1. Installation
### Install prerequisites
```bash
sudo apt-get update
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install protobuf-compiler python-pip python-toml python-jinja2 python-catkin-tools
sudo pip install --upgrade pip
sudo pip install -U catkin_tools

sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
```

### Install ROS packages
```bash
sudo apt-get install ros-indigo-hokuyo-node
sudo apt-get install ros-kinetic-cv-bridge
```

### Configure base station to connect with Jackal
Edit `/etc/hosts` and add the following line:

```
192.168.1.24    cpr-jackal
```

### Install this workspace
```bash
cd ~
git clone https://github.com/gavincangan/bayesian-robotics.git
cd ~/bayesian-robotics
```

### Configure Environment

Add this to your `~/.bashrc` file.

```bash
nano ~/.bashrc

## Add the following, save and quit
source $HOME/bayesian-robotics/br_f18_project_ws/devel/setup.bash
```

### Download Workspace Dependencies
```bash
cd ~/bayesian-robotics/br_f18_project_ws
wstool update -t src
```

### Build Workspace
```
cd ~/bayesian-robotics/br_f18_project_ws
catkin build
```


## 2. Operation

### Configure ROS MASTER
The following must be run on each terminal to communicate properly with the jackal. To avoid doing this every time, you can include these in you `~/.bashrc` file.
```
export ROS_MASTER_URI=http://cpr-jackal:11311
export ROS_IP=<your computer's IP address>
```

### Start nodes on jackal
```
#Kill odometry as that messes with frames
killall ekf_localization_node

nohup roslaunch mbz_c3_jackal accessories.launch &
roslaunch mbz_c3_jackal sensing.launch
```

### Start nodes on base station
```
rosrun mbz_c3_jackal visualize_trackers.py
roslaunch mbz_c3_jackal mapping.launch
rosrun mbz_c3_jackal information_loss_map.py
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
rviz
```

### Getting the Jackal to work with keyboard teleop
After installing the `teleop_twist_keyboard` package, fire up the node and control Jackal using the keyboard

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


## 3. Timeline

### 2018/09/26
- Team formed

### 2018/09/29
- Project kickstart meeting
- Got it moving with keyboard teleop, attempted LIDAR and webcam, installed hector

### 2018/10/03
- Meeting planned
- Get data from webcam & LIDAR - Done
- LIDAR & camera mounts designed. Printing in progress
- Gavin learned how to use FreeCAD :)

### 2018/10/03
- Meeting planned
- Get data from webcam & LIDAR - Done
- LIDAR & camera mounts designed. Printing in progress
- Gavin learned how to use FreeCAD :)

### 2018/10/10
* Run hector
* Mount LIDAR and webcam

### 2018/10/15
- Implemented ball detection
- Restructured repository

### 2018/10/24

- Implemented Kalman filter
- https://www.youtube.com/watch?v=IeikM8otWBU



## TODO
* Static IP for wifi
* Build map in realtime (offboard mapping/play with settings in hector?)
* Move using waypoint
* Document upstart and add webcam to startup scripts

## 5. Contributors
Team 3
- [Abdullah Abdul-Dayem](https://github.com/Abdullah-Abduldayem)
- [Barnabas Gavin Cangan](https://github.com/gavincangan)
- Lisheng Yang
