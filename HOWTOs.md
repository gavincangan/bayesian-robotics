## Getting the Jackal to work with keyboard teleop
- Used this package: https://wiki.ros.org/teleop_twist_keyboard
```
# On Ubuntu, ROS Kinetic
sudo apt-get install ros-kinetic-teleop-twist-keyboard
```
- Once we had that set up, we could fire up the node and control Jackal using the keyboard  

```
source <path-to-catkin/devel/setup.bash>
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```