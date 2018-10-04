# mbz_c3_jackal
Package for the MBZIRC 2020 Challenge 3. The objective of this package is to locate a fire using the Clearpath Jackal UGV and navigate towards it.

## Dependencies
```
sudo apt-get install ros-indigo-usb-cam
sudo apt-get install ros-indigo-image-view
```


## Configure ROS Master
Edit `/etc/hosts` and add the following line:

```
192.168.1.11    cpr-jackal
```

Afterwards, run the following in the terminal:
```
export ROS_MASTER_URI=http://cpr-jackal:11311
export ROS_IP=<your computer's IP address>
```

## Run
```
## Start laser
rosrun hokuyo_node hokuyo_node


```
