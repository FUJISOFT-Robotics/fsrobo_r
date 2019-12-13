fsrobo_r [![Build Status](https://travis-ci.com/FUJISOFT-Robotics/fsrobo_r.svg?branch=master)](https://travis-ci.com/FUJISOFT-Robotics/fsrobo_r)
===================================================================================================================================================

This repository provides ROS support for FSRobo.  
ROS distribution `Kinetic` and `Melodic` is supported.

## How to Launch
To start robot driver:
```
roslaunch fsrobo_r_bringup fsrobo_r_bringup.launch
```

If you have no real robot, use fake moveit controller:
```
roslaunch fsrobo_r_moveit_config demo.launch
```

## Supported Robot
* FSRobo-R