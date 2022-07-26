# Mobile Manipulator Experiment Code

This repository contains shared code for running experiments with the "Thing"
mobile manipulator.

## Installation

Clone this repository into the catkin workspace:
```
git clone https://github.com/utiasDSL/mobile_manipulator catkin_ws/src
```

Clone dependencies into the catkin workspace:
* [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) - for the UR10 arm
* [robotiq](https://github.com/ros-industrial/robotiq) - for the Robotiq 3F gripper

Build the workspace:
```
catkin build
```

## Usage

Start robot drivers:
```
roslaunch mobile_manipulator thing.launch
```
