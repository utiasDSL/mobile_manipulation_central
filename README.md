# Mobile Manipulator Shared Code

This repository contains shared code for running experiments with the "Thing"
mobile manipulator.

### Hardware

The robot consists of a UR10 manipulator mounted on a Ridgeback omnidirectional
mobile base. The base has a Hokuyo UST-10LX laser range finder mounted at the
front that provides a two-dimensional scan in a 270 degree arc in front of the
robot. The end effector has a Robotiq FT 300 force torque sensor mounted at the
wrist to measure the applied wrench, as well as Robotiq 3 finger gripper for
manipulation.

### Software

The Ridgeback is currently running Ubuntu 20.04 on its onboard computer. The
UR10 is running firmware version 3.15.

## Installation and setup

Clone this repository into the catkin workspace:
```
cd catkin_ws/src
git clone https://github.com/utiasDSL/dsl__projects__mobile_manipulation_central mobile_manipulation_central
```

Install dependencies into the catkin workspace:
* [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) - for the UR10 arm
* [robotiq](https://github.com/ros-industrial/robotiq) - for the Robotiq 3F gripper
* [vicon_bridge](https://github.com/ethz-asl/vicon_bridge) - required to track
  the position of the mobile base. May also be useful to track other objects,
  calibrate the EE pose, etc.

Build the workspace:
```
catkin build
```

The ROS master node runs onboard the Ridgeback computer and is started
automatically when the Ridgeback is turned on. You need to tell your laptop
where to reach the ROS master. First, add to `/etc/hosts`:
```
192.168.131.1 cpr-tor11-01
```
Then, in each terminal where you want to connect to the robot over ROS, run
```
export ROS_IP=192.168.131.100
export ROS_MASTER_URI=http://cpr-tor11-01:11311
```
To revert back to default settings (so you can run ROS locally, for example),
do:
```
export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP
unset ROS_HOSTNAME
```
It is convenient to put the above functions in a script that can be easily
sourced.

## Usage

Some scripts expect the environment variable
`MOBILE_MANIPULATION_CENTRAL_BAG_DIR` to point to the directory where bags are
stored (to create or read bag files). Export this variable in your `.bashrc`.

Start robot drivers:
```
roslaunch mobile_manipulator thing.launch
```

Interaction with the robot is done using
[ros_control](http://wiki.ros.org/ros_control). In particular:
* `scaled_vel_joint_traj_controller` is used to track trajectories of waypoints (interpolation between waypoints is done automatically);
* `joint_group_vel_controller` is used to forward velocity commands directly to the robot (this is the most low-level controller);
* `joint_state_controller` publishes robot state feedback.

Start or stop a particular controller using:
```
rosrun controller_manager controller_manager start <controller_name>
rosrun controller_manager controller_manager stop <controller_name>
```

## Issues/todo
* There is a conflict between the controller manager automatically started on
  board the Ridgeback and by the ur_robot_driver. The result is that the
  Ridgeback cannot be commanded once the ur_robot_driver has been launched.
* Scripts should automatically start the controllers they need if they are not
  already running
* Scripts should automatically read the the robot's current position in order
  to generate a trajectory from that point
* Currently `thing.launch` only starts the UR10; eventually it should start
  whatever is needed for the Ridgeback on the laptop side, as well as the
  Robotiq force-torque sensor.
* Add Vicon stuff.
