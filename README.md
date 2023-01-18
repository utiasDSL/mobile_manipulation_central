# Mobile Manipulator Shared Code

This repository contains shared code for running experiments with the "Thing"
mobile manipulator.

## System Information

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

## Documentation

Manuals and datasheets, as well as other documents, can be found in the `docs`
directory.

## Installation and Setup

Clone this repository into the catkin workspace:
```
cd catkin_ws/src
git clone https://github.com/utiasDSL/dsl__projects__mobile_manipulation_central mobile_manipulation_central
```

Install [Eigen](https://eigen.tuxfamily.org): `sudo apt install libeigen3-dev`

For kinematics, [Pinocchio](https://github.com/stack-of-tasks/pinocchio) is
required. I prefer to build this outside of the catkin workspace.  First,
install the dependencies
```
sudo apt install ros-noetic-eigenpy ros-noetic-hpp-fcl
```
Then follow the installation directions
[here](https://stack-of-tasks.github.io/pinocchio/download.html) (under the
"Build from Source" tab), using the cmake command:
```
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_WITH_COLLISION_SUPPORT=ON
```
Ensure that you also modify `$PYTHONPATH` to include the location of
Pinocchio's Python bindings.

Finally, install Python dependencies:
```
python3 -m pip install -r requirements.txt
```

### Real Hardware Setup

For working with real hardware, install the following packages into the catkin
workspace:
* [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) - for the UR10 arm.
* [robotiq](https://github.com/TAMS-Group/robotiq) - for the Robotiq 3F gripper. This is a fork of the original (now unmaintained) repo.
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

Connect to the robot via ethernet and set up a new Wired Connection named
`Thing`. In the `IPv4 Settings` tab, switch to method `Manual` and enter an
address of `192.168.131.100` with netmask `255.255.255.0`. Leave the gateway
blank. Once done, you should be able to ping the robot at `192.168.131.1`.

## Usage

### Simulation

This repo provides a basic simulation environment based on
[PyBullet](https://pybullet.org), a demo of which can be found in
`scripts/simulation/pyb_simulation.py`.

### Kinematics

Kinematics based on [Pinocchio](https://github.com/stack-of-tasks/pinocchio) is
provided in both C++ and Python. An example of the kinematics in C++ can be
found in `src/kinematics_example.cpp`


### Real Hardware

Some scripts expect the environment variable
`MOBILE_MANIPULATION_CENTRAL_BAG_DIR` to point to the directory where bags are
stored (to create or read bag files). Export this variable in your `.bashrc`.

Vicon is used to track the base position as well as any other objects in the
scene. You must be connected to the `DSL_DroneNet_5G` network.

Start the Vicon bridge, UR10 driver, and gripper driver:
```
roslaunch mobile_manipulation_central thing.launch
```
To stream commands to the UR10, you must start the onboard program.

Interaction with the robot is primarily done using the feedback topics (of type
`sensor_msgs/JointState`):
```
/ridgeback/joint_states
/ur10/joint_states
```
and the velocity command topics
```
/ridgeback/cmd_vel  # geometry_msgs/Twist
/ur10/cmd_vel       # std_msgs/Float64MultiArray
```
For Python nodes this is abstracted away using the interfaces in
`src/mobile_manipulation_central/ros_interface.py`.

#### Scripts

There are some convenient scripts in the `scripts` directory:

* `control/home.py` sends the robot to a particular home configuration, which is taken
  from `home.yaml`.
* `control/gripper.py` opens and closes the gripper.
* `control/sine_trajectory` tracks a sinusoidal trajectory with a single joint.

## Tests

Unit tests can be found in the `test` directory. Currently there are only
Python unit tests, which can be run using [pytest](https://pytest.org/).

## Robot specifications

### Ridgeback limits

With reference to [this
file](https://github.com/ridgeback/ridgeback/blob/melodic-devel/ridgeback_control/config/control.yaml),
the limits are:

| Joint | Velocity  | Acceleration |
| ----- | --------- | ------------ |
| x     | 1.1 m/s   | 2.5 m/s²     |
| y     | 1.1 m/s   | 2.5 m/s²     |
| yaw   | 2.0 rad/s | 1.0 rad/s²   |

### UR10 limits
With reference to the UR10 datasheet as well as the onboard UR10 configuration
files (for acceleration), the joint limits are:

| Joint | Position (rad) | Velocity (rad/s)  | Acceleration (rad/s²)  |
| ----- | -------------- | ----------------- | ---------------------- |
| 1     | ± 2π           | ± ⅔π              | ± 40                   |
| 2     | ± 2π           | ± ⅔π              | ± 40                   |
| 3     | ± 2π           | ± π               | ± 40                   |
| 4     | ± 2π           | ± π               | ± 40                   |
| 5     | ± 2π           | ± π               | ± 40                   |
| 6     | ± 2π           | ± π               | ± 40                   |

## Misc notes
* The Ridgeback controller automatically publishes a zero command (i.e. brake)
  to the Ridgeback when a new connection is made; i.e., when something new
  subscribes or publishes to the `cmd_vel` topic. This appears to be due to
  [this line](https://github.com/ridgeback/ridgeback/blob/melodic-devel/ridgeback_control/src/mecanum_drive_controller.cpp#L281).
* Start or stop a particular `ros_control` controller using:
  ```
  rosrun controller_manager controller_manager start <controller_name>
  rosrun controller_manager controller_manager stop <controller_name>
  ```

## Todo
* Basic PyBullet simulation support (that can be easily extended to suit one's
  own project)
* Kalman filter Ridgeback base and UR10 joints (velocities in particular)
