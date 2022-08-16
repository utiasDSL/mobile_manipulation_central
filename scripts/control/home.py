#!/usr/bin/env python3
"""Send the UR10 to its home position."""
import argparse
import sys

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mobile_manipulation_central import bound_array, load_home_position, MobileManipulatorROSInterface


MAX_JOINT_VELOCITY = 0.5
K = 0.5  # gain

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("name", nargs="?", default="default", help="Name of the home position to move to.")
    args = parser.parse_args()

    rospy.init_node("home")

    robot = MobileManipulatorROSInterface()
    home = load_home_position(args.name)
    rate = rospy.Rate(125)

    # wait until robot feedback has been received
    while not rospy.is_shutdown() and not robot.ready():
        rate.sleep()

    # use P control to navigate to robot home, with limits on the velocity
    while not rospy.is_shutdown():
        error = home - robot.q
        if np.linalg.norm(error) < 1e-3:
            break
        cmd_vel = K @ error
        cmd_vel = bound_array(cmd_vel, lb=-MAX_JOINT_VELOCITY, ub=MAX_JOINT_VELOCITY)
        robot.publish_cmd_vel(cmd_vel)

    print("Done.")
