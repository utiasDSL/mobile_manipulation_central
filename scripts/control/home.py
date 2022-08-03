#!/usr/bin/env python3
"""Send the UR10 to its home position."""
import argparse
import sys

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mobile_manipulation_central import TrajectoryClient, UR10_JOINT_NAMES, UR10_HOME


if __name__ == "__main__":
    rospy.init_node("home")

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "duration", type=float, help="Duration of homing trajectory."
    )
    cli_args = parser.parse_args()

    if cli_args.duration < 3.0:
        print("Home trajectory duration should be at least 3 seconds.")
        sys.exit(1)

    trajectory = JointTrajectory()
    trajectory.joint_names = UR10_JOINT_NAMES
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(cli_args.duration)
    point.positions = UR10_HOME
    trajectory.points.append(point)

    client = TrajectoryClient("scaled_vel_joint_traj_controller")
    client.send_joint_trajectory(trajectory)
