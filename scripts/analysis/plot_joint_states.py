"""Plot UR10 and Ridgeback joint position and velocity from a ROS bag."""
import argparse

import numpy as np
import rosbag
import matplotlib.pyplot as plt
from mobile_manipulation_central import ros_utils


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bagfile", help="Bag file to plot.")
    args = parser.parse_args()

    bag = rosbag.Bag(args.bagfile)

    ur10_msgs = [msg for _, msg, _ in bag.read_messages("/ur10/joint_states")]
    ridgeback_msgs = [msg for _, msg, _ in bag.read_messages("/ridgeback/joint_states")]

    tas, qas, vas = ros_utils.parse_ur10_joint_state_msgs(ur10_msgs)
    tbs, qbs, vbs = ros_utils.parse_ridgeback_joint_state_msgs(ridgeback_msgs)

    plt.figure()
    plt.plot(tbs, qbs)
    plt.title("Ridgeback Joint Positions")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint position")
    plt.grid()

    plt.figure()
    plt.plot(tbs, vbs)
    plt.title("Ridgeback Joint Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity")
    plt.grid()

    plt.figure()
    plt.plot(tas, qas)
    plt.title("UR10 Joint Positions")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint position (rad)")
    plt.grid()

    plt.figure()
    plt.plot(tas, vas)
    plt.title("UR10 Joint Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity (rad/s)")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()
