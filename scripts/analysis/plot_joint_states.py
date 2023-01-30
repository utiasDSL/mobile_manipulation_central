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

    ur10_cmd_msgs = [msg for _, msg, _ in bag.read_messages("/ur10/cmd_vel")]
    ur10_cmd_ts = np.array([t.to_sec() for _, _, t in bag.read_messages("/ur10/cmd_vel")])
    ur10_cmd_ts -= ur10_cmd_ts[0]

    ur10_cmd_vels = []
    for msg in ur10_cmd_msgs:
        ur10_cmd_vels.append(msg.data)
    ur10_cmd_vels = np.array(ur10_cmd_vels)

    tas, qas, vas = ros_utils.parse_ur10_joint_state_msgs(ur10_msgs)
    tbs, qbs, vbs = ros_utils.parse_ridgeback_joint_state_msgs(ridgeback_msgs)

    plt.figure()
    plt.plot(tbs, qbs[:, 0], label="x")
    plt.plot(tbs, qbs[:, 1], label="y")
    plt.plot(tbs, qbs[:, 2], label="θ")
    plt.title("Ridgeback Joint Positions")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint position")
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(tbs, vbs[:, 0], label="x")
    plt.plot(tbs, vbs[:, 1], label="y")
    plt.plot(tbs, vbs[:, 2], label="θ")
    plt.title("Ridgeback Joint Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity")
    plt.legend()
    plt.grid()

    plt.figure()
    for i in range(6):
        plt.plot(tas, qas[:, i], label=f"θ_{i+1}")
    plt.title("UR10 Joint Positions")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint position (rad)")
    plt.legend()
    plt.grid()

    plt.figure()
    for i in range(6):
        plt.plot(tas, vas[:, i], label=f"v_{i+1}")
    plt.title("UR10 Joint Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity (rad/s)")
    plt.legend()
    plt.grid()

    plt.figure()
    for i in range(6):
        plt.plot(ur10_cmd_ts, ur10_cmd_vels[:, i], label=f"vc_{i+1}")
    plt.title("UR10 Commanded Joint Velocities")
    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity (rad/s)")
    plt.legend()
    plt.grid()

    plt.show()


if __name__ == "__main__":
    main()
