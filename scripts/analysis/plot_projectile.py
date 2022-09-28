"""Plot position of Vicon objects from a ROS bag."""
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

    est_msgs = [msg for _, msg, _ in bag.read_messages("/projectile/joint_states")]
    est_positions = np.array([msg.position for msg in est_msgs])
    est_velocities = np.array([msg.velocity for msg in est_msgs])

    gt_msgs = [msg for _, msg, _ in bag.read_messages("/Projectile/true_joint_states")]
    gt_positions = np.array([msg.position for msg in gt_msgs])
    gt_velocities = np.array([msg.velocity for msg in gt_msgs])

    t0 = ros_utils.msg_time(gt_msgs[0])
    gt_times = ros_utils.parse_time(gt_msgs, t0=t0)
    est_times = ros_utils.parse_time(est_msgs, t0=t0)

    # x, y, z position vs. time
    plt.figure()
    plt.plot(gt_times, gt_positions[:, 0], label="x", color="r")
    plt.plot(gt_times, gt_positions[:, 1], label="y", color="g")
    plt.plot(gt_times, gt_positions[:, 2], label="z", color="b")
    plt.plot(est_times, est_positions[:, 0], label="x_est", linestyle="--", color="r")
    plt.plot(est_times, est_positions[:, 1], label="y_est", linestyle="--", color="g")
    plt.plot(est_times, est_positions[:, 2], label="z_est", linestyle="--", color="b")
    plt.xlabel("Time (s)")
    plt.ylabel("Position (m)")
    plt.title(f"Projectile position vs. time")
    plt.legend()
    plt.grid()

    plt.figure()
    plt.plot(gt_times, gt_velocities[:, 0], label="x", color="r")
    plt.plot(gt_times, gt_velocities[:, 1], label="y", color="g")
    plt.plot(gt_times, gt_velocities[:, 2], label="z", color="b")
    plt.plot(est_times, est_velocities[:, 0], label="x_est", linestyle="--", color="r")
    plt.plot(est_times, est_velocities[:, 1], label="y_est", linestyle="--", color="g")
    plt.plot(est_times, est_velocities[:, 2], label="z_est", linestyle="--", color="b")
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title(f"Projectile velocity vs. time")
    plt.legend()
    plt.grid()

    # # x, y, z position in 3D space
    # fig = plt.figure()
    # ax = fig.add_subplot(projection="3d")
    # ax.plot(positions[:, 0], positions[:, 1], zs=positions[:, 2])
    # ax.plot(positions[0, 0], positions[0, 1], positions[0, 2], "o", color="g", label="Start")
    # ax.plot(positions[-1, 0], positions[-1, 1], positions[-1, 2], "o", color="r", label="End")
    # ax.grid()
    # ax.legend()
    # ax.set_xlabel("x (m)")
    # ax.set_ylabel("y (m)")
    # ax.set_zlabel("z (m)")
    #
    # ax.set_xlim([-3, 3])
    # ax.set_ylim([-3, 3])

    plt.show()


if __name__ == "__main__":
    main()
