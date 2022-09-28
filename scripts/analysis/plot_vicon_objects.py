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

    # get all Vicon object topics
    topics = list(bag.get_type_and_topic_info()[1].keys())
    try:
        topics.remove("/vicon/markers")
    except ValueError:
        pass
    topics = [topic for topic in topics if topic.startswith("/vicon/")]
    names = [topic.split("/")[-1] for topic in topics]

    # plot each Vicon object
    for name, topic in zip(names, topics):
        msgs = [msg for _, msg, _ in bag.read_messages(topic)]
        positions = []
        for msg in msgs:
            p = msg.transform.translation
            positions.append([p.x, p.y, p.z])
        positions = np.array(positions)
        times = ros_utils.parse_time(msgs)

        # x, y, z position vs. time
        plt.figure()
        plt.plot(times, positions[:, 0], label="x")
        plt.plot(times, positions[:, 1], label="y")
        plt.plot(times, positions[:, 2], label="z")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (m)")
        plt.title(f"{name} position vs. time")
        plt.legend()
        plt.grid()

        # x, y, z position in 3D space
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.plot(positions[:, 0], positions[:, 1], zs=positions[:, 2])
        ax.plot(positions[0, 0], positions[0, 1], positions[0, 2], "o", color="g", label="Start")
        ax.plot(positions[-1, 0], positions[-1, 1], positions[-1, 2], "o", color="r", label="End")
        ax.grid()
        ax.legend()
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")

        ax.set_xlim([-3, 3])
        ax.set_ylim([-3, 3])

    plt.show()


if __name__ == "__main__":
    main()
