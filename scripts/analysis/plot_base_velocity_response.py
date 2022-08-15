import sys
from functools import partial

import numpy as np
import rosbag
import matplotlib.pyplot as plt
from scipy import signal, optimize
from mobile_manipulation_central import BAG_DIR, ros_utils

import IPython


BAG_PATHS = [
    BAG_DIR + "/2022-08-15/" + path
    for path in [
        "ridgeback_velocity_test_x_2022-08-15-15-45-07.bag",
        "ridgeback_velocity_test_y_2022-08-15-15-48-36.bag",
        "ridgeback_velocity_test_yaw_2022-08-15-15-50-09.bag",
    ]
]

JOINT_NAMES = ["Base X", "Base Y", "Base Yaw"]


def process_one_bag(path, joint_idx):
    bag = rosbag.Bag(path)

    feedback_msgs = [
        msg for _, msg, _ in bag.read_messages("/vicon/ThingBase/ThingBase")
    ]

    cmd_msgs = [msg for _, msg, _ in bag.read_messages("/ridgeback_velocity_controller/cmd_vel")][1:]
    cmd_ts = np.array([t.to_sec() for _, _, t in bag.read_messages("/ridgeback_velocity_controller/cmd_vel")])[1:]

    # trim messages to around the time of the step command
    t0 = cmd_ts[0] - 0.1
    t1 = cmd_ts[-1] + 1.0
    feedback_msgs = ros_utils.trim_msgs(feedback_msgs, t0=t0, t1=t1)

    # parse Vicon messages for base pose
    ts = []
    qs = []
    for msg in feedback_msgs:
        ts.append(ros_utils.msg_time(msg))
        x = msg.transform.translation.x
        y = msg.transform.translation.y
        θ = ros_utils.yaw_from_quaternion_msg(msg.transform.rotation)
        qs.append(np.array([x, y, θ]))

    # numerically differentiate base pose to get velocities
    vs = []
    for i in range(len(ts) - 1):
        dt = ts[i + 1] - ts[i]
        dq = qs[i + 1] - qs[i]
        vs.append(dq / dt)
    vs = np.array(vs)

    # command values should all be the same
    cmd_value = [cmd_msgs[0].linear.x, cmd_msgs[0].linear.y, cmd_msgs[0].angular.z][joint_idx]
    us = np.zeros_like(ts)
    for i in range(len(ts)):
        us[i] = 0 if ts[i] < cmd_ts[0] or ts[i] > cmd_ts[-1] else cmd_value

    ts = np.array(ts) - ts[0]

    plt.figure()

    # actual output
    plt.plot(ts[:-1], vs[:, joint_idx], label="Actual")

    # commands
    plt.plot(ts, us, linestyle="--", label="Commanded")

    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity (rad/s)")
    plt.legend()
    plt.grid()
    plt.title(f"{JOINT_NAMES[joint_idx]} Velocity")


def main():
    for i, path in enumerate(BAG_PATHS):
        process_one_bag(path, i)
    plt.show()


if __name__ == "__main__":
    main()
