"""Plot velocity step responses for the Ridgeback base."""
import sys
from functools import partial
import argparse

import numpy as np
import rosbag
import matplotlib.pyplot as plt
from scipy import signal, optimize
from mobile_manipulation_central import BAG_DIR, ros_utils
import mobile_manipulation_central.system_identification as sysid


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

    vicon_msgs = [
        msg for _, msg, _ in bag.read_messages("/vicon/ThingBase/ThingBase")
    ]

    # remove first cmd since it is always zero (ridgeback controller seems to
    # automatically publish a single zero command when a connection is first
    # made)
    cmd_msgs = [msg for _, msg, _ in bag.read_messages("/ridgeback_velocity_controller/cmd_vel")][1:]
    cmd_ts = np.array([t.to_sec() for _, _, t in bag.read_messages("/ridgeback_velocity_controller/cmd_vel")])[1:]

    # trim messages to around the time of the step command
    t0 = cmd_ts[0] - 0.1
    t1 = cmd_ts[-1] + 1.0
    vicon_msgs = ros_utils.trim_msgs(vicon_msgs, t0=t0, t1=t1)

    # parse Vicon messages for base pose
    ts, qs = ros_utils.parse_ridgeback_vicon_msgs(vicon_msgs)

    # numerically differentiate base pose to get velocities
    vs = []
    for i in range(len(ts) - 1):
        dt = ts[i + 1] - ts[i]
        dq = qs[i + 1] - qs[i]
        vs.append(dq / dt)
    vs = np.array(vs)[:, joint_idx]

    # remove last time to align with size of vs
    ts = np.array(ts)[:-1]

    # command values should all be the same
    cmd_value = [cmd_msgs[0].linear.x, cmd_msgs[0].linear.y, cmd_msgs[0].angular.z][joint_idx]
    us = np.zeros_like(ts)
    for i in range(len(ts)):
        us[i] = 0 if ts[i] < cmd_ts[0] or ts[i] > cmd_ts[-1] else cmd_value

    # normalize time
    ts -= ts[0]

    # fit first- and second-order models to the data
    ωn, ζ = sysid.identify_second_order_system(ts, us, vs)
    τ = sysid.identify_first_order_system(ts, us, vs)

    # forward simulate the models
    vs_fit1 = sysid.simulate_first_order_system(ts, τ, us)
    vs_fit2 = sysid.simulate_second_order_system(ts, ωn, ζ, us)

    # plot
    plt.figure()

    # actual output
    plt.plot(ts, vs, label="Actual")

    # commands
    plt.plot(ts, us, linestyle="--", label="Commanded")

    # fitted models
    plt.plot(ts, vs_fit1, label="First-order model")
    plt.plot(ts, vs_fit2, label="Second-order model")

    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity (rad/s)")
    plt.legend()
    plt.grid()
    plt.title(f"{JOINT_NAMES[joint_idx]} Velocity")

    if save:
        name = f"{JOINT_NAMES[joint_idx]} Velocity.png".lower()
        fig.savefig(name)
        print(f"Saved plot to {name}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--save", action="store_true", help="Save the plots to files.")
    args = parser.parse_args()

    for i, path in enumerate(BAG_PATHS):
        process_one_bag(path, i)
    plt.show()


if __name__ == "__main__":
    main()
