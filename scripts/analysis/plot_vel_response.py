import sys
from functools import partial

import numpy as np
import rosbag
import matplotlib.pyplot as plt
from scipy import signal, optimize
from mobile_manipulation_central import BAG_DIR, ros_utils


BAG_PATHS = [
    BAG_DIR + "/2022-07-25/velocity_test/" + path
    for path in [
        "velocity_test_joint0_v0.5_2022-07-25-15-32-43.bag",
        "velocity_test_joint1_v0.5_2022-07-25-15-39-15.bag",
        "velocity_test_joint2_v0.5_2022-07-25-15-46-14.bag",
        "velocity_test_joint3_v0.5_2022-07-25-15-55-46.bag",
        "velocity_test_joint4_v0.5_2022-07-25-16-01-38.bag",
        "velocity_test_joint5_v0.5_2022-07-25-16-04-08.bag",
    ]
]


def parse_velocity_data(feedback_msgs, cmd_msgs, cmd_ts, idx):
    t0 = cmd_ts[0] - 0.1  # small extra buffer before commands start
    t1 = cmd_ts[-1]
    feedback_msgs = ros_utils.trim_msgs(feedback_msgs, t0=t0, t1=t1)
    ts, _, vs = ros_utils.parse_feedback_msgs(feedback_msgs)

    # all of the commands should be the same
    cmd_value = cmd_msgs[0].data[idx]

    us = np.zeros_like(ts)
    for i in range(ts.shape[0]):
        us[i] = 0 if ts[i] < cmd_ts[0] else cmd_value

    # normalize time so that ts[0] = 0
    ts -= ts[0]

    # take only the velocity from joint idx
    vs = vs[:, idx]

    return ts, us, vs


# system identification routines adapted from
# https://medium.com/robotics-devs/system-identification-with-python-2079088b4d03
def simulate_second_order_system(ts, ωn, ζ, us, k=1):
    """Simulate a second-order system with parameters (k, ωn, ζ) and inputs us at times ts."""
    sys = signal.TransferFunction(k * (ωn ** 2), [1, 2 * ζ * ωn, ωn ** 2])
    _, ys, _ = signal.lsim2(sys, U=us, T=ts)
    return ys


def simulate_first_order_system(ts, τ, us, k=1):
    """Simulate a second-order system with parameters (k, ωn, ζ) and inputs us at times ts."""
    sys = signal.TransferFunction(k, [τ, 1])
    _, ys, _ = signal.lsim2(sys, U=us, T=ts)
    return ys


def identify_first_order_system(ts, us, ys, method="trf", p0=[1.0]):
    """Fit a second-order model to the inputs us and outputs ys at times ts."""
    # bounds: assume system is not overdamped
    bounds = ([0], [np.inf])
    model = partial(simulate_first_order_system, us=us)
    (τ,), covariance = optimize.curve_fit(
        model,
        ts,
        ys,
        method=method,
        p0=p0,
        bounds=bounds,
    )
    return τ


def identify_second_order_system(ts, us, ys, method="trf", p0=[10.0, 0.1]):
    """Fit a second-order model to the inputs us and outputs ys at times ts."""
    # bounds: assume system is not overdamped
    bounds = ([0, 0], [np.inf, 1.0])
    model = partial(simulate_second_order_system, us=us)
    (ωn, ζ), covariance = optimize.curve_fit(
        model,
        ts,
        ys,
        method=method,
        p0=p0,
        bounds=bounds,
    )
    return ωn, ζ


def process_one_bag(path, joint_idx):
    bag = rosbag.Bag(path)

    feedback_msgs = [
        msg for _, msg, _ in bag.read_messages("/mobile_manipulator_joint_states")
    ]

    cmd_msgs = [msg for _, msg, _ in bag.read_messages("/mobile_manipulator_cmd_vel")]

    # get times of the command messagse, since these do not contain a timestamp
    cmd_ts = np.array(
        [t.to_sec() for _, _, t in bag.read_messages("/mobile_manipulator_cmd_vel")]
    )

    ts, us, ys_actual = parse_velocity_data(
        feedback_msgs, cmd_msgs, cmd_ts, idx=joint_idx
    )

    # fit a second-order model to the data
    ωn, ζ = identify_second_order_system(ts, us, ys_actual)
    τ = identify_first_order_system(ts, us, ys_actual)

    ys_fit1 = simulate_first_order_system(ts, τ, us)
    ys_fit2 = simulate_second_order_system(ts, ωn, ζ, us)

    # make sure all steps are in positive direction for consistency
    if us[-1] < 0:
        us = -us
        ys_actual = -ys_actual
        ys_fit1 = -ys_fit1
        ys_fit2 = -ys_fit2

    plt.figure()

    # actual output
    plt.plot(ts, ys_actual, label="Actual")

    # commands
    plt.plot(ts, us, linestyle="--", label="Commanded")

    # fitted models
    plt.plot(ts, ys_fit1, label="First-order model")
    plt.plot(ts, ys_fit2, label="Second-order model")

    plt.text(0.6, 0.4, f"First-order\nτ = {τ:.2f}")
    plt.text(0.6, 0.2, f"Second-order\nω = {ωn:.2f}\nζ = {ζ:.2f}")

    plt.xlabel("Time (s)")
    plt.ylabel("Joint velocity (rad/s)")
    plt.legend()
    plt.grid()
    plt.title(f"Joint {joint_idx} Velocity")


def main():
    for i, path in enumerate(BAG_PATHS):
        process_one_bag(path, i)
    plt.show()


if __name__ == "__main__":
    main()
