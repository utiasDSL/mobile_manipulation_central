#!/usr/bin/env python3
"""Collect calibration data for the base.

Sends the base to a sequence of desired configurations, takes the average
measured configuration at each point, and saves the data.
"""
import argparse
import datetime

import rospy
import numpy as np

import mobile_manipulation_central as mm


MAX_JOINT_VELOCITY = 0.2
P_GAIN = 0.5
CONVERGENCE_TOL = 1e-2
RATE = 100  # Hz


def average_configuration(robot, rate, duration=5.0):
    qs = []
    t0 = rospy.Time.now().to_sec()
    t = t0
    while not rospy.is_shutdown() and t - t0 < duration:
        qs.append(robot.q.copy())
        rate.sleep()
        t = rospy.Time.now().to_sec()
    return np.mean(qs, axis=0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "filename",
        help="Filename for the saved data. Timestamp is automatically appended.",
        nargs="?",
        default="base_calibration_data",
    )
    args = parser.parse_args()

    rospy.init_node("base_calibration_data_collection")

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    robot = mm.RidgebackROSInterface()
    rate = rospy.Rate(RATE)

    # wait until robot feedback has been received
    while not rospy.is_shutdown() and not robot.ready():
        rate.sleep()

    # desired configurations: just rotate
    q0 = robot.q.copy()
    θds = np.array([0, np.pi / 6, np.pi / 3, np.pi / 2, 0]) + q0[2]
    num_configs = θds.shape[0]

    # measured configurations
    qs = []

    # only do control on the angle, which we then use to calibrate the position
    idx = 0
    while not rospy.is_shutdown():
        error = θds[idx] - robot.q[2]
        if np.abs(error) < CONVERGENCE_TOL:
            robot.brake()
            print(f"Converged to location {idx}.")

            idx += 1
            if idx >= num_configs:
                break

            q = average_configuration(robot, rate)
            qs.append(q)

            print(f"Average configuration = {q}.")

        cmd_vel = np.array([0, 0, P_GAIN * error])
        cmd_vel = mm.bound_array(cmd_vel, lb=-MAX_JOINT_VELOCITY, ub=MAX_JOINT_VELOCITY)
        robot.publish_cmd_vel(cmd_vel)

        rate.sleep()

    robot.brake()

    filename = f"{args.filename}_{timestamp}.npz"
    np.savez_compressed(filename, q0=q0, θds=θds, qs=qs)
    print(f"Base calibration data saved to {filename}.")


if __name__ == "__main__":
    main()
