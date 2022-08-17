#!/usr/bin/env python3
import argparse

import numpy as np
import rospy

import mobile_manipulation_central as mm

RATE = 125  # Hz
DURATION = 10.0  # seconds
P_GAIN = 0.5
MAX_JOINT_VELOCITY = 0.5  # just in case
AMPLITUDE = 0.2
FREQUENCY = 0.2


def sinusoid(amplitude, frequency, time):
    f = 2 * np.pi * frequency

    q = amplitude * (1 - np.cos(f * time))
    v = amplitude * f * np.sin(f * time)
    a = amplitude * f ** 2 * np.cos(f * time)

    return q, v, a


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "joint_index",
        help="Index of the arm joint to actuate with the sine trajectory",
        type=int,
    )
    parser.add_argument(
        "--dry-run",
        help="Don't send any commands, just print out what would be sent.",
        action="store_true",
    )
    args = parser.parse_args()

    rospy.init_node("sine_trajectory")

    robot = mm.UR10ROSInterface()

    # wait until robot feedback has been received
    rate = rospy.Rate(RATE)
    while not rospy.is_shutdown() and not robot.ready():
        rate.sleep()

    # start from wherever the robot currently is
    q0 = robot.q.copy()
    qd = q0.copy()
    vd = np.zeros(robot.nv)

    # control loop
    t0 = rospy.Time.now().to_sec()
    t = t0
    while not rospy.is_shutdown() and t - t0 < DURATION:
        qdi, vdi, _ = sinusoid(AMPLITUDE, FREQUENCY, t - t0)
        qd[args.joint_index] = q0[args.joint_index] + qdi
        vd[args.joint_index] = vdi

        cmd_vel = P_GAIN * (qd - robot.q) + vd
        cmd_vel = mm.bound_array(cmd_vel, lb=-MAX_JOINT_VELOCITY, ub=MAX_JOINT_VELOCITY)
        if args.dry_run:
            print(cmd_vel)
        else:
            robot.publish_cmd_vel(cmd_vel)
        t = rospy.Time.now().to_sec()

    robot.brake()


if __name__ == "__main__":
    main()
