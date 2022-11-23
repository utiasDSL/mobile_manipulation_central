#!/usr/bin/env python3
"""Send the robot to its home position.

The user can choose to send just the base, just the arm, or both to a
configurable home position by passing --base-only or --arm-only. The home
position is identified by passing a name as the first argument, corresponding
to an entry in config/home.yaml.
"""
import argparse

import rospy
import numpy as np

import mobile_manipulation_central as mm


MAX_JOINT_VELOCITY = 0.5
MAX_JOINT_ACCELERATION = 1.0
P_GAIN = 10
CONVERGENCE_TOL = 1e-2
RATE = 125  # Hz


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "name",
        help="Name of the home position to move to.",
        nargs="?",
        default="default",
    )
    parser.add_argument(
        "--dry-run",
        help="Don't send any commands, just print out what would be sent.",
        action="store_true",
    )
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument("--arm-only", help="Only home the arm", action="store_true")
    grp.add_argument("--base-only", help="Only home the base", action="store_true")
    args = parser.parse_args()

    rospy.init_node("home")

    home = mm.load_home_position(args.name)

    # select which robot component we're using
    if args.arm_only:
        robot = mm.UR10ROSInterface()
        home = home[-robot.nq :]
    elif args.base_only:
        robot = mm.RidgebackROSInterface()
        home = home[: robot.nq]
    else:
        robot = mm.MobileManipulatorROSInterface()

    rate = rospy.Rate(RATE)

    # wait until robot feedback has been received
    while not rospy.is_shutdown() and not robot.ready():
        rate.sleep()
    q0 = robot.q.copy()

    trajectory = PointToPointTrajectory.quintic(q0, home, MAX_JOINT_VELOCITY, MAX_JOINT_ACCELERATION)

    # use P control to navigate to robot home, with limits on the velocity
    while not rospy.is_shutdown():
        if np.linalg.norm(home - robot.q) < CONVERGENCE_TOL:
            break

        t = rospy.Time.now().to_sec()
        qd, vd, _ = trajectory.sample(t)
        cmd_vel = P_GAIN * (qd - q) + vd
        cmd_vel = mm.bound_array(cmd_vel, lb=-MAX_JOINT_VELOCITY, ub=MAX_JOINT_VELOCITY)

        if args.dry_run:
            print(cmd_vel)
        else:
            robot.publish_cmd_vel(cmd_vel)

        rate.sleep()

    robot.brake()

    print(f"Converged to within {np.linalg.norm(error)} of home position.")


if __name__ == "__main__":
    main()
