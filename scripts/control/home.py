#!/usr/bin/env python3
"""Send the robot to its home position.

The user can choose to send just the base, just the arm, or both to a
configurable home position by passing --base-only or --arm-only. The home
position is identified by passing a name as the first argument, corresponding
to an entry in home config file.

The default config file is located in `config/home.yaml`, but another can be
passed using the --config argument.
"""
import argparse
import signal

import rospy
import numpy as np

import mobile_manipulation_central as mm

# NOTE: as currently implemented, the base will always move so as to avoid
# wrapping around between +-π, even if that means taking the long way around
# the circle

MAX_JOINT_VELOCITY = 0.2
MAX_JOINT_ACCELERATION = 0.5
MIN_DURATION = 2.0  # seconds
P_GAIN = 1
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
    parser.add_argument(
        "--config",
        help="Path to YAML file to load the home configurations from.",
    )
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument("--arm-only", help="Only home the arm", action="store_true")
    grp.add_argument("--base-only", help="Only home the base", action="store_true")
    args = parser.parse_args()

    rospy.init_node("home", disable_signals=True)

    home = mm.load_home_position(name=args.name, path=args.config)

    # select which robot component we're using
    if args.arm_only:
        robot = mm.UR10ROSInterface()
        home = home[-robot.nq :]
    elif args.base_only:
        robot = mm.RidgebackROSInterface()
        home = home[: robot.nq]
    else:
        robot = mm.MobileManipulatorROSInterface()

    # enable custom signal handler
    signal_handler = mm.RobotSignalHandler(robot, args.dry_run)

    rate = rospy.Rate(RATE)

    # wait until robot feedback has been received
    while not rospy.is_shutdown() and not robot.ready():
        rate.sleep()
    q0 = robot.q.copy()

    # build the trajectory
    # to properly work with yaw angle, we need to use differences wrapped to π
    delta = home - q0
    if not args.arm_only:
        delta[2] = mm.wrap_to_pi(delta[2])
    trajectory = mm.PointToPointTrajectory.quintic(
        q0, delta, MAX_JOINT_VELOCITY, MAX_JOINT_ACCELERATION, min_duration=MIN_DURATION
    )

    # use P control + feedforward velocity to track the trajectory
    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        dist = np.linalg.norm(home - robot.q)

        # we want to both let the trajectory complete and ensure we've
        # converged properly
        if trajectory.done(t) and dist < CONVERGENCE_TOL:
            break

        qd, vd, _ = trajectory.sample(t)
        error = qd - robot.q
        if not args.arm_only:
            error[2] = mm.wrap_to_pi(error[2])
        cmd_vel = P_GAIN * error + vd

        # this shouldn't be needed unless the trajectory is poorly tracked, but
        # we do it just in case for safety
        cmd_vel = mm.bound_array(cmd_vel, lb=-MAX_JOINT_VELOCITY, ub=MAX_JOINT_VELOCITY)

        if args.dry_run:
            print(cmd_vel)
        else:
            robot.publish_cmd_vel(cmd_vel, bodyframe=False)

        rate.sleep()

    if not args.dry_run:
        robot.brake()

    print(f"Converged to within {dist} of home position.")


if __name__ == "__main__":
    main()
