#!/usr/bin/env python3
"""Collect calibration data for the arm.

Sends the arm to a sequence of desired configurations, takes the average
measured configuration at each point, and saves the data.

Note that the base must also be running, since its (stationary) pose is also
recorded by Vicon.
"""
import argparse
import datetime
import time

import rospy
import numpy as np
from scipy.spatial.transform import Rotation

import mobile_manipulation_central as mm


MAX_JOINT_VELOCITY = 0.2
MAX_JOINT_ACCELERATION = 1.0
P_GAIN = 1
CONVERGENCE_TOL = 1e-3
RATE = 125  # Hz
EE_OBJECT_NAME = "ThingWoodTray"

# fmt: off
# last arm configuration is same as the first, so the robot goes back when done
# calibrating
DESIRED_ARM_CONFIGURATIONS = np.array([
  [1.5708, -0.7854,  1.5708, -0.7854,  1.5708, 1.3100],
  [0,      -0.7854,  1.5708, -0.7854,  1.5708, 1.3100],
  [2.3562, -0.7854,  1.5708, -0.7854,  1.5708, 1.3100],
  [2.3562, -1.5708,  1.5708, -0.7854,  1.5708, 1.3100],
  [2.3562, -1.5708,  0.7854, -0.7854,  1.5708, 1.3100],
  [2.3562, -1.5708,  0.7854,       0,  1.5708, 1.3100],
  [2.3562, -1.5708,  0.7854,       0,       0, 1.3100],
  [1.5708, -1.5708,  0.7854,       0,       0, 0.5236],
  [0,      -1.5708,  0.7854,       0,       0, 0.5236],
  [1.5708, -0.7854,  1.5708, -0.7854,  1.5708, 1.3100]])
# fmt: on


def average_quaternion(Qs):
    """Compute the average of a set of quaternions.

    The quaternions are in ``[x, y, z, w]`` order.

    Parameters
    ----------
    Qs : np.ndarray, shape (n, 4)
        The quaternions to average.

    Returns
    -------
    : np.ndarray, shape (4,)
        The average quaternion.
    """
    Qs = np.array(Qs)
    e, V = np.linalg.eig(Qs.T @ Qs)
    i = np.argmax(e)
    Q_avg = V[:, i]

    # TODO could use scipy but I want to canonical option, which requires a
    # more recent version
    # Q_avg2 = Rotation.from_quat(Qs).mean().as_quat()
    # print(f"Q_avg = {Q_avg}")
    # print(f"Q_avg2 = {Q_avg2}")
    return Q_avg


def average_measurements(robot, vicon, rate, duration=5.0):
    """Compute the average measurements over a given duration.

    Parameters
    ----------
    robot : MobileManipulatorROSInterface
        Interface to the robot.
    vicon : ViconObjectInterface
        Interface to the Vicon object.
    rate : rospy.Rate
        Rate for the measurement loop.
    duration : float, non-negative
        The duration over which to compute the averages.

    Returns
    -------
    : tuple
        A tuple ``(q, r, Q)``, where ``q`` is the average joint configuration,
        ``r`` is the average object position, and ``Q`` is the average object
        orientation as a quaternion ``[x, y, z, w]``.
    """
    qs = []  # joint configurations
    Qs = []  # object orientation quaternions
    rs = []  # object translations

    t0 = rospy.Time.now().to_sec()
    t = t0
    while not rospy.is_shutdown() and t - t0 < duration:
        qs.append(robot.q.copy())
        rs.append(vicon.position.copy())
        Qs.append(vicon.orientation.copy())

        rate.sleep()
        t = rospy.Time.now().to_sec()

    q = np.mean(qs, axis=0)
    r = np.mean(rs, axis=0)
    Q = average_quaternion(Qs)
    return q, r, Q


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "filename",
        help="Filename for the saved data. Timestamp is automatically appended.",
        nargs="?",
        default="arm_calibration_data",
    )
    parser.add_argument(
        "--home-config",
        help="Path to YAML file to load the home configurations from.",
        required=True,
    )
    parser.add_argument(
        "--home-name",
        help="Name of the home position to use for the base.",
        default="default",
    )
    args = parser.parse_args()

    # home position is used to determine where the base should move to (and
    # stay at) for all data collection
    home = mm.load_home_position(name=args.home_name, path=args.home_config)

    rospy.init_node("arm_calibration_data_collection", disable_signals=True)
    signal_handler = mm.SimpleSignalHandler()

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    robot = mm.MobileManipulatorROSInterface()
    vicon = mm.ViconObjectInterface(EE_OBJECT_NAME)
    rate = rospy.Rate(RATE)

    # wait until robot and Vicon feedback has been received
    while not rospy.is_shutdown() and not signal_handler.received:
        if robot.ready() and vicon.ready():
            break
        rate.sleep()

    q0 = robot.q.copy()

    num_configs = DESIRED_ARM_CONFIGURATIONS.shape[0]
    goals = np.hstack(
        (
            np.tile(home[:3], (num_configs, 1)),
            DESIRED_ARM_CONFIGURATIONS,
        )
    )

    # measured configurations (not exactly the same as desired)
    qs = []

    # measured Vicon poses represented as (r, Q) = (translation vector,
    # orientation quaternion)
    rs = []
    Qs = []

    goal = goals[0, :]
    delta = goal - q0
    trajectory = mm.PointToPointTrajectory.quintic(
        start=q0,
        delta=delta,
        max_vel=MAX_JOINT_VELOCITY,
        max_acc=MAX_JOINT_ACCELERATION,
    )
    print(f"Moving to goal 0 with duration {trajectory.duration} seconds.")

    # use P control to navigate to robot home, with limits on the velocity
    idx = 0
    while not rospy.is_shutdown() and not signal_handler.received:
        t = rospy.Time.now().to_sec()
        dist = np.linalg.norm(goal - robot.q)
        if trajectory.done(t) and dist < CONVERGENCE_TOL:
            robot.brake()
            print(f"Converged to location {idx} with distance {dist}.")

            idx += 1
            if idx >= num_configs:
                break

            q, r, Q = average_measurements(robot, vicon, rate)
            qs.append(q)
            rs.append(r)
            Qs.append(Q)

            print(f"Average configuration = {q}.")
            print(f"Average position = {r}.")
            print(f"Average quaternion = {Q}.")

            # build trajectory to the next waypoint
            goal = goals[idx, :]
            delta = goal - q
            trajectory = mm.PointToPointTrajectory.quintic(
                start=q,
                delta=delta,
                max_vel=MAX_JOINT_VELOCITY,
                max_acc=MAX_JOINT_ACCELERATION,
            )
            print(f"Moving to goal {idx} with duration {trajectory.duration} seconds.")

        t = rospy.Time.now().to_sec()
        qd, vd, _ = trajectory.sample(t)
        cmd_vel = P_GAIN * (qd - robot.q) + vd
        cmd_vel = np.clip(
            a=cmd_vel, a_min=-MAX_JOINT_VELOCITY, a_max=MAX_JOINT_VELOCITY
        )
        robot.publish_cmd_vel(cmd_vel)

        rate.sleep()

    print("Braking robot")
    robot.brake()
    time.sleep(0.5)

    filename = f"{args.filename}_{timestamp}.npz"
    np.savez_compressed(filename, qds=goals, qs=qs, rs=rs, Qs=Qs)
    print(f"Arm calibration data saved to {filename}.")


if __name__ == "__main__":
    main()
