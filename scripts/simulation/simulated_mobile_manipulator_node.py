#!/usr/bin/env python3
"""Single integrator ROS end point to simulate base and arm.

This is intended to simulate to minimally simulate a robot with other real
components of the system.
"""
import rospy
import numpy as np
from spatialmath.base import rotz

from mobile_manipulation_central import (
    SimulatedRidgebackROSInterface,
    SimulatedUR10ROSInterface,
)


BASE_HOME = np.array([-2, 1, 0])
ARM_HOME = np.pi * np.array([0.5, -0.25, 0.5, -0.25, 0.5, 0.417])
RATE = 125  # Hz


def main():
    rospy.init_node("simulated_mobile_manipulator_node")

    base = SimulatedRidgebackROSInterface()
    q_base = BASE_HOME
    v_base_body = np.zeros(base.nv)

    arm = SimulatedUR10ROSInterface()
    q_arm = ARM_HOME
    v_arm = np.zeros(arm.nv)

    rate = rospy.Rate(RATE)

    t_prev = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec()
        dt = t - t_prev
        t_prev = t

        # arm
        if arm.cmd_vel is not None:
            v_arm = arm.cmd_vel
        q_arm = q_arm + dt * v_arm
        arm.publish_feedback(t, q_arm, v_arm)

        # base
        if base.cmd_vel is not None:
            v_base_body = base.cmd_vel
        C_wb = rotz(q_base[2])  # rotate from body to world
        v_base_world = C_wb @ v_base_body
        q_base = q_base + dt * v_base_world
        base.publish_feedback(t, q_base, v_base_world)

        rate.sleep()


if __name__ == "__main__":
    main()
