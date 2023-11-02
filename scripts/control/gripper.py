#!/usr/bin/env python3
"""Script to control the Robotiq 3-finger gripper.

usage:
  rosrun mm_gripper gripper.py cmd [delay]
where:
  cmd can be either "open" or "close"
  delay is the number of seconds to wait before execution

options:
  --pinched  use pinched mode
  --wide     use wide mode
"""
import argparse
import sys

import rospy
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput

import IPython


# values are in the range [0, 255]
SPEED = 255
FORCE = 255


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "cmd", help="Gripper position", choices=("open", "close")
    )
    parser.add_argument(
        "delay",
        help="Seconds to delay before executing the command.",
        nargs="?",
        type=float,
        default=0,
    )
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument("--pinched", help="Pinch mode", action="store_true")
    grp.add_argument("--wide", help="Wide mode", action="store_true")
    args = parser.parse_args()

    if args.delay < 0:
        raise ValueError("Delay must be non-negative.")

    rospy.init_node("gripper_node")
    pub = rospy.Publisher(
        "/Robotiq3FGripperRobotOutput", Robotiq3FGripperRobotOutput, queue_size=10
    )

    rospy.sleep(1.0 + args.delay)

    msg = Robotiq3FGripperRobotOutput()
    msg.rACT = 1

    # 0 for normal, 1 for pinched, 2 for wide mode, 3 for scissor mode
    # NOTE scissor mode may not be working
    if args.pinched:
        msg.rMOD = 1
    elif args.wide:
        msg.rMOD = 2
    else:
        msg.rMOD = 0

    msg.rGTO = 1
    msg.rATR = 0
    msg.rICF = 0

    # position [0, 255]
    if args.cmd == "open":
        msg.rPRA = 0
    else:
        msg.rPRA = 255

    msg.rSPA = SPEED
    msg.rFRA = FORCE

    pub.publish(msg)


if __name__ == "__main__":
    main()
