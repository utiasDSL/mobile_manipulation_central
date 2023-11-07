#!/usr/bin/env python3
"""Print out the Ridgeback motor input voltages."""
import rospy
from diagnostic_msgs.msg import DiagnosticArray

import sys
import IPython


def diag_cb(msg):
    for component in msg.status:
        if component.name.startswith("ridgeback_node: Puma motor driver"):
            can_id = -1
            voltage = -1
            for item in component.values:
                if item.key == "Driver CAN ID":
                    can_id = item.value
                if item.key == "Input terminal voltage (V)":
                    voltage = item.value
            print(f"Motor {can_id}: {component.message} (input voltage = {voltage})")


def main():
    rospy.init_node("motor_voltage_node")
    rb_diagnostics_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, diag_cb)
    rospy.spin()


if __name__ == "__main__":
    main()
