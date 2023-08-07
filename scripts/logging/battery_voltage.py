#!/usr/bin/env python3
"""Print out the Ridgeback battery voltage, as reported by the Ridgeback node."""
import rospy
from diagnostic_msgs.msg import DiagnosticArray


def diag_cb(msg):
    for component in msg.status:
        if component.name == "ridgeback_node: Battery":
            print(component)


def main():
    rospy.init_node("battery_voltage_node")
    rb_diagnostics_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, diag_cb)
    rospy.spin()


if __name__ == "__main__":
    main()
