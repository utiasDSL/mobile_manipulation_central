#!/usr/bin/env python3
"""Print positions of markers near a reference point."""
import rospy
import numpy as np

from vicon_bridge.msg import Markers

import mobile_manipulation_central as mm


# positions to check [meters]
# POSITIONS = np.array([[0, 0.25, 1], [1.5, 1, 1], [-0.5, 2, 1]])
POSITIONS = np.array([[0, 0, 0]])

# print marker position if it is within this distance [meters] of one of the
# above positions
RADIUS = 0.2

# Set entries to False to mask out x, y, or z components, respectively
COORDINATE_MASK = [True, True, False]

# Set True to only print out markers that do not belong to an object
ONLY_PRINT_UNATTACHED_MARKERS = True


class ViconMarkerPrinter:
    def __init__(self):
        """Print out markers near reference points."""
        self.marker_sub = rospy.Subscriber("/vicon/markers", Markers, self._marker_cb)

    def _marker_cb(self, msg):
        positions = []
        for marker in msg.markers:
            if ONLY_PRINT_UNATTACHED_MARKERS and marker.marker_name != "":
                continue

            p = marker.translation
            p = np.array([p.x, p.y, p.z]) / 1000  # convert to meters

            # get offset, mask out coordinates we don't care about
            r = (POSITIONS - p) * COORDINATE_MASK

            # distance from each point
            d = np.sum(r**2, axis=1)

            if np.any(d <= RADIUS**2):
                positions.append(p)

        # sort by x-position
        if len(positions) > 0:
            positions = np.array(positions)
            idx = np.argsort(positions[:, 0])
            print(positions[idx, :])
        else:
            print("no markers found")


def main():
    np.set_printoptions(precision=6, suppress=True)
    rospy.init_node("vicon_marker_printer")
    printer = ViconMarkerPrinter()
    rospy.spin()


if __name__ == "__main__":
    main()
