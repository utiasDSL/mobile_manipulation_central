#!/usr/bin/env python3
"""Calibrate the base using data collected with `collect_base_orientation_calibration_data.py`.
"""
import argparse

import numpy as np
from spatialmath.base import rotz, r2q


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="npz file containing the calibration data.")
    args = parser.parse_args()

    # load calibration data
    # skip the first location, which is the origin
    data = np.load(args.filename)
    q0 = data["q0"]
    qs = data["qs"][1:, :]
    xds = data["xds"]
    num_configs = qs.shape[0]

    # construct least squares problem: we want to find the angle offset that
    # best fixes the orientation error between the base frame and its Vicon
    # model frame
    A = np.ones((num_configs, 1))
    b = np.zeros(num_configs)
    for i in range(num_configs):
        yaw = qs[i, 2] - q0[2]
        C_bw = rotz(yaw)[:2, :2].T
        r_w = qs[i, :2] - q0[:2]
        r_b = C_bw @ (qs[i, :2] - q0[:2])
        angle = np.arctan2(r_b[1], r_b[0])
        b[i] = angle
        print(angle)

    # solve for optimal offset
    Δθ, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    assert Δθ.size == 1

    # Negative to match Vicon zero pose convention. The original value
    # corresponds to C_vb. The desired *zero pose* for the Vicon system would
    # be to rotate by C_bv to remove the orientation offset
    C_bv = rotz(-Δθ[0])
    Q_bv = r2q(C_bv, order="xyzs")

    print(f"Optimal base orientation offset = {Δθ}")
    print(f"Q_bv = {Q_bv}")
    print("This corresponds to the Vicon zero pose convention.")


if __name__ == "__main__":
    main()
