#!/usr/bin/env python3
"""Calibrate the base using data collected with `collect_base_calibration_data.py`.

Generates an offset r_bv_v to the measured Vicon poses such that base frame
origin stays as close to constant as possible under pure rotation.
"""
import argparse
from spatialmath.base import rotz, q2r

import numpy as np

# provide the Vicon zero pose orientation to rotate the offset into the correct
# base frame
# TODO load from file?
Q_bv = [0, 0, -0.00538166, 0.99998552]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="npz file containing the calibration data.")
    args = parser.parse_args()

    # load calibration data
    data = np.load(args.filename)
    q0 = data["q0"]
    qs = data["qs"]
    # θds = data["θds"]
    num_configs = qs.shape[0]

    r_vw_w0 = q0[:2]
    C_wv0 = rotz(q0[2])[:2, :2]

    # construct least squares problem: we want to find a base position that
    # does not move when the base is rotated
    A = np.zeros((2 * num_configs, 2))
    b = np.zeros(2 * num_configs)
    for i in range(num_configs):
        r_vw_w = qs[i, :2]
        C_wv = rotz(qs[i, 2])[:2, :2]

        A[i * 2 : i * 2 + 2, :] = C_wv - C_wv0
        b[i * 2 : i * 2 + 2] = r_vw_w0 - r_vw_w

    # solve for optimal offset
    r_bv_v, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

    C_bv = q2r(Q_bv, order="xyzs")
    r_vb_b = -C_bv @ np.append(r_bv_v, 0)
    print(f"r_vb_b = {r_vb_b} (use this for Vicon zero pose)")


if __name__ == "__main__":
    main()
