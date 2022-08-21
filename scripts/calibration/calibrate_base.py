#!/usr/bin/env python3
"""Calibrate the base using data collected with `collect_base_calibration_data.py`.

Generates an offset Δq_b to the measured configurations such that the desired
configurations are as close as possible to the measured ones in the sense of
least squares.
"""
import argparse

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="npz file containing the calibration data.")
    args = parser.parse_args()

    # load calibration data
    data = np.load(args.filename)
    q0 = data["q0"]
    qs = data["qs"]
    qds = data["qds"]
    num_configs = qs.shape[0]

    # construct least squares problem
    A = np.tile(np.eye(3), (num_configs, 1))
    b = np.zeros(3 * num_configs)
    for i in range(num_configs):
        b[i*3:i*3+3] = qds[i, :] - qs[i, :]

    # solve for optimal offset
    Δq, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

    print(f"Optimal base offset = {Δq}")


if __name__ == "__main__":
    main()
