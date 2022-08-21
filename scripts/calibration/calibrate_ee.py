#!/usr/bin/env python3
"""Calibrate the arm using data collected with `collect_arm_calibration_data.py`.

Generates an offset Δq_b to the measured configurations such that the desired
configurations are as close as possible to the measured ones in the sense of
least squares.
"""
import argparse

import numpy as np

import IPython


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("filename", help="npz file containing the calibration data.")
    args = parser.parse_args()

    # load calibration data
    data = np.load(args.filename)
    qs = data["qs"]
    qds = data["qds"]
    rs = data["rs"]
    Qs = data["Qs"]
    num_configs = qs.shape[0]

    IPython.embed()


if __name__ == "__main__":
    main()
