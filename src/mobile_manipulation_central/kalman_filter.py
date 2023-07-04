from collections import namedtuple

import numpy as np


GaussianEstimate = namedtuple("GaussianEstimate", ["x", "P"])

class KalmanFilter:
    @staticmethod
    def predict(e, A, Q, v):
        """Kalman filter prediction step.

        Parameters:
            e: current GaussianEstimate
            A: state transition matrix
            Q: process noise covariance
            v: input

        Returns: predicted GaussianEstimate
        """
        x = A @ e.x + v
        P = A @ e.P @ A.T + Q
        return GaussianEstimate(x, P)

    @staticmethod
    def correct(e, C, R, y):
        """Kalman filter correction step.

        Parameters:
            e: current GaussianEstimate
            C: measurement matrix
            R: measurement noise covariance
            y: measurement value

        Returns: corrected GaussianEstimate
        """
        CP = C @ e.P
        S = CP @ C.T + R
        z = y - C @ e.x

        x = e.x + CP.T @ np.linalg.solve(S, z)
        P = e.P - CP.T @ np.linalg.solve(S, CP)
        return GaussianEstimate(x, P)

    @staticmethod
    def nis(e, C, R, y):
        """Normalized innovation squared."""
        z = y - C @ e.x
        S = C @ P @ C.T + R
        return z @ np.linalg.solve(S, z)
