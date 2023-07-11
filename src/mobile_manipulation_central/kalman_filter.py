from collections import namedtuple

import numpy as np
import scipy


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

        # there are a few options for solving these equations; the main thing
        # is to exploit the fact that S is p.d. by telling scipy
        Xx = scipy.linalg.solve(S, np.hstack((CP, z[:, None])), assume_a="pos")
        x = e.x + CP.T @ Xx[:, -1]
        P = e.P - CP.T @ Xx[:, :-1]

        # another option is to compute the inverse directly
        # Sinv = scipy.linalg.solve(S, np.eye(R.shape[0]), assume_a="pos")
        # x = e.x + CP.T @ Sinv @ z
        # P = e.P - CP.T @ Sinv @ CP

        # or solve the two systems separately
        # x = e.x + CP.T @ scipy.linalg.solve(S, z, assume_a="pos")
        # P = e.P - CP.T @ scipy.linalg.solve(S, CP, assume_a="pos")
        return GaussianEstimate(x, P)

    @staticmethod
    def nis(e, C, R, y):
        """Normalized innovation squared."""
        z = y - C @ e.x
        S = C @ P @ C.T + R
        return z @ np.linalg.solve(S, z)
