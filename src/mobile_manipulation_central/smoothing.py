import numpy as np


class ExponentialSmoother:
    """Exponential smoothing filter with time constant τ."""

    def __init__(self, τ, x0):
        self.τ = τ  # time constant
        self.x = x0  # initial state/guess

    def update(self, y, dt):
        """Update state estimate with measurement y taken dt seconds after the
        previous update."""
        # zero time-constant means no filtering is done
        if self.τ <= 0:
            return y

        # we can choose to initialize with the first measured value, before
        # which x0 is None
        if self.x is None:
            self.x = y
        else:
            c = 1.0 - np.exp(-dt / self.τ)
            self.x = c * y + (1 - c) * self.x
        return self.x
