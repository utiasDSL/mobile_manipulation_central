import numpy as np


class ExponentialSmoother:
    """Exponential smoothing filter.

    See https://en.wikipedia.org/wiki/Exponential_smoothing.

    Parameters
    ----------
    τ : float
        The filter time constant. Must be non-negative.
    x0 :
        The initial value. Can be ``None``, in which case the first value
        passed to ``self.update`` will be used.

    Attributes
    ----------
    τ : float
        The filter time constant.
    x :
        The current estimate.

    Raises
    ------
    AssertionError
        If ``τ`` is negative.
    """

    def __init__(self, τ, x0):
        assert τ >= 0
        self.τ = τ  # time constant
        self.x = x0  # initial state/guess

    def update(self, y, dt):
        """Update the estimate.

        Parameters
        ----------
        y :
            A new measured value.
        dt :
            The time passed since the previous measured value was obtained.

        Returns
        -------
        :
            The updated estimate.
        """
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
