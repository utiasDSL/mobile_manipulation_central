"""PointToPointTrajectory trajectory and polynominal time-scalings."""

import numpy as np


class PointToPointTrajectory:
    """A straight-line trajectory between two points.

    Parameters
    ----------
    start :
        The start point of the trajectory
    delta :
        The position to travel to relative to start. In absolute terms, the
        goal point is ``start + delta``. We explicitly use ``delta`` here to
        allow the caller to handle e.g. wrapping values to π for angles.
    timescaling :
        The timescaling to use.
    t0 : float
        Optionally provide the start time of the trajectory. If not provided,
        the first sample time will be taken as ``t0``.
    """

    def __init__(self, start, delta, timescaling, t0=None):
        self.start = start
        self.delta = delta.reshape((delta.shape[0], 1))
        self.goal = start + delta
        self.timescaling = timescaling
        self.t0 = t0

    @classmethod
    def quintic(cls, start, delta, max_vel, max_acc, t0=None, min_duration=None):
        """Construct the trajectory using a quintic timescaling with duration
        suitable to satisfy velocity and acceleration constraints.

        Parameters
        ----------
        start :
            The start point of the trajectory
        delta :
            The position to travel to relative to start. In absolute terms, the
            goal point is ``start + delta``.
        max_vel : float
            Maximum allowable velocity.
        max_acc : float
            Maximum allowable acceleration.
        t0 : float
            Optionally provide the start time of the trajectory. If not provided,
            the first sample time will be taken as ``t0``.
        min_duration : float
            Optionally provide a minimum duration for the trajectory. If the
            timescaling obeying the maximum velocity and acceleration limits
            has a lower duration, we replace the timescaling with one of
            ``min_duration``.
        """
        distance = np.max(np.abs(delta))
        timescaling = QuinticTimeScaling.from_max_vel_acc(distance, max_vel, max_acc)
        if min_duration is not None and timescaling.duration < min_duration:
            timescaling = QuinticTimeScaling(min_duration)
        return cls(start, delta, timescaling, t0)

    @property
    def duration(self):
        """The duration of the trajectory."""
        return self.timescaling.duration

    @property
    def max_velocity(self):
        """The maximum velocity of the trajectory.

        This value is always positive, even if the true velocity is negative.
        """
        return np.abs(np.squeeze(self.timescaling.max_ds * self.delta))

    @property
    def max_acceleration(self):
        """The maximum acceleration of the trajectory.

        This value is always positive, even if the true acceleration is
        negative.
        """
        return np.abs(np.squeeze(self.timescaling.max_dds * self.delta))

    def done(self, t):
        """Check if the trajectory is done at the given time.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        :
            ``True`` if the trajectory is done, ``False`` otherwise. Also
            returns ``False`` if no initial time ``t0`` was supplied and the
            trajectory has not yet been sampled.
        """
        if self.t0 is None:
            return False
        return t - self.t0 >= self.duration

    def sample(self, t):
        """Sample the trajectory at the given time.

        If ``t0`` was not supplied when the trajectory was constructed, then
        ``t0`` will be set to the first ``t`` at which it is sampled.

        If ``t`` is beyond the end of the trajectory, then the final point of
        the trajectory is returned; i.e., all calls to ``self.sample`` with
        ``t >= self.duration`` return the same values.

        Parameters
        ----------
        t : float
            The sample time.

        Returns
        -------
        :
            A tuple (position, velocity, acceleration) for the sample time.
        """
        if self.t0 is None:
            self.t0 = t

        if self.done(t):
            return self.goal, np.zeros_like(self.goal), np.zeros_like(self.goal)

        s, ds, dds = self.timescaling.eval(t - self.t0)
        position = self.start + (s * self.delta).T
        velocity = (ds * self.delta).T
        acceleration = (dds * self.delta).T
        if np.ndim(t) == 0:
            position = np.squeeze(position)
            velocity = np.squeeze(velocity)
            acceleration = np.squeeze(acceleration)
        return position, velocity, acceleration


class QuinticTimeScaling:
    """Quintic time-scaling with zero velocity and acceleration at end points.

    Parameters
    ----------
    duration : float
        Non-negative duration of the trajectory.

    Attributes
    ----------
    coeffs :
        The coefficients of the time-scaling equation.
    max_ds : float
        The maximum value of the first time derivative.
    max_dds : float
        The maximum value of the second time derivative.

    Raises
    ------
    AssertionError
        If the ``duration`` is negative.
    """

    def __init__(self, duration):
        assert duration >= 0
        self.duration = duration
        T = duration
        A = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [1, T, T**2, T**3, T**4, T**5],
                [0, 1, 0, 0, 0, 0],
                [0, 1, 2 * T, 3 * T**2, 4 * T**3, 5 * T**4],
                [0, 0, 2, 0, 0, 0],
                [0, 0, 2, 6 * T, 12 * T**2, 20 * T**3],
            ]
        )
        b = np.array([0, 1, 0, 0, 0, 0])
        self.coeffs = np.linalg.solve(A, b)

        self.max_ds = 15 / (8 * T)
        self.max_dds = 10 / (T**2 * np.sqrt(3))

    @classmethod
    def from_max_vel_acc(cls, distance, max_vel, max_acc):
        """Make a timescaling with long enough duration to satisfy bounds on
        velocity and acceleration.

        Parameters
        ----------
        distance : float
            The maximum distance to be travelled.
        max_vel : float
            The maximum allowable velocity.
        max_acc : float
            The maximum allowable acceleration.

        Raises
        ------
        AssertionError
            If ``distance``, ``max_vel``, or ``max_acc`` are negative.
        """
        assert distance >= 0
        assert max_vel >= 0
        assert max_acc >= 0

        max_ds = max_vel / distance
        max_dds = max_acc / distance
        T = max(15 / (max_ds * 8), np.sqrt(10 / (np.sqrt(3) * max_dds)))
        return cls(T)

    def eval(self, t):
        """Evaluate the timescaling at a given time.

        Parameters
        ----------
        t : float
            Time, must be in the interval ``[0, self.duration]``.

        Returns
        -------
        :
            A tuple (s, ds, dds) representing the value of the time-scaling
            and its first two derivatives at the time ``t``.

        Raises
        ------
        AssertionError
            If ``t`` is not in the interval ``[0, self.duration]``.
        """
        assert 0 <= t <= self.duration
        s = self.coeffs.dot([np.ones_like(t), t, t**2, t**3, t**4, t**5])
        ds = self.coeffs[1:].dot(
            [np.ones_like(t), 2 * t, 3 * t**2, 4 * t**3, 5 * t**4]
        )
        dds = self.coeffs[2:].dot(
            [2 * np.ones_like(t), 6 * t, 12 * t**2, 20 * t**3]
        )
        return s, ds, dds
