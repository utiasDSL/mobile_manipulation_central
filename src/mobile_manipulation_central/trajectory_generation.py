import numpy as np


class PointToPointTrajectory:
    """A straight-line trajectory between two points."""

    def __init__(self, start, delta, timescaling, t0=None):
        self.start = start
        # self.goal = goal
        # self.delta = (goal - start)[:, None]
        self.delta = delta.reshape((delta.shape[0], 1))
        self.goal = start + delta
        self.timescaling = timescaling
        self.t0 = t0

    @classmethod
    def quintic(cls, start, delta, max_vel, max_acc, t0=None, min_duration=None):
        """Build the trajectory with a quintic timescaling with duration
        suitable to satisfy velocity and acceleration constraints.

        Parameters:
            start: the start position of the trajectory
            delta: change from the start position, such that the end of the
                trajectory equals `start + delta`
            max_vel: maximum allowable velocity
            max_acc: maximum allowable acceleration
            t0: (Optional) start time of the trajectory. If not provided, this
                will be set when the trajectory is first sampled.
            min_duration: (Optional) minimum duration of the trajectory. If the
                timescaling obeying the maximum velocity and acceleration
                limits has a lower duration, we replace the timescaling with
                one of `min_duration`.
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
        # TODO
        pass

    @property
    def max_acceleration(self):
        # TODO
        pass

    def done(self, t):
        """True if the trajectory is done, False otherwise."""
        if self.t0 is None:
            return False
        return t - self.t0 >= self.duration

    def sample(self, t):
        """Sample the trajectory at time t."""
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
    """Quintic time-scaling: zero velocity and acceleration at end points."""

    def __init__(self, duration):
        self.duration = duration
        T = duration
        A = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [1, T, T ** 2, T ** 3, T ** 4, T ** 5],
                [0, 1, 0, 0, 0, 0],
                [0, 1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                [0, 0, 2, 0, 0, 0],
                [0, 0, 2, 6 * T, 12 * T ** 2, 20 * T ** 3],
            ]
        )
        b = np.array([0, 1, 0, 0, 0, 0])
        self.coeffs = np.linalg.solve(A, b)

    @classmethod
    def from_max_vel_acc(cls, distance, max_vel, max_acc):
        """Make a timescaling with long enough to satisfy bounds on velocity and acceleration."""
        max_ds = max_vel / distance
        max_dds = max_acc / distance
        T = max(15 / (max_ds * 8), np.sqrt(10 / (np.sqrt(3) * max_dds)))
        return cls(T)

    def eval(self, t):
        """Evaluate the timescaling at time t, 0 <= t <= duration."""
        assert 0 <= t <= self.duration
        s = self.coeffs.dot([np.ones_like(t), t, t ** 2, t ** 3, t ** 4, t ** 5])
        ds = self.coeffs[1:].dot(
            [np.ones_like(t), 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4]
        )
        dds = self.coeffs[2:].dot(
            [2 * np.ones_like(t), 6 * t, 12 * t ** 2, 20 * t ** 3]
        )
        return s, ds, dds
