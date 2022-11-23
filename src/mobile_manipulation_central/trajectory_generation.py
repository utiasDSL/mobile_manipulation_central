import numpy as np


# TODO: functional, but perhaps not useful
# class CompositeTrajectory:
#     def __init__(self, trajectories, t0=None):
#         self.trajectories = trajectories
#         self.durations = np.cumsum([traj.duration for traj in trajectories])
#
#         self.t0 = None
#         self.idx = 0
#         self._switched = False
#
#     @property
#     def duration(self):
#         return self.durations[-1]
#
#     def switched(self):
#         return self._switched
#
#     def done(self, t):
#         """True if the trajectory is done, False otherwise."""
#         if self.t0 is None:
#             return False
#         return t - self.t0 >= self.duration
#
#     def sample(self, t):
#         if self.t0 is None:
#             self.t0 = t
#         Δt = t - self.t0
#         self._switched = Δt > self.durations[self.idx]
#         while Δt > self.durations[self.idx]:
#             self.idx += 1
#         return self.trajectories[self.idx].sample(Δt)


class PointToPointTrajectory:
    """A straight-line trajectory between two points."""

    def __init__(self, start, goal, timescaling, t0=None):
        self.start = start
        self.goal = goal
        self.delta = (goal - start)[:, None]
        self.timescaling = timescaling
        self.t0 = t0

    @classmethod
    def quintic(cls, start, end, max_vel, max_acc, t0=None):
        """Build the trajectory with a quintic timescaling with duration
        suitable to satisfy velocity and acceleration constraints."""
        distance = np.linalg.norm(end - start)
        timescaling = QuinticTimeScaling.from_max_vel_acc(distance, max_vel, max_acc)
        return cls(start, end, timescaling, t0)

    @property
    def duration(self):
        """The duration of the trajectory."""
        return self.timescaling.duration

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
