import pytest
import numpy as np

import mobile_manipulation_central as mm


def test_quintic_trajectory():
    start = np.array([0])
    delta = np.array([1])
    traj = mm.PointToPointTrajectory.quintic(
        start, delta, max_vel=1.0, max_acc=1.0, t0=0
    )

    # one of these limits should be tight
    assert np.isclose(traj.max_velocity, 1.0) or np.isclose(
        traj.max_acceleration, 1.0
    )

    # start
    p, v, a = traj.sample(0)
    assert np.isclose(p, 0)
    assert np.isclose(v, 0)
    assert np.isclose(a, 0)

    # middle
    p, v, a = traj.sample(0.5 * traj.duration)
    assert np.isclose(p, start + 0.5 * delta)
    assert np.isclose(v, traj.max_velocity)
    assert np.isclose(a, 0)

    # end
    p, v, a = traj.sample(traj.duration)
    assert np.isclose(p, start + delta)
    assert np.isclose(v, 0)
    assert np.isclose(a, 0)

    # this continues after the end of the trajectory
    p, v, a = traj.sample(traj.duration + 1)
    assert np.isclose(p, start + delta)
    assert np.isclose(v, 0)
    assert np.isclose(a, 0)

    assert traj.done(traj.duration + 1)
    assert not traj.done(0.5 * traj.duration)


def test_quintic_trajectory_array():
    start = np.array([0, 0])
    delta = np.array([1, 2])
    traj = mm.PointToPointTrajectory.quintic(
        start, delta, max_vel=1.0, max_acc=1.0, t0=0
    )
    v_max = traj.max_velocity
    a_max = traj.max_acceleration
    assert np.isclose(v_max[0], 0.5 * v_max[1])
    assert np.isclose(a_max[0], 0.5 * a_max[1])


def test_quintic_timescaling():
    duration = 10.0
    scaling = mm.QuinticTimeScaling(duration)

    # start
    s, ds, dds = scaling.eval(0)
    assert np.isclose(s, 0)
    assert np.isclose(ds, 0)
    assert np.isclose(dds, 0)

    # middle
    s = scaling.eval(0.5 * duration)[0]
    assert np.isclose(s, 0.5)

    # end
    s, ds, dds = scaling.eval(duration)
    assert np.isclose(s, 1.0)
    assert np.isclose(ds, 0)
    assert np.isclose(dds, 0)

    # beyond the bounds
    with pytest.raises(AssertionError):
        scaling.eval(-1)
    with pytest.raises(AssertionError):
        scaling.eval(duration + 1)

    # velocity scaling is highest in the middle
    ds0 = scaling.eval(0.5 * duration)[1]
    ds1 = scaling.eval(0.49 * duration)[1]
    ds2 = scaling.eval(0.51 * duration)[1]
    assert ds0 >= ds1
    assert ds0 >= ds2
