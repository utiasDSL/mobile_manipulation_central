import numpy as np

import mobile_manipulation_central as mm


def test_exponential_smoother():

    # when τ=0, no smoothing is done
    smoother = mm.ExponentialSmoother(τ=0, x0=0)
    x = smoother.update(y=1, dt=1)
    assert np.isclose(x, 1)

    # with time constant τ=1 and a unit step, we should reach 1 - 1 / e after
    # 1 second
    target = 1 - 1 / np.exp(1)

    # jump immediately to 1 second
    smoother = mm.ExponentialSmoother(τ=1, x0=0)
    x = smoother.update(y=1, dt=1)
    assert np.isclose(x, target)

    # or sample a lot in between
    smoother = mm.ExponentialSmoother(τ=1, x0=0)
    for _ in range(1000):
        x = smoother.update(y=1, dt=0.001)
    assert np.isclose(x, target)
