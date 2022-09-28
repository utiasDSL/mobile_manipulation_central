from functools import partial

import numpy as np
from scipy import signal, optimize

# system identification routines adapted from
# https://medium.com/robotics-devs/system-identification-with-python-2079088b4d03
def simulate_second_order_system(ts, ωn, ζ, us, k=1):
    """Simulate a second-order system with parameters (k, ωn, ζ) and inputs us at times ts."""
    sys = signal.TransferFunction(k * (ωn ** 2), [1, 2 * ζ * ωn, ωn ** 2])
    _, ys, _ = signal.lsim2(sys, U=us, T=ts)
    return ys


def simulate_first_order_system(ts, τ, us, k=1):
    """Simulate a second-order system with parameters (k, ωn, ζ) and inputs us at times ts."""
    sys = signal.TransferFunction(k, [τ, 1])
    _, ys, _ = signal.lsim2(sys, U=us, T=ts)
    return ys


def identify_first_order_system(ts, us, ys, method="trf", p0=[1.0]):
    """Fit a second-order model to the inputs us and outputs ys at times ts."""
    # bounds: assume system is not overdamped
    bounds = ([0], [np.inf])
    model = partial(simulate_first_order_system, us=us)
    (τ,), covariance = optimize.curve_fit(
        model,
        ts,
        ys,
        method=method,
        p0=p0,
        bounds=bounds,
    )
    return τ


def identify_second_order_system(ts, us, ys, step, method="trf", p0=(10.0, 0.1, 0)):
    """Fit a second-order model to the inputs us and outputs ys at times ts."""
    # bounds: assume system is not overdamped
    bounds = ([0, 0, 0], [np.inf, 1.0, np.inf])

    def model(ts, ωn, ζ, td):
        us2 = step.sample(ts, td=td)
        ys_sim = simulate_second_order_system(ts, ωn, ζ, us2)
        return ys_sim

    (ωn, ζ, td), covariance = optimize.curve_fit(
        model,
        ts,
        ys,
        method=method,
        p0=p0,
        bounds=bounds,
        # sigma=0.1 * np.ones_like(ys),
    )
    return ωn, ζ, td


