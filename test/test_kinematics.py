import pytest
import numpy as np

import mobile_manipulation_central as mm


def test_velocity_mapping():
    """Test mapping velocity between joint and task space."""
    np.random.seed(0)

    model = mm.MobileManipulatorKinematics()
    q = np.array([-1.0, 1.0, 0, 1.5708, -0.7854,  1.5708, -0.7854,  1.5708, 1.3100])
    v = np.random.random(model.nv)

    # map joint space velocity through the Jacobian
    J = model.jacobian(q)
    V_pred = J @ v

    # compute task velocity directly
    model.forward(q, v)
    V = np.concatenate((model.link_velocity()))

    assert np.allclose(V_pred, V)


def test_acceleration_mapping():
    """Test mapping acceleration between joint and task space."""
    np.random.seed(0)

    model = mm.MobileManipulatorKinematics()
    q = np.array([-1.0, 1.0, 0, 1.5708, -0.7854,  1.5708, -0.7854,  1.5708, 1.3100])
    v = np.random.random(model.nv)
    a = np.random.random(model.nv)

    J = model.jacobian(q)

    # compute the drift/bias term by passing in zero acceleration
    # this is easier than computing the Jacobian time derivative
    # see <https://github.com/stack-of-tasks/pinocchio/issues/1395>
    model.forward(q, v, np.zeros(model.nv))
    A_drift = np.concatenate((model.link_classical_acceleration()))
    A_pred = A_drift + J @ a

    # compute task acceleration directly
    model.forward(q, v, a)
    A = np.concatenate((model.link_classical_acceleration()))

    assert np.allclose(A_pred, A)
