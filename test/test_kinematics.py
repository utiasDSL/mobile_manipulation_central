import pytest
import numpy as np
from scipy.linalg import block_diag

import upright_core as core
import mobile_manipulation_central as mm


def adj(C=None, r=None):
    """Adjoint of SE(3)."""
    if C is None:
        C = np.eye(3)
    if r is None:
        r = np.zeros(3)
    Z = np.zeros((3, 3))
    R = core.math.skew3(r)
    return np.block([[C, R], [Z, C]])


def test_velocity_frames():
    """Testing mapping between local, local-world-aligned, and world velocity representations

    This is intended mostly as a reference to remember the relationships.
    """
    np.random.seed(0)

    model = mm.MobileManipulatorKinematics()
    q = np.array([-1.0, 1.0, 0, 1.5708, -0.7854, 1.5708, -0.7854, 1.5708, 1.3100])
    v = np.random.random(model.nv)

    model.forward(q, v)
    r_ew_w, C_we = model.link_pose(rotation_matrix=True)
    V = np.concatenate(model.link_velocity())
    V_local = np.concatenate(model.link_velocity(frame="local"))
    V_local_world_aligned = np.concatenate(
        model.link_velocity(frame="local_world_aligned")
    )
    V_world = np.concatenate(model.link_velocity(frame="world"))

    # check default frame is local-world-aligned
    assert np.allclose(V, V_local_world_aligned)

    # check relationshops between representations
    assert np.allclose(V_local, adj(C=C_we.T) @ V_local_world_aligned)
    assert np.allclose(V_world, adj(r=r_ew_w) @ V_local_world_aligned)


def test_acceleration_frames():
    """Testing mapping between local, local-world-aligned, and world accelerations representations

    This is intended mostly as a reference to remember the relationships.
    """
    np.random.seed(0)

    model = mm.MobileManipulatorKinematics()
    q = np.array([-1.0, 1.0, 0, 1.5708, -0.7854, 1.5708, -0.7854, 1.5708, 1.3100])
    v = np.random.random(model.nv)
    a = np.random.random(model.nv)

    model.forward(q, v, a)

    r_ew_w, C_we = model.link_pose(rotation_matrix=True)
    v_ew_w, ω_ew_w = model.link_velocity()

    A_local = np.concatenate((model.link_classical_acceleration(frame="local")))
    A_local_world_aligned = np.concatenate(
        (model.link_classical_acceleration(frame="local_world_aligned"))
    )
    A_world = np.concatenate((model.link_classical_acceleration(frame="world")))

    W = core.math.skew3(ω_ew_w)
    A_world_pred = adj(r=r_ew_w) @ A_local_world_aligned - np.concatenate(
        (W @ W @ r_ew_w, np.zeros(3))
    )

    assert np.allclose(A_local, adj(C=C_we.T) @ A_local_world_aligned)
    assert np.allclose(A_world, A_world_pred)


def test_velocity_mapping():
    """Test mapping velocity between joint and task space."""
    np.random.seed(0)

    model = mm.MobileManipulatorKinematics()
    q = np.array([-1.0, 1.0, 0, 1.5708, -0.7854, 1.5708, -0.7854, 1.5708, 1.3100])
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
    q = np.array([-1.0, 1.0, 0, 1.5708, -0.7854, 1.5708, -0.7854, 1.5708, 1.3100])
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
