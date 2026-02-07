"""Tests for the LQR controller."""

import numpy as np
import pytest

from backend.physics import Params
from backend.controller import compute_lqr_gains, compute_control


@pytest.fixture
def params():
    return Params()


@pytest.fixture
def gains(params):
    return compute_lqr_gains(params)


def test_gain_dimensions(gains):
    """LQR gains should have correct shapes."""
    K_lat, K_vert = gains
    assert K_lat.shape == (1, 4)
    assert K_vert.shape == (1, 2)


def test_closed_loop_stability(params, gains):
    """Closed-loop eigenvalues should all have negative real parts."""
    K_lat, K_vert = gains
    m_d, m_w, L, g = params.m_d, params.m_w, params.L, params.g

    # Lateral subsystem
    A_lat = np.array([
        [0, 1, 0, 0],
        [0, 0, m_w * g / m_d, 0],
        [0, 0, 0, 1],
        [0, 0, -(m_d + m_w) * g / (m_d * L), 0]
    ])
    B_lat = np.array([
        [0],
        [1 / m_d],
        [0],
        [-1 / (m_d * L)]
    ])

    A_cl_lat = A_lat - B_lat @ K_lat
    eigs_lat = np.linalg.eigvals(A_cl_lat)
    assert all(e.real < 0 for e in eigs_lat), f"Unstable lateral eigenvalues: {eigs_lat}"

    # Vertical subsystem
    A_vert = np.array([[0, 1], [0, 0]])
    B_vert = np.array([[0], [1 / (m_d + m_w)]])

    A_cl_vert = A_vert - B_vert @ K_vert
    eigs_vert = np.linalg.eigvals(A_cl_vert)
    assert all(e.real < 0 for e in eigs_vert), f"Unstable vertical eigenvalues: {eigs_vert}"


def test_gravity_feedforward(params, gains):
    """At equilibrium hovering over goal, Fz should equal total weight."""
    K_lat, K_vert = gains

    state = np.zeros(10)
    state[4] = params.L  # z_d = L (weight at z=0)
    goal = np.array([0.0, 0.0, 0.0])

    control = compute_control(state, goal, K_lat, K_vert, params)

    # Fx, Fy should be ~0
    assert control[0] == pytest.approx(0.0, abs=1e-10)
    assert control[1] == pytest.approx(0.0, abs=1e-10)
    # Fz should equal (m_d + m_w) * g
    assert control[2] == pytest.approx((params.m_d + params.m_w) * params.g, abs=1e-10)


def test_control_force_direction(params, gains):
    """Control should push drone toward goal when displaced."""
    K_lat, K_vert = gains

    # Drone at origin, goal weight at x=5 -> desired drone x=5
    state = np.zeros(10)
    state[4] = params.L
    goal = np.array([5.0, 0.0, 0.0])

    control = compute_control(state, goal, K_lat, K_vert, params)

    # Should push in +x direction
    assert control[0] > 0


def test_force_saturation(params, gains):
    """Forces should be clamped to limits."""
    K_lat, K_vert = gains

    state = np.zeros(10)
    state[4] = params.L
    goal = np.array([100.0, -100.0, 50.0])  # Very far goal

    control = compute_control(state, goal, K_lat, K_vert, params,
                              max_lateral=20.0, max_thrust=80.0)

    assert abs(control[0]) <= 20.0
    assert abs(control[1]) <= 20.0
    assert control[2] <= 80.0
    assert control[2] >= 0.0


def test_gains_positive(gains):
    """All gain elements should be positive (for this system)."""
    K_lat, K_vert = gains
    # Position and velocity gains should be positive
    assert K_lat[0, 0] > 0  # position gain
    assert K_lat[0, 1] > 0  # velocity gain
