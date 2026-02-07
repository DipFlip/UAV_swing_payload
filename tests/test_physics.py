"""Tests for the physics module."""

import numpy as np
import pytest

from backend.physics import Params, derivatives, rk4_step, weight_position


@pytest.fixture
def params():
    return Params()


def test_equilibrium_derivatives(params):
    """At equilibrium (hover, no angles), derivatives should be zero except gravity cancellation."""
    state = np.zeros(10)
    state[4] = params.L  # z_d = L

    # Apply gravity-canceling thrust
    control = np.array([0.0, 0.0, (params.m_d + params.m_w) * params.g])
    deriv = derivatives(state, control, params)

    np.testing.assert_allclose(deriv, 0.0, atol=1e-10)


def test_free_fall_vertical(params):
    """With no thrust, system should free-fall at g."""
    state = np.zeros(10)
    state[4] = 10.0  # Start at z=10

    control = np.array([0.0, 0.0, 0.0])
    deriv = derivatives(state, control, params)

    # z acceleration should be -g
    assert deriv[5] == pytest.approx(-params.g, abs=1e-10)

    # No horizontal accelerations (no forces, no angles)
    assert deriv[1] == pytest.approx(0.0, abs=1e-10)
    assert deriv[3] == pytest.approx(0.0, abs=1e-10)


def test_pendulum_natural_frequency(params):
    """Small-angle pendulum frequency should match linearized model eigenvalues."""
    m_d, m_w, L, g = params.m_d, params.m_w, params.L, params.g

    # Get natural frequency from linearized A matrix eigenvalues
    A_lat = np.array([
        [0, 1, 0, 0],
        [0, 0, m_w * g / m_d, 0],
        [0, 0, 0, 1],
        [0, 0, -(m_d + m_w) * g / (m_d * L), 0]
    ])
    eigs = np.linalg.eigvals(A_lat)
    # The oscillatory eigenvalues are purely imaginary
    omega_n = max(abs(e.imag) for e in eigs)
    period = 2 * np.pi / omega_n

    # Start with small angle, hover thrust, measure oscillation
    state = np.zeros(10)
    state[4] = L
    state[6] = 0.01  # very small phi_x for linear regime

    control = np.array([0.0, 0.0, (m_d + m_w) * g])

    dt = 0.0005
    times = []
    angles = []

    for i in range(int(4 * period / dt)):
        state = rk4_step(state, control, params, dt)
        times.append((i + 1) * dt)
        angles.append(state[6])

    # Find zero crossings (positive to negative) to estimate period
    angles = np.array(angles)
    crossings = []
    for i in range(1, len(angles)):
        if angles[i - 1] > 0 and angles[i] <= 0:
            t_cross = times[i - 1] + (times[i] - times[i - 1]) * angles[i - 1] / (angles[i - 1] - angles[i])
            crossings.append(t_cross)

    assert len(crossings) >= 2, f"Not enough zero crossings found: {len(crossings)}"
    measured_period = crossings[1] - crossings[0]
    assert measured_period == pytest.approx(period, rel=0.05)


def test_weight_position_at_equilibrium(params):
    """Weight should be directly below drone at equilibrium."""
    state = np.zeros(10)
    state[4] = params.L  # z_d = L

    x_w, y_w, z_w = weight_position(state, params.L)

    assert x_w == pytest.approx(0.0, abs=1e-10)
    assert y_w == pytest.approx(0.0, abs=1e-10)
    assert z_w == pytest.approx(0.0, abs=1e-10)  # z_d - L*cos(0)*cos(0) = L - L = 0


def test_weight_position_with_angle(params):
    """Weight position should change with pendulum angles."""
    state = np.zeros(10)
    state[4] = params.L
    state[6] = np.pi / 6  # phi_x = 30 degrees

    x_w, y_w, z_w = weight_position(state, params.L)

    # x_w = L * sin(pi/6) * cos(0) = L * 0.5 = 2.0
    assert x_w == pytest.approx(params.L * 0.5, abs=1e-10)
    # z_w = L - L * cos(pi/6) * cos(0) = L * (1 - cos(pi/6))
    expected_z = params.L - params.L * np.cos(np.pi / 6)
    assert z_w == pytest.approx(expected_z, abs=1e-10)


def test_rk4_energy_conservation(params):
    """For free pendulum (no damping), energy should be approximately conserved."""
    m_d, m_w, L, g = params.m_d, params.m_w, params.L, params.g

    state = np.zeros(10)
    state[4] = L
    state[6] = 0.1  # Initial angle

    control = np.array([0.0, 0.0, (m_d + m_w) * g])

    def total_energy(s):
        xd_dot = s[1]
        phi_x = s[6]
        phix_dot = s[7]
        # Lagrangian-derived total energy for x-phi_x subsystem:
        # T = 0.5*(m_d+m_w)*xd_dot^2 + m_w*L*xd_dot*phix_dot*cos(phi_x) + 0.5*m_w*L^2*phix_dot^2
        # V = -m_w*g*L*cos(phi_x)  (potential, with zero at pivot)
        T = (0.5 * (m_d + m_w) * xd_dot**2
             + m_w * L * xd_dot * phix_dot * np.cos(phi_x)
             + 0.5 * m_w * L**2 * phix_dot**2)
        V = -m_w * g * L * np.cos(phi_x)
        return T + V

    E0 = total_energy(state)

    dt = 0.001
    for _ in range(5000):
        state = rk4_step(state, control, params, dt)

    E_final = total_energy(state)

    # RK4 should conserve energy well over short periods
    assert E_final == pytest.approx(E0, rel=0.01)
