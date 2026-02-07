from __future__ import annotations

"""Nonlinear equations of motion for drone + hanging weight system.

State vector (10 elements):
    [x_d, x_dot_d, y_d, y_dot_d, z_d, z_dot_d, phi_x, phi_dot_x, phi_y, phi_dot_y]

Control vector (3 elements):
    [F_x, F_y, F_z]
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Params:
    m_d: float = 2.0    # drone mass (kg)
    m_w: float = 5.0    # weight mass (kg)
    L: float = 4.0      # rope length (m)
    g: float = 9.81     # gravity (m/s^2)


def derivatives(state: np.ndarray, control: np.ndarray, params: Params) -> np.ndarray:
    """Compute state derivatives from nonlinear equations of motion.

    Uses Lagrangian-derived EOM for the coupled drone-pendulum system.
    The x-phi_x and y-phi_y subsystems are coupled through the pendulum,
    and the vertical axis is independent.
    """
    m_d = params.m_d
    m_w = params.m_w
    L = params.L
    g = params.g

    # Unpack state
    x_d, xd_dot, y_d, yd_dot, z_d, zd_dot, phi_x, phix_dot, phi_y, phiy_dot = state
    F_x, F_y, F_z = control

    # --- X-phi_x subsystem ---
    # (m_d + m_w)*x_dd + m_w*L*phi_x_dd*cos(phi_x) - m_w*L*phi_x_dot^2*sin(phi_x) = F_x
    # m_w*L*x_dd*cos(phi_x) + m_w*L^2*phi_x_dd + m_w*g*L*sin(phi_x) = 0
    #
    # Solve 2x2 system: M * [x_dd, phi_x_dd]^T = rhs
    cos_px = np.cos(phi_x)
    sin_px = np.sin(phi_x)

    M_x = np.array([
        [m_d + m_w,          m_w * L * cos_px],
        [m_w * L * cos_px,   m_w * L**2]
    ])
    rhs_x = np.array([
        F_x + m_w * L * phix_dot**2 * sin_px,
        -m_w * g * L * sin_px
    ])
    x_dd, phix_dd = np.linalg.solve(M_x, rhs_x)

    # --- Y-phi_y subsystem ---
    cos_py = np.cos(phi_y)
    sin_py = np.sin(phi_y)

    M_y = np.array([
        [m_d + m_w,          m_w * L * cos_py],
        [m_w * L * cos_py,   m_w * L**2]
    ])
    rhs_y = np.array([
        F_y + m_w * L * phiy_dot**2 * sin_py,
        -m_w * g * L * sin_py
    ])
    y_dd, phiy_dd = np.linalg.solve(M_y, rhs_y)

    # --- Vertical subsystem ---
    z_dd = (F_z - (m_d + m_w) * g) / (m_d + m_w)

    return np.array([
        xd_dot, x_dd,
        yd_dot, y_dd,
        zd_dot, z_dd,
        phix_dot, phix_dd,
        phiy_dot, phiy_dd
    ])


def rk4_step(state: np.ndarray, control: np.ndarray, params: Params, dt: float) -> np.ndarray:
    """Advance state by one RK4 integration step."""
    k1 = derivatives(state, control, params)
    k2 = derivatives(state + 0.5 * dt * k1, control, params)
    k3 = derivatives(state + 0.5 * dt * k2, control, params)
    k4 = derivatives(state + dt * k3, control, params)
    return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)


def weight_position(state: np.ndarray, L: float) -> tuple[float, float, float]:
    """Compute weight position from drone state.

    Returns (x_w, y_w, z_w).
    """
    x_d = state[0]
    y_d = state[2]
    z_d = state[4]
    phi_x = state[6]
    phi_y = state[8]

    x_w = x_d + L * np.sin(phi_x) * np.cos(phi_y)
    y_w = y_d + L * np.sin(phi_y)
    z_w = z_d - L * np.cos(phi_x) * np.cos(phi_y)

    return float(x_w), float(y_w), float(z_w)
