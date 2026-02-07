from __future__ import annotations

"""LQR controller for the drone + hanging weight system."""

import numpy as np
from scipy.linalg import solve_continuous_are

from .physics import Params


def compute_lqr_gains(
    params: Params,
    Q_lat: np.ndarray | None = None,
    R_lat: np.ndarray | None = None,
    Q_vert: np.ndarray | None = None,
    R_vert: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute LQR gains for the linearized system.

    Returns:
        K_lat: (1, 4) gain matrix for each lateral subsystem (x-phi_x and y-phi_y)
        K_vert: (1, 2) gain matrix for the vertical subsystem
    """
    m_d = params.m_d
    m_w = params.m_w
    L = params.L
    g = params.g

    # --- Lateral subsystem (x-phi_x or y-phi_y) ---
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

    if Q_lat is None:
        Q_lat = np.diag([16.0, 4.0, 20.0, 5.0])
    if R_lat is None:
        R_lat = np.array([[0.3]])

    P_lat = solve_continuous_are(A_lat, B_lat, Q_lat, R_lat)
    K_lat = np.linalg.solve(R_lat, B_lat.T @ P_lat)

    # --- Vertical subsystem ---
    A_vert = np.array([
        [0, 1],
        [0, 0]
    ])
    B_vert = np.array([
        [0],
        [1 / (m_d + m_w)]
    ])

    if Q_vert is None:
        Q_vert = np.diag([60.0, 20.0])
    if R_vert is None:
        R_vert = np.array([[0.3]])

    P_vert = solve_continuous_are(A_vert, B_vert, Q_vert, R_vert)
    K_vert = np.linalg.solve(R_vert, B_vert.T @ P_vert)

    return K_lat, K_vert


def compute_control(
    state: np.ndarray,
    goal: np.ndarray,
    K_lat: np.ndarray,
    K_vert: np.ndarray,
    params: Params,
    max_lateral: float = 40.0,
    max_thrust: float = 100.0,
) -> np.ndarray:
    """Compute control forces given current state and goal weight position.

    Args:
        state: 10-element state vector
        goal: (x_w_goal, y_w_goal, z_w_goal) desired weight position
        K_lat: lateral LQR gain (1, 4)
        K_vert: vertical LQR gain (1, 2)
        params: system parameters
        max_lateral: lateral force saturation limit per axis (N)
        max_thrust: total vertical force limit (N)

    Returns:
        control: [F_x, F_y, F_z]
    """
    # Goal mapping: desired drone position is above the desired weight position
    x_d_des = goal[0]
    y_d_des = goal[1]
    z_d_des = goal[2] + params.L

    # --- X-axis control ---
    x_state = np.array([state[0], state[1], state[6], state[7]])  # x_d, xd_dot, phi_x, phix_dot
    x_desired = np.array([x_d_des, 0.0, 0.0, 0.0])
    F_x = (-K_lat @ (x_state - x_desired)).item()

    # --- Y-axis control ---
    y_state = np.array([state[2], state[3], state[8], state[9]])  # y_d, yd_dot, phi_y, phiy_dot
    y_desired = np.array([y_d_des, 0.0, 0.0, 0.0])
    F_y = (-K_lat @ (y_state - y_desired)).item()

    # --- Z-axis control ---
    z_state = np.array([state[4], state[5]])  # z_d, zd_dot
    z_desired = np.array([z_d_des, 0.0])
    F_z = (-K_vert @ (z_state - z_desired)).item()

    # Gravity feedforward
    F_z += (params.m_d + params.m_w) * params.g

    # Force saturation
    F_x = np.clip(F_x, -max_lateral, max_lateral)
    F_y = np.clip(F_y, -max_lateral, max_lateral)
    F_z = np.clip(F_z, 0.0, max_thrust)

    return np.array([F_x, F_y, F_z])
