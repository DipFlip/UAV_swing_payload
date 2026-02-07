from __future__ import annotations

"""Simulation manager for the drone + hanging weight system."""

import numpy as np

from .physics import Params, rk4_step, weight_position
from .controller import compute_lqr_gains, compute_control
from .pid_controller import PIDController


class Simulation:
    def __init__(self, params: Params | None = None, dt: float = 0.02,
                 controller_type: str = "lqr"):
        self.params = params or Params()
        self.dt = dt
        self.time = 0.0
        self.controller_type = controller_type

        # Initial state: drone hovering at z = L (so weight is at z = 0)
        self.state = np.zeros(10)
        self.state[4] = self.params.L  # z_d = L

        # Goal: weight at origin by default
        self.goal = np.array([0.0, 0.0, 0.0])

        # Last control for reporting
        self.last_control = np.array([0.0, 0.0, (self.params.m_d + self.params.m_w) * self.params.g])

        # Controllers
        self.K_lat, self.K_vert = compute_lqr_gains(self.params)
        self.pid = PIDController(self.params)

    def step(self) -> dict:
        """Advance simulation by one timestep and return state dict."""
        if self.controller_type == "pid":
            control = self.pid.compute_control(
                self.state, self.goal, self.dt
            )
        else:
            control = compute_control(
                self.state, self.goal, self.K_lat, self.K_vert, self.params
            )
        self.last_control = control

        self.state = rk4_step(self.state, control, self.params, self.dt)
        self.time += self.dt

        x_w, y_w, z_w = weight_position(self.state, self.params.L)

        return {
            "type": "state",
            "time": round(self.time, 4),
            "drone": {
                "x": round(float(self.state[0]), 4),
                "y": round(float(self.state[2]), 4),
                "z": round(float(self.state[4]), 4),
            },
            "weight": {
                "x": round(x_w, 4),
                "y": round(y_w, 4),
                "z": round(z_w, 4),
            },
            "phi_x": round(float(self.state[6]), 4),
            "phi_y": round(float(self.state[8]), 4),
            "goal": {
                "x": round(float(self.goal[0]), 4),
                "y": round(float(self.goal[1]), 4),
                "z": round(float(self.goal[2]), 4),
            },
            "control": {
                "Fx": round(float(control[0]), 2),
                "Fy": round(float(control[1]), 2),
                "Fz": round(float(control[2]), 2),
            },
        }

    def set_goal(self, x: float, y: float, z: float):
        """Set target weight position."""
        self.goal = np.array([x, y, z])

    def set_aggression(self, aggr: float):
        """Set controller aggressiveness (0.05 to 1.0).

        Scales LQR Q weights and PID gains proportionally.
        """
        aggr = max(0.05, min(1.0, aggr))
        # LQR: scale Q weights (higher = more aggressive)
        Q_lat = np.diag([16.0 * aggr, 4.0 * aggr, 20.0 * aggr, 5.0 * aggr])
        R_lat = np.array([[0.3 / aggr]])
        Q_vert = np.diag([60.0 * aggr, 20.0 * aggr])
        R_vert = np.array([[0.3 / aggr]])
        self.K_lat, self.K_vert = compute_lqr_gains(
            self.params, Q_lat, R_lat, Q_vert, R_vert
        )
        # PID: scale gains
        self.pid.pid_x.kp = 8.0 * aggr
        self.pid.pid_x.ki = 1.5 * aggr
        self.pid.pid_x.kd = 10.0 * aggr
        self.pid.pid_y.kp = 8.0 * aggr
        self.pid.pid_y.ki = 1.5 * aggr
        self.pid.pid_y.kd = 10.0 * aggr
        self.pid.pid_z.kp = 20.0 * aggr
        self.pid.pid_z.ki = 4.0 * aggr
        self.pid.pid_z.kd = 15.0 * aggr

    def reset(self):
        """Reset to initial hover state."""
        self.state = np.zeros(10)
        self.state[4] = self.params.L
        self.goal = np.array([0.0, 0.0, 0.0])
        self.time = 0.0
        self.last_control = np.array([0.0, 0.0, (self.params.m_d + self.params.m_w) * self.params.g])
        self.pid.reset()
