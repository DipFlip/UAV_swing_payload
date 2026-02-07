"""PID controller for the drone + hanging weight system.

This is a naive position-based PID that drives the drone to hover above
the goal. It has NO awareness of the pendulum dynamics, so the weight
will swing significantly compared to the LQR controller. This makes it
a useful baseline for comparison.
"""

import numpy as np

from .physics import Params


class PIDAxis:
    """Single-axis PID with derivative filtering and integral windup limit."""

    def __init__(self, kp: float, ki: float, kd: float, windup: float = 50.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.windup = windup
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error: float, dt: float) -> float:
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.windup, self.windup)

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class PIDController:
    """3-axis PID controller for the drone.

    Controls drone position to hover above the target weight position.
    No pendulum dynamics awareness — purely reactive position control.
    """

    def __init__(self, params: Params):
        self.params = params
        # Tuned to be reasonably aggressive but not unstable
        # Lateral: moderate gains — fast enough to be interesting
        self.pid_x = PIDAxis(kp=8.0, ki=1.5, kd=10.0)
        self.pid_y = PIDAxis(kp=8.0, ki=1.5, kd=10.0)
        # Vertical: higher gains since no coupling
        self.pid_z = PIDAxis(kp=20.0, ki=4.0, kd=15.0)

    def compute_control(
        self, state: np.ndarray, goal: np.ndarray, dt: float,
        max_lateral: float = 40.0, max_thrust: float = 100.0,
    ) -> np.ndarray:
        # Desired drone position: directly above goal weight position
        x_d_des = goal[0]
        y_d_des = goal[1]
        z_d_des = goal[2] + self.params.L

        # Position errors
        ex = x_d_des - state[0]
        ey = y_d_des - state[2]
        ez = z_d_des - state[4]

        F_x = self.pid_x.step(ex, dt)
        F_y = self.pid_y.step(ey, dt)
        F_z = self.pid_z.step(ez, dt)

        # Gravity feedforward
        F_z += (self.params.m_d + self.params.m_w) * self.params.g

        # Saturation
        F_x = np.clip(F_x, -max_lateral, max_lateral)
        F_y = np.clip(F_y, -max_lateral, max_lateral)
        F_z = np.clip(F_z, 0.0, max_thrust)

        return np.array([F_x, F_y, F_z])

    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
