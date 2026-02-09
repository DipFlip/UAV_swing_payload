# Future Control Algorithm Options

Notes on control algorithms for the drone + hanging payload system.

## Currently Implemented
- **LQR** (Linear Quadratic Regulator) — Linearized full-state feedback using all 10 state variables including pendulum angles. Solves CARE for optimal gains.
- **PID** — 3-axis position control blind to pendulum dynamics. Uses derivative-on-measurement to avoid derivative kick.
- **Cascade PD** — Two nested PD loops: outer on payload position, inner on drone position.
- **Flatness FF** — Differential flatness feedforward with reference trajectory filter + PD feedback.
- **Feedback Linearization** — Cancels nonlinear sin/cos coupling, applies PD on linearized system.
- **Sliding Mode** — Sliding surface + equivalent control + bounded switching term. Robust to model uncertainty.
- **MPC** — Finite-horizon LQR via backward discrete Riccati recursion. Gain caching for performance.
- **Energy-Based Swing Damping** (layerable) — Adds damping force proportional to pendulum angular velocity.
- **ZVD Input Shaping** (layerable) — Pre-filters goal signal with 3 impulses at pendulum natural frequency for zero residual swing.

## Reactive Control (no trajectory preview)

### Cascade (Chained) PD
Two nested control loops:
- **Outer loop**: PD on payload position error -> outputs a desired drone offset
- **Inner loop**: PD on drone position -> outputs force commands

Intuition: if the payload is too far right, command the drone to move further right to "pull" it back. Standard approach in overhead crane control. Unlike PID (blind to pendulum), it explicitly uses payload position feedback.

### Energy-Based Swing Damping (layerable)
Add a force component that dissipates pendulum energy. Detects when payload is swinging and applies force proportional to `phi_dot` in the direction that damps it:
```
F_damp_x = k_e * phi_dot_x
F_damp_y = k_e * phi_dot_y
```
Where `k_e` is a positive gain. The force "chases" the swing to damp it. Can be layered on top of any position controller. From energy analysis: `dE/dt = -m_w * L * cos(phi) * phi_dot * x_d_ddot`, so `x_d_ddot` must have the same sign as `phi_dot` for energy dissipation.

### Feedback Linearization
Apply a state-dependent coordinate transform that cancels the sin/cos nonlinearity, making the system behave like a double integrator. Then apply linear control (PD/LQR) on the transformed system. Works well for moderate angles but can demand large forces near singularities.

### Sliding Mode Control
Define a sliding surface combining position error and swing, then apply discontinuous control to force the state onto that surface. Very robust to model uncertainty (wrong mass, rope length, etc.) but produces chattery forces unless you add a boundary layer.

## Known Trajectory / Waypoints

### Differential Flatness + Feedforward
The drone-pendulum system is differentially flat with **payload position as the flat output**. Given a smooth desired payload trajectory, we can algebraically compute the exact drone position and forces needed.

From linearized dynamics:
- `phi = -x_w_ddot / g` (required pendulum angle)
- `x_d = x_w + (L/g) * x_w_ddot` (required drone position)
- `F_x = (m_d + m_w) * x_w_ddot + m_d * L / g * x_w_4dot` (feedforward force)

Implementation: pass the goal through a 2nd-order critically-damped reference filter to generate smooth trajectories with analytic derivatives, then compute feedforward + PD feedback.

### Input Shaping (ZVD Shaper)
Reshape the reference signal by convolving with impulses timed at half-periods of the pendulum's natural frequency `(2*pi*sqrt(L/g))`. Produces zero residual swing at endpoints. Works with any underlying position controller. Simple FIR filter on the reference.

### MPC with Preview
Model Predictive Control with trajectory lookahead. Solves an optimization at each timestep for the force sequence that minimizes tracking error + swing + effort. Can anticipate corners and pre-brake. Computationally heavier but feasible for this 10-state system.
