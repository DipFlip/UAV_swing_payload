# Drone + Hanging Weight Simulation

A browser-based 3D simulation comparing two control algorithms for a drone carrying a hanging payload on a rigid rope. Runs entirely client-side as a static site — no backend required.

**[Live Demo](https://dipflip.github.io/UAV_swing_payload/)**

## Physics Model

The system is a **3D point-mass drone** coupled to a **spherical pendulum** (the payload) via a rigid, massless rod of fixed length.

**Degrees of freedom (5):**
- Drone position: x, y, z
- Pendulum swing angles: phi_x (x-z plane), phi_y (y-z plane)

**State vector (10 elements):**
Position and velocity for each DOF — `[x, x_dot, y, y_dot, z, z_dot, phi_x, phi_x_dot, phi_y, phi_y_dot]`.

**Equations of motion** are derived from the Lagrangian of the coupled system. Each lateral axis (x, y) produces a 2x2 coupled mass-matrix system between the drone and the pendulum. The vertical axis is decoupled. Integration uses **RK4** at a fixed timestep of 0.02s.

**What's not modeled:** aerodynamic drag, rope elasticity/slack, drone rotational dynamics (attitude, motor lag), cross-coupling between x and y pendulum planes, ground contact.

## Control Algorithms

Both controllers receive the full state vector and a goal position for the payload. They output 3 force components (Fx, Fy, Fz) applied to the drone.

### LQR (Linear Quadratic Regulator)

Linearizes the dynamics around the hover equilibrium and solves the **Continuous Algebraic Riccati Equation (CARE)** to find optimal feedback gains. The gain matrix K maps the full state error (including pendulum angles and angular rates) to control forces.

- Uses all 10 state variables — sees the pendulum and anticipates swing
- Gains are precomputed offline (CARE solved via matrix sign function method)
- Mathematically optimal for the linearized system
- Aggression slider scales Q (state cost) up and R (control cost) down, then re-solves CARE

### PID (Proportional-Integral-Derivative)

Three independent single-axis PID controllers drive the drone position to hover above the goal. Each axis computes: `F = Kp * error + Ki * integral(error) + Kd * d(measurement)/dt`.

- Uses only drone position (3 of 10 state variables) — blind to pendulum dynamics
- Uses derivative-on-measurement to avoid setpoint derivative kick
- Oscillates more because it only reacts to swing after it has displaced the drone
- Aggression slider linearly scales all PID gains

### Aggression

The aggression parameter (5%–100%) controls how hard both controllers try to reach the goal:
- **LQR**: Scales Q and R matrices, re-solves the Riccati equation — maintains optimality at every level
- **PID**: Linearly scales Kp, Ki, Kd gains — higher values mean faster response but more oscillation

## Removed Algorithms

### Feedback Linearization (FBL)

Feedback linearization was removed in v4.16 after extensive testing showed it is fundamentally unsuitable for this system. The technique cancels the nonlinear sin/cos coupling between the drone and pendulum, replacing it with a virtual linear system. However, the drone-pendulum system has **non-collocated actuation** — the control force acts on the drone while the output of interest is the payload position, 8 meters below on a rope. This creates **non-minimum-phase dynamics**: when the drone accelerates, the payload initially swings backward before following.

FBL's cancellation approach requires precise force matching at large angles, where actuator saturation makes exact cancellation impossible. Headless testing showed it performed worst of all algorithms:
- 5m step response: 1.12m final error after 20s (did not settle), vs LQR's 0.009m
- 15N wind: 4.4m steady-state error (worst), vs LQR's 0.9m
- Square trajectory: 4.3m avg tracking error (worst), vs LQR's 1.0m

Despite multiple iterations of fixes (correcting angle feedback signs, bounding virtual input, using sin(phi) saturation, adjusting integral gains), the fundamental non-collocated mismatch prevented acceptable performance.

## Interactive Controls

| Control | Function |
|---------|----------|
| X, Y, Z sliders | Set goal position for the payload |
| Drone mass | Adjust drone mass (0.5–10 kg) |
| Payload | Adjust payload mass (0.5–20 kg) |
| Max force | Lateral force cap per axis (5–100 N) |
| Aggression | Controller responsiveness (5–100%) |
| Time scale | Simulation speed (5–100%, where 100% = realtime) |
| Pattern speed | Speed of the square pattern animation |
| Pause/Play | Freeze/resume the simulation |
| Go | Apply current slider goal |
| Square | Animate goal in a square pattern |
| Reset | Return to initial hover state |

## Visual Elements

- **Blue drone (LQR)** with red payload
- **Orange drone (PID)** with yellow payload
- **Green wireframe sphere** — goal marker
- **Red arrows** — applied force vectors on each drone
- **Colored trails** — 2-second position history for all objects
- **Drone tilt** — drones bank in the direction of lateral thrust (cosmetic, not part of physics)
- **Real-time charts** — distance to goal, swing angle, control forces, weight speed

## Deployment

The `frontend/` directory is a self-contained static site. It deploys automatically to GitHub Pages via the included workflow (`.github/workflows/deploy.yml`) on push to `main`.

To run locally:
```bash
cd frontend
python3 -m http.server 8000
# Open http://localhost:8000
```

## Project Structure

```
frontend/
  index.html        — UI layout and controls
  main.js           — Event wiring and simulation loop
  sim-engine.js     — Physics, CARE solver, LQR/PID controllers, Simulation class
  simulation.js     — Scene updates, trails, HUD
  scene.js          — Three.js scene setup (drones, ropes, grid, trails)
  charts.js         — Real-time rolling charts (pure Canvas 2D)
  style.css         — UI styling

backend/            — Original Python/FastAPI server (no longer required)
```
