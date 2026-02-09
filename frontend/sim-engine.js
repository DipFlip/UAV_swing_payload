/**
 * Client-side simulation engine for the drone + hanging weight system.
 * Ported from Python backend (physics.py, controller.py, pid_controller.py, simulation.py).
 *
 * State vector (10 elements):
 *   [x_d, x_dot_d, y_d, y_dot_d, z_d, z_dot_d, phi_x, phi_dot_x, phi_y, phi_dot_y]
 *
 * Control vector (3 elements):
 *   [F_x, F_y, F_z]
 */

// ─── Matrix utilities (flat Float64Array, column-major-ish row*cols) ──────────

function matCreate(rows, cols) {
    return { rows, cols, d: new Float64Array(rows * cols) };
}

function matGet(m, r, c) { return m.d[r * m.cols + c]; }
function matSet(m, r, c, v) { m.d[r * m.cols + c] = v; }

function matIdentity(n) {
    const m = matCreate(n, n);
    for (let i = 0; i < n; i++) matSet(m, i, i, 1);
    return m;
}

function matCopy(a) {
    return { rows: a.rows, cols: a.cols, d: new Float64Array(a.d) };
}

function matMul(a, b) {
    const m = matCreate(a.rows, b.cols);
    for (let i = 0; i < a.rows; i++)
        for (let j = 0; j < b.cols; j++) {
            let s = 0;
            for (let k = 0; k < a.cols; k++)
                s += matGet(a, i, k) * matGet(b, k, j);
            matSet(m, i, j, s);
        }
    return m;
}

function matAdd(a, b) {
    const m = matCreate(a.rows, a.cols);
    for (let i = 0; i < a.d.length; i++) m.d[i] = a.d[i] + b.d[i];
    return m;
}

function matSub(a, b) {
    const m = matCreate(a.rows, a.cols);
    for (let i = 0; i < a.d.length; i++) m.d[i] = a.d[i] - b.d[i];
    return m;
}

function matScale(a, s) {
    const m = matCreate(a.rows, a.cols);
    for (let i = 0; i < a.d.length; i++) m.d[i] = a.d[i] * s;
    return m;
}

function matTranspose(a) {
    const m = matCreate(a.cols, a.rows);
    for (let i = 0; i < a.rows; i++)
        for (let j = 0; j < a.cols; j++)
            matSet(m, j, i, matGet(a, i, j));
    return m;
}

function matInverse(a) {
    const n = a.rows;
    const aug = matCreate(n, 2 * n);
    // Build augmented [A | I]
    for (let i = 0; i < n; i++) {
        for (let j = 0; j < n; j++) matSet(aug, i, j, matGet(a, i, j));
        matSet(aug, i, n + i, 1);
    }
    // Gauss-Jordan with partial pivoting
    for (let col = 0; col < n; col++) {
        // Find pivot
        let maxVal = Math.abs(matGet(aug, col, col));
        let maxRow = col;
        for (let row = col + 1; row < n; row++) {
            const v = Math.abs(matGet(aug, row, col));
            if (v > maxVal) { maxVal = v; maxRow = row; }
        }
        // Swap rows
        if (maxRow !== col) {
            for (let j = 0; j < 2 * n; j++) {
                const tmp = matGet(aug, col, j);
                matSet(aug, col, j, matGet(aug, maxRow, j));
                matSet(aug, maxRow, j, tmp);
            }
        }
        const pivot = matGet(aug, col, col);
        if (Math.abs(pivot) < 1e-14) throw new Error('Singular matrix');
        // Scale pivot row
        for (let j = 0; j < 2 * n; j++)
            matSet(aug, col, j, matGet(aug, col, j) / pivot);
        // Eliminate column
        for (let row = 0; row < n; row++) {
            if (row === col) continue;
            const factor = matGet(aug, row, col);
            for (let j = 0; j < 2 * n; j++)
                matSet(aug, row, j, matGet(aug, row, j) - factor * matGet(aug, col, j));
        }
    }
    // Extract inverse
    const inv = matCreate(n, n);
    for (let i = 0; i < n; i++)
        for (let j = 0; j < n; j++)
            matSet(inv, i, j, matGet(aug, i, n + j));
    return inv;
}

function matFromArray(rows, cols, arr) {
    const m = matCreate(rows, cols);
    for (let i = 0; i < arr.length; i++) m.d[i] = arr[i];
    return m;
}

function matFrobeniusNorm(a) {
    let s = 0;
    for (let i = 0; i < a.d.length; i++) s += a.d[i] * a.d[i];
    return Math.sqrt(s);
}

// ─── CARE solver (matrix sign function method) ───────────────────────────────

function solveCARE(A, B, Q, R, n) {
    // R^{-1}
    const Rinv = matInverse(R);
    // S = B R^{-1} B^T
    const Bt = matTranspose(B);
    const S = matMul(matMul(B, Rinv), Bt);
    // -A^T
    const At = matTranspose(A);
    const negAt = matScale(At, -1);

    // Build 2n x 2n Hamiltonian H = [A, -S; -Q, -A^T]
    const N = 2 * n;
    const H = matCreate(N, N);
    for (let i = 0; i < n; i++)
        for (let j = 0; j < n; j++) {
            matSet(H, i, j, matGet(A, i, j));
            matSet(H, i, n + j, -matGet(S, i, j));
            matSet(H, n + i, j, -matGet(Q, i, j));
            matSet(H, n + i, n + j, matGet(negAt, i, j));
        }

    // Matrix sign iteration: Z ← 0.5(Z + Z^{-1})
    let Z = matCopy(H);
    for (let iter = 0; iter < 100; iter++) {
        const Zinv = matInverse(Z);
        const Znew = matScale(matAdd(Z, Zinv), 0.5);
        const diff = matFrobeniusNorm(matSub(Znew, Z));
        Z = Znew;
        if (diff < 1e-10) break;
    }

    // Extract blocks: signH = [W11 W12; W21 W22]
    // P = (I - W11)^{-1} * W21  ... but more stable: use  P = -W21_block solve
    // Standard extraction: P from sign(H)
    // sign(H) = [S11 S12; S21 S22]
    // P = (S12)^{-1} * (S11 - I)  ... multiple formulas exist
    // Using: P = -(W21)^{-1} * W22 ... no
    // Correct: From sign function, if S = [S11 S12; S21 S22],
    // then P can be recovered as: (I - S11) is invertible and P = S21 * inv(S12)
    // Actually the standard formula: P = (I + S11)^{-1} * (-S12) ... let me use the
    // well-known one:
    // From the sign function of the Hamiltonian:
    // P = -[I, 0] * (sign(H) + I)^{-1} ... complicated.
    // Simpler: extract W12 and W11, then P = -(W12)^{-1} * (W11 + I_n)
    // Wait — let me use the Schur-based extraction:
    // If sign(H) = S, then the stable invariant subspace gives:
    // [S11+I  S12] [X] = 0  =>  X = -(S11+I)^{-1} S12 ... this gives a matrix
    // Actually P comes from: let U = [U1; U2] be the columns spanning the stable
    // invariant subspace. Then P = U2 * U1^{-1}.
    // For sign function: stable subspace = range of (I - S)/2
    // So V = (I - S) / 2, take first n columns of V...
    // Simpler standard approach for CARE:
    // Extract W11 = Z[0:n, 0:n], W21 = Z[n:2n, 0:n]
    // P = W21 * inv(W11) ... nope, need (I-S)/2 columns

    // Use (I - sign(H))/2 projection approach
    const I2n = matIdentity(N);
    const proj = matScale(matSub(I2n, Z), 0.5);

    // The columns of proj span the stable invariant subspace
    // Take the first n columns: U1 = proj[0:n, 0:n], U2 = proj[n:2n, 0:n]
    const U1 = matCreate(n, n);
    const U2 = matCreate(n, n);
    for (let i = 0; i < n; i++)
        for (let j = 0; j < n; j++) {
            matSet(U1, i, j, matGet(proj, i, j));
            matSet(U2, i, j, matGet(proj, n + i, j));
        }

    // P = U2 * U1^{-1}
    const P = matMul(U2, matInverse(U1));
    return P;
}

// ─── Physics ─────────────────────────────────────────────────────────────────

const DEFAULT_PARAMS = {
    m_d: 2.0,          // drone mass (kg)
    m_w: 5.0,          // weight mass (kg)
    L: 4.0,            // rope length (m)
    g: 9.81,           // gravity (m/s^2)
    maxLateral: 80,    // max lateral force per axis (N)
    maxThrust: 200,    // max vertical thrust (N)
};

function solve2x2(a11, a12, a21, a22, b1, b2) {
    const det = a11 * a22 - a12 * a21;
    return [(a22 * b1 - a12 * b2) / det, (a11 * b2 - a21 * b1) / det];
}

function derivatives(state, control, params) {
    const { m_d, m_w, L, g } = params;
    const [x_d, xd_dot, y_d, yd_dot, z_d, zd_dot, phi_x, phix_dot, phi_y, phiy_dot] = state;
    const [F_x, F_y, F_z] = control;

    // X-phi_x subsystem
    const cos_px = Math.cos(phi_x);
    const sin_px = Math.sin(phi_x);
    const rhs_x0 = F_x + m_w * L * phix_dot * phix_dot * sin_px;
    const rhs_x1 = -m_w * g * L * sin_px;
    const [x_dd, phix_dd] = solve2x2(
        m_d + m_w, m_w * L * cos_px,
        m_w * L * cos_px, m_w * L * L,
        rhs_x0, rhs_x1
    );

    // Y-phi_y subsystem
    const cos_py = Math.cos(phi_y);
    const sin_py = Math.sin(phi_y);
    const rhs_y0 = F_y + m_w * L * phiy_dot * phiy_dot * sin_py;
    const rhs_y1 = -m_w * g * L * sin_py;
    const [y_dd, phiy_dd] = solve2x2(
        m_d + m_w, m_w * L * cos_py,
        m_w * L * cos_py, m_w * L * L,
        rhs_y0, rhs_y1
    );

    // Vertical subsystem
    const z_dd = (F_z - (m_d + m_w) * g) / (m_d + m_w);

    return [xd_dot, x_dd, yd_dot, y_dd, zd_dot, z_dd, phix_dot, phix_dd, phiy_dot, phiy_dd];
}

function rk4Step(state, control, params, dt) {
    const add = (a, b, s) => a.map((v, i) => v + b[i] * s);

    const k1 = derivatives(state, control, params);
    const k2 = derivatives(add(state, k1, 0.5 * dt), control, params);
    const k3 = derivatives(add(state, k2, 0.5 * dt), control, params);
    const k4 = derivatives(add(state, k3, dt), control, params);

    return state.map((v, i) => v + (dt / 6) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
}

function weightPosition(state, L) {
    const x_d = state[0], y_d = state[2], z_d = state[4];
    const phi_x = state[6], phi_y = state[8];
    return {
        x: x_d + L * Math.sin(phi_x) * Math.cos(phi_y),
        y: y_d + L * Math.sin(phi_y),
        z: z_d - L * Math.cos(phi_x) * Math.cos(phi_y),
    };
}

// ─── LQR Controller ─────────────────────────────────────────────────────────

function computeLqrGains(params, Qlat, Rlat, Qvert, Rvert) {
    const { m_d, m_w, L, g } = params;

    // Lateral subsystem A,B matrices (4x4, 4x1)
    const A_lat = matFromArray(4, 4, [
        0, 1, 0, 0,
        0, 0, m_w * g / m_d, 0,
        0, 0, 0, 1,
        0, 0, -(m_d + m_w) * g / (m_d * L), 0,
    ]);
    const B_lat = matFromArray(4, 1, [0, 1 / m_d, 0, -1 / (m_d * L)]);

    if (!Qlat) Qlat = matFromArray(4, 4, [100, 0, 0, 0, 0, 25, 0, 0, 0, 0, 120, 0, 0, 0, 0, 30]);
    if (!Rlat) Rlat = matFromArray(1, 1, [0.08]);

    const P_lat = solveCARE(A_lat, B_lat, Qlat, Rlat, 4);
    // K = R^{-1} B^T P
    const K_lat = matMul(matMul(matInverse(Rlat), matTranspose(B_lat)), P_lat);

    // Vertical subsystem (2x2, 2x1)
    const A_vert = matFromArray(2, 2, [0, 1, 0, 0]);
    const B_vert = matFromArray(2, 1, [0, 1 / (m_d + m_w)]);

    if (!Qvert) Qvert = matFromArray(2, 2, [250, 0, 0, 80]);
    if (!Rvert) Rvert = matFromArray(1, 1, [0.08]);

    const P_vert = solveCARE(A_vert, B_vert, Qvert, Rvert, 2);
    const K_vert = matMul(matMul(matInverse(Rvert), matTranspose(B_vert)), P_vert);

    return { K_lat, K_vert };
}

function computeLqrControl(state, goal, K_lat, K_vert, params) {
    const x_d_des = goal[0];
    const y_d_des = goal[1];
    const z_d_des = goal[2] + params.L;

    // X-axis
    const x_err = [state[0] - x_d_des, state[1], state[6], state[7]];
    let F_x = 0;
    for (let i = 0; i < 4; i++) F_x -= matGet(K_lat, 0, i) * x_err[i];

    // Y-axis
    const y_err = [state[2] - y_d_des, state[3], state[8], state[9]];
    let F_y = 0;
    for (let i = 0; i < 4; i++) F_y -= matGet(K_lat, 0, i) * y_err[i];

    // Z-axis
    const z_err = [state[4] - z_d_des, state[5]];
    let F_z = 0;
    for (let i = 0; i < 2; i++) F_z -= matGet(K_vert, 0, i) * z_err[i];

    // Gravity feedforward
    F_z += (params.m_d + params.m_w) * params.g;

    // Saturation
    const ml = params.maxLateral, mt = params.maxThrust;
    F_x = Math.max(-ml, Math.min(ml, F_x));
    F_y = Math.max(-ml, Math.min(ml, F_y));
    F_z = Math.max(0, Math.min(mt, F_z));

    return [F_x, F_y, F_z];
}

// ─── PID Controller ──────────────────────────────────────────────────────────

class PIDAxis {
    constructor(kp, ki, kd, windup = 50) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.windup = windup;
        this.integral = 0;
        this.prevMeasurement = null;
    }

    step(error, measurement, dt) {
        this.integral += error * dt;
        this.integral = Math.max(-this.windup, Math.min(this.windup, this.integral));
        // Derivative on measurement (not error) to avoid derivative kick on setpoint changes
        let derivative = 0;
        if (this.prevMeasurement !== null && dt > 0) {
            derivative = -(measurement - this.prevMeasurement) / dt;
        }
        this.prevMeasurement = measurement;
        return this.kp * error + this.ki * this.integral + this.kd * derivative;
    }

    reset() {
        this.integral = 0;
        this.prevMeasurement = null;
    }
}

class PIDController {
    constructor(params) {
        this.params = params;
        this.pidX = new PIDAxis(22, 3, 24);
        this.pidY = new PIDAxis(22, 3, 24);
        this.pidZ = new PIDAxis(50, 10, 30);
    }

    computeControl(state, goal, dt) {
        const { L, m_d, m_w, g } = this.params;
        const ex = goal[0] - state[0];
        const ey = goal[1] - state[2];
        const ez = (goal[2] + L) - state[4];

        let F_x = this.pidX.step(ex, state[0], dt);
        let F_y = this.pidY.step(ey, state[2], dt);
        let F_z = this.pidZ.step(ez, state[4], dt);

        F_z += (m_d + m_w) * g;

        const ml = this.params.maxLateral, mt = this.params.maxThrust;
        F_x = Math.max(-ml, Math.min(ml, F_x));
        F_y = Math.max(-ml, Math.min(ml, F_y));
        F_z = Math.max(0, Math.min(mt, F_z));

        return [F_x, F_y, F_z];
    }

    reset() {
        this.pidX.reset();
        this.pidY.reset();
        this.pidZ.reset();
    }
}

// ─── Cascade PD Controller ──────────────────────────────────────────────────
// Outer loop: PD on payload position → desired drone offset
// Inner loop: PD on drone position → force command

class CascadePDController {
    constructor(params) {
        this.params = params;
        // Outer loop gains (payload position → drone offset)
        this.kp_outer = 1.8;
        this.kd_outer = 1.2;
        // Inner loop gains (drone position → force)
        this.kp_inner = 30;
        this.kd_inner = 20;
        // Vertical
        this.kp_z = 50;
        this.kd_z = 30;
        this.prevPayloadX = null;
        this.prevPayloadY = null;
    }

    computeControl(state, goal, dt) {
        const { L, m_d, m_w, g, maxLateral, maxThrust } = this.params;

        // Current payload position
        const w = weightPosition(state, L);
        const goalDroneZ = goal[2] + L;

        // Payload velocity estimate (derivative on measurement)
        let wdx = 0, wdy = 0;
        if (this.prevPayloadX !== null && dt > 0) {
            wdx = (w.x - this.prevPayloadX) / dt;
            wdy = (w.y - this.prevPayloadY) / dt;
        }
        this.prevPayloadX = w.x;
        this.prevPayloadY = w.y;

        // Outer loop: desired drone position based on payload error
        const payload_ex = goal[0] - w.x;
        const payload_ey = goal[1] - w.y;
        const drone_x_des = w.x + this.kp_outer * payload_ex + this.kd_outer * (-wdx);
        const drone_y_des = w.y + this.kp_outer * payload_ey + this.kd_outer * (-wdy);

        // Inner loop: force from drone position error
        const drone_ex = drone_x_des - state[0];
        const drone_ey = drone_y_des - state[2];
        let F_x = this.kp_inner * drone_ex - this.kd_inner * state[1];
        let F_y = this.kp_inner * drone_ey - this.kd_inner * state[3];

        // Vertical (simple PD, same as PID)
        const ez = goalDroneZ - state[4];
        let F_z = this.kp_z * ez - this.kd_z * state[5] + (m_d + m_w) * g;

        F_x = Math.max(-maxLateral, Math.min(maxLateral, F_x));
        F_y = Math.max(-maxLateral, Math.min(maxLateral, F_y));
        F_z = Math.max(0, Math.min(maxThrust, F_z));

        return [F_x, F_y, F_z];
    }

    setAggression(aggr) {
        const s = aggr * (0.5 + 0.5 * aggr);
        this.kp_outer = 1.8 * s;
        this.kd_outer = 1.2 * s;
        this.kp_inner = 30 * s;
        this.kd_inner = 20 * s;
        this.kp_z = 50 * s;
        this.kd_z = 30 * s;
    }

    reset() {
        this.prevPayloadX = null;
        this.prevPayloadY = null;
    }
}

// ─── Differential Flatness + Feedforward Controller ─────────────────────────
// Smooths goal through a 2nd-order reference filter, computes feedforward
// drone position from flatness inversion, adds full-state feedback.

class RefTrajectory {
    constructor(pos0) {
        this.pos = pos0;
        this.vel = 0;
    }

    update(goal, omega, dt) {
        const acc = omega * omega * (goal - this.pos) - 2.0 * omega * this.vel;
        this.vel += acc * dt;
        this.pos += this.vel * dt;
        return { pos: this.pos, vel: this.vel, acc };
    }

    reset(pos) {
        this.pos = pos;
        this.vel = 0;
    }
}

class FlatnessController {
    constructor(params) {
        this.params = params;
        this.omega = 2.0;
        // Reference trajectories for payload position
        this.trajX = new RefTrajectory(0);
        this.trajY = new RefTrajectory(0);
        this.trajZ = new RefTrajectory(params.L);
        // Feedback gains
        this.kp = 25;
        this.kd = 15;
        this.kp_phi = 40;
        this.kd_phi = 15;
        this.kp_z = 50;
        this.kd_z = 30;
    }

    computeControl(state, goal, dt) {
        const { L, g, m_d, m_w, maxLateral, maxThrust } = this.params;

        // Generate smooth payload reference
        const xRef = this.trajX.update(goal[0], this.omega, dt);
        const yRef = this.trajY.update(goal[1], this.omega, dt);
        const zRef = this.trajZ.update(goal[2] + L, this.omega, dt);

        // Flatness inversion (linearized):
        // desired drone position = payload_ref + (L/g) * payload_accel
        const x_d_des = xRef.pos + (L / g) * xRef.acc;
        const y_d_des = yRef.pos + (L / g) * yRef.acc;

        // Desired pendulum angle from flatness: phi = -x_w_ddot / g
        const phi_x_des = -xRef.acc / g;
        const phi_y_des = -yRef.acc / g;

        // Feedforward force: F = (m_d + m_w) * x_w_ddot
        const Fx_ff = (m_d + m_w) * xRef.acc;
        const Fy_ff = (m_d + m_w) * yRef.acc;

        // Feedback: PD on drone position + PD on pendulum angle
        let F_x = Fx_ff
            + this.kp * (x_d_des - state[0]) - this.kd * state[1]
            - this.kp_phi * (state[6] - phi_x_des) - this.kd_phi * state[7];

        let F_y = Fy_ff
            + this.kp * (y_d_des - state[2]) - this.kd * state[3]
            - this.kp_phi * (state[8] - phi_y_des) - this.kd_phi * state[9];

        // Vertical (no pendulum coupling)
        let F_z = (m_d + m_w) * g
            + this.kp_z * (zRef.pos - state[4]) - this.kd_z * state[5];

        F_x = Math.max(-maxLateral, Math.min(maxLateral, F_x));
        F_y = Math.max(-maxLateral, Math.min(maxLateral, F_y));
        F_z = Math.max(0, Math.min(maxThrust, F_z));

        return [F_x, F_y, F_z];
    }

    setAggression(aggr) {
        const s = aggr * (0.5 + 0.5 * aggr);
        this.omega = 1.0 + 2.0 * aggr;
        this.kp = 25 * s;
        this.kd = 15 * s;
        this.kp_phi = 40 * s;
        this.kd_phi = 15 * s;
        this.kp_z = 50 * s;
        this.kd_z = 30 * s;
    }

    reset(params) {
        this.trajX.reset(0);
        this.trajY.reset(0);
        this.trajZ.reset(params.L);
    }
}

// ─── Energy-Based Swing Damping (layerable on any controller) ───────────────
// Adds force proportional to phi_dot to dissipate pendulum energy.
// F_damp = k_e * phi_dot (same direction as angular velocity → damps swing)

function applySwingDamping(control, state, params, gain) {
    const k_e = gain || (params.m_d + params.m_w) * Math.sqrt(params.g * params.L) * 0.3;
    const ml = params.maxLateral;
    let [F_x, F_y, F_z] = control;
    F_x += k_e * state[7]; // phi_x_dot
    F_y += k_e * state[9]; // phi_y_dot
    F_x = Math.max(-ml, Math.min(ml, F_x));
    F_y = Math.max(-ml, Math.min(ml, F_y));
    return [F_x, F_y, F_z];
}

// ─── Simulation ──────────────────────────────────────────────────────────────

export class Simulation {
    constructor(controllerType = 'lqr') {
        this.params = { ...DEFAULT_PARAMS };
        this.dt = 0.02;
        this.time = 0;
        this.controllerType = controllerType;
        this.swingDamping = false;

        // Initial state: drone at z = L, everything else zero
        this.state = new Array(10).fill(0);
        this.state[4] = this.params.L;

        this.goal = [0, 0, 0];
        this.lastControl = [0, 0, (this.params.m_d + this.params.m_w) * this.params.g];

        // Compute LQR gains
        const { K_lat, K_vert } = computeLqrGains(this.params);
        this.K_lat = K_lat;
        this.K_vert = K_vert;

        // PID controller
        this.pid = new PIDController(this.params);

        // Cascade PD controller
        this.cascade = new CascadePDController(this.params);

        // Flatness controller
        this.flatness = new FlatnessController(this.params);
    }

    step() {
        let control;
        switch (this.controllerType) {
            case 'pid':
                control = this.pid.computeControl(this.state, this.goal, this.dt);
                break;
            case 'cascade':
                control = this.cascade.computeControl(this.state, this.goal, this.dt);
                break;
            case 'flatness':
                control = this.flatness.computeControl(this.state, this.goal, this.dt);
                break;
            default: // 'lqr'
                control = computeLqrControl(this.state, this.goal, this.K_lat, this.K_vert, this.params);
                break;
        }

        if (this.swingDamping) {
            control = applySwingDamping(control, this.state, this.params);
        }

        this.lastControl = control;

        this.state = rk4Step(this.state, control, this.params, this.dt);
        this.time += this.dt;

        const w = weightPosition(this.state, this.params.L);

        return {
            type: 'state',
            time: Math.round(this.time * 10000) / 10000,
            drone: {
                x: Math.round(this.state[0] * 10000) / 10000,
                y: Math.round(this.state[2] * 10000) / 10000,
                z: Math.round(this.state[4] * 10000) / 10000,
            },
            weight: {
                x: Math.round(w.x * 10000) / 10000,
                y: Math.round(w.y * 10000) / 10000,
                z: Math.round(w.z * 10000) / 10000,
            },
            phi_x: Math.round(this.state[6] * 10000) / 10000,
            phi_y: Math.round(this.state[8] * 10000) / 10000,
            goal: {
                x: Math.round(this.goal[0] * 10000) / 10000,
                y: Math.round(this.goal[1] * 10000) / 10000,
                z: Math.round(this.goal[2] * 10000) / 10000,
            },
            control: {
                Fx: Math.round(this.lastControl[0] * 100) / 100,
                Fy: Math.round(this.lastControl[1] * 100) / 100,
                Fz: Math.round(this.lastControl[2] * 100) / 100,
            },
        };
    }

    setGoal(x, y, z) {
        this.goal = [x, y, z];
    }

    setControllerType(type) {
        this.controllerType = type;
    }

    setSwingDamping(enabled) {
        this.swingDamping = enabled;
    }

    setControllerParams(type, p) {
        switch (type) {
            case 'lqr': {
                const Qlat = matFromArray(4, 4, [
                    p.qpos, 0, 0, 0,
                    0, p.qpos * 0.25, 0, 0,
                    0, 0, p.qphi, 0,
                    0, 0, 0, p.qphi * 0.25,
                ]);
                const Rlat = matFromArray(1, 1, [p.rcost]);
                const Qvert = matFromArray(2, 2, [p.qpos * 2.5, 0, 0, p.qpos * 0.8]);
                const Rvert = matFromArray(1, 1, [p.rcost]);
                const { K_lat, K_vert } = computeLqrGains(this.params, Qlat, Rlat, Qvert, Rvert);
                this.K_lat = K_lat;
                this.K_vert = K_vert;
                break;
            }
            case 'pid':
                this.pid.pidX.kp = p.kp;
                this.pid.pidX.ki = p.ki;
                this.pid.pidX.kd = p.kd;
                this.pid.pidY.kp = p.kp;
                this.pid.pidY.ki = p.ki;
                this.pid.pidY.kd = p.kd;
                this.pid.pidZ.kp = p.kp * 2.3;
                this.pid.pidZ.ki = p.ki * 3.3;
                this.pid.pidZ.kd = p.kd * 1.25;
                break;
            case 'cascade':
                this.cascade.kp_outer = p.kp_outer;
                this.cascade.kd_outer = p.kd_outer;
                this.cascade.kp_inner = p.kp_inner;
                this.cascade.kd_inner = p.kd_inner;
                this.cascade.kp_z = p.kp_inner * 1.67;
                this.cascade.kd_z = p.kd_inner * 1.5;
                break;
            case 'flatness':
                this.flatness.omega = p.omega;
                this.flatness.kp = p.kp;
                this.flatness.kd = p.kp * 0.6;
                this.flatness.kp_phi = p.kp_phi;
                this.flatness.kd_phi = p.kp_phi * 0.375;
                this.flatness.kp_z = p.kp * 2;
                this.flatness.kd_z = p.kp * 1.2;
                break;
        }
    }

    setAggression(aggr) {
        aggr = Math.max(0.05, Math.min(1.0, aggr));

        // Quadratic scaling: gentle at low aggression, much stronger at high
        const qScale = aggr * aggr;
        const rScale = 1 / (aggr * aggr);

        // Re-compute LQR gains with scaled Q/R
        const Qlat = matFromArray(4, 4, [
            100 * qScale, 0, 0, 0,
            0, 25 * qScale, 0, 0,
            0, 0, 120 * qScale, 0,
            0, 0, 0, 30 * qScale,
        ]);
        const Rlat = matFromArray(1, 1, [0.08 * rScale]);
        const Qvert = matFromArray(2, 2, [250 * qScale, 0, 0, 80 * qScale]);
        const Rvert = matFromArray(1, 1, [0.08 * rScale]);

        const { K_lat, K_vert } = computeLqrGains(this.params, Qlat, Rlat, Qvert, Rvert);
        this.K_lat = K_lat;
        this.K_vert = K_vert;

        // Scale PID gains (quadratic for snappier high-end)
        const pScale = aggr * (0.5 + 0.5 * aggr);
        this.pid.pidX.kp = 22.0 * pScale;
        this.pid.pidX.ki = 3.0 * pScale;
        this.pid.pidX.kd = 24.0 * pScale;
        this.pid.pidY.kp = 22.0 * pScale;
        this.pid.pidY.ki = 3.0 * pScale;
        this.pid.pidY.kd = 24.0 * pScale;
        this.pid.pidZ.kp = 50.0 * pScale;
        this.pid.pidZ.ki = 10.0 * pScale;
        this.pid.pidZ.kd = 30.0 * pScale;

        // Scale cascade PD
        this.cascade.setAggression(aggr);

        // Scale flatness controller
        this.flatness.setAggression(aggr);
    }

    setParams(updates) {
        Object.assign(this.params, updates);
        this.pid.params = this.params;
        this.cascade.params = this.params;
        this.flatness.params = this.params;
        // Recompute LQR gains for new params
        const { K_lat, K_vert } = computeLqrGains(this.params);
        this.K_lat = K_lat;
        this.K_vert = K_vert;
    }

    reset() {
        this.state = new Array(10).fill(0);
        this.state[4] = this.params.L;
        this.goal = [0, 0, 0];
        this.time = 0;
        this.lastControl = [0, 0, (this.params.m_d + this.params.m_w) * this.params.g];
        this.pid.reset();
        this.cascade.reset();
        this.flatness.reset(this.params);
    }
}
