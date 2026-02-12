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
    m_d: 11.0,         // drone mass (kg)
    m_w: 7.0,          // weight mass (kg)
    L: 8.0,            // rope length (m)
    g: 9.81,           // gravity (m/s^2)
    maxLateral: 140,   // max lateral force per axis (N)
    maxThrust: 350,    // max vertical thrust (N)
    windMean: 0,        // mean wind force on payload (N)
    windStddev: 0,      // wind force standard deviation (N)
    windDir: 0,         // wind direction (radians, 0 = +x)
    windWander: 0,      // wind direction wander rate (rad/sqrt(s))
    windX: 0,           // instantaneous wind x-component (computed)
    windY: 0,           // instantaneous wind y-component (computed)
};

function solve2x2(a11, a12, a21, a22, b1, b2) {
    const det = a11 * a22 - a12 * a21;
    return [(a22 * b1 - a12 * b2) / det, (a11 * b2 - a21 * b1) / det];
}

function derivatives(state, control, params) {
    const { m_d, m_w, L, g } = params;
    const Fw_x = params.windX || 0;
    const Fw_y = params.windY || 0;
    const [x_d, xd_dot, y_d, yd_dot, z_d, zd_dot, phi_x, phix_dot, phi_y, phiy_dot] = state;
    const [F_x, F_y, F_z] = control;

    // X-phi_x subsystem (wind force on payload adds to both equations)
    const cos_px = Math.cos(phi_x);
    const sin_px = Math.sin(phi_x);
    const rhs_x0 = F_x + Fw_x + m_w * L * phix_dot * phix_dot * sin_px;
    const rhs_x1 = Fw_x * L * cos_px - m_w * g * L * sin_px;
    const [x_dd, phix_dd] = solve2x2(
        m_d + m_w, m_w * L * cos_px,
        m_w * L * cos_px, m_w * L * L,
        rhs_x0, rhs_x1
    );

    // Y-phi_y subsystem
    const cos_py = Math.cos(phi_y);
    const sin_py = Math.sin(phi_y);
    const rhs_y0 = F_y + Fw_y + m_w * L * phiy_dot * phiy_dot * sin_py;
    const rhs_y1 = Fw_y * L * cos_py - m_w * g * L * sin_py;
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

    // Augmented lateral subsystem (5x5): [x_d, x_dot, phi, phi_dot, xi]
    // where xi = integral of payload position error
    // C_pay = [1, 0, L, 0] (payload_x = x_d + L*phi, linearized)
    const A_lat = matFromArray(4, 4, [
        0, 1, 0, 0,
        0, 0, m_w * g / m_d, 0,
        0, 0, 0, 1,
        0, 0, -(m_d + m_w) * g / (m_d * L), 0,
    ]);
    const B_lat4 = matFromArray(4, 1, [0, 1 / m_d, 0, -1 / (m_d * L)]);

    // Build A_aug (5x5) = [A_lat | 0; C_pay | 0]
    const A_aug = matCreate(5, 5);
    for (let i = 0; i < 4; i++)
        for (let j = 0; j < 4; j++)
            matSet(A_aug, i, j, matGet(A_lat, i, j));
    // Row 4: C_pay = [1, 0, L, 0, 0]
    matSet(A_aug, 4, 0, 1);
    matSet(A_aug, 4, 2, L);

    // Build B_aug (5x1) = [B_lat; 0]
    const B_aug = matCreate(5, 1);
    for (let i = 0; i < 4; i++)
        matSet(B_aug, i, 0, matGet(B_lat4, i, 0));

    if (!Qlat) {
        const qpos = 10, qphi = 30;
        Qlat = matFromArray(5, 5, [
            qpos,     0,            qpos*L,     0,                    0,
            0,        qpos*0.25,    0,          qpos*0.25*L,          0,
            qpos*L,   0,            qpos*L*L,   0,                    0,
            0,        qpos*0.25*L,  0,          qpos*0.25*L*L + qphi, 0,
            0,        0,            0,          0,                    qpos*0.1,
        ]);
    }
    if (!Rlat) Rlat = matFromArray(1, 1, [0.08]);

    const P_lat = solveCARE(A_aug, B_aug, Qlat, Rlat, 5);
    // K = R^{-1} B^T P
    const K_lat = matMul(matMul(matInverse(Rlat), matTranspose(B_aug)), P_lat);

    // Vertical subsystem (2x2, 2x1) — unchanged
    const A_vert = matFromArray(2, 2, [0, 1, 0, 0]);
    const B_vert = matFromArray(2, 1, [0, 1 / (m_d + m_w)]);

    if (!Qvert) Qvert = matFromArray(2, 2, [250, 0, 0, 80]);
    if (!Rvert) Rvert = matFromArray(1, 1, [0.08]);

    const P_vert = solveCARE(A_vert, B_vert, Qvert, Rvert, 2);
    const K_vert = matMul(matMul(matInverse(Rvert), matTranspose(B_vert)), P_vert);

    return { K_lat, K_vert };
}

function computeLqrControl(state, ref, K_lat, K_vert, params, integX, integY) {
    const L = params.L;
    const z_d_des = ref.pos[2] + L;

    // X-axis: 5-element error [x_d - payload_des, x_dot - vel_des, phi, phi_dot, xi]
    const x_err = [state[0] - ref.pos[0], state[1] - ref.vel[0], state[6], state[7], integX || 0];
    let F_x = 0;
    for (let i = 0; i < 5; i++) F_x -= matGet(K_lat, 0, i) * x_err[i];

    // Y-axis: same 5-element error structure
    const y_err = [state[2] - ref.pos[1], state[3] - ref.vel[1], state[8], state[9], integY || 0];
    let F_y = 0;
    for (let i = 0; i < 5; i++) F_y -= matGet(K_lat, 0, i) * y_err[i];

    // Z-axis
    const z_err = [state[4] - z_d_des, state[5] - ref.vel[2]];
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
        // Slow integral of payload position error for wind compensation
        this._payloadIntX = 0;
        this._payloadIntY = 0;
    }

    computeControl(state, ref, dt) {
        const { L, m_d, m_w, g } = this.params;

        // PID tracks drone position (stable negative feedback)
        const ex = ref.pos[0] - state[0];
        const ey = ref.pos[1] - state[2];
        const ez = (ref.pos[2] + L) - state[4];

        let F_x = this.pidX.step(ex, state[0], dt);
        let F_y = this.pidY.step(ey, state[2], dt);
        let F_z = this.pidZ.step(ez, state[4], dt);

        // Slow integral of payload error: under wind the payload drifts from
        // the goal, this integral biases the drone upwind to compensate
        const w = weightPosition(state, L);
        this._payloadIntX += (ref.pos[0] - w.x) * dt;
        this._payloadIntY += (ref.pos[1] - w.y) * dt;
        this._payloadIntX = Math.max(-10, Math.min(10, this._payloadIntX));
        this._payloadIntY = Math.max(-10, Math.min(10, this._payloadIntY));
        const ki_wind = this.pidX.kp * 0.15;
        F_x += ki_wind * this._payloadIntX;
        F_y += ki_wind * this._payloadIntY;

        // Acceleration feedforward
        F_x += (m_d + m_w) * ref.acc[0];
        F_y += (m_d + m_w) * ref.acc[1];

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
        this._payloadIntX = 0;
        this._payloadIntY = 0;
    }
}

// ─── Cascade PD Controller ──────────────────────────────────────────────────
// Outer loop: PD on payload position → desired drone offset
// Inner loop: PD on drone position → force command

class CascadePDController {
    constructor(params) {
        this.params = params;
        // Outer loop gains (payload error → drone offset correction)
        this.kp_outer = 0.5;
        this.kd_outer = 1.0;
        // Inner loop gains (drone position → force)
        this.kp_inner = 20;
        this.kd_inner = 13;
        // Vertical
        this.kp_z = 50;
        this.kd_z = 30;
        // Integral accumulators for payload position error
        this._integX = 0;
        this._integY = 0;
    }

    computeControl(state, ref, dt) {
        const { L, m_d, m_w, g, maxLateral, maxThrust } = this.params;

        // Current payload position
        const w = weightPosition(state, L);
        const goalDroneZ = ref.pos[2] + L;

        // Outer loop: desired drone position based on payload error
        // Uses drone velocity (cleaner than numerically differentiating payload pos)
        const payload_ex = ref.pos[0] - w.x;
        const payload_ey = ref.pos[1] - w.y;

        // Integral of payload position error
        this._integX += payload_ex * dt;
        this._integY += payload_ey * dt;
        this._integX = Math.max(-10, Math.min(10, this._integX));
        this._integY = Math.max(-10, Math.min(10, this._integY));
        const ki_outer = this.kp_outer * 0.15;

        // Desired drone position: payload_pos + correction to reduce payload error
        const drone_x_des = w.x + this.kp_outer * payload_ex + this.kd_outer * (ref.vel[0] - state[1]) + ki_outer * this._integX;
        const drone_y_des = w.y + this.kp_outer * payload_ey + this.kd_outer * (ref.vel[1] - state[3]) + ki_outer * this._integY;

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
        this.kp_outer = 0.5 * s;
        this.kd_outer = 1.0 * s;
        this.kp_inner = 20 * s;
        this.kd_inner = 13 * s;
        this.kp_z = 50 * s;
        this.kd_z = 30 * s;
    }

    reset() {
        this._integX = 0;
        this._integY = 0;
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

// ─── Goal Smoother (3-axis 2nd-order critically-damped filter) ───────────────
// Converts raw [x,y,z] goal into {pos, vel, acc} reference with smooth transitions.

class GoalSmoother3D {
    constructor(pos0, omega) {
        this._pos = [...pos0];
        this._vel = [0, 0, 0];
        this._omega = omega;
    }

    update(goal, dt) {
        // Bypass mode: no smoothing, pass goal directly
        if (this._omega <= 0) {
            this._pos = [...goal];
            this._vel = [0, 0, 0];
            return { pos: [...goal], vel: [0, 0, 0], acc: [0, 0, 0] };
        }
        const omega = this._omega;
        const acc = [0, 0, 0];
        for (let i = 0; i < 3; i++) {
            acc[i] = omega * omega * (goal[i] - this._pos[i]) - 2.0 * omega * this._vel[i];
            this._vel[i] += acc[i] * dt;
            this._pos[i] += this._vel[i] * dt;
        }
        return { pos: [...this._pos], vel: [...this._vel], acc: [...acc] };
    }

    reset(pos) {
        this._pos = [...pos];
        this._vel = [0, 0, 0];
    }

    setOmega(omega) {
        this._omega = omega;
    }
}

// ─── Waypoint Trajectory (Catmull-Rom cubic Hermite spline) ──────────────────
// Takes {pos: [x,y,z], time: number}[] waypoints, provides C1-continuous interpolation.

class WaypointTrajectory {
    /**
     * @param {Array} waypoints - [{pos: [x,y,z], time: number}, ...]
     * @param {Object} opts
     * @param {boolean} opts.loop - wrap around at end
     * @param {number} opts.tension - 0=Catmull-Rom (smooth), 1=linear (sharp corners)
     */
    constructor(waypoints, { loop = false, tension = 0 } = {}) {
        this._wp = waypoints;
        this._loop = loop;
        this._tension = Math.max(0, Math.min(1, tension));
        this._duration = waypoints[waypoints.length - 1].time - waypoints[0].time;
    }

    get duration() { return this._duration; }
    get waypoints() { return this._wp; }

    evaluate(t) {
        const wp = this._wp;
        const n = wp.length;

        // Loop wrapping
        if (this._loop && this._duration > 0) {
            t = ((t % this._duration) + this._duration) % this._duration;
            t += wp[0].time;
        } else {
            t += wp[0].time;
        }

        // Clamp to endpoints
        if (t <= wp[0].time) {
            return { pos: [...wp[0].pos], vel: [0, 0, 0], acc: [0, 0, 0] };
        }
        if (!this._loop && t >= wp[n - 1].time) {
            return { pos: [...wp[n - 1].pos], vel: [0, 0, 0], acc: [0, 0, 0] };
        }

        // Find segment: wp[seg] <= t < wp[seg+1]
        let seg = 0;
        for (let i = 0; i < n - 1; i++) {
            if (t >= wp[i].time && t < wp[i + 1].time) { seg = i; break; }
        }

        const t0 = wp[seg].time;
        const t1 = wp[seg + 1].time;
        const dt = t1 - t0;
        const u = (t - t0) / dt; // normalized [0,1)

        const p0 = wp[seg].pos;
        const p1 = wp[seg + 1].pos;

        // Linear interpolation (tension = 1)
        if (this._tension >= 1) {
            const pos = [0, 0, 0], vel = [0, 0, 0];
            for (let i = 0; i < 3; i++) {
                pos[i] = p0[i] + (p1[i] - p0[i]) * u;
                vel[i] = (p1[i] - p0[i]) / dt;
            }
            return { pos, vel, acc: [0, 0, 0] };
        }

        // Catmull-Rom tangents (scaled by 1 - tension)
        const scale = 1 - this._tension;
        const m0 = this._tangent(seg, n, scale);
        const m1 = this._tangent(seg + 1, n, scale);

        // Hermite basis functions
        const u2 = u * u, u3 = u2 * u;
        const h00 = 2 * u3 - 3 * u2 + 1;
        const h10 = u3 - 2 * u2 + u;
        const h01 = -2 * u3 + 3 * u2;
        const h11 = u3 - u2;

        // Hermite basis derivatives (w.r.t. u)
        const dh00 = 6 * u2 - 6 * u;
        const dh10 = 3 * u2 - 4 * u + 1;
        const dh01 = -6 * u2 + 6 * u;
        const dh11 = 3 * u2 - 2 * u;

        // Hermite basis second derivatives (w.r.t. u)
        const ddh00 = 12 * u - 6;
        const ddh10 = 6 * u - 4;
        const ddh01 = -12 * u + 6;
        const ddh11 = 6 * u - 2;

        const pos = [0, 0, 0], vel = [0, 0, 0], acc = [0, 0, 0];
        for (let i = 0; i < 3; i++) {
            pos[i] = h00 * p0[i] + h10 * dt * m0[i] + h01 * p1[i] + h11 * dt * m1[i];
            vel[i] = (dh00 * p0[i] + dh10 * dt * m0[i] + dh01 * p1[i] + dh11 * dt * m1[i]) / dt;
            acc[i] = (ddh00 * p0[i] + ddh10 * dt * m0[i] + ddh01 * p1[i] + ddh11 * dt * m1[i]) / (dt * dt);
        }

        return { pos, vel, acc };
    }

    /**
     * Sample the trajectory path at uniform intervals for visualization.
     * @param {number} numPoints
     * @returns {Array<[number,number,number]>} positions
     */
    samplePath(numPoints) {
        const points = [];
        for (let i = 0; i <= numPoints; i++) {
            const t = (i / numPoints) * this._duration;
            const ref = this.evaluate(t);
            points.push(ref.pos);
        }
        return points;
    }

    _tangent(idx, n, scale) {
        const wp = this._wp;
        let prev, next, dtSpan;

        if (this._loop) {
            // For looping, wrap around (skip duplicated last point)
            const loopN = n - 1; // last point = first point
            const prevIdx = ((idx - 1) % loopN + loopN) % loopN;
            const nextIdx = ((idx + 1) % loopN + loopN) % loopN;
            prev = wp[prevIdx];
            next = wp[nextIdx];
            // Compute time span in the forward direction around the loop
            dtSpan = next.time - prev.time;
            if (dtSpan <= 0) {
                // Wraps around the loop boundary
                dtSpan = this._duration - prev.time + wp[0].time + (next.time - wp[0].time);
            }
        } else {
            prev = wp[Math.max(0, idx - 1)];
            next = wp[Math.min(n - 1, idx + 1)];
            dtSpan = next.time - prev.time || 1;
        }

        return [
            scale * (next.pos[0] - prev.pos[0]) / dtSpan,
            scale * (next.pos[1] - prev.pos[1]) / dtSpan,
            scale * (next.pos[2] - prev.pos[2]) / dtSpan,
        ];
    }
}

export function createSquareTrajectory(size, z, speed, tension = 1) {
    const segTime = size / speed;
    const half = size / 2;
    const waypoints = [
        { pos: [ half,  half, z], time: 0 },
        { pos: [-half,  half, z], time: segTime },
        { pos: [-half, -half, z], time: 2 * segTime },
        { pos: [ half, -half, z], time: 3 * segTime },
        { pos: [ half,  half, z], time: 4 * segTime },
    ];
    return new WaypointTrajectory(waypoints, { loop: true, tension });
}

export function createLawnmowerTrajectory(size, z, speed, tension = 1) {
    const strips = 5;
    const half = size / 2;
    const spacing = size / (strips - 1);
    const stripTime = size / speed;
    const shiftTime = spacing / speed;

    const waypoints = [];
    let t = 0;

    for (let i = 0; i < strips; i++) {
        const x = -half + i * spacing;
        const goingUp = (i % 2 === 0);
        const y0 = goingUp ? -half : half;
        const y1 = goingUp ? half : -half;

        waypoints.push({ pos: [x, y0, z], time: t });
        t += stripTime;
        waypoints.push({ pos: [x, y1, z], time: t });

        if (i < strips - 1) {
            t += shiftTime;
        }
    }

    // Return to start for loop closure
    const returnDist = Math.sqrt(size * size + size * size);
    t += returnDist / speed;
    waypoints.push({ pos: [...waypoints[0].pos], time: t });

    return new WaypointTrajectory(waypoints, { loop: true, tension });
}

class FlatnessController {
    constructor(params) {
        this.params = params;
        // Feedback gains
        this.kp = 12;
        this.kd = 8;
        this.kp_phi = 10;
        this.kd_phi = 4;
        this.kp_z = 50;
        this.kd_z = 30;
        // Integral of payload position error
        this._integX = 0;
        this._integY = 0;
    }

    computeControl(state, ref, dt) {
        const { L, g, m_d, m_w, maxLateral, maxThrust } = this.params;

        // Integral of payload position error for wind compensation
        const w = weightPosition(state, L);
        this._integX += (ref.pos[0] - w.x) * dt;
        this._integY += (ref.pos[1] - w.y) * dt;
        this._integX = Math.max(-10, Math.min(10, this._integX));
        this._integY = Math.max(-10, Math.min(10, this._integY));
        const ki = this.kp * 0.08;

        // Drone position target: goal + payload integral bias
        // (The (L/g)*ref.acc position offset is omitted because with L=8m it creates
        //  targets far beyond the goal, exceeding actuator limits and causing overshoot)
        const x_d_des = ref.pos[0] + ki * this._integX;
        const y_d_des = ref.pos[1] + ki * this._integY;

        // Desired pendulum angle from flatness: phi = -acc / g (clamped)
        const maxAcc = maxLateral / (m_d + m_w) * 0.4;
        const acc_x = Math.max(-maxAcc, Math.min(maxAcc, ref.acc[0]));
        const acc_y = Math.max(-maxAcc, Math.min(maxAcc, ref.acc[1]));
        const phi_x_des = -acc_x / g;
        const phi_y_des = -acc_y / g;

        // Feedforward force (still useful, clamped to actuator limits later)
        const Fx_ff = (m_d + m_w) * ref.acc[0];
        const Fy_ff = (m_d + m_w) * ref.acc[1];

        // Feedback: PD on drone position + PD on pendulum angle
        // Angle sign: +kp_phi*(phi-phi_des) creates stable stiffness through coupling
        // (non-collocated feedback requires the opposite sign from intuition)
        // Use sin() for angle term to bound correction at large angles
        let F_x = Fx_ff
            + this.kp * (x_d_des - state[0]) - this.kd * state[1]
            + this.kp_phi * Math.sin(state[6] - phi_x_des) + this.kd_phi * state[7];

        let F_y = Fy_ff
            + this.kp * (y_d_des - state[2]) - this.kd * state[3]
            + this.kp_phi * Math.sin(state[8] - phi_y_des) + this.kd_phi * state[9];

        // Vertical (no pendulum coupling)
        const zDes = ref.pos[2] + L;
        let F_z = (m_d + m_w) * g
            + this.kp_z * (zDes - state[4]) - this.kd_z * state[5];

        F_x = Math.max(-maxLateral, Math.min(maxLateral, F_x));
        F_y = Math.max(-maxLateral, Math.min(maxLateral, F_y));
        F_z = Math.max(0, Math.min(maxThrust, F_z));

        return [F_x, F_y, F_z];
    }

    setAggression(aggr) {
        const s = aggr * (0.5 + 0.5 * aggr);
        this.kp = 12 * s;
        this.kd = 8 * s;
        this.kp_phi = 10 * s;
        this.kd_phi = 4 * s;
        this.kp_z = 50 * s;
        this.kd_z = 30 * s;
    }

    reset() {
        this._integX = 0;
        this._integY = 0;
    }
}

// ─── ZVD Input Shaper (layerable pre-filter) ────────────────────────────────
// Pre-filters the goal with 3 impulses timed at the pendulum natural frequency.
// Produces zero residual vibration at the pendulum's natural frequency.

class ZVDShaper {
    constructor(params, dt) {
        this.dt = dt;
        this._smoothGoal = null;
        this._buildBuffer(params);
    }

    _buildBuffer(params) {
        // Coupled pendulum frequency: omega = sqrt((m_d+m_w)*g / (m_d*L))
        // Period: T = 2*pi / omega = 2*pi * sqrt(m_d*L / ((m_d+m_w)*g))
        const T = 2 * Math.PI * Math.sqrt(params.m_d * params.L / ((params.m_d + params.m_w) * params.g));
        this._period = T;
        this._halfIdx = Math.round((T / 2) / this.dt);
        this._fullIdx = Math.round(T / this.dt);
        const bufLen = this._fullIdx + 1;
        // Ring buffer stores past smoothed goals (3-element arrays)
        this._buf = new Array(bufLen);
        for (let i = 0; i < bufLen; i++) this._buf[i] = null;
        this._head = 0;
        this._len = bufLen;
        this._filled = 0;
        this._lastParams = { L: params.L, g: params.g, m_d: params.m_d, m_w: params.m_w };
        // Pre-smoother: 2nd-order (two cascaded 1st-order) filter with tau = 25% of period.
        // The 2nd-order filter starts with zero derivative (smooth onset), preventing
        // the discrete jumps at T/2 intervals that cause twitching with high-gain controllers.
        this._alpha = 1 - Math.exp(-this.dt / (T * 0.25));
    }

    shapeGoal(goal, params) {
        // Detect param change that affects pendulum frequency → rebuild buffer
        const lp = this._lastParams;
        if (params.L !== lp.L || params.g !== lp.g || params.m_d !== lp.m_d || params.m_w !== lp.m_w) {
            this._buildBuffer(params);
        }

        // Two-stage pre-smoother: cascaded 1st-order filters give 2nd-order response
        // with zero initial derivative, producing smooth ZVD output for all controllers
        if (this._smoothGoal === null) {
            this._smoothGoal = [goal[0], goal[1], goal[2]];
            this._smoothGoal2 = [goal[0], goal[1], goal[2]];
        } else {
            for (let i = 0; i < 3; i++) {
                this._smoothGoal[i] += this._alpha * (goal[i] - this._smoothGoal[i]);
                this._smoothGoal2[i] += this._alpha * (this._smoothGoal[i] - this._smoothGoal2[i]);
            }
        }

        // Store 2nd-stage output in ring buffer
        this._buf[this._head] = [this._smoothGoal2[0], this._smoothGoal2[1], this._smoothGoal2[2]];
        this._filled = Math.min(this._filled + 1, this._len);

        // If buffer not yet full, pass smoothed goal through
        if (this._filled < this._len) {
            this._head = (this._head + 1) % this._len;
            return [this._smoothGoal2[0], this._smoothGoal2[1], this._smoothGoal2[2]];
        }

        // ZVD: 0.25 * goal(t) + 0.5 * goal(t - T/2) + 0.25 * goal(t - T)
        const g0 = this._buf[this._head]; // current (t)
        const g1Idx = (this._head - this._halfIdx + this._len) % this._len;
        const g2Idx = (this._head - this._fullIdx + this._len) % this._len;
        const g1 = this._buf[g1Idx];
        const g2 = this._buf[g2Idx];

        const shaped = [
            0.25 * g0[0] + 0.5 * g1[0] + 0.25 * g2[0],
            0.25 * g0[1] + 0.5 * g1[1] + 0.25 * g2[1],
            0.25 * g0[2] + 0.5 * g1[2] + 0.25 * g2[2],
        ];

        this._head = (this._head + 1) % this._len;
        return shaped;
    }

    reset(params) {
        this._smoothGoal = null;
        this._smoothGoal2 = null;
        this._buildBuffer(params);
    }
}

// ─── Feedback Linearization Controller ──────────────────────────────────────
// Cancels nonlinear sin/cos coupling, then applies PD on the linearized system.

class FeedbackLinController {
    constructor(params) {
        this.params = params;
        this.kp = 2;
        this.ka = 5;
        this.kb = 1;
        // Integral of payload position error
        this._integX = 0;
        this._integY = 0;
    }

    _computeVirtualInput(posErr, velErr, phi, phiDot) {
        const kd = this.kp * 0.83;
        const ki = this.kp * 0.1;
        // Use sin(phi) for angle term: bounded ±1 at large angles, ≈phi at small angles
        // Clamp phi_dot to prevent rate-driven overcorrection
        const phiDotClamped = Math.max(-2, Math.min(2, phiDot));
        let vx = this.kp * posErr + kd * velErr
            + this.ka * Math.sin(phi) + this.kb * phiDotClamped;
        // Clamp virtual input to keep drone acceleration within reasonable bounds
        // (prevents exciting pendulum past the recovery range for L=8m rope)
        const vxMax = this.params.maxLateral / (this.params.m_d + this.params.m_w) * 0.8;
        return Math.max(-vxMax, Math.min(vxMax, vx));
    }

    computeControl(state, ref, dt) {
        const { m_d, m_w, L, g, maxLateral, maxThrust } = this.params;
        const ki = this.kp * 0.1;

        // X-axis feedback linearization
        const phi_x = state[6], phix_dot = state[7];
        const sin_px = Math.sin(phi_x), cos_px = Math.cos(phi_x);
        // Integral of payload position error (for wind compensation)
        const w_x = state[0] + L * Math.sin(phi_x);
        this._integX += (ref.pos[0] - w_x) * dt;
        this._integX = Math.max(-10, Math.min(10, this._integX));

        // Virtual input: P/D on drone position (stable), integral biases for payload offset
        // Angle signs: +ka*sin(phi) + kb*phi_dot creates stable stiffness and damping
        // in the phi dynamics (through coupling: phi_dd ≈ -(g/L + ka/L)*phi - (kb/L)*phi_dot)
        const vx = this._computeVirtualInput(
            ref.pos[0] + ki * this._integX - state[0],
            ref.vel[0] - state[1], phi_x, phix_dot
        );
        let F_x = (m_d + m_w * sin_px * sin_px) * vx
            - m_w * L * sin_px * phix_dot * phix_dot
            - m_w * g * sin_px * cos_px;

        // Y-axis feedback linearization
        const phi_y = state[8], phiy_dot = state[9];
        const sin_py = Math.sin(phi_y), cos_py = Math.cos(phi_y);
        const w_y = state[2] + L * Math.sin(phi_y);
        this._integY += (ref.pos[1] - w_y) * dt;
        this._integY = Math.max(-10, Math.min(10, this._integY));

        const vy = this._computeVirtualInput(
            ref.pos[1] + ki * this._integY - state[2],
            ref.vel[1] - state[3], phi_y, phiy_dot
        );
        let F_y = (m_d + m_w * sin_py * sin_py) * vy
            - m_w * L * sin_py * phiy_dot * phiy_dot
            - m_w * g * sin_py * cos_py;

        // Vertical (simple PD)
        const ez = (ref.pos[2] + L) - state[4];
        let F_z = (m_d + m_w) * g + 50 * ez - 30 * state[5];

        F_x = Math.max(-maxLateral, Math.min(maxLateral, F_x));
        F_y = Math.max(-maxLateral, Math.min(maxLateral, F_y));
        F_z = Math.max(0, Math.min(maxThrust, F_z));

        return [F_x, F_y, F_z];
    }

    reset() {
        this._integX = 0;
        this._integY = 0;
    }
}

// ─── Sliding Mode Controller ────────────────────────────────────────────────
// Sliding surface + equivalent control + bounded switching term.

class SlidingModeController {
    constructor(params) {
        this.params = params;
        this.lambda = 2;
        this.alpha = 8;
        this.kSwitch = 15;
        this.epsilon = 0.5;
        // Integral of payload position error for wind compensation
        this._integX = 0;
        this._integY = 0;
    }

    _computeAxis(posErr, velErr, phi, phiDot, params) {
        const { m_d, m_w, L, g } = params;
        const beta = this.alpha * 0.375;

        // Sliding surface: s = lambda*posErr + velErr + alpha*phi + beta*phiDot
        // posErr/velErr are drone-based (stable), integral bias handles payload offset
        const s = this.lambda * posErr + velErr + this.alpha * phi + beta * phiDot;

        // Coefficients from linearized dynamics for equivalent control
        const coeff_F = 1.0 / (m_d + m_w) - beta / ((m_d + m_w) * L);

        // Guard against singularity (when beta ≈ L)
        if (Math.abs(coeff_F) < 1e-6) {
            return -this.kSwitch * this._sat(s);
        }

        // Terms not involving F in ds/dt:
        const c_phi = m_w * g / (m_d + m_w) - beta * (m_d + m_w) * g / ((m_d + m_w) * L);
        // Equivalent control from ds/dt = 0:
        const F_eq = -(this.lambda * velErr + this.alpha * phiDot + c_phi * phi) / coeff_F;

        // Total control: equivalent + switching
        return F_eq - this.kSwitch * this._sat(s);
    }

    _sat(s) {
        if (s > this.epsilon) return 1;
        if (s < -this.epsilon) return -1;
        return s / this.epsilon;
    }

    computeControl(state, ref, dt) {
        const { L, m_d, m_w, g, maxLateral, maxThrust } = this.params;

        // Integral of payload error for wind compensation
        const w_x = state[0] + L * Math.sin(state[6]);
        const w_y = state[2] + L * Math.sin(state[8]);
        this._integX += (ref.pos[0] - w_x) * dt;
        this._integY += (ref.pos[1] - w_y) * dt;
        this._integX = Math.max(-10, Math.min(10, this._integX));
        this._integY = Math.max(-10, Math.min(10, this._integY));
        const ki = this.lambda * 0.1;

        // Surface uses drone position/velocity (stable negative feedback)
        // Integral of payload error biases the reference to compensate wind offset
        const corrRefX = ref.pos[0] + ki * this._integX;
        const corrRefY = ref.pos[1] + ki * this._integY;

        let F_x = this._computeAxis(
            state[0] - corrRefX, state[1] - ref.vel[0], state[6], state[7], this.params
        );
        let F_y = this._computeAxis(
            state[2] - corrRefY, state[3] - ref.vel[1], state[8], state[9], this.params
        );

        // Vertical (simple PD)
        const ez = (ref.pos[2] + L) - state[4];
        let F_z = (m_d + m_w) * g + 50 * ez - 30 * state[5];

        F_x = Math.max(-maxLateral, Math.min(maxLateral, F_x));
        F_y = Math.max(-maxLateral, Math.min(maxLateral, F_y));
        F_z = Math.max(0, Math.min(maxThrust, F_z));

        return [F_x, F_y, F_z];
    }

    reset() {
        this._integX = 0;
        this._integY = 0;
    }
}

// ─── MPC Controller (finite-horizon LQR via discrete Riccati) ───────────────
// Computes gain by backward Riccati recursion over N steps. Caches gains and
// only recomputes when parameters change (_dirty flag).

class MPCController {
    constructor(params, dt) {
        this.params = params;
        this.dt = dt;
        this.horizon = 50;
        this.qPos = 10;
        this.rCost = 0.08;
        this._dirty = true;
        this._K0_lat = null;
        this._K0_vert = null;
        // Integral of payload position error
        this._integX = 0;
        this._integY = 0;
    }

    _recompute() {
        const { m_d, m_w, L, g } = this.params;
        const dt = this.dt;
        const N = this.horizon;

        // --- Augmented lateral subsystem (5x5) ---
        const A_lat = matFromArray(4, 4, [
            0, 1, 0, 0,
            0, 0, m_w * g / m_d, 0,
            0, 0, 0, 1,
            0, 0, -(m_d + m_w) * g / (m_d * L), 0,
        ]);
        const B_lat4 = matFromArray(4, 1, [0, 1 / m_d, 0, -1 / (m_d * L)]);

        // Build A_aug (5x5) = [A_lat | 0; C_pay | 0]
        const A_aug = matCreate(5, 5);
        for (let i = 0; i < 4; i++)
            for (let j = 0; j < 4; j++)
                matSet(A_aug, i, j, matGet(A_lat, i, j));
        matSet(A_aug, 4, 0, 1);
        matSet(A_aug, 4, 2, L);

        // Build B_aug (5x1) = [B_lat; 0]
        const B_aug = matCreate(5, 1);
        for (let i = 0; i < 4; i++)
            matSet(B_aug, i, 0, matGet(B_lat4, i, 0));

        // Discretize: Ad = I + A*dt, Bd = B*dt
        const I5 = matIdentity(5);
        const Ad_lat = matAdd(I5, matScale(A_aug, dt));
        const Bd_lat = matScale(B_aug, dt);

        // Q with payload-position cross-terms
        const qp = this.qPos;
        const qphi = qp * 0.3;
        const Q_lat = matFromArray(5, 5, [
            qp,     0,          qp*L,     0,                  0,
            0,      qp*0.25,    0,        qp*0.25*L,          0,
            qp*L,   0,          qp*L*L,   0,                  0,
            0,      qp*0.25*L,  0,        qp*0.25*L*L + qphi, 0,
            0,      0,          0,        0,                  qp*0.1,
        ]);
        const R_lat = matFromArray(1, 1, [this.rCost]);
        const Qf_lat = matScale(Q_lat, 2); // terminal cost

        // Backward Riccati
        let P = matCopy(Qf_lat);
        const Ad_latT = matTranspose(Ad_lat);
        const Bd_latT = matTranspose(Bd_lat);
        let K0_lat = null;

        for (let i = N - 1; i >= 0; i--) {
            const BtP = matMul(Bd_latT, P);
            const BtPBpR = matAdd(R_lat, matMul(BtP, Bd_lat));
            const BtPA = matMul(BtP, Ad_lat);
            const K = matMul(matInverse(BtPBpR), BtPA);
            if (i === 0) K0_lat = K;
            P = matAdd(Q_lat, matSub(matMul(Ad_latT, matMul(P, Ad_lat)), matMul(matMul(Ad_latT, matMul(P, Bd_lat)), K)));
        }

        // --- Vertical subsystem (2x2) --- unchanged
        const A_vert = matFromArray(2, 2, [0, 1, 0, 0]);
        const B_vert = matFromArray(2, 1, [0, 1 / (m_d + m_w)]);
        const I2 = matIdentity(2);
        const Ad_vert = matAdd(I2, matScale(A_vert, dt));
        const Bd_vert = matScale(B_vert, dt);
        const Q_vert = matFromArray(2, 2, [this.qPos * 25, 0, 0, this.qPos * 8]);
        const R_vert = matFromArray(1, 1, [this.rCost]);
        const Qf_vert = matScale(Q_vert, 2);

        let Pv = matCopy(Qf_vert);
        const Ad_vertT = matTranspose(Ad_vert);
        const Bd_vertT = matTranspose(Bd_vert);
        let K0_vert = null;

        for (let i = N - 1; i >= 0; i--) {
            const BtP = matMul(Bd_vertT, Pv);
            const BtPBpR = matAdd(R_vert, matMul(BtP, Bd_vert));
            const BtPA = matMul(BtP, Ad_vert);
            const K = matMul(matInverse(BtPBpR), BtPA);
            if (i === 0) K0_vert = K;
            Pv = matAdd(Q_vert, matSub(matMul(Ad_vertT, matMul(Pv, Ad_vert)), matMul(matMul(Ad_vertT, matMul(Pv, Bd_vert)), K)));
        }

        this._K0_lat = K0_lat;
        this._K0_vert = K0_vert;
        this._dirty = false;
    }

    computeControl(state, ref, dt) {
        if (this._dirty) this._recompute();

        const { L, m_d, m_w, g, maxLateral, maxThrust } = this.params;
        const K_lat = this._K0_lat;
        const K_vert = this._K0_vert;

        // Update integral of payload error (payload - ref, matching CARE sign convention)
        const w = weightPosition(state, L);
        this._integX += (w.x - ref.pos[0]) * dt;
        this._integY += (w.y - ref.pos[1]) * dt;
        this._integX = Math.max(-10, Math.min(10, this._integX));
        this._integY = Math.max(-10, Math.min(10, this._integY));

        // X-axis: 5-element error [x_d - payload_des, x_dot - vel_des, phi, phi_dot, xi]
        const x_err = [state[0] - ref.pos[0], state[1] - ref.vel[0], state[6], state[7], this._integX];
        let F_x = 0;
        for (let i = 0; i < 5; i++) F_x -= matGet(K_lat, 0, i) * x_err[i];

        // Y-axis: 5-element error
        const y_err = [state[2] - ref.pos[1], state[3] - ref.vel[1], state[8], state[9], this._integY];
        let F_y = 0;
        for (let i = 0; i < 5; i++) F_y -= matGet(K_lat, 0, i) * y_err[i];

        // Z-axis error
        const z_err = [state[4] - (ref.pos[2] + L), state[5] - ref.vel[2]];
        let F_z = 0;
        for (let i = 0; i < 2; i++) F_z -= matGet(K_vert, 0, i) * z_err[i];
        F_z += (m_d + m_w) * g;

        F_x = Math.max(-maxLateral, Math.min(maxLateral, F_x));
        F_y = Math.max(-maxLateral, Math.min(maxLateral, F_y));
        F_z = Math.max(0, Math.min(maxThrust, F_z));

        return [F_x, F_y, F_z];
    }

    markDirty() { this._dirty = true; }

    reset() {
        this._dirty = true;
        this._integX = 0;
        this._integY = 0;
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
        this.inputShaping = false;

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

        // Feedback Linearization controller
        this.feedbacklin = new FeedbackLinController(this.params);

        // Sliding Mode controller
        this.sliding = new SlidingModeController(this.params);

        // MPC controller
        this.mpc = new MPCController(this.params, this.dt);

        // ZVD Input Shaper
        this.zvdShaper = new ZVDShaper(this.params, this.dt);

        // Goal smoother and trajectory state
        this._refSmoother = new GoalSmoother3D([0, 0, 0], 3.0);
        this._trajectory = null;
        this._trajStartTime = 0;
        this._useTrajectory = false;

        // LQR integral state (payload position error integral)
        this._lqrIntX = 0;
        this._lqrIntY = 0;

        // Wind OU process state (smooth time-varying wind)
        this._windStrState = 0;   // current wind strength (OU state)
        this._windDirState = 0;   // current wind direction (OU state)

        // Wind state for visualization
        this.lastWind = { strength: 0, dir: 0 };
    }

    _updateWind() {
        const { windMean, windStddev, windDir, windWander } = this.params;
        const sqrtDt = Math.sqrt(this.dt);

        // Ornstein-Uhlenbeck process for wind strength (smooth variation)
        const thetaStr = 2.0; // mean-reversion rate (~0.5s time constant)
        if (windStddev > 0) {
            const u1 = Math.random(), u2 = Math.random();
            const z = Math.sqrt(-2 * Math.log(u1 + 1e-12)) * Math.cos(2 * Math.PI * u2);
            this._windStrState += thetaStr * (windMean - this._windStrState) * this.dt + windStddev * sqrtDt * z;
            this._windStrState = Math.max(0, this._windStrState);
        } else {
            this._windStrState = windMean;
        }

        // Ornstein-Uhlenbeck process for wind direction (smooth wander)
        const thetaDir = 1.0; // slower mean-reversion for direction
        if (windWander > 0) {
            const u1 = Math.random(), u2 = Math.random();
            const z = Math.sqrt(-2 * Math.log(u1 + 1e-12)) * Math.cos(2 * Math.PI * u2);
            // Compute shortest angular difference for mean-reversion
            let diff = windDir - this._windDirState;
            diff = Math.atan2(Math.sin(diff), Math.cos(diff));
            this._windDirState += thetaDir * diff * this.dt + windWander * sqrtDt * z;
        } else {
            this._windDirState = windDir;
        }

        this.params.windX = this._windStrState * Math.cos(this._windDirState);
        this.params.windY = this._windStrState * Math.sin(this._windDirState);
        this.lastWind = { strength: this._windStrState, dir: this._windDirState };
    }

    step() {
        this._updateWind();

        // Compute reference: trajectory mode or goal mode with smoother
        let ref;
        if (this._useTrajectory && this._trajectory) {
            const tLocal = this.time - this._trajStartTime;
            ref = this._trajectory.evaluate(tLocal);
            this.goal = [...ref.pos]; // keep goal in sync for HUD/sliders
        } else {
            // Goal mode: optionally apply ZVD, then smooth
            let goalPos = this.goal;
            if (this.inputShaping) {
                goalPos = this.zvdShaper.shapeGoal(this.goal, this.params);
            }
            ref = this._refSmoother.update(goalPos, this.dt);
        }

        // Update LQR integral state (payload - ref, matching CARE sign convention)
        if (this.controllerType === 'lqr') {
            const w = weightPosition(this.state, this.params.L);
            this._lqrIntX += (w.x - ref.pos[0]) * this.dt;
            this._lqrIntY += (w.y - ref.pos[1]) * this.dt;
            this._lqrIntX = Math.max(-10, Math.min(10, this._lqrIntX));
            this._lqrIntY = Math.max(-10, Math.min(10, this._lqrIntY));
        }

        let control;
        switch (this.controllerType) {
            case 'off':
                control = [0, 0, (this.params.m_d + this.params.m_w) * this.params.g];
                break;
            case 'pid':
                control = this.pid.computeControl(this.state, ref, this.dt);
                break;
            case 'cascade':
                control = this.cascade.computeControl(this.state, ref, this.dt);
                break;
            case 'flatness':
                control = this.flatness.computeControl(this.state, ref, this.dt);
                break;
            case 'feedbacklin':
                control = this.feedbacklin.computeControl(this.state, ref, this.dt);
                break;
            case 'sliding':
                control = this.sliding.computeControl(this.state, ref, this.dt);
                break;
            case 'mpc':
                control = this.mpc.computeControl(this.state, ref, this.dt);
                break;
            default: // 'lqr'
                control = computeLqrControl(this.state, ref, this.K_lat, this.K_vert, this.params, this._lqrIntX, this._lqrIntY);
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
        this._useTrajectory = false;
        this._trajectory = null;
    }

    setTrajectory(traj, preserveProgress) {
        if (preserveProgress && this._trajectory && this._useTrajectory) {
            // Compute fractional progress through old trajectory and map to new
            const oldElapsed = this.time - this._trajStartTime;
            const oldDur = this._trajectory.duration;
            const frac = oldDur > 0 ? (oldElapsed % oldDur) / oldDur : 0;
            const newOffset = frac * traj.duration;
            this._trajStartTime = this.time - newOffset;
        } else {
            this._trajStartTime = this.time;
        }
        this._trajectory = traj;
        this._useTrajectory = true;
    }

    clearTrajectory() {
        if (this._trajectory && this._useTrajectory) {
            const t = this.time - this._trajStartTime;
            const ref = this._trajectory.evaluate(t);
            this.goal = [...ref.pos];
            this._refSmoother.reset(ref.pos);
        }
        this._trajectory = null;
        this._useTrajectory = false;
    }

    setSmootherOmega(omega) {
        this._refSmoother.setOmega(omega);
    }

    setControllerType(type) {
        this.controllerType = type;
        // Reset all controller integral states on switch
        this._lqrIntX = 0;
        this._lqrIntY = 0;
        this.pid.reset();
        this.cascade.reset();
        this.flatness.reset();
        this.feedbacklin.reset();
        this.mpc.reset();
    }

    setSwingDamping(enabled) {
        this.swingDamping = enabled;
    }

    setInputShaping(enabled) {
        this.inputShaping = enabled;
        if (enabled) this.zvdShaper.reset(this.params);
    }

    setControllerParams(type, p) {
        switch (type) {
            case 'lqr': {
                const L = this.params.L;
                const qint = p.qpos * 0.1;
                const Qlat = matFromArray(5, 5, [
                    p.qpos,     0,              p.qpos*L,     0,                          0,
                    0,          p.qpos*0.25,    0,            p.qpos*0.25*L,              0,
                    p.qpos*L,   0,              p.qpos*L*L,   0,                          0,
                    0,          p.qpos*0.25*L,  0,            p.qpos*0.25*L*L + p.qphi,   0,
                    0,          0,              0,            0,                          qint,
                ]);
                const Rlat = matFromArray(1, 1, [p.rcost]);
                // Vertical scaling compensates for lower qpos (payload Q amplifies lateral by L²)
                const Qvert = matFromArray(2, 2, [p.qpos * 25, 0, 0, p.qpos * 8]);
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
                this.flatness.kp = p.kp;
                this.flatness.kd = p.kp * 0.67;
                this.flatness.kp_phi = p.kp_phi;
                this.flatness.kd_phi = p.kp_phi * 0.4;
                this.flatness.kp_z = p.kp * 4;
                this.flatness.kd_z = p.kp * 2.5;
                break;
            case 'feedbacklin':
                this.feedbacklin.kp = p.kp;
                this.feedbacklin.ka = p.ka;
                this.feedbacklin.kb = p.kb;
                break;
            case 'sliding':
                this.sliding.lambda = p.lambda;
                this.sliding.alpha = p.alpha;
                this.sliding.kSwitch = p.kSwitch;
                this.sliding.epsilon = p.epsilon;
                break;
            case 'mpc':
                this.mpc.horizon = Math.round(p.horizon);
                this.mpc.qPos = p.qPos;
                this.mpc.rCost = p.rCost;
                this.mpc.markDirty();
                break;
        }
    }

    setAggression(aggr) {
        aggr = Math.max(0.05, Math.min(1.0, aggr));

        // Quadratic scaling: gentle at low aggression, much stronger at high
        const qScale = aggr * aggr;
        const rScale = 1 / (aggr * aggr);
        const L = this.params.L;

        // Re-compute LQR gains with scaled Q/R (5x5 payload-tracking)
        const qpos = 10 * qScale;
        const qphi = 30 * qScale;
        const qint = qpos * 0.1;
        const Qlat = matFromArray(5, 5, [
            qpos,     0,            qpos*L,     0,                    0,
            0,        qpos*0.25,    0,          qpos*0.25*L,          0,
            qpos*L,   0,            qpos*L*L,   0,                    0,
            0,        qpos*0.25*L,  0,          qpos*0.25*L*L + qphi, 0,
            0,        0,            0,          0,                    qint,
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
        this.feedbacklin.params = this.params;
        this.sliding.params = this.params;
        this.mpc.params = this.params;
        this.mpc.markDirty();
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
        this.flatness.reset();
        this.feedbacklin.reset();
        this.sliding.reset();
        this.mpc.reset();
        this.zvdShaper.reset(this.params);
        this._refSmoother.reset([0, 0, 0]);
        this._trajectory = null;
        this._useTrajectory = false;
        this._lqrIntX = 0;
        this._lqrIntY = 0;
        this._windStrState = this.params.windMean;
        this._windDirState = this.params.windDir;
    }
}
