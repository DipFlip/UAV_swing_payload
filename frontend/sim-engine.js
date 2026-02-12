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

    // Vertical subsystem — coupled to pendulum via rope tension
    // h = L*cos(phi_x)*cos(phi_y) is the vertical hang distance.
    // d²h/dt² contributes centripetal and angular-acceleration forces.
    const cxcy = cos_px * cos_py;
    const d2h = -L * (
        cxcy * (phix_dot * phix_dot + phiy_dot * phiy_dot)
        - 2 * sin_px * sin_py * phix_dot * phiy_dot
        + sin_px * cos_py * phix_dd
        + cos_px * sin_py * phiy_dd
    );
    const z_dd = (F_z - (m_d + m_w) * g + m_w * d2h) / (m_d + m_w);

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
        this._integX = Math.max(-20, Math.min(20, this._integX));
        this._integY = Math.max(-20, Math.min(20, this._integY));
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
        // Tunable parameters
        this.omega_c = 1.0;    // closed-loop bandwidth (rad/s)
        this.zeta = 1.0;       // damping ratio (1.0 = critically damped)
        // Integral of flat output error for wind disturbance rejection
        this._integX = 0;
        this._integY = 0;
        // Disturbance observer: estimates wind bias in y_ddot
        this._yDotPrevX = 0;
        this._yDotPrevY = 0;
        this._windBiasX = 0;
        this._windBiasY = 0;
        this._firstStep = true;
    }

    // Compute lateral force for one axis using differential flatness.
    //
    // Flat output: y = x_d + L*phi  (linearized payload position)
    // Derivatives:  y_dot = x_dot + L*phi_dot
    //               y_ddot = -g*phi + wind_bias  (model + DOB correction)
    //               y_dddot = -g*phi_dot  (correct under constant wind)
    // The force appears in y_ddddot (relative degree 4):
    //   y_ddddot = g*F/(m_d*L) - omega_n^2 * y_ddot + disturbance
    // Inversion: F = m_d*L/g * (v + omega_n^2 * y_ddot)
    //   where v is the virtual input chosen by pole placement.
    _computeAxisForce(y, y_dot, y_ddot, phi_dot, ref_pos, ref_vel, ref_acc, integ) {
        const { L, g, m_d, m_w } = this.params;
        const wc = this.omega_c;
        const z = this.zeta;
        const a = 0.5;  // integral pole at a*wc (relative to main poles)

        // 5th-order pole placement: (s^2 + 2*z*wc*s + wc^2)^2 * (s + a*wc)
        // All 5 gains designed together for well-damped 5th-order response.
        const wc2 = wc * wc;
        const wc3 = wc2 * wc;
        const wc4 = wc2 * wc2;
        const wc5 = wc4 * wc;
        const k0 = wc4 * (1 + 4 * z * a);
        const k1 = wc3 * (4 * z + (4 * z * z + 2) * a);
        const k2 = wc2 * (4 * z * z + 2 + 4 * z * a);
        const k3 = wc * (4 * z + a);
        const ki = a * wc5;

        // y_dddot: model-based, correct under constant wind since
        // d(Fw/m_w)/dt = 0 → y_dddot = dy_ddot/dt = -g*phi_dot
        const y_dddot = -g * phi_dot;

        // Virtual input with integral action for wind disturbance rejection.
        const v = -k0 * (y - ref_pos)
                  - k1 * (y_dot - ref_vel)
                  - k2 * (y_ddot - ref_acc)
                  - k3 * y_dddot
                  + ki * integ;

        // Force inversion (y_ddot includes wind bias from DOB)
        const omega_n_sq = (m_d + m_w) * g / (m_d * L);
        return m_d * L / g * (v + omega_n_sq * y_ddot);
    }

    computeControl(state, ref, dt) {
        const { L, g, m_d, m_w, maxLateral, maxThrust } = this.params;

        // Flat output and its velocity (measured from state)
        const y_x = state[0] + L * state[6];
        const y_y = state[2] + L * state[8];
        const yDot_x = state[1] + L * state[7];
        const yDot_y = state[3] + L * state[9];

        // Disturbance observer for wind estimation.
        // Model-based y_ddot = -g*phi is correct without wind but misses
        // the Fw/m_w term under wind. The DOB estimates this bias by
        // filtering the difference between numerical and model y_ddot.
        // Uses slow filter (wf = wc/2) to reject transient noise while
        // capturing the quasi-DC wind offset.
        if (this._firstStep) {
            this._firstStep = false;
        } else {
            const yDdotRaw_x = (yDot_x - this._yDotPrevX) / dt;
            const yDdotRaw_y = (yDot_y - this._yDotPrevY) / dt;
            const yDdotModel_x = -g * state[6];
            const yDdotModel_y = -g * state[8];
            // LPF on (numerical - model) → converges to Fw/m_w under wind
            const wf = this.omega_c * 0.5;
            const alpha = dt * wf / (1 + dt * wf);
            this._windBiasX += alpha * ((yDdotRaw_x - yDdotModel_x) - this._windBiasX);
            this._windBiasY += alpha * ((yDdotRaw_y - yDdotModel_y) - this._windBiasY);
        }
        this._yDotPrevX = yDot_x;
        this._yDotPrevY = yDot_y;

        // Corrected y_ddot: model dynamics + estimated wind bias
        const yDdot_x = -g * state[6] + this._windBiasX;
        const yDdot_y = -g * state[8] + this._windBiasY;

        // Integral of flat output error for residual disturbance rejection
        this._integX += (ref.pos[0] - y_x) * dt;
        this._integY += (ref.pos[1] - y_y) * dt;
        this._integX = Math.max(-20, Math.min(20, this._integX));
        this._integY = Math.max(-20, Math.min(20, this._integY));

        let F_x = this._computeAxisForce(
            y_x, yDot_x, yDdot_x, state[7],
            ref.pos[0], ref.vel[0], ref.acc[0], this._integX
        );
        let F_y = this._computeAxisForce(
            y_y, yDot_y, yDdot_y, state[9],
            ref.pos[1], ref.vel[1], ref.acc[1], this._integY
        );

        // Vertical (simple PD, no pendulum coupling needed)
        const zDes = ref.pos[2] + L;
        let F_z = (m_d + m_w) * g + 50 * (zDes - state[4]) - 30 * state[5];

        F_x = Math.max(-maxLateral, Math.min(maxLateral, F_x));
        F_y = Math.max(-maxLateral, Math.min(maxLateral, F_y));
        F_z = Math.max(0, Math.min(maxThrust, F_z));

        return [F_x, F_y, F_z];
    }

    reset() {
        this._integX = 0;
        this._integY = 0;
        this._yDotPrevX = 0;
        this._yDotPrevY = 0;
        this._windBiasX = 0;
        this._windBiasY = 0;
        this._firstStep = true;
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

// ─── Sliding Mode Controller ────────────────────────────────────────────────
// Sliding surface + equivalent control + bounded switching term.

class SlidingModeController {
    constructor(params) {
        this.params = params;
        this.lambda = 3;
        this.alpha = 80;       // swing damping: F_damp = alpha*phiDot
        this.kSwitch = 20;     // switching gain
        this.epsilon = 3.0;    // wide boundary layer for smooth control
        // Integral of payload position error for wind compensation
        this._integX = 0;
        this._integY = 0;
    }

    _computeAxis(dronePos, droneVel, phi, phiDot, refPos, refVel, params) {
        const { m_d, m_w, L, g } = params;

        const posErr = dronePos - refPos;
        const velErr = droneVel - refVel;

        // Sliding surface on drone position only — no angle terms.
        // For a hanging pendulum, mixing angle into the surface creates
        // conflicting objectives: the equivalent control fights gravity
        // coupling, destabilizing the swing.
        const s = this.lambda * posErr + velErr;

        // Partial equivalent control — velocity damping only.
        // We deliberately do NOT cancel the gravity coupling (m_w*g*phi)
        // in x_dd: for a hanging pendulum, this coupling naturally pulls
        // the drone toward the weight, which damps the swing. Canceling it
        // would push the drone AWAY from the weight, destabilizing it.
        const F_eq = -m_d * this.lambda * velErr;

        // Switching term for robustness (disturbance rejection)
        const F_sw = -this.kSwitch * this._sat(s);

        // Explicit swing damping: push drone to follow weight velocity.
        // Positive phiDot (weight swinging +x) → positive F → negative phi_dd
        // (decelerates swing). This is additive, not part of the surface.
        const F_damp = this.alpha * phiDot;

        return F_eq + F_sw + F_damp;
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
        this._integX = Math.max(-20, Math.min(20, this._integX));
        this._integY = Math.max(-20, Math.min(20, this._integY));
        const ki = this.lambda * 0.2;

        // Surface uses drone position/velocity (stable negative feedback)
        // Integral of payload error biases the reference to compensate wind offset
        const corrRefX = ref.pos[0] + ki * this._integX;
        const corrRefY = ref.pos[1] + ki * this._integY;

        let F_x = this._computeAxis(
            state[0], state[1], state[6], state[7], corrRefX, ref.vel[0], this.params
        );
        let F_y = this._computeAxis(
            state[2], state[3], state[8], state[9], corrRefY, ref.vel[1], this.params
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
        const R_val = this.rCost;

        // Converge Riccati to steady state (DARE solution)
        const Ad_latT = matTranspose(Ad_lat);
        const Bd_latT = matTranspose(Bd_lat);
        const Bd_vec = new Float64Array(5);
        for (let i = 0; i < 5; i++) Bd_vec[i] = matGet(Bd_lat, i, 0);

        let P = matCopy(Q_lat);
        for (let iter = 0; iter < 2000; iter++) {
            const BtP = matMul(Bd_latT, P);
            let BtPB = 0;
            for (let i = 0; i < 5; i++) BtPB += matGet(BtP, 0, i) * Bd_vec[i];
            const Sinv = 1 / (R_val + BtPB);
            const PBd = matMul(P, Bd_lat);
            const correction = matScale(matMul(PBd, matTranspose(PBd)), Sinv);
            const P_new = matAdd(Q_lat, matMul(matMul(Ad_latT, matSub(P, correction)), Ad_lat));
            // Check convergence
            let maxDiff = 0;
            for (let i = 0; i < 25; i++) maxDiff = Math.max(maxDiff, Math.abs(P_new.d[i] - P.d[i]));
            P = P_new;
            if (maxDiff < 1e-10) break;
        }

        // Extract converged gain K = (R + B'PB)^{-1} B'P Ad
        const BtP = matMul(Bd_latT, P);
        let BtPB = 0;
        for (let i = 0; i < 5; i++) BtPB += matGet(BtP, 0, i) * Bd_vec[i];
        this._K0_lat = matScale(matMul(BtP, Ad_lat), 1 / (R_val + BtPB));

        // --- Vertical subsystem (2x2) ---
        const A_vert = matFromArray(2, 2, [0, 1, 0, 0]);
        const B_vert = matFromArray(2, 1, [0, 1 / (m_d + m_w)]);
        const I2 = matIdentity(2);
        const Ad_vert = matAdd(I2, matScale(A_vert, dt));
        const Bd_vert = matScale(B_vert, dt);
        const Q_vert = matFromArray(2, 2, [this.qPos * 25, 0, 0, this.qPos * 8]);
        const R_vert = matFromArray(1, 1, [this.rCost]);

        let Pv = matCopy(Q_vert);
        const Ad_vertT = matTranspose(Ad_vert);
        const Bd_vertT = matTranspose(Bd_vert);

        for (let iter = 0; iter < 2000; iter++) {
            const BtPv = matMul(Bd_vertT, Pv);
            const BtPBpR = matAdd(R_vert, matMul(BtPv, Bd_vert));
            const Kv = matMul(matInverse(BtPBpR), matMul(BtPv, Ad_vert));
            const Pv_new = matAdd(Q_vert, matSub(
                matMul(Ad_vertT, matMul(Pv, Ad_vert)),
                matMul(matMul(Ad_vertT, matMul(Pv, Bd_vert)), Kv)
            ));
            let maxDiff = 0;
            for (let i = 0; i < 4; i++) maxDiff = Math.max(maxDiff, Math.abs(Pv_new.d[i] - Pv.d[i]));
            Pv = Pv_new;
            if (maxDiff < 1e-10) break;
        }

        const BtPv = matMul(Bd_vertT, Pv);
        this._K0_vert = matMul(matInverse(matAdd(R_vert, matMul(BtPv, Bd_vert))), matMul(BtPv, Ad_vert));
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
        this._integX = Math.max(-20, Math.min(20, this._integX));
        this._integY = Math.max(-20, Math.min(20, this._integY));

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

// ─── Trajectory-Preview MPC (time-varying affine LQR) ───────────────────────
// True model-predictive controller that looks ahead along the known trajectory.
// Caches Riccati matrices (P, K, A_cl) once when params change (heavy pass),
// then runs a lightweight affine backward pass each frame using future reference
// points to compute a feedforward term that anticipates upcoming turns.

class TrajectoryMPCController {
    constructor(params, dt) {
        this.params = params;
        this.dt = dt;
        this.horizon = 100;   // 2 seconds of preview at 50Hz
        this.qPos = 10;
        this.rCost = 0.08;
        this._dirty = true;

        // Cached matrices (recomputed only when params change)
        this._Ad = null;         // 5x5 discrete A
        this._Bd = null;         // 5x1 discrete B
        this._Bd_vec = null;     // Float64Array(5) column of Bd
        this._P_cache = null;    // Array[N+1] of 5x5 P matrices
        this._K_cache = null;    // Array[N] of 1x5 K matrices
        this._Sinv_cache = null; // Float64Array(N) of scalar S^{-1}
        this._AclT_cache = null; // Array[N] of 5x5 (A-BK)^T matrices

        // Vertical gains (LTI, no preview needed)
        this._K_vert = null;

        // Trajectory reference (set by Simulation before computeControl)
        this._trajectory = null;
        this._trajTime = 0;

        // Integral state
        this._integX = 0;
        this._integY = 0;
    }

    _recompute() {
        const { m_d, m_w, L, g } = this.params;
        const dt = this.dt;
        const N = this.horizon;

        // --- Augmented lateral subsystem (5x5): [x_d, x_dot, phi, phi_dot, xi] ---
        const A_lat = matFromArray(4, 4, [
            0, 1, 0, 0,
            0, 0, m_w * g / m_d, 0,
            0, 0, 0, 1,
            0, 0, -(m_d + m_w) * g / (m_d * L), 0,
        ]);
        const B_lat4 = matFromArray(4, 1, [0, 1 / m_d, 0, -1 / (m_d * L)]);

        const A_aug = matCreate(5, 5);
        for (let i = 0; i < 4; i++)
            for (let j = 0; j < 4; j++)
                matSet(A_aug, i, j, matGet(A_lat, i, j));
        matSet(A_aug, 4, 0, 1);
        matSet(A_aug, 4, 2, L);

        const B_aug = matCreate(5, 1);
        for (let i = 0; i < 4; i++)
            matSet(B_aug, i, 0, matGet(B_lat4, i, 0));

        // Discretize: Ad = I + A*dt, Bd = B*dt
        const I5 = matIdentity(5);
        this._Ad = matAdd(I5, matScale(A_aug, dt));
        this._Bd = matScale(B_aug, dt);
        this._Bd_vec = new Float64Array(5);
        for (let i = 0; i < 5; i++) this._Bd_vec[i] = matGet(this._Bd, i, 0);

        const AdT = matTranspose(this._Ad);

        // Q with payload-position cross-terms (same structure as MPC/LQR)
        const qp = this.qPos;
        const qphi = qp * 0.3;
        const Q = matFromArray(5, 5, [
            qp,     0,          qp*L,     0,                  0,
            0,      qp*0.25,    0,        qp*0.25*L,          0,
            qp*L,   0,          qp*L*L,   0,                  0,
            0,      qp*0.25*L,  0,        qp*0.25*L*L + qphi, 0,
            0,      0,          0,        0,                  qp*0.1,
        ]);
        this._Q = Q;
        this._R_val = this.rCost;

        // Converge Riccati to steady state (DARE) for terminal cost.
        // Without this, short preview horizons give grossly under-converged
        // gains because the integral state needs ~500 steps to reach steady state.
        const BdT = matTranspose(this._Bd);
        let P_ss = matCopy(Q);
        for (let iter = 0; iter < 2000; iter++) {
            const BtP = matMul(BdT, P_ss);
            let BtPB = 0;
            for (let i = 0; i < 5; i++) BtPB += matGet(BtP, 0, i) * this._Bd_vec[i];
            const Sinv = 1 / (this._R_val + BtPB);
            const PBd = matMul(P_ss, this._Bd);
            const correction = matScale(matMul(PBd, matTranspose(PBd)), Sinv);
            const P_new = matAdd(Q, matMul(matMul(AdT, matSub(P_ss, correction)), this._Ad));
            let maxDiff = 0;
            for (let i = 0; i < 25; i++) maxDiff = Math.max(maxDiff, Math.abs(P_new.d[i] - P_ss.d[i]));
            P_ss = P_new;
            if (maxDiff < 1e-10) break;
        }

        // Backward Riccati pass — cache P, K, Sinv, AclT for preview horizon.
        // Terminal cost P[N] = P_∞ (converged DARE) ensures correct gains
        // even with short preview windows.
        this._P_cache = new Array(N + 1);
        this._K_cache = new Array(N);
        this._Sinv_cache = new Float64Array(N);
        this._AclT_cache = new Array(N);

        this._P_cache[N] = P_ss;

        for (let k = N - 1; k >= 0; k--) {
            const P = this._P_cache[k + 1];

            // BtP = Bd^T * P (1x5)
            const BtP = matMul(BdT, P);

            // S = R + B^T P B (scalar)
            let BtPB = 0;
            for (let i = 0; i < 5; i++) BtPB += matGet(BtP, 0, i) * this._Bd_vec[i];
            const S = this._R_val + BtPB;
            const Sinv = 1 / S;
            this._Sinv_cache[k] = Sinv;

            // K = Sinv * BtP * Ad (1x5)
            const K = matScale(matMul(BtP, this._Ad), Sinv);
            this._K_cache[k] = K;

            // P_k = Q + Ad^T (P - Sinv * PBd PBd^T) Ad
            const PBd = matMul(P, this._Bd);
            const correction = matScale(matMul(PBd, matTranspose(PBd)), Sinv);
            const P_minus = matSub(P, correction);
            this._P_cache[k] = matAdd(Q, matMul(matMul(AdT, P_minus), this._Ad));

            // A_cl^T = (Ad - Bd * K)^T
            const Acl = matCopy(this._Ad);
            for (let i = 0; i < 5; i++)
                for (let j = 0; j < 5; j++)
                    Acl.d[i * 5 + j] -= this._Bd_vec[i] * matGet(K, 0, j);
            this._AclT_cache[k] = matTranspose(Acl);
        }

        // --- Vertical subsystem (2x2, converge to DARE steady state) ---
        const A_vert = matFromArray(2, 2, [0, 1, 0, 0]);
        const B_vert = matFromArray(2, 1, [0, 1 / (m_d + m_w)]);
        const I2 = matIdentity(2);
        const Ad_vert = matAdd(I2, matScale(A_vert, dt));
        const Bd_vert = matScale(B_vert, dt);
        const Q_vert = matFromArray(2, 2, [this.qPos * 25, 0, 0, this.qPos * 8]);
        const R_vert = matFromArray(1, 1, [this.rCost]);

        let Pv = matCopy(Q_vert);
        const Ad_vertT = matTranspose(Ad_vert);
        const Bd_vertT = matTranspose(Bd_vert);

        for (let iter = 0; iter < 2000; iter++) {
            const BtP = matMul(Bd_vertT, Pv);
            const BtPBpR = matAdd(R_vert, matMul(BtP, Bd_vert));
            const Kv = matMul(matInverse(BtPBpR), matMul(BtP, Ad_vert));
            const Pv_new = matAdd(Q_vert, matSub(
                matMul(Ad_vertT, matMul(Pv, Ad_vert)),
                matMul(matMul(Ad_vertT, matMul(Pv, Bd_vert)), Kv)
            ));
            let maxDiff = 0;
            for (let i = 0; i < 4; i++) maxDiff = Math.max(maxDiff, Math.abs(Pv_new.d[i] - Pv.d[i]));
            Pv = Pv_new;
            if (maxDiff < 1e-10) break;
        }

        const BtPv_final = matMul(Bd_vertT, Pv);
        this._K_vert = matMul(matInverse(matAdd(R_vert, matMul(BtPv_final, Bd_vert))), matMul(BtPv_final, Ad_vert));

        this._dirty = false;
    }

    /**
     * Solve one lateral axis with trajectory preview.
     * Runs the lightweight affine backward pass using cached Riccati matrices.
     * @param {number[]} x_state - [pos, vel, phi, phiDot, integ]
     * @param {Object[]} refs - Array of N+1 trajectory reference objects {pos, vel, acc}
     * @param {number} axis - 0 for X, 1 for Y
     * @returns {number} - optimal force for this axis
     */
    _solveAxisPreview(x_state, refs, axis) {
        const Ad = this._Ad;
        const Bd_vec = this._Bd_vec;
        const N = Math.min(this.horizon, refs.length - 1);

        // Affine backward pass: compute feedforward v_0 from future reference evolution
        // s_k = A_cl_k^T * (P_{k+1} * d_k + s_{k+1})
        // v_k = Sinv_k * Bd^T * (P_{k+1} * d_k + s_{k+1})
        // d_k = Ad * r_k - r_{k+1}  (reference mismatch at step k)
        let s = new Float64Array(5);  // s_N = 0
        let v0 = 0;

        for (let k = N - 1; k >= 0; k--) {
            const P = this._P_cache[k + 1];
            const Sinv = this._Sinv_cache[k];
            const AclT = this._AclT_cache[k];

            // Reference states: r_k = [pos, vel, 0, 0, 0]
            const r_pos = refs[k].pos[axis];
            const r_vel = refs[k].vel[axis];
            const r1_pos = refs[k + 1].pos[axis];
            const r1_vel = refs[k + 1].vel[axis];

            // d_k = Ad * r_k - r_{k+1} (only pos/vel elements of r_k are non-zero)
            const d = new Float64Array(5);
            for (let i = 0; i < 5; i++) {
                d[i] = matGet(Ad, i, 0) * r_pos + matGet(Ad, i, 1) * r_vel;
            }
            d[0] -= r1_pos;
            d[1] -= r1_vel;

            // Pd_plus_s = P_{k+1} * d_k + s_{k+1}
            const Pd_plus_s = new Float64Array(5);
            for (let i = 0; i < 5; i++) {
                let sum = s[i];
                for (let j = 0; j < 5; j++) sum += matGet(P, i, j) * d[j];
                Pd_plus_s[i] = sum;
            }

            // v_k = Sinv * Bd^T * Pd_plus_s (scalar)
            let Bt_Pds = 0;
            for (let i = 0; i < 5; i++) Bt_Pds += Bd_vec[i] * Pd_plus_s[i];
            const v_k = Sinv * Bt_Pds;

            if (k === 0) v0 = v_k;

            // s_k = A_cl_k^T * Pd_plus_s
            const s_new = new Float64Array(5);
            for (let i = 0; i < 5; i++) {
                let sum = 0;
                for (let j = 0; j < 5; j++) sum += matGet(AclT, i, j) * Pd_plus_s[j];
                s_new[i] = sum;
            }
            s = s_new;
        }

        // Control: u = -K_0 * (x - r_0) - v_0
        const K0 = this._K_cache[0];
        const r0_pos = refs[0].pos[axis];
        const r0_vel = refs[0].vel[axis];

        let u = -v0;
        u -= matGet(K0, 0, 0) * (x_state[0] - r0_pos);
        u -= matGet(K0, 0, 1) * (x_state[1] - r0_vel);
        u -= matGet(K0, 0, 2) * x_state[2];
        u -= matGet(K0, 0, 3) * x_state[3];
        u -= matGet(K0, 0, 4) * x_state[4];

        return u;
    }

    setTrajectory(traj, trajTime) {
        this._trajectory = traj;
        this._trajTime = trajTime;
    }

    computeControl(state, ref, dt) {
        if (this._dirty) this._recompute();

        const { L, m_d, m_w, g, maxLateral, maxThrust } = this.params;

        // Update integral of payload error
        const w = weightPosition(state, L);
        this._integX += (w.x - ref.pos[0]) * dt;
        this._integY += (w.y - ref.pos[1]) * dt;
        this._integX = Math.max(-20, Math.min(20, this._integX));
        this._integY = Math.max(-20, Math.min(20, this._integY));

        let F_x, F_y;

        if (this._trajectory) {
            // Sample future reference points along trajectory
            const N = this.horizon;
            const refs = new Array(N + 1);
            for (let k = 0; k <= N; k++) {
                refs[k] = this._trajectory.evaluate(this._trajTime + k * this.dt);
            }

            // Solve each lateral axis with trajectory preview
            F_x = this._solveAxisPreview(
                [state[0], state[1], state[6], state[7], this._integX],
                refs, 0
            );
            F_y = this._solveAxisPreview(
                [state[2], state[3], state[8], state[9], this._integY],
                refs, 1
            );
        } else {
            // No trajectory: fall back to constant-reference (same as regular MPC)
            const K0 = this._K_cache[0];
            const x_err = [state[0] - ref.pos[0], state[1] - ref.vel[0], state[6], state[7], this._integX];
            F_x = 0;
            for (let i = 0; i < 5; i++) F_x -= matGet(K0, 0, i) * x_err[i];

            const y_err = [state[2] - ref.pos[1], state[3] - ref.vel[1], state[8], state[9], this._integY];
            F_y = 0;
            for (let i = 0; i < 5; i++) F_y -= matGet(K0, 0, i) * y_err[i];
        }

        // Vertical (no preview, simple cached gain)
        const z_err = [state[4] - (ref.pos[2] + L), state[5] - ref.vel[2]];
        let F_z = 0;
        for (let i = 0; i < 2; i++) F_z -= matGet(this._K_vert, 0, i) * z_err[i];
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
        this._trajectory = null;
        this._trajTime = 0;
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

        // Sliding Mode controller
        this.sliding = new SlidingModeController(this.params);

        // MPC controller
        this.mpc = new MPCController(this.params, this.dt);

        // Trajectory-preview MPC controller
        this.trajmpc = new TrajectoryMPCController(this.params, this.dt);

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

        // All wind changes (slider changes AND stochastic fluctuations) transition
        // smoothly over ~3 seconds via OU mean-reversion.
        const theta = 1 / 3;  // mean-reversion rate (3s time constant)

        // Wind strength: smooth transition toward windMean, OU noise when stddev > 0
        this._windStrState += theta * (windMean - this._windStrState) * this.dt;
        if (windStddev > 0) {
            const u1 = Math.random(), u2 = Math.random();
            const z = Math.sqrt(-2 * Math.log(u1 + 1e-12)) * Math.cos(2 * Math.PI * u2);
            this._windStrState += windStddev * sqrtDt * z;
        }
        this._windStrState = Math.max(0, this._windStrState);

        // Wind direction: smooth transition toward windDir, OU wander when enabled
        let diff = windDir - this._windDirState;
        diff = Math.atan2(Math.sin(diff), Math.cos(diff));
        this._windDirState += theta * diff * this.dt;
        if (windWander > 0) {
            const u1 = Math.random(), u2 = Math.random();
            const z = Math.sqrt(-2 * Math.log(u1 + 1e-12)) * Math.cos(2 * Math.PI * u2);
            this._windDirState += windWander * sqrtDt * z;
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
            this._lqrIntX = Math.max(-20, Math.min(20, this._lqrIntX));
            this._lqrIntY = Math.max(-20, Math.min(20, this._lqrIntY));
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
            case 'sliding':
                control = this.sliding.computeControl(this.state, ref, this.dt);
                break;
            case 'mpc':
                control = this.mpc.computeControl(this.state, ref, this.dt);
                break;
            case 'trajmpc':
                if (this._useTrajectory && this._trajectory) {
                    this.trajmpc.setTrajectory(this._trajectory, this.time - this._trajStartTime);
                } else {
                    this.trajmpc.setTrajectory(null, 0);
                }
                control = this.trajmpc.computeControl(this.state, ref, this.dt);
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
        this.sliding.reset();
        this.mpc.reset();
        this.trajmpc.reset();
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
                this.flatness.omega_c = p.omega_c;
                this.flatness.zeta = p.zeta;
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
            case 'trajmpc':
                this.trajmpc.horizon = Math.round(p.horizon);
                this.trajmpc.qPos = p.qPos;
                this.trajmpc.rCost = p.rCost;
                this.trajmpc.markDirty();
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
        this.sliding.params = this.params;
        this.mpc.params = this.params;
        this.mpc.markDirty();
        this.trajmpc.params = this.params;
        this.trajmpc.markDirty();
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
        this.sliding.reset();
        this.mpc.reset();
        this.trajmpc.reset();
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
