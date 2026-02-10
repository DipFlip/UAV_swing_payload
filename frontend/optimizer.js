/**
 * Auto-tuning optimizer for controller parameters.
 * Uses Nelder-Mead simplex optimization on a headless simulation cost function.
 */

import { Simulation } from './sim-engine.js';

// ─── Nelder-Mead Simplex Optimizer ──────────────────────────────────────────

function nelderMead(costFn, x0, bounds, opts = {}) {
    const n = x0.length;
    const maxIter = opts.maxIter || 200;
    const tol = opts.tol || 1e-8;
    const alpha = 1.0;   // reflection
    const gamma = 2.0;   // expansion
    const rho = 0.5;     // contraction
    const sigma = 0.5;   // shrink

    function clamp(x) {
        return x.map((v, i) => Math.max(bounds[i][0], Math.min(bounds[i][1], v)));
    }

    // Build initial simplex: x0 + perturbations along each axis
    const simplex = [{ x: clamp([...x0]), cost: 0 }];
    simplex[0].cost = costFn(simplex[0].x);

    for (let i = 0; i < n; i++) {
        const xi = [...x0];
        const range = bounds[i][1] - bounds[i][0];
        xi[i] += range * 0.1;
        const xc = clamp(xi);
        simplex.push({ x: xc, cost: costFn(xc) });
    }

    function centroid(exclude) {
        const c = new Array(n).fill(0);
        for (let i = 0; i <= n; i++) {
            if (i === exclude) continue;
            for (let j = 0; j < n; j++) c[j] += simplex[i].x[j];
        }
        for (let j = 0; j < n; j++) c[j] /= n;
        return c;
    }

    let iters = 0;
    for (; iters < maxIter; iters++) {
        // Sort by cost
        simplex.sort((a, b) => a.cost - b.cost);

        // Convergence check
        const spread = simplex[n].cost - simplex[0].cost;
        if (spread < tol) break;

        const worst = n;
        const secondWorst = n - 1;
        const c = centroid(worst);

        // Reflection
        const xr = clamp(c.map((ci, j) => ci + alpha * (ci - simplex[worst].x[j])));
        const fr = costFn(xr);

        if (fr < simplex[secondWorst].cost && fr >= simplex[0].cost) {
            simplex[worst] = { x: xr, cost: fr };
            continue;
        }

        if (fr < simplex[0].cost) {
            // Expansion
            const xe = clamp(c.map((ci, j) => ci + gamma * (xr[j] - ci)));
            const fe = costFn(xe);
            simplex[worst] = fe < fr ? { x: xe, cost: fe } : { x: xr, cost: fr };
            continue;
        }

        // Contraction
        const xc = clamp(c.map((ci, j) => ci + rho * (simplex[worst].x[j] - ci)));
        const fc = costFn(xc);

        if (fc < simplex[worst].cost) {
            simplex[worst] = { x: xc, cost: fc };
            continue;
        }

        // Shrink
        for (let i = 1; i <= n; i++) {
            simplex[i].x = clamp(simplex[i].x.map((v, j) =>
                simplex[0].x[j] + sigma * (v - simplex[0].x[j])
            ));
            simplex[i].cost = costFn(simplex[i].x);
        }
    }

    simplex.sort((a, b) => a.cost - b.cost);
    return { x: simplex[0].x, cost: simplex[0].cost, iters };
}

// ─── Cost Function ──────────────────────────────────────────────────────────

const DEFAULT_COST_WEIGHTS = { n: 1, wTrack: 1, wSwing: 2, wEffort: 0.001, wSettle: 0.5 };

function evaluateController(algoType, paramValues, physicsParams, dt, costWeights) {
    const cw = costWeights || DEFAULT_COST_WEIGHTS;
    const sim = new Simulation(algoType);
    sim.dt = dt;

    // Copy physics params
    sim.setParams({ ...physicsParams });

    // Apply controller params
    sim.setControllerParams(algoType, paramValues);

    // Step response: from origin to (5, 0, 2)
    const goalX = 5, goalY = 0, goalZ = 2;
    sim.setGoal(goalX, goalY, goalZ);

    const L = sim.params.L;
    const g = sim.params.g;
    const totalSteps = 600; // 12 seconds at 50Hz

    let cost = 0;
    let settlingTime = 0;
    const stepMag = Math.sqrt(goalX * goalX + goalY * goalY + goalZ * goalZ);
    const settleBand = 0.05 * stepMag; // 5% of step magnitude

    for (let i = 0; i < totalSteps; i++) {
        sim.step();

        const t = (i + 1) * dt;
        const state = sim.state;
        const ctrl = sim.lastControl;

        // Payload position
        const phi_x = state[6], phi_y = state[8];
        const wx = state[0] + L * Math.sin(phi_x) * Math.cos(phi_y);
        const wy = state[2] + L * Math.sin(phi_y);
        const wz = state[4] - L * Math.cos(phi_x) * Math.cos(phi_y);

        // Tracking error (payload to goal)
        const ex = wx - goalX, ey = wy - goalY, ez = wz - goalZ;
        const trackErr = ex * ex + ey * ey + ez * ez;

        // Swing penalty
        const swingErr = phi_x * phi_x + phi_y * phi_y;

        // Control effort (lateral only)
        const effort = ctrl[0] * ctrl[0] + ctrl[1] * ctrl[1];

        // ITAE generalization: t^n weighting
        cost += (cw.wTrack * Math.pow(t, cw.n) * trackErr + cw.wSwing * swingErr + cw.wEffort * effort) * dt;

        // Settling time: last time error exceeds 5% band
        const posErr = Math.sqrt(trackErr);
        if (posErr > settleBand) {
            settlingTime = t;
        }

        // Divergence guard: if drone is way off, abort early with huge cost
        if (Math.abs(state[0]) > 50 || Math.abs(state[2]) > 50 || Math.abs(state[4]) > 100) {
            return 1e10;
        }
    }

    cost += cw.wSettle * settlingTime;
    return cost;
}

// ─── Evaluate cost for given params (used by tuneAll for "before" measurement) ─

export function evaluateCost(algoType, algoParamDefs, physicsParams, dt, costWeights) {
    const paramObj = {};
    algoParamDefs.forEach(p => { paramObj[p.key] = p.default; });
    return evaluateController(algoType, paramObj, physicsParams, dt, costWeights);
}

// ─── Auto-Tune Wrapper ─────────────────────────────────────────────────────

export function autoTune(algoType, algoParamDefs, physicsParams, dt, onProgress, costWeights) {
    // Build x0 and bounds from param definitions (use wide optMin/optMax if available)
    const x0 = algoParamDefs.map(p => p.default);
    const bounds = algoParamDefs.map(p => [p.optMin ?? p.min, p.optMax ?? p.max]);

    const costFn = (paramArray) => {
        const paramObj = {};
        algoParamDefs.forEach((p, i) => { paramObj[p.key] = paramArray[i]; });
        return evaluateController(algoType, paramObj, physicsParams, dt, costWeights);
    };

    // Run in async chunks to keep UI responsive
    return new Promise((resolve) => {
        const n = x0.length;
        const maxIter = 200;
        const tol = 1e-8;
        const alpha = 1.0, gamma_c = 2.0, rho = 0.5, sigma_c = 0.5;
        const batchSize = 20;

        function clamp(x) {
            return x.map((v, i) => Math.max(bounds[i][0], Math.min(bounds[i][1], v)));
        }

        // Build initial simplex
        const simplex = [{ x: clamp([...x0]), cost: 0 }];
        simplex[0].cost = costFn(simplex[0].x);

        for (let i = 0; i < n; i++) {
            const xi = [...x0];
            const range = bounds[i][1] - bounds[i][0];
            xi[i] += range * 0.1;
            const xc = clamp(xi);
            simplex.push({ x: xc, cost: costFn(xc) });
        }

        let iter = 0;

        function runBatch() {
            const batchEnd = Math.min(iter + batchSize, maxIter);

            for (; iter < batchEnd; iter++) {
                simplex.sort((a, b) => a.cost - b.cost);
                const spread = simplex[n].cost - simplex[0].cost;
                if (spread < tol) { iter = maxIter; break; }

                const worst = n;
                const secondWorst = n - 1;

                // Centroid (excluding worst)
                const c = new Array(n).fill(0);
                for (let i = 0; i <= n; i++) {
                    if (i === worst) continue;
                    for (let j = 0; j < n; j++) c[j] += simplex[i].x[j];
                }
                for (let j = 0; j < n; j++) c[j] /= n;

                // Reflection
                const xr = clamp(c.map((ci, j) => ci + alpha * (ci - simplex[worst].x[j])));
                const fr = costFn(xr);

                if (fr < simplex[secondWorst].cost && fr >= simplex[0].cost) {
                    simplex[worst] = { x: xr, cost: fr };
                    continue;
                }

                if (fr < simplex[0].cost) {
                    const xe = clamp(c.map((ci, j) => ci + gamma_c * (xr[j] - ci)));
                    const fe = costFn(xe);
                    simplex[worst] = fe < fr ? { x: xe, cost: fe } : { x: xr, cost: fr };
                    continue;
                }

                const xc = clamp(c.map((ci, j) => ci + rho * (simplex[worst].x[j] - ci)));
                const fc = costFn(xc);

                if (fc < simplex[worst].cost) {
                    simplex[worst] = { x: xc, cost: fc };
                    continue;
                }

                // Shrink
                for (let i = 1; i <= n; i++) {
                    simplex[i].x = clamp(simplex[i].x.map((v, j) =>
                        simplex[0].x[j] + sigma_c * (v - simplex[0].x[j])
                    ));
                    simplex[i].cost = costFn(simplex[i].x);
                }
            }

            if (onProgress) onProgress(Math.min(iter / maxIter, 1));

            if (iter >= maxIter) {
                // Done — build result
                simplex.sort((a, b) => a.cost - b.cost);
                const best = simplex[0];
                const params = {};
                algoParamDefs.forEach((p, i) => {
                    // Snap to slider step for clean values
                    const raw = best.x[i];
                    const snapped = Math.round(raw / p.step) * p.step;
                    params[p.key] = parseFloat(snapped.toFixed(6));
                });
                resolve({ params, cost: best.cost });
            } else {
                setTimeout(runBatch, 0);
            }
        }

        setTimeout(runBatch, 0);
    });
}

// ─── Tune All Algorithms ───────────────────────────────────────────────────

export async function tuneAll(allAlgoParams, physicsParams, dt, onProgress, costWeights) {
    const algoTypes = Object.keys(allAlgoParams);
    const results = [];

    for (let i = 0; i < algoTypes.length; i++) {
        const algoType = algoTypes[i];
        const paramDefs = allAlgoParams[algoType];

        if (onProgress) onProgress(i / algoTypes.length, algoType);

        // Evaluate cost with current defaults (before)
        const beforeCost = evaluateCost(algoType, paramDefs, physicsParams, dt, costWeights);

        // Run optimizer
        const result = await autoTune(algoType, paramDefs, physicsParams, dt, (pct) => {
            if (onProgress) onProgress((i + pct) / algoTypes.length, algoType);
        }, costWeights);

        results.push({
            algoType,
            beforeCost,
            afterCost: result.cost,
            params: result.params,
        });
    }

    if (onProgress) onProgress(1, 'done');
    return results;
}
