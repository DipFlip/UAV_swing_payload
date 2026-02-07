/**
 * State update handler with client-side interpolation.
 * Data arrives at ~25Hz, scene renders at 60fps via lerp.
 */

import { physicsToThree } from './scene.js';

// Store current and target positions for interpolation
const lerp = (a, b, t) => a + (b - a) * t;
const LERP_SPEED = 0.5; // per-frame blend factor (higher = less smoothing of swing)

const state = {
    lqr: { drone: {x:0,y:0,z:4}, weight: {x:0,y:0,z:0} },
    pid: { drone: {x:0,y:0,z:4}, weight: {x:0,y:0,z:0} },
    goal: {x:0, y:0, z:0},
};
const current = {
    lqr: { drone: {x:0,y:0,z:4}, weight: {x:0,y:0,z:0} },
    pid: { drone: {x:0,y:0,z:4}, weight: {x:0,y:0,z:0} },
    goal: {x:0, y:0, z:0},
};

function lerpPos(cur, tgt) {
    cur.x = lerp(cur.x, tgt.x, LERP_SPEED);
    cur.y = lerp(cur.y, tgt.y, LERP_SPEED);
    cur.z = lerp(cur.z, tgt.z, LERP_SPEED);
}

function updateDroneSystem(system, dronePos, weightPos) {
    const { droneGroup, rope, weight } = system;

    const dp = physicsToThree(dronePos.x, dronePos.y, dronePos.z);
    droneGroup.position.set(dp.x, dp.y, dp.z);

    const wp = physicsToThree(weightPos.x, weightPos.y, weightPos.z);
    weight.position.set(wp.x, wp.y, wp.z);

    const positions = rope.geometry.attributes.position;
    positions.setXYZ(0, dp.x, dp.y, dp.z);
    positions.setXYZ(1, wp.x, wp.y, wp.z);
    positions.needsUpdate = true;

    return { dronePos: dp, weightPos: wp };
}

export function setTargets(data) {
    state.lqr.drone = { ...data.lqr.drone };
    state.lqr.weight = { ...data.lqr.weight };
    state.pid.drone = { ...data.pid.drone };
    state.pid.weight = { ...data.pid.weight };
    state.goal = { ...data.lqr.goal };
}

export function interpolateScene(sceneObjects) {
    lerpPos(current.lqr.drone, state.lqr.drone);
    lerpPos(current.lqr.weight, state.lqr.weight);
    lerpPos(current.pid.drone, state.pid.drone);
    lerpPos(current.pid.weight, state.pid.weight);
    lerpPos(current.goal, state.goal);

    const { lqr, pid, goalMarker, lqrLabel, pidLabel } = sceneObjects;

    const lqrP = updateDroneSystem(lqr, current.lqr.drone, current.lqr.weight);
    const pidP = updateDroneSystem(pid, current.pid.drone, current.pid.weight);

    const gp = physicsToThree(current.goal.x, current.goal.y, current.goal.z);
    goalMarker.position.set(gp.x, gp.y, gp.z);

    lqrLabel.position.set(lqrP.dronePos.x, lqrP.dronePos.y + 0.8, lqrP.dronePos.z);
    pidLabel.position.set(pidP.dronePos.x, pidP.dronePos.y + 0.8, pidP.dronePos.z);
}

export function updateHUD(data) {
    const lqr = data.lqr;
    const pid = data.pid;

    document.getElementById('hud-time').textContent =
        `Time: ${lqr.time.toFixed(2)} s`;

    const lw = lqr.weight, pw = pid.weight;
    const g = lqr.goal;

    const lqrDist = Math.sqrt(
        (lw.x - g.x) ** 2 + (lw.y - g.y) ** 2 + (lw.z - g.z) ** 2
    ).toFixed(2);
    const pidDist = Math.sqrt(
        (pw.x - g.x) ** 2 + (pw.y - g.y) ** 2 + (pw.z - g.z) ** 2
    ).toFixed(2);

    document.getElementById('hud-drone').textContent =
        `LQR dist: ${lqrDist}m | PID dist: ${pidDist}m`;
    document.getElementById('hud-weight').textContent =
        `LQR wt: (${lw.x.toFixed(1)}, ${lw.y.toFixed(1)}, ${lw.z.toFixed(1)}) PID wt: (${pw.x.toFixed(1)}, ${pw.y.toFixed(1)}, ${pw.z.toFixed(1)})`;
    document.getElementById('hud-angles').textContent =
        `LQR \u03C6: ${(lqr.phi_x * 180 / Math.PI).toFixed(1)}\u00B0,${(lqr.phi_y * 180 / Math.PI).toFixed(1)}\u00B0 | PID \u03C6: ${(pid.phi_x * 180 / Math.PI).toFixed(1)}\u00B0,${(pid.phi_y * 180 / Math.PI).toFixed(1)}\u00B0`;
    document.getElementById('hud-control').textContent =
        `Goal: (${g.x.toFixed(1)}, ${g.y.toFixed(1)}, ${g.z.toFixed(1)})`;
}
