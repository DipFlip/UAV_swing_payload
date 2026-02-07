/**
 * Scene update and HUD display.
 * Direct position updates from local simulation — no interpolation needed.
 */

import { physicsToThree } from './scene.js';

function updateDroneSystem(system, dronePos, weightPos, control) {
    const { droneGroup, rope, weight } = system;

    const dp = physicsToThree(dronePos.x, dronePos.y, dronePos.z);
    droneGroup.position.set(dp.x, dp.y, dp.z);

    // Tilt drone based on lateral thrust (quadrotor banks to accelerate)
    if (control) {
        const Fz = Math.max(control.Fz, 1); // avoid division by zero
        droneGroup.rotation.set(
            Math.atan2(control.Fy, Fz),   // roll (physics y → Three.js z)
            0,
            -Math.atan2(control.Fx, Fz)   // pitch (physics x → Three.js x)
        );
    }

    const wp = physicsToThree(weightPos.x, weightPos.y, weightPos.z);
    weight.position.set(wp.x, wp.y, wp.z);

    const positions = rope.geometry.attributes.position;
    positions.setXYZ(0, dp.x, dp.y, dp.z);
    positions.setXYZ(1, wp.x, wp.y, wp.z);
    positions.needsUpdate = true;

    return { dronePos: dp, weightPos: wp };
}

export function updateScene(sceneObjects, data) {
    const { lqr, pid, goalMarker, lqrLabel, pidLabel } = sceneObjects;

    const lqrP = updateDroneSystem(lqr, data.lqr.drone, data.lqr.weight, data.lqr.control);
    const pidP = updateDroneSystem(pid, data.pid.drone, data.pid.weight, data.pid.control);

    const gp = physicsToThree(data.lqr.goal.x, data.lqr.goal.y, data.lqr.goal.z);
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
