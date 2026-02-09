/**
 * Scene update and HUD display.
 * Direct position updates from local simulation — no interpolation needed.
 */

import * as THREE from 'three';
import { physicsToThree } from './scene.js';

const TILT_SMOOTH = 0.15; // per-update blend factor for rotation smoothing
const lerp = (a, b, t) => a + (b - a) * t;

function updateDroneSystem(system, dronePos, weightPos, control) {
    const { droneGroup, rope, weight, forceArrow } = system;

    const dp = physicsToThree(dronePos.x, dronePos.y, dronePos.z);
    droneGroup.position.set(dp.x, dp.y, dp.z);

    // Tilt drone based on lateral thrust (smoothed to reduce jerkiness)
    if (control) {
        const Fz = Math.max(control.Fz, 1);
        const targetX = Math.atan2(control.Fy, Fz);
        const targetZ = -Math.atan2(control.Fx, Fz);
        droneGroup.rotation.x = lerp(droneGroup.rotation.x, targetX, TILT_SMOOTH);
        droneGroup.rotation.z = lerp(droneGroup.rotation.z, targetZ, TILT_SMOOTH);

        // Update force arrow (cylinder shaft + cone head)
        // Physics (Fx, Fy, Fz) → Three.js (Fx, Fz, Fy)
        const fDir = new THREE.Vector3(control.Fx, control.Fz, control.Fy);
        const fLen = fDir.length();
        if (fLen > 0.1) {
            const len = Math.min(fLen / 20, 3);
            const headLen = 0.5;
            const shaftLen = Math.max(len - headLen, 0.1);
            fDir.divideScalar(fLen);

            forceArrow.group.position.set(dp.x, dp.y, dp.z);
            forceArrow.group.quaternion.setFromUnitVectors(
                new THREE.Vector3(0, 1, 0), fDir
            );
            forceArrow.shaft.scale.set(1, shaftLen, 1);
            forceArrow.head.position.set(0, shaftLen, 0);
            forceArrow.group.visible = true;
        } else {
            forceArrow.group.visible = false;
        }
    }

    const wp = physicsToThree(weightPos.x, weightPos.y, weightPos.z);
    weight.position.set(wp.x, wp.y, wp.z);

    // Position rope cylinder between drone and weight
    const mid = new THREE.Vector3(
        (dp.x + wp.x) / 2, (dp.y + wp.y) / 2, (dp.z + wp.z) / 2
    );
    rope.position.copy(mid);
    const dir = new THREE.Vector3(wp.x - dp.x, wp.y - dp.y, wp.z - dp.z);
    const len = dir.length();
    rope.scale.set(1, len, 1);
    if (len > 0.001) {
        rope.quaternion.setFromUnitVectors(
            new THREE.Vector3(0, 1, 0), dir.divideScalar(len)
        );
    }

    return { dronePos: dp, weightPos: wp };
}

const TRAIL_RADIUS = 0.04;
const _mat4 = new THREE.Matrix4();
const _start = new THREE.Vector3();
const _end = new THREE.Vector3();
const _mid = new THREE.Vector3();
const _dir = new THREE.Vector3();
const _up = new THREE.Vector3(0, 1, 0);
const _quat = new THREE.Quaternion();
const _scale = new THREE.Vector3();

function rebuildTrailInstances(trail) {
    const pts = trail.points;
    const nPts = pts.length / 3;
    if (nPts < 2) { trail.mesh.count = 0; return; }

    for (let i = 0; i < nPts - 1; i++) {
        const si = i * 3;
        _start.set(pts[si], pts[si + 1], pts[si + 2]);
        _end.set(pts[si + 3], pts[si + 4], pts[si + 5]);
        _mid.lerpVectors(_start, _end, 0.5);
        _dir.subVectors(_end, _start);
        const len = _dir.length();
        if (len > 0.0001) {
            _dir.divideScalar(len);
            _quat.setFromUnitVectors(_up, _dir);
        }
        _scale.set(TRAIL_RADIUS, len, TRAIL_RADIUS);
        _mat4.compose(_mid, _quat, _scale);
        trail.mesh.setMatrixAt(i, _mat4);
    }
    trail.mesh.count = nPts - 1;
    trail.mesh.instanceMatrix.needsUpdate = true;
}

function pushTrailPoint(trail, x, y, z) {
    trail.points.push(x, y, z);
    if (trail.points.length > trail.maxPoints * 3) {
        trail.points.splice(0, 3);
    }
    rebuildTrailInstances(trail);
}

export function clearTrails(trails) {
    for (const key in trails) {
        const t = trails[key];
        t.points.length = 0;
        t.mesh.count = 0;
    }
}

export function updateScene(sceneObjects, data) {
    const { lqr, pid, goalMarker, lqrLabel, pidLabel, trails } = sceneObjects;

    const lqrP = updateDroneSystem(lqr, data.lqr.drone, data.lqr.weight, data.lqr.control);
    const pidP = updateDroneSystem(pid, data.pid.drone, data.pid.weight, data.pid.control);

    const gp = physicsToThree(data.lqr.goal.x, data.lqr.goal.y, data.lqr.goal.z);
    goalMarker.position.set(gp.x, gp.y, gp.z);

    lqrLabel.position.set(lqrP.dronePos.x, lqrP.dronePos.y + 0.8, lqrP.dronePos.z);
    pidLabel.position.set(pidP.dronePos.x, pidP.dronePos.y + 0.8, pidP.dronePos.z);

    // Push trail points
    pushTrailPoint(trails.lqrDrone, lqrP.dronePos.x, lqrP.dronePos.y, lqrP.dronePos.z);
    pushTrailPoint(trails.lqrWeight, lqrP.weightPos.x, lqrP.weightPos.y, lqrP.weightPos.z);
    pushTrailPoint(trails.pidDrone, pidP.dronePos.x, pidP.dronePos.y, pidP.dronePos.z);
    pushTrailPoint(trails.pidWeight, pidP.weightPos.x, pidP.weightPos.y, pidP.weightPos.z);
    pushTrailPoint(trails.goal, gp.x, gp.y, gp.z);
}

// Labels for HUD (updated when algorithm changes)
let hudLabelA = 'LQR';
let hudLabelB = 'PID';

export function setHUDLabels(a, b) {
    hudLabelA = a;
    hudLabelB = b;
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
        `${hudLabelA} dist: ${lqrDist}m | ${hudLabelB} dist: ${pidDist}m`;
    document.getElementById('hud-weight').textContent =
        `${hudLabelA} wt: (${lw.x.toFixed(1)}, ${lw.y.toFixed(1)}, ${lw.z.toFixed(1)}) ${hudLabelB} wt: (${pw.x.toFixed(1)}, ${pw.y.toFixed(1)}, ${pw.z.toFixed(1)})`;
    document.getElementById('hud-angles').textContent =
        `${hudLabelA} \u03C6: ${(lqr.phi_x * 180 / Math.PI).toFixed(1)}\u00B0,${(lqr.phi_y * 180 / Math.PI).toFixed(1)}\u00B0 | ${hudLabelB} \u03C6: ${(pid.phi_x * 180 / Math.PI).toFixed(1)}\u00B0,${(pid.phi_y * 180 / Math.PI).toFixed(1)}\u00B0`;
    document.getElementById('hud-control').textContent =
        `Goal: (${g.x.toFixed(1)}, ${g.y.toFixed(1)}, ${g.z.toFixed(1)})`;
}
