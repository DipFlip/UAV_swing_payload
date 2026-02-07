/**
 * Main entry point: local simulation loop and UI event wiring.
 */

import { createScene } from './scene.js';
import { updateScene, updateHUD } from './simulation.js';
import { ChartPanel } from './charts.js';
import { Simulation } from './sim-engine.js';

const canvas = document.getElementById('canvas');
const sceneObjects = createScene(canvas);
const chartPanel = new ChartPanel(document.getElementById('chart-content'));
let prevData = null;

// --- Simulations ---
const simLqr = new Simulation('lqr');
const simPid = new Simulation('pid');

const SIM_DT = 0.02;  // fixed sim timestep
let simAccum = 0;      // fractional sim-time accumulator

// --- Chart panel toggle ---
const chartPanelEl = document.getElementById('chart-panel');
const chartToggle = document.getElementById('chart-toggle');
const chartHeader = document.getElementById('chart-header');

function toggleCharts() {
    const collapsed = chartPanelEl.classList.toggle('collapsed');
    chartToggle.innerHTML = collapsed ? '&#x25B2;' : '&#x25BC;';
    if (!collapsed) chartPanel._resize();
}
chartHeader.addEventListener('click', toggleCharts);

// --- HUD toggle ---
const hudEl = document.getElementById('hud');
const hudToggle = document.getElementById('hud-toggle');
const hudHeader = document.getElementById('hud-header');

hudHeader.addEventListener('click', () => {
    const collapsed = hudEl.classList.toggle('collapsed');
    hudToggle.innerHTML = collapsed ? '&#x25B2;' : '&#x25BC;';
});

// Start collapsed on small screens
if (window.innerWidth < 768) {
    chartPanelEl.classList.add('collapsed');
    chartToggle.innerHTML = '&#x25B2;';
    hudEl.classList.add('collapsed');
    hudToggle.innerHTML = '&#x25B2;';
}

// --- Time scale slider ---
const sliderTimescale = document.getElementById('slider-timescale');
const timescaleVal = document.getElementById('timescale-val');
sliderTimescale.addEventListener('input', () => { timescaleVal.textContent = sliderTimescale.value; });

// --- Simulation loop via render callback ---
sceneObjects.setOnAnimate((wallDt) => {
    // Clamp to 100ms to handle tab backgrounding
    const dt = Math.min(wallDt, 0.1);
    const timeScale = parseFloat(sliderTimescale.value) / 100;
    simAccum += dt * timeScale;

    let latestLqr, latestPid;
    let stepped = false;
    while (simAccum >= SIM_DT) {
        simAccum -= SIM_DT;
        latestLqr = simLqr.step();
        latestPid = simPid.step();
        stepped = true;
    }

    if (stepped) {
        const data = { type: 'dual_state', lqr: latestLqr, pid: latestPid };
        updateScene(sceneObjects, data);
        updateHUD(data);
        chartPanel.update(data, prevData);
        prevData = data;
    }
});

// --- Slider value display ---
const sliderX = document.getElementById('slider-x');
const sliderY = document.getElementById('slider-y');
const sliderZ = document.getElementById('slider-z');
const xVal = document.getElementById('x-val');
const yVal = document.getElementById('y-val');
const zVal = document.getElementById('z-val');

function sendGoal() {
    const x = parseFloat(sliderX.value);
    const y = parseFloat(sliderY.value);
    const z = parseFloat(sliderZ.value);
    simLqr.setGoal(x, y, z);
    simPid.setGoal(x, y, z);
}

const sliderAggr = document.getElementById('slider-aggr');
const aggrVal = document.getElementById('aggr-val');

function sendAggression() {
    const aggr = parseFloat(sliderAggr.value) / 100.0;
    simLqr.setAggression(aggr);
    simPid.setAggression(aggr);
}

sliderX.addEventListener('input', () => { xVal.textContent = sliderX.value; sendGoal(); });
sliderY.addEventListener('input', () => { yVal.textContent = sliderY.value; sendGoal(); });
sliderZ.addEventListener('input', () => { zVal.textContent = sliderZ.value; sendGoal(); });
sliderAggr.addEventListener('input', () => { aggrVal.textContent = sliderAggr.value; sendAggression(); });

// --- Status ---
const statusEl = document.getElementById('hud-status');
statusEl.textContent = 'Status: Running locally';
statusEl.style.color = '#00ff88';

// --- Pattern animation ---
const sliderSpeed = document.getElementById('slider-speed');
const speedVal = document.getElementById('speed-val');
sliderSpeed.addEventListener('input', () => { speedVal.textContent = sliderSpeed.value; });

let patternActive = false;
let patternRaf = null;
const btnPattern = document.getElementById('btn-pattern');

// Square corners at height z=3, size 6x6
const SQUARE_SIZE = 6;
const SQUARE_Z = 3;
const squareCorners = [
    { x:  SQUARE_SIZE/2, y:  SQUARE_SIZE/2, z: SQUARE_Z },
    { x: -SQUARE_SIZE/2, y:  SQUARE_SIZE/2, z: SQUARE_Z },
    { x: -SQUARE_SIZE/2, y: -SQUARE_SIZE/2, z: SQUARE_Z },
    { x:  SQUARE_SIZE/2, y: -SQUARE_SIZE/2, z: SQUARE_Z },
];

function startPattern() {
    patternActive = true;
    btnPattern.textContent = 'Stop';
    btnPattern.style.background = '#ffaa00';

    let progress = 0;
    let lastTime = performance.now();

    function tick(now) {
        if (!patternActive) return;

        const dt = (now - lastTime) / 1000;
        lastTime = now;

        const speed = parseFloat(sliderSpeed.value) || 1.0;
        const timeScale = parseFloat(sliderTimescale.value) / 100;
        progress += (speed / 3.0) * dt * timeScale;
        if (progress >= 4) progress -= 4;

        const seg = Math.floor(progress);
        const t = progress - seg;
        const a = squareCorners[seg];
        const b = squareCorners[(seg + 1) % 4];
        const gx = a.x + (b.x - a.x) * t;
        const gy = a.y + (b.y - a.y) * t;
        const gz = a.z + (b.z - a.z) * t;

        simLqr.setGoal(gx, gy, gz);
        simPid.setGoal(gx, gy, gz);

        sliderX.value = gx; xVal.textContent = gx.toFixed(1);
        sliderY.value = gy; yVal.textContent = gy.toFixed(1);
        sliderZ.value = gz; zVal.textContent = gz.toFixed(1);

        patternRaf = requestAnimationFrame(tick);
    }
    patternRaf = requestAnimationFrame(tick);
}

function stopPattern() {
    patternActive = false;
    if (patternRaf) cancelAnimationFrame(patternRaf);
    patternRaf = null;
    btnPattern.textContent = 'Square';
    btnPattern.style.background = '#8855cc';
}

btnPattern.addEventListener('click', () => {
    if (patternActive) stopPattern();
    else startPattern();
});

// --- UI Buttons ---
document.getElementById('btn-go').addEventListener('click', sendGoal);

document.getElementById('btn-reset').addEventListener('click', () => {
    stopPattern();
    simLqr.reset();
    simPid.reset();
    sliderX.value = 0; xVal.textContent = '0.0';
    sliderY.value = 0; yVal.textContent = '0.0';
    sliderZ.value = 0; zVal.textContent = '0.0';
    prevData = null;
});
