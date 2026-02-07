/**
 * Main entry point: WebSocket connection and UI event wiring.
 */

import { createScene } from './scene.js';
import { setTargets, interpolateScene, updateHUD } from './simulation.js';
import { ChartPanel } from './charts.js';

const canvas = document.getElementById('canvas');
const sceneObjects = createScene(canvas);
const chartPanel = new ChartPanel(document.getElementById('chart-content'));
let prevData = null;

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

// Run interpolation every frame via the render loop
sceneObjects.setOnAnimate(() => interpolateScene(sceneObjects));

// --- Slider value display ---
const sliderX = document.getElementById('slider-x');
const sliderY = document.getElementById('slider-y');
const sliderZ = document.getElementById('slider-z');
const xVal = document.getElementById('x-val');
const yVal = document.getElementById('y-val');
const zVal = document.getElementById('z-val');

function sendGoal() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'set_goal',
            x: parseFloat(sliderX.value),
            y: parseFloat(sliderY.value),
            z: parseFloat(sliderZ.value),
        }));
    }
}

const sliderAggr = document.getElementById('slider-aggr');
const aggrVal = document.getElementById('aggr-val');

function sendAggression() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({
            type: 'set_aggression',
            value: parseFloat(sliderAggr.value) / 100.0,
        }));
    }
}

sliderX.addEventListener('input', () => { xVal.textContent = sliderX.value; sendGoal(); });
sliderY.addEventListener('input', () => { yVal.textContent = sliderY.value; sendGoal(); });
sliderZ.addEventListener('input', () => { zVal.textContent = sliderZ.value; sendGoal(); });
sliderAggr.addEventListener('input', () => { aggrVal.textContent = sliderAggr.value; sendAggression(); });

// --- WebSocket ---
let ws = null;
const statusEl = document.getElementById('hud-status');

function connect() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        statusEl.textContent = 'Status: Connected';
        statusEl.style.color = '#00ff88';
    };

    ws.onclose = () => {
        statusEl.textContent = 'Status: Disconnected';
        statusEl.style.color = '#ff4444';
        setTimeout(connect, 2000);
    };

    ws.onerror = () => {
        statusEl.textContent = 'Status: Error';
        statusEl.style.color = '#ff4444';
    };

    ws.onmessage = (event) => {
        const raw = JSON.parse(event.data);
        const data = raw.t === 'd'
            ? { type: 'dual_state', lqr: raw.l, pid: raw.p }
            : raw;
        if (data.type === 'dual_state') {
            setTargets(data);
            updateHUD(data);
            chartPanel.update(data, prevData);
            prevData = data;
        }
    };
}

connect();

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

    // Total perimeter progress [0, 4) — each integer is one corner
    let progress = 0;
    let lastTime = performance.now();

    function tick(now) {
        if (!patternActive) return;

        const dt = (now - lastTime) / 1000; // seconds
        lastTime = now;

        const speed = parseFloat(sliderSpeed.value) || 1.0;
        // speed=1 → full lap in 12s (3s per edge), speed=3 → 4s per lap
        progress += (speed / 3.0) * dt;
        if (progress >= 4) progress -= 4;

        const seg = Math.floor(progress);
        const t = progress - seg;
        const a = squareCorners[seg];
        const b = squareCorners[(seg + 1) % 4];
        const gx = a.x + (b.x - a.x) * t;
        const gy = a.y + (b.y - a.y) * t;
        const gz = a.z + (b.z - a.z) * t;

        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'set_goal', x: gx, y: gy, z: gz }));
        }

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
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'reset' }));
        sliderX.value = 0; xVal.textContent = '0.0';
        sliderY.value = 0; yVal.textContent = '0.0';
        sliderZ.value = 0; zVal.textContent = '0.0';
        prevData = null;
    }
});
