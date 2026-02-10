/**
 * Main entry point: local simulation loop and UI event wiring.
 */

import { createScene } from './scene.js';
import { updateScene, updateHUD, setHUDLabels, clearTrails, clearDroneTrails } from './simulation.js';
import { ChartPanel } from './charts.js';
import { Simulation, createSquareTrajectory, createLawnmowerTrajectory } from './sim-engine.js';
import { autoTune, tuneAll } from './optimizer.js';

const canvas = document.getElementById('canvas');
const sceneObjects = createScene(canvas);
const chartPanel = new ChartPanel(document.getElementById('chart-content'));
let prevData = null;

// --- Simulations (simA = blue drone, simB = orange drone) ---
const simLqr = new Simulation('lqr');
const simPid = new Simulation('pid');
simLqr.setGoal(0, 0, 1);
simPid.setGoal(0, 0, 1);

// --- Algorithm name mapping for labels ---
const ALGO_LABELS = { off: 'OFF', lqr: 'LQR', pid: 'PID', cascade: 'CPD', flatness: 'FF', feedbacklin: 'FBL', sliding: 'SMC', mpc: 'MPC' };

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

// --- Pause button ---
let paused = false;
const btnPause = document.getElementById('btn-pause');
btnPause.addEventListener('click', () => {
    paused = !paused;
    btnPause.textContent = paused ? 'Play' : 'Pause';
    btnPause.style.background = paused ? '#44bb44' : '#aaaaaa';
});

// --- Simulation loop via render callback ---
sceneObjects.setOnAnimate((wallDt) => {
    if (paused) return;
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
        updateScene(sceneObjects, data, algoA.value, algoB.value);
        updateHUD(data);
        chartPanel.update(data, prevData);
        prevData = data;

        // Sync sliders from trajectory position
        if (patternActive) {
            const g = simLqr.goal;
            sliderX.value = g[0]; xVal.textContent = g[0].toFixed(1);
            sliderY.value = g[1]; yVal.textContent = g[1].toFixed(1);
            sliderZ.value = g[2]; zVal.textContent = g[2].toFixed(1);
        }

        // Update wind visualization
        const wind = simLqr.lastWind;
        sceneObjects.updateWind(wind.strength, wind.dir, dt);
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
    // Manual goal change cancels active pattern
    if (patternActive) {
        patternActive = false;
        sceneObjects.hidePatternPreview();
        const btn = document.getElementById('btn-pattern');
        btn.textContent = 'Start';
        btn.style.background = '#8855cc';
    }
}

sliderX.addEventListener('input', () => { xVal.textContent = sliderX.value; sendGoal(); });
sliderY.addEventListener('input', () => { yVal.textContent = sliderY.value; sendGoal(); });
sliderZ.addEventListener('input', () => { zVal.textContent = sliderZ.value; sendGoal(); });

// --- Goal smoothing slider ---
const sliderSmooth = document.getElementById('slider-smooth');
const smoothVal = document.getElementById('smooth-val');
sliderSmooth.addEventListener('input', () => {
    const val = parseFloat(sliderSmooth.value);
    smoothVal.textContent = val.toFixed(1);
    // Map to smoother omega: 0=bypass (no smoothing), >0=omega value
    simLqr.setSmootherOmega(val);
    simPid.setSmootherOmega(val);
    // Rebuild trajectory with new tension if pattern is active
    if (patternActive) {
        rebuildActiveTrajectory();
    }
});

// --- Physics sliders ---
const sliderMd = document.getElementById('slider-md');
const mdVal = document.getElementById('md-val');
const sliderMw = document.getElementById('slider-mw');
const mwVal = document.getElementById('mw-val');
const sliderRope = document.getElementById('slider-rope');
const ropeVal = document.getElementById('rope-val');
const sliderFmax = document.getElementById('slider-fmax');
const fmaxVal = document.getElementById('fmax-val');

const hoverVal = document.getElementById('hover-val');

function updateHoverDisplay() {
    const m_d = parseFloat(sliderMd.value);
    const m_w = parseFloat(sliderMw.value);
    const hover = Math.round((m_d + m_w) * 9.81);
    hoverVal.textContent = `(hover: ${hover})`;
}

function sendParams() {
    const m_d = parseFloat(sliderMd.value);
    const m_w = parseFloat(sliderMw.value);
    const L = parseFloat(sliderRope.value);
    const maxThrust = parseFloat(sliderFmax.value);
    const maxLateral = maxThrust * 0.4;
    simLqr.setParams({ m_d, m_w, L, maxLateral, maxThrust });
    simPid.setParams({ m_d, m_w, L, maxLateral, maxThrust });
    updateHoverDisplay();
}

sliderMd.addEventListener('input', () => { mdVal.textContent = sliderMd.value; sendParams(); });
sliderRope.addEventListener('input', () => { ropeVal.textContent = sliderRope.value; sendParams(); });
sliderMw.addEventListener('input', () => { mwVal.textContent = sliderMw.value; sendParams(); });
sliderFmax.addEventListener('input', () => { fmaxVal.textContent = sliderFmax.value; sendParams(); });

// --- Wind sliders ---
const sliderWindStr = document.getElementById('slider-wind-str');
const windStrVal = document.getElementById('wind-str-val');
const sliderWindStd = document.getElementById('slider-wind-std');
const windStdVal = document.getElementById('wind-std-val');
const sliderWindDir = document.getElementById('slider-wind-dir');
const windDirVal = document.getElementById('wind-dir-val');

function sendWind() {
    const windMean = parseFloat(sliderWindStr.value);
    const windStddev = parseFloat(sliderWindStd.value);
    const dirDeg = parseFloat(sliderWindDir.value);
    const windDir = dirDeg * Math.PI / 180;
    simLqr.setParams({ windMean, windStddev, windDir });
    simPid.setParams({ windMean, windStddev, windDir });
}

sliderWindStr.addEventListener('input', () => { windStrVal.textContent = sliderWindStr.value; sendWind(); });
sliderWindStd.addEventListener('input', () => { windStdVal.textContent = sliderWindStd.value; sendWind(); });
sliderWindDir.addEventListener('input', () => { windDirVal.textContent = sliderWindDir.value; sendWind(); });

// --- Status ---
const statusEl = document.getElementById('hud-status');
statusEl.textContent = 'Status: Running locally';
statusEl.style.color = '#00ff88';

// --- Collapsible section toggles ---
function setupCollapsible(headerId, contentId) {
    const header = document.getElementById(headerId);
    const content = document.getElementById(contentId);
    const arrow = header.querySelector('.section-arrow');
    header.addEventListener('click', () => {
        const collapsed = content.classList.toggle('collapsed');
        arrow.innerHTML = collapsed ? '&#x25B6;' : '&#x25BC;';
    });
}
setupCollapsible('goal-header', 'goal-content');
setupCollapsible('sim-header', 'sim-content');
setupCollapsible('algo-header', 'algo-content');

// --- Info panel (bottom-left) ---
const infoPanel = document.getElementById('algo-info-panel');
const infoPanelHeader = document.getElementById('info-panel-header');
const infoPanelToggle = document.getElementById('info-panel-toggle');

infoPanelHeader.addEventListener('click', () => {
    const collapsed = infoPanel.classList.toggle('collapsed');
    infoPanelToggle.innerHTML = collapsed ? '&#x25B2;' : '&#x25BC;';
});

// --- Algorithm-specific parameter definitions ---
const ALGO_PARAMS = {
    lqr: [
        { key: 'qpos', label: 'Pos weight', min: 10, max: 500, step: 10, default: 100, optMin: 1, optMax: 5000 },
        { key: 'qphi', label: 'Swing weight', min: 10, max: 500, step: 10, default: 120, optMin: 1, optMax: 5000 },
        { key: 'rcost', label: 'Ctrl cost', min: 0.01, max: 1, step: 0.01, default: 0.08, optMin: 0.001, optMax: 10 },
    ],
    pid: [
        { key: 'kp', label: 'Kp', min: 5, max: 80, step: 1, default: 22, optMin: 0.5, optMax: 500 },
        { key: 'ki', label: 'Ki', min: 0, max: 20, step: 0.5, default: 3, optMin: 0, optMax: 100 },
        { key: 'kd', label: 'Kd', min: 5, max: 80, step: 1, default: 24, optMin: 0.5, optMax: 500 },
    ],
    cascade: [
        { key: 'kp_outer', label: 'Outer Kp', min: 0.2, max: 5, step: 0.1, default: 1.8, optMin: 0.01, optMax: 50 },
        { key: 'kd_outer', label: 'Outer Kd', min: 0.2, max: 5, step: 0.1, default: 1.2, optMin: 0.01, optMax: 50 },
        { key: 'kp_inner', label: 'Inner Kp', min: 5, max: 80, step: 1, default: 30, optMin: 0.5, optMax: 500 },
        { key: 'kd_inner', label: 'Inner Kd', min: 5, max: 60, step: 1, default: 20, optMin: 0.5, optMax: 500 },
    ],
    flatness: [
        { key: 'kp', label: 'Pos Kp', min: 5, max: 80, step: 1, default: 25, optMin: 0.5, optMax: 500 },
        { key: 'kp_phi', label: 'Angle Kp', min: 5, max: 100, step: 1, default: 40, optMin: 0.5, optMax: 500 },
    ],
    feedbacklin: [
        { key: 'kp', label: 'Pos Kp', min: 2, max: 40, step: 1, default: 12, optMin: 0.1, optMax: 200 },
        { key: 'ka', label: 'Angle Ka', min: 5, max: 80, step: 1, default: 30, optMin: 0.5, optMax: 500 },
        { key: 'kb', label: 'Rate Kb', min: 2, max: 30, step: 1, default: 12, optMin: 0.1, optMax: 200 },
    ],
    sliding: [
        { key: 'lambda', label: 'Conv \u03bb', min: 0.5, max: 8, step: 0.5, default: 2, optMin: 0.05, optMax: 50 },
        { key: 'alpha', label: 'Angle wt \u03b1', min: 1, max: 30, step: 1, default: 8, optMin: 0.1, optMax: 200 },
        { key: 'kSwitch', label: 'Switch gain', min: 2, max: 50, step: 1, default: 15, optMin: 0.1, optMax: 300 },
        { key: 'epsilon', label: 'Boundary \u03b5', min: 0.05, max: 2, step: 0.05, default: 0.5, optMin: 0.01, optMax: 10 },
    ],
    mpc: [
        { key: 'horizon', label: 'Horizon N', min: 5, max: 200, step: 5, default: 50, optMin: 2, optMax: 500 },
        { key: 'qPos', label: 'Pos weight', min: 10, max: 500, step: 10, default: 100, optMin: 1, optMax: 5000 },
        { key: 'rCost', label: 'Ctrl cost', min: 0.01, max: 1, step: 0.01, default: 0.08, optMin: 0.001, optMax: 10 },
    ],
};

// --- localStorage persistence for tuned params ---
const LS_KEY = 'dronehangsim_tuned';

function loadSavedParams() {
    try {
        const raw = localStorage.getItem(LS_KEY);
        return raw ? JSON.parse(raw) : {};
    } catch { return {}; }
}

function saveTunedParams(algoType, params) {
    const saved = loadSavedParams();
    saved[algoType] = params;
    localStorage.setItem(LS_KEY, JSON.stringify(saved));
}

function expandSliderRanges(algoType, tunedParams) {
    const defs = ALGO_PARAMS[algoType];
    if (!defs) return;
    for (const p of defs) {
        const val = tunedParams[p.key];
        if (val === undefined) continue;
        // Expand slider min/max to include tuned value with ~50% margin for user exploration
        const margin = Math.max(Math.abs(val) * 0.5, p.step * 3);
        const wantMin = Math.floor((val - margin) / p.step) * p.step;
        const wantMax = Math.ceil((val + margin) / p.step) * p.step;
        p.min = Math.min(p.min, Math.max(p.optMin ?? 0, parseFloat(wantMin.toFixed(6))));
        p.max = Math.max(p.max, parseFloat(wantMax.toFixed(6)));
    }
}

// On startup: overwrite ALGO_PARAMS defaults with any saved tuned values
(function applySavedDefaults() {
    const saved = loadSavedParams();
    for (const [algoType, params] of Object.entries(saved)) {
        const defs = ALGO_PARAMS[algoType];
        if (!defs) continue;
        for (const p of defs) {
            if (params[p.key] !== undefined) {
                p.default = params[p.key];
            }
        }
        expandSliderRanges(algoType, params);
    }
})();

// Stored parameter values per drone per algorithm (persists across switches)
const droneParamValues = {
    a: {}, b: {},
};

function buildAlgoSliders(containerId, droneId, sim, algoType) {
    const container = document.getElementById(containerId);
    container.innerHTML = '';
    const paramDefs = ALGO_PARAMS[algoType];
    if (!paramDefs) return;

    const saved = droneParamValues[droneId][algoType];

    paramDefs.forEach(p => {
        const val = saved ? saved[p.key] : p.default;
        const div = document.createElement('div');
        div.className = 'slider-group';
        const lbl = document.createElement('label');
        const span = document.createElement('span');
        span.textContent = val;
        lbl.append(p.label + ': ', span);
        const input = document.createElement('input');
        input.type = 'range';
        input.min = p.min;
        input.max = p.max;
        input.step = p.step;
        input.value = val;
        input.addEventListener('input', () => {
            span.textContent = input.value;
            applyAlgoParams(containerId, droneId, sim, algoType);
        });
        div.append(lbl, input);
        container.appendChild(div);
    });

    // Apply initial values
    applyAlgoParams(containerId, droneId, sim, algoType);
}

function applyAlgoParams(containerId, droneId, sim, algoType) {
    const container = document.getElementById(containerId);
    const paramDefs = ALGO_PARAMS[algoType];
    const values = {};
    const inputs = container.querySelectorAll('input[type="range"]');
    paramDefs.forEach((p, i) => {
        values[p.key] = parseFloat(inputs[i].value);
    });
    droneParamValues[droneId][algoType] = values;
    sim.setControllerParams(algoType, values);
}

// --- Algorithm dropdowns & swing damping checkboxes ---
const algoA = document.getElementById('algo-a');
const algoB = document.getElementById('algo-b');
const swingA = document.getElementById('swing-a');
const swingB = document.getElementById('swing-b');
const shapingA = document.getElementById('shaping-a');
const shapingB = document.getElementById('shaping-b');

// Anti-swing only benefits controllers that don't already have pendulum feedback
const SWING_USEFUL = new Set(['pid', 'cascade']);

function updateSwingVisibility(algoValue, swingEl, sim) {
    const show = SWING_USEFUL.has(algoValue);
    swingEl.parentElement.style.display = show ? '' : 'none';
    if (!show && swingEl.checked) {
        swingEl.checked = false;
        sim.setSwingDamping(false);
    }
}

function getAlgoLabel(selectEl, swingEl, shapingEl) {
    let label = ALGO_LABELS[selectEl.value] || selectEl.value;
    if (swingEl.checked) label += '+SD';
    if (shapingEl.checked) label += '+IS';
    return label;
}

function syncAlgoLabels() {
    const labelA = getAlgoLabel(algoA, swingA, shapingA);
    const labelB = getAlgoLabel(algoB, swingB, shapingB);
    sceneObjects.updateLabelText(sceneObjects.lqrLabel, labelA, '#4499ff');
    sceneObjects.updateLabelText(sceneObjects.pidLabel, labelB, '#ff8800');
    setHUDLabels(labelA, labelB);
    chartPanel.setLabels(labelA, labelB);
}

algoA.addEventListener('change', () => {
    simLqr.setControllerType(algoA.value);
    buildAlgoSliders('params-a', 'a', simLqr, algoA.value);
    updateSwingVisibility(algoA.value, swingA, simLqr);
    if (algoA.value === 'off') {
        clearDroneTrails(sceneObjects.trails, 'lqr');
    }
    syncAlgoLabels();
});
algoB.addEventListener('change', () => {
    simPid.setControllerType(algoB.value);
    buildAlgoSliders('params-b', 'b', simPid, algoB.value);
    updateSwingVisibility(algoB.value, swingB, simPid);
    if (algoB.value === 'off') {
        clearDroneTrails(sceneObjects.trails, 'pid');
    }
    syncAlgoLabels();
});

// Set initial visibility (A=lqr → hidden, B=pid → shown)
updateSwingVisibility(algoA.value, swingA, simLqr);
updateSwingVisibility(algoB.value, swingB, simPid);
swingA.addEventListener('change', () => {
    simLqr.setSwingDamping(swingA.checked);
    syncAlgoLabels();
});
swingB.addEventListener('change', () => {
    simPid.setSwingDamping(swingB.checked);
    syncAlgoLabels();
});
shapingA.addEventListener('change', () => {
    simLqr.setInputShaping(shapingA.checked);
    syncAlgoLabels();
});
shapingB.addEventListener('change', () => {
    simPid.setInputShaping(shapingB.checked);
    syncAlgoLabels();
});

// Build initial sliders
buildAlgoSliders('params-a', 'a', simLqr, 'lqr');
buildAlgoSliders('params-b', 'b', simPid, 'pid');

// --- Reset-to-default buttons ---
document.getElementById('reset-a').addEventListener('click', () => {
    delete droneParamValues.a[algoA.value];
    buildAlgoSliders('params-a', 'a', simLqr, algoA.value);
});
document.getElementById('reset-b').addEventListener('click', () => {
    delete droneParamValues.b[algoB.value];
    buildAlgoSliders('params-b', 'b', simPid, algoB.value);
});

// --- Tune config modal ---
const tuneConfigModal = document.getElementById('tune-config');
const tuneConfigSliders = {
    n: document.getElementById('tc-n'),
    wTrack: document.getElementById('tc-track'),
    wSwing: document.getElementById('tc-swing'),
    wEffort: document.getElementById('tc-effort'),
    wSettle: document.getElementById('tc-settle'),
};
const tuneConfigValSpans = {
    n: document.getElementById('tc-n-val'),
    wTrack: document.getElementById('tc-track-val'),
    wSwing: document.getElementById('tc-swing-val'),
    wEffort: document.getElementById('tc-effort-val'),
    wSettle: document.getElementById('tc-settle-val'),
};

// Wire up live value display for config sliders
for (const key of Object.keys(tuneConfigSliders)) {
    tuneConfigSliders[key].addEventListener('input', () => {
        tuneConfigValSpans[key].textContent = tuneConfigSliders[key].value;
    });
}

let tuneConfigCallback = null;

function showTuneConfig(onStart) {
    tuneConfigCallback = onStart;
    tuneConfigModal.classList.remove('hidden');
}

document.getElementById('tune-config-close').addEventListener('click', () => {
    tuneConfigModal.classList.add('hidden');
    tuneConfigCallback = null;
});

document.getElementById('tune-config-start').addEventListener('click', () => {
    const costWeights = {
        n: parseFloat(tuneConfigSliders.n.value),
        wTrack: parseFloat(tuneConfigSliders.wTrack.value),
        wSwing: parseFloat(tuneConfigSliders.wSwing.value),
        wEffort: parseFloat(tuneConfigSliders.wEffort.value),
        wSettle: parseFloat(tuneConfigSliders.wSettle.value),
    };
    tuneConfigModal.classList.add('hidden');
    if (tuneConfigCallback) tuneConfigCallback(costWeights);
    tuneConfigCallback = null;
});

// --- Auto-tune buttons ---
const tuneA = document.getElementById('tune-a');
const tuneB = document.getElementById('tune-b');

tuneA.addEventListener('click', () => {
    showTuneConfig((costWeights) => {
        const algoType = algoA.value;
        tuneA.disabled = true;
        tuneA.textContent = 'Tuning...';
        autoTune(algoType, ALGO_PARAMS[algoType], simLqr.params, SIM_DT, (pct) => {
            tuneA.textContent = `Tuning ${Math.round(pct * 100)}%`;
        }, costWeights).then(result => {
            saveTunedParams(algoType, result.params);
            ALGO_PARAMS[algoType].forEach(p => { if (result.params[p.key] !== undefined) p.default = result.params[p.key]; });
            expandSliderRanges(algoType, result.params);
            droneParamValues.a[algoType] = result.params;
            droneParamValues.b[algoType] = { ...result.params };
            buildAlgoSliders('params-a', 'a', simLqr, algoType);
            if (algoB.value === algoType) buildAlgoSliders('params-b', 'b', simPid, algoType);
            tuneA.disabled = false;
            tuneA.textContent = 'Auto-tune';
        });
    });
});

tuneB.addEventListener('click', () => {
    showTuneConfig((costWeights) => {
        const algoType = algoB.value;
        tuneB.disabled = true;
        tuneB.textContent = 'Tuning...';
        autoTune(algoType, ALGO_PARAMS[algoType], simPid.params, SIM_DT, (pct) => {
            tuneB.textContent = `Tuning ${Math.round(pct * 100)}%`;
        }, costWeights).then(result => {
            saveTunedParams(algoType, result.params);
            ALGO_PARAMS[algoType].forEach(p => { if (result.params[p.key] !== undefined) p.default = result.params[p.key]; });
            expandSliderRanges(algoType, result.params);
            droneParamValues.b[algoType] = result.params;
            droneParamValues.a[algoType] = { ...result.params };
            buildAlgoSliders('params-b', 'b', simPid, algoType);
            if (algoA.value === algoType) buildAlgoSliders('params-a', 'a', simLqr, algoType);
            tuneB.disabled = false;
            tuneB.textContent = 'Auto-tune';
        });
    });
});

// --- Pattern animation ---
const sliderSpeed = document.getElementById('slider-speed');
const speedVal = document.getElementById('speed-val');
sliderSpeed.addEventListener('input', () => {
    speedVal.textContent = sliderSpeed.value;
    if (patternActive) {
        rebuildActiveTrajectory();
    }
});

let patternActive = false;
const btnPattern = document.getElementById('btn-pattern');
const patternSelect = document.getElementById('pattern-type');

const SQUARE_SIZE = 6;
const SQUARE_Z = 3;
const MOWER_SIZE = 8;

function getTension() {
    const val = parseFloat(sliderSmooth.value);
    // Map smooth slider to tension: 0=sharp (tension=1), 8=smooth (tension=0)
    return Math.max(0, 1 - val / 8);
}

function buildTrajectory() {
    const speed = parseFloat(sliderSpeed.value) || 1.0;
    const tension = getTension();
    const type = patternSelect.value;
    if (type === 'mower') {
        return createLawnmowerTrajectory(MOWER_SIZE, SQUARE_Z, speed, tension);
    }
    return createSquareTrajectory(SQUARE_SIZE, SQUARE_Z, speed, tension);
}

function rebuildActiveTrajectory() {
    const traj = buildTrajectory();
    simLqr.setTrajectory(traj);
    simPid.setTrajectory(traj);
    sceneObjects.updatePatternPreview(traj.samplePath(200));
}

function startPattern() {
    patternActive = true;
    btnPattern.textContent = 'Stop';
    btnPattern.style.background = '#ffaa00';
    rebuildActiveTrajectory();
}

function stopPattern() {
    patternActive = false;
    simLqr.clearTrajectory();
    simPid.clearTrajectory();
    sceneObjects.hidePatternPreview();
    btnPattern.textContent = 'Start';
    btnPattern.style.background = '#8855cc';
}

btnPattern.addEventListener('click', () => {
    if (patternActive) stopPattern();
    else startPattern();
});

patternSelect.addEventListener('change', () => {
    if (patternActive) {
        rebuildActiveTrajectory();
    }
});

// --- Tune All handler ---
const tuneAllBtn = document.getElementById('tune-all');
tuneAllBtn.addEventListener('click', () => {
    showTuneConfig((costWeights) => {
        tuneAllBtn.disabled = true;
        tuneAllBtn.textContent = 'Tuning...';

        tuneAll(ALGO_PARAMS, simLqr.params, SIM_DT, (pct, algoType) => {
            const label = ALGO_LABELS[algoType] || algoType;
            const idx = Object.keys(ALGO_PARAMS).indexOf(algoType) + 1;
            const total = Object.keys(ALGO_PARAMS).length;
            tuneAllBtn.textContent = `Tuning ${label} (${idx}/${total}) ${Math.round(pct * 100)}%`;
        }, costWeights).then(results => {
            for (const r of results) {
                saveTunedParams(r.algoType, r.params);
                ALGO_PARAMS[r.algoType].forEach(p => {
                    if (r.params[p.key] !== undefined) p.default = r.params[p.key];
                });
                expandSliderRanges(r.algoType, r.params);
                droneParamValues.a[r.algoType] = { ...r.params };
                droneParamValues.b[r.algoType] = { ...r.params };
            }

            buildAlgoSliders('params-a', 'a', simLqr, algoA.value);
            buildAlgoSliders('params-b', 'b', simPid, algoB.value);

            tuneAllBtn.disabled = false;
            tuneAllBtn.textContent = 'Tune All Algorithms';

            showTuneResults(results, simLqr.params);
        });
    });
});

// --- Results modal ---
function showTuneResults(results, physicsParams) {
    const modal = document.getElementById('tune-modal');
    const body = document.getElementById('tune-modal-body');

    let html = '<table class="tune-results-table"><thead><tr><th>Algorithm</th><th>Before</th><th>After</th><th>Change</th></tr></thead><tbody>';

    for (const r of results) {
        const label = ALGO_LABELS[r.algoType] || r.algoType;
        const before = r.beforeCost.toFixed(1);
        const after = r.afterCost.toFixed(1);
        const changePct = r.beforeCost > 0 ? ((r.afterCost - r.beforeCost) / r.beforeCost * 100) : 0;
        const changeStr = (changePct <= 0 ? '' : '+') + changePct.toFixed(1) + '%';
        const changeColor = changePct <= 0 ? '#44ff88' : '#ff4444';
        html += `<tr><td>${label}</td><td>${before}</td><td>${after}</td><td style="color:${changeColor}">${changeStr}</td></tr>`;
    }

    html += '</tbody></table>';
    html += `<div class="tune-physics-note">Physics: m<sub>d</sub>=${physicsParams.m_d}kg, m<sub>w</sub>=${physicsParams.m_w}kg, L=${physicsParams.L}m, F<sub>max</sub>=${physicsParams.maxLateral}N</div>`;

    body.innerHTML = html;
    modal.classList.remove('hidden');
}

document.getElementById('tune-modal-close').addEventListener('click', () => {
    document.getElementById('tune-modal').classList.add('hidden');
});

// --- UI Buttons ---
document.getElementById('btn-reset').addEventListener('click', () => {
    stopPattern();
    simLqr.reset();
    simPid.reset();
    clearTrails(sceneObjects.trails);
    sliderX.value = 0; xVal.textContent = '0.0';
    sliderY.value = 0; yVal.textContent = '0.0';
    sliderZ.value = 0; zVal.textContent = '0.0';
    prevData = null;
});
