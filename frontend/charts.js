/**
 * Real-time rolling charts for dual controller comparison.
 * Pure Canvas 2D — no dependencies.
 */

const HISTORY_SIZE = 200;
const CHART_PADDING = { top: 22, right: 8, bottom: 4, left: 40 };

class RollingChart {
    constructor(canvas, title, series, opts = {}) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.title = title;
        this.series = series; // [{label, color, dash?}]
        this.autoRange = opts.autoRange !== false;
        this.fixedMin = opts.min ?? 0;
        this.fixedMax = opts.max ?? 1;
        this.unit = opts.unit ?? '';
        this.data = series.map(() => []);
        this.cssW = 300;
        this.cssH = 100;
    }

    push(values) {
        for (let i = 0; i < this.series.length; i++) {
            this.data[i].push(values[i] ?? 0);
            if (this.data[i].length > HISTORY_SIZE) this.data[i].shift();
        }
    }

    draw() {
        const ctx = this.ctx;
        const W = this.cssW;
        const H = this.cssH;
        const p = CHART_PADDING;
        const plotW = W - p.left - p.right;
        const plotH = H - p.top - p.bottom;

        // Background
        ctx.fillStyle = 'rgba(20, 20, 30, 0.95)';
        ctx.fillRect(0, 0, W, H);

        // Compute range
        let min, max;
        if (this.autoRange) {
            let allVals = this.data.flat();
            if (allVals.length === 0) allVals = [0];
            min = Math.min(...allVals);
            max = Math.max(...allVals);
            const margin = (max - min) * 0.15 || 1;
            min -= margin;
            max += margin;
        } else {
            min = this.fixedMin;
            max = this.fixedMax;
        }

        // Grid lines
        ctx.strokeStyle = 'rgba(255,255,255,0.08)';
        ctx.lineWidth = 1;
        const gridLines = 3;
        for (let i = 0; i <= gridLines; i++) {
            const y = p.top + (plotH * i / gridLines);
            ctx.beginPath();
            ctx.moveTo(p.left, y);
            ctx.lineTo(p.left + plotW, y);
            ctx.stroke();

            const val = max - (max - min) * (i / gridLines);
            ctx.fillStyle = 'rgba(255,255,255,0.4)';
            ctx.font = '9px monospace';
            ctx.textAlign = 'right';
            ctx.fillText(val.toFixed(1), p.left - 4, y + 3);
        }

        // Zero line
        if (min < 0 && max > 0) {
            const zeroY = p.top + plotH * (max / (max - min));
            ctx.strokeStyle = 'rgba(255,255,255,0.2)';
            ctx.setLineDash([4, 4]);
            ctx.beginPath();
            ctx.moveTo(p.left, zeroY);
            ctx.lineTo(p.left + plotW, zeroY);
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // Plot border
        ctx.strokeStyle = 'rgba(255,255,255,0.15)';
        ctx.lineWidth = 1;
        ctx.strokeRect(p.left, p.top, plotW, plotH);

        // Draw series
        for (let s = 0; s < this.series.length; s++) {
            const d = this.data[s];
            if (d.length < 2) continue;

            ctx.strokeStyle = this.series[s].color;
            ctx.lineWidth = this.series[s].thick ? 2 : 1.5;
            if (this.series[s].dash) {
                ctx.setLineDash(this.series[s].dash);
            }
            ctx.beginPath();

            for (let i = 0; i < d.length; i++) {
                const x = p.left + (plotW * i / (HISTORY_SIZE - 1));
                const y = p.top + plotH * (1 - (d[i] - min) / (max - min));
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // Title
        ctx.fillStyle = '#00d4ff';
        ctx.font = 'bold 11px sans-serif';
        ctx.textAlign = 'left';
        ctx.fillText(this.title, p.left, 14);

        // Legend (right-aligned)
        let legendX = W - p.right;
        ctx.font = '9px monospace';
        ctx.textAlign = 'right';
        for (let s = this.series.length - 1; s >= 0; s--) {
            const d = this.data[s];
            const val = d.length > 0 ? d[d.length - 1] : 0;
            const text = `${this.series[s].label}:${val.toFixed(1)}${this.unit}`;
            const tw = ctx.measureText(text).width;

            ctx.fillStyle = this.series[s].color;
            ctx.fillText(text, legendX, 14);
            legendX -= tw + 10;
        }
    }
}

export class ChartPanel {
    constructor(container) {
        this.container = container;
        this.charts = [];
        this._createCharts();
        this._resize();
        window.addEventListener('resize', () => this._resize());
    }

    _makeCanvas() {
        const c = document.createElement('canvas');
        this.container.appendChild(c);
        return c;
    }

    _createCharts() {
        // 1. Distance to goal — LQR vs PID
        this.distChart = new RollingChart(
            this._makeCanvas(),
            'Distance to Goal',
            [
                { label: 'LQR', color: '#4499ff', thick: true },
                { label: 'PID', color: '#ff8800', thick: true },
            ],
            { unit: 'm', autoRange: true }
        );
        this.charts.push(this.distChart);

        // 2. Swing angle magnitude comparison
        this.swingChart = new RollingChart(
            this._makeCanvas(),
            'Swing Magnitude',
            [
                { label: 'LQR', color: '#4499ff', thick: true },
                { label: 'PID', color: '#ff8800', thick: true },
            ],
            { unit: '\u00B0', autoRange: true }
        );
        this.charts.push(this.swingChart);

        // 3. LQR control forces
        this.lqrForceChart = new RollingChart(
            this._makeCanvas(),
            'LQR Forces',
            [
                { label: 'Fx', color: '#ff4466' },
                { label: 'Fy', color: '#44ff66' },
                { label: 'Fz', color: '#4488ff' },
            ],
            { unit: 'N', autoRange: true }
        );
        this.charts.push(this.lqrForceChart);

        // 4. PID control forces
        this.pidForceChart = new RollingChart(
            this._makeCanvas(),
            'PID Forces',
            [
                { label: 'Fx', color: '#ff8844' },
                { label: 'Fy', color: '#aaff44' },
                { label: 'Fz', color: '#ffaa44' },
            ],
            { unit: 'N', autoRange: true }
        );
        this.charts.push(this.pidForceChart);

        // 5. Weight speed comparison
        this.velChart = new RollingChart(
            this._makeCanvas(),
            'Weight Speed',
            [
                { label: 'LQR', color: '#4499ff', thick: true },
                { label: 'PID', color: '#ff8800', thick: true },
            ],
            { unit: 'm/s', autoRange: true }
        );
        this.charts.push(this.velChart);
    }

    _resize() {
        const rect = this.container.getBoundingClientRect();
        const pad = parseInt(getComputedStyle(this.container).padding) || 6;
        const gap = 4;
        const innerW = rect.width - pad * 2;
        const chartH = Math.floor((rect.height - pad * 2 - gap * (this.charts.length - 1)) / this.charts.length);
        const dpr = window.devicePixelRatio;
        this.charts.forEach(c => {
            c.cssW = innerW;
            c.cssH = chartH;
            c.canvas.width = innerW * dpr;
            c.canvas.height = chartH * dpr;
            c.canvas.style.width = innerW + 'px';
            c.canvas.style.height = chartH + 'px';
        });
    }

    update(data, prevData) {
        const lqr = data.lqr;
        const pid = data.pid;
        const g = lqr.goal;

        // Distance to goal
        const lqrDist = Math.sqrt(
            (lqr.weight.x - g.x) ** 2 + (lqr.weight.y - g.y) ** 2 + (lqr.weight.z - g.z) ** 2
        );
        const pidDist = Math.sqrt(
            (pid.weight.x - g.x) ** 2 + (pid.weight.y - g.y) ** 2 + (pid.weight.z - g.z) ** 2
        );
        this.distChart.push([lqrDist, pidDist]);

        // Swing angle magnitude
        const lqrSwing = Math.sqrt(lqr.phi_x ** 2 + lqr.phi_y ** 2) * 180 / Math.PI;
        const pidSwing = Math.sqrt(pid.phi_x ** 2 + pid.phi_y ** 2) * 180 / Math.PI;
        this.swingChart.push([lqrSwing, pidSwing]);

        // Control forces
        this.lqrForceChart.push([lqr.control.Fx, lqr.control.Fy, lqr.control.Fz]);
        this.pidForceChart.push([pid.control.Fx, pid.control.Fy, pid.control.Fz]);

        // Weight speed
        if (prevData) {
            const dt = lqr.time - prevData.lqr.time;
            if (dt > 0) {
                const lqrSpeed = Math.sqrt(
                    ((lqr.weight.x - prevData.lqr.weight.x) / dt) ** 2 +
                    ((lqr.weight.y - prevData.lqr.weight.y) / dt) ** 2 +
                    ((lqr.weight.z - prevData.lqr.weight.z) / dt) ** 2
                );
                const pidSpeed = Math.sqrt(
                    ((pid.weight.x - prevData.pid.weight.x) / dt) ** 2 +
                    ((pid.weight.y - prevData.pid.weight.y) / dt) ** 2 +
                    ((pid.weight.z - prevData.pid.weight.z) / dt) ** 2
                );
                this.velChart.push([lqrSpeed, pidSpeed]);
            } else {
                this.velChart.push([0, 0]);
            }
        } else {
            this.velChart.push([0, 0]);
        }

        // Redraw all
        const dpr = window.devicePixelRatio;
        this.charts.forEach(c => {
            c.ctx.setTransform(1, 0, 0, 1, 0, 0);
            c.ctx.clearRect(0, 0, c.canvas.width, c.canvas.height);
            c.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
            c.draw();
        });
    }
}
