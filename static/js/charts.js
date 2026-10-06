/* Minimal dependency-free real-time strip chart renderer (canvas 2D). */

class StripChart {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.color = opts.color || '#38e0ff';
    this.fill = opts.fill !== false;
    this.maxPoints = opts.maxPoints || 600;
    this.unit = opts.unit || '';
    this.decimals = opts.decimals ?? 1;
    this.series = [];
    this.dpr = window.devicePixelRatio || 1;
    this._resizeObserver = new ResizeObserver(() => this._resize());
    this._resizeObserver.observe(canvas);
    this._resize();
  }

  _resize() {
    const rect = this.canvas.getBoundingClientRect();
    const h = rect.height || 140;
    const w = rect.width || 300;
    this.canvas.width = Math.max(1, Math.round(w * this.dpr));
    this.canvas.height = Math.max(1, Math.round(h * this.dpr));
    this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    this.w = w;
    this.h = h;
    this.draw();
  }

  push(t, value) {
    this.series.push([t, value]);
    if (this.series.length > this.maxPoints) this.series.shift();
    this.draw();
  }

  reset() {
    this.series = [];
    this.draw();
  }

  draw() {
    const { ctx, w, h } = this;
    if (!w || !h) return;
    ctx.clearRect(0, 0, w, h);
    const pad = { l: 6, r: 6, t: 8, b: 6 };
    const plotW = w - pad.l - pad.r;
    const plotH = h - pad.t - pad.b;

    // grid lines
    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 3; i++) {
      const y = pad.t + (plotH / 3) * i;
      ctx.beginPath();
      ctx.moveTo(pad.l, y);
      ctx.lineTo(w - pad.r, y);
      ctx.stroke();
    }

    if (this.series.length < 2) return;

    const xs = this.series.map((p) => p[0]);
    const ys = this.series.map((p) => p[1]);
    let minX = xs[0], maxX = xs[xs.length - 1];
    let minY = Math.min(...ys), maxY = Math.max(...ys);
    if (maxX - minX < 1e-6) maxX = minX + 1;
    if (maxY - minY < 1e-6) { minY -= 1; maxY += 1; }
    const yPad = (maxY - minY) * 0.12;
    minY -= yPad;
    maxY += yPad;

    const xToPx = (x) => pad.l + ((x - minX) / (maxX - minX)) * plotW;
    const yToPx = (y) => pad.t + plotH - ((y - minY) / (maxY - minY)) * plotH;

    // filled area
    if (this.fill) {
      const grad = ctx.createLinearGradient(0, pad.t, 0, pad.t + plotH);
      grad.addColorStop(0, this._alpha(this.color, 0.32));
      grad.addColorStop(1, this._alpha(this.color, 0.0));
      ctx.beginPath();
      ctx.moveTo(xToPx(this.series[0][0]), pad.t + plotH);
      for (const [x, y] of this.series) ctx.lineTo(xToPx(x), yToPx(y));
      ctx.lineTo(xToPx(this.series[this.series.length - 1][0]), pad.t + plotH);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();
    }

    // line
    ctx.beginPath();
    ctx.lineWidth = 1.8;
    ctx.strokeStyle = this.color;
    ctx.lineJoin = 'round';
    this.series.forEach(([x, y], i) => {
      const px = xToPx(x), py = yToPx(y);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.shadowColor = this.color;
    ctx.shadowBlur = 6;
    ctx.stroke();
    ctx.shadowBlur = 0;

    // current point marker
    const last = this.series[this.series.length - 1];
    const lx = xToPx(last[0]), ly = yToPx(last[1]);
    ctx.beginPath();
    ctx.arc(lx, ly, 3, 0, Math.PI * 2);
    ctx.fillStyle = this.color;
    ctx.fill();
  }

  _alpha(hex, a) {
    const c = hex.replace('#', '');
    const r = parseInt(c.substring(0, 2), 16);
    const g = parseInt(c.substring(2, 4), 16);
    const b = parseInt(c.substring(4, 6), 16);
    return `rgba(${r},${g},${b},${a})`;
  }
}

/* Simple 2D ground-track / altitude-vs-downrange trajectory canvas. */
class TrajectoryCanvas {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.dpr = window.devicePixelRatio || 1;
    this.points = { primary: [], booster: [] };
    new ResizeObserver(() => this._resize()).observe(canvas);
    this._resize();
  }

  _resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.w = rect.width || 300;
    this.h = rect.height || 220;
    this.canvas.width = Math.max(1, Math.round(this.w * this.dpr));
    this.canvas.height = Math.max(1, Math.round(this.h * this.dpr));
    this.ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    this.draw();
  }

  push(key, downrangeKm, altKm) {
    const arr = this.points[key];
    arr.push([downrangeKm, altKm]);
    if (arr.length > 2000) arr.shift();
    this.draw();
  }

  reset() {
    this.points = { primary: [], booster: [] };
    this.draw();
  }

  draw() {
    const { ctx, w, h } = this;
    ctx.clearRect(0, 0, w, h);
    const pad = 26;
    const all = [...this.points.primary, ...this.points.booster];
    if (all.length < 2) {
      ctx.fillStyle = 'rgba(255,255,255,0.25)';
      ctx.font = '11px Inter, sans-serif';
      ctx.fillText('awaiting trajectory data…', pad, h / 2);
      return;
    }
    const xs = all.map((p) => p[0]);
    const ys = all.map((p) => p[1]);
    const minX = Math.min(0, ...xs), maxX = Math.max(...xs, 1);
    const minY = 0, maxY = Math.max(...ys, 1);
    const xToPx = (x) => pad + ((x - minX) / (maxX - minX || 1)) * (w - pad * 2);
    const yToPx = (y) => h - pad - ((y - minY) / (maxY - minY || 1)) * (h - pad * 2);

    ctx.strokeStyle = 'rgba(255,255,255,0.06)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = pad + ((h - pad * 2) / 4) * i;
      ctx.beginPath(); ctx.moveTo(pad, y); ctx.lineTo(w - pad, y); ctx.stroke();
    }

    const plot = (arr, color) => {
      if (arr.length < 2) return;
      ctx.beginPath();
      arr.forEach(([x, y], i) => {
        const px = xToPx(x), py = yToPx(y);
        if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
      });
      ctx.strokeStyle = color;
      ctx.lineWidth = 2;
      ctx.shadowColor = color;
      ctx.shadowBlur = 5;
      ctx.stroke();
      ctx.shadowBlur = 0;
      const [lx, ly] = arr[arr.length - 1];
      ctx.beginPath();
      ctx.arc(xToPx(lx), yToPx(ly), 3.5, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
    };
    plot(this.points.primary, '#38e0ff');
    plot(this.points.booster, '#ffb454');

    ctx.fillStyle = 'rgba(255,255,255,0.35)';
    ctx.font = '10px monospace';
    ctx.fillText('downrange →', w - 90, h - 8);
    ctx.save();
    ctx.translate(10, pad + 8);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText('altitude ↑', 0, 0);
    ctx.restore();
  }
}

window.StripChart = StripChart;
window.TrajectoryCanvas = TrajectoryCanvas;
