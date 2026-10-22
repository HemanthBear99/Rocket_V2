/* Static SVG charts for the report and campaign pages (print/PDF friendly). */
(function () {
  'use strict';

  const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));

  function niceTicks(lo, hi, n = 5) {
    if (!(hi > lo)) hi = lo + 1;
    const raw = (hi - lo) / n;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= raw) || raw;
    const ticks = [];
    for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) ticks.push(+v.toFixed(10));
    return ticks;
  }

  function frame(w, h, xr, yr, opts) {
    const m = { l: 52, r: 14, t: 26, b: 36 };
    const sx = (x) => m.l + ((x - xr[0]) / (xr[1] - xr[0] || 1)) * (w - m.l - m.r);
    const sy = (y) => h - m.b - ((y - yr[0]) / (yr[1] - yr[0] || 1)) * (h - m.t - m.b);
    let g = `<rect x="${m.l}" y="${m.t}" width="${w - m.l - m.r}" height="${h - m.t - m.b}" class="rc-plot"/>`;
    niceTicks(yr[0], yr[1]).forEach((v) => {
      g += `<line x1="${m.l}" x2="${w - m.r}" y1="${sy(v)}" y2="${sy(v)}" class="rc-grid"/>`
        + `<text x="${m.l - 6}" y="${sy(v) + 3}" text-anchor="end" class="rc-tick">${v}</text>`;
    });
    niceTicks(xr[0], xr[1], 6).forEach((v) => {
      g += `<text x="${sx(v)}" y="${h - m.b + 14}" text-anchor="middle" class="rc-tick">${v}</text>`;
    });
    g += `<text x="${m.l}" y="16" class="rc-title">${esc(opts.title || '')}</text>`
      + `<text x="${(m.l + w - m.r) / 2}" y="${h - 4}" text-anchor="middle" class="rc-axis">${esc(opts.xlabel || '')}</text>`
      + `<text transform="translate(12 ${(m.t + h - m.b) / 2}) rotate(-90)" text-anchor="middle" class="rc-axis">${esc(opts.ylabel || '')}</text>`;
    return { g, sx, sy, m };
  }

  /** series: [{xs, ys, color, label}] */
  function lineChart(series, opts = {}) {
    const w = opts.width || 520, h = opts.height || 240;
    const all = series.filter((s) => s.xs.length);
    if (!all.length) return '<div class="help-text">No data</div>';
    const xs = all.flatMap((s) => s.xs), ys = all.flatMap((s) => s.ys).filter(Number.isFinite);
    const f = frame(w, h, [Math.min(...xs), Math.max(...xs)], [Math.min(0, ...ys), Math.max(...ys) * 1.05], opts);
    let paths = '', legend = '';
    all.forEach((s, i) => {
      const step = Math.max(1, Math.floor(s.xs.length / 600));
      let d = '';
      for (let j = 0; j < s.xs.length; j += step) {
        if (Number.isFinite(s.ys[j])) d += `${d ? 'L' : 'M'}${f.sx(s.xs[j]).toFixed(1)},${f.sy(s.ys[j]).toFixed(1)}`;
      }
      paths += `<path d="${d}" fill="none" stroke="${s.color}" stroke-width="1.8"/>`;
      legend += `<g transform="translate(${w - f.m.r - 140} ${f.m.t + 8 + i * 15})"><rect width="10" height="3" y="-4" fill="${s.color}"/><text x="14" class="rc-tick">${esc(s.label)}</text></g>`;
    });
    return `<svg viewBox="0 0 ${w} ${h}" class="rc-svg">${f.g}${paths}${legend}</svg>`;
  }

  /** values: number[]; limit: optional pass/fail threshold line */
  function histogram(values, opts = {}) {
    const w = opts.width || 520, h = opts.height || 220;
    const v = values.filter(Number.isFinite);
    if (!v.length) return '<div class="help-text">No data</div>';
    const lo = Math.min(...v, opts.limit ?? Infinity, 0), hi = Math.max(...v, opts.limit ?? -Infinity);
    const nb = opts.bins || 20, bw = (hi - lo) / nb || 1;
    const counts = new Array(nb).fill(0);
    v.forEach((x) => { counts[Math.min(nb - 1, Math.floor((x - lo) / bw))] += 1; });
    const f = frame(w, h, [lo, lo + nb * bw], [0, Math.max(...counts) * 1.1], { ...opts, ylabel: opts.ylabel || 'Runs' });
    let bars = '';
    counts.forEach((c, i) => {
      const x0 = f.sx(lo + i * bw), x1 = f.sx(lo + (i + 1) * bw);
      const bad = opts.limit !== undefined && lo + (i + 0.5) * bw > opts.limit;
      bars += `<rect x="${x0 + 1}" y="${f.sy(c)}" width="${Math.max(x1 - x0 - 2, 1)}" height="${f.sy(0) - f.sy(c)}" fill="${bad ? 'var(--red)' : 'var(--accent)'}" opacity="0.85"/>`;
    });
    if (opts.limit !== undefined) {
      const x = f.sx(opts.limit);
      bars += `<line x1="${x}" x2="${x}" y1="${f.m.t}" y2="${f.sy(0)}" stroke="var(--amber)" stroke-dasharray="4 3"/>`
        + `<text x="${x + 4}" y="${f.m.t + 12}" class="rc-tick" fill="var(--amber)">limit ${opts.limit}</text>`;
    }
    return `<svg viewBox="0 0 ${w} ${h}" class="rc-svg">${f.g}${bars}</svg>`;
  }

  /** entries: [[label, count]] horizontal bars */
  function barList(entries, total) {
    if (!entries.length) return '<div class="help-text">No failures recorded — every case passed.</div>';
    const max = Math.max(...entries.map((e) => e[1]));
    return entries.sort((a, b) => b[1] - a[1]).map(([k, n]) => `
      <div class="rc-bar-row"><div class="rc-bar-label">${esc(k.replace(/_/g, ' '))}</div>
        <div class="rc-bar-track"><div class="rc-bar-fill" style="width:${(100 * n / max).toFixed(1)}%"></div></div>
        <div class="rc-bar-val">${n}${total ? ` <span>(${(100 * n / total).toFixed(0)}%)</span>` : ''}</div></div>`).join('');
  }

  /** Semicircle success gauge with confidence interval arc. */
  function gauge(rate, ci) {
    const r = 70, cx = 90, cy = 86;
    const pt = (f) => [cx - r * Math.cos(Math.PI * f), cy - r * Math.sin(Math.PI * f)];
    const arc = (a, b, cls) => {
      const [x0, y0] = pt(a), [x1, y1] = pt(b);
      return `<path d="M${x0},${y0} A${r},${r} 0 0 1 ${x1},${y1}" class="${cls}"/>`;
    };
    const color = rate >= 0.9 ? 'var(--green)' : rate >= 0.7 ? 'var(--amber)' : 'var(--red)';
    return `<svg viewBox="0 0 180 104" class="rc-gauge">${arc(0, 1, 'rc-g-bg')}
      ${ci ? arc(ci[0], ci[1], 'rc-g-ci') : ''}
      <path d="M${pt(0).join(',')} A${r},${r} 0 0 1 ${pt(Math.max(rate, 0.001)).join(',')}" stroke="${color}" class="rc-g-val"/>
      <text x="${cx}" y="${cy - 6}" text-anchor="middle" class="rc-g-num">${(rate * 100).toFixed(0)}%</text>
      <text x="${cx}" y="${cy + 12}" text-anchor="middle" class="rc-tick">${ci ? `95% CI ${(ci[0] * 100).toFixed(0)}–${(ci[1] * 100).toFixed(0)}%` : ''}</text></svg>`;
  }

  window.ReportCharts = { lineChart, histogram, barList, gauge };
})();
