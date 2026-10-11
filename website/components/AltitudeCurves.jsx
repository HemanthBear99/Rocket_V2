'use client';
import { useEffect, useRef } from 'react';
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';
import { CURVES } from '@/lib/data';

const W = 1000, H = 440, M = { l: 56, r: 130, t: 16, b: 44 };
const X = (s) => M.l + (s / 146) * (W - M.l - M.r);
const Y = (km) => H - M.b - (km / 80) * (H - M.t - M.b);
const LABEL_NUDGE = { 'CRS-11': 8, 'NROL-76': -2, Zuma: 16, 'Orbcomm OG2': -12, Vyom: 0 };

/** Real Falcon 9 ascent curves with the simulator drawn over them, traced in as you scroll. */
export default function AltitudeCurves() {
  const root = useRef(null);
  useEffect(() => {
    gsap.registerPlugin(ScrollTrigger);
    const paths = root.current.querySelectorAll('path[data-draw]');
    const labels = root.current.querySelectorAll('[data-label]');
    paths.forEach((p) => { const len = p.getTotalLength(); p.style.strokeDasharray = `${len}`; p.style.strokeDashoffset = `${len}`; });
    gsap.set(labels, { opacity: 0 });
    const tl = gsap.timeline({ scrollTrigger: { trigger: root.current, start: 'top 75%', end: 'top 20%', scrub: 0.6 } });
    tl.to(paths, { strokeDashoffset: 0, ease: 'none', stagger: 0.08 }).to(labels, { opacity: 1, stagger: 0.05, duration: 0.2 }, '-=0.15');
    return () => { tl.scrollTrigger && tl.scrollTrigger.kill(); tl.kill(); };
  }, []);

  return (
    <svg ref={root} viewBox={`0 0 ${W} ${H}`} className="curves" role="img"
      aria-label="Altitude against time to stage-1 cutoff for four Falcon 9 flights, with the Vyom simulation drawn over them">
      <defs>
        <filter id="glow" x="-20%" y="-20%" width="140%" height="140%"><feGaussianBlur stdDeviation="4" result="b" /><feMerge><feMergeNode in="b" /><feMergeNode in="SourceGraphic" /></feMerge></filter>
      </defs>
      {[0, 20, 40, 60, 80].map((v) => (
        <g key={v}><line x1={M.l} x2={W - M.r} y1={Y(v)} y2={Y(v)} className="grid" /><text x={M.l - 10} y={Y(v) + 4} textAnchor="end" className="tick">{v} km</text></g>
      ))}
      {[0, 30, 60, 90, 120].map((v) => <text key={v} x={X(v)} y={H - M.b + 24} textAnchor="middle" className="tick">T+{v} s</text>)}
      {Object.entries(CURVES).map(([name, data]) => {
        const sim = name === 'Vyom';
        const d = data.map((a, i) => `${i ? 'L' : 'M'}${X(i * 2).toFixed(1)},${Y(a).toFixed(1)}`).join('');
        const lx = X((data.length - 1) * 2), ly = Y(data[data.length - 1]);
        return (
          <g key={name}>
            <path data-draw d={d} className={sim ? 'sim' : 'real'} filter={sim ? 'url(#glow)' : undefined} />
            <g data-label>
              <circle cx={lx} cy={ly} r={sim ? 4.5 : 2.5} className={sim ? 'sim-dot' : 'real-dot'} />
              <text x={lx + 10} y={ly + 4 + (LABEL_NUDGE[name] || 0)} className={sim ? 'sim-label' : 'tick'}>{sim ? 'Vyom sim' : name}</text>
            </g>
          </g>
        );
      })}
    </svg>
  );
}
