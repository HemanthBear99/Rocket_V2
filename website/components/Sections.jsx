'use client';
import { useEffect, useRef, useState } from 'react';
import dynamic from 'next/dynamic';
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';
import AltitudeCurves from './AltitudeCurves';
import { FLIGHTS, SPECS, PLANS, FAQS, DOWNLOAD_URL } from '@/lib/data';

const MonteCarloPad = dynamic(() => import('./MonteCarloPad'), { ssr: false });

export function Nav() {
  const ref = useRef(null);
  useEffect(() => {
    const on = () => ref.current && ref.current.classList.toggle('solid', window.scrollY > window.innerHeight * 5.02);
    window.addEventListener('scroll', on, { passive: true }); on();
    return () => window.removeEventListener('scroll', on);
  }, []);
  return (
    <header ref={ref} className="nav">
      <nav className="wrap nav-inner">
        <a href="#top" className="brand" aria-label="Vyom home">
          <svg width="30" height="30" viewBox="0 0 32 32" fill="none" aria-hidden="true"><path d="M5 27C5.5 10 24 4.5 25.5 22.5" stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" /><path d="M21 28.2h9" stroke="#FF7A3D" strokeWidth="2.4" strokeLinecap="round" /><circle cx="25.5" cy="22.5" r="2.1" fill="#FF7A3D" /></svg>
          <span>Vyom</span>
        </a>
        <div className="nav-links">
          <a href="#proof">Results</a><a href="#validation">Validation</a><a href="#pricing">Pricing</a><a href="#faq">FAQ</a>
        </div>
        <a className="btn btn-primary btn-sm magnetic" href="#download">Free trial</a>
      </nav>
    </header>
  );
}

function useReveal() {
  useEffect(() => {
    gsap.registerPlugin(ScrollTrigger);
    if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return;
    const els = gsap.utils.toArray('[data-reveal]');
    const tweens = els.map((el) => gsap.fromTo(el, { y: 40, opacity: 0.001 }, {
      y: 0, opacity: 1, duration: 1.1, ease: 'expo.out',
      scrollTrigger: { trigger: el, start: 'top 88%', once: true },
    }));
    return () => tweens.forEach((t) => { t.scrollTrigger && t.scrollTrigger.kill(); t.kill(); });
  }, []);
}

function Stat({ value, suffix, prefix, color, children }) {
  const n = useRef(null);
  useEffect(() => {
    gsap.registerPlugin(ScrollTrigger);
    const obj = { v: 0 };
    const tw = gsap.to(obj, { v: value, duration: 1.6, ease: 'power3.out', paused: true, onUpdate: () => { if (n.current) n.current.textContent = Math.round(obj.v); } });
    const st = ScrollTrigger.create({ trigger: n.current, start: 'top 85%', once: true, onEnter: () => { obj.v = 0; tw.restart(); } });
    return () => { st.kill(); tw.kill(); };
  }, [value]);
  return (
    <div className="stat" data-reveal>
      <b style={{ color }}>{prefix}<span ref={n}>{value}</span>{suffix}</b>
      <span>{children}</span>
    </div>
  );
}

export default function Sections() {
  useReveal();
  const [cur, setCur] = useState('inr');
  return (
    <>
      <section id="proof" className="wrap section">
        <div className="split-head" data-reveal>
          <h2 className="h2">That flight took 35 seconds to compute.</h2>
          <p className="lede">Every number on this page comes from the simulator itself or from published flight telemetry. Nothing here is a mock-up.</p>
        </div>
        <div className="grid4">
          <Stat value={94} suffix="%" color="var(--good)">of 100 dispersed Monte Carlo missions succeed (95% CI 87.5–97.2%)</Stat>
          <Stat value={4} prefix="±" suffix="%">stage-1 cutoff speed against four Falcon&nbsp;9 flights</Stat>
          <Stat value={35} suffix={<small> s</small>}>per full mission, down from 290 s</Stat>
          <Stat value={263}>automated physics and regression tests</Stat>
        </div>
      </section>

      <section className="band mc">
        <div className="mc-copy" data-reveal>
          <div className="eyebrow">Monte Carlo campaigns</div>
          <h2 className="h2">100 boosters.<br />94 on the pad.</h2>
          <p className="lede">Thrust, Isp, mass, wind, air density and pitch-over are dispersed together. Each streak is one run from the latest campaign, falling to where it touched down.</p>
          <div className="legend"><span><i style={{ background: 'var(--good)' }} />Landed</span><span><i style={{ background: 'var(--bad)' }} />Failed a gate</span><span><i className="dash" />50 m pad edge</span></div>
          <p className="fine">Distance from the pad centre is real (log scale). Bearing is illustrative.</p>
        </div>
        <MonteCarloPad />
      </section>

      <section id="validation" className="wrap section">
        <div className="split-head" data-reveal>
          <div><div className="eyebrow">Validation</div><h2 className="h2">Flown against the real thing</h2></div>
          <p className="lede">Four Falcon&nbsp;9 ascents, extracted from SpaceX launch webcasts, with the simulator's run traced over them as you scroll. Where the model disagrees, it says so.</p>
        </div>
        <div className="panel"><AltitudeCurves /></div>
        <div className="grid3">
          <div className="verdict good" data-reveal><span className="eyebrow">Matches</span><h3>Cutoff speed within 1.4–3.9%</h3><p>Boostback ignition lands within 2% of the real timeline on every flight that shows it.</p></div>
          <div className="verdict warn" data-reveal><span className="eyebrow">Known gap</span><h3>Cutoff comes 6–10% early</h3><p>The simulated booster holds back a larger landing reserve than Falcon&nbsp;9 does.</p></div>
          <div className="verdict warn" data-reveal><span className="eyebrow">Known gap</span><h3>Descent drag about 1.5× high</h3><p>Fitted to flight data near Mach&nbsp;1. Import your own tunnel or CFD tables to replace it.</p></div>
        </div>
        <div className="table-scroll" data-reveal>
          <table className="tbl">
            <thead><tr><th>Flight</th><th>Cutoff speed, flight (m/s)</th><th>Cutoff speed, sim (m/s)</th><th>Difference</th><th>Boostback, flight</th><th>Boostback, sim</th></tr></thead>
            <tbody>{FLIGHTS.map((f) => <tr key={f.name}><td>{f.name}</td><td>{f.real}</td><td>{f.sim}</td><td className="good-text">{f.diff}</td><td>{f.bbReal}</td><td>{f.bbSim}</td></tr>)}</tbody>
          </table>
        </div>
      </section>

      <section className="band">
        <div className="wrap section">
          <div data-reveal><div className="eyebrow">Inside the simulator</div><h2 className="h2">Built for people<br />who design rockets</h2></div>
          <div className="specs">{SPECS.map((s) => <div key={s.k} className="spec" data-reveal><span className="k">{s.k}</span><h3>{s.title}</h3><p>{s.text}</p></div>)}</div>
        </div>
      </section>

      <section id="pricing" className="wrap section">
        <div className="split-head" data-reveal>
          <div><div className="eyebrow">Pricing</div><h2 className="h2">Every plan starts<br />with 14 days free</h2></div>
          <div className="seg" role="group" aria-label="Currency">
            <button type="button" className={cur === 'inr' ? 'on' : ''} aria-pressed={cur === 'inr'} onClick={() => setCur('inr')}>₹ INR</button>
            <button type="button" className={cur === 'usd' ? 'on' : ''} aria-pressed={cur === 'usd'} onClick={() => setCur('usd')}>$ USD</button>
          </div>
        </div>
        <div className="grid4">
          {PLANS.map((p) => (
            <div key={p.name} className={p.hot ? 'price hot' : 'price'} data-reveal>
              <div className="price-head"><h3>{p.name}</h3>{p.hot && <span className="eyebrow">Most popular</span>}</div>
              <p className="who">{p.who}</p>
              <div className="amt-row"><span className="amt">{cur === 'inr' ? p.inr : p.usd}</span><span>{p.per}</span></div>
              <ul>{p.items.map((it) => <li key={it}><span className="good-text">✓</span>{it}</li>)}</ul>
              <a className={p.hot ? 'btn btn-primary' : 'btn btn-ghost'} href={p.href}>{p.cta}</a>
            </div>
          ))}
        </div>
        <p className="fine">Prices per year, excluding taxes. Academic pricing needs an institutional email. Vyom is an engineering and teaching tool, not certified for flight-critical decisions.</p>
      </section>

      <section id="download" className="download">
        <div className="wrap download-inner">
          <div data-reveal>
            <div className="eyebrow">Free for 14 days · every feature</div>
            <h2 className="h2 huge">Your first<br />landing is<br />5 minutes away</h2>
            <div className="row-gap">
              <a className="btn btn-primary magnetic" href={DOWNLOAD_URL}>Download for Windows
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" aria-hidden="true"><path d="M12 4v12M6 10l6 6 6-6M4 20h16" /></svg></a>
              <a className="btn btn-ghost magnetic" href="mailto:sales@example.com">Licence a university lab</a>
            </div>
          </div>
          <dl className="specsheet" data-reveal>
            <dt>FILE</dt><dd>VyomSetup-0.3.0.exe · 181 MB</dd>
            <dt>SYSTEM</dt><dd>Windows 10 or 11, 64-bit, 8 GB RAM</dd>
            <dt>INSTALL</dt><dd>No admin rights needed</dd>
            <dt>DATA</dt><dd>Stays on your PC, works offline</dd>
          </dl>
        </div>
      </section>

      <section id="faq" className="wrap section faq">
        <div data-reveal><div className="eyebrow">FAQ</div><h2 className="h2">Questions engineers ask</h2></div>
        <div>{FAQS.map((f) => <details key={f.q}><summary>{f.q}</summary><p>{f.a}</p></details>)}</div>
      </section>

      <footer className="footer">
        <div className="wrap footer-inner">
          <span>© 2026 Vyom · Made in India</span>
          <span className="mono">v0.3.0 · Flight data: shahar603/Telemetry-Data · Earth imagery: NASA</span>
        </div>
      </footer>
    </>
  );
}
