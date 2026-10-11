'use client';
import { useEffect, useRef, useState } from 'react';
import dynamic from 'next/dynamic';
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';
import { flight, interp, smooth, P2T, T2A, T2V, PHASES } from '@/lib/mission';
import { CHAPTERS } from '@/lib/data';

const FlightScene = dynamic(() => import('./FlightScene'), { ssr: false });

function Headline() {
  const lines = ['Launch it.', 'Bring it back.'];
  let i = 0;
  return (
    <h1 className="hero-title" aria-label="Launch it. Bring it back.">
      {lines.map((line, li) => (
        <span key={li} aria-hidden="true" className={li ? 'accent' : ''}>
          {Array.from(line).map((ch) => {
            const d = 0.15 + (i++) * 0.04 + li * 0.25;
            return <span key={i} className="letter" style={{ animationDelay: `${d.toFixed(2)}s` }}>{ch === ' ' ? ' ' : ch}</span>;
          })}
        </span>
      ))}
    </h1>
  );
}

export default function FlightSection() {
  const section = useRef(null), intro = useRef(null), cue = useRef(null), hud = useRef(null), rail = useRef(null);
  const hudT = useRef(null), hudAlt = useRef(null), hudVel = useRef(null), hudThr = useRef(null), hudBar = useRef(null), hudPhase = useRef(null);
  const chapterEls = useRef([]);
  const [active, setActive] = useState(true);

  useEffect(() => {
    gsap.registerPlugin(ScrollTrigger);
    const st = ScrollTrigger.create({
      trigger: section.current, start: 'top top', end: 'bottom bottom', scrub: true,
      onUpdate: (self) => { flight.p = self.progress; },
    });
    const io = new IntersectionObserver(([e]) => setActive(e.isIntersecting), { rootMargin: '200px' });
    io.observe(section.current);

    const tick = () => {
      const p = flight.p;
      const k = smooth(0, 0.06, p);
      if (intro.current) { intro.current.style.opacity = 1 - k; intro.current.style.transform = `translate3d(0, ${-80 * k}px, 0)`; intro.current.style.pointerEvents = k > 0.5 ? 'none' : 'auto'; }
      if (cue.current) cue.current.style.opacity = 1 - smooth(0, 0.025, p);
      if (hud.current) hud.current.style.opacity = smooth(0.05, 0.1, p);
      CHAPTERS.forEach((c, i) => {
        const el = chapterEls.current[i]; if (!el) return;
        const v = Math.min(smooth(c.a, c.a + 0.025, p), 1 - smooth(c.b - 0.025, c.b, p));
        el.style.opacity = v; el.style.transform = `translate3d(0, ${(1 - v) * 28}px, 0)`;
        el.style.filter = v < 1 ? `blur(${(1 - v) * 6}px)` : 'none';
      });
      const t = interp(P2T, p), s = Math.round(t);
      if (hudT.current) hudT.current.textContent = `T+${String(Math.floor(s / 60)).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
      if (hudAlt.current) hudAlt.current.textContent = `${interp(T2A, t).toFixed(1)} km`;
      if (hudVel.current) hudVel.current.textContent = `${Math.round(interp(T2V, t)).toLocaleString('en-US')} m/s`;
      if (hudThr.current) hudThr.current.textContent = `${Math.round(flight.thr * 100)}%`;
      if (hudBar.current) hudBar.current.style.transform = `scaleX(${flight.thr})`;
      if (hudPhase.current) { const ph = PHASES.find(([edge]) => p < edge); if (ph && hudPhase.current.textContent !== ph[1]) hudPhase.current.textContent = ph[1]; }
      if (rail.current) rail.current.style.transform = `scaleX(${p})`;
    };
    gsap.ticker.add(tick);
    return () => { gsap.ticker.remove(tick); st.kill(); io.disconnect(); };
  }, []);

  return (
    <section id="top" ref={section} className="flight">
      <div className="stage">
        <div className="stage-canvas"><FlightScene active={active} /></div>
        <div className="stage-shade" />

        <div ref={intro} className="intro wrap">
          <div className="eyebrow">6-DOF launch &amp; landing simulator</div>
          <Headline />
          <p className="lede intro-lede">Fly a reusable rocket to orbit and land its booster back on the pad. Full six-degree-of-freedom physics, convex-optimization landing guidance, and Monte Carlo campaigns checked against real Falcon&nbsp;9 flights.</p>
          <div className="row-gap">
            <a className="btn btn-primary magnetic" href="#download">Download 14-day trial
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" aria-hidden="true"><path d="M12 4v12M6 10l6 6 6-6M4 20h16" /></svg></a>
            <a className="btn btn-ghost magnetic" href="#validation">How accurate is it?</a>
          </div>
        </div>

        <div ref={cue} className="cue"><span>SCROLL TO FLY</span><span className="cue-mouse" aria-hidden="true"><i /></span></div>

        {CHAPTERS.map((c, i) => (
          <div key={i} ref={(el) => { chapterEls.current[i] = el; }} className="chapter">
            <span className="chapter-t">{c.t}</span>
            <h3>{c.title}</h3>
            <p>{c.text}</p>
          </div>
        ))}

        <div ref={hud} className="hud" aria-hidden="true">
          <div className="hud-row"><span>MISSION TIME</span><b ref={hudT}>T+00:00</b></div>
          <div className="hud-row"><span>BOOSTER ALT</span><b ref={hudAlt}>0.0 km</b></div>
          <div className="hud-row"><span>SPEED</span><b ref={hudVel}>0 m/s</b></div>
          <div className="hud-row"><span>THROTTLE</span><b ref={hudThr}>0%</b></div>
          <div className="hud-bar"><i ref={hudBar} /></div>
          <div ref={hudPhase} className="hud-phase">ON THE PAD</div>
        </div>
        <div className="rail" aria-hidden="true"><i ref={rail} /></div>
      </div>
    </section>
  );
}
