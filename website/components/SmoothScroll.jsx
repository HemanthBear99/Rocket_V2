'use client';
import { useEffect } from 'react';
import Lenis from 'lenis';
import gsap from 'gsap';
import { ScrollTrigger } from 'gsap/ScrollTrigger';

/** Lenis inertial scrolling driven by GSAP's ticker, plus magnetic buttons. */
export default function SmoothScroll() {
  useEffect(() => {
    gsap.registerPlugin(ScrollTrigger);
    const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    let lenis = null;
    const raf = (time) => lenis && lenis.raf(time * 1000);
    if (!reduced) {
      lenis = new Lenis({ duration: 1.15, easing: (t) => 1 - Math.pow(1 - t, 3.2), smoothWheel: true });
      lenis.on('scroll', ScrollTrigger.update);
      gsap.ticker.add(raf);
      gsap.ticker.lagSmoothing(0);
      document.querySelectorAll('a[href^="#"]').forEach((a) => a.addEventListener('click', (e) => {
        const id = a.getAttribute('href'); if (id.length < 2) return;
        const el = document.querySelector(id); if (!el) return;
        e.preventDefault(); lenis.scrollTo(el, { offset: -10, duration: 1.6 });
      }));
    }

    const mags = Array.from(document.querySelectorAll('.magnetic'));
    const move = (e) => {
      const el = e.currentTarget, r = el.getBoundingClientRect();
      const x = (e.clientX - r.left - r.width / 2) * 0.22, y = (e.clientY - r.top - r.height / 2) * 0.32;
      gsap.to(el, { x, y, duration: 0.35, ease: 'power3.out' });
    };
    const leave = (e) => gsap.to(e.currentTarget, { x: 0, y: 0, duration: 0.6, ease: 'elastic.out(1, 0.4)' });
    if (!reduced) mags.forEach((m) => { m.addEventListener('pointermove', move); m.addEventListener('pointerleave', leave); });

    return () => {
      gsap.ticker.remove(raf); if (lenis) lenis.destroy();
      mags.forEach((m) => { m.removeEventListener('pointermove', move); m.removeEventListener('pointerleave', leave); });
    };
  }, []);
  return null;
}
