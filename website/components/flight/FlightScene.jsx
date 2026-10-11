'use client';
import { Suspense, useEffect, useMemo, useRef } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import { Stars } from '@react-three/drei';
import { EffectComposer, Bloom, Vignette, Noise, ToneMapping } from '@react-three/postprocessing';
import { ToneMappingMode } from 'postprocessing';
import * as THREE from 'three';
import { Booster, UpperStage, S1_LEN } from './Rocket';
import Earth from './Earth';
import LaunchSite from './LaunchSite';
import { Particles } from './particles';
import { flight, pose, makePose, smooth, lerp, clamp01, LZ } from '@/lib/mission';

const Z = new THREE.Vector3(0, 0, 1);
const UP = new THREE.Vector3(0, 1, 0);

function glowTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 128;
  const g = c.getContext('2d'); const gr = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  gr.addColorStop(0, 'rgba(255,255,255,1)'); gr.addColorStop(0.2, 'rgba(255,255,255,.55)'); gr.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = gr; g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}

function Flight({ lowPower }) {
  const { scene, camera } = useThree();
  const boosterRef = useRef(), upperRef = useRef(), finRefs = useRef([]), legRefs = useRef([]);
  const glowRef = useRef(), s2GlowRef = useRef(), paintRef = useRef(), beaconRef = useRef();
  const st = useMemo(() => makePose(), []);
  const fire = useMemo(() => new Particles(lowPower ? 1400 : 3200), [lowPower]);
  const smoke = useMemo(() => new Particles(lowPower ? 500 : 1100, { additive: false, smoke: true }), [lowPower]);
  const tex = useMemo(() => glowTexture(), []);
  const sprites = useMemo(() => {
    const mk = (color) => new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, color, transparent: true, blending: THREE.AdditiveBlending, depthWrite: false, toneMapped: false, opacity: 0 }));
    return { flare: mk(new THREE.Color(5, 2.6, 1.2)), s2: mk(new THREE.Color(1.6, 2.6, 5)), plasma: mk(new THREE.Color(6, 1.8, 0.5)) };
  }, [tex]);
  const shock = useMemo(() => new THREE.Mesh(new THREE.RingGeometry(0.85, 1, 96),
    new THREE.MeshBasicMaterial({ color: new THREE.Color(3, 2.4, 1.8), transparent: true, opacity: 0, side: THREE.DoubleSide, depthWrite: false, toneMapped: false })), []);
  const engineLight = useMemo(() => new THREE.PointLight('#ff9a52', 0, 160, 1.6), []);
  const cam = useMemo(() => ({ pos: new THREE.Vector3(34, 9, 48), look: new THREE.Vector3(0, 8, 0), touch: -1 }), []);
  const tmp = useMemo(() => ({ axis: new THREE.Vector3(), tail: new THREE.Vector3(), down: new THREE.Vector3(), a: new THREE.Vector3(), b: new THREE.Vector3(), side: new THREE.Vector3() }), []);

  useEffect(() => {
    shock.rotation.x = -Math.PI / 2; shock.position.set(LZ.x, 0.25, LZ.z);
    const objs = [fire.points, smoke.points, sprites.flare, sprites.s2, sprites.plasma, shock, engineLight];
    objs.forEach((o) => scene.add(o));
    return () => { objs.forEach((o) => scene.remove(o)); fire.dispose(); smoke.dispose(); };
  }, [scene, fire, smoke, sprites, shock, engineLight]);

  useFrame((state, delta) => {
    const dt = Math.min(delta, 0.05), now = state.clock.elapsedTime;
    const p = flight.p;
    pose(p, st); flight.thr = st.thr;
    const B = boosterRef.current, U = upperRef.current; if (!B || !U) return;

    B.position.copy(st.pos); B.rotation.set(0, 0, st.tilt);
    finRefs.current.forEach((f) => { if (f) f.rotation.z = lerp(-Math.PI / 2 + 0.06, 0, st.fins); });
    legRefs.current.forEach((l) => { if (l) l.rotation.z = lerp(0, -2.5, st.legs); });
    if (paintRef.current) paintRef.current.userData.soot.value = smooth(0.42, 0.76, p);
    if (glowRef.current) glowRef.current.opacity = st.thr;

    if (st.s2Attached) { U.position.copy(tmp.a.set(0, S1_LEN + 0.5, 0).applyAxisAngle(Z, st.tilt).add(st.pos)); U.rotation.set(0, 0, st.tilt); }
    else { U.position.copy(st.s2pos); U.rotation.set(0, 0, st.s2tilt); }
    if (s2GlowRef.current) s2GlowRef.current.opacity = st.s2Attached ? 0 : st.s2thr;

    const axis = tmp.axis.copy(UP).applyAxisAngle(Z, st.tilt);
    const tail = tmp.tail.copy(axis).multiplyScalar(-0.3).add(st.pos);
    const down = tmp.down.copy(axis).negate();
    const alt = st.pos.y;

    if (st.thr > 0.01) {
      const n = Math.round((st.engines === 9 ? 70 : st.engines === 3 ? 38 : 18) * st.thr * (lowPower ? 0.55 : 1));
      const spread = 2.2 + Math.min(alt * 0.05, 26);
      fire.emit(n, tail, down, 70 + alt * 0.3, spread, alt < 40 ? 0.5 : 0.85, st.engines === 9 ? 1.1 : 0.8, alt < 60 ? 3.2 : 8 + alt * 0.018, 0);
      if (alt < 45) smoke.emit(Math.round(12 * st.thr), tmp.a.set(st.pos.x, 0.6, st.pos.z), tmp.b.set(0, 0.12, 0), 22, 20, 5, 5, 30);
    }
    if (st.steam > 0.02 && Math.random() < 0.7) smoke.emit(2, tmp.a.set(0.5, 1.5 + Math.random() * 8, 0.4), tmp.b.set(1, 0.25, 0.4), 2.4, 1.2, 3.2, 0.8, 7);
    if (st.rcs && Math.random() < 0.85) {
      const top = tmp.a.copy(axis).multiplyScalar(S1_LEN - 0.1).add(st.pos);
      const side = tmp.side.copy(Z).cross(axis).normalize(); if (Math.random() < 0.5) side.negate();
      fire.emit(3, top, side, 16, 1.4, 0.35, 0.5, 2.6, 2);
    }
    if (!st.s2Attached && st.s2thr > 0.01) {
      const ax2 = tmp.b.copy(UP).applyAxisAngle(Z, st.s2tilt);
      fire.emit(Math.round(26 * st.s2thr), tmp.a.copy(ax2).multiplyScalar(-0.6).add(st.s2pos), ax2.negate(), 75, 12, 1.3, 1.6, 26, 1);
    }
    fire.update(dt); smoke.update(dt);

    const camDist = 26 + Math.min(alt, 800) * 1.12;
    engineLight.position.copy(tail); engineLight.intensity = st.thr * (st.engines === 9 ? 900 : 420) * (1 + 0.15 * Math.sin(now * 40));
    sprites.flare.position.copy(tail); sprites.flare.material.opacity = st.thr * 0.8;
    const fs = (st.engines === 9 ? 3.6 : 2.4) + camDist * 0.03; sprites.flare.scale.set(fs, fs, 1);
    sprites.s2.position.copy(U.position); sprites.s2.material.opacity = st.s2Attached ? 0 : st.s2thr * 0.8;
    const f2 = 5 + camDist * 0.06; sprites.s2.scale.set(f2, f2, 1);
    sprites.plasma.position.copy(tail).addScaledVector(down, 1.2);
    sprites.plasma.material.opacity = st.plasma * (0.75 + 0.25 * Math.sin(now * 23));
    const ps = 7 + camDist * 0.045; sprites.plasma.scale.set(ps, ps * 1.35, 1);
    if (beaconRef.current) beaconRef.current.color.setRGB(Math.sin(now * 3.2) > 0 ? 8 : 0.4, 0.5, 0.4);

    if (p >= 0.95) { if (cam.touch < 0) cam.touch = now; } else cam.touch = -1;
    if (cam.touch > 0) {
      const k = Math.min((now - cam.touch) / 1.8, 1);
      shock.scale.setScalar(1 + k * 30); shock.material.opacity = (1 - k) * 0.9;
      if (now - cam.touch < 0.6) smoke.emit(8, tmp.a.set(LZ.x, 0.7, LZ.z), tmp.b.set(0, 0.04, 0), 28, 30, 3.5, 4, 24);
    } else shock.material.opacity = 0;

    // Cinematic camera: orbit the pad, follow the climb out to where the Earth curves away, then drop to LZ-1.
    const want = tmp.a, look = tmp.b;
    if (p < 0.1) {
      const o = now * 0.05;
      want.set(Math.cos(o) * 32 + 4, 7 + p * 50, Math.sin(o) * 32 + 34); look.set(0, 8 + p * 30, 0);
    } else if (p > 0.84) {
      const k = smooth(0.84, 0.9, p);
      want.set(LZ.x + 24, Math.max(st.pos.y * 0.55, 3.5), LZ.z + 32).lerp(tmp.side.copy(st.pos).add({ x: -camDist * 0.5, y: camDist * 0.2, z: camDist }), 1 - k);
      look.copy(st.pos).addScaledVector(axis, 3);
    } else {
      const hi = smooth(0.14, 0.32, p) * (1 - smooth(0.74, 0.86, p));
      look.copy(st.pos).addScaledVector(axis, 4);
      want.copy(look).add({ x: -camDist * 0.55, y: camDist * (0.18 + 0.4 * hi), z: camDist });
    }
    cam.pos.lerp(want, 1 - Math.pow(0.0009, dt)); cam.look.lerp(look, 1 - Math.pow(0.0002, dt));
    camera.position.copy(cam.pos); camera.lookAt(cam.look);

    const hiK = clamp01(cam.pos.y / 650);
    scene.fog.density = lerp(0.0032, 0.00001, Math.pow(hiK, 0.3));
    scene.fog.color.setRGB(lerp(0.035, 0.002, hiK), lerp(0.065, 0.005, hiK), lerp(0.15, 0.014, hiK));
    scene.background.copy(scene.fog.color);
  });

  return (
    <>
      <Booster ref={boosterRef} finRefs={finRefs} legRefs={legRefs} glowRef={glowRef} paintRef={paintRef} />
      <UpperStage ref={upperRef} glowRef={s2GlowRef} />
      <LaunchSite beaconRef={beaconRef} />
    </>
  );
}

export default function FlightScene({ active }) {
  const lowPower = typeof window !== 'undefined' && (window.innerWidth < 760 || (navigator.hardwareConcurrency || 8) <= 4);
  return (
    <Canvas
      frameloop={active ? 'always' : 'never'}
      dpr={[1, lowPower ? 1.25 : 1.75]}
      gl={{ antialias: false, powerPreference: 'high-performance' }}
      camera={{ fov: 40, near: 1.5, far: 60000, position: [34, 9, 48] }}
      onCreated={({ scene }) => { scene.background = new THREE.Color('#09101f'); scene.fog = new THREE.FogExp2('#09101f', 0.0032); }}
      aria-hidden="true"
    >
      <hemisphereLight args={['#6f8fd0', '#0a0f1c', 0.6]} />
      <directionalLight position={[-300, 500, 400]} intensity={1.1} color="#c4d6ff" />
      <directionalLight position={[120, 60, 160]} intensity={0.55} color="#ffd9b8" />
      <Suspense fallback={null}><Earth /></Suspense>
      <Stars radius={20000} depth={8000} count={lowPower ? 3000 : 7000} factor={260} saturation={0} fade speed={0.3} />
      <Flight lowPower={lowPower} />
      <EffectComposer multisampling={0} disableNormalPass>
        <Bloom mipmapBlur intensity={1.25} luminanceThreshold={0.82} luminanceSmoothing={0.2} radius={0.78} />
        <Noise opacity={0.045} premultiply />
        <Vignette offset={0.28} darkness={0.72} />
        <ToneMapping mode={ToneMappingMode.ACES_FILMIC} />
      </EffectComposer>
    </Canvas>
  );
}
