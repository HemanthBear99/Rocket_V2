'use client';
import { useEffect, useMemo, useRef, useState } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { EffectComposer, Bloom, Vignette } from '@react-three/postprocessing';
import * as THREE from 'three';
import { MONTE_CARLO } from '@/lib/data';
import { clamp01, lerp } from '@/lib/mission';

const radius = (e) => 2 + 7.5 * Math.log10(1 + e);   // log scale: 1 m, 10 m and 50 m rings are readable

function Ring({ r, color, dashed }) {
  const line = useMemo(() => {
    const pts = []; for (let i = 0; i <= 160; i++) { const a = i / 160 * Math.PI * 2; pts.push(new THREE.Vector3(Math.cos(a) * r, 0.04, Math.sin(a) * r)); }
    const g = new THREE.BufferGeometry().setFromPoints(pts);
    const m = dashed ? new THREE.LineDashedMaterial({ color, dashSize: 0.7, gapSize: 0.5, toneMapped: false }) : new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.55 });
    const l = new THREE.Line(g, m); if (dashed) l.computeLineDistances(); return l;
  }, [r, color, dashed]);
  return <primitive object={line} />;
}

function Drops({ counterRef, reduced, visibleRef }) {
  const drops = useMemo(() => MONTE_CARLO.map((d, i) => {
    const bearing = (i * 2.399963) % (Math.PI * 2);
    const r = radius(Math.min(d.error, 300)) + (d.error >= 300 ? 4 : 0);
    return { x: Math.cos(bearing) * r, z: Math.sin(bearing) * r, ok: d.ok, delay: i * 0.07 + (i % 7) * 0.03 };
  }), []);
  const heads = useRef(), marks = useRef(), trails = useRef();
  const dummy = useMemo(() => new THREE.Object3D(), []);
  const colors = useMemo(() => drops.map((d) => (d.ok ? new THREE.Color(0.6, 3.2, 2.0) : new THREE.Color(4.2, 0.7, 0.8))), [drops]);
  const trailGeo = useMemo(() => {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(drops.length * 6), 3));
    const c = new Float32Array(drops.length * 6);
    drops.forEach((d, i) => { const col = colors[i]; c.set([col.r * 0.6, col.g * 0.6, col.b * 0.6, 0, 0, 0], i * 6); });
    g.setAttribute('color', new THREE.BufferAttribute(c, 3));
    return g;
  }, [drops, colors]);
  const start = useRef(null);

  useEffect(() => {
    drops.forEach((_, i) => { heads.current.setColorAt(i, colors[i]); marks.current.setColorAt(i, colors[i]); });
    heads.current.instanceColor.needsUpdate = true; marks.current.instanceColor.needsUpdate = true;
    // The materials compiled before instance colours existed; recompile with them.
    heads.current.material.needsUpdate = true; marks.current.material.needsUpdate = true;
  }, [drops, colors]);

  useFrame((state) => {
    if (!visibleRef.current) { start.current = null; return; }
    if (start.current === null) start.current = state.clock.elapsedTime;
    const t = reduced ? 1e6 : (state.clock.elapsedTime - start.current) % 13;
    const a = state.clock.elapsedTime * 0.08;
    state.camera.position.set(Math.cos(a) * 54, 36, Math.sin(a) * 54); state.camera.lookAt(0, 0, 0);
    const pos = trailGeo.attributes.position.array;
    let landed = 0;
    drops.forEach((d, i) => {
      const k = clamp01((t - d.delay) / 0.9), e = 1 - Math.pow(1 - k, 3), y = lerp(48, 0.35, e);
      dummy.position.set(d.x, k > 0 && k < 1 ? y : -100, d.z); dummy.scale.setScalar(d.ok ? 1 : 1.4); dummy.updateMatrix();
      heads.current.setMatrixAt(i, dummy.matrix);
      const top = k > 0 && k < 1 ? Math.min(y + 10 * (1 - e) + 0.6, 48) : -100;
      pos.set([d.x, top, d.z, d.x, k > 0 && k < 1 ? y : -100, d.z], i * 6);
      if (k >= 1) {
        landed++;
        const age = clamp01((t - d.delay - 0.9) / 0.7);
        dummy.position.set(d.x, 0.06, d.z); dummy.rotation.set(-Math.PI / 2, 0, 0); dummy.scale.setScalar((d.ok ? 1 : 1.4) * (1 + (1 - age) * 2.4)); dummy.updateMatrix();
        dummy.rotation.set(0, 0, 0);
      } else { dummy.position.set(0, -100, 0); dummy.updateMatrix(); }
      marks.current.setMatrixAt(i, dummy.matrix);
    });
    heads.current.instanceMatrix.needsUpdate = true; marks.current.instanceMatrix.needsUpdate = true; trailGeo.attributes.position.needsUpdate = true;
    if (counterRef.current) counterRef.current.textContent = String(landed);
  });

  return (
    <>
      <instancedMesh ref={heads} args={[null, null, drops.length]}><sphereGeometry args={[0.3, 14, 10]} /><meshBasicMaterial toneMapped={false} /></instancedMesh>
      <instancedMesh ref={marks} args={[null, null, drops.length]}><ringGeometry args={[0.32, 0.46, 28]} /><meshBasicMaterial toneMapped={false} side={THREE.DoubleSide} /></instancedMesh>
      <lineSegments geometry={trailGeo}><lineBasicMaterial vertexColors toneMapped={false} transparent opacity={0.8} /></lineSegments>
    </>
  );
}

export default function MonteCarloPad() {
  const wrap = useRef(null), counter = useRef(null), visibleRef = useRef(false);
  const [reduced, setReduced] = useState(false);
  useEffect(() => {
    setReduced(window.matchMedia('(prefers-reduced-motion: reduce)').matches);
    const io = new IntersectionObserver(([e]) => { visibleRef.current = e.isIntersecting; }, { rootMargin: '100px' });
    io.observe(wrap.current); return () => io.disconnect();
  }, []);
  return (
    <div ref={wrap} className="mc-canvas" role="img" aria-label="3D landing pad with 100 Monte Carlo touchdowns: 94 green inside the 50 metre pad, 6 red failures">
      <Canvas dpr={[1, 1.75]} camera={{ fov: 36, position: [54, 36, 0] }}
        onCreated={({ scene }) => { scene.background = new THREE.Color('#060a14'); scene.fog = new THREE.Fog('#060a14', 70, 170); }}>
        <mesh rotation={[-Math.PI / 2, 0, 0]}><planeGeometry args={[400, 400]} /><meshBasicMaterial color="#0a1222" /></mesh>
        <gridHelper args={[220, 55, '#1c2741', '#101828']} position={[0, 0.02, 0]} />
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.03, 0]}><circleGeometry args={[radius(50), 96]} /><meshBasicMaterial color="#111a2c" /></mesh>
        <Ring r={radius(1)} color="#2c3d61" />
        <Ring r={radius(10)} color="#2c3d61" />
        <Ring r={radius(50)} color="#f5b84b" dashed />
        <Drops counterRef={counter} reduced={reduced} visibleRef={visibleRef} />
        <EffectComposer multisampling={0}>
          <Bloom mipmapBlur intensity={1.1} luminanceThreshold={0.7} radius={0.7} />
          <Vignette offset={0.3} darkness={0.7} />
        </EffectComposer>
      </Canvas>
      <div className="mc-counter"><span>CAMPAIGN · SEED 42</span><b><span ref={counter}>100</span> / 100 runs</b></div>
    </div>
  );
}
