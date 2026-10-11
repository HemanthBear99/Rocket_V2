'use client';
import { forwardRef, useMemo } from 'react';
import * as THREE from 'three';

// 1 unit = 5 m. Falcon-class proportions: 3.7 m diameter, 42 m first stage.
export const R = 0.37;
export const S1_LEN = 8.4;

function bellGeometry(rThroat, rExit, len, segments = 28) {
  const pts = [];
  for (let i = 0; i <= 16; i++) {
    const t = i / 16;
    pts.push(new THREE.Vector2(rThroat + (rExit - rThroat) * Math.pow(t, 0.62), -t * len));
  }
  return new THREE.LatheGeometry(pts, segments);
}

// White paint that darkens with soot from the base up as the booster re-enters.
function useSootedPaint() {
  return useMemo(() => {
    const m = new THREE.MeshPhysicalMaterial({ color: '#e9edf3', roughness: 0.38, metalness: 0.12, clearcoat: 0.35, clearcoatRoughness: 0.4 });
    m.userData.soot = { value: 0 };
    m.onBeforeCompile = (shader) => {
      shader.uniforms.uSoot = m.userData.soot;
      shader.vertexShader = shader.vertexShader
        .replace('#include <common>', '#include <common>\nvarying vec3 vObj;')
        .replace('#include <begin_vertex>', '#include <begin_vertex>\nvObj = position;');
      shader.fragmentShader = shader.fragmentShader
        .replace('#include <common>', '#include <common>\nvarying vec3 vObj;\nuniform float uSoot;')
        .replace('#include <color_fragment>', `#include <color_fragment>
          float ang = atan(vObj.z, vObj.x);
          float streak = 0.55 + 0.45 * sin(ang * 31.0 + sin(vObj.y * 2.1) * 2.0);
          float h = clamp(1.0 - (vObj.y + 4.2) / 6.5, 0.0, 1.0);
          float soot = uSoot * smoothstep(0.0, 1.0, h) * streak;
          diffuseColor.rgb = mix(diffuseColor.rgb, vec3(0.09, 0.08, 0.07), clamp(soot, 0.0, 0.92));`);
    };
    return m;
  }, []);
}

const dark = new THREE.MeshStandardMaterial({ color: '#15181f', roughness: 0.55, metalness: 0.6 });
const black = new THREE.MeshStandardMaterial({ color: '#0b0d12', roughness: 0.8, metalness: 0.2 });
const finMat = new THREE.MeshStandardMaterial({ color: '#2c2f36', roughness: 0.35, metalness: 0.9 });

function GridFin() {
  const bars = [];
  for (let i = 0; i < 5; i++) bars.push(<mesh key={'v' + i} material={finMat} position={[0.06 + i * 0.075, 0, 0]}><boxGeometry args={[0.012, 0.03, 0.34]} /></mesh>);
  for (let j = 0; j < 5; j++) bars.push(<mesh key={'h' + j} material={finMat} position={[0.21, 0, -0.15 + j * 0.075]}><boxGeometry args={[0.32, 0.03, 0.012]} /></mesh>);
  return (
    <group>
      {bars}
      <mesh material={finMat} position={[0.025, 0, 0]}><boxGeometry args={[0.05, 0.05, 0.12]} /></mesh>
    </group>
  );
}

/** Reusable first stage. Children refs let the scene animate fins, legs and engine glow. */
export const Booster = forwardRef(function Booster({ finRefs, legRefs, glowRef, paintRef }, ref) {
  const paint = useSootedPaint();
  if (paintRef) paintRef.current = paint;
  const bell = useMemo(() => bellGeometry(0.035, 0.085, 0.24), []);
  const glowMat = useMemo(() => new THREE.MeshBasicMaterial({ color: new THREE.Color(4, 1.6, 0.5), toneMapped: false, transparent: true, opacity: 0 }), []);
  if (glowRef) glowRef.current = glowMat;
  const engines = useMemo(() => {
    const out = [[0, 0]];
    for (let i = 0; i < 8; i++) out.push([Math.cos(i / 8 * Math.PI * 2) * R * 0.66, Math.sin(i / 8 * Math.PI * 2) * R * 0.66]);
    return out;
  }, []);
  return (
    <group ref={ref}>
      <mesh material={paint} position={[0, S1_LEN / 2 + 0.25, 0]} castShadow><cylinderGeometry args={[R, R, S1_LEN, 56, 24]} /></mesh>
      {/* panel seams */}
      {[1.6, 3.4, 5.2, 6.9].map((y) => <mesh key={y} material={black} position={[0, y, 0]}><torusGeometry args={[R * 1.002, 0.004, 4, 56]} /></mesh>)}
      <mesh material={black} position={[0, S1_LEN + 0.25 - 0.4, 0]}><cylinderGeometry args={[R * 1.012, R * 1.012, 0.8, 56]} /></mesh>
      <mesh material={dark} position={[0, 0.14, 0]}><cylinderGeometry args={[R, R * 0.94, 0.28, 56]} /></mesh>
      <mesh material={black} position={[0, 0.0, 0]} rotation={[Math.PI, 0, 0]}><circleGeometry args={[R * 0.94, 40]} /></mesh>
      {engines.map(([x, z], i) => (
        <group key={i} position={[x, 0.0, z]}>
          <mesh geometry={bell} material={dark} />
          <mesh material={glowMat} position={[0, -0.235, 0]} rotation={[Math.PI / 2, 0, 0]}><circleGeometry args={[0.08, 20]} /></mesh>
        </group>
      ))}
      {[0, 1, 2, 3].map((i) => {
        const a = i * Math.PI / 2 + Math.PI / 4;
        return (
          <group key={'f' + i} position={[Math.cos(a) * R, S1_LEN - 0.35, Math.sin(a) * R]} rotation={[0, -a, 0]}>
            <group ref={(el) => { if (finRefs) finRefs.current[i] = el; }}><GridFin /></group>
          </group>
        );
      })}
      {[0, 1, 2, 3].map((i) => {
        const a = i * Math.PI / 2;
        return (
          <group key={'l' + i} position={[Math.cos(a) * R, 0.35, Math.sin(a) * R]} rotation={[0, -a, 0]}>
            <group ref={(el) => { if (legRefs) legRefs.current[i] = el; }}>
              <mesh material={black} position={[0.035, 1.35, 0]}><boxGeometry args={[0.07, 2.7, 0.18]} /></mesh>
              <mesh material={dark} position={[0.04, 0.02, 0]}><boxGeometry args={[0.12, 0.08, 0.3]} /></mesh>
            </group>
          </group>
        );
      })}
    </group>
  );
});

/** Second stage with payload fairing and vacuum engine. */
export const UpperStage = forwardRef(function UpperStage({ glowRef }, ref) {
  const fairing = useMemo(() => {
    const pts = [];
    for (let i = 0; i <= 32; i++) {
      const t = i / 32;
      const r = R * 1.42 * Math.sqrt(Math.max(1 - Math.pow(Math.max(t - 0.25, 0) / 0.75, 2.2), 0));
      pts.push(new THREE.Vector2(Math.max(r, 0.001), t * 3.1));
    }
    return new THREE.LatheGeometry(pts, 56);
  }, []);
  const white = useMemo(() => new THREE.MeshPhysicalMaterial({ color: '#f2f5fa', roughness: 0.32, metalness: 0.08, clearcoat: 0.5 }), []);
  const mvac = useMemo(() => bellGeometry(0.06, 0.34, 0.75, 40), []);
  const niobium = useMemo(() => new THREE.MeshStandardMaterial({ color: '#3a3328', roughness: 0.3, metalness: 0.95, side: THREE.DoubleSide }), []);
  const glowMat = useMemo(() => new THREE.MeshBasicMaterial({ color: new THREE.Color(1.2, 2.2, 4.5), toneMapped: false, transparent: true, opacity: 0, side: THREE.DoubleSide }), []);
  if (glowRef) glowRef.current = glowMat;
  return (
    <group ref={ref}>
      <mesh material={white} position={[0, 1.55, 0]}><cylinderGeometry args={[R, R, 2.7, 56]} /></mesh>
      <mesh material={black} position={[0, 2.95, 0]}><cylinderGeometry args={[R * 1.01, R * 1.42, 0.12, 56]} /></mesh>
      <mesh geometry={fairing} material={white} position={[0, 2.95, 0]} />
      <mesh geometry={mvac} material={niobium} position={[0, 0.2, 0]} />
      <mesh material={glowMat} position={[0, -0.52, 0]} rotation={[Math.PI / 2, 0, 0]}><circleGeometry args={[0.32, 32]} /></mesh>
    </group>
  );
});
