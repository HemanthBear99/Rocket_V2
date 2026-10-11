'use client';
import { forwardRef, useMemo } from 'react';
import * as THREE from 'three';
import { LZ } from '@/lib/mission';

function padTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 512;
  const g = c.getContext('2d');
  g.fillStyle = '#121822'; g.beginPath(); g.arc(256, 256, 254, 0, Math.PI * 2); g.fill();
  g.strokeStyle = '#d9a23f'; g.lineWidth = 12; g.beginPath(); g.arc(256, 256, 206, 0, Math.PI * 2); g.stroke();
  g.lineWidth = 30; g.beginPath(); g.moveTo(180, 180); g.lineTo(332, 332); g.moveTo(332, 180); g.lineTo(180, 332); g.stroke();
  const t = new THREE.CanvasTexture(c); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 8; return t;
}

const steel = new THREE.MeshStandardMaterial({ color: '#4a5468', roughness: 0.6, metalness: 0.6 });
const concrete = new THREE.MeshStandardMaterial({ color: '#1d2230', roughness: 0.95 });

/** Launch mount, tower with beacon, floodlights and the LZ-1 landing pad. */
const LaunchSite = forwardRef(function LaunchSite({ beaconRef }, ref) {
  const tex = useMemo(() => (typeof document !== 'undefined' ? padTexture() : null), []);
  const lamp = useMemo(() => new THREE.MeshBasicMaterial({ color: new THREE.Color(6, 4.8, 3.2), toneMapped: false }), []);
  const beacon = useMemo(() => new THREE.MeshBasicMaterial({ color: new THREE.Color(8, 0.6, 0.5), toneMapped: false }), []);
  if (beaconRef) beaconRef.current = beacon;
  const levels = Array.from({ length: 15 }, (_, i) => i);
  return (
    <group ref={ref}>
      <mesh material={concrete} position={[0, 0.3, 0]} receiveShadow><cylinderGeometry args={[7, 8, 0.6, 48]} /></mesh>
      <mesh material={concrete} position={[0, 0.02, 0]} rotation={[-Math.PI / 2, 0, 0]}><circleGeometry args={[30, 64]} /></mesh>
      <group position={[-3.4, 0, 0]}>
        {levels.map((i) => <mesh key={i} material={steel} position={[0, 0.7 + i * 1.22, 0]}><boxGeometry args={[2.0, 0.1, 2.0]} /></mesh>)}
        {[[-0.95, -0.95], [0.95, -0.95], [-0.95, 0.95], [0.95, 0.95]].map(([x, z]) => (
          <mesh key={x + ',' + z} material={steel} position={[x, 9.6, z]}><boxGeometry args={[0.1, 19, 0.1]} /></mesh>
        ))}
        {levels.slice(0, 14).map((i) => (
          <mesh key={'x' + i} material={steel} position={[0.95, 1.3 + i * 1.22, 0]} rotation={[i % 2 ? 0.55 : -0.55, 0, 0]}><boxGeometry args={[0.05, 0.05, 2.2]} /></mesh>
        ))}
        <mesh material={steel} position={[1.4, 12.5, 0]}><boxGeometry args={[1.6, 0.18, 0.4]} /></mesh>
        <mesh material={beacon} position={[0, 19.4, 0]}><sphereGeometry args={[0.16, 12, 12]} /></mesh>
      </group>
      {[[14, 9], [-12, 12], [10, -14]].map(([x, z]) => (
        <group key={x} position={[x, 0, z]}>
          <mesh material={steel} position={[0, 4, 0]}><cylinderGeometry args={[0.08, 0.1, 8, 8]} /></mesh>
          <mesh material={lamp} position={[0, 8.1, 0]}><boxGeometry args={[0.9, 0.4, 0.2]} /></mesh>
        </group>
      ))}
      <pointLight position={[12, 9, 12]} color="#ffd6a8" intensity={60} distance={70} decay={2} />
      <pointLight position={[-10, 9, 12]} color="#ffd6a8" intensity={40} distance={60} decay={2} />
      {tex && (
        <group position={[LZ.x, 0.06, LZ.z]}>
          <mesh rotation={[-Math.PI / 2, 0, 0]}><circleGeometry args={[9, 72]} /><meshStandardMaterial map={tex} roughness={0.9} /></mesh>
          {Array.from({ length: 16 }, (_, i) => {
            const a = i / 16 * Math.PI * 2;
            return <mesh key={i} material={lamp} position={[Math.cos(a) * 9.4, 0.1, Math.sin(a) * 9.4]}><sphereGeometry args={[0.09, 8, 8]} /></mesh>;
          })}
          <pointLight position={[0, 6, 0]} color="#ffcf9a" intensity={30} distance={40} decay={2} />
        </group>
      )}
    </group>
  );
});

export default LaunchSite;
