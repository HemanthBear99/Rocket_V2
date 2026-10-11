import * as THREE from 'three';
import { lerp } from '@/lib/mission';

const VERT = `
attribute float size; attribute float alpha; attribute vec3 color;
varying vec3 vC; varying float vA; uniform float scale;
void main(){
  vC = color; vA = alpha;
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  gl_PointSize = min(size * scale / max(-mv.z, 0.1), 512.0);
  gl_Position = projectionMatrix * mv;
}`;
const FRAG = `
varying vec3 vC; varying float vA;
void main(){
  vec2 d = gl_PointCoord - 0.5;
  float r = length(d) * 2.0;
  float a = smoothstep(1.0, 0.0, r);
  a *= a;
  if (a * vA < 0.003) discard;
  gl_FragColor = vec4(vC, a * vA);
}`;

/**
 * Pooled particle system. kind: 0 kerosene flame, 1 vacuum (blue) flame,
 * 2 cold-gas puff; smoke systems use a normal blend and grey colors.
 */
export class Particles {
  constructor(count, { additive = true, smoke = false } = {}) {
    this.n = count; this.smoke = smoke; this.next = 0;
    this.P = Array.from({ length: count }, () => ({ life: 1, max: 0, x: 0, y: -1e5, z: 0, vx: 0, vy: 0, vz: 0, s0: 1, s1: 1, kind: 0 }));
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(count * 3), 3));
    geo.setAttribute('color', new THREE.BufferAttribute(new Float32Array(count * 3), 3));
    geo.setAttribute('size', new THREE.BufferAttribute(new Float32Array(count), 1));
    geo.setAttribute('alpha', new THREE.BufferAttribute(new Float32Array(count), 1));
    this.geo = geo;
    this.material = new THREE.ShaderMaterial({
      uniforms: { scale: { value: 520 } }, vertexShader: VERT, fragmentShader: FRAG,
      transparent: true, depthWrite: false, toneMapped: false,
      blending: additive ? THREE.AdditiveBlending : THREE.NormalBlending,
    });
    this.points = new THREE.Points(geo, this.material);
    this.points.frustumCulled = false;
  }

  emit(count, origin, dir, speed, spread, life, s0, s1, kind = 0) {
    for (let i = 0; i < count; i++) {
      const q = this.P[this.next]; this.next = (this.next + 1) % this.n;
      q.x = origin.x + (Math.random() - 0.5) * spread * 0.25;
      q.y = origin.y + (Math.random() - 0.5) * spread * 0.1;
      q.z = origin.z + (Math.random() - 0.5) * spread * 0.25;
      const sp = speed * (0.65 + Math.random() * 0.7);
      q.vx = dir.x * sp + (Math.random() - 0.5) * spread;
      q.vy = dir.y * sp + (Math.random() - 0.5) * spread;
      q.vz = dir.z * sp + (Math.random() - 0.5) * spread;
      q.life = 0; q.max = life * (0.6 + Math.random() * 0.8); q.s0 = s0; q.s1 = s1; q.kind = kind;
    }
  }

  update(dt) {
    const pos = this.geo.attributes.position.array, col = this.geo.attributes.color.array;
    const size = this.geo.attributes.size.array, al = this.geo.attributes.alpha.array;
    for (let i = 0; i < this.n; i++) {
      const q = this.P[i];
      if (q.life >= q.max) { al[i] = 0; continue; }
      q.life += dt; const k = Math.min(q.life / q.max, 1);
      q.x += q.vx * dt; q.y += q.vy * dt; q.z += q.vz * dt;
      if (q.y < 0.25 && q.vy < 0) {
        // Flame and smoke hitting the pad spread out sideways.
        const s = Math.abs(q.vy); const a = Math.random() * Math.PI * 2;
        q.vx += Math.cos(a) * s * 0.7; q.vz += Math.sin(a) * s * 0.7; q.vy = s * 0.04; q.y = 0.25;
      }
      const drag = this.smoke ? 0.965 : 0.984;
      q.vx *= drag; q.vz *= drag; q.vy = q.vy * drag + (this.smoke ? 0.9 * dt : 0);
      pos[i * 3] = q.x; pos[i * 3 + 1] = q.y; pos[i * 3 + 2] = q.z;
      size[i] = lerp(q.s0, q.s1, Math.sqrt(k));
      if (this.smoke) {
        const g = lerp(0.62, 0.3, k);
        col[i * 3] = g; col[i * 3 + 1] = g * 1.02; col[i * 3 + 2] = g * 1.1;
        al[i] = (1 - k) * Math.min(k * 5, 1) * 0.38;
      } else if (q.kind === 1) {
        col[i * 3] = lerp(2.2, 0.25, k); col[i * 3 + 1] = lerp(2.8, 0.6, k); col[i * 3 + 2] = lerp(4.0, 1.6, k);
        al[i] = (1 - k) * (k < 0.06 ? k / 0.06 : 1) * 0.55;
      } else if (q.kind === 2) {
        col[i * 3] = 1.4; col[i * 3 + 1] = 1.5; col[i * 3 + 2] = 1.6;
        al[i] = (1 - k) * 0.45;
      } else {
        const h = Math.min(k * 1.7, 1);
        col[i * 3] = lerp(5.0, 1.6, h); col[i * 3 + 1] = lerp(3.6, 0.42, h); col[i * 3 + 2] = lerp(2.2, 0.08, Math.min(k * 2.4, 1));
        al[i] = (1 - k) * (k < 0.05 ? k / 0.05 : 1) * 0.7;
      }
    }
    this.geo.attributes.position.needsUpdate = true; this.geo.attributes.color.needsUpdate = true;
    this.geo.attributes.size.needsUpdate = true; this.geo.attributes.alpha.needsUpdate = true;
  }

  dispose() { this.geo.dispose(); this.material.dispose(); }
}
