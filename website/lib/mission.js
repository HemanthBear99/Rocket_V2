import * as THREE from 'three';

// Shared scroll state: written by ScrollTrigger, read every frame by the
// 3D scene and the HUD (no React re-renders on scroll).
export const flight = { p: 0, thr: 0 };

export const clamp01 = (x) => (x < 0 ? 0 : x > 1 ? 1 : x);
export const smooth = (a, b, x) => { const t = clamp01((x - a) / (b - a)); return t * t * (3 - 2 * t); };
export const lerp = (a, b, t) => a + (b - a) * t;

export function interp(table, x) {
  if (x <= table[0][0]) return table[0][1];
  for (let i = 1; i < table.length; i++) {
    if (x <= table[i][0]) {
      const [x0, y0] = table[i - 1]; const [x1, y1] = table[i];
      return lerp(y0, y1, (x - x0) / (x1 - x0));
    }
  }
  return table[table.length - 1][1];
}

// Scroll progress -> mission time (s), keyed to the default simulated mission.
export const P2T = [[0, 0], [0.10, 0], [0.36, 130], [0.42, 157], [0.52, 194], [0.62, 341], [0.72, 372], [0.86, 456], [0.95, 505], [1, 505]];
// Mission time -> booster speed (m/s) and altitude (km), from the simulated mission.
export const T2V = [[0, 0], [63, 420], [130, 1652], [157, 1404], [194, 430], [260, 380], [341, 1192], [372, 760], [456, 114], [505, 4.4]];
export const T2A = [[0, 0], [130, 74], [157, 108], [194, 135], [260, 140], [341, 70], [372, 40], [456, 6.2], [505, 0]];
export const PHASES = [[0.10, 'ON THE PAD'], [0.36, 'ASCENT'], [0.42, 'SEPARATION · FLIP'], [0.52, 'BOOSTBACK BURN'], [0.62, 'COAST TO APOGEE'], [0.72, 'ENTRY BURN'], [0.86, 'AERO DESCENT'], [0.95, 'LANDING BURN'], [9, 'TOUCHDOWN · LZ-1']];

// Scene units: 1 unit = 5 m for the vehicle; altitude and downrange are
// exaggerated (6 units per km) so the whole flight fits on screen.
export const LZ = new THREE.Vector3(-70, 0, 26);
const SEP = new THREE.Vector3(450, 444.6, 0);
const RETURN_PATH = new THREE.CatmullRomCurve3([
  SEP.clone(), new THREE.Vector3(540, 610, -10), new THREE.Vector3(470, 740, -20), new THREE.Vector3(300, 770, -10),
  new THREE.Vector3(140, 580, 10), new THREE.Vector3(20, 280, 22), new THREE.Vector3(-55, 70, 26), LZ.clone(),
], false, 'centripetal');

const _v = new THREE.Vector3();

/** Vehicle state for scroll progress p in [0, 1]. */
export function pose(p, out) {
  const s = out;
  s.thr = 0; s.engines = 0; s.s2Attached = true; s.s2thr = 0; s.fins = 0; s.legs = 0; s.plasma = 0; s.rcs = false; s.steam = 0;
  if (p < 0.10) {
    s.pos.set(0, 0.6, 0); s.tilt = 0;
    s.steam = 1 - smooth(0.07, 0.1, p);
    s.thr = smooth(0.075, 0.10, p); s.engines = 9;
  } else if (p < 0.36) {
    const u = (p - 0.10) / 0.26;
    const altKm = 74 * Math.pow(u, 1.9), downKm = 75 * Math.pow(u, 2.6);
    s.pos.set(downKm * 6, 0.6 + altKm * 6, 0);
    s.tilt = -0.98 * Math.pow(u, 1.25);
    s.thr = u > 0.38 && u < 0.6 ? 0.78 : 1; s.engines = 9;
  } else {
    let u;
    if (p < 0.42) u = 0.0;
    else if (p < 0.52) u = lerp(0.0, 0.22, (p - 0.42) / 0.10);
    else if (p < 0.62) u = lerp(0.22, 0.56, smooth(0.52, 0.62, p));
    else if (p < 0.72) u = lerp(0.56, 0.68, (p - 0.62) / 0.10);
    else if (p < 0.86) u = lerp(0.68, 0.90, (p - 0.72) / 0.14);
    else if (p < 0.95) u = lerp(0.90, 1.0, 1 - Math.pow(1 - (p - 0.86) / 0.09, 1.7));
    else u = 1;
    RETURN_PATH.getPointAt(Math.min(u, 1), s.pos);
    if (p < 0.42) s.pos.lerp(_v.copy(SEP).add({ x: (p - 0.36) * 400, y: (p - 0.36) * 300, z: 0 }), 1);
    const flip = smooth(0.365, 0.415, p), upright = smooth(0.52, 0.60, p);
    s.tilt = lerp(lerp(-0.98, 1.35, flip), 0.18, upright) * (1 - smooth(0.80, 0.90, p));
    s.rcs = (p > 0.365 && p < 0.415) || (p > 0.52 && p < 0.58);
    if (p >= 0.42 && p < 0.52) { s.thr = smooth(0.42, 0.43, p); s.engines = 3; }
    if (p >= 0.62 && p < 0.72) { s.thr = smooth(0.62, 0.63, p) * (1 - smooth(0.71, 0.72, p)); s.engines = 3; }
    if (p >= 0.86 && p < 0.95) { s.thr = smooth(0.86, 0.87, p) * lerp(1, 0.55, (p - 0.86) / 0.09); s.engines = 1; }
    s.fins = smooth(0.54, 0.58, p);
    s.legs = smooth(0.905, 0.93, p);
    s.plasma = smooth(0.60, 0.64, p) * (1 - smooth(0.72, 0.80, p));
    if (p >= 0.95) s.pos.y = 0.05;
    const ds = p - 0.36;
    s.s2Attached = false;
    s.s2pos.set(450 + ds * 3600 + ds * ds * 9000, 444.6 + ds * 900 + ds * ds * 1500 + 9 * (1 - smooth(0.36, 0.38, p)), -ds * 300);
    s.s2tilt = -1.15 - ds * 0.6;
    s.s2thr = smooth(0.375, 0.39, p);
  }
  return s;
}

export function makePose() {
  return { pos: new THREE.Vector3(), tilt: 0, s2pos: new THREE.Vector3(), s2tilt: 0, thr: 0, engines: 0, s2Attached: true, s2thr: 0, fins: 0, legs: 0, plasma: 0, rcs: false, steam: 0 };
}
