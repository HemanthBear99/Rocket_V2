'use client';
import { useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { useTexture } from '@react-three/drei';
import * as THREE from 'three';

export const EARTH_R = 4000;

// Unit direction of a lat/lon on three's SphereGeometry with an equirectangular map.
function dirFor(latDeg, lonDeg) {
  const phi = THREE.MathUtils.degToRad(lonDeg + 180);
  const theta = THREE.MathUtils.degToRad(90 - latDeg);
  return new THREE.Vector3(-Math.cos(phi) * Math.sin(theta), Math.cos(theta), Math.sin(phi) * Math.sin(theta));
}

const EARTH_VERT = `
varying vec2 vUv; varying vec3 vN; varying vec3 vView;
void main(){
  vUv = uv;
  vec4 wp = modelMatrix * vec4(position, 1.0);
  vN = normalize(mat3(modelMatrix) * normal);
  vView = normalize(cameraPosition - wp.xyz);
  gl_Position = projectionMatrix * viewMatrix * wp;
}`;
const EARTH_FRAG = `
uniform sampler2D day; uniform sampler2D night; uniform vec3 sunDir; uniform float uHigh; uniform float uLights;
varying vec2 vUv; varying vec3 vN; varying vec3 vView;
void main(){
  vec3 d = texture2D(day, vUv).rgb;
  vec3 n = texture2D(night, vUv).rgb;
  float sun = dot(normalize(vN), normalize(sunDir));
  float dayK = smoothstep(-0.15, 0.35, sun);
  vec3 lights = pow(n, vec3(1.6)) * vec3(2.6, 1.9, 1.2) * (1.0 - dayK) * uLights;
  vec3 col = d * (0.06 + 0.9 * dayK) + lights;
  float rim = pow(1.0 - max(dot(normalize(vN), vView), 0.0), 3.0);
  col += vec3(0.18, 0.42, 0.95) * rim * 0.9 * uHigh;
  gl_FragColor = vec4(col, 1.0);
}`;
// The camera starts inside this shell, so the glow fades in only once it is high
// enough to see the limb; from the ground it would wash out the night sky.
const ATMO_FRAG = `
uniform float uHigh;
varying vec3 vN; varying vec3 vView;
void main(){
  float i = pow(max(0.8 - dot(normalize(vN), vView), 0.0), 4.2) * uHigh;
  gl_FragColor = vec4(vec3(0.32, 0.62, 1.0) * i * 1.35, i);
}`;

export default function Earth() {
  const [day, night] = useTexture(['/textures/earth_day.jpg', '/textures/earth_night.jpg']);
  day.colorSpace = THREE.SRGBColorSpace; night.colorSpace = THREE.SRGBColorSpace;
  day.anisotropy = 8; night.anisotropy = 8;

  // Rotate the globe so Cape Canaveral (28.6 N, 80.6 W) sits at the top, under the pad.
  const quat = useMemo(() => new THREE.Quaternion().setFromUnitVectors(dirFor(28.6, -80.6), new THREE.Vector3(0, 1, 0)), []);
  const earthMat = useMemo(() => new THREE.ShaderMaterial({
    uniforms: { uHigh: { value: 0 }, uLights: { value: 0 }, day: { value: day }, night: { value: night }, sunDir: { value: new THREE.Vector3(0.9, -0.25, -0.35).normalize() } },
    vertexShader: EARTH_VERT, fragmentShader: EARTH_FRAG,
  }), [day, night]);
  const atmoMat = useMemo(() => new THREE.ShaderMaterial({
    uniforms: { uHigh: { value: 0 } },
    vertexShader: EARTH_VERT, fragmentShader: ATMO_FRAG,
    transparent: true, blending: THREE.AdditiveBlending, side: THREE.BackSide, depthWrite: false, toneMapped: false,
  }), []);

  useFrame(({ camera }) => {
    const k = THREE.MathUtils.smoothstep(camera.position.y, 70, 320);
    earthMat.uniforms.uHigh.value = k; atmoMat.uniforms.uHigh.value = k;
    // City lights read as distant points only from altitude; at ground level the
    // texture's pixels would smear into big orange blobs.
    earthMat.uniforms.uLights.value = THREE.MathUtils.smoothstep(camera.position.y, 25, 140);
  });

  return (
    <group position={[0, -EARTH_R, 0]}>
      <mesh material={earthMat} quaternion={quat}><sphereGeometry args={[EARTH_R, 384, 192]} /></mesh>
      <mesh material={atmoMat}><sphereGeometry args={[EARTH_R * 1.04, 384, 192]} /></mesh>
    </group>
  );
}
