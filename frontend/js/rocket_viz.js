/* Realistic 3D rocket flight visualization, built on Three.js.
 * Two synchronized scenes: Orbiter/Ascent stack, and Booster Recovery.
 * Driven by live telemetry payloads; keeps its own render loop so flame,
 * smoke, and camera motion stay smooth between telemetry ticks.
 *
 * Public API is unchanged from the previous 2D-canvas implementation so
 * app.js needs no changes: `new RocketScene(canvasOrbiter, canvasBooster)`,
 * `.update(payload)`, `.onMissionComplete(outcomes)`, `.reset()`, `.stop()`.
 */
(() => {
  'use strict';
  const T = window.THREE;
  if (!T) { console.error('rocket_viz: THREE.js not loaded'); return; }

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const lerp = (a, b, t) => a + (b - a) * t;
  const deg2rad = (d) => (d * Math.PI) / 180;
  const easeOutCubic = (t) => 1 - Math.pow(1 - t, 3);

  function altitudeFrac(altM, refM) {
    if (altM <= 0) return 0;
    const f = Math.log10(1 + altM / 40) / Math.log10(1 + refM / 40);
    return clamp(f, 0, 1);
  }

  // ------------------------------------------------------------- textures
  function softDiscTexture(hexColor) {
    const size = 64;
    const c = document.createElement('canvas');
    c.width = c.height = size;
    const ctx = c.getContext('2d');
    const g = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
    g.addColorStop(0, hexColor.replace('ALPHA', '0.95'));
    g.addColorStop(0.4, hexColor.replace('ALPHA', '0.55'));
    g.addColorStop(1, hexColor.replace('ALPHA', '0'));
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, size, size);
    const tex = new T.CanvasTexture(c);
    tex.needsUpdate = true;
    return tex;
  }

  function starFieldTexture() {
    return softDiscTexture('rgba(255,255,255,ALPHA)');
  }

  // A procedurally shaded Earth: banded ocean/continent gradient plus soft
  // cloud speckle and polar caps, rendered onto an equirectangular canvas.
  // Deliberately continuous-gradient / noise-based (no flat cartoon fills)
  // so it reads as a rendered planet rather than a clip-art globe.
  function earthTexture() {
    const w = 1024, h = 512;
    const c = document.createElement('canvas');
    c.width = w; c.height = h;
    const ctx = c.getContext('2d');
    const ocean = ctx.createLinearGradient(0, 0, 0, h);
    ocean.addColorStop(0, '#0a2a4a');
    ocean.addColorStop(0.5, '#0e3f66');
    ocean.addColorStop(1, '#081d33');
    ctx.fillStyle = ocean;
    ctx.fillRect(0, 0, w, h);

    // land masses: soft irregular blobs via layered radial gradients
    let seed = 1337;
    const rand = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
    for (let i = 0; i < 46; i++) {
      const x = rand() * w, y = h * 0.18 + rand() * h * 0.64;
      const r = 30 + rand() * 90;
      const g = ctx.createRadialGradient(x, y, 0, x, y, r);
      const tone = 40 + rand() * 40;
      g.addColorStop(0, `rgba(${tone + 40},${tone + 60},${tone + 20},0.55)`);
      g.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = g;
      ctx.beginPath(); ctx.ellipse(x, y, r, r * 0.6, rand() * Math.PI, 0, Math.PI * 2); ctx.fill();
    }
    // cloud speckle
    ctx.globalAlpha = 0.12;
    for (let i = 0; i < 900; i++) {
      const x = rand() * w, y = rand() * h;
      ctx.fillStyle = '#ffffff';
      ctx.beginPath(); ctx.arc(x, y, 1 + rand() * 2.2, 0, Math.PI * 2); ctx.fill();
    }
    ctx.globalAlpha = 1;
    // polar caps
    const capTop = ctx.createLinearGradient(0, 0, 0, h * 0.12);
    capTop.addColorStop(0, 'rgba(235,242,250,0.85)');
    capTop.addColorStop(1, 'rgba(235,242,250,0)');
    ctx.fillStyle = capTop; ctx.fillRect(0, 0, w, h * 0.12);
    const capBot = ctx.createLinearGradient(0, h * 0.88, 0, h);
    capBot.addColorStop(0, 'rgba(235,242,250,0)');
    capBot.addColorStop(1, 'rgba(235,242,250,0.85)');
    ctx.fillStyle = capBot; ctx.fillRect(0, h * 0.88, w, h * 0.12);

    const tex = new T.CanvasTexture(c);
    tex.needsUpdate = true;
    tex.wrapS = T.RepeatWrapping;
    return tex;
  }

  // ------------------------------------------------------------- particles
  class ParticleSystem {
    constructor(scene, texture, size, blending) {
      this.max = 260;
      this.positions = new Float32Array(this.max * 3);
      this.alphas = new Float32Array(this.max);
      this.sizes = new Float32Array(this.max);
      this.data = []; // {vx,vy,vz,life,maxLife,gravity,growth}
      this.geo = new T.BufferGeometry();
      this.geo.setAttribute('position', new T.BufferAttribute(this.positions, 3));
      this.mat = new T.PointsMaterial({
        map: texture, size, transparent: true, depthWrite: false,
        blending: blending || T.NormalBlending, sizeAttenuation: true, color: 0xffffff,
      });
      this.points = new T.Points(this.geo, this.mat);
      this.points.frustumCulled = false;
      scene.add(this.points);
    }
    spawn(p) {
      if (this.data.length >= this.max) { this.data.shift(); }
      this.data.push(p);
    }
    step(dt) {
      for (const p of this.data) {
        p.x += p.vx * dt; p.y += p.vy * dt; p.z += (p.vz || 0) * dt;
        p.vy += (p.gravity || 0) * dt;
        p.life -= dt;
        p.r += (p.growth || 0) * dt;
      }
      this.data = this.data.filter((p) => p.life > 0);
      const n = Math.min(this.data.length, this.max);
      for (let i = 0; i < n; i++) {
        const p = this.data[i];
        this.positions[i * 3] = p.x; this.positions[i * 3 + 1] = p.y; this.positions[i * 3 + 2] = p.z;
      }
      this.geo.setDrawRange(0, n);
      this.geo.attributes.position.needsUpdate = true;
      // Approximate per-particle fade/size by averaging into material scalar
      // (keeps the system cheap — good enough for smoke/spark billboards).
      if (n > 0) {
        const avgLife = this.data.reduce((s, p) => s + clamp(p.life / p.maxLife, 0, 1), 0) / n;
        const avgR = this.data.reduce((s, p) => s + p.r, 0) / n;
        this.mat.opacity = clamp(avgLife * 0.9, 0, 0.9);
        this.mat.size = clamp(avgR * 2, 1, 40);
      }
    }
    clear() { this.data = []; this.geo.setDrawRange(0, 0); }
  }

  // ------------------------------------------------------------- rocket build
  const MAT = {
    hull: () => new T.MeshStandardMaterial({ color: 0xe6ebf3, metalness: 0.55, roughness: 0.38 }),
    hullDark: () => new T.MeshStandardMaterial({ color: 0xc7cedb, metalness: 0.6, roughness: 0.42 }),
    engine: () => new T.MeshStandardMaterial({ color: 0x484f5c, metalness: 0.8, roughness: 0.45 }),
    scorch: () => new T.MeshStandardMaterial({ color: 0x1b1b1b, metalness: 0.2, roughness: 0.9 }),
    strut: () => new T.MeshStandardMaterial({ color: 0x6b7688, metalness: 0.7, roughness: 0.4 }),
    accent: () => new T.MeshStandardMaterial({ color: 0x38e0ff, emissive: 0x0c3f4a, metalness: 0.3, roughness: 0.5 }),
  };

  function buildEngineCluster(count, radius, bellR, bellLen) {
    const g = new T.Group();
    const positions = count === 1 ? [[0, 0]] : Array.from({ length: count }, (_, i) => {
      const a = (i / count) * Math.PI * 2;
      return [Math.cos(a) * radius, Math.sin(a) * radius];
    });
    positions.forEach(([x, z]) => {
      const bell = new T.Mesh(new T.CylinderGeometry(bellR * 0.55, bellR, bellLen, 12, 1, true), MAT.engine());
      bell.position.set(x, -bellLen / 2, z);
      g.add(bell);
    });
    return g;
  }

  function buildFlame(bellR, count, radius) {
    // Layered additive cones: soft outer plume + bright inner core, plus a
    // point light so the flame actually illuminates the vehicle body.
    const group = new T.Group();
    const positions = count === 1 ? [[0, 0]] : [[-radius, 0], [0, 0], [radius, 0]].slice(0, count);
    const cores = [];
    positions.forEach(([x, z]) => {
      const outer = new T.Mesh(
        new T.ConeGeometry(bellR * 0.95, 1, 14, 1, true),
        new T.MeshBasicMaterial({ color: 0x5fb2ff, transparent: true, opacity: 0.55, blending: T.AdditiveBlending, depthWrite: false }),
      );
      outer.rotation.x = Math.PI;
      const inner = new T.Mesh(
        new T.ConeGeometry(bellR * 0.5, 1, 12, 1, true),
        new T.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.9, blending: T.AdditiveBlending, depthWrite: false }),
      );
      inner.rotation.x = Math.PI;
      const sub = new T.Group();
      sub.position.set(x, 0, z);
      sub.add(outer); sub.add(inner);
      group.add(sub);
      cores.push({ outer, inner, x, z });
    });
    const light = new T.PointLight(0x8fc8ff, 0, 40);
    group.add(light);
    group.userData = { cores, light };
    return group;
  }

  function setFlame(flameGroup, throttle, lenScale, flicker) {
    const on = throttle > 0.02;
    flameGroup.visible = on;
    if (!on) { flameGroup.userData.light.intensity = 0; return; }
    const flick = 1 + 0.10 * Math.sin(flicker * 26) + 0.05 * Math.sin(flicker * 53 + 1);
    const len = lenScale * (2.4 + 3.4 * throttle) * flick;
    flameGroup.userData.cores.forEach(({ outer, inner }) => {
      outer.scale.set(1, len, 1); outer.position.y = -len / 2;
      inner.scale.set(1, len * 0.62, 1); inner.position.y = -len * 0.31;
    });
    flameGroup.userData.light.intensity = 2.2 * throttle * flick;
  }

  function buildGridFin() {
    const geo = new T.BoxGeometry(0.9, 0.9, 0.06);
    const mat = new T.MeshStandardMaterial({ color: 0x59637a, metalness: 0.7, roughness: 0.5, wireframe: false });
    const panel = new T.Mesh(geo, mat);
    // lattice look via a couple of thin cross bars
    const bar = new T.MeshStandardMaterial({ color: 0x2a303c, metalness: 0.6, roughness: 0.6 });
    for (let i = -1; i <= 1; i++) {
      const b1 = new T.Mesh(new T.BoxGeometry(0.9, 0.03, 0.08), bar);
      b1.position.y = i * 0.28;
      panel.add(b1);
    }
    return panel;
  }

  function buildLandingLeg() {
    const g = new T.Group();
    const strut = new T.Mesh(new T.CylinderGeometry(0.05, 0.06, 1, 8), MAT.strut());
    strut.position.y = -0.5;
    g.add(strut);
    const foot = new T.Mesh(new T.BoxGeometry(0.4, 0.05, 0.18), MAT.strut());
    foot.position.y = -1.0;
    g.add(foot);
    return g;
  }

  // Builds a stage body (cylinder + optional nose cone) with engines, grid
  // fins, and landing legs attached as animatable sub-groups.
  function buildStage({ len, radius, nose, engineCount, engineBellR, gridFins, legs, accentBand }) {
    const group = new T.Group();
    const bodyLen = nose ? len * 0.82 : len;
    const body = new T.Mesh(new T.CylinderGeometry(radius, radius, bodyLen, 24), MAT.hull());
    body.position.y = nose ? -len * 0.09 : 0;
    group.add(body);

    if (nose) {
      const cone = new T.Mesh(new T.ConeGeometry(radius, len * 0.22, 24), MAT.hullDark());
      cone.position.y = bodyLen / 2 - len * 0.09 + len * 0.11;
      group.add(cone);
    }
    if (accentBand) {
      const band = new T.Mesh(new T.CylinderGeometry(radius * 1.001, radius * 1.001, len * 0.04, 24), MAT.accent());
      band.position.y = len * 0.28;
      group.add(band);
    }
    // scorched base ring
    const scorch = new T.Mesh(new T.CylinderGeometry(radius * 1.002, radius * 1.002, len * 0.05, 24), MAT.scorch());
    scorch.position.y = -len / 2 + len * 0.02;
    group.add(scorch);

    const engines = buildEngineCluster(engineCount, radius * 0.5, engineBellR, len * 0.1);
    engines.position.y = -len / 2;
    group.add(engines);

    const flame = buildFlame(engineBellR, engineCount, radius * 0.5);
    flame.position.y = -len / 2 - len * 0.05;
    group.add(flame);

    let gridFinGroup = null, legGroup = null;
    if (gridFins) {
      gridFinGroup = new T.Group();
      [-1, 1].forEach((side) => {
        const fin = buildGridFin();
        fin.position.set(side * radius * 1.05, len * 0.28, 0);
        fin.rotation.y = Math.PI / 2;
        gridFinGroup.add(fin);
      });
      group.add(gridFinGroup);
    }
    if (legs) {
      legGroup = new T.Group();
      [0, 1, 2, 3].forEach((i) => {
        const a = (i / 4) * Math.PI * 2 + Math.PI / 4;
        const leg = buildLandingLeg();
        leg.position.set(Math.cos(a) * radius * 0.9, -len / 2, Math.sin(a) * radius * 0.9);
        leg.userData.baseAngle = a;
        legGroup.add(leg);
      });
      group.add(legGroup);
    }

    return { group, body, flame, gridFinGroup, legGroup, len, radius, engineBellR, engineCount };
  }

  function animateGridFins(finGroup, openness) {
    if (!finGroup) return;
    finGroup.children.forEach((fin, i) => {
      const side = i === 0 ? -1 : 1;
      fin.rotation.z = side * lerp(deg2rad(4), deg2rad(75), openness);
    });
  }

  function animateLegs(legGroup, openness) {
    if (!legGroup) return;
    // Legs are stowed flush inside the interstage/skirt on a real vehicle,
    // not visible at all until deployment starts -- previously they were
    // always rendered, just small, which read as "there are no legs"
    // rather than "they're tucked away".
    legGroup.visible = openness > 0.02;
    legGroup.children.forEach((leg) => {
      const a = leg.userData.baseAngle;
      const spread = lerp(0.04, 0.55, openness);
      leg.position.x = Math.cos(a) * (leg.parent ? 1 : 1);
      leg.rotation.z = Math.cos(a) * spread * -1;
      leg.rotation.x = Math.sin(a) * spread;
      leg.scale.setScalar(lerp(0.55, 1, openness));
    });
  }

  // ------------------------------------------------------------- one panel
  class RocketPanel3D {
    constructor(canvas, opts) {
      this.canvas = canvas;
      this.role = opts.role; // 'orbiter' | 'booster'
      this.maxAltRef = opts.maxAltRef || 120000;

      this.renderer = new T.WebGLRenderer({ canvas, antialias: true, alpha: false, powerPreference: 'low-power' });
      this.renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
      this.renderer.outputColorSpace = T.SRGBColorSpace || T.LinearSRGBColorSpace;

      this.scene = new T.Scene();
      this.camera = new T.PerspectiveCamera(42, 1, 0.05, 20000);

      this._buildWorld();
      this._buildVehicle();

      this.target = { altitude: 0, velocity: 0, throttle: 0, pitchDeg: 0, downrange: 0, gridFins: false, legs: false, phase: 'PRELAUNCH', t: 0 };
      this.disp = { altitude: 0, throttle: 0, pitchDeg: 0, downrange: 0 };
      this.separated = false;
      this.sepAnimT = null;
      this.landed = null;
      this.landingFxT = null;
      this.gridFinsAnim = 0;
      this.legsAnim = 0;
      this.touchdownShakeT = null;
      this.flicker = Math.random() * 10;
      this.camShake = 0;
      this._hasTelemetry = false;

      // User-controlled zoom: a multiplier on top of the auto-framing
      // camera distance, so the viewer can scroll in to inspect detail
      // (grid fins, legs, engines) instead of being stuck at whatever
      // distance the auto-follow camera picks for the current altitude.
      // userYaw/userPitch let the viewer drag-orbit around the vehicle at
      // any zoom level instead of being locked to the auto-follow angle --
      // previously there was no rotate control at all, so zooming in just
      // got you a closer view of the same fixed angle, often clipping the
      // vehicle out of frame. Double-click resets all three to automatic.
      this.userZoom = 1.0;
      this.userYaw = 0;
      this.userPitch = 0;
      canvas.addEventListener('wheel', (e) => {
        e.preventDefault();
        const factor = e.deltaY > 0 ? 1.12 : 1 / 1.12;
        this.userZoom = clamp(this.userZoom * factor, 0.12, 5.0);
      }, { passive: false });

      let dragging = false, lastX = 0, lastY = 0;
      canvas.addEventListener('mousedown', (e) => {
        if (e.button !== 0) return;
        dragging = true; lastX = e.clientX; lastY = e.clientY;
        canvas.style.cursor = 'grabbing';
        e.preventDefault();
      });
      window.addEventListener('mousemove', (e) => {
        if (!dragging) return;
        const dx = e.clientX - lastX, dy = e.clientY - lastY;
        lastX = e.clientX; lastY = e.clientY;
        this.userYaw -= dx * 0.008;
        this.userPitch = clamp(this.userPitch - dy * 0.008, -1.3, 1.3);
      });
      window.addEventListener('mouseup', () => {
        if (dragging) { dragging = false; canvas.style.cursor = 'grab'; }
      });
      canvas.addEventListener('dblclick', () => {
        this.userZoom = 1.0; this.userYaw = 0; this.userPitch = 0;
      });
      canvas.style.cursor = 'grab';

      new ResizeObserver(() => this._resize()).observe(canvas);
      this._resize();
    }

    _buildWorld() {
      this.scene.background = new T.Color(0x03050a);
      this.hemi = new T.HemisphereLight(0x9fb4ff, 0x05070c, 0.55);
      this.scene.add(this.hemi);
      this.sun = new T.DirectionalLight(0xffffff, 1.5);
      this.sun.position.set(60, 40, 30);
      this.scene.add(this.sun);
      this.scene.add(new T.AmbientLight(0x223344, 0.35));

      // starfield
      const starCount = 1400;
      const starPos = new Float32Array(starCount * 3);
      for (let i = 0; i < starCount; i++) {
        const r = 3000 + Math.random() * 1000;
        const th = Math.random() * Math.PI * 2, ph = Math.acos(2 * Math.random() - 1);
        starPos[i * 3] = r * Math.sin(ph) * Math.cos(th);
        starPos[i * 3 + 1] = Math.abs(r * Math.cos(ph)) * 0.6 + 200;
        starPos[i * 3 + 2] = r * Math.sin(ph) * Math.sin(th);
      }
      const starGeo = new T.BufferGeometry();
      starGeo.setAttribute('position', new T.BufferAttribute(starPos, 3));
      const starMat = new T.PointsMaterial({ map: starFieldTexture(), size: 3.2, transparent: true, depthWrite: false, sizeAttenuation: false, opacity: 0 });
      this.stars = new T.Points(starGeo, starMat);
      this.stars.frustumCulled = false;
      this.scene.add(this.stars);

      // Earth: only meaningful in the far/high-altitude view
      const earthR = 1400;
      this.earth = new T.Mesh(
        new T.SphereGeometry(earthR, 48, 48),
        new T.MeshStandardMaterial({ map: earthTexture(), roughness: 0.9, metalness: 0.0 }),
      );
      this.earth.position.set(0, -earthR - 30, -300);
      this.earth.visible = false;
      this.scene.add(this.earth);
      // thin atmospheric rim glow
      this.atmo = new T.Mesh(
        new T.SphereGeometry(earthR * 1.01, 48, 48),
        new T.MeshBasicMaterial({ color: 0x4fa8ff, transparent: true, opacity: 0.12, side: T.BackSide }),
      );
      this.atmo.position.copy(this.earth.position);
      this.atmo.visible = false;
      this.scene.add(this.atmo);

      // ground plane + pad (low-altitude view)
      this.ground = new T.Mesh(
        new T.CircleGeometry(60, 48),
        new T.MeshStandardMaterial({ color: this.role === 'booster' ? 0x0a1420 : 0x0c1119, roughness: 0.95, metalness: 0.05 }),
      );
      this.ground.rotation.x = -Math.PI / 2;
      this.scene.add(this.ground);

      const padColor = this.role === 'booster' ? 0x38e0ff : 0x3a4658;
      this.pad = new T.Mesh(
        new T.CylinderGeometry(9, 9, 0.4, 32),
        new T.MeshStandardMaterial({ color: 0x1a222e, roughness: 0.7, metalness: 0.3 }),
      );
      this.scene.add(this.pad);
      const ring = new T.Mesh(
        new T.RingGeometry(8.4, 9, 48),
        new T.MeshBasicMaterial({ color: padColor, side: T.DoubleSide }),
      );
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = 0.21;
      this.scene.add(ring);

      if (this.role === 'orbiter') {
        // launch tower
        this.tower = new T.Group();
        const rail = new T.Mesh(new T.BoxGeometry(0.5, 22, 0.5), MAT.strut());
        rail.position.set(-7, 11, 0);
        this.tower.add(rail);
        for (let i = 0; i < 6; i++) {
          const arm = new T.Mesh(new T.BoxGeometry(2.4, 0.2, 0.2), MAT.strut());
          arm.position.set(-6, 3 + i * 3.5, 0);
          this.tower.add(arm);
        }
        this.scene.add(this.tower);
      }

      // orbit path ring, shown once orbit is achieved (orbiter scene only)
      this.orbitRing = new T.Mesh(
        new T.TorusGeometry(earthR + 190, 1.2, 8, 128),
        new T.MeshBasicMaterial({ color: 0x3ddc84, transparent: true, opacity: 0 }),
      );
      this.orbitRing.rotation.x = Math.PI / 2.3;
      this.orbitRing.position.copy(this.earth.position);
      this.scene.add(this.orbitRing);

      this.smoke = new ParticleSystem(this.scene, softDiscTexture('rgba(220,222,230,ALPHA)'), 3, T.NormalBlending);
      this.spark = new ParticleSystem(this.scene, softDiscTexture('rgba(255,170,80,ALPHA)'), 2, T.AdditiveBlending);
    }

    _buildVehicle() {
      const boosterSpec = {
        len: 14, radius: 1.05, nose: false, engineCount: 5, engineBellR: 0.32, gridFins: true, legs: true,
      };
      const upperSpec = {
        len: 9, radius: 0.95, nose: true, engineCount: 1, engineBellR: 0.4, gridFins: false, legs: false, accentBand: true,
      };
      this.boosterLen = boosterSpec.len;
      this.upperLen = upperSpec.len;
      // buildStage() always places a stage's own base (engines/scorch ring)
      // at local y = -len/2 regardless of nose cone, so two stages stack
      // flush by putting the booster's base at y=0 and the upper stage's
      // base at the booster's top (minus a small interstage overlap so no
      // gap is visible between them).
      // Pad clearance: engines/flame anchor at the group's local base
      // (y = -len/2, minus a further 5% for the flame nesting offset in
      // buildStage/buildFlame). Without this lift the engine bells sit
      // exactly at y=0 -- inside the solid pad mesh (top surface y=0.2) --
      // so they and the flame render invisible, hidden behind opaque
      // geometry, and the whole stack looks like it's missing its base.
      const padClearance = 0.9;
      const interstageOverlap = 0.5;
      this.boosterRestY = boosterSpec.len / 2 + padClearance;
      this.upperRestY = boosterSpec.len - interstageOverlap + upperSpec.len / 2 + padClearance;

      if (this.role === 'booster') {
        this.stack = new T.Group();
        this.booster = buildStage(boosterSpec);
        this.booster.group.position.y = this.boosterRestY;
        this.stack.add(this.booster.group);
        this.scene.add(this.stack);
      } else {
        this.stack = new T.Group();
        this.booster = buildStage(boosterSpec);
        this.upper = buildStage(upperSpec);
        this.booster.group.position.y = this.boosterRestY;
        this.upper.group.position.y = this.upperRestY;
        this.stack.add(this.booster.group);
        this.stack.add(this.upper.group);
        this.scene.add(this.stack);
      }
    }

    reset() {
      this.target = { altitude: 0, velocity: 0, throttle: 0, pitchDeg: 0, downrange: 0, gridFins: false, legs: false, phase: 'PRELAUNCH', t: 0 };
      this.disp = { altitude: 0, throttle: 0, pitchDeg: 0, downrange: 0 };
      this.separated = false;
      this.sepAnimT = null;
      this.landed = null;
      this.landingFxT = null;
      this.gridFinsAnim = 0;
      this.legsAnim = 0;
      this.touchdownShakeT = null;
      this.smoke.clear();
      this.spark.clear();
      if (this.upper) { this.upper.group.visible = true; this.upper.group.position.y = this.upperRestY; this.upper.group.rotation.set(0, 0, 0); this.upper.group.scale.setScalar(1); }
      if (this.booster) { this.booster.group.visible = true; this.booster.group.position.y = this.boosterRestY; this.booster.group.rotation.set(0, 0, 0); this.booster.group.scale.setScalar(1); this._setGroupOpacity(this.booster.group, 1); }
    }

    _setGroupOpacity(group, alpha) {
      group.traverse((o) => {
        if (o.material && o.material.transparent !== undefined) {
          if (!o.material.userData) o.material.userData = {};
          if (o.material.userData._baseTransparent === undefined) o.material.userData._baseTransparent = o.material.transparent;
          o.material.transparent = alpha < 1 ? true : o.material.userData._baseTransparent;
          o.material.opacity = alpha;
        }
      });
    }

    setState(next) {
      this._hasTelemetry = true;
      const wasPhase = this.target.phase;
      Object.assign(this.target, next);
      if (this.role === 'orbiter' && !this.separated) {
        const sepPhases = ['STAGE_SEPARATION', 'S2_COAST_TO_APOGEE', 'ORBIT_INSERTION', 'ORBIT_ACHIEVED', 'ORBIT_FAILED', 'APOGEE_REACHED', 'S2_ORBIT_HOLD', 'S2_DEORBIT', 'S2_ENTRY', 'S2_LANDING'];
        if (sepPhases.includes(next.phase) && wasPhase !== next.phase) {
          this.separated = true;
          this.sepAnimT = 0;
        }
      }
    }

    markLanded(kind) {
      this.landed = kind;
      this.landingFxT = 0;
      this.touchdownShakeT = 0;
      const originY = this.role === 'booster' ? 0.3 : 0.3;
      if (kind === 'success') {
        for (let i = 0; i < 50; i++) this._spawnDust(originY);
      } else {
        for (let i = 0; i < 70; i++) this._spawnExplosion(originY);
      }
    }

    _spawnDust(y) {
      const a = Math.random() * Math.PI * 2;
      const spd = 1.5 + Math.random() * 4;
      this.smoke.spawn({
        x: (Math.random() - 0.5) * 1.5, y, z: (Math.random() - 0.5) * 1.5,
        vx: Math.cos(a) * spd, vy: 1 + Math.random() * 2, vz: Math.sin(a) * spd,
        gravity: -2.5, r: 0.4 + Math.random() * 0.6, growth: 0.8, life: 1.2 + Math.random(), maxLife: 1.8,
      });
    }

    _spawnExplosion(y) {
      const a = Math.random() * Math.PI * 2;
      const el = Math.random() * Math.PI - Math.PI / 2;
      const spd = 3 + Math.random() * 10;
      this.spark.spawn({
        x: 0, y, z: 0,
        vx: Math.cos(a) * Math.cos(el) * spd, vy: Math.sin(el) * spd + 2, vz: Math.sin(a) * Math.cos(el) * spd,
        gravity: -6, r: 0.25 + Math.random() * 0.4, growth: 0.3, life: 0.5 + Math.random() * 0.6, maxLife: 1.0,
      });
    }

    _spawnTrail(anchorY) {
      this.smoke.spawn({
        x: (Math.random() - 0.5) * 0.3, y: anchorY, z: (Math.random() - 0.5) * 0.3,
        vx: (Math.random() - 0.5) * 0.6, vy: -1.2 - Math.random(), vz: (Math.random() - 0.5) * 0.6,
        gravity: 0.15, r: 0.15 + Math.random() * 0.15, growth: 0.35, life: 0.5 + Math.random() * 0.4, maxLife: 0.9,
      });
    }

    _spawnLaunchSmoke() {
      const bx = (Math.random() - 0.5) * 4;
      this.smoke.spawn({
        x: bx, y: 0.1, z: (Math.random() - 0.5) * 4,
        vx: (Math.random() - 0.5) * 1.2, vy: 0.6 + Math.random() * 1.2, vz: (Math.random() - 0.5) * 1.2,
        gravity: 0.1, r: 0.7 + Math.random() * 0.9, growth: 0.7, life: 1.4 + Math.random(), maxLife: 2.2,
      });
    }

    step(dt) {
      const k = 1 - Math.pow(0.001, dt);
      this.disp.altitude = lerp(this.disp.altitude, this.target.altitude, k);
      this.disp.throttle = lerp(this.disp.throttle, this.target.throttle, k * 1.6);
      this.disp.downrange = lerp(this.disp.downrange, this.target.downrange, k);
      let pitchDelta = this.target.pitchDeg - this.disp.pitchDeg;
      if (pitchDelta > 180) pitchDelta -= 360;
      if (pitchDelta < -180) pitchDelta += 360;
      this.disp.pitchDeg += pitchDelta * k;

      if (this.sepAnimT !== null) { this.sepAnimT += dt; if (this.sepAnimT > 2.4) this.sepAnimT = null; }
      if (this.landingFxT !== null) this.landingFxT += dt;
      if (this.touchdownShakeT !== null) { this.touchdownShakeT += dt; if (this.touchdownShakeT > 1.0) this.touchdownShakeT = null; }

      const finTarget = this.target.gridFins ? 1 : 0;
      const legTarget = this.target.legs ? 1 : 0;
      const deployRate = dt / 1.1;
      this.gridFinsAnim += clamp(finTarget - this.gridFinsAnim, -deployRate, deployRate);
      this.legsAnim += clamp(legTarget - this.legsAnim, -deployRate, deployRate);

      this.flicker += dt;
      this.smoke.step(dt);
      this.spark.step(dt);

      const onGround = this.disp.altitude < 40;
      if (onGround && this.disp.throttle > 0.3 && this.role === 'orbiter') {
        if (Math.random() < 0.9) this._spawnLaunchSmoke();
      }

      this._applyVehicleState();
      this._updateCamera(dt);
      this.renderer.render(this.scene, this.camera);
    }

    _applyVehicleState() {
      const frac = altitudeFrac(this.disp.altitude, this.maxAltRef);
      // Previously gated on `frac`, which is scaled against maxAltRef
      // (100km for the booster panel) -- at just 20km that log curve was
      // already past the 0.55 "in space" cutoff, so the full Earth sphere
      // appeared while still well inside the atmosphere. These are real
      // altitude bands in meters instead, so "the whole Earth is visible"
      // only starts near the actual edge of space, not at 20km.
      const alt = this.disp.altitude;
      const inSpace = alt > 50000;
      this.earth.visible = inSpace;
      this.atmo.visible = inSpace;
      this.stars.material.opacity = clamp((alt - 20000) / 40000, 0, 1);
      this.ground.visible = alt < 30000;
      this.pad.visible = alt < 30000;
      if (this.tower) this.tower.visible = alt < 20000;

      const orbitAchieved = ['ORBIT_ACHIEVED', 'S2_ORBIT_HOLD', 'S2_COAST_TO_APOGEE', 'APOGEE_REACHED'].includes(this.target.phase);
      this.orbitRing.material.opacity = lerp(this.orbitRing.material.opacity, orbitAchieved ? 0.55 : 0, 0.05);

      if (this.role === 'orbiter') {
        this._applyOrbiterState(frac);
      } else {
        this._applyBoosterState(frac);
      }
    }

    _applyOrbiterState(frac) {
      const sepProgress = this.sepAnimT !== null ? clamp(this.sepAnimT / 2.2, 0, 1) : (this.separated ? 1 : 0);
      if (this.separated) {
        const drop = easeOutCubic(clamp((this.sepAnimT || 2.4) / 1.6, 0, 1));
        this.booster.group.position.y = this.boosterRestY - drop * this.boosterLen;
        this.booster.group.rotation.x = drop * 0.8;
        this._setGroupOpacity(this.booster.group, clamp(1 - drop * 1.2, 0, 1));
        this.upper.group.position.y = this.upperRestY + drop * 2.2;
        if (drop >= 1 && this.booster.group.visible) this.booster.group.visible = false;
      }
      animateGridFins(this.booster.gridFinGroup, this.gridFinsAnim);
      animateLegs(this.booster.legGroup, this.legsAnim);

      const boosterThrottle = this.separated ? 0 : this.disp.throttle;
      const upperThrottle = this.separated && sepProgress > 0.4 ? this.disp.throttle : 0;
      setFlame(this.booster.flame, boosterThrottle, this.booster.radius, this.flicker);
      setFlame(this.upper.flame, upperThrottle, this.upper.radius, this.flicker);

      if (this.disp.throttle > 0.08) {
        const density = clamp(1 - this.disp.altitude / 20000, 0.08, 1);
        if (Math.random() < 0.5 * density) {
          const anchor = this.separated ? this.upper.group.position.y - this.upper.len / 2 - 1 : -this.booster.len / 2 - 1;
          this._spawnTrail(anchor);
        }
      }
    }

    _applyBoosterState(frac) {
      animateGridFins(this.booster.gridFinGroup, this.gridFinsAnim);
      animateLegs(this.booster.legGroup, this.legsAnim);
      const phase = this.target.phase;
      const burning = ['BOOSTER_BOOSTBACK', 'BOOSTER_ENTRY', 'BOOSTER_LANDING'].includes(phase) && this.disp.throttle > 0.02;
      setFlame(this.booster.flame, burning ? this.disp.throttle : 0, this.booster.radius, this.flicker);
      // relight pattern differs: entry burn uses 3 outer engines (handled via
      // flame group having up to 3 cone positions), boostback/landing use the
      // single center engine — approximate by scaling the outer cores down.
      const outerOn = phase === 'BOOSTER_ENTRY';
      this.booster.flame.userData.cores.forEach((c, i) => {
        const isCenter = c.x === 0 && c.z === 0;
        c.outer.visible = burning && (isCenter || outerOn);
        c.inner.visible = c.outer.visible;
      });

      if (burning && Math.random() < 0.5) this._spawnTrail(-this.booster.len / 2 - 1);

      // re-entry heating glow
      const wantGlow = phase === 'BOOSTER_ENTRY';
      if (!this._glow) {
        this._glow = new T.Mesh(new T.SphereGeometry(this.booster.radius * 1.4, 16, 16), new T.MeshBasicMaterial({ color: 0xff8c46, transparent: true, opacity: 0, blending: T.AdditiveBlending, depthWrite: false }));
        this.stack.add(this._glow);
        this._glow.position.y = this.booster.len * 0.15;
      }
      this._glow.material.opacity = lerp(this._glow.material.opacity, wantGlow ? (0.35 + 0.1 * Math.sin(this.flicker * 8)) : 0, 0.08);
    }

    _updateCamera(dt) {
      const frac = altitudeFrac(this.disp.altitude, this.maxAltRef);
      const ef = easeOutCubic(frac);
      const drift = clamp(this.disp.downrange / 4000, -1, 1) * 6;

      // The rocket actually climbs in world space (rather than staying at
      // y=0 while only the camera pretends to pull back) so the vehicle
      // stays inside the frame the camera is aimed at, and the ground/pad
      // recede below it the way a real chase camera would show.
      const climbY = this.role === 'booster'
        ? Math.min(150, this.disp.altitude / 35)
        : Math.min(260, this.disp.altitude / 55) * ef;
      this.stack.position.set(drift, climbY, 0);
      this.stack.rotation.z = -deg2rad(this.disp.pitchDeg) * 0.35;

      let sx = 0, sy = 0;
      if (this.touchdownShakeT !== null) {
        const decay = Math.max(0, 1 - this.touchdownShakeT / 0.5);
        sx = (Math.random() - 0.5) * 0.4 * decay;
        sy = (Math.random() - 0.5) * 0.3 * decay;
      }

      let camDist, camHeightOffset, lookYOffset;
      if (this.role === 'booster') {
        const idle = !this._hasTelemetry;
        if (idle) {
          camDist = 46; camHeightOffset = 12; lookYOffset = 6;
        } else if (this.disp.altitude > 2000) {
          // Coast/entry at tens of km: the old code capped distance at the
          // same value used for the final 3km approach, so from
          // separation (~90km) all the way down to 2km the camera never
          // pulled back -- it looked "zoomed in and stuck" for most of
          // the descent, and hid the vehicle's actual position relative
          // to Earth's limb the whole time. Scale distance with altitude
          // (same altitudeFrac the orbiter panel already uses) so higher
          // altitude means a wider view, not the same fixed close shot.
          const highFrac = altitudeFrac(this.disp.altitude, this.maxAltRef);
          camDist = lerp(34, 140, highFrac);
          camHeightOffset = lerp(16, 55, highFrac);
          lookYOffset = lerp(8, 25, highFrac);
        } else {
          const closeFrac = clamp(this.disp.altitude / 2000, 0, 1); // 1=far(2km), 0=touchdown
          camDist = lerp(14, 34, closeFrac);
          camHeightOffset = lerp(5, 16, closeFrac);
          lookYOffset = lerp(3, 8, closeFrac);
        }
      } else {
        // Pre-separation the full two-stage stack is ~2.5x taller than the
        // single upper stage left after separation, so it needs a much
        // wider frame or the nose and engines clip out of view (looked
        // like a "wrong shape" -- it was just badly cropped).
        const baseDist = this.separated ? 20 : 34;
        const baseHeight = this.separated ? 4 : 13;
        const baseLook = this.separated ? 3 : 12;
        camDist = lerp(baseDist, 300, ef);
        camHeightOffset = lerp(baseHeight, 90, ef);
        lookYOffset = lerp(baseLook, 3, ef);
      }
      const camX = drift * 0.4 + sx;
      const lookAtX = camX * 0.4;
      const lookAtY = climbY + lookYOffset;
      const lookAtZ = 0;

      // True dolly zoom: scale the offset from the look-at point to the
      // camera, not distance and height independently. Scaling camDist
      // and camHeightOffset separately (the previous approach) changes
      // the viewing angle as you zoom -- at large zoom-out the camera
      // flew up much faster than it pulled back, ending up pointed
      // almost straight down ("taken off into the sky"). Scaling the
      // whole offset vector uniformly keeps the same viewing angle at
      // every zoom level, only the distance changes.
      const offsetX = camX - lookAtX;
      const offsetY = climbY + camHeightOffset + sy - lookAtY;
      const offsetZ = camDist - lookAtZ;

      // Apply the user's drag-orbit (yaw around the world Y axis, then
      // pitch around the resulting local right axis) to the offset vector
      // before scaling by zoom, so the viewer can inspect the vehicle from
      // any angle at any zoom level instead of only the auto-follow angle.
      const cosYaw = Math.cos(this.userYaw), sinYaw = Math.sin(this.userYaw);
      const yx = offsetX * cosYaw + offsetZ * sinYaw;
      const yz = -offsetX * sinYaw + offsetZ * cosYaw;
      const cosPitch = Math.cos(this.userPitch), sinPitch = Math.sin(this.userPitch);
      const ry = offsetY * cosPitch - yz * sinPitch;
      const rz = offsetY * sinPitch + yz * cosPitch;

      this.camera.position.set(
        lookAtX + yx * this.userZoom,
        lookAtY + ry * this.userZoom,
        lookAtZ + rz * this.userZoom,
      );
      this.camera.lookAt(lookAtX, lookAtY, lookAtZ);
    }

    resize() { this._resize(); }
    _resize() {
      const rect = this.canvas.getBoundingClientRect();
      const w = Math.max(1, rect.width || 360), h = Math.max(1, rect.height || 360);
      this.renderer.setSize(w, h, false);
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
    }
  }

  class RocketScene {
    constructor(canvasOrbiter, canvasBooster) {
      this.orbiter = new RocketPanel3D(canvasOrbiter, { role: 'orbiter', maxAltRef: 400000 });
      this.booster = new RocketPanel3D(canvasBooster, { role: 'booster', maxAltRef: 100000 });
      this._last = performance.now();
      this._running = true;
      requestAnimationFrame(this._tick.bind(this));
      this._fallbackTimer = setInterval(() => { if (document.hidden) this._tick(performance.now()); }, 100);
    }

    reset() { this.orbiter.reset(); this.booster.reset(); }

    update(payload) {
      this.orbiter.setState({
        altitude: payload.altitude || 0, velocity: payload.velocity || 0, throttle: payload.throttle || 0,
        pitchDeg: payload.pitch_angle ?? 0, downrange: payload.downrange || 0,
        gridFins: false, legs: false, phase: payload.phase || 'PRELAUNCH', t: payload.time || 0,
      });
      if (payload.booster_altitude !== undefined) {
        this.booster.setState({
          altitude: payload.booster_altitude || 0, velocity: payload.booster_velocity || 0,
          throttle: payload.booster_throttle || 0, pitchDeg: payload.booster_pitch_angle ?? 0,
          downrange: payload.booster_downrange || 0,
          gridFins: payload.booster_grid_fins === 'DEPLOYED', legs: payload.booster_landing_legs === 'DEPLOYED',
          phase: payload.booster_phase || 'BOOSTER_FLIP', t: payload.time || 0,
        });
      }
    }

    onMissionComplete(outcomes) {
      if (!outcomes) return;
      this.orbiter.markLanded(outcomes.orbiter_success ? 'success' : 'failure');
      if (this.booster.target.altitude !== undefined) {
        this.booster.markLanded(outcomes.booster_success ? 'success' : 'failure');
      }
    }

    _tick(now) {
      if (!this._running) return;
      const dt = Math.min(0.05, (now - this._last) / 1000);
      this._last = now;
      this.orbiter.step(dt);
      this.booster.step(dt);
      requestAnimationFrame(this._tick.bind(this));
    }

    stop() { this._running = false; clearInterval(this._fallbackTimer); }
  }

  window.RocketScene = RocketScene;
})();
