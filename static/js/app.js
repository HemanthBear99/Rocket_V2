/* Boostback — dashboard application logic. No external dependencies. */
(() => {
  'use strict';

  // ---------------------------------------------------------------- state
  const state = {
    baseUrl: localStorage.getItem('rlv_base_url') || window.location.origin,
    apiKey: localStorage.getItem('rlv_api_key') || '',
    defaults: null,
    formValues: { vehicle: {}, mission: {}, recovery: {}, physics: {} },
    advancedOverrides: null,
    demoMode: false,
    s2Recovery: false,
    realtime: true,
    sim: { id: null, token: null, status: 'idle', startedAt: null, hasBooster: false },
    activeVehicleTab: 'primary',
    campaign: { id: null, pollHandle: null },
    charts: {},
    clockHandle: null,
  };

  const FIELD_META = {
    vehicle: {
      stage1_dry_mass: { label: 'Stage 1 Dry Mass', unit: 'kg', step: 10 },
      stage1_prop_mass: { label: 'Stage 1 Propellant', unit: 'kg', step: 100 },
      stage2_dry_mass: { label: 'Stage 2 Dry Mass', unit: 'kg', step: 10 },
      stage2_prop_mass: { label: 'Stage 2 Propellant', unit: 'kg', step: 100 },
      payload_mass: { label: 'Payload Mass', unit: 'kg', step: 10 },
      thrust_sl: { label: 'Thrust (Sea Level)', unit: 'N', step: 1000 },
      thrust_vac: { label: 'Thrust (Vacuum)', unit: 'N', step: 1000 },
      isp_sl: { label: 'Specific Impulse (SL)', unit: 's', step: 1 },
      isp_vac: { label: 'Specific Impulse (Vac)', unit: 's', step: 1 },
      max_gimbal_deg: { label: 'Max Gimbal Angle', unit: 'deg', step: 0.5 },
      throttle_min: { label: 'Min Throttle', unit: 'frac', step: 0.01 },
      throttle_max: { label: 'Max Throttle', unit: 'frac', step: 0.01 },
      stage1_engines: { label: 'Stage 1 Engines', unit: 'count', step: 1 },
      boostback_engines: { label: 'Boostback Engines', unit: 'count', step: 1 },
      entry_engines: { label: 'Entry Burn Engines', unit: 'count', step: 1 },
      landing_engines: { label: 'Landing Engines', unit: 'count', step: 1 },
      diameter_m: { label: 'Vehicle Diameter', unit: 'm', step: 0.1 },
    },
    mission: {
      target_alt_km: { label: 'Target Orbit Altitude', unit: 'km', step: 1 },
      target_inclination_deg: { label: 'Target Inclination', unit: 'deg', step: 0.1 },
      orbit_insertion_start_alt_km: { label: 'Insertion Start Altitude', unit: 'km', step: 1 },
      stage_sep_velocity: { label: 'Stage Separation Velocity', unit: 'm/s', step: 10 },
      dt: { label: 'Integration Step', unit: 's', step: 0.01 },
      t_max: { label: 'Max Mission Duration', unit: 's', step: 10 },
    },
    recovery: {
      landing_lat: { label: 'Landing Site Latitude', unit: 'deg', step: 0.001 },
      landing_lon: { label: 'Landing Site Longitude', unit: 'deg', step: 0.001 },
    },
  };

  const BOOLEAN_FIELD_META = {
    recovery: {
      grid_fins: { label: 'Grid Fins', desc: 'Enable aerodynamic grid fin control during descent' },
      landing_legs: { label: 'Landing Legs', desc: 'Deploy landing legs prior to touchdown' },
      gfold_guidance: { label: 'G-FOLD Landing Guidance', desc: 'Convex minimum-fuel powered descent instead of the heuristic suicide burn (needs cvxpy)' },
    },
    physics: {
      j2: { label: 'J2 Perturbation', desc: 'Earth oblateness gravity term' },
      atmosphere: { label: 'Atmosphere Model', desc: 'US-76 / high-fidelity atmosphere' },
      drag: { label: 'Aerodynamic Drag', desc: 'Drag force on the vehicle body' },
      lift: { label: 'Aerodynamic Lift', desc: 'Lift force from angle of attack' },
      sensor_noise: { label: 'Sensor Noise', desc: 'IMU / GPS / altimeter noise models' },
    },
  };

  // ---------------------------------------------------------------- utils
  const $ = (sel, root = document) => root.querySelector(sel);
  const $$ = (sel, root = document) => Array.from(root.querySelectorAll(sel));

  function confirmModal(title, body) {
    return new Promise((resolve) => {
      const overlay = $('#confirm-overlay');
      $('#confirm-title').textContent = title;
      $('#confirm-body').textContent = body;
      overlay.classList.remove('hidden');
      const cleanup = (result) => {
        overlay.classList.add('hidden');
        okBtn.removeEventListener('click', onOk);
        cancelBtn.removeEventListener('click', onCancel);
        overlay.removeEventListener('click', onOverlay);
        resolve(result);
      };
      const okBtn = $('#confirm-ok');
      const cancelBtn = $('#confirm-cancel');
      const onOk = () => cleanup(true);
      const onCancel = () => cleanup(false);
      const onOverlay = (e) => { if (e.target === overlay) cleanup(false); };
      okBtn.addEventListener('click', onOk);
      cancelBtn.addEventListener('click', onCancel);
      overlay.addEventListener('click', onOverlay);
    });
  }

  function toast(msg, kind = 'info') {
    const stack = $('#toast-stack');
    const el = document.createElement('div');
    el.className = `toast ${kind}`;
    el.textContent = msg;
    stack.appendChild(el);
    setTimeout(() => { el.style.opacity = '0'; el.style.transition = 'opacity .3s'; }, 3600);
    setTimeout(() => el.remove(), 4000);
  }

  function escapeHtml(value) {
    return String(value ?? '').replace(/[&<>"']/g, (ch) => (
      { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[ch]
    ));
  }

  function fmt(n, decimals = 1) {
    if (n === null || n === undefined || Number.isNaN(n)) return '—';
    return Number(n).toLocaleString(undefined, { minimumFractionDigits: decimals, maximumFractionDigits: decimals });
  }

  function apiHeaders(extra = {}) {
    const h = { 'Content-Type': 'application/json', ...extra };
    if (state.apiKey) h['X-RLV-API-Key'] = state.apiKey;
    return h;
  }

  async function api(path, opts = {}) {
    const res = await fetch(state.baseUrl + path, {
      ...opts,
      headers: { ...(opts.body && !(opts.body instanceof FormData) ? apiHeaders() : (state.apiKey ? { 'X-RLV-API-Key': state.apiKey } : {})), ...(opts.headers || {}) },
    });
    if (!res.ok) {
      let detail = res.statusText;
      try { const j = await res.json(); detail = j.detail || JSON.stringify(j); } catch (_) {}
      throw new Error(detail);
    }
    return res.json();
  }

  // ---------------------------------------------------------------- nav
  function showView(name) {
    $$('.view').forEach((v) => v.classList.add('hidden'));
    $(`#view-${name}`).classList.remove('hidden');
    $$('.nav-item').forEach((n) => n.classList.toggle('active', n.dataset.view === name));
    const titles = {
      config: ['Mission Setup', 'Configure the vehicle, mission profile, and physics model before launch.'],
      telemetry: ['Live Telemetry', 'Real-time vehicle state streamed from the running simulation.'],
      report: ['Mission Report', 'Final pass/fail assessment and key flight metrics.'],
      campaign: ['Monte Carlo Campaigns', 'Batch dispersion analysis across randomized flight parameters.'],
      settings: ['Settings', 'Backend connection and application preferences.'],
    };
    const [t, s] = titles[name];
    $('#topbar-title').textContent = t;
    $('#topbar-sub').textContent = s;
  }
  $$('.nav-item').forEach((el) => el.addEventListener('click', () => showView(el.dataset.view)));

  $$('.tab').forEach((tab) => {
    tab.addEventListener('click', () => {
      $$('.tab').forEach((t) => t.classList.remove('active'));
      $$('.tab-panel').forEach((p) => p.classList.remove('active'));
      tab.classList.add('active');
      $(`.tab-panel[data-panel="${tab.dataset.tab}"]`).classList.add('active');
    });
  });

  // ---------------------------------------------------------------- forms
  function buildNumberField(container, section, key, value) {
    const meta = FIELD_META[section][key] || { label: key, unit: '', step: 1 };
    const wrap = document.createElement('div');
    wrap.className = 'field';
    wrap.innerHTML = `<label>${meta.label} <span class="unit">${meta.unit}</span></label>
      <input type="number" step="${meta.step}" data-section="${section}" data-key="${key}" value="${value}"/>`;
    container.appendChild(wrap);
    $('input', wrap).addEventListener('input', (e) => {
      state.formValues[section][key] = parseFloat(e.target.value);
    });
    state.formValues[section][key] = value;
  }

  function buildToggleField(container, section, key, value) {
    const meta = BOOLEAN_FIELD_META[section][key];
    const wrap = document.createElement('div');
    wrap.className = 'card';
    wrap.style.gridColumn = 'span 2';
    wrap.innerHTML = `<div class="toggle-row" style="border-bottom:none;padding:2px 4px;">
      <div><div class="lbl">${meta.label}</div><div class="desc">${meta.desc}</div></div>
      <label class="switch"><input type="checkbox" data-section="${section}" data-key="${key}" ${value ? 'checked' : ''}/><span class="track"><span class="thumb"></span></span></label>
    </div>`;
    container.appendChild(wrap);
    $('input', wrap).addEventListener('change', (e) => {
      state.formValues[section][key] = e.target.checked;
    });
    state.formValues[section][key] = value;
  }

  function populateForms(defaults) {
    state.defaults = defaults;
    state.formValues = { vehicle: {}, mission: {}, recovery: {}, physics: {} };

    const vC = $('#form-vehicle'); vC.innerHTML = '';
    Object.entries(defaults.vehicle).forEach(([k, v]) => buildNumberField(vC, 'vehicle', k, v));

    const mC = $('#form-mission'); mC.innerHTML = '';
    Object.entries(defaults.mission).forEach(([k, v]) => buildNumberField(mC, 'mission', k, v));

    const rC = $('#form-recovery'); rC.innerHTML = '';
    ['landing_lat', 'landing_lon'].forEach((k) => buildNumberField(rC, 'recovery', k, defaults.recovery[k]));
    ['grid_fins', 'landing_legs'].forEach((k) => buildToggleField(rC, 'recovery', k, defaults.recovery[k]));
    const quick = $('#quick-gfold');
    quick.innerHTML = '';
    if (defaults.recovery.gfold_available) {
      buildToggleField(quick, 'recovery', 'gfold_guidance', !!defaults.recovery.gfold_guidance);
    }
    state.formValues.recovery.mode = 'RTLS';
    state.formValues.recovery.landing_burn_alt = 'auto';
    state.formValues.recovery.suicide_burn = true;

    const pC = $('#form-physics'); pC.innerHTML = '';
    Object.entries(defaults.physics).forEach(([k, v]) => buildToggleField(pC, 'physics', k, v));

    $('#toggle-demo').checked = false;
    $('#toggle-s2').checked = !!defaults.enable_s2_recovery;
  }

  async function loadDefaults() {
    try {
      const defaults = await api('/api/config/defaults');
      state.advancedOverrides = null;
      populateForms(defaults);
      toast('Loaded validated default configuration', 'success');
    } catch (e) {
      toast('Failed to load defaults: ' + e.message, 'error');
    }
  }

  $('#btn-load-defaults').addEventListener('click', loadDefaults);

  $('#toggle-demo').addEventListener('change', (e) => {
    state.demoMode = e.target.checked;
    // Demo mode's whole point is a fast coarse-step preview; leaving
    // real-time playback on would throttle it back down to ~wall-clock
    // speed, silently defeating the toggle. Force it off (and lock it)
    // while demo mode is active.
    const rt = $('#toggle-realtime');
    if (state.demoMode) {
      rt.checked = false;
      rt.disabled = true;
      state.realtime = false;
    } else {
      rt.disabled = false;
    }
  });
  $('#toggle-s2').addEventListener('change', (e) => { state.s2Recovery = e.target.checked; });
  $('#toggle-realtime').addEventListener('change', (e) => { state.realtime = e.target.checked; });

  function currentSetup() {
    return {
      vehicle: state.formValues.vehicle,
      mission: state.formValues.mission,
      recovery: state.formValues.recovery,
      physics: state.formValues.physics,
      realtime_mode: state.realtime,
      demo_mode: state.demoMode,
      enable_s2_recovery: state.s2Recovery,
      ...(state.advancedOverrides ? { advanced_overrides: state.advancedOverrides } : {}),
    };
  }

  // Fields consumed by populateFromFullConfig()'s form mapping below. Any
  // field in an uploaded full-config JSON that isn't in this set has no
  // form control, so it used to be silently dropped when a file was
  // uploaded through the UI and then launched -- Launch only ever POSTed
  // the curated form fields. Now anything outside this set is preserved
  // in state.advancedOverrides and passed straight through to the backend
  // (see advanced_overrides in api_models.py / server.py's map_config()).
  const FULL_CONFIG_CONSUMED_KEYS = new Set([
    'runtime_thrust_scale', 'runtime_isp_scale', 'stage1_dry_mass', 'stage1_prop_mass',
    'stage2_dry_mass', 'stage2_prop_mass', 'payload_mass', 'stage2_thrust_vac',
    'stage2_isp_vac', 'max_gimbal_angle_deg', 'min_engine_throttle_fraction',
    'max_engine_throttle_fraction', 'orbit_target_altitude_m', 'target_inclination_deg',
    'orbit_insertion_start_altitude_m', 'stage_sep_velocity', 'dt', 'max_time',
    'booster_landing_site_lat_deg', 'launch_site_lat_deg', 'booster_landing_site_lon_deg',
    'launch_site_lon_deg', 'enable_grid_fins', 'enable_landing_legs', 'enable_j2',
    'enable_atmosphere', 'enable_drag', 'enable_lift', 'enable_imu', 'enable_gps',
    'enable_landing_altimeter', 'enable_s2_recovery', 'booster_landing_guidance',
    'stage1_engine_count', 'stage1_engine_thrust_n', 'boostback_engine_count',
    'entry_engine_count', 'landing_engine_count', 'vehicle_diameter_m',
  ]);

  $('#btn-export-config').addEventListener('click', () => {
    const blob = new Blob([JSON.stringify(currentSetup(), null, 2)], { type: 'application/json' });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'rlv_mission_config.json';
    a.click();
    toast('Configuration exported', 'success');
  });

  // ---------------------------------------------------------------- upload
  const dropzone = $('#dropzone');
  const fileInput = $('#file-input');
  dropzone.addEventListener('click', () => fileInput.click());
  ['dragover', 'dragenter'].forEach((ev) => dropzone.addEventListener(ev, (e) => { e.preventDefault(); dropzone.classList.add('drag'); }));
  ['dragleave', 'drop'].forEach((ev) => dropzone.addEventListener(ev, (e) => { e.preventDefault(); dropzone.classList.remove('drag'); }));
  dropzone.addEventListener('drop', (e) => { if (e.dataTransfer.files.length) handleConfigFile(e.dataTransfer.files[0]); });
  fileInput.addEventListener('change', (e) => { if (e.target.files.length) handleConfigFile(e.target.files[0]); });

  async function handleConfigFile(file) {
    try {
      const text = await file.text();
      const raw = JSON.parse(text);
      // Accept either a full SimulationSetup export, or raw backend config fields.
      if (raw.vehicle && raw.mission) {
        populateFromSetupShape(raw);
        toast(`Loaded configuration from ${file.name}`, 'success');
      } else {
        const fd = new FormData();
        fd.append('file', file);
        const result = await api('/api/config/validate', { method: 'POST', body: fd });
        populateFromFullConfig(result.config);
        toast(`Validated & loaded ${file.name}`, 'success');
      }
    } catch (e) {
      toast('Could not load configuration: ' + e.message, 'error');
    }
  }

  function populateFromSetupShape(setup) {
    const merged = {
      vehicle: setup.vehicle,
      mission: setup.mission,
      recovery: setup.recovery,
      physics: setup.physics,
      enable_s2_recovery: setup.enable_s2_recovery,
    };
    state.advancedOverrides = setup.advanced_overrides || null;
    populateForms(merged);
    state.demoMode = !!setup.demo_mode;
    state.realtime = setup.realtime_mode !== false;
    $('#toggle-demo').checked = state.demoMode;
    $('#toggle-realtime').checked = state.demoMode ? false : state.realtime;
    $('#toggle-realtime').disabled = state.demoMode;
    if (state.demoMode) state.realtime = false;
  }

  // Mirror server.py's own defaults derivation (server.py:373,375): the flat
  // SimulationConfig has no stage1_thrust_sl/stage1_isp_sl fields -- sea-level
  // thrust/Isp are only recoverable via the runtime_*_scale multipliers on
  // the reference constants. Reading a nonexistent field silently produced
  // stale/undefined values here; keep these in sync with constants.py
  // THRUST_MAGNITUDE / ISP if those reference values ever change.
  const REFERENCE_THRUST_SL_N = 7.607e6;
  const REFERENCE_ISP_SL_S = 282.0;

  function populateFromFullConfig(cfg) {
    const engineThrust = Number.isFinite(cfg.stage1_engine_thrust_n) && Number.isFinite(cfg.stage1_engine_count)
      ? cfg.stage1_engine_thrust_n * cfg.stage1_engine_count
      : REFERENCE_THRUST_SL_N;
    const thrustSl = Number.isFinite(cfg.runtime_thrust_scale)
      ? engineThrust * cfg.runtime_thrust_scale
      : state.formValues.vehicle.thrust_sl;
    const ispSl = Number.isFinite(cfg.runtime_isp_scale)
      ? REFERENCE_ISP_SL_S * cfg.runtime_isp_scale
      : state.formValues.vehicle.isp_sl;
    const defaults = {
      vehicle: {
        stage1_dry_mass: cfg.stage1_dry_mass, stage1_prop_mass: cfg.stage1_prop_mass,
        stage2_dry_mass: cfg.stage2_dry_mass, stage2_prop_mass: cfg.stage2_prop_mass,
        payload_mass: cfg.payload_mass, thrust_sl: thrustSl,
        thrust_vac: cfg.stage2_thrust_vac, isp_sl: ispSl,
        isp_vac: cfg.stage2_isp_vac, max_gimbal_deg: cfg.max_gimbal_angle_deg,
        throttle_min: cfg.min_engine_throttle_fraction, throttle_max: cfg.max_engine_throttle_fraction,
        stage1_engines: cfg.stage1_engine_count ?? 9, boostback_engines: cfg.boostback_engine_count ?? 3,
        entry_engines: cfg.entry_engine_count ?? 3, landing_engines: cfg.landing_engine_count ?? 1,
        diameter_m: cfg.vehicle_diameter_m ?? 3.7,
      },
      mission: {
        target_alt_km: cfg.orbit_target_altitude_m / 1000.0,
        target_inclination_deg: cfg.target_inclination_deg,
        orbit_insertion_start_alt_km: cfg.orbit_insertion_start_altitude_m / 1000.0,
        stage_sep_velocity: cfg.stage_sep_velocity, dt: cfg.dt, t_max: cfg.max_time,
      },
      recovery: {
        landing_lat: cfg.booster_landing_site_lat_deg ?? cfg.launch_site_lat_deg,
        landing_lon: cfg.booster_landing_site_lon_deg ?? cfg.launch_site_lon_deg,
        grid_fins: cfg.enable_grid_fins, landing_legs: cfg.enable_landing_legs,
        gfold_guidance: cfg.booster_landing_guidance === 'gfold',
      },
      physics: {
        j2: cfg.enable_j2, atmosphere: cfg.enable_atmosphere, drag: cfg.enable_drag,
        lift: cfg.enable_lift, sensor_noise: !!(cfg.enable_imu || cfg.enable_gps || cfg.enable_landing_altimeter),
      },
      enable_s2_recovery: !!cfg.enable_s2_recovery,
    };
    const extra = {};
    for (const [key, value] of Object.entries(cfg)) {
      if (!FULL_CONFIG_CONSUMED_KEYS.has(key) && value !== null && value !== undefined) {
        extra[key] = value;
      }
    }
    state.advancedOverrides = Object.keys(extra).length ? extra : null;
    populateForms(defaults);
    if (state.advancedOverrides) {
      const names = Object.keys(state.advancedOverrides);
      toast(
        `${names.length} advanced field${names.length === 1 ? '' : 's'} outside the form `
        + `(e.g. ${names.slice(0, 3).join(', ')}) will still be applied on launch, `
        + `just not shown in the UI.`,
        'success',
      );
    }
  }

  // ---------------------------------------------------------------- launch / websocket
  let ws = null;
  const telemetryLog = [];

  function setStatus(kind, label) {
    const pill = $('#status-pill');
    pill.className = `status-pill ${kind}`;
    $('#status-text').textContent = label;
    const badge = $('#nav-live-badge');
    badge.textContent = label.toLowerCase();
    badge.classList.toggle('live', kind === 'running');
  }

  function resetTelemetryUi() {
    telemetryLog.length = 0;
    Object.values(state.charts).forEach((c) => c.reset && c.reset());
    if (rocketScene) rocketScene.reset();
    $('#hw-row').innerHTML = '';
    $('#timeline-strip').innerHTML = '';
    $('#viz-orbiter-readout').innerHTML = '';
    $('#viz-booster-readout').innerHTML = '<span class="ph">Awaiting stage separation</span>';
    ['#vp-bst-alt', '#vp-bst-vel', '#vp-bst-thr', '#vp-bst-err', '#vp-bst-fuel', '#vp-bst-fuel-kg'].forEach((sel) => setText(sel, '—'));
    setText('#vp-bst-phase', 'Awaiting separation');
    lastTimelinePhase = null;
  }

  let rocketScene = null;

  function ensureCharts() {
    if (state.charts.altitude) return;
    rocketScene = new RocketScene($('#viz-orbiter'), $('#viz-booster'));
    state.charts.altitude = new StripChart($('#chart-altitude'), { color: '#38e0ff', unit: 'km' });
    state.charts.velocity = new StripChart($('#chart-velocity'), { color: '#7c8bff' });
    state.charts.mass = new StripChart($('#chart-mass'), { color: '#3ddc84' });
    state.charts.q = new StripChart($('#chart-q'), { color: '#ff5470' });
    state.charts.traj = new TrajectoryCanvas($('#chart-traj'));
  }

  function fmtClock(t) {
    const m = Math.floor(t / 60).toString().padStart(3, '0');
    const s = (t % 60).toFixed(1).padStart(4, '0');
    return `${m}:${s}`;
  }

  function updateTelemetryUi(payload) {
    telemetryLog.push(payload);
    const veh = state.activeVehicleTab;
    const prefix = veh === 'booster' ? 'booster_' : '';
    const alt = payload[`${prefix}altitude`] ?? payload.altitude;
    const vel = payload[`${prefix}velocity`] ?? payload.velocity;
    const mass = payload[`${prefix}mass`] ?? payload.mass;
    const throttle = payload[`${prefix}throttle`] ?? payload.throttle;
    const downrange = payload[`${prefix}downrange`] ?? payload.downrange;

    updateVehiclePanels(payload);

    const shownPhase = veh === 'booster' ? (payload.booster_phase || '—') : (payload.phase || '—');
    $('#phase-chip').textContent = PHASE_LABELS[shownPhase] || shownPhase;
    renderTimeline(shownPhase, veh === 'booster');
    updateVizReadout('#viz-orbiter-readout', payload.phase, payload.altitude, payload.velocity, payload.throttle);
    if (payload.booster_altitude !== undefined) {
      updateVizReadout('#viz-booster-readout', payload.booster_phase, payload.booster_altitude, payload.booster_velocity, payload.booster_throttle);
    } else {
      $('#viz-booster-readout').innerHTML = '<span class="ph">Awaiting stage separation</span>';
    }

    ensureCharts();
    if (rocketScene) rocketScene.update(payload);
    const t = payload.time;
    state.charts.altitude.push(t, alt / 1000);
    $('#ch-alt-now').textContent = fmt(alt / 1000, 1) + ' km';
    state.charts.velocity.push(t, vel);
    $('#ch-vel-now').textContent = fmt(vel, 0) + ' m/s';
    state.charts.mass.push(t, mass);
    $('#ch-mass-now').textContent = fmt(mass, 0) + ' kg';
    if (veh !== 'booster') {
      state.charts.q.push(t, (payload.dynamic_pressure || 0) / 1000);
      $('#ch-q-now').textContent = fmt((payload.dynamic_pressure || 0) / 1000, 1) + ' kPa';
    }
    state.charts.traj.push('primary', (payload.downrange || 0) / 1000, (payload.altitude || 0) / 1000);
    if (payload.booster_altitude !== undefined) {
      state.charts.traj.push('booster', (payload.booster_downrange || 0) / 1000, (payload.booster_altitude || 0) / 1000);
    }

    if (payload.booster_altitude !== undefined && !state.sim.hasBooster) {
      state.sim.hasBooster = true;
      $('#vehicle-toggle').style.display = 'flex';
    }

    const hwRow = $('#hw-row');
    hwRow.innerHTML = '';
    if (payload.booster_grid_fins) hwRow.appendChild(hwChip('Grid Fins', payload.booster_grid_fins === 'DEPLOYED'));
    if (payload.booster_landing_legs) hwRow.appendChild(hwChip('Landing Legs', payload.booster_landing_legs === 'DEPLOYED'));

    if (state.sim.startedAt) {
      $('#mission-clock').textContent = fmtClock(t);
    }
  }

  function setText(sel, text) {
    const el = $(sel);
    if (el) el.textContent = text;
  }

  function updateVehiclePanels(p) {
    setText('#vp-orb-phase', PHASE_LABELS[p.phase] || p.phase || '—');
    setText('#vp-orb-alt', fmt((p.altitude || 0) / 1000, 1));
    setText('#vp-orb-vel', fmt(p.velocity || 0, 0));
    setText('#vp-orb-thr', fmt((p.throttle || 0) * 100, 0));
    setText('#vp-orb-q', fmt((p.dynamic_pressure || 0) / 1000, 1));
    setText('#vp-orb-dr', fmt((p.downrange || 0) / 1000, 1));
    setText('#vp-orb-mass', fmt(p.mass || 0, 0));
    if (p.fuel_remaining_pct !== undefined) {
      setText('#vp-orb-fuel-lbl', p.fuel_stage === 'S2' ? 'S2 propellant' : 'S1 propellant (stack)');
      setFuelBar('#vp-orb-fuel', '#vp-orb-fuel-fill', p.fuel_remaining_pct);
      setText('#vp-orb-fuel-kg', fmt(p.fuel_remaining_kg, 0));
    }
    if (p.booster_altitude === undefined) return;
    setText('#vp-bst-phase', PHASE_LABELS[p.booster_phase] || p.booster_phase || '—');
    setText('#vp-bst-alt', fmt(p.booster_altitude / 1000, 2));
    setText('#vp-bst-vel', fmt(p.booster_velocity || 0, 0));
    setText('#vp-bst-thr', fmt((p.booster_throttle || 0) * 100, 0));
    setText('#vp-bst-err', p.booster_landing_error != null ? fmt(p.booster_landing_error, 0) : '—');
    setFuelBar('#vp-bst-fuel', '#vp-bst-fuel-fill', p.booster_fuel_remaining ?? 0);
    if (p.booster_fuel_remaining_kg !== undefined) setText('#vp-bst-fuel-kg', fmt(p.booster_fuel_remaining_kg, 0));
  }

  function setFuelBar(textSel, fillSel, pct) {
    const v = Math.max(0, Math.min(100, pct));
    setText(textSel, `${fmt(v, v < 10 ? 1 : 0)}%`);
    const fill = $(fillSel);
    if (fill) {
      fill.style.width = `${v}%`;
      fill.classList.toggle('low', v < 15);
    }
  }

  // ---------------------------------------------------------------- phase timeline
  const ORBITER_PHASES = [
    'PRELAUNCH', 'ASCENT', 'STAGE_SEPARATION', 'S2_COAST_TO_APOGEE',
    'ORBIT_INSERTION', 'ORBIT_ACHIEVED',
  ];
  const BOOSTER_PHASES = [
    'BOOSTER_FLIP', 'BOOSTER_BOOSTBACK', 'BOOSTER_ENTRY', 'BOOSTER_LANDING',
  ];
  const PHASE_LABELS = {
    PRELAUNCH: 'Prelaunch', ASCENT: 'Ascent', STAGE_SEPARATION: 'Stage Sep',
    S2_COAST_TO_APOGEE: 'Coast', ORBIT_INSERTION: 'Insertion', ORBIT_ACHIEVED: 'Orbit',
    ORBIT_FAILED: 'Orbit Failed', APOGEE_REACHED: 'Apogee', S2_ORBIT_HOLD: 'Orbit Hold',
    S2_DEORBIT: 'Deorbit', S2_ENTRY: 'S2 Entry', S2_LANDING: 'S2 Landing',
    BOOSTER_FLIP: 'Flip', BOOSTER_BOOSTBACK: 'Boostback', BOOSTER_COAST: 'Coast', BOOSTER_ENTRY: 'Entry Burn',
    BOOSTER_LANDING: 'Landing Burn',
  };
  let lastTimelinePhase = null;

  function renderTimeline(phase, boosterView) {
    // One vehicle's path at a time: the orbiter's ascent-to-orbit sequence, or
    // the booster's (shared ascent + recovery). Merging both made booster steps
    // show as "done" whenever the orbiter was further along.
    const key = `${boosterView ? 'B' : 'O'}:${phase}`;
    if (key === lastTimelinePhase) return;
    lastTimelinePhase = key;
    const steps = boosterView ? [...ORBITER_PHASES.slice(0, 3), ...BOOSTER_PHASES] : ORBITER_PHASES;
    const idx = steps.indexOf(phase);
    const strip = $('#timeline-strip');
    strip.innerHTML = steps.map((p, i) => {
      const cls = idx === -1 ? '' : (i < idx ? 'done' : i === idx ? 'now' : '');
      const connector = i > 0 ? '<span class="timeline-connector"></span>' : '';
      return `${connector}<span class="timeline-step ${cls}"><span class="dot"></span><span class="lbl">${PHASE_LABELS[p] || p}</span></span>`;
    }).join('');
  }

  function updateVizReadout(id, phase, alt, vel, thr) {
    const el = $(id);
    if (!el) return;
    el.innerHTML = `<span class="ph">${PHASE_LABELS[phase] || phase || '—'}</span>ALT ${fmt(alt / 1000, 2)} km &nbsp; VEL ${fmt(vel, 0)} m/s &nbsp; THR ${Math.round((thr || 0) * 100)}%`;
  }

  function hwChip(label, on) {
    const el = document.createElement('div');
    el.className = `hw-chip ${on ? 'on' : ''}`;
    el.innerHTML = `<span class="dot"></span>${label}`;
    return el;
  }

  $$('#vehicle-toggle button').forEach((btn) => {
    btn.addEventListener('click', () => {
      $$('#vehicle-toggle button').forEach((b) => b.classList.remove('active'));
      btn.classList.add('active');
      state.activeVehicleTab = btn.dataset.veh;
      if (telemetryLog.length) updateTelemetryUi(telemetryLog[telemetryLog.length - 1]);
    });
  });

  async function launchSimulation() {
    $('#launch-error').textContent = '';
    try {
      const setup = currentSetup();
      const result = await api('/api/simulations/start', { method: 'POST', body: JSON.stringify(setup) });
      state.sim = { id: result.simulation_id, token: result.access_token, status: 'running', startedAt: Date.now(), hasBooster: false };
      state.activeVehicleTab = 'primary';
      resetTelemetryUi();
      $('#telemetry-empty').style.display = 'none';
      $('#telemetry-live').style.display = 'block';
      $('#vehicle-toggle').style.display = 'none';
      $('#btn-pause').style.display = 'inline-flex';
      $('#btn-resume').style.display = 'none';
      $('#btn-stop').disabled = false;
      showView('telemetry');
      setStatus('running', 'RUNNING');
      connectTelemetryWs();
      toast('Simulation launched: ' + result.simulation_id, 'success');
    } catch (e) {
      $('#launch-error').textContent = e.message;
      toast('Launch failed: ' + e.message, 'error');
    }
  }
  $('#btn-launch').addEventListener('click', launchSimulation);

  function connectTelemetryWs() {
    if (ws) { try { ws.close(); } catch (_) {} }
    const wsBase = state.baseUrl.replace(/^http/, 'ws');
    const params = new URLSearchParams();
    if (state.sim.token) params.set('access_token', state.sim.token);
    if (state.apiKey) params.set('api_key', state.apiKey);
    ws = new WebSocket(`${wsBase}/ws/simulations/${state.sim.id}/telemetry?${params.toString()}`);
    ws.onmessage = (evt) => {
      const payload = JSON.parse(evt.data);
      if (payload.error) {
        toast('Simulation error: ' + payload.error, 'error');
        setStatus('error', 'ERROR');
        disableSimControls();
        return;
      }
      if (payload.event === 'completed' || payload.event === 'stopped') {
        setStatus(payload.status === 'completed' ? 'completed' : 'idle', payload.status.toUpperCase());
        // Previously the Pause/Resume/Stop control bar stayed visible and
        // clickable after the mission actually ended (both on manual stop
        // and natural completion) -- nothing here ever hid it. Clicking
        // Pause/Stop again against a terminal-state simulation just got a
        // 409 "not running" error, which looked indistinguishable from
        // "Stop did nothing" since the buttons never changed.
        disableSimControls();
        if (payload.outcomes) {
          buildReport(payload.outcomes);
          if (rocketScene) rocketScene.onMissionComplete(payload.outcomes);
          const oc = payload.outcomes;
          $('#viz-orbiter-readout').insertAdjacentHTML('afterbegin', `<span class="banner ${oc.orbiter_success ? 'ok' : 'fail'}">${oc.orbiter_success ? 'ORBIT ACHIEVED' : 'ORBIT FAILED'}</span>`);
          $('#viz-booster-readout').insertAdjacentHTML('afterbegin', `<span class="banner ${oc.booster_success ? 'ok' : 'fail'}">${oc.booster_success ? 'TOUCHDOWN CONFIRMED' : 'LANDING FAILED'}</span>`);
        }
        return;
      }
      updateTelemetryUi(payload);
    };
    ws.onerror = () => toast('Telemetry connection error', 'error');
  }

  function disableSimControls() {
    $('#btn-pause').style.display = 'none';
    $('#btn-resume').style.display = 'none';
    $('#btn-stop').disabled = true;
  }

  async function simControl(action) {
    if (!state.sim.id) return;
    try {
      await api(`/api/simulations/${state.sim.id}/${action}`, {
        method: 'POST',
        headers: { 'X-Simulation-Token': state.sim.token },
      });
      if (action === 'pause') { setStatus('paused', 'PAUSED'); $('#btn-pause').style.display = 'none'; $('#btn-resume').style.display = 'inline-flex'; }
      if (action === 'resume') { setStatus('running', 'RUNNING'); $('#btn-pause').style.display = 'inline-flex'; $('#btn-resume').style.display = 'none'; }
      if (action === 'stop') { setStatus('idle', 'STOPPED'); }
    } catch (e) {
      toast(`Could not ${action}: ${e.message}`, 'error');
    }
  }
  $('#btn-pause').addEventListener('click', () => simControl('pause'));
  $('#btn-resume').addEventListener('click', () => simControl('resume'));
  $('#btn-stop').addEventListener('click', async () => {
    const ok = await confirmModal('Stop simulation?', 'This ends the running mission early. Progress so far will not produce a mission report.');
    if (ok) simControl('stop');
  });

  // ---------------------------------------------------------------- report
  function extractNumber(text, pattern) {
    if (!text) return null;
    const m = text.match(pattern);
    return m ? parseFloat(m[1]) : null;
  }

  function downloadBlob(filename, text, type) {
    const blob = new Blob([text], { type });
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = filename;
    a.click();
  }

  function telemetryToCsv() {
    if (!telemetryLog.length) return '';
    const cols = [...new Set(telemetryLog.flatMap((row) =>
      Object.keys(row).filter((k) => row[k] === null || typeof row[k] !== 'object')
    ))];
    const escapeCell = (value) => {
      const text = value == null || typeof value === 'object' ? '' : String(value);
      return /[",\r\n]/.test(text) ? `"${text.replace(/"/g, '""')}"` : text;
    };
    const rows = telemetryLog.map((row) => cols.map((c) => escapeCell(row[c])).join(','));
    return [cols.map(escapeCell).join(','), ...rows].join('\n');
  }

  function buildReport(outcomes) {
    $('#report-empty').style.display = 'none';
    const content = $('#report-content');
    content.style.display = 'block';
    const success = outcomes.orbiter_success && outcomes.booster_success;
    const last = telemetryLog[telemetryLog.length - 1] || {};
    const a = outcomes.assessment || null;
    const orbit = a && a.orbit || null;
    const landing = a && a.landing || null;
    const s2 = a && a.s2_recovery || null;

    // Telemetry ticks don't necessarily land exactly on the touchdown instant,
    // so prefer the authoritative assessment computed server-side; fall back
    // to parsing the outcome text, then the last sampled telemetry point.
    const touchdownVelocity = landing?.touchdown_speed_mps
      ?? extractNumber(outcomes.booster_reason, /at\s+([\d.]+)\s*m\/s/i) ?? last.booster_velocity;
    const landingError = landing?.site_error_m
      ?? extractNumber(outcomes.booster_reason, /error\s+([\d.]+)\s*m\b/i) ?? last.booster_landing_error;

    const failedCriteria = (a?.failed_criteria || []);
    const failedCriteriaHtml = failedCriteria.length
      ? `<div class="help-text" style="margin-top:8px;">Failed gates: ${failedCriteria.map((f) => `<span class="badge bad" style="margin-right:4px;">${f}</span>`).join('')}</div>`
      : '';

    content.innerHTML = `
      <div class="result-banner ${success ? 'success' : 'failure'}">
        <div class="icon">
          ${success
            ? '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"><path d="M20 6L9 17l-5-5"/></svg>'
            : '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2"><path d="M18 6L6 18M6 6l12 12"/></svg>'}
        </div>
        <div>
          <h2>${success ? 'Mission Successful' : 'Mission Incomplete'}</h2>
          <p>${outcomes.reason || ''}</p>
          ${failedCriteriaHtml}
        </div>
      </div>
      <div class="grid grid-2">
        <div class="card">
          <div class="card-head"><div class="card-title">Orbiter</div><span class="badge ${outcomes.orbiter_success ? 'ok' : 'bad'}">${outcomes.orbiter_success ? 'SUCCESS' : 'FAIL'}</span></div>
          <div class="metric-grid" style="grid-template-columns:repeat(2,1fr);">
            <div class="metric-cell"><div class="k">Apogee</div><div class="v">${fmt(orbit?.apogee_altitude_km ?? last.apogee_altitude / 1000, 1)} km</div></div>
            <div class="metric-cell"><div class="k">Perigee</div><div class="v">${fmt(orbit?.perigee_altitude_km ?? last.perigee_altitude / 1000, 1)} km</div></div>
            <div class="metric-cell"><div class="k">Eccentricity</div><div class="v">${fmt(orbit?.eccentricity ?? last.eccentricity, 4)}</div></div>
            <div class="metric-cell"><div class="k">Final Mass</div><div class="v">${fmt(last.mass, 0)} kg</div></div>
            <div class="metric-cell"><div class="k">Velocity Deficit</div><div class="v">${fmt(orbit?.velocity_deficit_mps, 1)} m/s</div></div>
            <div class="metric-cell"><div class="k">Apogee Error</div><div class="v">${fmt(orbit?.apogee_error_km, 2)} km</div></div>
            <div class="metric-cell"><div class="k">Perigee Error</div><div class="v">${fmt(orbit?.perigee_error_km, 2)} km</div></div>
            <div class="metric-cell"><div class="k">Tolerance</div><div class="v">±${fmt(orbit?.altitude_tolerance_km, 0)} km</div></div>
          </div>
          <p class="help-text">${outcomes.orbiter_reason || ''}</p>
        </div>
        <div class="card">
          <div class="card-head"><div class="card-title">Booster Recovery</div><span class="badge ${outcomes.booster_success ? 'ok' : 'bad'}">${outcomes.booster_success ? 'SUCCESS' : 'FAIL'}</span></div>
          <div class="metric-grid" style="grid-template-columns:repeat(2,1fr);">
            <div class="metric-cell"><div class="k">Landing Error</div><div class="v">${fmt(landingError, 0)} m</div></div>
            <div class="metric-cell"><div class="k">Touchdown Velocity</div><div class="v">${fmt(touchdownVelocity, 1)} m/s</div></div>
            <div class="metric-cell"><div class="k">Final Mass</div><div class="v">${fmt(last.booster_mass, 0)} kg</div></div>
            <div class="metric-cell"><div class="k">Fuel Reserve</div><div class="v">${fmt(last.booster_fuel_remaining, 0)} %</div></div>
            <div class="metric-cell"><div class="k">Vertical Speed</div><div class="v">${fmt(landing?.vertical_speed_mps, 1)} m/s</div></div>
            <div class="metric-cell"><div class="k">Horizontal Speed</div><div class="v">${fmt(landing?.horizontal_speed_mps, 1)} m/s</div></div>
            <div class="metric-cell"><div class="k">Landing Legs</div><div class="v">${landing?.landing_leg_status ?? '—'}</div></div>
            <div class="metric-cell"><div class="k">Touchdown Contact</div><div class="v">${landing?.touchdown_contact_status ?? '—'}</div></div>
          </div>
          <p class="help-text">${outcomes.booster_reason || ''}</p>
        </div>
      </div>
      <div class="card" style="margin-top:16px;">
        <div class="card-head"><div class="card-title">Mission Envelope &amp; Loads</div></div>
        <div class="metric-grid" style="grid-template-columns:repeat(3,1fr);">
          <div class="metric-cell"><div class="k">Max Ascent Q</div><div class="v">${fmt(a?.max_ascent_q_pa, 0)} Pa</div></div>
          <div class="metric-cell"><div class="k">Max Recovery Q</div><div class="v">${fmt(a?.max_recovery_q_pa, 0)} Pa</div></div>
          <div class="metric-cell"><div class="k">Max Recovery Q-Alpha</div><div class="v">${fmt(a?.max_recovery_q_alpha_pa_rad, 1)} Pa·rad</div></div>
          <div class="metric-cell"><div class="k">Separation Time</div><div class="v">${fmt(a?.separation_time_s, 1)} s</div></div>
          <div class="metric-cell"><div class="k">Mission Duration</div><div class="v">${fmt(a?.mission_duration_s, 1)} s</div></div>
          <div class="metric-cell"><div class="k">Terminal Attitude Error</div><div class="v">${fmt(a?.terminal_attitude_error_deg, 2)}°</div></div>
        </div>
      </div>
      ${s2 ? `
      <div class="card" style="margin-top:16px;">
        <div class="card-head"><div class="card-title">Stage-2 Recovery</div><span class="badge ${s2.landing_success ? 'ok' : 'bad'}">${s2.landing_success ? 'SUCCESS' : 'FAIL'}</span></div>
        <div class="metric-grid" style="grid-template-columns:repeat(2,1fr);">
          <div class="metric-cell"><div class="k">Initial Orbit</div><div class="v">${s2.orbit_achieved ? 'ACHIEVED' : 'MISSED'}</div></div>
          <div class="metric-cell"><div class="k">Apogee</div><div class="v">${fmt(s2.achieved_apogee_km, 1)} km</div></div>
          <div class="metric-cell"><div class="k">Perigee</div><div class="v">${fmt(s2.achieved_perigee_km, 1)} km</div></div>
          <div class="metric-cell"><div class="k">Eccentricity</div><div class="v">${fmt(s2.achieved_eccentricity, 4)}</div></div>
          <div class="metric-cell"><div class="k">Touchdown Speed</div><div class="v">${fmt(s2.touchdown_speed_mps, 1)} m/s</div></div>
          <div class="metric-cell"><div class="k">Downrange</div><div class="v">${fmt(s2.downrange_km, 0)} km</div></div>
          <div class="metric-cell"><div class="k">Fuel Remaining</div><div class="v">${fmt(s2.fuel_remaining_kg, 0)} kg</div></div>
          <div class="metric-cell"><div class="k">Touchdown Time</div><div class="v">${fmt(s2.touchdown_time_s, 1)} s</div></div>
        </div>
        <p class="help-text">${s2.reason || ''}</p>
      </div>` : ''}
      <div style="display:flex;gap:10px;margin-top:16px;">
        <button class="btn btn-sm" id="btn-export-telemetry-csv">Export Telemetry CSV</button>
        <button class="btn btn-sm" id="btn-export-report-json">Export Report JSON</button>
      </div>`;
    $('#btn-export-telemetry-csv').addEventListener('click', () => {
      downloadBlob(`rlv_telemetry_${state.sim.id || 'run'}.csv`, telemetryToCsv(), 'text/csv');
    });
    $('#btn-export-report-json').addEventListener('click', () => {
      downloadBlob(`rlv_report_${state.sim.id || 'run'}.json`, JSON.stringify({ outcomes, final: last }, null, 2), 'application/json');
    });
    showView('report');
  }

  // ---------------------------------------------------------------- campaign
  $('#btn-run-campaign').addEventListener('click', async () => {
    try {
      const body = {
        mode: $('#camp-mode').value,
        runs: parseInt($('#camp-runs').value, 10),
        seed: parseInt($('#camp-seed').value, 10),
        workers: parseInt($('#camp-workers').value, 10),
        setup: currentSetup(),
      };
      body.setup.recovery = { ...body.setup.recovery, gfold_guidance: $('#camp-guidance').value === 'gfold' };
      const result = await api('/api/campaigns/start', { method: 'POST', body: JSON.stringify(body) });
      state.campaign.id = result.campaign_id;
      $('#camp-idle').style.display = 'none';
      $('#camp-progress').style.display = 'block';
      $('#camp-results').style.display = 'none';
      toast('Campaign started: ' + result.campaign_id, 'success');
      pollCampaign();
    } catch (e) {
      toast('Campaign launch failed: ' + e.message, 'error');
    }
  });

  $('#btn-cancel-campaign').addEventListener('click', async () => {
    if (!state.campaign.id) return;
    try {
      await api(`/api/campaigns/${state.campaign.id}/cancel`, { method: 'POST' });
      toast('Cancelling campaign…', 'info');
    } catch (e) {
      toast('Campaign cancel failed: ' + e.message, 'error');
    }
  });

  function pollCampaign() {
    clearInterval(state.campaign.pollHandle);
    state.campaign.pollHandle = setInterval(async () => {
      try {
        const r = await api(`/api/campaigns/${state.campaign.id}`);
        const pct = r.total ? Math.round((r.completed / r.total) * 100) : 0;
        $('#camp-fill').style.width = pct + '%';
        const passN = r.completed_success ?? 0;
        const failN = r.completed - passN;
        $('#camp-progress-text').textContent = `${r.completed} / ${r.total} cases complete · ${passN} pass / ${failN} fail`;
        if (r.status === 'completed' || r.status === 'cancelled' || r.status === 'error') {
          clearInterval(state.campaign.pollHandle);
          $('#camp-progress').style.display = 'none';
          $('#camp-idle').style.display = 'block';
          $('#camp-idle').textContent = `Last campaign: ${r.status}`;
          if (r.error) toast('Campaign error: ' + r.error, 'error');
          if (r.summary) renderCampaignResults(r.summary, r.results);
        }
      } catch (e) {
        clearInterval(state.campaign.pollHandle);
        toast('Campaign polling failed: ' + e.message, 'error');
      }
    }, 1500);
  }

  function renderCampaignResults(summary, results) {
    $('#camp-results').style.display = 'block';
    const grid = $('#camp-summary-grid');
    const ci = summary.mission_success_rate_95pct_ci || [0, 0];
    grid.innerHTML = `
      <div class="metric-cell"><div class="k">Runs</div><div class="v">${summary.runs}</div></div>
      <div class="metric-cell"><div class="k">Mission Success Rate</div><div class="v ${summary.mission_success_rate > 0.9 ? 'pass' : 'fail'}">${fmt(summary.mission_success_rate * 100, 1)}%</div></div>
      <div class="metric-cell"><div class="k">95% CI</div><div class="v">[${fmt(ci[0] * 100, 1)}, ${fmt(ci[1] * 100, 1)}]%</div></div>
      <div class="metric-cell"><div class="k">Orbit / Landing Rate</div><div class="v">${fmt(summary.orbit_success_rate * 100, 0)}% / ${fmt(summary.landing_success_rate * 100, 0)}%</div></div>`;

    const failureGrid = $('#camp-failure-grid');
    const failureCounts = summary.failure_criteria_counts || {};
    const failureEntries = Object.entries(failureCounts);
    failureGrid.innerHTML = failureEntries.length
      ? failureEntries.map(([k, v]) => `<div class="metric-cell"><div class="k">${k}</div><div class="v fail">${v}</div></div>`).join('')
      : '<div class="help-text">No failure criteria recorded — all cases passed.</div>';

    const metricsTable = $('#camp-metrics-table');
    const metrics = summary.metrics || {};
    const metricRows = Object.entries(metrics).filter(([, v]) => v);
    metricsTable.querySelector('thead').innerHTML =
      '<tr><th>Metric</th><th>Median</th><th>P05</th><th>P95</th><th>Min</th><th>Max</th></tr>';
    metricsTable.querySelector('tbody').innerHTML = metricRows.length
      ? metricRows.map(([name, s]) => `<tr><td>${escapeHtml(name)}</td><td>${fmt(s.median, 2)}</td><td>${fmt(s.p05, 2)}</td><td>${fmt(s.p95, 2)}</td><td>${fmt(s.min, 2)}</td><td>${fmt(s.max, 2)}</td></tr>`).join('')
      : '<tr><td colspan="6">No metric data available.</td></tr>';

    const table = $('#camp-table');
    const cols = ['case_id', 'mode', 'mission_success', 'orbit_success', 'landing_success', 'wall_time_s', 'reason'];
    table.querySelector('thead').innerHTML = `<tr>${cols.map((c) => `<th>${c === 'reason' ? 'Failure Reason' : c}</th>`).join('')}</tr>`;
    table.querySelector('tbody').innerHTML = results.slice(0, 300).map((row) => `<tr>${cols.map((c) => {
      if (c === 'mission_success' || c === 'orbit_success' || c === 'landing_success') {
        return `<td><span class="badge ${row[c] ? 'ok' : 'bad'}">${row[c] ? 'PASS' : 'FAIL'}</span></td>`;
      }
      if (c === 'wall_time_s') return `<td>${fmt(row[c], 2)}s</td>`;
      if (c === 'reason') {
        if (row.mission_success) return '<td>—</td>';
        const reason = [!row.orbit_success ? row.orbiter_reason : '', !row.landing_success ? row.booster_reason : ''].filter(Boolean).join(' / ');
        return `<td style="max-width:280px;white-space:normal;">${reason ? escapeHtml(reason) : '—'}</td>`;
      }
      return `<td>${escapeHtml(row[c])}</td>`;
    }).join('')}</tr>`).join('');
  }

  // ---------------------------------------------------------------- settings
  $('#set-base-url').value = state.baseUrl;
  $('#set-api-key').value = state.apiKey;
  $('#btn-save-settings').addEventListener('click', () => {
    state.baseUrl = $('#set-base-url').value.trim() || window.location.origin;
    state.apiKey = $('#set-api-key').value.trim();
    localStorage.setItem('rlv_base_url', state.baseUrl);
    localStorage.setItem('rlv_api_key', state.apiKey);
    toast('Settings saved', 'success');
    checkBackend();
  });

  async function checkBackend() {
    try {
      await api('/api/health');
      $('#backend-dot').className = 'dot ok';
      $('#backend-label').textContent = 'backend connected';
    } catch (e) {
      $('#backend-dot').className = 'dot bad';
      $('#backend-label').textContent = 'backend unreachable';
    }
  }

  // ---------------------------------------------------------- aero validation
  $('#btn-run-aero-validation').addEventListener('click', async () => {
    const btn = $('#btn-run-aero-validation');
    btn.disabled = true;
    btn.textContent = 'Running…';
    try {
      const report = await api('/api/aero/validation');
      renderAeroValidation(report);
      toast(
        report.all_passed
          ? `All ${report.total} aero sanity checks passed`
          : `${report.failed} of ${report.total} aero sanity checks failed`,
        report.all_passed ? 'success' : 'error',
      );
    } catch (e) {
      toast('Aero validation failed: ' + e.message, 'error');
    } finally {
      btn.disabled = false;
      btn.textContent = 'Run Checks';
    }
  });

  function renderAeroValidation(report) {
    $('#aero-validation-empty').style.display = 'none';
    $('#aero-validation-results').style.display = 'block';

    const summary = $('#aero-validation-summary');
    summary.innerHTML = `
      <div class="metric-cell"><div class="k">Checks</div><div class="v">${report.total}</div></div>
      <div class="metric-cell"><div class="k">Passed</div><div class="v pass">${report.passed}</div></div>
      <div class="metric-cell"><div class="k">Failed</div><div class="v ${report.failed ? 'fail' : ''}">${report.failed}</div></div>`;

    const table = $('#aero-validation-table');
    table.querySelector('thead').innerHTML = '<tr><th>Check</th><th>Status</th><th>Detail</th></tr>';
    table.querySelector('tbody').innerHTML = report.checks.map((c) => `<tr>
        <td>${c.name}</td>
        <td><span class="badge ${c.passed ? 'ok' : 'bad'}">${c.passed ? 'PASS' : 'FAIL'}</span></td>
        <td style="max-width:420px;white-space:normal;">${c.detail}</td>
      </tr>`).join('');
  }

  $('#btn-run-aero-tier2').addEventListener('click', async () => {
    const btn = $('#btn-run-aero-tier2');
    btn.disabled = true;
    btn.textContent = 'Running…';
    try {
      const report = await api('/api/aero/validation/tier2');
      renderAeroTier2(report);
      toast(
        report.all_passed
          ? `All ${report.total} Tier 2 literature checks passed`
          : `${report.failed} of ${report.total} Tier 2 literature checks failed`,
        report.all_passed ? 'success' : 'error',
      );
    } catch (e) {
      toast('Tier 2 aero validation failed: ' + e.message, 'error');
    } finally {
      btn.disabled = false;
      btn.textContent = 'Run Checks';
    }
  });

  function renderAeroTier2(report) {
    $('#aero-tier2-empty').style.display = 'none';
    $('#aero-tier2-results').style.display = 'block';

    const summary = $('#aero-tier2-summary');
    summary.innerHTML = `
      <div class="metric-cell"><div class="k">Checks</div><div class="v">${report.total}</div></div>
      <div class="metric-cell"><div class="k">Passed</div><div class="v pass">${report.passed}</div></div>
      <div class="metric-cell"><div class="k">Failed</div><div class="v ${report.failed ? 'fail' : ''}">${report.failed}</div></div>`;

    const table = $('#aero-tier2-table');
    table.querySelector('thead').innerHTML = '<tr><th>Check</th><th>Status</th><th>Detail</th><th>Source</th></tr>';
    table.querySelector('tbody').innerHTML = report.checks.map((c) => `<tr>
        <td>${c.name}</td>
        <td><span class="badge ${c.passed ? 'ok' : 'bad'}">${c.passed ? 'PASS' : 'FAIL'}</span></td>
        <td style="max-width:380px;white-space:normal;">${c.detail}</td>
        <td style="max-width:220px;white-space:normal;font-size:10.5px;color:var(--text-lo);">${c.source || '—'}</td>
      </tr>`).join('');
  }

  // ---------------------------------------------------------------- init
  (async function init() {
    await checkBackend();
    await loadDefaults();
    setInterval(checkBackend, 15000);
  })();
})();
