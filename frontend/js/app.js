/* Vyom — dashboard application logic. No external dependencies. */
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
    autoReserve: false,
    realtime: true,
    sim: { id: null, token: null, status: 'idle', startedAt: null, hasBooster: false },
    activeVehicleTab: 'primary',
    campaign: { id: null, pollHandle: null, request: null, lastResults: null },
    schema: [],
    schemaMap: {},
    plugins: [],
    pluginFolder: '',
    gfoldAvailable: false,
    pendingSetup: null,
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
      const err = new Error(detail);
      err.status = res.status;
      throw err;
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
      compare: ['Compare Runs', 'Overlay two saved missions or Monte Carlo campaigns side by side.'],
      validation: ['Flight Validation', 'Simulated ascent against Falcon 9 flight telemetry.'],
      model: ['Model & Limits', 'What is modelled, from which sources, how credible each input is, and where the model stops being valid.'],
      settings: ['Settings', 'License, backend connection and aerodynamic data.'],
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
      if (tab.dataset.tab === 'advanced') renderAdvanced();
      if (tab.dataset.tab === 'wind') renderWindForm();
      if (tab.dataset.tab === 'recovery') renderGuidancePicker();
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
    if (defaults.recovery.gfold_available !== undefined) state.gfoldAvailable = !!defaults.recovery.gfold_available;
    state.formValues.recovery.gfold_guidance = !!defaults.recovery.gfold_guidance;
    state.formValues.recovery.mode = 'RTLS';
    state.formValues.recovery.landing_burn_alt = 'auto';
    state.formValues.recovery.suicide_burn = true;

    const pC = $('#form-physics'); pC.innerHTML = '';
    Object.entries(defaults.physics).forEach(([k, v]) => buildToggleField(pC, 'physics', k, v));

    $('#toggle-demo').checked = false;
    $('#toggle-s2').checked = !!defaults.enable_s2_recovery;
    state.autoReserve = !!defaults.auto_recovery_reserve;
    $('#toggle-auto-reserve').checked = state.autoReserve;
    pruneOverrides();
    renderGuidancePicker();
    renderWindForm();
    renderAdvanced();
    updateAdvancedCount();
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
  $('#toggle-auto-reserve').addEventListener('change', (e) => { state.autoReserve = e.target.checked; refreshPlan(); });
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
      auto_recovery_reserve: state.autoReserve,
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
    'orbit_insertion_start_altitude_m', 'dt', 'max_time',
    'booster_landing_site_lat_deg', 'launch_site_lat_deg', 'booster_landing_site_lon_deg',
    'launch_site_lon_deg', 'enable_grid_fins', 'enable_landing_legs', 'enable_j2',
    'enable_atmosphere', 'enable_drag', 'enable_lift', 'enable_imu', 'enable_gps',
    'enable_landing_altimeter', 'enable_s2_recovery', 'booster_landing_guidance', 'auto_size_recovery_reserve',
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
      auto_recovery_reserve: setup.auto_recovery_reserve,
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
        dt: cfg.dt, t_max: cfg.max_time,
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
      auto_recovery_reserve: !!cfg.auto_size_recovery_reserve,
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
        `${names.length} setting${names.length === 1 ? '' : 's'} differ from the defaults `
        + `(e.g. ${names.slice(0, 3).join(', ')}); see the All Settings tab.`,
        'success',
      );
    }
  }

  // ------------------------------------------------- all settings (schema)
  // Every SimulationConfig field comes from /api/config/schema. Edits are
  // stored in state.advancedOverrides only when they differ from the
  // default, and travel to the backend as advanced_overrides on launch.
  const WIND_FIELDS = {
    runtime_wind_offset_mps: 'Mean wind offset',
    booster_descent_drag_scale: 'Booster descent drag scale',
    runtime_atmosphere_density_scale: 'Atmosphere density scale',
    recovery_reserve_wind_sensitivity_kg_per_mps: 'Extra reserve per m/s headwind',
    enable_stochastic_wind: 'Turbulence and gusts',
    wind_turbulence_intensity: 'Turbulence intensity',
    wind_gust_magnitude: 'Gust magnitude',
    wind_seed: 'Turbulence random seed',
  };
  const WIND_MODES_FIELD = 'runtime_wind_profile_modes';
  // Edited by the landing-guidance picker in the Booster Recovery tab.
  const GUIDANCE_FIELDS = new Set(['booster_landing_guidance', 'booster_landing_plugin']);

  function humanize(name) {
    const text = name.replace(/_(kg_per_mps|kgm2|mps2|mps|kg|km|deg|pa_rad|pa|rad|m2|n|s|m)$/, '').replace(/_/g, ' ');
    return text.charAt(0).toUpperCase() + text.slice(1);
  }

  const sameValue = (a, b) => JSON.stringify(a ?? null) === JSON.stringify(b ?? null);

  function overrideValue(name) {
    const o = state.advancedOverrides || {};
    if (Object.prototype.hasOwnProperty.call(o, name)) return o[name];
    const row = state.schemaMap[name];
    return row ? row.default : undefined;
  }

  function setOverride(name, value) {
    const o = { ...(state.advancedOverrides || {}) };
    const row = state.schemaMap[name];
    if (row && sameValue(value, row.default)) delete o[name]; else o[name] = value;
    state.advancedOverrides = Object.keys(o).length ? o : null;
    updateAdvancedCount();
    clearTimeout(planTimer);
    planTimer = setTimeout(refreshPlan, 400);
  }

  // Drop overrides that only restate the default (e.g. from an uploaded full config).
  function pruneOverrides() {
    if (!state.advancedOverrides) return;
    const o = {};
    Object.entries(state.advancedOverrides).forEach(([k, v]) => {
      const row = state.schemaMap[k];
      if (!row || !sameValue(v, row.default)) o[k] = v;
    });
    state.advancedOverrides = Object.keys(o).length ? o : null;
  }

  function changedSettingNames() {
    return Object.keys(state.advancedOverrides || {}).filter((k) => state.schemaMap[k]);
  }

  function updateAdvancedCount() {
    const n = changedSettingNames().length;
    const el = $('#adv-count');
    if (el) el.textContent = n ? `${n} changed` : '';
  }

  function boundsText(row) {
    const parts = [];
    if (row.min !== undefined && row.min !== null) parts.push(`${row.min_exclusive ? '>' : '≥'} ${row.min}`);
    if (row.max !== undefined && row.max !== null) parts.push(`${row.max_exclusive ? '<' : '≤'} ${row.max}`);
    return parts.join(', ');
  }

  function boundsError(row, value) {
    if (row.min !== undefined && row.min !== null && (row.min_exclusive ? value <= row.min : value < row.min)) return `must be ${boundsText(row)}`;
    if (row.max !== undefined && row.max !== null && (row.max_exclusive ? value >= row.max : value > row.max)) return `must be ${boundsText(row)}`;
    if (row.kind === 'int' && !Number.isInteger(value)) return 'must be a whole number';
    return null;
  }

  function fmtDefault(v) {
    if (v === null || v === undefined) return 'auto';
    if (Array.isArray(v)) return v.length ? JSON.stringify(v) : 'none';
    return String(v);
  }

  /** One editor for a schema field; onChange() runs after a valid edit. */
  function buildSchemaField(row, label, onChange) {
    const wrap = document.createElement('div');
    wrap.className = 'field schema-field';
    wrap.dataset.name = row.name;
    const value = overrideValue(row.name);
    const changed = !sameValue(value, row.default);
    wrap.classList.toggle('changed', changed);
    const help = escapeHtml(row.description || '');
    const bounds = boundsText(row);
    let control;
    if (row.kind === 'bool') {
      control = `<label class="switch"><input type="checkbox" ${value ? 'checked' : ''}/><span class="track"><span class="thumb"></span></span></label>`;
    } else if (row.kind === 'choice') {
      control = `<select>${row.choices.map((c) => `<option value="${escapeHtml(c)}" ${c === value ? 'selected' : ''}>${escapeHtml(c)}</option>`).join('')}</select>`;
    } else if (row.kind === 'wind_modes') {
      control = `<div class="help-text">${(value || []).length} mode(s) — edit in the Wind &amp; Dispersions tab</div>`;
    } else if (row.kind === 'str') {
      control = `<input type="text" value="${escapeHtml(value ?? '')}"/>`;
    } else {
      const step = row.kind === 'int' ? 1 : 'any';
      control = `<input type="number" step="${step}" value="${value ?? ''}" placeholder="${row.nullable ? 'auto' : ''}"/>`;
    }
    wrap.innerHTML = `<label title="${escapeHtml(row.name)}">${escapeHtml(label || humanize(row.name))}
        <span class="unit">${escapeHtml(row.unit || '')}</span></label>
      ${control}
      <div class="field-meta"><span>default ${escapeHtml(fmtDefault(row.default))}${bounds ? ` · ${escapeHtml(bounds)}` : ''}</span>
        <button class="link-btn reset" type="button" ${changed ? '' : 'hidden'}>reset</button></div>
      ${help ? `<div class="field-help">${help}</div>` : ''}
      <div class="field-error"></div>`;
    const input = $('input, select', wrap);
    const err = $('.field-error', wrap);
    const commit = (v) => {
      setOverride(row.name, v);
      const isChanged = !sameValue(v, row.default);
      wrap.classList.toggle('changed', isChanged);
      $('.reset', wrap).hidden = !isChanged;
      if (onChange) onChange(v);
    };
    if (input && row.kind !== 'wind_modes') {
      input.addEventListener(row.kind === 'bool' || row.kind === 'choice' ? 'change' : 'input', () => {
        err.textContent = '';
        wrap.classList.remove('invalid');
        if (row.kind === 'bool') return commit(input.checked);
        if (row.kind === 'choice' || row.kind === 'str') return commit(input.value);
        if (input.value === '' && row.nullable) return commit(null);
        const v = Number(input.value);
        const problem = input.value === '' || !Number.isFinite(v) ? 'enter a number' : boundsError(row, v);
        if (problem) { err.textContent = problem; wrap.classList.add('invalid'); return undefined; }
        return commit(v);
      });
    }
    $('.reset', wrap).addEventListener('click', () => {
      commit(row.default);
      const fresh = buildSchemaField(row, label, onChange);
      wrap.replaceWith(fresh);
    });
    return wrap;
  }

  function renderWindForm() {
    const box = $('#form-wind');
    if (!box || !state.schema.length) return;
    box.innerHTML = '';
    Object.entries(WIND_FIELDS).forEach(([name, label]) => {
      const row = state.schemaMap[name];
      if (row) box.appendChild(buildSchemaField(row, label));
    });
    renderWindModes();
  }

  function windModes() {
    return (overrideValue(WIND_MODES_FIELD) || []).map((m) => [Number(m[0]), Number(m[1]), Number(m[2])]);
  }

  function renderWindModes() {
    const modes = windModes();
    const body = $('#wind-modes-table tbody');
    $('#wind-modes-empty').style.display = modes.length ? 'none' : 'block';
    $('#wind-modes-table').style.display = modes.length ? '' : 'none';
    body.innerHTML = modes.map((m, i) => `<tr data-i="${i}">
        <td><input type="number" step="any" data-k="0" value="${m[0]}"/></td>
        <td><input type="number" step="any" min="1" data-k="1" value="${m[1]}"/></td>
        <td><input type="number" step="any" data-k="2" value="${+(m[2] * 180 / Math.PI).toFixed(2)}"/></td>
        <td><button class="btn btn-sm btn-danger" data-remove="${i}">Remove</button></td></tr>`).join('');
    $$('input', body).forEach((input) => input.addEventListener('input', () => {
      const i = Number(input.closest('tr').dataset.i);
      const k = Number(input.dataset.k);
      const v = Number(input.value);
      const bad = input.value === '' || !Number.isFinite(v) || (k === 1 && v <= 0);
      input.classList.toggle('invalid', bad);
      if (bad) return;
      const next = windModes();
      next[i][k] = k === 2 ? v * Math.PI / 180 : v;
      setOverride(WIND_MODES_FIELD, next);
    }));
    $$('[data-remove]', body).forEach((btn) => btn.addEventListener('click', () => {
      const next = windModes();
      next.splice(Number(btn.dataset.remove), 1);
      setOverride(WIND_MODES_FIELD, next);
      renderWindModes();
    }));
  }

  $('#btn-wind-add').addEventListener('click', () => {
    const next = windModes();
    if (next.length >= 12) { toast('Up to 12 wind modes', 'error'); return; }
    next.push([2.0, 5000.0, 0.0]);
    setOverride(WIND_MODES_FIELD, next);
    renderWindModes();
  });
  $('#btn-wind-preset').addEventListener('click', () => {
    // Same wavelengths the extended Monte Carlo dispersion uses.
    const next = windModes().concat([[2, 2000, 0], [2, 5000, 0], [2, 12000, 0], [2, 25000, 0]]);
    setOverride(WIND_MODES_FIELD, next.slice(0, 12));
    renderWindModes();
  });
  $('#btn-wind-clear').addEventListener('click', () => { setOverride(WIND_MODES_FIELD, []); renderWindModes(); });

  function renderAdvanced() {
    const box = $('#adv-groups');
    if (!box || !state.schema.length) return;
    const query = ($('#adv-search').value || '').trim().toLowerCase();
    const changedOnly = $('#adv-changed-only').checked;
    const groups = {};
    state.schema.forEach((row) => {
      if (row.on_form || GUIDANCE_FIELDS.has(row.name)) return;
      if (changedOnly && sameValue(overrideValue(row.name), row.default)) return;
      if (query && !(`${row.name} ${row.name.replace(/_/g, ' ')} ${row.description} ${row.group}`.toLowerCase().includes(query))) return;
      (groups[row.group] = groups[row.group] || []).push(row);
    });
    const open = new Set($$('details.adv-group[open]', box).map((d) => d.dataset.group));
    box.innerHTML = '';
    const names = Object.keys(groups).sort();
    if (!names.length) { box.innerHTML = '<div class="help-text">No settings match.</div>'; return; }
    names.forEach((group) => {
      const rows = groups[group];
      const nChanged = rows.filter((r) => !sameValue(overrideValue(r.name), r.default)).length;
      const det = document.createElement('details');
      det.className = 'adv-group';
      det.dataset.group = group;
      det.open = open.has(group) || Boolean(query) || changedOnly;
      det.innerHTML = `<summary>${escapeHtml(group)} <span class="unit">${rows.length} setting${rows.length === 1 ? '' : 's'}${nChanged ? ` · <b>${nChanged} changed</b>` : ''}</span></summary><div class="grid grid-3 adv-grid"></div>`;
      const grid = $('.adv-grid', det);
      rows.forEach((row) => grid.appendChild(buildSchemaField(row)));
      box.appendChild(det);
    });
  }
  let advTimer = null;
  $('#adv-search').addEventListener('input', () => { clearTimeout(advTimer); advTimer = setTimeout(renderAdvanced, 200); });
  $('#adv-changed-only').addEventListener('change', renderAdvanced);
  $('#btn-adv-reset').addEventListener('click', async () => {
    const ok = await confirmModal('Reset all settings?', 'Every value in All Settings and Wind & Dispersions goes back to its default. The other tabs are not changed.');
    if (!ok) return;
    const keep = {};
    Object.entries(state.advancedOverrides || {}).forEach(([k, v]) => { if (!state.schemaMap[k]) keep[k] = v; });
    state.advancedOverrides = Object.keys(keep).length ? keep : null;
    renderWindForm();
    renderAdvanced();
    renderGuidancePicker();
    updateAdvancedCount();
    toast('Settings reset to defaults', 'success');
  });

  // ------------------------------------------------- landing guidance picker
  function guidanceOptionsHtml(selected) {
    const opts = [['heuristic', 'Built-in: suicide burn with ZEM/ZEV divert', false]];
    opts.push(['gfold', state.gfoldAvailable ? 'Built-in: G-FOLD convex descent' : 'G-FOLD convex descent (needs cvxpy)', !state.gfoldAvailable]);
    state.plugins.forEach((p) => opts.push([`plugin:${p.name}`, `Plugin: ${p.title}${p.error ? ' (error)' : ''}`, Boolean(p.error)]));
    return opts.map(([v, label, disabled]) => `<option value="${escapeHtml(v)}" ${v === selected ? 'selected' : ''} ${disabled ? 'disabled' : ''}>${escapeHtml(label)}</option>`).join('');
  }

  function currentGuidanceValue() {
    if (overrideValue('booster_landing_guidance') === 'plugin') return `plugin:${overrideValue('booster_landing_plugin')}`;
    return state.formValues.recovery.gfold_guidance ? 'gfold' : 'heuristic';
  }

  /** Apply a picker value ('heuristic' | 'gfold' | 'plugin:<name>') to a setup copy or the live form. */
  function applyGuidance(value, setup) {
    const overrides = { ...((setup ? setup.advanced_overrides : state.advancedOverrides) || {}) };
    delete overrides.booster_landing_guidance;
    delete overrides.booster_landing_plugin;
    if (value.startsWith('plugin:')) {
      overrides.booster_landing_guidance = 'plugin';
      overrides.booster_landing_plugin = value.slice(7);
    }
    const gfold = value === 'gfold';
    if (setup) {
      setup.recovery = { ...setup.recovery, gfold_guidance: gfold };
      setup.advanced_overrides = Object.keys(overrides).length ? overrides : undefined;
      return setup;
    }
    state.formValues.recovery.gfold_guidance = gfold;
    state.advancedOverrides = Object.keys(overrides).length ? overrides : null;
    updateAdvancedCount();
    return null;
  }

  function renderGuidancePicker() {
    const box = $('#form-recovery');
    if (!box) return;
    let wrap = $('#guidance-picker', box);
    if (!wrap) {
      wrap = document.createElement('div');
      wrap.className = 'field';
      wrap.id = 'guidance-picker';
      wrap.style.gridColumn = 'span 2';
      box.prepend(wrap);
    }
    wrap.innerHTML = `<label>Landing guidance <span class="unit">landing burn</span></label>
      <select id="guidance-select">${guidanceOptionsHtml(currentGuidanceValue())}</select>
      <div class="field-help">Plugins are Python files in your plugin folder (see Model &amp; Limits).</div>`;
    $('#guidance-select').addEventListener('change', (e) => applyGuidance(e.target.value));
    const camp = $('#camp-guidance');
    if (camp) {
      const prev = camp.value || 'heuristic';
      camp.innerHTML = guidanceOptionsHtml(prev);
      if (camp.selectedOptions[0] && camp.selectedOptions[0].disabled) camp.value = 'heuristic';
    }
  }

  async function loadSchemaAndPlugins() {
    try {
      state.schema = await api('/api/config/schema');
      state.schemaMap = Object.fromEntries(state.schema.map((r) => [r.name, r]));
    } catch (e) {
      toast('Could not load the settings list: ' + e.message, 'error');
    }
    await refreshPlugins();
  }

  async function refreshPlugins() {
    try {
      const r = await api('/api/plugins');
      state.plugins = r.plugins;
      state.pluginFolder = r.folder;
    } catch (e) { state.plugins = []; }
    renderGuidancePicker();
    renderPluginTable();
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
      return `${connector}<span class="timeline-step ${cls}"><span class="dot"></span><span class="lbl">${escapeHtml(PHASE_LABELS[p] || p)}</span></span>`;
    }).join('');
  }

  function updateVizReadout(id, phase, alt, vel, thr) {
    const el = $(id);
    if (!el) return;
    el.innerHTML = `<span class="ph">${escapeHtml(PHASE_LABELS[phase] || phase || '—')}</span>ALT ${fmt(alt / 1000, 2)} km &nbsp; VEL ${fmt(vel, 0)} m/s &nbsp; THR ${Math.round((thr || 0) * 100)}%`;
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

  // Mission planner: rocket-equation feasibility, refreshed as the form changes.
  let planTimer = null;
  async function refreshPlan() {
    const el = $('#plan-status');
    if (!el) return;
    try {
      const plan = await api('/api/mission/plan', { method: 'POST', body: JSON.stringify(currentSetup()) });
      const colors = { ok: 'var(--green)', warn: 'var(--amber, orange)', fail: 'var(--red)' };
      el.innerHTML = plan.checks.map((c) =>
        `<div style="color:${colors[c.status]}">${c.status === 'ok' ? '✓' : c.status === 'warn' ? '⚠' : '✗'} ${escapeHtml(c.message)}</div>`).join('');
    } catch (err) {
      el.textContent = '';
    }
  }
  document.addEventListener('input', (e) => {
    if (!e.target.closest || !e.target.closest('.grid')) return;
    clearTimeout(planTimer);
    planTimer = setTimeout(refreshPlan, 400);
  });
  setTimeout(refreshPlan, 1500);

  async function launchSimulation() {
    $('#launch-error').textContent = '';
    try {
      const setup = state.pendingSetup || currentSetup();
      state.pendingSetup = null;
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
      state.pendingSetup = null;
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
        finishSimulation(payload.status, payload.outcomes);
        return;
      }
      updateTelemetryUi(payload);
    };
    ws.onerror = () => toast('Telemetry connection error', 'error');
    // The stream can drop before the final message (network blip, throttled
    // background window); recover the result from the status endpoint
    // instead of showing RUNNING forever.
    const thisWs = ws;
    ws.onclose = () => { if (thisWs === ws && !state.sim.finished) pollSimulationResult(state.sim.id); };
  }

  async function pollSimulationResult(simId) {
    for (let i = 0; i < 600 && state.sim.id === simId && !state.sim.finished; i += 1) {
      try {
        const r = await api(`/api/simulations/${simId}`, { headers: state.sim.token ? { 'X-Simulation-Token': state.sim.token } : {} });
        if (r.status === 'error') { toast('Simulation error: ' + (r.error || 'unknown'), 'error'); setStatus('error', 'ERROR'); disableSimControls(); return; }
        if (['completed', 'stopped', 'cancelled'].includes(r.status)) { finishSimulation(r.status, r.outcomes); return; }
      } catch (e) { /* transient: retry */ }
      await new Promise((res) => setTimeout(res, 2000));
    }
  }

  function finishSimulation(status, outcomes) {
    if (state.sim.finished) return;
    state.sim.finished = true;
    setStatus(status === 'completed' ? 'completed' : 'idle', String(status).toUpperCase());
    // Hide Pause/Resume/Stop once the mission has ended: clicking them on a
    // terminal-state simulation only produced a confusing 409.
    disableSimControls();
    if (!outcomes) return;
    buildReport(outcomes);
    if (rocketScene) rocketScene.onMissionComplete(outcomes);
    $('#viz-orbiter-readout').insertAdjacentHTML('afterbegin', `<span class="banner ${outcomes.orbiter_success ? 'ok' : 'fail'}">${outcomes.orbiter_success ? 'ORBIT ACHIEVED' : 'ORBIT FAILED'}</span>`);
    $('#viz-booster-readout').insertAdjacentHTML('afterbegin', `<span class="banner ${outcomes.booster_success ? 'ok' : 'fail'}">${outcomes.booster_success ? 'TOUCHDOWN CONFIRMED' : 'LANDING FAILED'}</span>`);
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

  // PDF export: the browser's print-to-PDF with print styles for one view.
  function printView(name) {
    document.body.dataset.print = name;
    window.print();
    delete document.body.dataset.print;
  }

  function reportHeaderHtml() {
    const v = state.formValues.vehicle || {}, m = state.formValues.mission || {}, r = state.formValues.recovery || {};
    const engines = v.stage1_engines || 9;
    const item = (k, val) => `<div class="metric-cell"><div class="k">${k}</div><div class="v">${escapeHtml(val)}</div></div>`;
    return `<div class="card report-header">
      <div class="card-head"><div class="card-title">Mission Summary</div>
        <div class="help-text">${escapeHtml(new Date().toLocaleString())} · run ${escapeHtml(state.sim.id || '—')}</div></div>
      <div class="metric-grid" style="grid-template-columns:repeat(3,1fr);">
        ${item('Payload', `${fmt(v.payload_mass, 0)} kg`)}
        ${item('Target orbit', `${fmt(m.target_alt_km, 0)} km`)}
        ${item('Stage-1 engines', `${engines} × ${fmt((v.thrust_sl || 0) / engines / 1000, 0)} kN`)}
        ${item('Liftoff thrust', `${fmt((v.thrust_sl || 0) / 1e6, 2)} MN`)}
        ${item('Diameter', `${fmt(v.diameter_m, 1)} m`)}
        ${item('Landing guidance', r.gfold_guidance ? 'G-FOLD' : 'Heuristic')}
      </div></div>`;
  }

  function phaseTimelineHtml() {
    const events = [];
    let prev = null, prevB = null;
    telemetryLog.forEach((p) => {
      if (p.phase && p.phase !== prev) { events.push([p.time, 'Orbiter', p.phase]); prev = p.phase; }
      if (p.booster_phase && p.booster_phase !== prevB && p.booster_phase !== 'UNKNOWN') {
        events.push([p.time, 'Booster', p.booster_phase]); prevB = p.booster_phase;
      }
    });
    if (!events.length) return '';
    const tEnd = Math.max(...telemetryLog.map((p) => p.time), 1);
    events.sort((a, b) => a[0] - b[0]);
    const rows = events.map(([t, who, ph]) =>
      `<tr><td class="mono">T+${fmt(t, 1)} s</td><td><span class="badge ${who === 'Booster' ? 'info' : 'ok'}">${who}</span></td><td>${escapeHtml(PHASE_LABELS[ph] || ph)}</td></tr>`).join('');
    const ticks = events.map(([t, who, ph]) =>
      `<div class="tl-tick ${who === 'Booster' ? 'b' : 'o'}" style="left:${(100 * t / tEnd).toFixed(2)}%" title="${escapeHtml(PHASE_LABELS[ph] || ph)}"></div>`).join('');
    return `<div class="card" style="margin-top:16px;">
      <div class="card-head"><div class="card-title">Phase Timeline</div><div class="help-text">${fmt(tEnd, 0)} s total</div></div>
      <div class="tl-track">${ticks}</div>
      <table class="data-table" style="margin-top:10px;"><thead><tr><th>Time</th><th>Vehicle</th><th>Phase</th></tr></thead><tbody>${rows}</tbody></table>
    </div>`;
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
      ? `<div class="help-text" style="margin-top:8px;">Failed gates: ${failedCriteria.map((f) => `<span class="badge bad" style="margin-right:4px;">${escapeHtml(f)}</span>`).join('')}</div>`
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
          <p>${escapeHtml(outcomes.reason || '')}</p>
          ${failedCriteriaHtml}
        </div>
      </div>
      ${reportHeaderHtml()}
      <div class="grid grid-2 report-charts">
        <div class="card">${ReportCharts.lineChart([
          { xs: telemetryLog.map((p) => p.time), ys: telemetryLog.map((p) => p.altitude / 1000), color: '#38e0ff', label: 'Orbiter / stack' },
          { xs: telemetryLog.filter((p) => p.booster_altitude !== undefined).map((p) => p.time),
            ys: telemetryLog.filter((p) => p.booster_altitude !== undefined).map((p) => p.booster_altitude / 1000), color: '#ffb454', label: 'Booster' },
        ], { title: 'Altitude profile', xlabel: 'Mission time (s)', ylabel: 'Altitude (km)' })}</div>
        <div class="card">${ReportCharts.lineChart([
          { xs: telemetryLog.map((p) => p.time), ys: telemetryLog.map((p) => p.velocity), color: '#38e0ff', label: 'Orbiter / stack' },
          { xs: telemetryLog.filter((p) => p.booster_velocity !== undefined).map((p) => p.time),
            ys: telemetryLog.filter((p) => p.booster_velocity !== undefined).map((p) => p.booster_velocity), color: '#ffb454', label: 'Booster' },
        ], { title: 'Speed profile', xlabel: 'Mission time (s)', ylabel: 'Speed (m/s)' })}</div>
      </div>
      ${phaseTimelineHtml()}
      <div class="grid grid-2" style="margin-top:16px;">
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
      <div class="no-print" style="display:flex;gap:10px;margin-top:16px;">
        <button class="btn btn-sm btn-primary" id="btn-export-report-pdf">Export PDF</button>
        <button class="btn btn-sm" id="btn-export-telemetry-csv">Export Telemetry CSV</button>
        <button class="btn btn-sm" id="btn-export-report-json">Export Report JSON</button>
      </div>`;
    $('#btn-export-report-pdf').addEventListener('click', () => printView('report'));
    $('#btn-export-telemetry-csv').addEventListener('click', () => {
      downloadBlob(`rlv_telemetry_${state.sim.id || 'run'}.csv`, telemetryToCsv(), 'text/csv');
    });
    $('#btn-export-report-json').addEventListener('click', () => {
      downloadBlob(`rlv_report_${state.sim.id || 'run'}.json`, JSON.stringify({ outcomes, final: last }, null, 2), 'application/json');
    });
    saveRunRecord('mission', outcomes, last).catch(() => {});
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
        dispersions: $('#camp-dispersions').value,
        setup: applyGuidance($('#camp-guidance').value || 'heuristic', JSON.parse(JSON.stringify(currentSetup()))),
      };
      await startCampaign(body);
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
          if (r.summary) {
            renderCampaignResults(r.summary, r.results);
            saveCampaignRecord(r.summary, r.results).catch(() => {});
          }
        }
      } catch (e) {
        clearInterval(state.campaign.pollHandle);
        toast('Campaign polling failed: ' + e.message, 'error');
      }
    }, 1500);
  }

  $('#btn-export-campaign-pdf').addEventListener('click', () => printView('campaign'));

  function renderCampaignResults(summary, results) {
    $('#camp-results').style.display = 'block';
    const grid = $('#camp-summary-grid');
    const ci = summary.mission_success_rate_95pct_ci || [0, 0];
    $('#camp-gauge').innerHTML = ReportCharts.gauge(summary.mission_success_rate || 0, ci);
    const col = (k) => results.map((r) => Number(r[k]));
    $('#camp-hist-touchdown').innerHTML = ReportCharts.histogram(col('touchdown_speed_mps'),
      { title: 'Touchdown speed', xlabel: 'm/s', limit: 6 });
    $('#camp-hist-site').innerHTML = ReportCharts.histogram(col('site_error_m').map((x) => Math.min(x, 200)),
      { title: 'Landing site error (clipped at 200 m)', xlabel: 'm', limit: 50 });
    grid.innerHTML = `
      <div class="metric-cell"><div class="k">Runs</div><div class="v">${summary.runs}</div></div>
      <div class="metric-cell"><div class="k">Mission Success Rate</div><div class="v ${summary.mission_success_rate > 0.9 ? 'pass' : 'fail'}">${fmt(summary.mission_success_rate * 100, 1)}%</div></div>
      <div class="metric-cell"><div class="k">95% CI</div><div class="v">[${fmt(ci[0] * 100, 1)}, ${fmt(ci[1] * 100, 1)}]%</div></div>
      <div class="metric-cell"><div class="k">Orbit / Landing Rate</div><div class="v">${fmt(summary.orbit_success_rate * 100, 0)}% / ${fmt(summary.landing_success_rate * 100, 0)}%</div></div>
      <div class="metric-cell" title="One-sided Clopper-Pearson bound: with 90% confidence the true success rate is at least this."><div class="k">Success, 90% lower bound</div><div class="v">${fmt((summary.mission_success_lower_bound_90pct || 0) * 100, 1)}%</div></div>
      <div class="metric-cell" title="Failure-free runs needed to demonstrate 99.73% (3σ) reliability at 90% confidence."><div class="k">Runs to show 99.73% @ 90%</div><div class="v">${summary.zero_failure_runs_for_99_73pct_at_90pct ?? '—'} <span class="unit">no failures</span></div></div>`;
    state.campaign.lastResults = results;
    $('#btn-rerun-failed').style.display = results.some((r) => !r.mission_success) && state.campaign.request ? '' : 'none';
    renderDrivers(summary);

    const failureGrid = $('#camp-failure-grid');
    const failureCounts = summary.failure_criteria_counts || {};
    const failureEntries = Object.entries(failureCounts);
    failureGrid.innerHTML = ReportCharts.barList(failureEntries, summary.runs);

    const metricsTable = $('#camp-metrics-table');
    const metrics = summary.metrics || {};
    const metricRows = Object.entries(metrics).filter(([, v]) => v);
    metricsTable.querySelector('thead').innerHTML =
      '<tr><th>Metric</th><th>Median</th><th>P05</th><th>P95</th><th title="3σ low tail">P0.135</th><th title="3σ high tail">P99.865</th><th>Min</th><th>Max</th></tr>';
    metricsTable.querySelector('tbody').innerHTML = metricRows.length
      ? metricRows.map(([name, s]) => `<tr><td>${escapeHtml(name)}</td><td>${fmt(s.median, 2)}</td><td>${fmt(s.p05, 2)}</td><td>${fmt(s.p95, 2)}</td><td>${fmt(s.p00135, 2)}</td><td>${fmt(s.p99865, 2)}</td><td>${fmt(s.min, 2)}</td><td>${fmt(s.max, 2)}</td></tr>`).join('')
      : '<tr><td colspan="8">No metric data available.</td></tr>';

    const table = $('#camp-table');
    const cols = ['case_id', 'mode', 'mission_success', 'orbit_success', 'landing_success', 'wall_time_s', 'reason', 'replay'];
    table.querySelector('thead').innerHTML = `<tr>${cols.map((c) => `<th>${c === 'reason' ? 'Failure Reason' : c === 'replay' ? '' : c}</th>`).join('')}</tr>`;
    table.querySelector('tbody').innerHTML = results.slice(0, 300).map((row) => `<tr>${cols.map((c) => {
      if (c === 'mission_success' || c === 'orbit_success' || c === 'landing_success') {
        return `<td><span class="badge ${row[c] ? 'ok' : 'bad'}">${row[c] ? 'PASS' : 'FAIL'}</span></td>`;
      }
      if (c === 'wall_time_s') return `<td>${fmt(row[c], 2)}s</td>`;
      if (c === 'replay') {
        return state.campaign.request
          ? `<td><button class="btn btn-sm" data-replay="${escapeHtml(row.case_id)}" title="Fly this case again with live telemetry">Replay</button></td>`
          : '<td></td>';
      }
      if (c === 'reason') {
        if (row.mission_success) return '<td>—</td>';
        const reason = [!row.orbit_success ? row.orbiter_reason : '', !row.landing_success ? row.booster_reason : ''].filter(Boolean).join(' / ');
        return `<td style="max-width:280px;white-space:normal;">${reason ? escapeHtml(reason) : '—'}</td>`;
      }
      return `<td>${escapeHtml(row[c])}</td>`;
    }).join('')}</tr>`).join('');
    $$('[data-replay]', table).forEach((btn) => btn.addEventListener('click', () => replayCase(btn.dataset.replay)));
  }

  // ------------------------------------------------- campaign: drivers & replay
  const DRIVER_METRICS = [
    ['touchdown_speed_mps', 'Touchdown speed', 'm/s'],
    ['site_error_m', 'Pad error', 'm'],
    ['booster_propellant_remaining_kg', 'Booster propellant left', 'kg'],
    ['perigee_altitude_km', 'Perigee', 'km'],
  ];

  function signedBar(value, max, digits) {
    const width = max > 0 ? Math.min(100, (100 * Math.abs(value)) / max) : 0;
    return `<div class="sbar"><div class="sbar-track"><div class="sbar-fill ${value < 0 ? 'neg' : 'pos'}" style="width:${width.toFixed(1)}%"></div></div><span>${value > 0 ? '+' : ''}${fmt(value, digits)}</span></div>`;
  }

  function renderDrivers(summary) {
    const wrap = $('#camp-drivers-wrap');
    const effects = summary.one_sigma_effects;
    const corr = summary.pearson_correlations;
    const source = effects && Object.keys(effects).length ? effects : (corr && Object.keys(corr).length ? corr : null);
    if (!source) { wrap.style.display = 'none'; return; }
    wrap.style.display = 'block';
    const isEffect = source === effects;
    $('#camp-drivers-help').textContent = isEffect
      ? 'Change in each metric per 1σ of each dispersion (one-at-a-time sweep). Longer bars are the dominant drivers.'
      : 'Correlation (−1 to 1) between each dispersion and each metric across the Monte Carlo runs. Values near 0 mean no linear influence.';
    const metrics = DRIVER_METRICS.filter(([key]) => Object.values(source).some((m) => m[key] !== undefined));
    const max = Object.fromEntries(metrics.map(([key]) => [key, Math.max(1e-12, ...Object.values(source).map((m) => Math.abs(m[key] || 0)))]));
    const ranked = Object.entries(source).sort((a, b) => Math.abs(b[1][metrics[0]?.[0]] || 0) - Math.abs(a[1][metrics[0]?.[0]] || 0));
    const table = $('#camp-drivers-table');
    table.querySelector('thead').innerHTML = `<tr><th>Dispersion</th>${metrics.map(([, label, unit]) => `<th>${label}${isEffect ? ` (${unit} per σ)` : ''}</th>`).join('')}</tr>`;
    table.querySelector('tbody').innerHTML = ranked.map(([param, m]) => `<tr><td>${escapeHtml(param.replace(/_/g, ' '))}</td>${metrics.map(([key]) =>
      `<td>${m[key] === undefined ? '—' : signedBar(m[key], isEffect ? max[key] : 1, isEffect ? 2 : 2)}</td>`).join('')}</tr>`).join('');
  }

  async function launchSimulationWith(setup, label) {
    // Shares launchSimulation's UI flow; used to replay one campaign case live.
    state.pendingSetup = setup;
    await launchSimulation();
    if (label) toast(label, 'info');
  }

  function replayCase(caseId) {
    const req = state.campaign.request;
    if (!req) return;
    const setup = JSON.parse(JSON.stringify(req.setup));
    setup.campaign_case = { mode: req.mode, runs: req.runs, seed: req.seed, dispersions: req.dispersions, case_id: caseId };
    setup.realtime_mode = state.realtime;
    launchSimulationWith(setup, `Replaying ${caseId} with live telemetry`);
  }

  async function startCampaign(body) {
    const result = await api('/api/campaigns/start', { method: 'POST', body: JSON.stringify(body) });
    state.campaign.id = result.campaign_id;
    state.campaign.request = body;
    $('#camp-idle').style.display = 'none';
    $('#camp-progress').style.display = 'block';
    $('#camp-results').style.display = 'none';
    toast('Campaign started: ' + result.campaign_id, 'success');
    pollCampaign();
  }

  $('#btn-rerun-failed').addEventListener('click', async () => {
    const req = state.campaign.request;
    const failed = (state.campaign.lastResults || []).filter((r) => !r.mission_success).map((r) => r.case_id);
    if (!req || !failed.length) { toast('No failed cases to re-run', 'info'); return; }
    try {
      await startCampaign({ ...req, case_ids: failed });
    } catch (e) {
      toast('Re-run failed: ' + e.message, 'error');
    }
  });

  // ------------------------------------------------- flight validation view
  $('#btn-run-flight-validation').addEventListener('click', async () => {
    const btn = $('#btn-run-flight-validation');
    btn.disabled = true;
    btn.textContent = 'Flying validation cases…';
    try {
      renderFlightValidation(await api('/api/validation/flights'));
    } catch (e) {
      toast('Flight validation failed: ' + e.message, 'error');
    } finally {
      btn.disabled = false;
      btn.textContent = 'Run comparison';
    }
  });

  function renderFlightValidation(report) {
    $('#val-empty').style.display = 'none';
    $('#val-notes').innerHTML = report.notes.map((n) => `<div>• ${escapeHtml(n)}</div>`).join('')
      + `<div>Source: <a href="${escapeHtml(report.source_url)}" target="_blank" rel="noopener">${escapeHtml(report.source)}</a></div>`;
    $('#val-results').innerHTML = report.flights.map((f, i) => `
      <div class="section-title">${escapeHtml(f.name.replace(/_/g, ' '))}
        <span class="unit">sim payload ${fmt(f.sim_payload_kg, 0)} kg${f.payload_published ? '' : ' (flight payload unpublished)'}
        · ascent RMS error ${fmt(f.altitude_rms_km, 2)} km altitude, ${fmt(f.speed_rms_mps, 0)} m/s speed</span></div>
      <div class="grid grid-2">
        <div class="card" id="val-alt-${i}"></div>
        <div class="card" id="val-spd-${i}"></div>
      </div>
      <div class="card" style="margin-top:12px;overflow:auto;"><table class="data-table"><thead><tr><th>Quantity</th><th>Flight</th><th>Sim</th><th>Difference</th></tr></thead><tbody>
        ${f.metrics.map((m) => `<tr><td>${escapeHtml(m.quantity)}</td><td>${fmt(m.flight, 1)}</td><td>${fmt(m.sim, 1)}</td>
          <td class="${m.difference_pct !== null && Math.abs(m.difference_pct) > 10 ? 'bad-text' : ''}">${m.difference_pct === null ? '—' : `${m.difference_pct > 0 ? '+' : ''}${fmt(m.difference_pct, 1)}%`}</td></tr>`).join('')}
      </tbody></table></div>`).join('');
    report.flights.forEach((f, i) => {
      $(`#val-alt-${i}`).innerHTML = ReportCharts.lineChart([
        { xs: f.real.t, ys: f.real.altitude_km, color: '#38e0ff', label: 'Flight' },
        { xs: f.sim.t, ys: f.sim.altitude_km, color: '#ffb454', label: 'Sim' },
      ], { title: 'Altitude to MECO', xlabel: 'Time (s)', ylabel: 'km' });
      $(`#val-spd-${i}`).innerHTML = ReportCharts.lineChart([
        { xs: f.real.t, ys: f.real.speed_mps, color: '#38e0ff', label: 'Flight' },
        { xs: f.sim.t, ys: f.sim.speed_mps, color: '#ffb454', label: 'Sim' },
      ], { title: 'Surface-relative speed to MECO', xlabel: 'Time (s)', ylabel: 'm/s' });
    });
  }

  // ------------------------------------------------- model & limits view
  const pedigreeBadge = (p) => `<span class="badge ${p >= 3 ? 'ok' : p >= 2 ? '' : 'bad'}">${p}</span>`;
  let modelLoaded = false;

  async function loadModelCard() {
    try {
      const card = await api('/api/model');
      modelLoaded = true;
      $('#model-limits').innerHTML = card.known_limits.map((l) => `<li>${escapeHtml(l)}</li>`).join('');
      $('#model-table tbody').innerHTML = card.models.map((m) => `<tr><td><b>${escapeHtml(m.area)}</b></td><td>${escapeHtml(m.model)}</td>
        <td>${escapeHtml(m.basis)}</td><td>${escapeHtml(m.limit)}</td><td>${pedigreeBadge(m.pedigree)}</td></tr>`).join('');
      const cred = card.credibility;
      $('#model-pedigree-score').textContent = `overall input pedigree ${cred.input_pedigree_score} / 4`;
      $('#model-inputs-table tbody').innerHTML = Object.entries(cred.model_inputs).map(([k, v]) =>
        `<tr><td>${escapeHtml(k.replace(/_/g, ' '))}</td><td>${pedigreeBadge(v.pedigree)}</td><td>${escapeHtml(v.source)}</td></tr>`).join('');
      $('#model-disp-table tbody').innerHTML = Object.entries(cred.dispersions).map(([k, v]) =>
        `<tr><td>${escapeHtml(k.replace(/_/g, ' '))}</td><td>${fmt(v.sigma, 3)}</td><td>${escapeHtml(v.units)}</td><td>${pedigreeBadge(v.pedigree)}</td><td>${escapeHtml(v.source)}</td></tr>`).join('');
    } catch (e) {
      toast('Could not load the model card: ' + e.message, 'error');
    }
  }

  function renderPluginTable() {
    const folder = $('#plugin-folder');
    if (!folder) return;
    folder.textContent = state.pluginFolder || '—';
    $('#plugin-table tbody').innerHTML = state.plugins.length
      ? state.plugins.map((p) => `<tr><td><b>${escapeHtml(p.title)}</b><div class="unit">${escapeHtml(p.name)}</div></td>
          <td style="white-space:normal;">${escapeHtml(p.description || '—')}</td><td>${p.source === 'user' ? 'your folder' : 'bundled example'}</td>
          <td>${p.error ? `<span class="badge bad" title="${escapeHtml(p.error)}">error</span> <span class="bad-text">${escapeHtml(p.error)}</span>` : '<span class="badge ok">ready</span>'}</td></tr>`).join('')
      : '<tr><td colspan="4">No plugins found.</td></tr>';
  }

  $('#btn-plugins-reload').addEventListener('click', async () => {
    await refreshPlugins();
    toast(`${state.plugins.length} plugin(s) found`, 'success');
  });
  $$('.nav-item[data-view=model]').forEach((n) => n.addEventListener('click', () => { if (!modelLoaded) loadModelCard(); }));

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

  // ------------------------------------------------------------- scenarios
  async function refreshScenarios(selectId) {
    try {
      const list = await api('/api/scenarios');
      const sel = $('#scenario-select');
      sel.innerHTML = list.length
        ? list.map((s) => `<option value="${escapeHtml(s.id)}">${escapeHtml(s.name)}</option>`).join('')
        : '<option value="">— none saved —</option>';
      if (selectId) sel.value = selectId;
    } catch (e) { /* storage unavailable: keep the form usable */ }
  }
  $('#btn-scenario-save').addEventListener('click', async () => {
    const name = $('#scenario-name').value.trim();
    if (!name) { toast('Give the scenario a name first', 'error'); return; }
    try {
      const meta = await api('/api/scenarios', { method: 'POST', body: JSON.stringify({ name, setup: currentSetup() }) });
      await refreshScenarios(meta.id);
      toast(`Saved scenario "${meta.name}"`, 'success');
    } catch (e) { toast('Save failed: ' + e.message, 'error'); }
  });
  $('#btn-scenario-load').addEventListener('click', async () => {
    const id = $('#scenario-select').value;
    if (!id) return;
    try {
      const rec = await api(`/api/scenarios/${encodeURIComponent(id)}`);
      populateFromSetupShape(rec.setup);
      $('#scenario-name').value = rec.name;
      refreshPlan();
      toast(`Loaded "${rec.name}"`, 'success');
    } catch (e) { toast('Load failed: ' + e.message, 'error'); }
  });
  $('#btn-scenario-delete').addEventListener('click', async () => {
    const sel = $('#scenario-select');
    if (!sel.value) return;
    const name = sel.selectedOptions[0].text;
    if (!(await confirmModal('Delete scenario?', `"${name}" will be permanently removed.`))) return;
    try {
      await api(`/api/scenarios/${encodeURIComponent(sel.value)}`, { method: 'DELETE' });
      await refreshScenarios();
      toast(`Deleted "${name}"`, 'info');
    } catch (e) { toast('Delete failed: ' + e.message, 'error'); }
  });
  refreshScenarios();

  // ----------------------------------------------------------- run history
  function downsample(arr, n = 600) {
    const step = Math.max(1, Math.ceil(arr.length / n));
    return arr.filter((_, i) => i % step === 0 || i === arr.length - 1);
  }
  function runLabel() {
    const v = state.formValues.vehicle || {}, r = state.formValues.recovery || {};
    const scen = $('#scenario-name').value.trim();
    return `${scen ? scen + ' · ' : ''}${fmt((v.payload_mass || 0) / 1000, 1)} t · ${r.gfold_guidance ? 'G-FOLD' : 'heuristic'}`;
  }
  async function saveRunRecord(kind, outcomes, last) {
    const pts = downsample(telemetryLog);
    const col = (k) => pts.map((p) => (p[k] === undefined ? null : p[k]));
    const a = outcomes.assessment || {};
    const headline = {
      success: !!(outcomes.orbiter_success && outcomes.booster_success),
      orbit: !!outcomes.orbiter_success, landing: !!outcomes.booster_success,
      apogee_km: a.orbit?.apogee_altitude_km, perigee_km: a.orbit?.perigee_altitude_km,
      touchdown_mps: a.landing?.touchdown_speed_mps, site_error_m: a.landing?.site_error_m,
      max_q_pa: a.max_ascent_q_pa, separation_s: a.separation_time_s, duration_s: a.mission_duration_s,
      booster_fuel_pct: last.booster_fuel_remaining,
    };
    await api('/api/runs', { method: 'POST', body: JSON.stringify({
      kind, name: runLabel(), headline,
      payload: { setup: currentSetup(), outcomes,
        series: { time: col('time'), altitude: col('altitude'), velocity: col('velocity'),
          booster_altitude: col('booster_altitude'), booster_velocity: col('booster_velocity') } },
    }) });
  }
  async function saveCampaignRecord(summary, results) {
    const keep = ['mission_success', 'touchdown_speed_mps', 'site_error_m', 'failed_criteria'];
    await api('/api/runs', { method: 'POST', body: JSON.stringify({
      kind: 'campaign', name: `${runLabel()} · ${summary.runs} runs`,
      headline: { runs: summary.runs, success_rate: summary.mission_success_rate,
        ci: summary.mission_success_rate_95pct_ci, orbit_rate: summary.orbit_success_rate,
        landing_rate: summary.landing_success_rate },
      payload: { setup: currentSetup(), summary,
        results: results.map((r) => Object.fromEntries(keep.map((k) => [k, r[k]]))) },
    }) });
  }

  // ---------------------------------------------------------------- compare
  async function refreshCompareLists() {
    try {
      const runs = await api(`/api/runs?kind=${$('#cmp-kind').value}`);
      const opts = runs.length
        ? runs.map((r) => `<option value="${escapeHtml(r.id)}">${escapeHtml(new Date(r.saved_at).toLocaleString())} — ${escapeHtml(r.name)}</option>`).join('')
        : '<option value="">— no saved runs yet —</option>';
      $('#cmp-a').innerHTML = opts; $('#cmp-b').innerHTML = opts;
      if (runs.length > 1) $('#cmp-b').selectedIndex = 1;
    } catch (e) { toast('Could not load run history: ' + e.message, 'error'); }
  }
  $('#cmp-kind').addEventListener('change', refreshCompareLists);
  $$('.nav-item[data-view=compare]').forEach((n) => n.addEventListener('click', refreshCompareLists));

  function flatten(obj, prefix = '') {
    return Object.entries(obj || {}).reduce((acc, [k, v]) => {
      const key = prefix ? `${prefix}.${k}` : k;
      if (v && typeof v === 'object' && !Array.isArray(v)) Object.assign(acc, flatten(v, key));
      else acc[key] = v;
      return acc;
    }, {});
  }
  function setupDiffHtml(a, b) {
    const fa = flatten(a), fb = flatten(b);
    const keys = [...new Set([...Object.keys(fa), ...Object.keys(fb)])]
      .filter((k) => JSON.stringify(fa[k]) !== JSON.stringify(fb[k])).sort();
    const rows = keys.map((k) => `<tr><td>${escapeHtml(k)}</td><td>${escapeHtml(JSON.stringify(fa[k]) ?? '—')}</td><td>${escapeHtml(JSON.stringify(fb[k]) ?? '—')}</td></tr>`).join('');
    return `<div class="card" style="margin-top:16px;"><div class="card-head"><div class="card-title">Setup Differences</div><div class="help-text">${keys.length} field(s) differ</div></div>
      ${keys.length ? `<table class="data-table"><thead><tr><th>Field</th><th>A</th><th>B</th></tr></thead><tbody>${rows}</tbody></table>` : '<div class="help-text">Identical setups.</div>'}</div>`;
  }
  function metricTable(rows) {
    return `<table class="data-table"><thead><tr><th>Metric</th><th><span class="cmp-dot a"></span>A</th><th><span class="cmp-dot b"></span>B</th><th>B − A</th></tr></thead><tbody>${
      rows.map(([label, a, b, d = 1, unit = '']) => {
        const num = typeof a === 'number' && typeof b === 'number';
        const fa = typeof a === 'boolean' ? (a ? 'PASS' : 'FAIL') : fmt(a, d), fb = typeof b === 'boolean' ? (b ? 'PASS' : 'FAIL') : fmt(b, d);
        return `<tr><td>${label}</td><td>${fa}${num ? unit : ''}</td><td>${fb}${num ? unit : ''}</td><td>${num ? (b - a >= 0 ? '+' : '') + fmt(b - a, d) + unit : ''}</td></tr>`;
      }).join('')}</tbody></table>`;
  }
  function seriesOf(rec, key, scale = 1) {
    const s = rec.payload.series || {};
    const xs = [], ys = [];
    (s.time || []).forEach((t, i) => { const v = (s[key] || [])[i]; if (v !== null && v !== undefined) { xs.push(t); ys.push(v * scale); } });
    return { xs, ys };
  }

  $('#btn-compare').addEventListener('click', async () => {
    const ia = $('#cmp-a').value, ib = $('#cmp-b').value;
    if (!ia || !ib) { toast('Save at least one run first', 'error'); return; }
    try {
      const [A, B] = await Promise.all([api(`/api/runs/${encodeURIComponent(ia)}`), api(`/api/runs/${encodeURIComponent(ib)}`)]);
      const out = $('#cmp-results');
      const ha = A.headline || {}, hb = B.headline || {};
      if (A.kind === 'mission') {
        const chart = (key, bkey, scale, title, ylabel) => ReportCharts.lineChart([
          { ...seriesOf(A, key, scale), color: '#38e0ff', label: 'A stack/orbiter' },
          { ...seriesOf(A, bkey, scale), color: '#7c8bff', label: 'A booster' },
          { ...seriesOf(B, key, scale), color: '#ffb454', label: 'B stack/orbiter' },
          { ...seriesOf(B, bkey, scale), color: '#ff5470', label: 'B booster' },
        ], { title, xlabel: 'Mission time (s)', ylabel });
        out.innerHTML = `
          <div class="grid grid-2" style="margin-top:16px;">
            <div class="card">${chart('altitude', 'booster_altitude', 1 / 1000, 'Altitude', 'km')}</div>
            <div class="card">${chart('velocity', 'booster_velocity', 1, 'Speed', 'm/s')}</div>
          </div>
          <div class="card" style="margin-top:16px;"><div class="card-head"><div class="card-title">Key Metrics</div></div>${metricTable([
            ['Mission', ha.success, hb.success], ['Orbit', ha.orbit, hb.orbit], ['Booster landing', ha.landing, hb.landing],
            ['Apogee', ha.apogee_km, hb.apogee_km, 1, ' km'], ['Perigee', ha.perigee_km, hb.perigee_km, 1, ' km'],
            ['Touchdown speed', ha.touchdown_mps, hb.touchdown_mps, 2, ' m/s'], ['Landing error', ha.site_error_m, hb.site_error_m, 1, ' m'],
            ['Max ascent q', ha.max_q_pa, hb.max_q_pa, 0, ' Pa'], ['Stage separation', ha.separation_s, hb.separation_s, 1, ' s'],
            ['Mission duration', ha.duration_s, hb.duration_s, 0, ' s'], ['Booster fuel left', ha.booster_fuel_pct, hb.booster_fuel_pct, 1, ' %'],
          ])}</div>
          ${setupDiffHtml(A.payload.setup, B.payload.setup)}`;
      } else {
        const col = (R, k) => (R.payload.results || []).map((r) => Number(r[k]));
        const failures = (R) => R.payload.summary?.failure_criteria_counts || {};
        const fk = [...new Set([...Object.keys(failures(A)), ...Object.keys(failures(B))])];
        out.innerHTML = `
          <div class="grid grid-2" style="margin-top:16px;">
            <div class="card"><div class="card-title"><span class="cmp-dot a"></span>A</div><div class="camp-gauge">${ReportCharts.gauge(ha.success_rate || 0, ha.ci)}</div></div>
            <div class="card"><div class="card-title"><span class="cmp-dot b"></span>B</div><div class="camp-gauge">${ReportCharts.gauge(hb.success_rate || 0, hb.ci)}</div></div>
          </div>
          <div class="card" style="margin-top:16px;"><div class="card-head"><div class="card-title">Key Metrics</div></div>${metricTable([
            ['Runs', ha.runs, hb.runs, 0], ['Mission success', (ha.success_rate || 0) * 100, (hb.success_rate || 0) * 100, 1, ' %'],
            ['Orbit success', (ha.orbit_rate || 0) * 100, (hb.orbit_rate || 0) * 100, 1, ' %'],
            ['Landing success', (ha.landing_rate || 0) * 100, (hb.landing_rate || 0) * 100, 1, ' %'],
            ...fk.map((k) => [`Failures: ${k.replace(/_/g, ' ')}`, failures(A)[k] || 0, failures(B)[k] || 0, 0]),
          ])}</div>
          <div class="grid grid-2" style="margin-top:16px;">
            <div class="card">${ReportCharts.histogram(col(A, 'touchdown_speed_mps'), { title: 'A — touchdown speed', xlabel: 'm/s', limit: 6 })}</div>
            <div class="card">${ReportCharts.histogram(col(B, 'touchdown_speed_mps'), { title: 'B — touchdown speed', xlabel: 'm/s', limit: 6 })}</div>
          </div>
          ${setupDiffHtml(A.payload.setup, B.payload.setup)}`;
      }
      $('#cmp-empty').style.display = 'none';
      out.style.display = 'block';
    } catch (e) { toast('Compare failed: ' + e.message, 'error'); }
  });
  $('#btn-run-delete').addEventListener('click', async () => {
    const sel = $('#cmp-a');
    if (!sel.value) return;
    if (!(await confirmModal('Delete saved run?', `"${sel.selectedOptions[0].text}" will be permanently removed.`))) return;
    try {
      await api(`/api/runs/${encodeURIComponent(sel.value)}`, { method: 'DELETE' });
      await refreshCompareLists();
      toast('Run deleted', 'info');
    } catch (e) { toast('Delete failed: ' + e.message, 'error'); }
  });
  $('#btn-compare-pdf').addEventListener('click', () => printView('compare'));

  // ------------------------------------------------------- about & license
  async function refreshAbout() {
    try {
      const a = await api('/api/about');
      const l = a.license;
      const badge = $('#lic-badge');
      badge.className = `badge ${l.state === 'licensed' ? 'ok' : l.state === 'trial' ? 'info' : 'bad'}`;
      badge.textContent = l.state === 'licensed' ? 'LICENSED' : l.state === 'trial' ? `TRIAL · ${l.trial_days_left} days left` : 'EVALUATION ENDED';
      const cell = (k, v) => `<div class="metric-cell"><div class="k">${k}</div><div class="v" style="font-size:13px;word-break:break-all;">${escapeHtml(v)}</div></div>`;
      $('#about-grid').innerHTML = cell('Version', `Vyom ${a.version}`)
        + cell('License', l.state === 'licensed' ? `${l.licensee} · ${l.plan} · until ${l.expires}` : (l.problem || (l.enforced ? 'Evaluation' : 'Developer build (not enforced)')))
        + cell('Data folder', a.data_dir);
    } catch (e) { /* older backend */ }
  }
  $('#btn-lic-activate').addEventListener('click', async () => {
    try {
      await api('/api/license', { method: 'POST', body: JSON.stringify({ key: $('#lic-key').value }) });
      $('#lic-key').value = '';
      await refreshAbout();
      toast('License activated', 'success');
    } catch (e) { toast('Activation failed: ' + e.message, 'error'); }
  });
  refreshAbout();

  // ---------------------------------------------------------- aero data import
  function selectAeroDeck(path) {
    const overrides = { ...(state.advancedOverrides || {}) };
    if (path) overrides.aero_deck_path = path; else delete overrides.aero_deck_path;
    state.advancedOverrides = Object.keys(overrides).length ? overrides : null;
  }

  async function refreshAeroDecks(selected) {
    try {
      const decks = await api('/api/aero/decks');
      const sel = $('#aero-deck-select');
      const current = selected ?? (state.advancedOverrides || {}).aero_deck_path ?? '';
      sel.innerHTML = '<option value="">Built-in model</option>' + decks.map((d) =>
        `<option value="${escapeHtml(d.aero_deck_path)}">${escapeHtml(d.name)}${d.user ? '' : ' (bundled)'}</option>`).join('');
      sel.value = current;
    } catch (e) { /* settings page still usable without the list */ }
  }
  $('#aero-deck-select').addEventListener('change', (e) => {
    selectAeroDeck(e.target.value);
    toast(e.target.value ? 'Simulations will use the selected aero deck' : 'Simulations will use the built-in aero model', 'info');
  });

  $('#btn-aero-import').addEventListener('click', async () => {
    const file = $('#aero-import-file').files[0];
    if (!file) { toast('Choose a .csv or .json aero data file first', 'error'); return; }
    const fd = new FormData();
    fd.append('file', file);
    const name = $('#aero-import-name').value.trim();
    try {
      const url = '/api/aero/import' + (name ? `?name=${encodeURIComponent(name)}` : '');
      let r;
      try {
        r = await api(url, { method: 'POST', body: fd });
      } catch (err) {
        // A deck with this name exists and saved scenarios may use it: replace only on confirmation.
        if (err.status !== 409 || !window.confirm(`${err.message}. Replace it?`)) throw err;
        r = await api(url + (name ? '&' : '?') + 'overwrite=true', { method: 'POST', body: fd });
      }
      $('#aero-import-results').style.display = 'block';
      const failed = r.checks.filter((c) => !c.passed);
      $('#aero-import-warnings').innerHTML =
        `<div class="help-text">Imported <b>${escapeHtml(r.name)}</b>: ${r.grid.mach} Mach × ${r.grid.alpha} alpha points, `
        + `${r.coefficients.join(', ')}. Interpolation checks: ${r.checks.length - failed.length}/${r.checks.length} passed.</div>`
        + r.warnings.map((w) => `<div class="help-text" style="color:var(--amber)">⚠ ${escapeHtml(w)}</div>`).join('')
        + failed.map((c) => `<div class="help-text" style="color:var(--red)">✗ ${escapeHtml(c.name)}: ${escapeHtml(c.detail || '')}</div>`).join('');
      const rows = r.comparison;
      $('#aero-import-chart').innerHTML = ReportCharts.lineChart([
        { xs: rows.map((x) => x.mach), ys: rows.map((x) => x.cd_imported), color: '#38e0ff', label: 'Your data' },
        { xs: rows.map((x) => x.mach), ys: rows.map((x) => x.cd_model), color: '#ffb454', label: 'Built-in model' },
      ], { title: 'Drag coefficient at zero angle of attack', xlabel: 'Mach', ylabel: 'CD' });
      const t = $('#aero-import-table');
      t.querySelector('thead').innerHTML = '<tr><th>Mach</th><th>Your CD</th><th>Built-in CD</th><th>Ratio</th></tr>';
      t.querySelector('tbody').innerHTML = rows.map((x) =>
        `<tr><td>${fmt(x.mach, 2)}</td><td>${fmt(x.cd_imported, 3)}</td><td>${fmt(x.cd_model, 3)}</td><td>${fmt(x.ratio, 2)}</td></tr>`).join('');
      await refreshAeroDecks(r.aero_deck_path);
      selectAeroDeck(r.aero_deck_path);
      toast(`Imported ${r.name}; simulations will use it`, 'success');
    } catch (e) {
      toast('Aero import failed: ' + e.message, 'error');
    }
  });
  refreshAeroDecks();

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
    await loadSchemaAndPlugins();
    await loadDefaults();
    setInterval(checkBackend, 15000);
  })();
})();
