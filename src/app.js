// ============================================================================
// app.js: UI, modes, flight loop, engine diagram, particle worker.
// ============================================================================

import { DEFAULT_DESIGN, RANGES, predictMission, FlightSim, isa, WING, D2R, K, DH_WARN } from './physics.js';
import { planeLayout, ductProfile, nacelleOutline, interp } from './layout.js';
import { FlowField, Particles } from './flow.js';
const $ = (id) => document.getElementById(id);
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
const nf0 = new Intl.NumberFormat('en-US', { maximumFractionDigits: 0 });
const nf1 = new Intl.NumberFormat('en-US', { maximumFractionDigits: 1, minimumFractionDigits: 1 });
const n0 = (v) => nf0.format(Math.round(v));
const kmh = (ms) => n0(ms * 3.6);
const QS = new URLSearchParams(location.search);
const PCOUNT = +(QS.get('particles') || 6000), TRAIL = 22;
const STORE = 'fanjet-design-v2';

const S = {
  mode: 'design',
  cond: 'cruise',
  view: 'overview',
  warp: 'auto',
  warpNow: 1,
  design: loadDesign(),
  pred: null,
  basePred: null,
  layout: null,
  sim: null,
  geoKey: '',
  lastUI: 0,
  lastPanel: 0,
  seedTimer: 0,
  focus: -1,
  tryToken: 0,
};

function loadDesign() {
  try {
    const raw = localStorage.getItem(STORE);
    if (raw) return { ...DEFAULT_DESIGN, ...JSON.parse(raw) };
  } catch (e) { /* storage unavailable */ }
  return { ...DEFAULT_DESIGN };
}
function saveDesign() {
  try { localStorage.setItem(STORE, JSON.stringify(S.design)); } catch (e) { /* ignore */ }
}

// ---------------------------------------------------------------- particle worker
// The particles run in a worker. The single-file build has no separate script
// to point a Worker at, so it inlines the worker source as a text block instead.
function spawnWorker() {
  const inline = document.getElementById('worker-src');
  if (inline) return new Worker(URL.createObjectURL(new Blob([inline.textContent], { type: 'text/javascript' })));
  return new Worker(new URL('./particles.worker.js', import.meta.url), { type: 'module' });
}

class ParticleDriver {
  constructor(count, onSeg) {
    this.count = count; this.onSeg = onSeg;
    this.busy = false; this.pool = [];
    for (let i = 0; i < 3; i++) this.pool.push(new ArrayBuffer(count * 8 * 4));
    this.local = null;
    try {
      if (/noworker/.test(location.search)) throw new Error('worker disabled');
      this.worker = spawnWorker();
      this.worker.onmessage = (e) => this.onMsg(e.data);
      this.worker.onerror = (e) => { e.preventDefault && e.preventDefault(); this.toLocal(); };
    } catch (e) {
      this.worker = null;
    }
  }
  toLocal() {
    if (this.worker) { try { this.worker.terminate(); } catch (e) { /* */ } }
    this.worker = null;
    this.busy = false;
    if (this.lastInit) this.init(...this.lastInit);
  }
  init(design, state, focus) {
    this.lastInit = [design, state, focus];
    if (this.worker) {
      this.worker.postMessage({ type: 'init', design, state, count: this.count, focus });
    } else {
      const flow = new FlowField(planeLayout(design));
      flow.setState(state);
      const parts = new Particles(Math.round(this.count * 0.6));
      parts.focus = focus;
      parts.attach(flow);
      this.local = { flow, parts, buf: new Float32Array(this.count * 8) };
    }
  }
  setState(state, reseed, focus) {
    if (this.lastInit) this.lastInit[1] = state;
    if (this.worker) this.worker.postMessage({ type: 'state', state, reseed, focus });
    else if (this.local) {
      this.local.flow.setState(state);
      const p = this.local.parts;
      if (p.focus !== focus) p.refocus(focus); else if (reseed) p.updateSeeds();
    }
  }
  step(dt) {
    if (this.worker) {
      if (this.busy || !this.pool.length) return;
      const buf = this.pool.pop();
      this.busy = true;
      this.worker.postMessage({ type: 'step', dt, buf }, [buf]);
    } else if (this.local) {
      const b = this.local.buf;
      this.local.parts.step(dt, (i, ax, ay, az, bx, by, bz, sp, fr) => {
        const o = i * 8; b[o] = ax; b[o + 1] = ay; b[o + 2] = az; b[o + 3] = bx; b[o + 4] = by; b[o + 5] = bz; b[o + 6] = sp; b[o + 7] = fr;
      });
      this.onSeg(b, this.local.parts.n);
    }
  }
  onMsg(m) {
    if (m.type === 'seg') {
      this.busy = false;
      if (m.n) this.onSeg(new Float32Array(m.buf), m.n);
      this.pool.push(m.buf);
    } else if (m.type === 'error') {
      console.warn('Particle worker error, running on the main thread instead.', m.message);
      this.toLocal();
    }
  }
}

// ---------------------------------------------------------------- view setup
const canvas = $('gl');
let SCN = null;
try {
  SCN = await import('./scene.js');
} catch (e) {
  console.warn('3D view unavailable', e);
}
const V3 = SCN ? SCN.createView(canvas) : null;
let plane = null, trails = null, world = null, lab = null, rig = null, driver = null;
let pcount = PCOUNT;
if (!V3) {
  $('nogl').hidden = false;
  $('views').hidden = true;
  $('legend').hidden = true;
} else {
  trails = new SCN.TrailRenderer(pcount, TRAIL);
  V3.planeRoot.add(trails.mesh);
  world = new SCN.World(V3.scene);
  lab = new SCN.Lab(V3.scene);
  rig = new SCN.CameraRig(V3.camera, V3.controls);
  driver = new ParticleDriver(pcount, (buf, n) => trails.push(buf, n));
}

// Lower the load on slow graphics: first the pixel density, then the streak count.
const perf = { ema: 1 / 60, t: 0, level: 0 };
function adaptQuality(rawDt) {
  if (!V3) return;
  perf.ema += (Math.min(rawDt, 0.25) - perf.ema) * 0.05;
  perf.t += rawDt;
  if (perf.t < 4 || perf.level >= 3) return;
  if (perf.ema > 0.026) {
    perf.t = 0;
    perf.level++;
    if (perf.level === 1 && V3.renderer.getPixelRatio() > 1) { V3.renderer.setPixelRatio(1); resize(); }
    else setParticleCount(Math.max(1500, Math.round(pcount * 0.55)));
  } else perf.t = 3;
}

function setParticleCount(n) {
  if (n === pcount) return;
  pcount = n;
  V3.planeRoot.remove(trails.mesh);
  trails.mesh.geometry.dispose();
  const xr = isXray();
  trails = new SCN.TrailRenderer(pcount, TRAIL);
  V3.planeRoot.add(trails.mesh);
  if (driver && driver.worker) driver.worker.terminate();
  driver = new ParticleDriver(pcount, (buf, k) => trails.push(buf, k));
  resize();
  const fs = currentFlowState();
  if (fs) driver.init(S.design, fs.msg, S.focus);
  trails.setLook(S.mode === 'fly' ? xr : true, S.mode === 'fly' && !xr ? 0.5 : 0.95, S.mode === 'fly' && !xr ? 1.6 : 2.2);
}

function resize() {
  if (!V3) return;
  const r = $('viewport').getBoundingClientRect();
  const w = Math.max(1, r.width), h = Math.max(1, r.height);
  V3.renderer.setSize(w, h, false);
  V3.camera.aspect = w / h;
  V3.camera.updateProjectionMatrix();
  const pr = V3.renderer.getPixelRatio();
  trails.mat.uniforms.uRes.value.set(w * pr, h * pr);
}
if (V3) {
  new ResizeObserver(resize).observe($('viewport'));
  resize();
}

// ---------------------------------------------------------------- controls
const SLIDERS = {
  fanD: (v) => `${v.toFixed(2)} m`,
  stages: (v) => `${v}`,
  nozzleRatio: (v) => `${Math.round(v * 100)}% (${(v * S.design.fanD).toFixed(2)} m)`,
  enginesPerWing: (v) => `${v} (${2 * v} in all)`,
  powerKW: (v) => `${n0(v)} kW (${n0(v / (2 * S.design.enginesPerWing))} each)`,
  people: (v) => `${v}`,
  batteryKg: (v) => `${n0(v)} kg (${n0((v * S.design.whPerKg) / 1000)} kWh)`,
  whPerKg: (v) => `${n0(v)} Wh/kg`,
  cruiseAltM: (v) => `${n0(v)} m`,
  cruiseKmh: (v) => `${n0(v)} km/h`,
};
const GEOM_KEYS = ['fanD', 'stages', 'nozzleRatio', 'swirl', 'enginesPerWing'];

function setupControls() {
  for (const k of Object.keys(SLIDERS)) {
    const el = $(k);
    const [mn, mx, st] = RANGES[k];
    el.min = mn; el.max = mx; el.step = st;
    el.value = S.design[k];
    el.addEventListener('input', () => {
      S.design[k] = Number(el.value);
      onDesignChange(GEOM_KEYS.includes(k));
    });
  }
  for (const b of $('swirl').querySelectorAll('button')) {
    b.addEventListener('click', () => { S.design.swirl = b.dataset.v; onDesignChange(true); });
  }
  $('reset-design').addEventListener('click', () => { S.design = { ...DEFAULT_DESIGN }; syncControls(); onDesignChange(true); });
  $('go-fly').addEventListener('click', () => setMode('fly'));
  $('tab-design').addEventListener('click', () => setMode('design'));
  $('tab-fly').addEventListener('click', () => setMode('fly'));
  for (const b of $('cond').querySelectorAll('button')) {
    b.addEventListener('click', () => { S.cond = b.dataset.cond; syncCond(); pushFlowState(true); updateEnginePanel(true); });
  }
  for (const b of $('warp').querySelectorAll('button')) {
    b.addEventListener('click', () => { S.warp = b.dataset.warp; for (const o of $('warp').querySelectorAll('button')) o.setAttribute('aria-pressed', String(o === b)); });
  }
  const fc = $('fl-cruise');
  fc.min = RANGES.cruiseKmh[0]; fc.max = RANGES.cruiseKmh[1]; fc.step = RANGES.cruiseKmh[2];
  fc.addEventListener('input', () => {
    $('fl-cruise-out').textContent = `${fc.value} km/h`;
    if (S.sim) S.sim.setCruiseSpeed(Number(fc.value));
  });
  $('fl-restart').addEventListener('click', () => startFlight());
  $('fl-design').addEventListener('click', () => setMode('design'));
  syncControls();
}

function syncControls() {
  for (const k of Object.keys(SLIDERS)) {
    $(k).value = S.design[k];
    $(`${k}-out`).textContent = SLIDERS[k](Number(S.design[k]));
  }
  for (const b of $('swirl').querySelectorAll('button')) b.setAttribute('aria-pressed', String(b.dataset.v === S.design.swirl));
  $('swirl-hint').textContent = {
    none: 'Nothing between them. Each fan adds its spin on top of the last, and the spin is wasted.',
    stators: 'Fixed vanes after each fan turn the spin back into push.',
    contra: 'Every second fan spins the other way and cancels the spin. Works in pairs.',
  }[S.design.swirl];
}

function syncCond() {
  for (const b of $('cond').querySelectorAll('button')) b.setAttribute('aria-pressed', String(b.dataset.cond === S.cond));
}

let predTimer = 0, geomTimer = 0;
function onDesignChange(geom) {
  syncControls();
  saveDesign();
  clearTimeout(predTimer);
  predTimer = setTimeout(() => { recompute(); }, 60);
  if (geom) {
    clearTimeout(geomTimer);
    geomTimer = setTimeout(() => rebuildPlane(), 140);
  }
}

function recompute() {
  S.pred = predictMission(S.design);
  S.layout = planeLayout(S.design);
  updateResults();
  pushFlowState(true);
  updateEnginePanel(true);
  scheduleTries();
}

// ---------------------------------------------------------------- plane (re)build
function rebuildPlane() {
  S.layout = planeLayout(S.design);
  if (!V3) return;
  if (plane) { V3.planeRoot.remove(plane.group); plane.dispose(); }
  plane = new SCN.PlaneModel(S.layout, V3.mats);
  V3.planeRoot.add(plane.group);
  plane.setXray(isXray());
  trails.clear();
  const fs = currentFlowState();
  if (fs) driver.init(S.design, fs.msg, S.focus);
  applyView(S.view, true);
}

// ---------------------------------------------------------------- flow state
function slimEng(e) {
  if (!e) return null;
  return {
    mdot: e.mdot, Ve: e.Ve, swirlExit: e.swirlExit || 0,
    stages: (e.stages || []).map((s) => s && { swirlOut: s.swirlOut, swirlAfter: s.swirlAfter, stall: s.stall, rpm: s.rpm }),
  };
}

function currentFlowState() {
  if (!S.pred) return null;
  if (S.mode === 'fly' && S.sim) {
    const s = S.sim.s;
    const rho = isa(s.h).rho;
    const g0 = s.V > 1 ? (4 * s.L) / (rho * s.V * Math.PI * WING.b) : 0;
    const msg = { V: s.V, alphaWing: s.alpha, alphaBody: s.alpha - WING.incidence, gamma0: Math.max(0, g0), rho, eng: slimEng(s.eng), CL: s.CL };
    return { msg, theta: s.theta, gear: s.gear, flapPos: s.flapPos, eng: s.eng, V: s.V };
  }
  const pr = S.pred;
  if (S.cond === 'takeoff' || !pr.cruise) {
    const e = pr.staticEng;
    const msg = { V: 0, alphaWing: WING.incidence, alphaBody: 0, gamma0: 0, rho: isa(0).rho, eng: slimEng(e), CL: 0 };
    return { msg, theta: 0, gear: 1, flapPos: 0.45, eng: e, V: 0 };
  }
  const c = pr.cruise;
  const aw = WING.alpha0.clean + c.CL / WING.cla;
  const g0 = (4 * pr.ac.W) / (c.rho * c.V * Math.PI * WING.b);
  const msg = { V: c.V, alphaWing: aw, alphaBody: aw - WING.incidence, gamma0: g0, rho: c.rho, eng: slimEng(c.eng), CL: c.CL };
  return { msg, theta: aw - WING.incidence, gear: 0, flapPos: 0, eng: c.eng, V: c.V };
}

let lastSentV = -1;
function pushFlowState(reseed) {
  if (!driver) return;
  const fs = currentFlowState();
  if (!fs) return;
  const dv = Math.abs(fs.V - lastSentV);
  const rs = reseed || dv > Math.max(3, 0.06 * fs.V);
  if (rs) lastSentV = fs.V;
  driver.setState(fs.msg, rs, S.focus);
}

// ---------------------------------------------------------------- modes and views
const VIEWS = {
  design: [
    ['overview', 'Overview'], ['engine', 'Engine'], ['section', 'Wing section'], ['tip', 'Wing tip'],
  ],
  fly: [
    ['chase', 'Chase'], ['side', 'Side'], ['engine', 'Engine'], ['airflow', 'Airflow'],
  ],
};

function isXray() {
  return S.mode === 'design' || S.view === 'engine' || S.view === 'airflow';
}

function buildViewButtons() {
  const wrap = $('views');
  wrap.innerHTML = '';
  for (const [id, label] of VIEWS[S.mode]) {
    const b = document.createElement('button');
    b.id = `view-${S.mode}-${id}`;
    b.textContent = label;
    b.setAttribute('aria-pressed', String(id === S.view));
    b.addEventListener('click', () => applyView(id));
    wrap.appendChild(b);
  }
}

function applyView(id, instant) {
  S.view = id;
  for (const b of $('views').querySelectorAll('button')) b.setAttribute('aria-pressed', String(b.id.endsWith(`-${id}`)));
  const L = S.layout;
  if (!V3 || !L) return;
  const en = L.engines[0];
  const half = L.wing.half;
  let pos, tgt;
  S.focus = -1;
  switch (id) {
    case 'engine':
      pos = [en.xLip + 1.5, en.y + 0.55, en.z + 1.9]; tgt = [en.xLip - L.geo.L * 0.55, en.y, en.z]; S.focus = 0; break;
    case 'tip':
      pos = [-10.5, 1.7, half + 1.9]; tgt = [-2, 0.85, half - 0.6]; S.focus = -2; break;
    case 'section': {
      const zs = L.sectionZ;
      pos = [-0.45, 0.84, zs + 3.9]; tgt = [-0.45, 0.8, zs]; S.focus = -3; break;
    }
    case 'side':
      pos = S.mode === 'fly' ? [-2, 1.5, -24] : [-1, 1.2, -15]; tgt = [-0.6, 0.3, 0]; break;
    case 'chase':
      pos = [-14, 4.2, 8.5]; tgt = [-0.5, 0.4, 0]; break;
    case 'airflow':
    case 'overview':
    default:
      pos = [6.9, 2.7, 7.9]; tgt = [-0.9, 0.3, 0.4];
  }
  const still = window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches;
  if (instant || still) { V3.camera.position.set(...pos); V3.controls.target.set(...tgt); rig.anim = null; }
  else rig.goTo(pos, tgt);
  const xr = isXray();
  if (S.focus !== S.lastFocus) { S.lastFocus = S.focus; trails.clear(); }
  if (plane) { plane.setXray(xr); plane.setSection(id === 'section' ? L.sectionZ : null); }
  updateScenery();
  if (S.mode === 'fly') trails.setLook(xr, xr ? 0.95 : 0.5, xr ? 2.2 : 1.6);
  else trails.setLook(true, 0.95, 2.2);
  trails.mat.uniforms.uCalm.value = id === 'tip' || id === 'section' ? 0.75 : 0.2;
  pushFlowState(true);
}

function updateScenery() {
  if (!V3) return;
  const showWorld = S.mode === 'fly' && !isXray();
  world.setVisible(showWorld);
  lab.setVisible(!showWorld);
}

function setMode(mode) {
  if (mode === S.mode) return;
  S.mode = mode;
  $('tab-design').setAttribute('aria-selected', String(mode === 'design'));
  $('tab-fly').setAttribute('aria-selected', String(mode === 'fly'));
  $('design-panel').hidden = mode !== 'design';
  $('fly-panel').hidden = mode !== 'fly';
  $('cond').hidden = mode !== 'design';
  $('hud').hidden = mode !== 'fly';
  $('warpwrap').hidden = mode !== 'fly';
  $('card').hidden = true;
  S.view = mode === 'fly' ? 'chase' : 'overview';
  buildViewButtons();
  if (mode === 'fly') startFlight();
  else { S.sim = null; if (world) world.setLanding(null); }
  applyView(S.view, true);
  updateEnginePanel(true);
}

// ---------------------------------------------------------------- flight
function startFlight() {
  S.sim = new FlightSim(S.design);
  S.logCount = -1;
  S.lastPhase = 'ready';
  $('fl-cruise').value = S.design.cruiseKmh;
  $('fl-cruise-out').textContent = `${S.design.cruiseKmh} km/h`;
  if (world) world.setLanding(null);
  S.runwayShown = null;
  if (trails) trails.clear();
  const pr = S.sim.pred;
  const msg = pr.ok
    ? `<p>Predicted range about <b>${n0(pr.range / 1000)} km</b>, landing with ${Math.round(K.reserve * 100)}% battery left.</p>`
    : `<p>${pr.notes[0] ? escapeHtml(pr.notes[0].text) : 'This design is unlikely to fly.'} You can still try.</p>`;
  showCard(`<h2>On the runway</h2>${msg}<div class="row"><button class="vbtn primary" id="card-go">Take off</button><button class="vbtn" id="card-back">Change the design</button></div>`);
  $('card-go').addEventListener('click', () => { $('card').hidden = true; S.sim.start(); });
  $('card-back').addEventListener('click', () => setMode('design'));
  renderTrip(true);
  $('fl-log').innerHTML = '';
}

function showCard(html) {
  $('card-inner').innerHTML = html;
  $('card').hidden = false;
}

function escapeHtml(s) { return String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c])); }

const PHASE = {
  ready: 'Ready', roll: 'Takeoff run', climb: 'Climbing', cruise: 'Cruising', descent: 'Descending',
  approach: 'Final approach', flare: 'Flare', rollout: 'Braking', landed: 'Landed', aborted: 'Takeoff abandoned', glide: 'Gliding, battery empty',
};

function autoWarp(s) {
  switch (s.phase) {
    case 'climb': return s.h < 60 ? 1 : s.h < 350 ? 4 : 14;
    case 'cruise': return 45;
    case 'descent': return s.h > 700 ? 22 : 8;
    case 'approach': return s.h > 120 ? 5 : 2;
    case 'glide': return s.h > 300 ? 14 : 3;
    default: return 1;
  }
}

function advanceFlight(dt) {
  const sim = S.sim;
  if (!sim) return;
  const s = sim.s;
  const target = S.warp === 'auto' ? autoWarp(s) : Number(S.warp);
  S.warpNow += (target - S.warpNow) * Math.min(1, dt * 2.5);
  const total = dt * S.warpNow;
  const dtMax = clamp(0.05 * Math.pow(S.warpNow, 0.6), 0.05, 0.45);
  const n = Math.min(40, Math.ceil(total / dtMax));
  for (let i = 0; i < n; i++) sim.step(total / n);
  const rw = s.runway2 != null && !s.fieldLanding ? s.runway2 : null;
  if (world && rw !== S.runwayShown) { S.runwayShown = rw; world.setLanding(rw); }
  if (s.phase !== S.lastPhase) {
    S.lastPhase = s.phase;
    if (s.phase === 'landed') landedCard();
    if (s.phase === 'aborted') abortedCard();
  }
  syncLog();
}

const EVENT_TEXT = {
  roll: 'Brakes off, full power', climb: 'Lift-off, climbing', cruise: 'Levelled off, cruising', descent: 'Heading down',
  approach: 'Final approach', flare: 'Flare', rollout: 'Touchdown', landed: 'Stopped', aborted: 'Takeoff abandoned', glide: 'Gliding',
};

function syncLog() {
  const s = S.sim.s;
  const n = s.events.length + s.notes.length;
  if (n === S.logCount) return;
  S.logCount = n;
  const items = [
    ...s.events.map((e) => ({ t: e.t, text: EVENT_TEXT[e.phase] || e.phase })),
    ...s.notes.filter((x) => x.text !== 'Rotate' && !/heading down/i.test(x.text)).map((x) => ({ t: x.t, text: x.text })),
  ].sort((a, b) => a.t - b.t);
  const ol = $('fl-log');
  ol.innerHTML = '';
  for (const it of items) {
    const li = document.createElement('li');
    li.innerHTML = `<span>${fmtTime(it.t)}</span>${escapeHtml(it.text)}`;
    ol.appendChild(li);
  }
}

function fmtTime(t) {
  const m = Math.floor(t / 60), sec = Math.floor(t % 60);
  return `${m}:${String(sec).padStart(2, '0')}`;
}

function landedCard() {
  const s = S.sim.s, pr = S.sim.pred;
  const field = s.fieldLanding ? ' in a field' : '';
  showCard(`<h2>Landed${field}</h2>
    <div class="stats">
      <div><b>${nf1.format(s.x / 1000)}</b><span>km flown</span></div>
      <div><b>${fmtTime(s.t)}</b><span>in the air</span></div>
      <div><b>${Math.round(S.sim.soc * 100)}%</b><span>battery left</span></div>
    </div>
    <p>${pr.ok ? `The quick estimate said ${n0(pr.range / 1000)} km. The step-by-step flight adds the level-off, the approach and small control losses.` : 'It made it down safely.'}</p>
    <div class="row"><button class="vbtn primary" id="card-again">Fly again</button><button class="vbtn" id="card-design">Change the design</button></div>`);
  $('card-again').addEventListener('click', () => startFlight());
  $('card-design').addEventListener('click', () => setMode('design'));
}

function abortedCard() {
  const s = S.sim.s, pr = S.sim.pred;
  const e = s.eng;
  let why = s.abortReason || 'It could not reach flying speed.';
  if (e && e.T <= 1) {
    why = S.design.swirl === 'none'
      ? 'The engines make no push. With nothing between the fans, each one adds spin on top of the last; the fans stall and the air leaves spinning instead of going backwards.'
      : 'The engines make no push: the fans stall because the nozzle is too small for the push they are asked to make.';
  }
  showCard(`<h2>Couldn&rsquo;t take off</h2><p>${escapeHtml(why)}</p>
    <p>Static push was ${nf1.format((pr.staticThrust || 0) / 1000)} kN for a ${n0(pr.ac.m)} kg plane. A light plane needs roughly a quarter of its weight in push.</p>
    <div class="row"><button class="vbtn primary" id="card-design">Change the design</button></div>`);
  $('card-design').addEventListener('click', () => setMode('design'));
}

// ---------------------------------------------------------------- results panel
function updateResults() {
  const pr = S.pred;
  if (!S.basePred) S.basePred = predictMission(DEFAULT_DESIGN);
  const ac = pr.ac;
  const ok = pr.ok;
  $('r-range').innerHTML = ok ? `${n0(pr.range / 1000)}<small>km</small>` : `0<small>km</small>`;
  $('r-time').textContent = ok ? `${Math.round(pr.time / 60)} min in the air, landing with ${Math.round(K.reserve * 100)}% battery` : 'This design does not complete a flight';
  const base = S.basePred.ok ? S.basePred.range : 0;
  const delta = (ok ? pr.range : 0) - base;
  const dEl = $('r-delta');
  const same = JSON.stringify(S.design) === JSON.stringify(DEFAULT_DESIGN);
  dEl.hidden = same;
  if (!same) {
    dEl.textContent = `${delta >= 0 ? '+' : '−'}${n0(Math.abs(delta) / 1000)} km vs original`;
    dEl.className = `delta ${delta > 500 ? 'up' : delta < -500 ? 'down' : ''}`;
  }
  $('r-to').innerHTML = pr.fail === 'thrust' || !pr.takeoffRoll ? '&ndash;' : `${n0(pr.takeoffRoll)}<small> m</small>`;
  $('r-thrust').innerHTML = `${nf1.format(pr.staticThrust / 1000)}<small> kN (${Math.round(pr.tw * 100)}% of weight)</small>`;
  $('r-pow').innerHTML = pr.cruise ? `${n0(pr.cruise.Pbatt / 1000)}<small> kW</small>` : '&ndash;';
  $('r-eff').innerHTML = pr.cruise ? `${Math.round(pr.cruise.etaOverall * 100)}<small>%</small>` : '&ndash;';
  $('r-vmax').innerHTML = pr.vmax ? `${kmh(pr.vmax)}<small> km/h</small>` : '&ndash;';
  $('r-vbest').innerHTML = pr.bestRangeV ? `${kmh(pr.bestRangeV)}<small> km/h</small>` : '&ndash;';
  $('r-mass').innerHTML = `${n0(ac.m)}<small> kg</small>`;
  $('r-energy').innerHTML = `${n0(ac.energyJ / 3.6e6)}<small> kWh</small>`;
  $('cruise-hint').innerHTML = pr.bestRangeV ? `<span class="speedmark">Best for range: ${kmh(pr.bestRangeV)} km/h.</span> Faster costs battery, slower costs time.` : '';
  const notes = $('notes');
  notes.innerHTML = '';
  const list = pr.notes.slice();
  const e0 = pr.staticEng;
  if (e0 && e0.stall > 0 && e0.T > 1) list.push({ level: e0.stall === 2 ? 'bad' : 'warn', text: e0.stall === 2 ? 'At full power the fans stall: the nozzle is too tight for the push they are asked to make.' : 'At full power the first fan is close to stalling. A slightly bigger nozzle or another fan would ease it.' });
  if (pr.cruise && pr.cruise.eng.swirlLossW > 0.08 * pr.cruise.eng.shaftW) list.push({ level: 'warn', text: `${Math.round((pr.cruise.eng.swirlLossW / pr.cruise.eng.shaftW) * 100)}% of the fan power leaves as spin in the jet.` });
  if (!list.length && pr.cruise) {
    const e = pr.cruise.eng;
    list.push({ level: 'info', text: `Jet leaves at ${kmh(e.Ve)} km/h while the plane flies at ${kmh(e.V0)} km/h. The closer those two are, the less energy is left behind in the jet.` });
  }
  for (const n of list.slice(0, 4)) {
    const li = document.createElement('li');
    li.className = n.level;
    li.textContent = n.text;
    notes.appendChild(li);
  }
}

// ---------------------------------------------------------------- "try this" suggestions
function scheduleTries() {
  const token = ++S.tryToken;
  const d = { ...S.design };
  const cur = S.pred;
  const curR = cur.ok ? cur.range : 0;
  const cands = [];
  const R = RANGES;
  const add = (label, patch) => {
    const nd = { ...d, ...patch };
    for (const k of Object.keys(patch)) if (typeof patch[k] === 'number') nd[k] = clamp(patch[k], R[k][0], R[k][1]);
    if (JSON.stringify(nd) !== JSON.stringify(d)) cands.push({ label, patch: nd });
  };
  if (d.swirl !== 'stators') add('Put stators between the fans', { swirl: 'stators' });
  if (d.swirl === 'contra' && d.stages % 2 === 1 && d.stages < 6) add(`Use ${d.stages + 1} fans so they pair up`, { stages: d.stages + 1 });
  add(`Bigger nozzle (${Math.round(Math.min(1, d.nozzleRatio + 0.1) * 100)}%)`, { nozzleRatio: +(d.nozzleRatio + 0.1).toFixed(2) });
  add(`Smaller nozzle (${Math.round(Math.max(0.3, d.nozzleRatio - 0.1) * 100)}%)`, { nozzleRatio: +(d.nozzleRatio - 0.1).toFixed(2) });
  if (d.stages < 6) add(`One more fan in each engine (${d.stages + 1})`, { stages: d.stages + 1 });
  if (d.stages > 1) add(`One fan fewer (${d.stages - 1})`, { stages: d.stages - 1 });
  add(`Bigger intake fan (${(d.fanD + 0.05).toFixed(2)} m)`, { fanD: +(d.fanD + 0.05).toFixed(2) });
  add(`Smaller intake fan (${(d.fanD - 0.05).toFixed(2)} m)`, { fanD: +(d.fanD - 0.05).toFixed(2) });
  if (cur.bestRangeV && Math.abs(cur.bestRangeV * 3.6 - d.cruiseKmh) > 12) add(`Cruise at ${Math.round(cur.bestRangeV * 3.6 / 5) * 5} km/h`, { cruiseKmh: Math.round(cur.bestRangeV * 3.6 / 5) * 5 });
  if (!cur.ok || cur.tw < 0.22) add(`More motor power (${d.powerKW + 60} kW)`, { powerKW: d.powerKW + 60 });
  if (cur.ok && cur.ac.m < K.mtowKg - 120) add(`100 kg more battery`, { batteryKg: d.batteryKg + 100 });
  const results = [];
  let i = 0;
  const stepFn = () => {
    if (token !== S.tryToken) return;
    if (i < cands.length) {
      const c = cands[i++];
      const p = predictMission(c.patch);
      results.push({ ...c, gain: (p.ok ? p.range : 0) - curR, ok: p.ok });
      setTimeout(stepFn, 0);
    } else renderTries(results);
  };
  setTimeout(stepFn, 250);
}

function renderTries(results) {
  const wrap = $('tries');
  wrap.innerHTML = '';
  const good = results.filter((r) => r.gain > 1500).sort((a, b) => b.gain - a.gain).slice(0, 3);
  if (!good.length) return;
  const lbl = document.createElement('span');
  lbl.className = 'lbl';
  lbl.textContent = 'Try this';
  wrap.appendChild(lbl);
  good.forEach((r, i) => {
    const b = document.createElement('button');
    b.className = 'try';
    b.id = `try-${i}`;
    b.innerHTML = `<span>${escapeHtml(r.label)}</span><b>+${n0(r.gain / 1000)} km</b>`;
    b.addEventListener('click', () => {
      const geom = GEOM_KEYS.some((k) => r.patch[k] !== S.design[k]);
      S.design = { ...r.patch };
      onDesignChange(geom);
    });
    wrap.appendChild(b);
  });
}

// ---------------------------------------------------------------- engine panel
const SVGNS = 'http://www.w3.org/2000/svg';
function el(tag, attrs, parent) {
  const e = document.createElementNS(SVGNS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(e);
  return e;
}

function currentEngine() {
  const fs = currentFlowState();
  if (!fs) return null;
  return { e: fs.eng, V0: fs.V, rho: fs.msg.rho };
}

function updateEnginePanel(force) {
  const now = performance.now();
  if (!force && now - S.lastPanel < 160) return;
  S.lastPanel = now;
  const ce = currentEngine();
  if (!ce || !ce.e || !S.layout) return;
  const idle = S.mode === 'fly' && S.sim && (S.sim.s.phase === 'ready' || S.sim.s.throttle < 0.015);
  drawEngine(ce.e, ce.V0, ce.rho, idle);
  drawFans(ce.e, idle);
  drawEnergy(ce.e, ce.V0, idle);
}

function drawEngine(e, V0, rho, idle) {
  const svg = $('engine-svg');
  const geo = S.layout.geo;
  const cs = getComputedStyle(document.documentElement);
  const C = (n) => cs.getPropertyValue(n).trim();
  svg.innerHTML = '';
  const W = 760, H = 236;
  const nac = nacelleOutline(geo);
  const duct = ductProfile(geo);
  const sc = Math.min(380 / geo.L, 60 / nac.Rmax);
  const engW = geo.L * sc;
  const jetLen = Math.min(260, W - 70 - engW - 30);
  const x0 = 70;
  const cy = 160;
  const X = (s) => x0 + s * sc;
  const Y = (r) => cy - r * sc;
  const Yb = (r) => cy + r * sc;
  const ink2 = C('--ink-2'), ink3 = C('--ink-3'), line = C('--line'), accent = C('--accent'), target = C('--target');
  const defs = el('defs', {}, svg);
  const grad = el('linearGradient', { id: 'jetg', x1: '0', x2: '1', y1: '0', y2: '0' }, defs);
  el('stop', { offset: '0', 'stop-color': C('--e-wake'), 'stop-opacity': '0.5' }, grad);
  el('stop', { offset: '1', 'stop-color': C('--e-wake'), 'stop-opacity': '0' }, grad);
  const exitX = X(geo.L);
  if (e.Ve > 0.5 && !idle) el('path', { d: `M${exitX},${Y(geo.Rn)} L${exitX + jetLen},${Y(geo.Rn * 1.9)} L${exitX + jetLen},${Yb(geo.Rn * 1.9)} L${exitX},${Yb(geo.Rn)} Z`, fill: 'url(#jetg)' }, svg);
  // air coming in
  el('path', { d: `M${x0 - 52},${cy} L${x0 - 18},${cy} M${x0 - 26},${cy - 5} L${x0 - 18},${cy} L${x0 - 26},${cy + 5}`, stroke: ink3, 'stroke-width': '1.5', fill: 'none' }, svg);
  const tin = el('text', { x: x0 - 54, y: cy - 9, 'font-size': '11', fill: ink3 }, svg); tin.textContent = 'air in';
  const pathOf = (pts, mirror) => pts.map((p, i) => `${i ? 'L' : 'M'}${X(p[0]).toFixed(1)},${(mirror ? Yb(p[1]) : Y(p[1])).toFixed(1)}`).join(' ') + ' Z';
  for (const m of [false, true]) el('path', { d: pathOf(nac.pts, m), fill: C('--line-2'), stroke: ink3, 'stroke-width': '1' }, svg);
  const hub = duct.hub;
  const hd = hub.map((p, i) => `${i ? 'L' : 'M'}${X(p[0]).toFixed(1)},${Y(p[1]).toFixed(1)}`).join(' ') + ' ' + hub.slice().reverse().map((p) => `L${X(p[0]).toFixed(1)},${Yb(p[1]).toFixed(1)}`).join(' ') + ' Z';
  el('path', { d: hd, fill: C('--raised'), stroke: ink3, 'stroke-width': '1' }, svg);
  geo.stages.forEach((st, k) => {
    const rec = e.stages[k];
    const col = idle || !rec ? ink3 : rec.stall === 2 ? C('--bad') : rec.stall === 1 ? C('--warn') : accent;
    const xx = X(st.x);
    const bw = Math.max(4, Math.min(7, 0.07 * st.D * sc));
    el('rect', { x: xx - bw / 2, y: Y(st.R * 0.985), width: bw, height: (st.R - st.rh) * sc * 0.985, fill: col, rx: 1.5 }, svg);
    el('rect', { x: xx - bw / 2, y: Yb(st.rh), width: bw, height: (st.R - st.rh) * sc * 0.985, fill: col, rx: 1.5 }, svg);
    if (S.layout.d.swirl === 'stators') {
      const sx = X(st.x + 0.2 * st.D);
      el('rect', { x: sx - 1.5, y: Y(st.R), width: 3, height: (st.R - st.rh) * sc, fill: ink3 }, svg);
      el('rect', { x: sx - 1.5, y: Yb(st.rh), width: 3, height: (st.R - st.rh) * sc, fill: ink3 }, svg);
    }
    const t = el('text', { x: xx, y: Yb(nac.Rmax) + 15, 'text-anchor': 'middle', 'font-size': '11', fill: ink2, 'font-weight': '700' }, svg);
    t.textContent = `${k + 1}`;
    if (rec && !idle) {
      const sw = Math.abs(rec.swirlAfter);
      if (sw > 1.5) {
        const r = clamp(3 + sw * 0.12, 3, 10);
        const sx = X(st.x + 0.3 * st.D) + 2;
        const sy = cy - ((st.R + st.rh) / 2) * sc;
        el('path', { d: `M${sx - r},${sy} A${r},${r} 0 1,1 ${sx},${sy + r}`, fill: 'none', stroke: C('--e-swirl'), 'stroke-width': '1.6' }, svg);
        el('path', { d: `M${sx},${sy + r} l4,-3 M${sx},${sy + r} l4,3`, stroke: C('--e-swirl'), 'stroke-width': '1.6', fill: 'none' }, svg);
      }
    }
  });
  const tf = el('text', { x: X(geo.stages[0].x) - 10, y: Yb(nac.Rmax) + 15, 'text-anchor': 'end', 'font-size': '11', fill: ink3 }, svg);
  tf.textContent = 'fans';
  // air speed along the engine
  const mdot = idle ? 0 : e.mdot || 0;
  const top = 14, bot = Y(nac.Rmax) - 14;
  const Vmax = Math.max(idle ? 0 : e.Ve || 0, V0, 20) * 1.18;
  const Ys = (v) => bot - (v / Vmax) * (bot - top);
  el('line', { x1: x0 - 54, x2: W - 6, y1: bot, y2: bot, stroke: line }, svg);
  const ax = el('text', { x: x0 - 54, y: top + 2, 'font-size': '11', fill: ink3 }, svg); ax.textContent = 'air speed';
  if (V0 > 0.5) {
    el('line', { x1: x0 - 54, x2: W - 6, y1: Ys(V0), y2: Ys(V0), stroke: target, 'stroke-dasharray': '4 4', 'stroke-width': '1.2' }, svg);
    const t = el('text', { x: W - 8, y: Ys(V0) - 5, 'text-anchor': 'end', 'font-size': '11', fill: target }, svg);
    t.textContent = `plane ${kmh(V0)} km/h`;
  }
  if (mdot > 0) {
    const pts = [[x0 - 54, Ys(V0)]];
    const N = 60;
    for (let i = 0; i <= N; i++) {
      const s = (geo.L * i) / N;
      const ro = interp(duct.outer, s), rh = Math.min(interp(duct.hub, s), ro * 0.9);
      pts.push([X(s), Ys(mdot / (rho * Math.PI * (ro * ro - rh * rh)))]);
    }
    for (let i = 1; i <= 12; i++) {
      const dj = ((i / 12) * jetLen) / sc;
      const dec = Math.min(1, Math.pow((4.5 * geo.Dn) / Math.max(dj, 1e-3), 0.9));
      pts.push([exitX + (i / 12) * jetLen, Ys(V0 + (e.Ve - V0) * dec)]);
    }
    el('path', { d: pts.map((p, i) => `${i ? 'L' : 'M'}${p[0].toFixed(1)},${p[1].toFixed(1)}`).join(' '), fill: 'none', stroke: accent, 'stroke-width': '2.2' }, svg);
    el('circle', { cx: exitX, cy: Ys(e.Ve), r: 3.5, fill: accent }, svg);
    const lab = el('text', { x: exitX + 8, y: Math.max(top + 12, Ys(e.Ve) - 7), 'font-size': '12', fill: C('--ink'), 'font-weight': '700' }, svg);
    lab.textContent = `jet ${kmh(e.Ve)} km/h`;
    const fanFace = mdot / (rho * geo.stages[0].A);
    el('circle', { cx: X(geo.stages[0].x), cy: Ys(fanFace), r: 3, fill: accent }, svg);
    const lab3 = el('text', { x: X(geo.stages[0].x), y: Math.min(bot - 5, Ys(fanFace) + 15), 'text-anchor': 'middle', 'font-size': '11', fill: ink2 }, svg);
    lab3.textContent = `${kmh(fanFace)} km/h at fan 1`;
  }
  const T = idle ? 0 : e.T || 0;
  const nE = S.layout.engines.length;
  $('ep-sum').innerHTML = idle
    ? 'Engines at idle'
    : T > 1
      ? `Push <b>${nf1.format(T / 1000)} kN</b> each, <b>${nf1.format((T * nE) / 1000)} kN</b> total &middot; <b>${nf1.format(mdot)} kg/s</b> of air per engine`
      : `<b>No push</b>: the fans can&rsquo;t drive air through this nozzle`;
}

function drawFans(e, idle) {
  const wrap = $('fans');
  wrap.innerHTML = '';
  e.stages.forEach((rec, k) => {
    if (!rec) return;
    const row = document.createElement('div');
    row.className = 'fanrow';
    const surge = !idle && !(e.mdot > 0);
    const st = idle ? ['', 'Idle'] : surge ? ['bad', 'Surging'] : rec.stall === 2 ? ['bad', 'Stalling'] : rec.stall === 1 ? ['warn', 'Near stall'] : ['ok', 'Smooth'];
    const m = idle ? '&ndash;' : surge ? 'no steady airflow' : `${n0(rec.rpm)} rpm &middot; ${nf1.format(rec.powerW / 1000)} kW`;
    row.innerHTML = `<span class="n">Fan ${k + 1}</span><span class="m">${m}</span><span class="chip ${st[0]}">${st[1]}</span>`;
    row.title = `Blade speed ${n0(rec.U)} m/s, spin left behind ${n0(Math.abs(rec.swirlAfter))} m/s, slowing ratio ${rec.dh.toFixed(2)} (below ${DH_WARN} the blades start to stall)`;
    wrap.appendChild(row);
  });
}

function drawEnergy(e, V0, idle) {
  const elecW = idle ? 0 : e.elecW || 0;
  const battW = elecW / K.etaBatt;
  const bar = $('ebar'), key = $('ekey');
  bar.innerHTML = ''; key.innerHTML = '';
  if (battW < 1) { $('en-eff').textContent = 'Engines off'; return; }
  const parts = [
    ['Pushes the plane', e.usefulW, '--e-useful'],
    ['Left in the jet wake', e.wakeLossW, '--e-wake'],
    ['Spin left in the jet', e.swirlLossW, '--e-swirl'],
    ['Fan blade losses', e.fanLossW, '--e-fan'],
    ['Duct and nozzle', e.ductLossW + e.nozzleLossW, '--e-duct'],
    ['Motors and battery', e.motorLossW + (battW - elecW), '--e-motor'],
  ];
  const sum = parts.reduce((a, p) => a + Math.max(0, p[1]), 0) || 1;
  for (const [label, w, c] of parts) {
    const f = Math.max(0, w) / sum;
    const sp = document.createElement('span');
    sp.style.width = `${(f * 100).toFixed(2)}%`;
    sp.style.background = `var(${c})`;
    sp.title = `${label}: ${Math.round(f * 100)}%`;
    bar.appendChild(sp);
    const k = document.createElement('span');
    k.innerHTML = `<i style="background:var(${c})"></i>${label}<b>${Math.round(f * 100)}%</b>`;
    key.appendChild(k);
  }
  $('en-title').textContent = V0 > 0.5 ? 'Where the battery’s power goes' : 'Standing still, the power all goes into the air (no distance covered yet)';
  $('en-eff').textContent = V0 > 0.5 ? `${Math.round((e.usefulW / sum) * 100)}% pushes the plane along` : `${nf1.format((e.T || 0) / Math.max(1, battW / 1000))} N of push per kW`;
}

// ---------------------------------------------------------------- HUD + trip
function updateHud() {
  const sim = S.sim;
  if (!sim) return;
  const s = sim.s;
  $('hud-phase').textContent = PHASE[s.phase] || s.phase;
  $('hud-spd').innerHTML = `${kmh(s.V)}<small>km/h</small>`;
  $('hud-alt').innerHTML = `${n0(s.h)}<small>m</small>`;
  $('hud-bat').innerHTML = `${Math.round(sim.soc * 100)}<small>%</small>`;
  $('hud-dist').innerHTML = `${nf1.format(s.x / 1000)}<small>km</small>`;
  $('fl-phase').textContent = PHASE[s.phase] || s.phase;
  $('fl-dist').innerHTML = `${nf1.format(s.x / 1000)}<small>km</small>`;
  $('fl-time').textContent = fmtTime(s.t);
  $('fl-pow').innerHTML = `${n0(s.Pbatt / 1000)}<small>kW</small>`;
  $('fl-thrust').innerHTML = `${nf1.format(Math.max(0, s.T) / 1000)}<small>kN</small>`;
  $('fl-bat').textContent = `${Math.round(sim.soc * 100)}%`;
  const bb = $('fl-batbar');
  bb.style.width = `${clamp(sim.soc * 100, 0, 100)}%`;
  bb.style.background = sim.soc < K.reserve + 0.03 ? 'var(--bad)' : sim.soc < 0.4 ? 'var(--warn)' : 'var(--good)';
  $('lg-ref').textContent = s.V > 3 ? `white = ${kmh(s.V)} km/h` : 'standing still';
}

function renderTrip(force) {
  const svg = $('trip-svg');
  const sim = S.sim;
  if (!sim) return;
  const cs = getComputedStyle(document.documentElement);
  const C = (n) => cs.getPropertyValue(n).trim();
  const s = sim.s, pr = sim.pred;
  const W = 340, H = 150, l = 46, r = 34, t = 10, b = 22;
  const xmax = Math.max(pr.ok ? pr.range * 1.1 : 0, s.x * 1.05, 20000);
  const hmax = Math.max(S.design.cruiseAltM * 1.25, s.h * 1.1, 500);
  const X = (x) => l + (x / xmax) * (W - l - r);
  const Yh = (h) => H - b - (h / hmax) * (H - t - b);
  const Yb = (soc) => H - b - soc * (H - t - b);
  svg.innerHTML = '';
  // grid
  for (let i = 0; i <= 4; i++) {
    const y = t + (i * (H - t - b)) / 4;
    el('line', { x1: l, x2: W - r, y1: y, y2: y, stroke: C('--line-2') }, svg);
  }
  el('line', { x1: l, x2: W - r, y1: H - b, y2: H - b, stroke: C('--line') }, svg);
  const tick = (x, y, txt, anchor, col) => { const tx = el('text', { x, y, 'text-anchor': anchor, 'font-size': '10', fill: col || C('--ink-3') }, svg); tx.textContent = txt; };
  const step = xmax > 200000 ? 50000 : xmax > 80000 ? 25000 : 10000;
  for (let x = 0; x <= xmax - step * 0.4; x += step) tick(X(x), H - 7, `${x / 1000}`, 'middle');
  tick(W - r + 4, H - 7, 'km', 'start');
  tick(l - 4, Yh(hmax) + 9, `${n0(hmax)} m`, 'end');
  tick(l - 4, H - b, '0', 'end');
  tick(W - r + 4, t + 9, '100%', 'start', C('--good'));
  tick(W - r + 4, Yb(K.reserve) + 3, `${Math.round(K.reserve * 100)}%`, 'start', C('--bad'));
  el('line', { x1: l, x2: W - r, y1: Yb(K.reserve), y2: Yb(K.reserve), stroke: C('--bad'), 'stroke-dasharray': '2 3', 'stroke-opacity': '0.6' }, svg);
  if (pr.ok) {
    el('line', { x1: X(pr.range), x2: X(pr.range), y1: t, y2: H - b, stroke: C('--target'), 'stroke-dasharray': '4 3' }, svg);
    tick(X(pr.range) - 3, t + 9, 'estimate', 'end', C('--target'));
  }
  const log = s.log.concat([{ x: s.x, h: s.h, soc: sim.soc }]);
  if (log.length > 1) {
    const area = `M${X(0)},${H - b} ` + log.map((p) => `L${X(p.x).toFixed(1)},${Yh(p.h).toFixed(1)}`).join(' ') + ` L${X(s.x).toFixed(1)},${H - b} Z`;
    el('path', { d: area, fill: C('--accent'), 'fill-opacity': '0.18', stroke: C('--accent'), 'stroke-width': '1.6' }, svg);
    const bl = log.map((p, i) => `${i ? 'L' : 'M'}${X(p.x).toFixed(1)},${Yb(p.soc).toFixed(1)}`).join(' ');
    el('path', { d: bl, fill: 'none', stroke: C('--good'), 'stroke-width': '1.6' }, svg);
  }
  el('circle', { cx: X(s.x), cy: Yh(s.h), r: 3.5, fill: C('--accent') }, svg);
  tick(l + 4, t + 9, 'height', 'start', C('--accent'));
  tick(l + 48, t + 9, 'battery', 'start', C('--good'));
}

// ---------------------------------------------------------------- main loop
let last = performance.now();
let seedCount = 0;
function frame(now) {
  requestAnimationFrame(frame);
  const raw = Math.max(0, (now - last) / 1000);
  const dt = clamp(raw, 0, 0.05);
  last = now;
  if (!document.hidden) adaptQuality(raw);
  if (S.mode === 'fly' && S.sim) advanceFlight(dt);
  const fs = currentFlowState();
  if (V3 && fs && plane) {
    seedCount++;
    pushFlowState(seedCount % 30 === 0);
    const flowScale = clamp(4.2 / Math.max(fs.V, 18), 0.05, 0.24);
    driver.step(dt * flowScale);
    plane.update({ theta: fs.theta, gear: fs.gear, flapPos: fs.flapPos, eng: fs.eng, visScale: flowScale }, dt);
    V3.planeRoot.rotation.set(0, 0, 0);
    rig.update(dt);
    V3.controls.update();
    if (S.mode === 'fly' && S.sim) {
      const s = S.sim.s;
      world.update(s.x, s.h, S.layout.gearH, V3.camera.position);
      world.cloudBase = clamp(S.design.cruiseAltM - 380, 650, 4000);
    }
    lab.update(V3.camera.position);
    V3.renderer.render(V3.scene, V3.camera);
    if (S.mode === 'design') {
      $('lg-ref').textContent = fs.V > 3 ? `white = ${kmh(fs.V)} km/h` : 'standing still';
      const flowLabel = `Air shown at 1/${Math.round(1 / flowScale)} speed.`;
      const cap = fs.V > 3
        ? `Cruising at <b>${kmh(fs.V)} km/h</b>, ${n0(S.pred.cruiseAlt)} m up. ${flowLabel}`
        : S.cond === 'cruise'
          ? `This design can&rsquo;t get airborne, so it&rsquo;s shown at full power on the ground. ${flowLabel}`
          : `Brakes on, full power. ${flowLabel}`;
      if (cap !== S.capText) { S.capText = cap; $('cond-cap').innerHTML = cap; }
    }
  }
  if (now - S.lastUI > 120) {
    S.lastUI = now;
    if (S.mode === 'fly') { updateHud(); updateEnginePanel(false); }
  }
  if (S.mode === 'fly' && S.sim && (!S.lastTrip || now - S.lastTrip > 500)) { S.lastTrip = now; renderTrip(); }
}

// ---------------------------------------------------------------- boot
setupControls();
buildViewButtons();
S.pred = predictMission(S.design);
S.layout = planeLayout(S.design);
updateResults();
scheduleTries();
syncCond();
rebuildPlane();
updateEnginePanel(true);
setModeInitial();
requestAnimationFrame(frame);



function setModeInitial() {
  $('design-panel').hidden = false;
  $('fly-panel').hidden = true;
  $('hud').hidden = true;
  $('warpwrap').hidden = true;
  updateScenery();
  if (trails) trails.setLook(true, 0.95, 2.2);
}

window.addEventListener('keydown', (ev) => {
  if (ev.key === 'Escape' && !$('card').hidden && S.sim && S.sim.s.phase !== 'ready') $('card').hidden = true;
});
const mq = window.matchMedia ? window.matchMedia('(prefers-color-scheme: dark)') : null;
if (mq && mq.addEventListener) mq.addEventListener('change', () => updateEnginePanel(true));
