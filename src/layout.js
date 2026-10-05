// ============================================================================
// layout.js: where every part of the plane sits, in the body frame.
//   +x forward (nose), +y up, +z right wing. Origin near the wing quarter chord.
// ============================================================================
import { WING, D2R, engineGeometry } from './physics.js';
// Joukowski airfoil (normalised, c0 = 1). Its exact potential flow drives the
// streamlines around the wing, so the rendered wing uses the same shape.
const JK = (() => {
  const ex = 0.1, ey = 0.05;
  const mux = -ex, muy = ey;
  const R = Math.hypot(1 - mux, muy);
  const beta = Math.asin(muy / R);
  const N = 96;
  const raw = [];
  let xLE = 1e9;
  for (let i = 0; i < N; i++) {
    const th = -beta + (2 * Math.PI * i) / N;
    const zx = mux + R * Math.cos(th), zy = muy + R * Math.sin(th);
    const d = zx * zx + zy * zy;
    const X = zx + zx / d, Y = zy - zy / d;
    raw.push([X, Y]);
    if (X < xLE) xLE = X;
  }
  const chord = 2 - xLE;
  // normalised outline: x from 0 (LE) to 1 (TE), y up; ordered TE -> upper -> LE -> lower -> TE
  const pts = raw.map(([X, Y]) => [(X - xLE) / chord, Y / chord]);
  let tmax = 0;
  for (let i = 0; i < N; i++) {
    const [x, y] = pts[i];
    // find lower surface y at same x (crude): max vertical extent
    for (let j = 0; j < N; j++) {
      if (Math.abs(pts[j][0] - x) < 0.01 && pts[j][1] < y) tmax = Math.max(tmax, y - pts[j][1]);
    }
  }
  return { mux, muy, R, beta, xLE, chord, pts, thickness: tmax };
})();

const FUSELAGE = [
  // x, half-width, half-height, centre y
  [4.1, 0.02, 0.02, -0.06],
  [3.92, 0.24, 0.22, -0.06],
  [3.6, 0.42, 0.4, -0.04],
  [3.1, 0.58, 0.58, -0.01],
  [2.4, 0.67, 0.69, 0.01],
  [1.4, 0.69, 0.72, 0.02],
  [0.2, 0.69, 0.72, 0.02],
  [-0.9, 0.66, 0.68, 0.05],
  [-2.0, 0.53, 0.55, 0.13],
  [-3.1, 0.34, 0.38, 0.27],
  [-4.1, 0.16, 0.22, 0.4],
  [-4.55, 0.05, 0.07, 0.45],
];

function fuselageAt(x) {
  const F = FUSELAGE;
  if (x >= F[0][0] || x <= F[F.length - 1][0]) return null;
  for (let i = 0; i < F.length - 1; i++) {
    const a = F[i], b = F[i + 1];
    if (x <= a[0] && x >= b[0]) {
      const t = (a[0] - x) / (a[0] - b[0]);
      // smooth the nose
      return { a: a[1] + (b[1] - a[1]) * t, b: a[2] + (b[2] - a[2]) * t, c: a[3] + (b[3] - a[3]) * t };
    }
  }
  return null;
}

function planeLayout(d) {
  const geo = engineGeometry(d);
  const half = WING.b / 2;
  const wing = {
    half,
    yLE0: 0.78,
    xQC: 0.35,
    dihedral: 1.0 * D2R,
    inc: WING.incidence,
    chord(z) { return WING.cr + (WING.ct - WING.cr) * Math.min(1, Math.abs(z) / half); },
    xLE(z) { return this.xQC + 0.25 * this.chord(z); },
    yLE(z) { return this.yLE0 + Math.abs(z) * Math.tan(this.dihedral); },
  };
  // chord-frame unit vectors in body coordinates (aft along chord, chord-normal up)
  wing.er = [-Math.cos(wing.inc), -Math.sin(wing.inc)];
  wing.en = [-Math.sin(wing.inc), Math.cos(wing.inc)];

  const Rmax = geo.Dmax / 2;
  const n = d.enginesPerWing;
  const z1 = 0.69 + 0.32 + Rmax;
  const sp = Math.max(2 * Rmax + 0.32, 1.12);
  const engines = [];
  for (const side of [1, -1]) {
    for (let i = 0; i < n; i++) {
      const z = side * (z1 + i * sp);
      const c = wing.chord(z);
      const yAxis = wing.yLE(z) - 0.075 * c - 0.11 - Rmax;
      const xLip = wing.xLE(z) + 0.42 * geo.L;
      engines.push({ z, y: yAxis, xLip, side, i, chord: c });
    }
  }
  // tip vortex start points (trailing edge at the tips)
  const ct = wing.chord(half);
  const tipTE = {
    x: wing.xLE(half) - ct * Math.cos(wing.inc),
    y: wing.yLE(half) - ct * Math.sin(wing.inc),
    z: half * 0.985,
  };
  // spanwise station for the wing-section view: clear of the outer engine
  const outer = engines.reduce((m, e) => Math.max(m, e.z), 0);
  const sectionZ = Math.min(half - 0.7, Math.max(4.5, outer + Rmax + 0.4));
  return {
    d, geo, wing, engines, tipTE, sectionZ,
    gearH: 1.25,       // wheels below the fuselage axis
    span: WING.b,
  };
}

// Duct cross-section along the engine (s = metres aft of the intake lip)
function ductProfile(geo) {
  const Df = geo.Df;
  const Rf = Df / 2;
  const st = geo.stages;
  const outer = [[0, Rf * 1.02], [0.3 * geo.inletLen, Rf]];
  const hub = [[0, 0], [Math.max(0.02, geo.inletLen - 0.55 * Df), 0]];
  // spinner: rounded cone up to the first fan hub
  const sp0 = Math.max(0.02, geo.inletLen - 0.55 * Df);
  for (let i = 1; i <= 6; i++) {
    const t = i / 6;
    hub.push([sp0 + (geo.inletLen - 0.04 * Df - sp0) * t, st[0].rh * Math.sqrt(1 - (1 - t) * (1 - t))]);
  }
  for (const s of st) {
    outer.push([s.x, s.R]);
    hub.push([s.x, s.rh]);
    outer.push([s.x + 0.3 * s.D, s.R]);
    hub.push([s.x + 0.3 * s.D, s.rh]);
  }
  const last = st[st.length - 1];
  const noseEnd = geo.L;
  // nozzle: smooth convergence to the exit radius
  for (let i = 1; i <= 6; i++) {
    const t = i / 6;
    const e = 0.5 - 0.5 * Math.cos(Math.PI * t);
    outer.push([geo.fanEnd + (noseEnd - geo.fanEnd) * t, last.R + (geo.Rn - last.R) * e]);
  }
  // tail cone ends just inside the nozzle
  const tcEnd = noseEnd - 0.12 * Df;
  for (let i = 1; i <= 6; i++) {
    const t = i / 6;
    hub.push([geo.fanEnd + (tcEnd - geo.fanEnd) * t, last.rh * Math.cos((Math.PI / 2) * t)]);
  }
  hub.push([noseEnd, 0]);
  return { outer, hub, L: geo.L };
}

function interp(table, s) {
  if (s <= table[0][0]) return table[0][1];
  for (let i = 0; i < table.length - 1; i++) {
    const a = table[i], b = table[i + 1];
    if (s <= b[0]) {
      const t = b[0] > a[0] ? (s - a[0]) / (b[0] - a[0]) : 1;
      return a[1] + (b[1] - a[1]) * t;
    }
  }
  return table[table.length - 1][1];
}

// Nacelle wall as one closed outline (s, r), revolved around the engine axis:
// inner duct wall (nozzle -> intake), rounded lip, outer cowl (intake -> nozzle).
function nacelleOutline(geo) {
  const duct = ductProfile(geo).outer;
  const Rf = geo.Df / 2;
  const Rmax = geo.Dmax / 2;
  const Rin0 = duct[0][1];
  const Rout0 = 1.14 * Rf;
  const cy = 0.5 * (Rin0 + Rout0), rc = 0.5 * (Rout0 - Rin0);
  const L = geo.L;
  const RnOut = geo.Rn + 0.03 * geo.Df + 0.008;
  const pts = [];
  for (let i = duct.length - 1; i >= 0; i--) pts.push([duct[i][0], duct[i][1]]);
  for (let i = 1; i < 12; i++) {
    const a = (Math.PI / 180) * (270 - (180 * i) / 12);
    pts.push([1.5 * rc * Math.cos(a), cy + rc * Math.sin(a)]);
  }
  const out = [[0, Rout0], [0.1 * L, Rout0 + (Rmax - Rout0) * 0.7], [0.3 * L, Rmax], [0.6 * L, Rmax * 0.985]];
  for (let i = 1; i <= 7; i++) {
    const t = i / 7, e = t * t * (3 - 2 * t);
    out.push([0.6 * L + 0.4 * L * t, Rmax * 0.985 + (RnOut - Rmax * 0.985) * e]);
  }
  for (const p of out) pts.push(p);
  return { pts, outer: out, Rmax, Rin0, Rout0, RnOut, lipS: -1.5 * rc };
}


export { JK, FUSELAGE, fuselageAt, planeLayout, ductProfile, interp, nacelleOutline };
