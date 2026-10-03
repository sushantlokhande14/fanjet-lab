// ============================================================================
// physics.js: first-order physics for a battery-electric plane driven by
// multi-stage ducted fans (intake fan + a row of fans + converging nozzle).
// SI units throughout. Angles in radians unless the name ends in Deg.
// ============================================================================

const G = 9.80665;
const D2R = Math.PI / 180;
const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);

// International Standard Atmosphere (troposphere)
function isa(h) {
  const hh = clamp(h, -500, 11000);
  const T = 288.15 - 0.0065 * hh;
  const p = 101325 * Math.pow(T / 288.15, 5.25588);
  return { T, p, rho: p / (287.053 * T) };
}

// ---------------------------------------------------------------------------
// Design inputs. Defaults are the idea as described: a big intake fan,
// two more fans behind it, a small nozzle, three engines per wing.
// ---------------------------------------------------------------------------
const DEFAULT_DESIGN = {
  enginesPerWing: 3,
  fanD: 0.5,          // intake fan tip diameter, m
  stages: 3,          // fans in a row inside each engine
  nozzleRatio: 0.55,  // nozzle exit diameter / intake fan diameter
  swirl: 'stators',   // 'none' | 'stators' | 'contra'
  powerKW: 300,       // total motor power, all engines
  batteryKg: 450,
  whPerKg: 250,       // pack level. ~250 is today's best, ~180 in certified trainers
  people: 4,
  cruiseAltM: 1500,
  cruiseKmh: 220,
};

const RANGES = {
  enginesPerWing: [1, 4, 1],
  fanD: [0.3, 0.8, 0.01],
  stages: [1, 6, 1],
  nozzleRatio: [0.3, 1.0, 0.01],
  powerKW: [80, 600, 10],
  batteryKg: [100, 900, 10],
  whPerKg: [150, 500, 10],
  people: [1, 6, 1],
  cruiseAltM: [500, 6000, 100],
  cruiseKmh: [140, 450, 5],
};

const K = {
  airframeKg: 600,     // structure, gear, cabin, systems (no motors, fans, battery, people)
  personKg: 90,        // person + bag
  motorKWperKg: 4.0,   // motor + inverter. Safran ENGINeUS: 3.5 continuous to 5 peak
  hubRatio: 0.38,      // fan hub / tip diameter (the hub holds the motor)
  maxTipSpeed: 240,    // m/s, keeps blade tips well below the speed of sound
  etaMotor: 0.94,      // motor + inverter
  etaBatt: 0.97,       // battery internal losses
  hotelW: 1500,        // avionics, cabin, cooling
  reserve: 0.2,        // land with 20 % battery left
  mtowKg: 2300,
};

// Wing and airframe aerodynamics
const WING = (() => {
  const b = 13.0, cr = 1.75, ct = 1.05;
  const S = (b * (cr + ct)) / 2;
  const AR = (b * b) / S;
  return {
    b, cr, ct, S, AR,
    e: 0.8,
    cla: (2 * Math.PI * AR) / (2 + Math.sqrt(AR * AR + 4)), // Helmbold, per rad
    incidence: 1.5 * D2R,
    alpha0: { clean: -2.5 * D2R, to: -6 * D2R, land: -8 * D2R },
    clmax: { clean: 1.45, to: 1.85, land: 2.05 },
    dcdFlaps: { clean: 0, to: 0.012, land: 0.035 },
    cd0: 0.0215,       // clean airframe, gear up
    dcdGear: 0.012,
  };
})();

// ---------------------------------------------------------------------------
// Engine geometry
// ---------------------------------------------------------------------------
function engineGeometry(d) {
  const N = Math.round(d.stages);
  const Df = d.fanD;
  const Dn = d.nozzleRatio * Df;
  const h = K.hubRatio;
  // The duct narrows gently through the fan row (halfway to the nozzle size),
  // then the nozzle does the rest.
  const Dlast = Df - 0.5 * Math.max(0, Df - Dn);
  const inletLen = 0.62 * Df;
  const stages = [];
  let x = inletLen;
  for (let k = 0; k < N; k++) {
    const t = N > 1 ? k / (N - 1) : 0;
    const D = Df + (Dlast - Df) * t;
    const R = D / 2, rh = h * R;
    const A = Math.PI * (R * R - rh * rh);
    const rrms = Math.sqrt((R * R + rh * rh) / 2);
    stages.push({ k, D, R, rh, A, rrms, x, Umax: (K.maxTipSpeed * rrms) / R });
    x += 0.42 * D;
  }
  const nozzleLen = Math.max(0.35 * Df, 1.2 * Math.max(0, Dlast - Dn) + 0.25 * Df);
  const Rn = Dn / 2;
  return {
    N, Df, Dn, Dlast,
    Dmax: 1.16 * Df,
    Dlip: 1.05 * Df,
    inletLen,
    fanEnd: x,
    nozzleLen,
    L: x + nozzleLen,
    stages,
    Rn,
    An: Math.PI * Rn * Rn,
    rnRms: Rn / Math.SQRT2,
  };
}

// ---------------------------------------------------------------------------
// Fan stage model (Euler turbomachine equation, mean-line)
//   Each fan has its own motor and gets an equal share of the engine's power.
//   Blade exit angle is fixed, so a fan's work is w = U * (Vtheta_out - Vtheta_in).
//   Swirl left behind by one fan reaches the next one unless stators remove it.
//   Stall is flagged with the de Haller ratio W2/W1 (below ~0.7 the blades stall).
// ---------------------------------------------------------------------------
const PHI_D = 0.62;                    // design flow coefficient Vx/U
const PSI_D = 0.4;                     // design work coefficient w/U^2
const TAN_B2 = (1 - PSI_D) / PHI_D;    // front fans: designed for no inlet swirl
const TAN_B2_REAR = 1 / PHI_D;         // contra-rotating rear fans: cancel swirl at design
// Peak stage efficiency. A fan + stator stage loses a little in the stator but
// gets the swirl back as pressure; a lone rotor is cleaner but its swirl is lost.
const ETA_MAX = { stators: 0.89, none: 0.92, contra: 0.91 };
const K_ETA = 0.6;
const K_IN = 0.03;                     // inlet loss, x fan-face dynamic pressure
const K_DUCT = 0.008;                  // per stage: duct wall friction
const CV = 0.985;                      // nozzle velocity coefficient
const SWIRL_GAP_DECAY = 0.95;
const NOZZLE_SWIRL_AMP = 1.15;         // swirl spins up a little as the nozzle narrows
const DH_WARN = 0.7, DH_STALL = 0.62;

// March the air through the engine for a given mass flow. Returns jet speed.
function chain(geo, mode, V0, rho, Pstage, mdot, rec) {
  const st0 = geo.stages[0];
  const Vx0 = mdot / (rho * st0.A);
  const inletL = K_IN * 0.5 * rho * Vx0 * Vx0;
  let ptg = 0.5 * rho * V0 * V0 - inletL; // total pressure above ambient
  let swirl = 0, rPrev = st0.rrms;
  let shaft = 0, fanLoss = 0, ductLossPa = inletL;
  const wReq = Pstage / Math.max(mdot, 1e-6);
  for (let k = 0; k < geo.N; k++) {
    const st = geo.stages[k];
    const Vx = mdot / (rho * st.A);
    if (k > 0) swirl *= (rPrev / st.rrms) * SWIRL_GAP_DECAY; // angular momentum carries over
    const rear = mode === 'contra' && (k & 1) === 1;
    const dir = rear ? -1 : 1;
    const vin = dir * swirl;                 // inlet swirl, in this fan's spin direction
    // Without stators, each fan after the first is pitched for the swirl it
    // receives, so it can still add work, but it adds its swirl on top.
    const adapted = mode === 'none' && k > 0;
    const vinRel = adapted ? 0 : vin;
    const b = Vx * (rear ? TAN_B2_REAR : TAN_B2) + vinRel;
    let U = 0.5 * (b + Math.sqrt(b * b + 4 * wReq));
    let w = wReq;
    let capped = false;
    if (U > st.Umax) { U = st.Umax; w = Math.max(0, U * (U - b)); capped = true; }
    const dV = U > 1e-6 ? w / U : 0;
    const vout = vin + dV;
    const W1 = Math.hypot(Vx, U - vinRel);
    const W2 = Math.hypot(Vx, U - vinRel - dV);
    const dh = W1 > 1e-6 ? W2 / W1 : 1;
    const phi = U > 1e-6 ? Vx / U : 1;
    const etaMax = ETA_MAX[mode] - 0.05 * clamp((0.45 - st.D) / 0.2, 0, 1); // small fans lose a bit
    let eta = etaMax - K_ETA * (phi - PHI_D) * (phi - PHI_D);
    const sf = clamp((dh - 0.55) / 0.15, 0, 1);
    eta = Math.max(0.15, eta * (0.45 + 0.55 * sf));
    ptg += rho * eta * w;
    const dl = K_DUCT * 0.5 * rho * Vx * Vx;
    ptg -= dl; ductLossPa += dl;
    shaft += mdot * w;
    fanLoss += mdot * (1 - eta) * w;
    swirl = dir * vout;
    let swirlAfter = swirl;
    if (mode === 'stators') { swirl = 0; swirlAfter = 0; } // stator turns swirl back into pressure
    rPrev = st.rrms;
    if (rec) {
      rec.stages[k] = {
        U, Vx, phi, dh, eta, w, capped,
        rpm: (U / st.rrms) * 60 / (2 * Math.PI),
        tipSpeed: (U * st.R) / st.rrms,
        powerW: mdot * w,
        swirlIn: vin * dir,      // absolute swirl entering (+ = fan-1 direction)
        swirlOut: dir * vout,    // absolute swirl leaving the rotor
        swirlAfter,              // after the stator (if any)
        dir,
        ptg,                     // total pressure above ambient after this stage
        stall: dh < DH_STALL ? 2 : dh < DH_WARN ? 1 : 0,
      };
    }
  }
  const swirlExit = swirl * Math.min(NOZZLE_SWIRL_AMP, rPrev / geo.rnRms);
  const dyn = ptg - 0.5 * rho * swirlExit * swirlExit;
  const VeI = dyn > 0 ? Math.sqrt((2 * dyn) / rho) : 0;
  const Ve = CV * VeI;
  if (rec) {
    rec.Vx0 = Vx0; rec.swirlExit = swirlExit; rec.VeIdeal = VeI;
    rec.shaftW = shaft; rec.fanLossW = fanLoss;
    rec.ductLossW = (mdot / rho) * ductLossPa;
    rec.swirlLossW = 0.5 * mdot * swirlExit * swirlExit;
    rec.nozzleLossW = 0.5 * mdot * (VeI * VeI - Ve * Ve);
    rec.ptgExit = ptg;
  }
  return Ve;
}

// Find the operating point (mass flow) where the nozzle passes exactly the air
// the fans push. Returns the full engine state. PshaftEngine = shaft power, W.
function solveEngine(geo, mode, V0, rho, PshaftEngine, guess) {
  const Pstage = Math.max(0, PshaftEngine) / geo.N;
  const An = geo.An;
  const g = (m) => rho * An * chain(geo, mode, V0, rho, Pstage, m, null) - m;
  let lo = 0, hi = 0, found = false;
  if (guess > 1e-4) {
    const a = guess * 0.9, b = guess * 1.12;
    if (g(a) > 0 && g(b) < 0) { lo = a; hi = b; found = true; }
  }
  if (!found) {
    hi = rho * geo.stages[0].A * 300;
    for (let i = 0; i < 70; i++) {
      const m = hi * 0.82;
      if (g(m) > 0) { lo = m; found = true; break; }
      hi = m;
    }
  }
  const rec = { stages: new Array(geo.N) };
  if (!found) {
    chain(geo, mode, V0, rho, Pstage, 1e-6, rec);
    return finish(rec, 0, 0, V0, rho, geo);
  }
  for (let it = 0; it < 48 && hi - lo > 1e-5 * hi; it++) {
    const mid = 0.5 * (lo + hi);
    if (g(mid) > 0) lo = mid; else hi = mid;
  }
  const mdot = 0.5 * (lo + hi);
  const Ve = chain(geo, mode, V0, rho, Pstage, mdot, rec);
  return finish(rec, mdot, Ve, V0, rho, geo);
}

function finish(rec, mdot, Ve, V0, rho, geo) {
  const T = mdot * (Ve - V0);
  rec.mdot = mdot;
  rec.Ve = Ve;
  rec.T = T;
  rec.V0 = V0;
  rec.wakeLossW = Ve > V0 ? 0.5 * mdot * (Ve - V0) * (Ve - V0) : 0;
  rec.usefulW = Math.max(0, T * V0);
  rec.elecW = rec.shaftW / K.etaMotor;
  rec.motorLossW = rec.elecW - rec.shaftW;
  rec.etaProp = Ve > V0 && V0 > 0.5 ? (2 * V0) / (Ve + V0) : 0;
  // Air captured by the intake vs. the intake lip area (below ~0.5 the lip spills air)
  const Alip = (Math.PI / 4) * geo.Dlip * geo.Dlip;
  rec.captureArea = V0 > 0.5 ? mdot / (rho * V0) : Infinity;
  rec.mfr = V0 > 0.5 ? rec.captureArea / Alip : 9;
  let worst = 0, worstDh = 9;
  for (const s of rec.stages) { if (s) { worst = Math.max(worst, s.stall); worstDh = Math.min(worstDh, s.dh); } }
  rec.stall = worst;
  rec.worstDh = worstDh;
  return rec;
}

// ---------------------------------------------------------------------------
// Whole aircraft
// ---------------------------------------------------------------------------
function massBreakdown(d, geo) {
  const nEng = 2 * d.enginesPerWing;
  const motors = d.powerKW / K.motorKWperKg;
  let fans = 0;
  for (const s of geo.stages) {
    fans += 14 * s.D * s.D + 0.8;                       // rotor + hub bearings
    if (d.swirl === 'stators') fans += 7 * s.D * s.D;   // stator vanes
  }
  const wet = Math.PI * geo.Dmax * geo.L + Math.PI * 0.85 * geo.Df * geo.L;
  const nacelle = 3.0 * wet + 3.5;                      // composite skins + pylon
  const perEngine = fans + nacelle;
  const engines = nEng * perEngine;
  const people = d.people * K.personKg;
  const total = K.airframeKg + motors + engines + d.batteryKg + people;
  return { airframe: K.airframeKg, motors, engines, perEngine, battery: d.batteryKg, people, total };
}

function buildAircraft(design) {
  const d = { ...DEFAULT_DESIGN, ...design };
  const geo = engineGeometry(d);
  const nEng = 2 * d.enginesPerWing;
  const mass = massBreakdown(d, geo);
  const wetOuter = Math.PI * geo.Dmax * geo.L;
  const nacelleCdA = 0.0044 * wetOuter + 0.004;
  const ac = {
    d, geo, nEng, mass,
    m: mass.total,
    W: mass.total * G,
    PengShaftMax: (d.powerKW * 1000) / nEng,
    nacelleCdA,
    energyJ: d.batteryKg * d.whPerKg * 3600,
  };
  const rho0 = isa(0).rho;
  ac.vsClean = Math.sqrt((2 * ac.W) / (rho0 * WING.S * WING.clmax.clean));
  ac.vsTO = Math.sqrt((2 * ac.W) / (rho0 * WING.S * WING.clmax.to));
  ac.vsLand = Math.sqrt((2 * ac.W) / (rho0 * WING.S * WING.clmax.land));
  ac.vr = 1.1 * ac.vsTO;
  ac.vApp = 1.3 * ac.vsLand;
  return ac;
}

// Spillage drag: a big intake asked to swallow only a little air spills the rest
// around its lip.
function spillDrag(ac, q, mfr) {
  const Alip = (Math.PI / 4) * ac.geo.Dlip * ac.geo.Dlip;
  const x = Math.max(0, 0.55 - mfr);
  return q * Alip * 0.1 * (x * x) / 0.3025;
}

// Ground effect on induced drag (McCormick)
function groundEffect(hAgl) {
  if (hAgl === undefined || hAgl > 3 * WING.b) return 1;
  const r = (16 * Math.max(hAgl + 1.5, 0.5)) / WING.b;
  return (r * r) / (1 + r * r);
}

function aeroForces(ac, V, rho, CL, cfg, gearDown, eng, hAgl) {
  const q = 0.5 * rho * V * V;
  const cdi = (CL * CL) / (Math.PI * WING.e * WING.AR) * groundEffect(hAgl);
  const cd0 = WING.cd0 + WING.dcdFlaps[cfg] + (gearDown ? WING.dcdGear : 0);
  const Dair = q * WING.S * (cd0 + cdi);
  const Dnac = q * ac.nEng * ac.nacelleCdA;
  const Dspill = eng ? ac.nEng * spillDrag(ac, q, eng.mfr) : 0;
  return {
    q, L: q * WING.S * CL,
    D: Dair + Dnac + Dspill,
    Dparts: { wing: q * WING.S * cd0, induced: q * WING.S * cdi, nacelles: Dnac, spill: Dspill },
  };
}

function batteryPower(ac, eng) {
  return (ac.nEng * eng.elecW + K.hotelW) / K.etaBatt;
}

// Throttle (0..1) that gives total thrust Treq at speed V. Returns {throttle, eng}.
function throttleFor(ac, Treq, V, rho, guessMdot) {
  let lo = 0, hi = 1;
  let engHi = solveEngine(ac.geo, ac.d.swirl, V, rho, ac.PengShaftMax, guessMdot);
  if (ac.nEng * engHi.T <= Treq) return { throttle: 1, eng: engHi, saturated: true };
  let eng = engHi;
  for (let i = 0; i < 22; i++) {
    const mid = 0.5 * (lo + hi);
    eng = solveEngine(ac.geo, ac.d.swirl, V, rho, mid * ac.PengShaftMax, eng.mdot);
    if (ac.nEng * eng.T > Treq) hi = mid; else lo = mid;
    if (hi - lo < 0.002) break;
  }
  const th = 0.5 * (lo + hi);
  eng = solveEngine(ac.geo, ac.d.swirl, V, rho, th * ac.PengShaftMax, eng.mdot);
  return { throttle: th, eng, saturated: false };
}

// Steady, level flight at speed V and altitude h.
function levelFlight(ac, V, h, cfg = 'clean', gearDown = false) {
  const { rho } = isa(h);
  const q = 0.5 * rho * V * V;
  const CL = ac.W / (q * WING.S);
  if (CL > WING.clmax[cfg] * 0.95) return { ok: false, reason: 'stall', CL };
  // drag depends on spillage, which depends on the engine: iterate twice
  let a = aeroForces(ac, V, rho, CL, cfg, gearDown, null, 1e9);
  let r = throttleFor(ac, a.D, V, rho);
  a = aeroForces(ac, V, rho, CL, cfg, gearDown, r.eng, 1e9);
  r = throttleFor(ac, a.D, V, rho, r.eng.mdot);
  const P = batteryPower(ac, r.eng);
  return {
    ok: !r.saturated, V, h, rho, CL, D: a.D, Dparts: a.Dparts,
    throttle: r.throttle, eng: r.eng, Pbatt: P,
    LoD: ac.W / a.D,
    etaOverall: (a.D * V) / P,
  };
}

// Fastest steady level speed at altitude h (full power)
function maxLevelSpeed(ac, h) {
  const { rho } = isa(h);
  const excess = (V) => {
    const q = 0.5 * rho * V * V;
    const CL = ac.W / (q * WING.S);
    const eng = solveEngine(ac.geo, ac.d.swirl, V, rho, ac.PengShaftMax);
    const a = aeroForces(ac, V, rho, CL, 'clean', false, eng, 1e9);
    return ac.nEng * eng.T - a.D;
  };
  const vmin = ac.vsClean * Math.sqrt(isa(0).rho / rho) * 1.15;
  let lo = vmin, hi = 170;
  if (excess(lo) <= 0) return 0;
  if (excess(hi) > 0) return hi;
  for (let i = 0; i < 30; i++) {
    const mid = 0.5 * (lo + hi);
    if (excess(mid) > 0) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);
}

// Best climb speed (max rate of climb) at altitude h, clean config
function bestClimb(ac, h) {
  const { rho } = isa(h);
  const vs = ac.vsClean * Math.sqrt(isa(0).rho / rho);
  let best = { roc: -1e9, V: 1.3 * vs };
  for (let i = 0; i <= 8; i++) {
    const V = vs * (1.2 + i * 0.12);
    const q = 0.5 * rho * V * V;
    const CL = ac.W / (q * WING.S);
    const eng = solveEngine(ac.geo, ac.d.swirl, V, rho, ac.PengShaftMax);
    const a = aeroForces(ac, V, rho, CL, 'clean', false, eng, 1e9);
    const roc = ((ac.nEng * eng.T - a.D) * V) / ac.W;
    if (roc > best.roc) best = { roc, V, eng, D: a.D };
  }
  return best;
}

// Speed that flies farthest per unit of battery energy at altitude h
function bestRangeSpeed(ac, h, vmax) {
  const { rho } = isa(h);
  const vs = ac.vsClean * Math.sqrt(isa(0).rho / rho);
  const top = Math.max(vs * 1.4, vmax || 120);
  let best = { V: vs * 1.4, kmPerKwh: 0 };
  const evalV = (V) => {
    const lf = levelFlight(ac, V, h);
    return lf.ok ? V / lf.Pbatt : 0; // m per J
  };
  for (let i = 0; i <= 12; i++) {
    const V = vs * 1.25 + ((top - vs * 1.25) * i) / 12;
    const e = evalV(V);
    if (e > best.kmPerKwh) best = { V, kmPerKwh: e };
  }
  // golden refine
  let a = Math.max(vs * 1.2, best.V - (top - vs) / 12), b = Math.min(top, best.V + (top - vs) / 12);
  for (let i = 0; i < 14; i++) {
    const m1 = a + 0.382 * (b - a), m2 = a + 0.618 * (b - a);
    if (evalV(m1) > evalV(m2)) b = m2; else a = m1;
  }
  const V = 0.5 * (a + b);
  return { V, mPerJ: evalV(V) };
}

// ---------------------------------------------------------------------------
// Fast mission prediction (quasi-steady segments)
// ---------------------------------------------------------------------------
function predictMission(design) {
  const ac = buildAircraft(design);
  const d = ac.d;
  const out = { ac, ok: true, notes: [] };
  const rho0 = isa(0).rho;
  // static thrust
  const eng0 = solveEngine(ac.geo, d.swirl, 0, rho0, ac.PengShaftMax);
  out.staticThrust = ac.nEng * eng0.T;
  out.staticEng = eng0;
  out.tw = out.staticThrust / ac.W;
  if (ac.m > K.mtowKg) out.notes.push({ level: 'bad', text: `Too heavy: ${Math.round(ac.m)} kg is over the ${K.mtowKg} kg limit for this airframe.` });

  // --- takeoff roll
  let V = 0, x = 0, t = 0, E = 0, guess = eng0.mdot;
  const mu = 0.03;
  const aG = WING.incidence;
  const CLg = WING.cla * (aG - WING.alpha0.to);
  let rolled = false;
  while (t < 240) {
    const dt = 0.25;
    const eng = solveEngine(ac.geo, d.swirl, V, rho0, ac.PengShaftMax, guess);
    guess = eng.mdot;
    const a = aeroForces(ac, V, rho0, CLg, 'to', true, eng, 0);
    const N = Math.max(0, ac.W - a.L);
    const acc = (ac.nEng * eng.T - a.D - mu * N) / ac.m;
    E += batteryPower(ac, eng) * dt;
    V = Math.max(0, V + acc * dt); x += V * dt; t += dt;
    if (V >= ac.vr) { rolled = true; break; }
    if (acc < 0.05 && t > 5) break;
    if (x > 4000) break;
  }
  out.takeoffRoll = x;
  out.takeoffTime = t;
  if (!rolled) {
    out.ok = false;
    out.fail = x > 4000 ? 'runway' : 'thrust';
    out.notes.unshift({ level: 'bad', text: x > 4000 ? 'Can’t take off: it would need more than 4 km of runway.' : 'Can’t take off: the engines can’t push hard enough to reach flying speed.' });
    out.range = 0;
    return out;
  }
  if (x > 1200) out.notes.push({ level: 'warn', text: `Long takeoff run (${Math.round(x)} m). Small airfields have 600–1,000 m.` });
  let dist = x; // count distance from brake release
  let time = t;
  // short transition / initial climb to 15 m at V2 (approximation)
  // --- climb in bands
  const hc = d.cruiseAltM;
  let h = 0;
  const climbLog = [];
  let ceilingHit = false;
  const band = 100;
  while (h < hc - 1) {
    const hm = h + band / 2;
    const bc = bestClimb(ac, hm);
    if (bc.roc < 1.0) { ceilingHit = true; break; }
    const dh = Math.min(band, hc - h);
    const dt = dh / bc.roc;
    const P = batteryPower(ac, bc.eng);
    E += P * dt;
    time += dt;
    dist += Math.sqrt(Math.max(0, bc.V * bc.V - bc.roc * bc.roc)) * dt;
    h += dh;
    climbLog.push({ h, V: bc.V, roc: bc.roc });
  }
  out.climbTime = time - t;
  out.climbLog = climbLog;
  out.cruiseAlt = h;
  out.avgRoc = h > 0 ? h / out.climbTime : 0;
  if (ceilingHit && h < 150) {
    out.ok = false;
    out.fail = 'climb';
    out.notes.unshift({ level: 'bad', text: 'It lifts off but can barely climb. The engines only just beat the drag.' });
    out.range = 0;
    out.climbE = E;
    return out;
  }
  if (ceilingHit) {
    out.notes.push({ level: 'warn', text: `Can’t climb to ${hc.toLocaleString()} m. It levels off at about ${Math.round(h / 10) * 10} m.` });
  }

  // --- cruise
  const vmax = maxLevelSpeed(ac, h);
  out.vmax = vmax;
  if (!(vmax > 0)) {
    out.ok = false;
    out.fail = 'climb';
    out.notes.unshift({ level: 'bad', text: 'It can’t hold level flight at any speed.' });
    out.range = 0;
    out.climbE = E;
    return out;
  }
  let Vc = d.cruiseKmh / 3.6;
  if (vmax > 0 && Vc > vmax * 0.995) {
    out.notes.push({ level: 'warn', text: `Can’t reach ${d.cruiseKmh} km/h. Top speed at this height is ${Math.round(vmax * 3.6)} km/h.` });
    Vc = vmax * 0.995;
  }
  const vsAlt = ac.vsClean * Math.sqrt(rho0 / isa(h).rho);
  if (Vc < vsAlt * 1.25) Vc = vsAlt * 1.25;
  const cr = levelFlight(ac, Vc, h);
  out.cruise = cr;
  out.cruiseV = Vc;
  const br = bestRangeSpeed(ac, h, vmax);
  out.bestRangeV = br.V;

  // --- descent at idle, speed ~ cruise, from h to 300 m, then 3-degree approach
  const descent = descentPlan(ac, h, Vc);
  out.descent = descent;
  const Euse = ac.energyJ * (1 - K.reserve);
  const Ecruise = Euse - E - descent.E;
  if (Ecruise <= 0) {
    out.ok = false;
    out.fail = 'battery';
    out.notes.unshift({ level: 'bad', text: 'Battery runs out before the plane can climb and come back down. Add battery or cut weight.' });
    out.range = 0;
    out.climbE = E;
    return out;
  }
  const tc = Ecruise / cr.Pbatt;
  out.cruiseTime = tc;
  out.cruiseDist = Vc * tc;
  out.climbE = E;
  dist += out.cruiseDist + descent.dist;
  time += tc + descent.time;
  out.range = dist;
  out.time = time;
  out.Edescent = descent.E;
  out.descentStartE = descent.E * 1.12 + ac.energyJ * K.reserve; // when to start down
  return out;
}

function descentPlan(ac, h, Vc) {
  let E = 0, dist = 0, time = 0;
  let hh = h;
  const idle = 0.04;
  const band = 100;
  const Vd = Math.min(Vc, 1.9 * ac.vsClean);
  while (hh > 300) {
    const hm = hh - band / 2;
    const { rho } = isa(hm);
    const q = 0.5 * rho * Vd * Vd;
    const CL = ac.W / (q * WING.S);
    const eng = solveEngine(ac.geo, ac.d.swirl, Vd, rho, idle * ac.PengShaftMax);
    const a = aeroForces(ac, Vd, rho, CL, 'clean', false, eng, 1e9);
    const sinG = (ac.nEng * eng.T - a.D) / ac.W; // negative
    const rod = Math.max(1, -sinG * Vd);
    const dh = Math.min(band, hh - 300);
    const dt = dh / rod;
    E += batteryPower(ac, eng) * dt;
    time += dt;
    dist += Vd * dt;
    hh -= dh;
  }
  // approach: 3 degree glide path at Vapp, flaps + gear
  const Va = ac.vApp;
  const { rho } = isa(150);
  const q = 0.5 * rho * Va * Va;
  const CL = ac.W / (q * WING.S);
  const g3 = 3 * D2R;
  const a0 = aeroForces(ac, Va, rho, CL, 'land', true, null, 1e9);
  const Treq = Math.max(0, a0.D - ac.W * Math.sin(g3));
  const r = throttleFor(ac, Treq, Va, rho);
  const ta = Math.min(hh, 300) / (Va * Math.sin(g3));
  E += batteryPower(ac, r.eng) * ta;
  time += ta;
  dist += Math.min(hh, 300) / Math.tan(g3);
  return { E, dist, time, Vd };
}

export { G, D2R, isa, DEFAULT_DESIGN, RANGES, K, WING, engineGeometry, DH_WARN, solveEngine, massBreakdown, buildAircraft, aeroForces, batteryPower, throttleFor, levelFlight, maxLevelSpeed, bestClimb, bestRangeSpeed, predictMission, descentPlan };
