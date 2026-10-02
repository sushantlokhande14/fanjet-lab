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

export { G, D2R, isa, DEFAULT_DESIGN, RANGES, K, WING, engineGeometry, DH_WARN, solveEngine };
