// ============================================================================
// flow.js: approximate air flow around the plane, for the particle streaks.
//   wing: exact 2D potential flow round a Joukowski section, scaled to the
//         local lift, plus the two rolled-up tip vortices (Biot-Savart)
//   fuselage: slender-body source line
//   engines: intake sink, lip and nozzle rings, then the duct itself
//            (continuity sets the speed, swirl from each fan), then the jet
// All in the plane's body frame (+x forward, +y up, +z right).
// ============================================================================
import { JK, FUSELAGE, fuselageAt, ductProfile, interp, nacelleOutline } from './layout.js';
import { WING, D2R } from './physics.js';
const TAU = Math.PI * 2;
const sstep = (a, b, x) => { const t = Math.min(1, Math.max(0, (x - a) / (b - a))); return t * t * (3 - 2 * t); };

class FlowField {
  constructor(layout) {
    this.L = layout;
    this.geo = layout.geo;
    this.buildEngineTables();
    this.buildFuselageSources();
    this.state = null;
    this.vortex = [[], []];
  }

  // --- per-frame flight condition
  setState(st) {
    // st: { V, alphaWing, alphaBody, gamma0, rho, eng }
    this.state = st;
    const L = this.L;
    const V = st.V;
    this.uInf = [-V * Math.cos(st.alphaBody), V * Math.sin(st.alphaBody), 0];
    // tip vortex polylines
    const half = L.wing.half;
    const zr = (Math.PI / 4) * half;
    const down = Math.min(0.12, Math.max(0, st.CL || 0) / (Math.PI * WING.AR) * 2);
    // near the plane the core stays close to the tip; it drifts inboard to
    // pi/4 of the semi-span only far downstream
    const ds = [0, 1.5, 5, 12, 30, 600];
    for (let side = 0; side < 2; side++) {
      const sg = side === 0 ? 1 : -1;
      const pts = [];
      for (const d of ds) {
        const near = 0.22 * (1 - Math.exp(-d / 4));
        const far = (L.tipTE.z - 0.22 - zr) * (1 - Math.exp(-d / (4 * L.span)));
        pts.push([
          L.tipTE.x - d * Math.cos(st.alphaBody),
          L.tipTE.y + d * Math.sin(st.alphaBody) - down * d * 0.35,
          sg * (L.tipTE.z - near - far),
        ]);
      }
      this.vortex[side] = pts;
    }
    const e = st.eng;
    this.mdotQ = e && e.mdot > 0 ? e.mdot / st.rho : 0; // m^3/s through each engine
    this.Ve = e ? e.Ve : 0;
    this.swirlExit = e ? e.swirlExit || 0 : 0;
    // duct station speeds
    this.buildDuctStations();
  }

  // ---------------------------------------------------------------- engines
  buildEngineTables() {
    const geo = this.geo;
    const D = geo.Df;
    const nac = nacelleOutline(geo);
    this.nac = nac;
    this.duct = ductProfile(geo);
    const s0 = -2.4 * D, s1 = geo.L + 1.0 * D;
    const r1 = 2.4 * D;
    const ns = 120, nr = 64;
    const ds = (s1 - s0) / (ns - 1), dr = r1 / (nr - 1);
    const Rlip = 0.5 * (nac.Rin0 + nac.Rout0);
    const eps2 = (0.05 * D) ** 2;
    // intake: a sink spread over the intake disk (finite speed at the centre)
    const sinkPts = [];
    const a = nac.Rin0 * 0.92, sFace = 0.12 * geo.inletLen;
    sinkPts.push([0, 1]);
    for (const [rr, n] of [[0.35, 6], [0.62, 10], [0.86, 14]]) {
      for (let k = 0; k < n; k++) sinkPts.push([rr * a, n, (TAU * k) / n]);
    }
    const wsum = 1 + 6 + 10 + 14;
    const nRing = 24;
    const fields = new Float32Array(ns * nr * 6); // sink(us,ur) lipRing(us,ur) nozzleRing(us,ur)
    for (let i = 0; i < ns; i++) {
      const s = s0 + i * ds;
      for (let j = 0; j < nr; j++) {
        const r = j * dr;
        const o = (i * nr + j) * 6;
        let us = 0, ur = 0;
        for (const p of sinkPts) {
          const pr = p[0], th = p.length > 2 ? p[2] : 0;
          const dx = s - sFace, dy = r - pr * Math.cos(th), dz = -pr * Math.sin(th);
          const d2 = dx * dx + dy * dy + dz * dz + eps2, inv = 1 / (4 * Math.PI * d2 * Math.sqrt(d2) * wsum);
          us -= dx * inv; ur -= dy * inv;
        }
        fields[o] = us; fields[o + 1] = ur;
        let ls = 0, lr = 0, ns_ = 0, nr_ = 0;
        for (let k = 0; k < nRing; k++) {
          const t = (TAU * (k + 0.5)) / nRing;
          const ct = Math.cos(t), stt = Math.sin(t);
          {
            const dx = s - nac.lipS * 0.6, dy = r - Rlip * ct, dz = -Rlip * stt;
            const d2 = dx * dx + dy * dy + dz * dz + eps2, inv = 1 / (4 * Math.PI * d2 * Math.sqrt(d2) * nRing);
            ls += dx * inv; lr += dy * inv;
          }
          {
            const dx = s - geo.L * 0.97, dy = r - nac.RnOut * ct, dz = -nac.RnOut * stt;
            const d2 = dx * dx + dy * dy + dz * dz + eps2, inv = 1 / (4 * Math.PI * d2 * Math.sqrt(d2) * nRing);
            ns_ -= dx * inv; nr_ -= dy * inv;
          }
        }
        fields[o + 2] = ls; fields[o + 3] = lr;
        fields[o + 4] = ns_; fields[o + 5] = nr_;
      }
    }
    this.tab = { s0, s1, r1, ns, nr, ds, dr, fields };
    this.Aface = Math.PI * a * a;
  }

  buildDuctStations() {
    const geo = this.geo, st = this.state;
    const e = st.eng;
    const n = 80;
    const L = geo.L;
    const S = new Float32Array(n), Ro = new Float32Array(n), Rh = new Float32Array(n), Vx = new Float32Array(n), Vt = new Float32Array(n), Stall = new Uint8Array(n);
    const rho = st.rho;
    const mdot = e ? e.mdot : 0;
    for (let i = 0; i < n; i++) {
      const s = (L * i) / (n - 1);
      S[i] = s;
      Ro[i] = interp(this.duct.outer, s);
      Rh[i] = Math.min(interp(this.duct.hub, s), Ro[i] * 0.9);
      const A = Math.max(1e-3, Math.PI * (Ro[i] * Ro[i] - Rh[i] * Rh[i]));
      Vx[i] = mdot / (rho * A);
      // swirl from the fan just upstream
      let vt = 0, stall = 0;
      if (e && e.stages) {
        for (let k = 0; k < geo.N; k++) {
          const sk = geo.stages[k];
          const rec = e.stages[k];
          if (!rec) continue;
          if (s >= sk.x + 0.06 * sk.D) {
            vt = rec.swirlOut;
            if (s >= sk.x + 0.24 * sk.D) vt = rec.swirlAfter;
            stall = Math.abs(s - sk.x) < 0.35 * sk.D ? rec.stall : stall;
          } else if (Math.abs(s - sk.x) < 0.2 * sk.D) stall = Math.max(stall, rec.stall);
        }
        // angular momentum: swirl spins up as the duct narrows
        if (s > geo.fanEnd && vt !== 0) {
          const last = geo.stages[geo.N - 1];
          const rr = Math.sqrt(0.5 * (Ro[i] * Ro[i] + Rh[i] * Rh[i]));
          vt *= Math.min(1.15, last.rrms / Math.max(rr, 1e-3));
        }
      }
      Vt[i] = vt;
      Stall[i] = stall;
    }
    this.ds = { n, S, Ro, Rh, Vx, Vt, Stall, L };
  }

  ductAt(s, out) {
    const d = this.ds;
    const f = Math.min(d.n - 1.001, Math.max(0, (s / d.L) * (d.n - 1)));
    const i = f | 0, t = f - i;
    out.ro = d.Ro[i] + (d.Ro[i + 1] - d.Ro[i]) * t;
    out.rh = d.Rh[i] + (d.Rh[i + 1] - d.Rh[i]) * t;
    out.vx = d.Vx[i] + (d.Vx[i + 1] - d.Vx[i]) * t;
    out.vt = d.Vt[t < 0.5 ? i : i + 1];
    out.stall = d.Stall[t < 0.5 ? i : i + 1];
    return out;
  }

  // ---------------------------------------------------------------- fuselage
  buildFuselageSources() {
    const pts = [];
    const n = 18;
    const x0 = FUSELAGE[0][0] - 0.01, x1 = FUSELAGE[FUSELAGE.length - 1][0] + 0.01;
    const area = (x) => { const f = fuselageAt(x); return f ? Math.PI * f.a * f.b : 0; };
    for (let i = 0; i < n; i++) {
      const xa = x0 + ((x1 - x0) * i) / n, xb = x0 + ((x1 - x0) * (i + 1)) / n;
      const xm = 0.5 * (xa + xb);
      const f = fuselageAt(xm);
      pts.push({ x: xm, y: f ? f.c : 0, dS: area(xb) - area(xa) });
    }
    this.fusSrc = pts;
  }

  // ---------------------------------------------------------------- the field
  // Velocity (body frame) for a free particle. Returns 0 normally, -1 if the
  // point is inside a solid part.
  velocity(px, py, pz, out) {
    const st = this.state;
    const V = st.V;
    let ux = this.uInf[0], uy = this.uInf[1], uz = 0;
    const L = this.L;
    // ---- wing section (2D potential flow, perturbation part)
    const az = Math.abs(pz);
    const W = L.wing;
    if (az < W.half + 0.4 && V > 0.5) {
      const c = W.chord(pz);
      const dx = px - W.xLE(pz), dy = py - W.yLE(pz);
      const xi = dx * W.er[0] + dy * W.er[1];
      const eta = dx * W.en[0] + dy * W.en[1];
      const reach = 4.5 * c;
      if (xi > -reach && xi < c + reach && eta > -reach && eta < reach) {
        const s = c / JK.chord;
        const zx = JK.xLE + xi / s, zy = eta / s;
        const wx = zx * zx - zy * zy - 4, wy = 2 * zx * zy;
        const m = Math.hypot(wx, wy);
        const sx = Math.sqrt(Math.max(0, 0.5 * (m + wx)));
        let sy = Math.sqrt(Math.max(0, 0.5 * (m - wx)));
        if (wy < 0) sy = -sy;
        const ax = 0.5 * (zx + sx), ay = 0.5 * (zy + sy);
        const bx = 0.5 * (zx - sx), by = 0.5 * (zy - sy);
        const da = Math.hypot(ax - JK.mux, ay - JK.muy), db = Math.hypot(bx - JK.mux, by - JK.muy);
        let qx, qy, zetx, zety, dist;
        if (da >= db) { zetx = ax; zety = ay; dist = da; } else { zetx = bx; zety = by; dist = db; }
        const tipFade = 1 - sstep(W.half - 0.35, W.half + 0.25, az);
        if (dist < JK.R * 1.0 && tipFade > 0.5 && az < W.half) return -1; // inside the wing
        qx = zetx - JK.mux; qy = zety - JK.muy;
        const ell = az < W.half ? Math.sqrt(1 - (az / W.half) ** 2) : 0;
        const GammaJ = (st.gamma0 * ell) / s;
        const arg = Math.max(-0.95, Math.min(0.95, GammaJ / (2 * TAU * V * JK.R)));
        const aEff = Math.asin(arg) - JK.beta;
        const ca = Math.cos(aEff), sa = Math.sin(aEff);
        const q2x = qx * qx - qy * qy, q2y = 2 * qx * qy, q2m = q2x * q2x + q2y * q2y;
        const iq2x = q2x / q2m, iq2y = -q2y / q2m;
        const R2 = JK.R * JK.R;
        const tx = R2 * (ca * iq2x - sa * iq2y), ty = R2 * (ca * iq2y + sa * iq2x);
        const qm = qx * qx + qy * qy, k = GammaJ / (TAU * qm);
        const Wx = V * (ca - tx) + k * qy, Wy = V * (-sa - ty) + k * qx;
        const z2x = zetx * zetx - zety * zety, z2y = 2 * zetx * zety, zm = z2x * z2x + z2y * z2y;
        const dzx = 1 - z2x / zm, dzy = z2y / zm;
        const dm = dzx * dzx + dzy * dzy;
        if (dm > 2e-4) {
          let uc = (Wx * dzx + Wy * dzy) / dm;
          let vc = -(Wy * dzx - Wx * dzy) / dm;
          let pu = uc - V * ca, pv = vc - V * sa;
          const pm = Math.hypot(pu, pv);
          if (pm > 2.2 * V) { pu *= (2.2 * V) / pm; pv *= (2.2 * V) / pm; }
          const far = Math.max(-xi, xi - c, Math.abs(eta));
          const f = tipFade * (1 - sstep(2.8 * c, reach, far));
          ux += f * (pu * W.er[0] + pv * W.en[0]);
          uy += f * (pu * W.er[1] + pv * W.en[1]);
        }
      }
    }
    // ---- tip vortices
    if (st.gamma0 > 0.01) {
      for (let side = 0; side < 2; side++) {
        const G = side === 0 ? st.gamma0 : -st.gamma0;
        const P = this.vortex[side];
        for (let k = 0; k < P.length - 1; k++) {
          const A = P[k], B = P[k + 1];
          const r1x = px - A[0], r1y = py - A[1], r1z = pz - A[2];
          const r2x = px - B[0], r2y = py - B[1], r2z = pz - B[2];
          const cx = r1y * r2z - r1z * r2y, cy = r1z * r2x - r1x * r2z, cz = r1x * r2y - r1y * r2x;
          const r0x = B[0] - A[0], r0y = B[1] - A[1], r0z = B[2] - A[2];
          const r0m2 = r0x * r0x + r0y * r0y + r0z * r0z;
          const c2 = cx * cx + cy * cy + cz * cz + 0.0225 * r0m2; // 0.15 m core
          const r1m = Math.sqrt(r1x * r1x + r1y * r1y + r1z * r1z) + 1e-6;
          const r2m = Math.sqrt(r2x * r2x + r2y * r2y + r2z * r2z) + 1e-6;
          const dot = r0x * (r1x / r1m - r2x / r2m) + r0y * (r1y / r1m - r2y / r2m) + r0z * (r1z / r1m - r2z / r2m);
          const f = (G / (2 * TAU)) * dot / c2;
          ux += f * cx; uy += f * cy; uz += f * cz;
        }
      }
    }
    // ---- fuselage (slender body)
    if (az < 2.6 && px < 6 && px > -7 && py > -2.6 && py < 3) {
      const fu = fuselageAt(px);
      if (fu) {
        const dyy = (py - fu.c) / fu.b, dzz = pz / fu.a;
        if (dyy * dyy + dzz * dzz < 1) return -1;
      }
      const Vf = V * Math.cos(st.alphaBody);
      for (const p of this.fusSrc) {
        const dx = px - p.x, dy = py - p.y, dz = pz;
        const d2 = dx * dx + dy * dy + dz * dz + 0.01;
        const Q = Vf * p.dS;
        const f = Q / (2 * TAU * d2 * Math.sqrt(d2));
        ux += f * dx; uy += f * dy; uz += f * dz;
      }
    }
    // ---- engines (external flow + jet)
    const geo = this.geo, tab = this.tab, nac = this.nac;
    const engs = L.engines;
    for (let ei = 0; ei < engs.length; ei++) {
      const en = engs[ei];
      const dy = py - en.y, dz = pz - en.z;
      const r2 = dy * dy + dz * dz;
      const s = en.xLip - px; // metres aft of the lip
      const dsj = s - geo.L;
      if (r2 > (tab.r1 + 0.5) ** 2 && (dsj < 0 || r2 > 9)) continue;
      const r = Math.sqrt(r2);
      if (s > nac.lipS && s < geo.L && r < nac.Rmax * 1.03) {
        const ri = interp(this.duct.outer, Math.max(0, s));
        if (s >= 0 && r < ri) { out[3] = ei; return -2; }
        const ro = interp(nac.outer, Math.max(0, s));
        if (r < ro || s < 0.03) return -1;
      }
      if (r < tab.r1 && s > tab.s0 && s < tab.s1) {
        const fi = (s - tab.s0) / tab.ds, fj = r / tab.dr;
        const i = Math.min(tab.ns - 2, fi | 0), j = Math.min(tab.nr - 2, fj | 0);
        const ti = fi - i, tj = fj - j;
        const F = tab.fields;
        const o00 = (i * tab.nr + j) * 6, o01 = o00 + 6, o10 = o00 + tab.nr * 6, o11 = o10 + 6;
        const w00 = (1 - ti) * (1 - tj), w01 = (1 - ti) * tj, w10 = ti * (1 - tj), w11 = ti * tj;
        // intake face: a sheet that slows the air to the fan-face speed (cruise)
        // or speeds it up (standing still)
        const Qs = 1.5 * (this.mdotQ - V * this.Aface);
        const Qr = V * Math.PI * (nac.Rmax * nac.Rmax - nac.Rin0 * nac.Rin0) * 0.6;
        const Qn = (s > 0.8 * geo.L && r < nac.RnOut * 1.25) ? 0 : V * Math.PI * (nac.Rmax * nac.Rmax - nac.RnOut * nac.RnOut);
        let us = 0, ur = 0;
        for (let q = 0; q < 3; q++) {
          const Q = q === 0 ? Qs : q === 1 ? Qr : Qn;
          if (Q === 0) continue;
          const a = q * 2;
          us += Q * (F[o00 + a] * w00 + F[o01 + a] * w01 + F[o10 + a] * w10 + F[o11 + a] * w11);
          ur += Q * (F[o00 + a + 1] * w00 + F[o01 + a + 1] * w01 + F[o10 + a + 1] * w10 + F[o11 + a + 1] * w11);
        }
        ux -= us; // +s is aft (-x)
        if (r > 1e-4) { uy += (ur * dy) / r; uz += (ur * dz) / r; }
      }
      // jet behind the nozzle
      if (dsj > -0.02 && this.Ve > 0.5) {
        const Rn = geo.Rn;
        const dj = Math.max(0, dsj);
        const b = Rn + 0.1 * dj;
        if (r < 3 * b) {
          const decay = Math.min(1, Math.pow((4.5 * geo.Dn) / Math.max(dj, 1e-3), 0.9));
          const q = (r / b) * (r / b);
          const flat = Math.exp(-dj / (1.5 * geo.Dn));
          const prof = flat * Math.exp(-q * q * 2) + (1 - flat) * Math.exp(-q * 1.4);
          const ex = (this.Ve - V * Math.cos(st.alphaBody)) * decay * prof;
          ux -= ex;
          const spread = 0.07 * Math.abs(ex) * (r / b);
          if (r > 1e-4) { uy += (spread * dy) / r; uz += (spread * dz) / r; }
          if (this.swirlExit !== 0 && r > 1e-4) {
            const sw = this.swirlExit * decay * (r / b) * Math.exp(-((r / b) ** 2)) * 1.8 * en.side;
            uy += (-sw * dz) / r; uz += (sw * dy) / r;
          }
        }
      }
    }
    out[0] = ux; out[1] = uy; out[2] = uz;
    return 0;
  }
}


// ============================================================================
// Particles: free particles ride the field above; captured ones run down the
// duct (speed from continuity, spin from each fan) and leave as the jet.
// ============================================================================
class Particles {
  constructor(count) {
    this.n = count;
    const n = count;
    this.x = new Float32Array(n); this.y = new Float32Array(n); this.z = new Float32Array(n);
    this.mode = new Uint8Array(n);      // 0 free, 1 duct
    this.eng = new Uint8Array(n);
    this.s = new Float32Array(n); this.rf = new Float32Array(n); this.th = new Float32Array(n);
    this.age = new Float32Array(n);
    this.spd = new Float32Array(n);
    this.fresh = new Uint8Array(n);     // 1 = just spawned (no trail segment yet)
    this.tmp = new Float32Array(4);
    this.d = { ro: 0, rh: 0, vx: 0, vt: 0, stall: 0 };
    this.flow = null;
    this.focus = -1;
    this.rand = mulberry(7);
  }

  attach(flow) {
    this.flow = flow;
    this.seeds = null;
    if (this.focus === undefined) this.focus = -1;
    this.updateSeeds();
    for (let i = 0; i < this.n; i++) this.spawn(i, true);
  }

  // switch what the particles concentrate on, and start them fresh
  refocus(f) {
    this.focus = f;
    this.updateSeeds();
    for (let i = 0; i < this.n; i++) this.spawn(i, true);
  }

  // Trace the air that ends up in each intake back upstream, so new particles
  // can be released on the right streamlines (the wing's upwash bends them).
  updateSeeds() {
    const f = this.flow;
    if (!f || !f.state) return;
    const V = f.state.V;
    const out = this.tmp;
    this.seeds = f.L.engines.map((en) => {
      if (V < 9 || f.mdotQ < 0.05) return null;
      let x = en.xLip + 0.03, y = en.y, z = en.z;
      const back = this.focus >= 0 ? 1.6 : 3.6;
      for (let k = 0; k < 120 && x < en.xLip + back; k++) {
        const c = f.velocity(x, y, z, out);
        if (c !== 0) break;
        const sp = Math.hypot(out[0], out[1], out[2]) || 1;
        const h = 0.06 / sp;
        x -= out[0] * h; y -= out[1] * h; z -= out[2] * h;
      }
      const Rc = Math.min(1.6 * f.nac.Rmax, Math.sqrt(f.mdotQ / (Math.PI * Math.max(V, 1))));
      return { x, y, z, Rc };
    });
  }

  spawn(i, initial) {
    const f = this.flow, L = f.L, st = f.state, R = this.rand;
    const V = st ? st.V : 0;
    const W = L.wing;
    this.mode[i] = 0; this.age[i] = 0; this.fresh[i] = 1;
    const xFront = W.xLE(0) + 5.5;
    const xBack = -15;
    let pEng = Math.min(0.75, Math.max(0.36, 0.36 + 0.45 * (1 - V / 35)));
    if (this.focus >= 0) pEng = 0.8;
    if (this.focus === -2) pEng = 0.15;
    if (this.focus === -3) pEng = 0;
    const u = R();
    if (this.focus === -3 && V > 3) {
      // wing-section view: a thin sheet of air, like smoke from a rake in a wind tunnel
      const zs = L.sectionZ;
      this.x[i] = initial ? W.xLE(zs) + 4.5 - 18 * R() : W.xLE(zs) + 4 + R() * 0.5;
      this.y[i] = W.yLE(zs) + (R() - 0.5) * 2.6;
      this.z[i] = zs + (R() - 0.5) * 0.1;
      return;
    }
    if (this.focus === -2 && V > 9 && u > 0.15 && u < 0.85) {
      // wing-tip view: release air just ahead of the right tip so it wraps into the vortex
      const zt = W.half;
      this.x[i] = initial ? W.xLE(zt) - 8 * R() : W.xLE(zt) + 1.2 + R() * 0.4;
      this.y[i] = W.yLE(zt) + gauss(R) * 0.32;
      this.z[i] = zt + gauss(R) * 0.32;
      return;
    }
    const engs = L.engines;
    let x, y, z;
    if (u < pEng && engs.length && f.mdotQ > 0.05) {
      const ei = this.focus >= 0 ? this.focus : (R() * engs.length) | 0;
      const en = engs[ei];
      const D = f.geo.Df;
      const sd = this.seeds && this.seeds[ei];
      if (V > 9 && sd) {
        const rr = sd.Rc * 1.05 * Math.sqrt(R()), th = R() * TAU;
        const t = initial ? R() : 0;
        x = sd.x + (en.xLip - sd.x) * t; y = sd.y + (en.y - sd.y) * t + rr * Math.cos(th); z = sd.z + (en.z - sd.z) * t + rr * Math.sin(th);
      } else {
        // standing still: air is drawn in from all around the intake
        const rr = D * (0.45 + 1.4 * R());
        const ct = 1 - 1.25 * R(), sth = Math.sqrt(Math.max(0, 1 - ct * ct)), ph = R() * TAU;
        x = en.xLip + 0.15 * D + rr * ct; y = en.y + rr * sth * Math.cos(ph); z = en.z + rr * sth * Math.sin(ph);
      }
    } else if (V < 3) {
      // nothing moves far from the engines when standing still: keep them near the intakes
      const en = engs.length ? engs[(R() * engs.length) | 0] : { xLip: 1, y: 0, z: 0 };
      const D = f.geo.Df;
      const rr = D * (0.6 + 2.2 * R());
      const ct = 1 - 1.4 * R(), sth = Math.sqrt(Math.max(0, 1 - ct * ct)), ph = R() * TAU;
      x = en.xLip + rr * ct; y = en.y + rr * sth * Math.cos(ph); z = en.z + rr * sth * Math.sin(ph);
    } else {
      const v = R();
      x = initial ? xBack + (xFront - xBack) * R() : xFront + R() * 0.5;
      if (v < 0.6) {
        // around the wing, denser near it
        z = (R() * 2 - 1) * (W.half + 0.3);
        y = W.yLE(z) + gauss(R) * 0.75;
      } else if (v < 0.82) {
        const sg = R() < 0.5 ? -1 : 1;
        z = sg * (W.half + gauss(R) * 0.55);
        y = W.yLE(W.half) + gauss(R) * 0.55;
      } else {
        z = (R() * 2 - 1) * (W.half + 2.5);
        y = -2.6 + R() * 5.6;
      }
    }
    this.x[i] = x; this.y[i] = y; this.z[i] = z;
  }

  // Advance every particle by dt (seconds of flow time). For each particle the
  // writer gets the segment it moved along this frame.
  step(dt, write) {
    const f = this.flow;
    if (!f || !f.state) return;
    const st = f.state;
    const V = st.V;
    const Vref = Math.max(V, 22);
    const geo = f.geo;
    const engs = f.L.engines;
    const tmp = this.tmp, d = this.d, R = this.rand;
    const half = f.L.wing.half;
    for (let i = 0; i < this.n; i++) {
      const x0 = this.x[i], y0 = this.y[i], z0 = this.z[i];
      let fresh = this.fresh[i];
      this.fresh[i] = 0;
      this.age[i] += dt;
      if (this.mode[i] === 1) {
        // ---- inside the duct
        const en = engs[this.eng[i]];
        let s = this.s[i];
        f.ductAt(s, d);
        let rf = this.rf[i];
        if (d.stall === 2) {
          rf += (R() - 0.5) * 0.25;
          if (R() < 0.12) s -= d.vx * dt * (0.5 + R());
        } else if (d.stall === 1) rf += (R() - 0.5) * 0.06;
        rf = Math.min(0.97, Math.max(0.03, rf));
        this.rf[i] = rf;
        s += Math.max(d.vx, 2) * dt;
        const r = d.rh + rf * (d.ro - d.rh);
        this.th[i] += (d.vt / Math.max(r, 0.02)) * dt * en.side;
        this.s[i] = s;
        const th = this.th[i];
        this.x[i] = en.xLip - s; this.y[i] = en.y + r * Math.cos(th); this.z[i] = en.z + r * Math.sin(th);
        this.spd[i] = Math.hypot(d.vx, d.vt) / Vref;
        if (s >= geo.L) this.mode[i] = 0; // out of the nozzle: now part of the jet
      } else {
        // ---- free air
        const code = f.velocity(x0, y0, z0, tmp);
        if (code === -2) {
          // entered an intake
          const en = engs[tmp[3]];
          const dy = y0 - en.y, dz = z0 - en.z;
          const s = Math.max(0, en.xLip - x0);
          f.ductAt(s, d);
          const r = Math.hypot(dy, dz);
          this.mode[i] = 1; this.eng[i] = tmp[3]; this.s[i] = s;
          this.rf[i] = Math.min(0.97, Math.max(0.03, (r - d.rh) / Math.max(1e-3, d.ro - d.rh)));
          this.th[i] = Math.atan2(dz, dy);
          this.spd[i] = Math.hypot(d.vx, d.vt) / Vref;
          write(i, x0, y0, z0, x0, y0, z0, this.spd[i], fresh);
          continue;
        }
        if (code === -1) { this.spawn(i, false); write(i, this.x[i], this.y[i], this.z[i], this.x[i], this.y[i], this.z[i], 1, 1); continue; }
        let vx = tmp[0], vy = tmp[1], vz = tmp[2];
        const sp = Math.sqrt(vx * vx + vy * vy + vz * vz);
        this.spd[i] = sp / Vref;
        this.x[i] = x0 + vx * dt; this.y[i] = y0 + vy * dt; this.z[i] = z0 + vz * dt;
        const x = this.x[i], y = this.y[i], z = this.z[i];
        const tooSlow = sp < 0.4 && this.age[i] > 1.5;
        if (x < -15 || x > 9 || y < -5.5 || y > 6 || Math.abs(z) > half + 5.5 || this.age[i] > 14 || tooSlow) {
          this.spawn(i, false);
          write(i, this.x[i], this.y[i], this.z[i], this.x[i], this.y[i], this.z[i], 1, 1);
          continue;
        }
      }
      write(i, x0, y0, z0, this.x[i], this.y[i], this.z[i], this.spd[i], fresh);
    }
  }
}

function gauss(R) {
  const u = Math.max(1e-6, R()), v = R();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * v);
}

function mulberry(a) {
  return function () {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export { FlowField, Particles };
