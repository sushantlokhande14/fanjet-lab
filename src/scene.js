// ============================================================================
// scene.js: Three.js plane model, world, wind-tunnel backdrop, air streaks.
// ============================================================================
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { JK, FUSELAGE, ductProfile, interp, nacelleOutline } from './layout.js';
const TAU = Math.PI * 2;

// ---------------------------------------------------------------- materials
function makeMaterials() {
  const m = {
    paint: new THREE.MeshStandardMaterial({ color: 0xffffff, roughness: 0.42, metalness: 0.05, vertexColors: true }),
    white: new THREE.MeshStandardMaterial({ color: 0xe4e9ed, roughness: 0.42, metalness: 0.05 }),
    cowl: new THREE.MeshStandardMaterial({ color: 0xe9edf0, roughness: 0.3, metalness: 0.15, side: THREE.DoubleSide }),
    glassCowl: new THREE.ShaderMaterial({
      uniforms: { uColor: { value: new THREE.Color(0x9fdcf0) }, uBase: { value: 0.05 }, uRim: { value: 0.55 } },
      vertexShader: `varying vec3 vN; varying vec3 vV;
        void main(){ vec4 mv = modelViewMatrix * vec4(position,1.0); vN = normalize(normalMatrix*normal); vV = normalize(-mv.xyz); gl_Position = projectionMatrix*mv; }`,
      fragmentShader: `uniform vec3 uColor; uniform float uBase; uniform float uRim; varying vec3 vN; varying vec3 vV;
        void main(){ float f = 1.0 - abs(dot(normalize(vN), normalize(vV))); gl_FragColor = vec4(uColor, uBase + uRim*pow(f, 2.2)); }`,
      transparent: true, depthWrite: false, side: THREE.DoubleSide,
    }),
    duct: new THREE.MeshStandardMaterial({ color: 0x6c7782, roughness: 0.45, metalness: 0.6, side: THREE.DoubleSide }),
    hub: new THREE.MeshStandardMaterial({ color: 0x9aa5b0, roughness: 0.25, metalness: 0.85 }),
    stator: new THREE.MeshStandardMaterial({ color: 0x47525c, roughness: 0.5, metalness: 0.5, side: THREE.DoubleSide }),
    glass: new THREE.MeshStandardMaterial({ color: 0x18232d, roughness: 0.06, metalness: 0.7 }),
    tire: new THREE.MeshStandardMaterial({ color: 0x1a1c1f, roughness: 0.9, metalness: 0.0 }),
    strut: new THREE.MeshStandardMaterial({ color: 0xb8c0c8, roughness: 0.3, metalness: 0.8 }),
    rotor: [],
  };
  return m;
}

// ---------------------------------------------------------------- geometry helpers
function geomFrom(pos, idx, col, uv) {
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
  if (col) g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
  g.setIndex(idx);
  g.computeVertexNormals();
  return g;
}

// revolve an (s, r) outline round the x axis; s runs aft (-x)
function revolve(profile, segs = 48) {
  const pos = [], idx = [];
  const n = profile.length;
  for (let i = 0; i < n; i++) {
    const [s, r] = profile[i];
    for (let j = 0; j <= segs; j++) {
      const t = (j / segs) * TAU;
      pos.push(-s, r * Math.cos(t), r * Math.sin(t));
    }
  }
  for (let i = 0; i < n - 1; i++) {
    for (let j = 0; j < segs; j++) {
      const a = i * (segs + 1) + j, b = a + segs + 1;
      idx.push(a, b, a + 1, b, b + 1, a + 1);
    }
  }
  return geomFrom(pos, idx);
}

// smooth fuselage (Catmull-Rom through the station table)
function fusSmooth(x) {
  const F = FUSELAGE;
  if (x >= F[0][0]) return { a: F[0][1], b: F[0][2], c: F[0][3] };
  if (x <= F[F.length - 1][0]) return { a: F[F.length - 1][1], b: F[F.length - 1][2], c: F[F.length - 1][3] };
  let i = 0;
  while (i < F.length - 2 && x < F[i + 1][0]) i++;
  const p0 = F[Math.max(0, i - 1)], p1 = F[i], p2 = F[i + 1], p3 = F[Math.min(F.length - 1, i + 2)];
  const t = (p1[0] - x) / (p1[0] - p2[0]);
  const cr = (k) => {
    const a = p0[k], b = p1[k], c = p2[k], d = p3[k];
    return 0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t * t + (-a + 3 * b - 3 * c + d) * t * t * t);
  };
  return { a: Math.max(0.01, cr(1)), b: Math.max(0.01, cr(2)), c: cr(3) };
}

const SE = 2.4; // superellipse exponent of the cabin cross-section
function fusPoint(x, t, out) {
  const f = fusSmooth(x);
  const ct = Math.cos(t), st = Math.sin(t);
  const z = f.a * Math.sign(ct) * Math.pow(Math.abs(ct), 2 / SE);
  const y = f.c + f.b * Math.sign(st) * Math.pow(Math.abs(st), 2 / SE);
  out[0] = x; out[1] = y; out[2] = z;
  return f;
}

function buildFuselage(accent) {
  const nx = 110, nt = 56;
  const x0 = FUSELAGE[0][0], x1 = FUSELAGE[FUSELAGE.length - 1][0];
  const pos = [], col = [], idx = [];
  const p = [0, 0, 0];
  const white = new THREE.Color(0xe9edf0), acc = new THREE.Color(accent), belly = new THREE.Color(0xb9c3cb);
  for (let i = 0; i <= nx; i++) {
    const u = i / nx;
    const x = x0 + (x1 - x0) * (0.5 - 0.5 * Math.cos(Math.PI * u)) * 0.6 + (x1 - x0) * u * 0.4;
    for (let j = 0; j <= nt; j++) {
      const t = (j / nt) * TAU;
      const f = fusPoint(x, t, p);
      pos.push(p[0], p[1], p[2]);
      const hy = (p[1] - f.c) / f.b; // -1 bottom .. 1 top
      let c = white;
      if (hy > -0.2 && hy < -0.05 && x < 3.3 && x > -3.6) c = acc;
      else if (hy < -0.62) c = belly;
      col.push(c.r, c.g, c.b);
    }
  }
  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < nt; j++) {
      const a = i * (nt + 1) + j, b = a + nt + 1;
      idx.push(a, a + 1, b, b, a + 1, b + 1);
    }
  }
  return geomFrom(pos, idx, col);
}

// a patch of the fuselage skin, pushed out a little (windows)
function fusPatch(xa, xb, ta, tb, lift = 0.006, nx = 10, nt = 8) {
  const pos = [], idx = [];
  const p = [0, 0, 0], q = [0, 0, 0];
  for (let i = 0; i <= nx; i++) {
    const x = xa + ((xb - xa) * i) / nx;
    for (let j = 0; j <= nt; j++) {
      const t = ta + ((tb - ta) * j) / nt;
      const f = fusPoint(x, t, p);
      // outward direction from the section centre
      const dy = p[1] - f.c, dz = p[2];
      const L = Math.hypot(dy, dz) || 1;
      pos.push(p[0], p[1] + (dy / L) * lift, p[2] + (dz / L) * lift);
    }
  }
  for (let i = 0; i < nx; i++) {
    for (let j = 0; j < nt; j++) {
      const a = i * (nt + 1) + j, b = a + nt + 1;
      idx.push(a, a + 1, b, b, a + 1, b + 1);
    }
  }
  return geomFrom(pos, idx);
}

// wing / tail surface: list of sections {le:[x,y,z], chord, er:[..], en:[..]} and a closed airfoil outline
function liftingSurface(sections, outline, capEnds = true) {
  const pos = [], idx = [];
  const m = outline.length;
  for (const s of sections) {
    for (const [u, v] of outline) {
      pos.push(
        s.le[0] + u * s.chord * s.er[0] + v * s.chord * s.en[0],
        s.le[1] + u * s.chord * s.er[1] + v * s.chord * s.en[1],
        s.le[2] + u * s.chord * s.er[2] + v * s.chord * s.en[2],
      );
    }
  }
  for (let i = 0; i < sections.length - 1; i++) {
    for (let j = 0; j < m; j++) {
      const a = i * m + j, b = i * m + ((j + 1) % m), c = a + m, d = b + m;
      idx.push(a, c, b, b, c, d);
    }
  }
  if (capEnds) {
    for (const [si, flip] of [[0, true], [sections.length - 1, false]]) {
      const s = sections[si];
      const ci = pos.length / 3;
      pos.push(s.le[0] + 0.4 * s.chord * s.er[0], s.le[1] + 0.4 * s.chord * s.er[1], s.le[2] + 0.4 * s.chord * s.er[2]);
      for (let j = 0; j < m; j++) {
        const a = si * m + j, b = si * m + ((j + 1) % m);
        if (flip) idx.push(ci, a, b); else idx.push(ci, b, a);
      }
    }
  }
  return geomFrom(pos, idx);
}

function naca00(t, n = 40) {
  const up = [], lo = [];
  for (let i = 0; i <= n; i++) {
    const x = 0.5 - 0.5 * Math.cos((Math.PI * i) / n);
    const y = 5 * t * (0.2969 * Math.sqrt(x) - 0.126 * x - 0.3516 * x * x + 0.2843 * x ** 3 - 0.1036 * x ** 4);
    up.push([x, y]); lo.push([x, -y]);
  }
  // TE -> upper -> LE -> lower -> TE
  const out = [];
  for (let i = n; i >= 0; i--) out.push(up[i]);
  for (let i = 1; i < n; i++) out.push(lo[i]);
  return out;
}

// ---------------------------------------------------------------- plane
class PlaneModel {
  constructor(layout, mats) {
    this.L = layout;
    this.m = mats;
    this.group = new THREE.Group();
    this.rotors = [];   // per engine: array of rotor groups
    this.flaps = [];
    this.gear = [];
    this.cowls = [];
    this.build();
  }

  build() {
    const L = this.L, m = this.m, g = this.group;
    const W = L.wing;
    this.others = [];
    const keep = (o) => { this.others.push(o); return o; };
    // fuselage + windows
    g.add(keep(new THREE.Mesh(buildFuselage(0x0a6c81), m.paint)));
    g.add(keep(new THREE.Mesh(fusPatch(2.45, 3.32, Math.PI * 0.22, Math.PI * 0.78, 0.008, 12, 14), m.glass)));
    for (const side of [1, -1]) {
      for (let k = 0; k < 3; k++) {
        const xa = 1.85 - k * 0.78, xb = xa - 0.52;
        const ta = side > 0 ? 0.18 : Math.PI - 0.62, tb = side > 0 ? 0.62 : Math.PI - 0.18;
        g.add(keep(new THREE.Mesh(fusPatch(xb, xa, ta, tb, 0.006, 6, 6), m.glass)));
      }
    }
    // wing (Joukowski section, cosine-spaced stations)
    const er = [W.er[0], W.er[1], 0], en = [W.en[0], W.en[1], 0];
    const secs = [];
    const ns = 44;
    for (let i = 0; i <= ns; i++) {
      const u = i / ns;
      const z = -W.half + 2 * W.half * (0.5 - 0.5 * Math.cos(Math.PI * u));
      secs.push({ le: [W.xLE(z), W.yLE(z), z], chord: W.chord(z), er, en });
    }
    const wingOutline = JK.pts.slice();
    g.add(new THREE.Mesh(liftingSurface(secs, wingOutline), m.white));
    // flaps: aft 26 % of the chord, inboard, hinge rotates them down
    const flapOutline = [];
    for (const [u, v] of JK.pts) if (u >= 0.72) flapOutline.push([(u - 0.72) / 0.28, v / 0.28]);
    for (const side of [1, -1]) {
      const za = side * 0.78, zb = side * Math.min(W.half * 0.62, 4.1);
      const hinge = (z) => { const c = W.chord(z); return [W.xLE(z) + 0.72 * c * W.er[0], W.yLE(z) + 0.72 * c * W.er[1] - 0.012 * c, z]; };
      const ha = hinge(za), hb = hinge(zb);
      const piv = new THREE.Group();
      piv.position.set(ha[0], ha[1], ha[2]);
      const axis = new THREE.Vector3(hb[0] - ha[0], hb[1] - ha[1], hb[2] - ha[2]).normalize();
      const fs = [];
      for (let i = 0; i <= 6; i++) {
        const z = za + ((zb - za) * i) / 6;
        const h = hinge(z);
        fs.push({ le: [h[0] - ha[0], h[1] - ha[1], h[2] - ha[2]], chord: 0.28 * W.chord(z), er, en });
      }
      const flap = new THREE.Mesh(liftingSurface(fs, flapOutline.length > 4 ? flapOutline : naca00(0.1)), m.white);
      piv.add(flap);
      g.add(keep(piv));
      this.flaps.push({ piv, axis, side });
    }
    // tail: fin + T stabiliser
    const fin = [];
    const finE = [-1, 0, 0], finN = [0, 0, 1];
    for (let i = 0; i <= 8; i++) {
      const t = i / 8;
      const y = 0.45 + t * 1.6, xle = -2.6 - t * 1.05, c = 1.85 - t * 0.95;
      fin.push({ le: [xle, y, 0], chord: c, er: finE, en: finN });
    }
    g.add(keep(new THREE.Mesh(liftingSurface(fin, naca00(0.11)), m.white)));
    const ht = [];
    for (let i = 0; i <= 12; i++) {
      const u = i / 12;
      const z = -2.25 + 4.5 * u;
      const t = Math.abs(z) / 2.25;
      ht.push({ le: [-3.62 - t * 0.28, 2.04, z], chord: 0.9 - 0.32 * t, er: [-1, 0, 0], en: [0, 1, 0] });
    }
    g.add(keep(new THREE.Mesh(liftingSurface(ht, naca00(0.1)), m.white)));
    // engines
    for (const en of L.engines) g.add(keep(this.buildEngine(en)));
    // landing gear
    this.buildGear();
  }

  buildEngine(en) {
    const L = this.L, geo = L.geo, m = this.m;
    const eg = new THREE.Group();
    eg.position.set(en.xLip, en.y, en.z);
    const nac = nacelleOutline(geo);
    const shell = new THREE.Mesh(revolve(nac.pts, 56), m.cowl);
    shell.renderOrder = 2;
    eg.add(shell);
    this.cowls.push(shell);
    // duct liner (visible through the glass cowl)
    const duct = ductProfile(geo);
    const liner = new THREE.Mesh(revolve(duct.outer.map(([s, r]) => [s, r * 0.995]), 48), m.duct);
    liner.visible = false;
    eg.add(liner);
    this.cowls.push({ liner });
    // hub, spinner, tail cone
    const hubProf = duct.hub.filter((p, i) => i > 0);
    eg.add(new THREE.Mesh(revolve(hubProf, 32), m.hub));
    // rotors + stators
    const rotors = [];
    const mode = L.d.swirl;
    geo.stages.forEach((st, k) => {
      const rear = mode === 'contra' && (k & 1) === 1;
      const dir = (rear ? -1 : 1) * en.side;
      if (!m.rotor[k]) m.rotor[k] = new THREE.MeshStandardMaterial({ color: 0xc7cfd6, roughness: 0.3, metalness: 0.75, side: THREE.DoubleSide });
      const rg = new THREE.Group();
      rg.position.x = -st.x;
      rg.add(new THREE.Mesh(bladeRing(st.rh * 0.98, st.R * 0.985, 11 + 2 * k, 0.17 * st.D, dir, 0.62, 0.95), m.rotor[k]));
      eg.add(rg);
      rotors.push({ g: rg, dir, k });
      if (mode === 'stators') {
        const sg = new THREE.Mesh(bladeRing(st.rh * 0.98, st.R * 0.99, 9 + 2 * k, 0.14 * st.D, -dir, 0.18, 0.22), m.stator);
        sg.position.x = -(st.x + 0.2 * st.D);
        eg.add(sg);
      }
    });
    this.rotors.push(rotors);
    // pylon
    const W = L.wing;
    const ywing = W.yLE(en.z) - 0.05 * en.chord;
    const top = ywing - en.y;
    const pyl = [];
    const pc = 0.62 * geo.L;
    for (let i = 0; i <= 4; i++) {
      const t = i / 4;
      const y = nac.Rmax * 0.8 + t * (top - nac.Rmax * 0.8);
      pyl.push({ le: [-0.2 * geo.L + t * 0.05, y, 0], chord: pc - t * 0.1, er: [-1, 0, 0], en: [0, 0, 1] });
    }
    eg.add(new THREE.Mesh(liftingSurface(pyl, naca00(0.12)), m.white));
    return eg;
  }

  buildGear() {
    const m = this.m, g = this.group;
    const wheel = (r, w) => { const geo = new THREE.CylinderGeometry(r, r, w, 22); geo.rotateX(Math.PI / 2); return geo; };
    const mk = (x, z, r, w, top) => {
      const gg = new THREE.Group();
      const strut = new THREE.Mesh(new THREE.CylinderGeometry(0.035, 0.04, top + 1.25 - r, 10), m.strut);
      strut.position.set(x, (top - 1.25 + r) / 2, z);
      gg.add(strut);
      const wh = new THREE.Mesh(wheel(r, w), m.tire);
      wh.position.set(x, -1.25 + r, z);
      gg.add(wh);
      g.add(gg);
      this.others.push(gg);
      this.gear.push(gg);
    };
    mk(2.95, 0, 0.17, 0.12, -0.45);
    mk(-0.25, 1.02, 0.24, 0.16, -0.5);
    mk(-0.25, -1.02, 0.24, 0.16, -0.5);
    // sponsons the main gear folds into
    for (const s of [1, -1]) {
      const sp = new THREE.Mesh(new THREE.SphereGeometry(1, 20, 12), m.white);
      sp.scale.set(0.95, 0.22, 0.3);
      sp.position.set(-0.25, -0.5, s * 0.7);
      g.add(sp);
      this.others.push(sp);
    }
  }

  // cut the right wing open at z = zc to show the airfoil section (null = no cut)
  setSection(zc) {
    const m = this.m.white;
    m.clippingPlanes = zc == null ? [] : [new THREE.Plane(new THREE.Vector3(0, 0, -1), zc)];
    m.side = zc == null ? THREE.FrontSide : THREE.DoubleSide;
    m.needsUpdate = true;
    this.sectionOn = zc != null;
    for (const o of this.others) o.visible = zc == null;
  }

  setXray(on) {
    for (const c of this.cowls) {
      if (c.liner) c.liner.visible = false;
      else c.material = on ? this.m.glassCowl : this.m.cowl;
    }
  }

  // st: { theta, gear, flapPos, eng (rec), visScale }
  update(st, dt) {
    this.group.rotation.z = st.theta || 0;
    for (const f of this.flaps) {
      // positive rotation about a +z hinge swings the trailing edge down
      f.piv.quaternion.setFromAxisAngle(f.axis, (st.flapPos || 0) * 0.55 * Math.sign(f.axis.z || 1));
    }
    for (const gg of this.gear) {
      const k = Math.max(0, Math.min(1, st.gear ?? 1));
      gg.visible = k > 0.03 && !this.sectionOn;
      gg.position.y = (1 - k) * 0.75;
    }
    const e = st.eng;
    if (e && e.stages) {
      for (const rotors of this.rotors) {
        for (const r of rotors) {
          const rec = e.stages[r.k];
          if (!rec) continue;
          const w = Math.min((rec.rpm / 60) * TAU * (st.visScale || 0.05), TAU * 2.4);
          r.g.rotation.x += w * dt * r.dir;
        }
      }
      e.stages.forEach((rec, k) => {
        const mat = this.m.rotor[k];
        if (!mat || !rec) return;
        const c = rec.stall === 2 ? 0xff5a4a : rec.stall === 1 ? 0xffb347 : 0xc7cfd6;
        mat.color.setHex(c);
        mat.emissive.setHex(rec.stall === 2 ? 0x551008 : 0x000000);
      });
    }
  }

  dispose() {
    this.group.traverse((o) => { if (o.geometry) o.geometry.dispose(); });
  }
}

// ring of thin twisted blades between radii r0 and r1, chord c, about the x axis
function bladeRing(r0, r1, n, c, dir, stagHub, stagTip) {
  const pos = [], idx = [];
  const ns = 6;
  for (let b = 0; b < n; b++) {
    const th0 = (b / n) * TAU;
    const base = pos.length / 3;
    for (let i = 0; i <= ns; i++) {
      const t = i / ns;
      const r = r0 + (r1 - r0) * t;
      const stg = stagHub + (stagTip - stagHub) * t; // radians from the axis
      const cc = c * (1 - 0.25 * t);
      const ax = Math.cos(stg) * cc * 0.5, tg = Math.sin(stg) * cc * 0.5 * dir;
      for (const sgn of [1, -1]) {
        const th = th0 + (sgn * tg) / r;
        pos.push(sgn * ax, r * Math.cos(th), r * Math.sin(th));
      }
    }
    for (let i = 0; i < ns; i++) {
      const a = base + i * 2;
      idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
    }
  }
  return geomFrom(pos, idx);
}

// ---------------------------------------------------------------- air streaks
class TrailRenderer {
  constructor(P, K) {
    this.P = P; this.K = K;
    const N = P * K;
    const geo = new THREE.InstancedBufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute([0, -1, 0, 0, 1, 0, 1, -1, 0, 1, 1, 0], 3));
    geo.setIndex([0, 2, 1, 2, 3, 1]);
    const mk = (n) => new THREE.InstancedBufferAttribute(new Float32Array(N * n), n).setUsage(THREE.DynamicDrawUsage);
    this.aA = mk(3); this.aB = mk(3); this.aS = mk(2); this.aF = mk(1);
    this.aF.array.fill(-1e9);
    geo.setAttribute('aA', this.aA); geo.setAttribute('aB', this.aB); geo.setAttribute('aS', this.aS); geo.setAttribute('aF', this.aF);
    geo.instanceCount = N;
    this.mat = new THREE.ShaderMaterial({
      uniforms: {
        uFrame: { value: 0 }, uK: { value: K }, uWidth: { value: 2.2 }, uRes: { value: new THREE.Vector2(800, 600) },
        uAlpha: { value: 0.9 }, uAdd: { value: 1 }, uCalm: { value: 0.2 },
      },
      vertexShader: `
        attribute vec3 aA; attribute vec3 aB; attribute vec2 aS; attribute float aF;
        uniform float uFrame; uniform float uK; uniform float uWidth; uniform vec2 uRes;
        varying float vA; varying float vS; varying float vY;
        void main(){
          float age = uFrame - aF;
          float life = 1.0 - age / uK;
          vec4 cA = projectionMatrix * modelViewMatrix * vec4(aA, 1.0);
          vec4 cB = projectionMatrix * modelViewMatrix * vec4(aB, 1.0);
          vec2 sA = cA.xy / max(cA.w, 1e-4) * uRes * 0.5;
          vec2 sB = cB.xy / max(cB.w, 1e-4) * uRes * 0.5;
          vec2 d = sB - sA; float len = length(d);
          d = len > 1e-3 ? d / len : vec2(1.0, 0.0);
          vec2 nrm = vec2(-d.y, d.x);
          vec4 c = mix(cA, cB, position.x);
          float w = uWidth * (0.45 + 0.55 * life);
          c.xy += nrm * position.y * w / uRes * c.w;
          gl_Position = c;
          vA = (age < 0.0 || age >= uK || cA.w <= 0.0 || cB.w <= 0.0) ? 0.0 : life * smoothstep(0.02, 0.6, len);
          vS = mix(aS.x, aS.y, position.x);
          vY = position.y;
        }`,
      fragmentShader: `
        uniform float uAlpha; uniform float uAdd;
        varying float vA; varying float vS; varying float vY;
        vec3 ramp(float s){
          vec3 c0 = vec3(0.12, 0.37, 0.95), c1 = vec3(0.55, 0.78, 1.0), c2 = vec3(0.95, 0.97, 1.0);
          vec3 c3 = vec3(1.0, 0.80, 0.27), c4 = vec3(1.0, 0.42, 0.10), c5 = vec3(0.94, 0.15, 0.45);
          if (s < 0.65) return mix(c0, c1, s / 0.65);
          if (s < 0.95) return mix(c1, c2, (s - 0.65) / 0.3);
          if (s < 1.25) return mix(c2, c3, (s - 0.95) / 0.3);
          if (s < 1.7) return mix(c3, c4, (s - 1.25) / 0.45);
          return mix(c4, c5, clamp((s - 1.7) / 0.9, 0.0, 1.0));
        }
        uniform float uCalm;
        void main(){
          float dev = clamp(abs(vS - 1.0) * 3.2, 0.0, 1.0);
          float a = vA * (1.0 - vY * vY) * uAlpha * mix(uCalm, 1.0, dev);
          if (a < 0.004) discard;
          vec3 col = pow(ramp(vS), vec3(2.2));
          gl_FragColor = vec4(col, a);
          #include <colorspace_fragment>
        }`,
      transparent: true, depthWrite: false, blending: THREE.AdditiveBlending,
    });
    this.mesh = new THREE.Mesh(geo, this.mat);
    this.mesh.frustumCulled = false;
    this.mesh.renderOrder = 5;
    this.frame = 0;
  }

  setLook(additive, alpha, width) {
    this.mat.blending = additive ? THREE.AdditiveBlending : THREE.NormalBlending;
    this.mat.uniforms.uAlpha.value = alpha;
    this.mat.uniforms.uWidth.value = width;
    this.mat.needsUpdate = true;
  }

  clear() { this.aF.array.fill(-1e9); this.aF.needsUpdate = true; }

  // seg: Float32Array P*8 = ax ay az bx by bz speed fresh
  push(seg, count) {
    const P = this.P, K = this.K;
    this.frame++;
    const slot = this.frame % K;
    const base = slot * P;
    const A = this.aA.array, B = this.aB.array, S = this.aS.array, F = this.aF.array;
    const n = Math.min(count, P);
    for (let i = 0; i < n; i++) {
      const o = i * 8, j = base + i;
      A[j * 3] = seg[o]; A[j * 3 + 1] = seg[o + 1]; A[j * 3 + 2] = seg[o + 2];
      B[j * 3] = seg[o + 3]; B[j * 3 + 1] = seg[o + 4]; B[j * 3 + 2] = seg[o + 5];
      const sp = seg[o + 6];
      S[j * 2] = this.lastS ? this.lastS[i] : sp; S[j * 2 + 1] = sp;
      F[j] = seg[o + 7] > 0.5 ? -1e9 : this.frame;
    }
    if (!this.lastS || this.lastS.length !== P) this.lastS = new Float32Array(P);
    for (let i = 0; i < n; i++) this.lastS[i] = seg[i * 8 + 6];
    for (const [attr, w] of [[this.aA, 3], [this.aB, 3], [this.aS, 2], [this.aF, 1]]) {
      attr.clearUpdateRanges();
      attr.addUpdateRange(base * w, P * w);
      attr.needsUpdate = true;
    }
    this.mat.uniforms.uFrame.value = this.frame;
  }
}

// ---------------------------------------------------------------- world
const groundVS = `
  varying vec3 vP;
  void main(){ vec4 w = modelMatrix * vec4(position,1.0); vP = w.xyz; gl_Position = projectionMatrix * viewMatrix * w; }`;
const groundFS = `
  uniform vec2 uOff; uniform vec3 uFog; uniform float uFogK; uniform vec4 uRw1; uniform vec4 uRw2;
  varying vec3 vP;
  float h21(vec2 p){ p = fract(p*vec2(123.34, 456.21)); p += dot(p, p+45.32); return fract(p.x*p.y); }
  vec2 h22(vec2 p){ float n = h21(p); return vec2(n, h21(p + n + 17.17)); }
  float vn(vec2 p){ vec2 i = floor(p), f = fract(p); vec2 u = f*f*(3.0-2.0*f);
    return mix(mix(h21(i), h21(i+vec2(1.0,0.0)), u.x), mix(h21(i+vec2(0.0,1.0)), h21(i+vec2(1.0,1.0)), u.x), u.y); }
  float fbm(vec2 p){ float s = 0.0, a = 0.5; for (int i = 0; i < 4; i++){ s += a*vn(p); p = p*2.03 + 1.7; a *= 0.5; } return s; }
  vec3 rwy(vec2 w, vec4 rw, inout float m){
    if (rw.w < 0.5) return vec3(0.0);
    float lx = w.x - rw.x, L = rw.y - rw.x, hw = rw.z, az = abs(w.y);
    float apron = step(-60.0, lx) * step(lx, L + 60.0) * step(az, hw + 22.0);
    if (lx < 0.0 || lx > L || az > hw) { m = max(m, apron * 0.6); return vec3(0.42, 0.55, 0.30); }
    m = 1.0;
    vec3 asph = vec3(0.24, 0.25, 0.26) * (0.92 + 0.16*vn(w*0.6));
    float cl = step(az, 0.45) * step(mod(lx, 50.0), 30.0) * step(70.0, lx) * step(lx, L - 70.0);
    float le = min(lx, L - lx);
    float th = (le > 6.0 && le < 40.0) ? step(2.0, az) * step(az, hw - 2.0) * step(mod(az - 2.0, 3.4), 1.7) : 0.0;
    float ed = step(hw - 0.9, az);
    float tz = (le > 150.0 && le < 450.0) ? step(4.0, az) * step(az, 9.0) * step(mod(le, 150.0), 22.0) : 0.0;
    return mix(asph, vec3(0.93), max(max(cl, th), max(ed, tz)));
  }
  void main(){
    float vD = distance(vP, cameraPosition);
    vec2 w = vP.xz + uOff;
    vec2 g = w / 230.0; vec2 ip = floor(g), fp = fract(g);
    float d1 = 9.0, d2 = 9.0; vec2 id = vec2(0.0);
    for (int j = -1; j <= 1; j++) for (int i = -1; i <= 1; i++){
      vec2 o = vec2(float(i), float(j));
      vec2 r = o + h22(ip + o) * 0.8 - fp;
      r.x *= 0.8;
      float d = dot(r, r);
      if (d < d1){ d2 = d1; d1 = d; id = ip + o; } else if (d < d2) d2 = d;
    }
    float edge = sqrt(d2) - sqrt(d1);
    float hs = h21(id);
    vec3 c = hs < 0.22 ? vec3(0.37, 0.52, 0.23) : hs < 0.4 ? vec3(0.55, 0.60, 0.28) : hs < 0.55 ? vec3(0.74, 0.66, 0.40) :
             hs < 0.68 ? vec3(0.50, 0.40, 0.29) : hs < 0.83 ? vec3(0.31, 0.46, 0.21) : vec3(0.62, 0.64, 0.38);
    float ang = hs * 6.2831; vec2 dir = vec2(cos(ang), sin(ang));
    float rows = 0.5 + 0.5 * sin(dot(w, dir) * 1.1);
    c *= 0.93 + 0.07 * rows * smoothstep(2500.0, 300.0, vD);
    c *= 0.86 + 0.28 * vn(w / 41.0);
    c = mix(c, vec3(0.22, 0.31, 0.15), (1.0 - smoothstep(0.0, 0.04, edge)) * 0.75);
    float fo = fbm(w / 1500.0);
    c = mix(c, vec3(0.13, 0.23, 0.12) * (0.75 + 0.5 * vn(w / 8.0)), smoothstep(0.6, 0.64, fo));
    float lk = fbm(w / 3200.0 + 11.3);
    c = mix(c, vec3(0.23, 0.36, 0.45), smoothstep(0.715, 0.725, lk));
    float rx = abs(fract(w.x / 3100.0 + 0.25 * fbm(w / 5200.0)) - 0.5) * 3100.0;
    float rz = abs(fract(w.y / 2700.0 + 0.25 * fbm(w / 4900.0 + 7.0)) - 0.5) * 2700.0;
    c = mix(c, vec3(0.45, 0.45, 0.43), (1.0 - smoothstep(3.0, 5.5, min(rx, rz))) * 0.85);
    float m = 0.0;
    vec3 r1 = rwy(w, uRw1, m);
    if (m > 0.0) c = mix(c, r1, m);
    float m2 = 0.0;
    vec3 r2 = rwy(w, uRw2, m2);
    if (m2 > 0.0) c = mix(c, r2, m2);
    c = pow(c, vec3(2.2));
    float f = 1.0 - exp(-pow(vD * uFogK, 1.4));
    c = mix(c, uFog, clamp(f, 0.0, 1.0));
    gl_FragColor = vec4(c, 1.0);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }`;

const skyFS = `
  uniform vec3 uTop; uniform vec3 uHor; uniform vec3 uSun; varying vec3 vDir;
  void main(){
    vec3 d = normalize(vDir);
    float t = clamp(d.y, -0.2, 1.0);
    vec3 c = mix(uHor, uTop, pow(max(t, 0.0), 0.5));
    float s = max(dot(d, normalize(uSun)), 0.0);
    c += vec3(1.0, 0.92, 0.78) * (pow(s, 600.0) * 2.5 + pow(s, 12.0) * 0.18);
    gl_FragColor = vec4(c, 1.0);
    #include <tonemapping_fragment>
    #include <colorspace_fragment>
  }`;

class World {
  constructor(scene) {
    this.scene = scene;
    this.group = new THREE.Group();     // things placed in world metres
    this.fixed = new THREE.Group();     // things that follow the camera
    const hor = new THREE.Color(0xc9dcea), top = new THREE.Color(0x3f7fc8);
    this.hor = hor;
    this.sky = new THREE.Mesh(
      new THREE.SphereGeometry(50000, 32, 16),
      new THREE.ShaderMaterial({
        uniforms: { uTop: { value: top.clone().convertSRGBToLinear() }, uHor: { value: hor.clone().convertSRGBToLinear() }, uSun: { value: new THREE.Vector3(-0.4, 0.75, 0.35) } },
        vertexShader: 'varying vec3 vDir; void main(){ vDir = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
        fragmentShader: skyFS, side: THREE.BackSide, depthWrite: false,
      }),
    );
    this.sky.renderOrder = -10;
    this.fixed.add(this.sky);
    const gm = new THREE.ShaderMaterial({
      uniforms: {
        uOff: { value: new THREE.Vector2() }, uFog: { value: hor.clone().convertSRGBToLinear() }, uFogK: { value: 4.4e-5 },
        uRw1: { value: new THREE.Vector4(-60, 1440, 18, 1) }, uRw2: { value: new THREE.Vector4(0, 0, 18, 0) },
      },
      vertexShader: groundVS, fragmentShader: groundFS,
    });
    this.ground = new THREE.Mesh(new THREE.PlaneGeometry(120000, 120000, 1, 1).rotateX(-Math.PI / 2), gm);
    this.gm = gm;
    this.fixed.add(this.ground);
    // shadow blob
    const sc = document.createElement('canvas'); sc.width = sc.height = 128;
    const sx = sc.getContext('2d');
    const gr = sx.createRadialGradient(64, 64, 4, 64, 64, 62);
    gr.addColorStop(0, 'rgba(0,0,0,0.55)'); gr.addColorStop(1, 'rgba(0,0,0,0)');
    sx.fillStyle = gr; sx.fillRect(0, 0, 128, 128);
    this.shadow = new THREE.Mesh(new THREE.PlaneGeometry(1, 1).rotateX(-Math.PI / 2),
      new THREE.MeshBasicMaterial({ map: new THREE.CanvasTexture(sc), transparent: true, depthWrite: false }));
    this.fixed.add(this.shadow);
    // clouds
    this.cloudTex = cloudTexture();
    this.clouds = [];
    for (let i = 0; i < 150; i++) {
      const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: this.cloudTex, transparent: true, depthWrite: false, color: 0xffffff, opacity: 0.92 }));
      s.visible = false;
      this.group.add(s);
      this.clouds.push(s);
    }
    // airfield: trees, hangar, tower at both ends
    this.field1 = airfield(-60, 1500, 11);
    this.group.add(this.field1);
    this.field2 = null;
    this.scene.add(this.group, this.fixed);
    this.cloudBase = 1100;
  }

  setVisible(v) { this.group.visible = v; this.fixed.visible = v; }

  setLanding(x0) {
    if (this.field2) { this.group.remove(this.field2); this.field2 = null; }
    if (x0 == null) { this.gm.uniforms.uRw2.value.w = 0; return; }
    this.gm.uniforms.uRw2.value.set(x0, x0 + 1500, 18, 1);
    this.field2 = airfield(x0, 1500, 23);
    this.group.add(this.field2);
  }

  update(xw, h, gearH, camPos) {
    const gy = -(h + gearH);
    this.group.position.set(-xw, gy, 0);
    this.sky.position.copy(camPos);
    this.ground.position.set(camPos.x, gy, camPos.z);
    this.gm.uniforms.uOff.value.set(xw, 0);
    // shadow under the plane
    const k = Math.exp(-h / 90);
    this.shadow.visible = k > 0.02;
    this.shadow.position.set(0, gy + 0.05, 0);
    this.shadow.scale.set(9 + h * 0.05, 1, 13 + h * 0.05);
    this.shadow.material.opacity = k;
    // clouds on a hashed grid around the plane
    const tile = 1700, cx = Math.floor(xw / tile);
    let n = 0;
    for (let i = -7; i <= 16 && n < this.clouds.length; i++) {
      for (let j = -7; j <= 7 && n < this.clouds.length; j++) {
        const tx = cx + i, tz = j;
        if (hash2(tx, tz) > 0.42) continue;
        const s = this.clouds[n++];
        const size = 260 + 520 * hash2(tx + 3, tz + 5);
        s.position.set((tx + hash2(tx + 11, tz)) * tile, this.cloudBase + 220 * hash2(tx, tz + 1), (tz + hash2(tx, tz + 7) - 0.5) * tile);
        s.scale.set(size, size * 0.5, 1);
        s.visible = true;
      }
    }
    for (; n < this.clouds.length; n++) this.clouds[n].visible = false;
  }
}

function hash2(a, b) {
  let h = Math.imul(a | 0, 374761393) ^ Math.imul(b | 0, 668265263);
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  return ((h ^ (h >>> 16)) >>> 0) / 4294967296;
}

function cloudTexture() {
  const c = document.createElement('canvas'); c.width = 256; c.height = 128;
  const x = c.getContext('2d');
  let seed = 3;
  const r = () => { seed = (seed * 16807) % 2147483647; return seed / 2147483647; };
  for (let i = 0; i < 26; i++) {
    const px = 40 + r() * 176, py = 50 + r() * 40 - Math.abs(px - 128) * 0.12, rad = 18 + r() * 30;
    const g = x.createRadialGradient(px, py, 1, px, py, rad);
    g.addColorStop(0, 'rgba(255,255,255,0.55)'); g.addColorStop(1, 'rgba(255,255,255,0)');
    x.fillStyle = g; x.beginPath(); x.arc(px, py, rad, 0, TAU); x.fill();
  }
  const t = new THREE.CanvasTexture(c);
  t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

function airfield(x0, len, seed) {
  const g = new THREE.Group();
  let s = seed;
  const r = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
  // trees
  const cone = new THREE.ConeGeometry(1, 2.6, 7).translate(0, 2.3, 0);
  const trunk = new THREE.CylinderGeometry(0.18, 0.22, 1.2, 5).translate(0, 0.6, 0);
  const tm = new THREE.MeshStandardMaterial({ color: 0x2f5a2c, roughness: 0.9 });
  const km = new THREE.MeshStandardMaterial({ color: 0x5a4632, roughness: 0.9 });
  const N = 420;
  const ti = new THREE.InstancedMesh(cone, tm, N), ki = new THREE.InstancedMesh(trunk, km, N);
  const M = new THREE.Matrix4(), q = new THREE.Quaternion(), sc = new THREE.Vector3(), p = new THREE.Vector3();
  for (let i = 0; i < N; i++) {
    let x, z;
    do {
      const cl = Math.floor(r() * 9);
      x = x0 - 500 + (cl / 9) * (len + 1100) + (r() - 0.5) * 260;
      z = (r() < 0.5 ? -1 : 1) * (95 + r() * 520);
    } while (Math.abs(z) < 80);
    const k = 3 + r() * 4.5;
    p.set(x, 0, z); sc.set(k, k * (0.9 + r() * 0.5), k);
    M.compose(p, q, sc);
    ti.setMatrixAt(i, M); ki.setMatrixAt(i, M);
  }
  g.add(ti, ki);
  // hangar + tower
  const wall = new THREE.MeshStandardMaterial({ color: 0xc8ced3, roughness: 0.7 });
  const roof = new THREE.MeshStandardMaterial({ color: 0x5d6a73, roughness: 0.6 });
  const hg = new THREE.Mesh(new THREE.BoxGeometry(42, 11, 30), wall); hg.position.set(x0 + 380, 5.5, -95); g.add(hg);
  const hr = new THREE.Mesh(new THREE.CylinderGeometry(15.5, 15.5, 42, 16, 1, false, 0, Math.PI).rotateZ(Math.PI / 2).rotateY(Math.PI / 2), roof);
  hr.scale.set(1, 0.35, 1); hr.position.set(x0 + 380, 11, -95); g.add(hr);
  const tw = new THREE.Mesh(new THREE.CylinderGeometry(2.2, 2.6, 16, 10), wall); tw.position.set(x0 + 560, 8, -82); g.add(tw);
  const cab = new THREE.Mesh(new THREE.CylinderGeometry(3.6, 3.0, 3.2, 10), new THREE.MeshStandardMaterial({ color: 0x223644, roughness: 0.15, metalness: 0.6 }));
  cab.position.set(x0 + 560, 17.6, -82); g.add(cab);
  return g;
}

// ---------------------------------------------------------------- wind tunnel backdrop
class Lab {
  constructor(scene) {
    this.group = new THREE.Group();
    const bg = new THREE.Mesh(
      new THREE.SphereGeometry(400, 32, 16),
      new THREE.ShaderMaterial({
        uniforms: {},
        vertexShader: 'varying vec3 vDir; void main(){ vDir = position; gl_Position = projectionMatrix * modelViewMatrix * vec4(position,1.0); }',
        fragmentShader: `varying vec3 vDir; void main(){ vec3 d = normalize(vDir);
          vec3 c = mix(vec3(0.020, 0.032, 0.045), vec3(0.052, 0.078, 0.10), smoothstep(-0.6, 0.6, d.y));
          gl_FragColor = vec4(c, 1.0); }`,
        side: THREE.BackSide, depthWrite: false,
      }),
    );
    bg.renderOrder = -10;
    this.bg = bg;
    this.group.add(bg);
    const grid = new THREE.GridHelper(60, 60, 0x2a4556, 0x18303d);
    grid.position.y = -2.6;
    grid.material.transparent = true; grid.material.opacity = 0.55;
    this.group.add(grid);
    scene.add(this.group);
  }
  setVisible(v) { this.group.visible = v; }
  update(camPos) { this.bg.position.copy(camPos); }
}

// ---------------------------------------------------------------- renderer + camera
function createView(canvas) {
  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
  } catch (e) {
    return null;
  }
  if (!renderer.getContext()) return null;
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 0.95;
  renderer.localClippingEnabled = true;
  const scene = new THREE.Scene();
  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.4;
  const camera = new THREE.PerspectiveCamera(40, 1, 0.05, 90000);
  camera.position.set(8, 3.4, 9);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(-0.5, 0.4, 0);
  controls.minDistance = 1.5;
  controls.maxDistance = 140;
  controls.maxPolarAngle = Math.PI * 0.92;
  const hemi = new THREE.HemisphereLight(0xe2efff, 0x56604c, 0.9);
  const sun = new THREE.DirectionalLight(0xfff4e6, 2.4);
  sun.position.set(-20, 40, 18);
  const fill = new THREE.DirectionalLight(0xbfd8ff, 0.5);
  fill.position.set(15, 6, -20);
  scene.add(hemi, sun, fill);
  const planeRoot = new THREE.Group();
  scene.add(planeRoot);
  return { THREE, renderer, scene, camera, controls, planeRoot, mats: makeMaterials(), hemi };
}

class CameraRig {
  constructor(camera, controls) {
    this.cam = camera; this.ctl = controls;
    this.anim = null;
    controls.addEventListener('start', () => { this.anim = null; });
  }
  goTo(pos, target, dur = 0.9) {
    this.anim = {
      p0: this.cam.position.clone(), t0: this.ctl.target.clone(),
      p1: new THREE.Vector3(...pos), t1: new THREE.Vector3(...target), k: 0, dur,
    };
  }
  update(dt) {
    const a = this.anim;
    if (!a) return;
    a.k = Math.min(1, a.k + dt / a.dur);
    const e = a.k * a.k * (3 - 2 * a.k);
    this.cam.position.lerpVectors(a.p0, a.p1, e);
    this.ctl.target.lerpVectors(a.t0, a.t1, e);
    if (a.k >= 1) this.anim = null;
  }
}

export { PlaneModel, TrailRenderer, World, Lab, createView, CameraRig };
