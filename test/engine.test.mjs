import { test } from 'node:test';
import assert from 'node:assert/strict';
import { isa, DEFAULT_DESIGN, engineGeometry, solveEngine } from '../src/physics.js';

const rho0 = isa(0).rho;
const geo = (patch = {}) => engineGeometry({ ...DEFAULT_DESIGN, ...patch });
// one engine's shaft power at full throttle: 300 kW over six engines
const POWER = 50e3;
const CRUISE = 220 / 3.6;

const close = (a, b, rel, msg) => assert.ok(Math.abs(a - b) <= rel * Math.abs(b), `${msg}: ${a} vs ${b}`);

test('standard atmosphere matches the textbook values', () => {
  close(rho0, 1.225, 1e-3, 'sea-level density');
  const top = isa(11000);
  close(top.T, 216.65, 1e-4, 'tropopause temperature');
  close(top.rho, 0.3639, 1e-3, 'tropopause density');
});

test('the operating point passes the same air through the fans and the nozzle', () => {
  for (const V0 of [0, 30, CRUISE]) {
    const g = geo();
    const e = solveEngine(g, 'stators', V0, rho0, POWER);
    assert.ok(e.mdot > 0, 'air is flowing');
    close(rho0 * g.An * e.Ve, e.mdot, 1e-3, `nozzle continuity at ${V0} m/s`);
  }
});

test('every watt of shaft power is accounted for', () => {
  for (const swirl of ['stators', 'contra']) {
    for (const V0 of [0, 30, CRUISE]) {
      const e = solveEngine(geo({ swirl }), swirl, V0, rho0, POWER);
      const parts = e.usefulW + e.wakeLossW + e.swirlLossW + e.fanLossW + e.ductLossW + e.nozzleLossW;
      close(parts, e.shaftW, 1e-9, `${swirl} at ${V0} m/s`);
    }
  }
});

test('propulsive efficiency is 2 V0 / (Ve + V0)', () => {
  const e = solveEngine(geo(), 'stators', CRUISE, rho0, 0.4 * POWER);
  close(e.etaProp, (2 * CRUISE) / (e.Ve + CRUISE), 1e-12, 'formula');
  close(e.etaProp, e.usefulW / (e.usefulW + e.wakeLossW), 1e-9, 'from the energy split');
});

test('a smaller nozzle makes a faster jet', () => {
  const jet = (nozzleRatio) => solveEngine(geo({ nozzleRatio }), 'stators', CRUISE, rho0, 0.4 * POWER).Ve;
  const speeds = [0.55, 0.7, 0.85, 1.0].map(jet);
  for (let i = 1; i < speeds.length; i++) assert.ok(speeds[i] < speeds[i - 1], `jet speeds ${speeds.map((v) => v.toFixed(1))}`);
});

test('without stators the swirl piles up until the fans stall', () => {
  const none = solveEngine(geo({ swirl: 'none' }), 'none', 0, rho0, POWER);
  const stators = solveEngine(geo(), 'stators', 0, rho0, POWER);
  assert.ok(none.T < 1, `no push without stators, got ${none.T.toFixed(1)} N`);
  assert.ok(stators.T > 500, `real push with stators, got ${stators.T.toFixed(0)} N`);
});

test('stators take the spin out of the jet, counter-spinning fans mostly cancel it', () => {
  const st = solveEngine(geo(), 'stators', CRUISE, rho0, 0.4 * POWER);
  const co = solveEngine(geo({ swirl: 'contra' }), 'contra', CRUISE, rho0, 0.4 * POWER);
  assert.equal(st.swirlLossW, 0);
  assert.ok(co.swirlLossW > 0 && co.swirlLossW < 0.15 * co.shaftW, `contra swirl loss ${(co.swirlLossW / co.shaftW).toFixed(3)}`);
});
