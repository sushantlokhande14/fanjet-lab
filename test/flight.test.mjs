import { test } from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_DESIGN, K, FlightSim } from '../src/physics.js';

function fly(design, dt = 0.25, limit = 4 * 3600) {
  const sim = new FlightSim(design);
  const seen = new Set();
  let lastE = sim.s.energyJ;
  let rising = 0;
  sim.start();
  while (!['landed', 'aborted'].includes(sim.s.phase) && sim.s.t < limit) {
    sim.step(dt);
    seen.add(sim.s.phase);
    if (sim.s.energyJ > lastE + 1e-6) rising++;
    lastE = sim.s.energyJ;
  }
  return { sim, seen, rising };
}

test('a full flight: takeoff, climb, cruise, descent, landing on the second runway', () => {
  const { sim, seen, rising } = fly(DEFAULT_DESIGN);
  const s = sim.s;
  assert.equal(s.phase, 'landed');
  assert.ok(!s.fieldLanding, 'landed on a runway, not in a field');
  for (const p of ['roll', 'climb', 'cruise', 'descent', 'approach', 'flare', 'rollout']) assert.ok(seen.has(p), `went through ${p}`);
  assert.equal(rising, 0, 'the battery never charges itself');
  assert.ok(sim.soc > K.reserve - 0.05 && sim.soc < K.reserve + 0.1, `landed with ${(sim.soc * 100).toFixed(1)}% battery`);
  const pred = sim.pred.range;
  assert.ok(Math.abs(s.x - pred) < 0.1 * pred, `flew ${(s.x / 1000).toFixed(1)} km, estimate ${(pred / 1000).toFixed(1)} km`);
});

test('a design without push stays on the runway', () => {
  const { sim } = fly({ ...DEFAULT_DESIGN, swirl: 'none' });
  assert.equal(sim.s.phase, 'aborted');
  assert.ok(sim.s.h === 0);
});
