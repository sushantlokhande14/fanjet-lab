import { test } from 'node:test';
import assert from 'node:assert/strict';
import { DEFAULT_DESIGN, K, buildAircraft, levelFlight, predictMission } from '../src/physics.js';

const D = DEFAULT_DESIGN;

test('the original idea takes off and flies a useful distance', () => {
  const p = predictMission(D);
  assert.ok(p.ok, p.notes.map((n) => n.text).join(' / '));
  assert.ok(p.takeoffRoll > 100 && p.takeoffRoll < 600, `takeoff roll ${p.takeoffRoll.toFixed(0)} m`);
  assert.ok(p.range > 100e3 && p.range < 200e3, `range ${(p.range / 1000).toFixed(1)} km`);
  assert.ok(p.cruiseAlt >= D.cruiseAltM - 1, 'reaches cruise height');
});

test('level flight: the engines push exactly as hard as the drag', () => {
  const ac = buildAircraft(D);
  const lf = levelFlight(ac, 220 / 3.6, 1500);
  assert.ok(lf.ok);
  const T = ac.nEng * lf.eng.T;
  assert.ok(Math.abs(T - lf.D) < 0.01 * lf.D, `thrust ${T.toFixed(0)} N vs drag ${lf.D.toFixed(0)} N`);
  assert.ok(lf.etaOverall > 0 && lf.etaOverall < 1);
});

test('the best speed for range is below top speed and really does fly farther', () => {
  const p = predictMission(D);
  assert.ok(p.bestRangeV < p.vmax);
  const ac = buildAircraft(D);
  const perJoule = (V) => V / levelFlight(ac, V, p.cruiseAlt).Pbatt;
  assert.ok(perJoule(p.bestRangeV) > perJoule(0.95 * p.vmax));
  assert.ok(perJoule(p.bestRangeV) >= perJoule(p.bestRangeV * 1.08) * 0.999);
});

test('a design that cannot push says so instead of reporting a range', () => {
  const p = predictMission({ ...D, swirl: 'none' });
  assert.equal(p.ok, false);
  assert.equal(p.fail, 'thrust');
  assert.equal(p.range, 0);
  assert.equal(p.notes[0].level, 'bad');
});

test('overloading the airframe is flagged', () => {
  const p = predictMission({ ...D, batteryKg: 900, people: 6, powerKW: 600 });
  assert.ok(p.ac.m > K.mtowKg);
  assert.ok(p.notes.some((n) => n.level === 'bad' && /Too heavy/.test(n.text)));
});

test('more battery energy per kilogram means more range', () => {
  const a = predictMission({ ...D, whPerKg: 250 });
  const b = predictMission({ ...D, whPerKg: 400 });
  assert.ok(b.range > 1.4 * a.range, `${(a.range / 1000).toFixed(0)} km -> ${(b.range / 1000).toFixed(0)} km`);
});
