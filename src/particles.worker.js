// ============================================================================
// particles.worker.js: moves the air particles off the main thread. Each step
// it hands back one trail segment per particle (a, b, speed, fresh).
// ============================================================================
import { planeLayout } from './layout.js';
import { FlowField, Particles } from './flow.js';

let flow = null, parts = null;
self.onmessage = (e) => {
  const m = e.data;
  try {
    if (m.type === 'init') {
      const layout = planeLayout(m.design);
      flow = new FlowField(layout);
      flow.setState(m.state);
      parts = new Particles(m.count);
      parts.focus = m.focus;
      parts.attach(flow);
    } else if (m.type === 'state') {
      if (!flow) return;
      flow.setState(m.state);
      if (parts) { if (parts.focus !== m.focus) parts.refocus(m.focus); else if (m.reseed) parts.updateSeeds(); }
    } else if (m.type === 'step') {
      const buf = new Float32Array(m.buf);
      let n = 0;
      if (parts) {
        n = parts.n;
        parts.step(m.dt, (i, ax, ay, az, bx, by, bz, sp, fr) => {
          const o = i * 8; buf[o] = ax; buf[o+1] = ay; buf[o+2] = az; buf[o+3] = bx; buf[o+4] = by; buf[o+5] = bz; buf[o+6] = sp; buf[o+7] = fr;
        });
      }
      self.postMessage({ type: 'seg', buf: m.buf, n }, [m.buf]);
    }
  } catch (err) {
    self.postMessage({ type: 'error', message: String((err && err.stack) || err) });
  }
};
