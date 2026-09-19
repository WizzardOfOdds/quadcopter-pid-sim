// Test cases shared by the Node runner (quad.test.mjs) and the browser (tests/index.html).
import { PID } from "../src/pid.js";
import { Controller, G, PARAMS, initialState, step, stepMetrics } from "../src/quad.js";

function assert(cond, msg) {
  if (!cond) throw new Error(msg);
}

// Fly for `seconds` towards `target`, starting in a hover at `start`.
function fly({ target, start = { x: 0, z: 3 }, seconds = 10, gains, antiWindup = true, wind = () => 0 }) {
  let s = { ...initialState(), ...start, t1: (PARAMS.mass * G) / 2, t2: (PARAMS.mass * G) / 2 };
  const c = new Controller();
  for (const [loop, [kp, ki, kd]] of Object.entries(gains ?? {})) Object.assign(c.loops()[loop], { kp, ki, kd });
  for (const pid of Object.values(c.loops())) pid.antiWindup = antiWindup;
  const xs = [], zs = [];
  let cmd = [0, 0], maxTilt = 0;
  for (let i = 0; i < seconds * 1000; i++) {
    if (i % 5 === 0) cmd = c.update(target, s, 0.005).cmd;
    s = step(s, cmd[0], cmd[1], 0.001, wind(s.time));
    if (i % 10 === 0) { xs.push({ t: s.time, v: s.x }); zs.push({ t: s.time, v: s.z }); }
    maxTilt = Math.max(maxTilt, Math.abs(s.theta));
  }
  return { s, xs, zs, maxTiltDeg: (maxTilt * 180) / Math.PI };
}

export const cases = [
  ["hover stays put", () => {
    const { s } = fly({ target: { x: 0, z: 3 }, seconds: 5 });
    assert(Math.abs(s.z - 3) < 0.01 && Math.abs(s.x) < 0.01, `drifted to x=${s.x}, z=${s.z}`);
  }],

  ["altitude step: small overshoot, settles within 4 s", () => {
    const { zs } = fly({ target: { x: 0, z: 5 } });
    const m = stepMetrics(zs, 3, 5, 0);
    assert(m.overshootPct < 10, `overshoot ${m.overshootPct}%`);
    assert(m.settlingS !== null && m.settlingS < 4, `settling ${m.settlingS}`);
  }],

  ["position step: settles, and tilt stays near the 30 degree limit", () => {
    const { xs, maxTiltDeg } = fly({ target: { x: 3, z: 3 } });
    const m = stepMetrics(xs, 0, 3, 0);
    assert(m.settlingS !== null && m.settlingS < 6, `settling ${m.settlingS}`);
    assert(maxTiltDeg < 36, `tilted ${maxTiltDeg} degrees`);
  }],

  ["no derivative: much more overshoot", () => {
    const withD = stepMetrics(fly({ target: { x: 0, z: 5 } }).zs, 3, 5, 0);
    const noD = stepMetrics(fly({ target: { x: 0, z: 5 }, gains: { altitude: [5, 0.2, 0] } }).zs, 3, 5, 0);
    assert(noD.overshootPct > withD.overshootPct + 30, `with D ${withD.overshootPct}, without ${noD.overshootPct}`);
  }],

  ["anti-windup limits overshoot on a saturated climb", () => {
    const run = (antiWindup) => stepMetrics(fly({
      target: { x: 0, z: 7 }, start: { x: 0, z: 1 }, seconds: 15, antiWindup,
      gains: { altitude: [5, 6, 4] },
    }).zs, 1, 7, 0).overshootPct;
    const on = run(true), off = run(false);
    assert(off > 1.5 * on, `overshoot with anti-windup ${on}%, without ${off}%`);
  }],

  ["recovers from a gust", () => {
    const gust = (t) => (t > 2 && t < 3.5 ? 6 : 0);
    const { s } = fly({ target: { x: 0, z: 3 }, seconds: 10, wind: gust });
    assert(Math.abs(s.x) < 0.1, `still ${s.x} m off after the gust`);
  }],

  ["PID output is clamped", () => {
    const pid = new PID({ kp: 100, min: -1, max: 1 });
    assert(pid.update(10, 0, 0.01) === 1, "not clamped high");
    assert(pid.update(-10, 0, 0.01) === -1, "not clamped low");
  }],

  ["derivative on measurement: no kick on a setpoint jump", () => {
    const kick = (onMeasurement) => {
      const pid = new PID({ kd: 1, dFilterHz: 1000 });
      pid.derivativeOnMeasurement = onMeasurement;
      pid.update(0, 0, 0.01);
      return Math.abs(pid.update(1, 0, 0.01));   // setpoint jumps, measurement does not move
    };
    assert(kick(true) === 0, "derivative on measurement reacted to the setpoint");
    assert(kick(false) > 10, "derivative on error should spike");
  }],

  ["stepMetrics", () => {
    const samples = [0, 0.5, 1.2, 1.1, 1.0, 1.0].map((v, i) => ({ t: i, v }));
    const m = stepMetrics(samples, 0, 1, 0);
    assert(Math.abs(m.overshootPct - 20) < 1e-9, `overshoot ${m.overshootPct}`);
    assert(m.settlingS === 4, `settling ${m.settlingS}`);
  }],
];
