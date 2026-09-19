// Planar quadcopter: a rigid body in the x-z plane with two rotors.
// State: position (x, z), velocity (vx, vz), pitch angle theta (positive tilts
// thrust towards +x) and pitch rate omega. Rotors don't respond instantly:
// each follows its command with a first-order lag and is limited to
// 0..maxThrust.

import { PID } from "./pid.js";

export const G = 9.81;

export const PARAMS = {
  mass: 1.2,          // kg
  arm: 0.2,           // m, rotor to centre
  inertia: 0.015,     // kg m^2 about the pitch axis
  maxThrust: 12,      // N per rotor, thrust-to-weight about 2
  motorTau: 0.04,     // s, rotor spin-up time constant
  drag: 0.3,          // N per m/s, linear air drag
};

export function initialState() {
  return { x: 0, z: 0, vx: 0, vz: 0, theta: 0, omega: 0, t1: 0, t2: 0, time: 0 };
}

// One explicit-Euler step of the rigid body. dt should be about 1 ms.
export function step(s, cmd1, cmd2, dt, wind = 0, p = PARAMS) {
  const clamp = (v) => Math.min(p.maxThrust, Math.max(0, v));
  const t1 = s.t1 + (clamp(cmd1) - s.t1) * (dt / p.motorTau);
  const t2 = s.t2 + (clamp(cmd2) - s.t2) * (dt / p.motorTau);
  const thrust = t1 + t2;

  const ax = (thrust * Math.sin(s.theta) - p.drag * (s.vx - wind)) / p.mass;
  const az = (thrust * Math.cos(s.theta) - p.drag * s.vz) / p.mass - G;
  // A stronger left rotor lifts the left side and tilts thrust towards +x.
  const alpha = ((t1 - t2) * p.arm) / p.inertia;

  const n = { ...s, t1, t2, time: s.time + dt };
  n.vx += ax * dt;
  n.vz += az * dt;
  n.omega += alpha * dt;
  n.x += n.vx * dt;
  n.z += n.vz * dt;
  n.theta += n.omega * dt;
  if (n.z <= 0) {                  // the ground
    n.z = 0;
    n.vz = Math.max(0, n.vz);
    if (thrust < p.mass * G) { n.vx = 0; n.omega = 0; n.theta = 0; }
  }
  return n;
}

// Cascaded control, as on most real flight controllers:
//   altitude error -> vertical acceleration -> collective thrust
//   horizontal error -> horizontal acceleration -> pitch angle to hold
//   pitch error (inner, fastest loop) -> torque -> difference between rotors
// The inner loop has to be several times faster than the outer ones,
// otherwise the outer loops ask for angles the drone can't reach in time.
export class Controller {
  constructor(p = PARAMS) {
    this.p = p;
    this.altitude = new PID({ kp: 5, ki: 0.2, kd: 4, min: -6, max: 6 });          // -> m/s^2
    this.position = new PID({ kp: 2, ki: 0.1, kd: 2.5, min: -5, max: 5 });       // -> m/s^2
    this.attitude = new PID({ kp: 60, ki: 0, kd: 8, min: -200, max: 200, dFilterHz: 40 }); // -> rad/s^2
    this.maxTilt = (30 * Math.PI) / 180;
  }

  loops() {
    return { altitude: this.altitude, position: this.position, attitude: this.attitude };
  }

  reset() {
    for (const l of Object.values(this.loops())) l.reset();
  }

  // Returns rotor commands [left, right] and the pitch angle it asked for.
  update(target, meas, dt) {
    const azCmd = this.altitude.update(target.z, meas.z, dt);
    const axCmd = this.position.update(target.x, meas.x, dt);

    const thetaCmd = Math.max(-this.maxTilt, Math.min(this.maxTilt, Math.atan2(axCmd, G + azCmd)));
    const alphaCmd = this.attitude.update(thetaCmd, meas.theta, dt);

    const collective = (this.p.mass * (G + azCmd)) / Math.max(0.5, Math.cos(meas.theta));
    const diff = (alphaCmd * this.p.inertia) / this.p.arm;   // left minus right
    return { cmd: [collective / 2 + diff / 2, collective / 2 - diff / 2], thetaCmd };
  }
}

// Overshoot and 5 % settling time of a step from `from` to `to`.
export function stepMetrics(samples, from, to, tStart) {
  const span = to - from;
  if (Math.abs(span) < 1e-6 || samples.length === 0) return null;
  let peak = 0;
  let settled = null;
  for (const { t, v } of samples) {
    const progress = (v - from) / span;
    peak = Math.max(peak, progress);
    const inBand = Math.abs(progress - 1) <= 0.05;
    if (inBand && settled === null) settled = t;
    if (!inBand) settled = null;
  }
  return {
    overshootPct: Math.max(0, (peak - 1) * 100),
    settlingS: settled === null ? null : settled - tStart,
  };
}
