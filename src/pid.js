// PID controller with the three fixes every real one needs:
//  - derivative on the measurement, not the error, so a setpoint jump does
//    not produce a spike ("derivative kick")
//  - a low-pass filter on the derivative, because differentiating noise
//    amplifies it
//  - anti-windup: the integrator stops growing while the output is saturated

export class PID {
  constructor({ kp = 0, ki = 0, kd = 0, min = -Infinity, max = Infinity, dFilterHz = 20 } = {}) {
    Object.assign(this, { kp, ki, kd, min, max, dFilterHz });
    this.antiWindup = true;
    this.derivativeOnMeasurement = true;
    this.reset();
  }

  reset() {
    this.integral = 0;
    this.prevMeasurement = null;
    this.prevError = null;
    this.dFiltered = 0;
  }

  update(setpoint, measurement, dt) {
    const error = setpoint - measurement;

    let raw;
    if (this.prevMeasurement === null) raw = 0;
    else if (this.derivativeOnMeasurement) raw = -(measurement - this.prevMeasurement) / dt;
    else raw = (error - this.prevError) / dt;
    this.prevMeasurement = measurement;
    this.prevError = error;

    const alpha = dt / (dt + 1 / (2 * Math.PI * this.dFilterHz));
    this.dFiltered += alpha * (raw - this.dFiltered);

    const unsat = this.kp * error + this.ki * this.integral + this.kd * this.dFiltered;
    const out = Math.min(this.max, Math.max(this.min, unsat));

    // Only integrate if that would not push further into saturation.
    const pushingOut = (out === this.max && error > 0) || (out === this.min && error < 0);
    if (!this.antiWindup || !pushingOut) this.integral += error * dt;

    return out;
  }
}
