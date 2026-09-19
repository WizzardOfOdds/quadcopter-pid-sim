import { Controller, G, PARAMS, initialState, step, stepMetrics } from "./quad.js";

const PHYSICS_DT = 0.001;
const CONTROL_EVERY = 5;               // 200 Hz
const WORLD = { xMin: -6, xMax: 6, zMax: 8 };
const HISTORY_S = 15;

const LOOPS = {
  altitude: { label: "Altitude", note: "error in m → vertical accel.", ranges: { kp: 20, ki: 10, kd: 15 } },
  position: { label: "Position", note: "error in m → horizontal accel.", ranges: { kp: 6, ki: 2, kd: 6 } },
  attitude: { label: "Attitude", note: "inner loop: pitch → torque", ranges: { kp: 300, ki: 50, kd: 40 } },
};

const PRESETS = {
  tuned: {
    label: "Tuned",
    note: "Reasonable gains: the inner attitude loop is much faster than the outer loops.",
    gains: { altitude: [5, 0.2, 4], position: [2, 0.1, 2.5], attitude: [60, 0, 8] },
  },
  noD: {
    label: "No derivative",
    note: "Without D there is no damping. Each move overshoots and rings for a long time.",
    gains: { altitude: [5, 0.2, 0], position: [2, 0.1, 0], attitude: [60, 0, 8] },
  },
  slowInner: {
    label: "Slow inner loop",
    note: "Attitude loop too slow for the position loop on top of it. The drone tilts late, and the outer loop overcorrects.",
    gains: { altitude: [5, 0.2, 4], position: [3, 0.1, 2.5], attitude: [20, 0, 4] },
  },
  twitchy: {
    label: "Too aggressive",
    note: "Attitude gain five times too high. With 40 ms of rotor lag the inner loop can't keep up with itself and settles into a fast wobble.",
    gains: { altitude: [5, 0.2, 4], position: [2, 0.1, 2.5], attitude: [300, 0, 15] },
  },
  windup: {
    label: "Integral windup",
    note: "High altitude I-gain with anti-windup off. Climbing saturates the rotors, the integral keeps growing, and the drone overshoots badly.",
    gains: { altitude: [5, 6, 4], position: [2, 0.1, 2.5], attitude: [60, 0, 8] },
    options: { antiWindup: false },
  },
};

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

let state, ctrl, target, history, stepInfo, gustUntil, tick, thetaCmd, crashed;

function reset() {
  state = initialState();
  ctrl = new Controller();
  applyGainsFromUI();
  applyOptions();
  target = { x: 0, z: 3 };
  history = [];
  stepInfo = { from: { x: 0, z: 0 }, to: { ...target }, t0: 0 };
  gustUntil = -1;
  tick = 0;
  thetaCmd = 0;
  crashed = false;
}

// ---------------------------------------------------------------- controls

function buildGainControls() {
  const box = $("gains");
  for (const [loop, meta] of Object.entries(LOOPS)) {
    const fs = document.createElement("fieldset");
    fs.innerHTML = `<legend>${meta.label} <small>${meta.note}</small></legend>`;
    for (const k of ["kp", "ki", "kd"]) {
      const row = document.createElement("div");
      row.className = "gain";
      const max = meta.ranges[k];
      row.innerHTML = `<span>${k[1].toUpperCase()}</span>
        <input type="range" id="${loop}-${k}" min="0" max="${max}" step="${max / 600}"
               aria-label="${meta.label} ${k[1].toUpperCase()} gain">
        <span id="${loop}-${k}-v"></span>`;
      fs.appendChild(row);
    }
    box.appendChild(fs);
  }
  box.addEventListener("input", () => { applyGainsFromUI(); $("preset").value = ""; showNote(""); });
}

function setGains(gains) {
  for (const [loop, [kp, ki, kd]] of Object.entries(gains)) {
    $(`${loop}-kp`).value = kp;
    $(`${loop}-ki`).value = ki;
    $(`${loop}-kd`).value = kd;
  }
  applyGainsFromUI();
}

function applyGainsFromUI() {
  if (!ctrl) return;
  for (const [loop, pid] of Object.entries(ctrl.loops())) {
    for (const k of ["kp", "ki", "kd"]) {
      const v = parseFloat($(`${loop}-${k}`).value);
      pid[k] = v;
      $(`${loop}-${k}-v`).textContent = v >= 10 ? v.toFixed(0) : v.toFixed(2);
    }
  }
}

function applyOptions() {
  for (const pid of Object.values(ctrl.loops())) {
    pid.antiWindup = $("antiWindup").checked;
    pid.derivativeOnMeasurement = $("dOnMeas").checked;
  }
}

function showNote(text) { $("presetNote").textContent = text; }

function loadPreset(key) {
  const p = PRESETS[key];
  setGains(p.gains);
  $("antiWindup").checked = p.options?.antiWindup ?? true;
  applyOptions();
  showNote(p.note);
  if (key === "windup") setTarget({ x: target.x, z: target.z < 4 ? 7 : 1 });
}

function setTarget(t) {
  stepInfo = { from: { x: state.x, z: state.z }, to: { ...t }, t0: state.time };
  target = t;
}

// ---------------------------------------------------------------- simulation

const rand = () => {                              // standard normal, Box-Muller
  const u = 1 - Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random());
};

let cmd = [0, 0];

function advance(seconds) {
  if (crashed) return;
  const steps = Math.round(seconds / PHYSICS_DT);
  for (let i = 0; i < steps; i++) {
    if (Math.abs(state.theta) > Math.PI / 2 || Math.abs(state.x) > 50) {
      crashed = true;                           // upside down or gone: stop and say so
      break;
    }
    if (tick % CONTROL_EVERY === 0) {
      const noisy = $("noise").checked;
      const meas = {
        x: state.x + (noisy ? 0.05 * rand() : 0),
        z: state.z + (noisy ? 0.05 * rand() : 0),
        theta: state.theta + (noisy ? 0.01 * rand() : 0),
      };
      const out = ctrl.update(target, meas, PHYSICS_DT * CONTROL_EVERY);
      cmd = out.cmd;
      thetaCmd = out.thetaCmd;
    }
    let wind = $("wind").checked ? 3 : 0;
    if (state.time < gustUntil) wind += 6;
    state = step(state, cmd[0], cmd[1], PHYSICS_DT, wind);
    tick++;
    if (tick % 10 === 0) history.push({ t: state.time, x: state.x, z: state.z, tx: target.x, tz: target.z });
  }
  while (history.length && history[0].t < state.time - HISTORY_S) history.shift();
}

// ---------------------------------------------------------------- drawing

function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function worldToPx(w, h) {
  const groundPx = 24;
  const sx = w / (WORLD.xMax - WORLD.xMin);
  const sz = (h - groundPx) / WORLD.zMax;
  const s = Math.min(sx, sz);
  const ox = w / 2 - ((WORLD.xMin + WORLD.xMax) / 2) * s;
  return {
    s,
    x: (x) => ox + x * s,
    y: (z) => h - groundPx - z * s,
    inv: (px, py) => ({ x: (px - ox) / s, z: (h - groundPx - py) / s }),
  };
}

function drawWorld() {
  const { ctx, w, h } = fitCanvas($("world"));
  const m = worldToPx(w, h);
  ctx.fillStyle = css("--sky");
  ctx.fillRect(0, 0, w, h);
  ctx.fillStyle = css("--ground");
  ctx.fillRect(0, m.y(0), w, h - m.y(0));

  // grid, one line per metre
  ctx.strokeStyle = css("--line");
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = Math.ceil(WORLD.xMin); x <= WORLD.xMax; x++) { ctx.moveTo(m.x(x), 0); ctx.lineTo(m.x(x), m.y(0)); }
  for (let z = 1; z <= WORLD.zMax; z++) { ctx.moveTo(0, m.y(z)); ctx.lineTo(w, m.y(z)); }
  ctx.stroke();

  // target
  ctx.strokeStyle = css("--target");
  ctx.lineWidth = 1.5;
  const tx = m.x(target.x), ty = m.y(target.z);
  ctx.beginPath();
  ctx.arc(tx, ty, 7, 0, 2 * Math.PI);
  ctx.moveTo(tx - 12, ty); ctx.lineTo(tx + 12, ty);
  ctx.moveTo(tx, ty - 12); ctx.lineTo(tx, ty + 12);
  ctx.stroke();

  // trail
  ctx.strokeStyle = css("--muted");
  ctx.globalAlpha = 0.4;
  ctx.beginPath();
  history.forEach((p, i) => (i ? ctx.lineTo(m.x(p.x), m.y(p.z)) : ctx.moveTo(m.x(p.x), m.y(p.z))));
  ctx.stroke();
  ctx.globalAlpha = 1;

  // drone: body rotated so that positive pitch leans the thrust towards +x
  const arm = PARAMS.arm * m.s * 2.2;
  ctx.save();
  ctx.translate(m.x(state.x), m.y(state.z));
  ctx.rotate(state.theta);
  ctx.setLineDash([4, 4]);
  ctx.strokeStyle = css("--muted");
  ctx.beginPath();
  ctx.rotate(thetaCmd - state.theta);
  ctx.moveTo(-arm * 1.3, 0); ctx.lineTo(arm * 1.3, 0);
  ctx.rotate(state.theta - thetaCmd);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.strokeStyle = css("--ink");
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(-arm, 0); ctx.lineTo(arm, 0);
  ctx.stroke();
  ctx.fillStyle = css("--ink");
  ctx.fillRect(-7, -4, 14, 8);
  for (const [side, thrust] of [[-1, state.t1], [1, state.t2]]) {
    const len = (thrust / PARAMS.maxThrust) * 26;
    ctx.fillStyle = css("--ink");
    ctx.fillRect(side * arm - 10, -6, 20, 3);
    ctx.fillStyle = css("--accent");
    ctx.globalAlpha = 0.5;
    ctx.fillRect(side * arm - 2, 2, 4, len);
    ctx.globalAlpha = 1;
  }
  ctx.restore();

  ctx.fillStyle = css("--muted");
  ctx.font = "12px system-ui, sans-serif";
  ctx.fillText(`x ${state.x.toFixed(2)} m   z ${state.z.toFixed(2)} m   pitch ${(state.theta * 180 / Math.PI).toFixed(1)}°`, 8, 16);
  if (state.time < gustUntil) ctx.fillText("gust →", w - 60, 16);
  if (crashed) {
    ctx.fillStyle = css("--target");
    ctx.font = "600 16px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("Crashed: the drone flipped over. Press Reset.", w / 2, h / 2);
    ctx.textAlign = "start";
  }
}

function drawPlot() {
  const { ctx, w, h } = fitCanvas($("plot"));
  ctx.clearRect(0, 0, w, h);
  const pad = { l: 34, r: 8, t: 26, b: 20 };
  const t1 = state.time, t0 = t1 - HISTORY_S;
  const vals = history.flatMap((p) => [p.x, p.z, p.tx, p.tz]);
  const lo = Math.min(-1, ...vals), hi = Math.max(4, ...vals);
  const X = (t) => pad.l + ((t - t0) / HISTORY_S) * (w - pad.l - pad.r);
  const Y = (v) => pad.t + (1 - (v - lo) / (hi - lo)) * (h - pad.t - pad.b);

  ctx.strokeStyle = css("--line");
  ctx.fillStyle = css("--muted");
  ctx.font = "11px system-ui, sans-serif";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let v = Math.ceil(lo); v <= hi; v++) {
    ctx.moveTo(pad.l, Y(v)); ctx.lineTo(w - pad.r, Y(v));
    ctx.fillText(`${v}`, 8, Y(v) + 4);
  }
  ctx.stroke();
  ctx.fillText("m", 8, pad.t - 8);
  ctx.fillText(`last ${HISTORY_S} s`, w - 60, h - 5);

  const line = (key, color, dash) => {
    ctx.strokeStyle = color;
    ctx.setLineDash(dash);
    ctx.lineWidth = dash.length ? 1 : 1.8;
    ctx.beginPath();
    history.forEach((p, i) => (i ? ctx.lineTo(X(p.t), Y(p[key])) : ctx.moveTo(X(p.t), Y(p[key]))));
    ctx.stroke();
    ctx.setLineDash([]);
  };
  line("tz", css("--plot-a"), [4, 3]);
  line("tx", css("--plot-b"), [4, 3]);
  line("z", css("--plot-a"), []);
  line("x", css("--plot-b"), []);

  ctx.font = "12px system-ui, sans-serif";
  ctx.fillStyle = css("--plot-a");
  ctx.fillText("altitude z", pad.l, 12);
  ctx.fillStyle = css("--plot-b");
  ctx.fillText("position x", pad.l + 76, 12);
  ctx.fillStyle = css("--muted");
  ctx.fillText("dashed: target", pad.l + 152, 12);
}

function drawMetrics() {
  const samples = history.filter((p) => p.t >= stepInfo.t0);
  const fmt = (key, label) => {
    const r = stepMetrics(samples.map((p) => ({ t: p.t, v: p[key] })), stepInfo.from[key], stepInfo.to[key], stepInfo.t0);
    if (!r) return `<span>${label}: <b>-</b></span>`;
    const settle = r.settlingS === null ? "not settled" : `settles in ${r.settlingS.toFixed(1)} s`;
    return `<span>${label}: <b>${r.overshootPct.toFixed(0)} %</b> overshoot, <b>${settle}</b></span>`;
  };
  $("metrics").innerHTML = `${fmt("z", "Altitude step")} ${fmt("x", "Position step")}`;
}

// ---------------------------------------------------------------- wiring

function init() {
  buildGainControls();
  const sel = $("preset");
  sel.innerHTML = `<option value="">Custom</option>` +
    Object.entries(PRESETS).map(([k, p]) => `<option value="${k}">${p.label}</option>`).join("");
  reset();
  sel.value = "tuned";
  loadPreset("tuned");

  sel.addEventListener("change", () => sel.value && loadPreset(sel.value));
  $("reset").addEventListener("click", () => { const p = sel.value; reset(); if (p) loadPreset(p); });
  $("gust").addEventListener("click", () => { gustUntil = state.time + 1.5; });
  $("antiWindup").addEventListener("change", applyOptions);
  $("dOnMeas").addEventListener("change", applyOptions);
  $("world").addEventListener("pointerdown", (e) => {
    const r = e.currentTarget.getBoundingClientRect();
    const m = worldToPx(r.width, r.height);
    const p = m.inv(e.clientX - r.left, e.clientY - r.top);
    setTarget({
      x: Math.max(WORLD.xMin + 0.5, Math.min(WORLD.xMax - 0.5, p.x)),
      z: Math.max(0.5, Math.min(WORLD.zMax - 0.5, p.z)),
    });
  });

  // ?demo flies a short scripted route before the first frame (used for the README screenshot).
  if (new URLSearchParams(location.search).has("demo")) {
    for (const [seconds, t] of [[5, { x: 0, z: 3 }], [5, { x: 3, z: 5 }], [3.5, { x: -2, z: 2 }]]) {
      setTarget(t);
      advance(seconds);
    }
  }

  let last = performance.now();
  const frame = (now) => {
    const dt = Math.min(0.05, (now - last) / 1000);   // a background tab should not jump ahead
    last = now;
    advance(dt);
    drawWorld();
    drawPlot();
    drawMetrics();
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}

// Exposed for quick experiments in the console.
window.sim = { get state() { return state; }, setTarget, G };

init();
