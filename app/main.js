"use strict";

// ── Gauge constants (r=38, circumference=238.76) ──────────────────────────────
const G_CIRC    = 238.76;
const G_MAX_ARC = G_CIRC * 0.75; // 270° = 179.07

function setGauge(id, ratio) {
  const el = document.getElementById(id);
  if (!el) return;
  const fill = Math.max(0, Math.min(1, ratio)) * G_MAX_ARC;
  el.style.strokeDasharray = `${fill.toFixed(1)} ${G_CIRC}`;
}

// ── State ─────────────────────────────────────────────────────────────────────
const state = {
  do_stop:           true,
  do_remote_control: false,
  do_brake:          false,
  on_headlight:      false,
  on_hazard:         false,
  move_speed:        0.0,
  acceleration:      0.0,
  steer:             0.0,
};

let targetSpeedRaw = 0;
let targetAccelRaw = 0;
const keysDown = new Set();

// ── Elements ──────────────────────────────────────────────────────────────────
const swDrive      = document.getElementById("sw-drive");
const swRemote     = document.getElementById("sw-remote");
const sliderSpeed  = document.getElementById("slider-speed");
const sliderAccel  = document.getElementById("slider-accel");
const speedVal     = document.getElementById("speed-val");
const accelVal     = document.getElementById("accel-val");
const btnHeadlight = document.getElementById("btn-headlight");
const btnHazard    = document.getElementById("btn-hazard");
const wsStatus     = document.getElementById("ws-status");
const kbdHint      = document.getElementById("kbd-hint");

const dispSpeed   = document.getElementById("disp-speed");
const dispAccel   = document.getElementById("disp-accel");
const gAccel      = document.getElementById("g-accel");
const dispFront   = document.getElementById("disp-front");
const dispBack    = document.getElementById("disp-back");
const dispLeft    = document.getElementById("disp-left");
const dispRight   = document.getElementById("disp-right");
const dispVsig    = document.getElementById("disp-vsig");
const dispVpow    = document.getElementById("disp-vpow");
const gVsig       = document.getElementById("g-vsig");
const dispErr     = document.getElementById("disp-err");
const dispErrItem = document.getElementById("disp-err-item");
const gVolt       = document.getElementById("g-volt");

// ── WebSocket ─────────────────────────────────────────────────────────────────
let ws = null;

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onopen = () => {
    wsStatus.textContent = "● ONLINE";
    wsStatus.className = "ws-status connected";
    sendState();
  };

  ws.onclose = () => {
    wsStatus.textContent = "● OFFLINE";
    wsStatus.className = "ws-status disconnected";
    setTimeout(connect, 2000);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);

    // Speed gauge (0–5 m/s absolute)
    dispSpeed.textContent = d.speed.toFixed(1);
    setGauge("g-speed", Math.abs(d.speed) / 5);

    // Voltage gauge with colour coding (0–15V range)
    dispVpow.textContent = d.volt_power.toFixed(1);
    setGauge("g-volt", d.volt_power / 15);
    gVolt.style.stroke =
      d.volt_power > 11.5 ? "#16a34a" :
      d.volt_power >  9.5 ? "#ca8a04" : "#e02020";

    dispAccel.textContent = d.acceleration.toFixed(1);
    setGauge("g-accel", Math.abs(d.acceleration) / 5);
    gAccel.style.stroke = d.acceleration >= 0 ? "#ca8a04" : "#e06020";
    dispFront.textContent  = `${d.dist_front} cm`;
    dispBack.textContent   = `${d.dist_back} cm`;
    dispLeft.textContent   = `${d.dist_left} cm`;
    dispRight.textContent  = `${d.dist_right} cm`;
    dispVsig.textContent = d.volt_signal.toFixed(1);
    setGauge("g-vsig", d.volt_signal / 5);

    const err = d.motor_error;
    dispErr.textContent = err ? "ERROR" : "OK";
    dispErrItem.classList.toggle("error", err);
  };
}

function sendState() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(state));
  }
}

// ── Control logic ─────────────────────────────────────────────────────────────
function applyControls() {
  state.acceleration = targetAccelRaw * 0.1;

  if (state.do_remote_control) {
    const mag = Math.abs(targetSpeedRaw) * 0.1;
    if      (keysDown.has("w")) state.move_speed =  mag;
    else if (keysDown.has("s")) state.move_speed = -mag;
    else                        state.move_speed =  0.0;

    if      (keysDown.has("a")) state.steer = -1.0;
    else if (keysDown.has("d")) state.steer =  1.0;
    else                        state.steer =  0.0;
  } else {
    state.move_speed = targetSpeedRaw * 0.1;
    state.steer = 0.0;
  }

  sendState();
}

// ── UI handlers ───────────────────────────────────────────────────────────────
swDrive.addEventListener("change", () => {
  state.do_stop = !swDrive.checked;
  sendState();
});

swRemote.addEventListener("change", () => {
  state.do_remote_control = swRemote.checked;
  kbdHint.classList.toggle("visible", swRemote.checked);
  if (!swRemote.checked) keysDown.clear();
  applyControls();
});

sliderSpeed.addEventListener("input", () => {
  targetSpeedRaw = parseInt(sliderSpeed.value);
  speedVal.textContent = `${(targetSpeedRaw * 0.1).toFixed(1)} m/s`;
  applyControls();
});

sliderAccel.addEventListener("input", () => {
  targetAccelRaw = parseInt(sliderAccel.value);
  accelVal.textContent = `${(targetAccelRaw * 0.1).toFixed(1)} m/s²`;
  applyControls();
});

btnHeadlight.addEventListener("click", () => {
  state.on_headlight = !state.on_headlight;
  btnHeadlight.classList.toggle("active", state.on_headlight);
  sendState();
});

btnHazard.addEventListener("click", () => {
  state.on_hazard = !state.on_hazard;
  btnHazard.classList.toggle("active", state.on_hazard);
  sendState();
});

// ── Keyboard ──────────────────────────────────────────────────────────────────
document.addEventListener("keydown", (e) => {
  if (e.code === "Space") {
    e.preventDefault();
    if (!state.do_brake) { state.do_brake = true; sendState(); }
    return;
  }
  if (!state.do_remote_control) return;
  const key = e.key.toLowerCase();
  if (!["w","a","s","d"].includes(key)) return;
  e.preventDefault();
  if (keysDown.has(key)) return;
  keysDown.add(key);
  applyControls();
});

document.addEventListener("keyup", (e) => {
  if (e.code === "Space") {
    state.do_brake = false; sendState(); return;
  }
  const key = e.key.toLowerCase();
  if (!keysDown.has(key)) return;
  keysDown.delete(key);
  applyControls();
});

// ── Init ──────────────────────────────────────────────────────────────────────
connect();
