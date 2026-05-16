"use strict";

// ── State ────────────────────────────────────────────────────────────────────

const state = {
  do_stop:          true,
  do_remote_control: false,
  do_brake:         false,
  on_headlight:     false,
  move_speed:       0.0,
  acceleration:     0.0,
  steer:            0.0,
};

let targetSpeedRaw = 0;  // slider int (-128..127), m/s = raw * 0.1
let targetAccelRaw = 0;  // slider int (0..127),    m/s² = raw * 0.1
const keysDown = new Set();

// ── Elements ─────────────────────────────────────────────────────────────────

const swDrive      = document.getElementById("sw-drive");
const swRemote     = document.getElementById("sw-remote");
const sliderSpeed  = document.getElementById("slider-speed");
const sliderAccel  = document.getElementById("slider-accel");
const speedVal     = document.getElementById("speed-val");
const accelVal     = document.getElementById("accel-val");
const btnHeadlight = document.getElementById("btn-headlight");
const btnBrake     = document.getElementById("btn-brake");
const wsStatus     = document.getElementById("ws-status");
const kbdHint      = document.getElementById("kbd-hint");

const dispSpeed  = document.getElementById("disp-speed");
const dispAccel  = document.getElementById("disp-accel");
const dispFront  = document.getElementById("disp-front");
const dispBack   = document.getElementById("disp-back");
const dispLeft   = document.getElementById("disp-left");
const dispRight  = document.getElementById("disp-right");
const dispVsig   = document.getElementById("disp-vsig");
const dispVpow   = document.getElementById("disp-vpow");
const dispErr    = document.getElementById("disp-err");
const dispErrItem = document.getElementById("disp-err-item");

// ── WebSocket ─────────────────────────────────────────────────────────────────

let ws = null;

function connect() {
  const url = `ws://${location.host}/ws`;
  ws = new WebSocket(url);

  ws.onopen = () => {
    wsStatus.textContent = "接続中";
    wsStatus.className = "ws-status connected";
    sendState();
  };

  ws.onclose = () => {
    wsStatus.textContent = "切断中";
    wsStatus.className = "ws-status disconnected";
    setTimeout(connect, 2000);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = (e) => {
    const d = JSON.parse(e.data);
    dispSpeed.textContent  = `${d.speed.toFixed(1)} m/s`;
    dispAccel.textContent  = `${d.acceleration.toFixed(1)} m/s²`;
    dispFront.textContent  = `${d.dist_front} cm`;
    dispBack.textContent   = `${d.dist_back} cm`;
    dispLeft.textContent   = `${d.dist_left} cm`;
    dispRight.textContent  = `${d.dist_right} cm`;
    dispVsig.textContent   = `${d.volt_signal.toFixed(1)} V`;
    dispVpow.textContent   = `${d.volt_power.toFixed(1)} V`;

    const err = d.motor_error;
    dispErr.textContent = err ? "エラー" : "正常";
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
    if      (keysDown.has("w")) state.move_speed = mag;
    else if (keysDown.has("s")) state.move_speed = -mag;
    else                        state.move_speed = 0.0;

    if      (keysDown.has("a")) state.steer = -1.0;
    else if (keysDown.has("d")) state.steer =  1.0;
    else                        state.steer =  0.0;
  } else {
    state.move_speed = targetSpeedRaw * 0.1;
    state.steer = 0.0;
  }

  sendState();
}

// ── UI event handlers ─────────────────────────────────────────────────────────

swDrive.addEventListener("change", () => {
  state.do_stop = !swDrive.checked;
  sendState();
});

swRemote.addEventListener("change", () => {
  state.do_remote_control = swRemote.checked;
  kbdHint.classList.toggle("visible", swRemote.checked);
  if (!swRemote.checked) {
    keysDown.clear();
  }
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

btnBrake.addEventListener("click", () => {
  state.do_brake = !state.do_brake;
  btnBrake.classList.toggle("active", state.do_brake);
  sendState();
});

// ── Keyboard ──────────────────────────────────────────────────────────────────

document.addEventListener("keydown", (e) => {
  if (!state.do_remote_control) return;
  const key = e.key.toLowerCase();
  if (!["w", "a", "s", "d"].includes(key)) return;
  e.preventDefault();
  if (keysDown.has(key)) return;
  keysDown.add(key);
  applyControls();
});

document.addEventListener("keyup", (e) => {
  const key = e.key.toLowerCase();
  if (!keysDown.has(key)) return;
  keysDown.delete(key);
  applyControls();
});

// ── Init ──────────────────────────────────────────────────────────────────────

connect();
