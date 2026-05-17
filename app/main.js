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
  do_brake:          false,
  on_headlight:      false,
  on_hazard:         false,
  play_sound:        false,
  enable_auto_brake: false,
  mode:              0,
  move_speed:        0.0,
  acceleration:      0.0,
  steer:             0.0,
};

let currentMode = null; // null | 'drive' | 'autobrake' | 'remote' | 'mode1' | 'mode2'

let targetSpeedRaw = 10;
let targetAccelRaw = 10;
const keysDown = new Set();

// Gamepad state
let gpIndex     = null;
let gpRafId     = null;
let gpSpeed     = 0;     // -1..+1  right stick Y (positive = forward)
let gpSteer     = 0;     // -1..+1  left stick X
let gpBraking   = false; // R1
let gpLightPrev = false; // L2 edge detection

const GP_DEAD = 0.12;

// ── Elements ──────────────────────────────────────────────────────────────────
const sliderSpeed  = document.getElementById("slider-speed");
const sliderAccel  = document.getElementById("slider-accel");
const speedVal     = document.getElementById("speed-val");
const accelVal     = document.getElementById("accel-val");
const btnHeadlight = document.getElementById("btn-headlight");
const btnHazard    = document.getElementById("btn-hazard");
const btnHorn      = document.getElementById("btn-horn");
const wsStatus     = document.getElementById("ws-status");
const kbdHint      = document.getElementById("kbd-hint");

const gpStatusEl  = document.getElementById("gp-status");
const dispSpeed   = document.getElementById("disp-speed");
const dispAccel   = document.getElementById("disp-accel");
const gAccel      = document.getElementById("g-accel");
const dispVsig    = document.getElementById("disp-vsig");
const dispVpow    = document.getElementById("disp-vpow");
const gVsig       = document.getElementById("g-vsig");
const dispErr = document.getElementById("disp-err");
const gVolt   = document.getElementById("g-volt");

const dispRssi    = document.getElementById("disp-rssi");
const dispCpuTemp = document.getElementById("disp-cpu-temp");
const dispCpuLoad = document.getElementById("disp-cpu-load");
const dispMem     = document.getElementById("disp-mem");

const gTempLeft     = document.getElementById("g-temp-left");
const gTempRight    = document.getElementById("g-temp-right");
const gTempSteer    = document.getElementById("g-temp-steer");
const dispTempLeft  = document.getElementById("disp-temp-left");
const dispTempRight = document.getElementById("disp-temp-right");
const dispTempSteer = document.getElementById("disp-temp-steer");

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

    // Voltage gauge with colour coding (8–12V range)
    dispVpow.textContent = d.volt_power.toFixed(1);
    setGauge("g-volt", (d.volt_power - 8) / 4);

    dispAccel.textContent = d.acceleration.toFixed(1);
    setGauge("g-accel", Math.abs(d.acceleration) / 5);
    updateRadar(d.dists);
    updateAHI(d.pitch, d.roll);
    updateGMeter(d.accel_x, d.accel_y);
    dispVsig.textContent = d.volt_signal.toFixed(1);
    setGauge("g-vsig", (d.volt_signal - 8) / 4);

    updateTemp(gTempLeft,  dispTempLeft,  d.temp_left);
    updateTemp(gTempRight, dispTempRight, d.temp_right);
    updateTemp(gTempSteer, dispTempSteer, d.temp_steer);

    const err = d.motor_error;
    dispErr.textContent = err ? "ERROR" : "OK";
    dispErr.className = "hdr-telem-val" + (err ? " error" : "");

    const cpu = d.cpu_temp;
    dispCpuTemp.textContent = cpu != null ? `${cpu}` : "--";
    dispCpuTemp.className = "hdr-telem-val" + (!cpu ? "" : cpu >= 80 ? " error" : cpu >= 70 ? " warn" : "");

    const load = d.cpu_load;
    dispCpuLoad.textContent = load != null ? `${load}%` : "--%";
    dispCpuLoad.className = "hdr-telem-val" + (!load ? "" : load >= 80 ? " error" : load >= 50 ? " warn" : "");

    const mem = d.mem_usage;
    dispMem.textContent = mem != null ? `${mem}%` : "--%";
    dispMem.className = "hdr-telem-val" + (!mem ? "" : mem >= 85 ? " error" : mem >= 70 ? " warn" : "");

    const rssi = d.wifi_tx_mbps;
    dispRssi.textContent = rssi !== null && rssi !== undefined ? `${rssi}` : "--";
    dispRssi.className = "hdr-telem-val" + (rssi == null ? "" : rssi >= -60 ? "" : rssi >= -75 ? " warn" : " error");
  };
}

function sendState() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(state));
  }
}

// ── Gamepad ───────────────────────────────────────────────────────────────────
function gpApplyDead(v) {
  return Math.abs(v) > GP_DEAD ? Math.max(-1, Math.min(1, v)) : 0;
}

function gpConnect(index) {
  if (gpIndex !== null) return;
  gpIndex = index;
  gpStatusEl.textContent = "GAMEPAD READY";
  gpStatusEl.classList.add("connected");
  if (!gpRafId) gpRafId = requestAnimationFrame(pollGamepad);
}

function gpDisconnect() {
  cancelAnimationFrame(gpRafId);
  gpRafId = null;
  gpIndex = null;
  gpSpeed = 0;
  gpSteer = 0;
  if (gpBraking) { gpBraking = false; state.do_brake = false; sendState(); }
  gpStatusEl.textContent = "GAMEPAD --";
  gpStatusEl.classList.remove("connected");
}

// Event-based detection (Chrome/Firefox)
window.addEventListener("gamepadconnected",    (e) => gpConnect(e.gamepad.index));
window.addEventListener("gamepaddisconnected", (e) => { if (e.gamepad.index === gpIndex) gpDisconnect(); });

// Polling fallback for Safari (gamepadconnected may not fire)
setInterval(() => {
  if (gpIndex !== null) return;
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  for (let i = 0; i < pads.length; i++) {
    if (pads[i]) { gpConnect(i); break; }
  }
}, 500);

function pollGamepad() {
  gpRafId = requestAnimationFrame(pollGamepad);

  const gp = navigator.getGamepads()[gpIndex];
  if (!gp) { gpDisconnect(); return; }

  // Left stick Y → speed (axes[1]; up = −1 → invert for forward)
  const newSpeed = gpApplyDead(-gp.axes[1]);
  // Right stick X → steer (axes[2] on JC-U3613M)
  const newSteer = gpApplyDead(gp.axes[2]);

  if (newSpeed !== gpSpeed || newSteer !== gpSteer) {
    gpSpeed = newSpeed;
    gpSteer = newSteer;
    if (currentMode === 'remote') applyControls();
  }

  // R1 (button[5]) → brake
  const brakeNow = gp.buttons[5]?.pressed ?? false;
  if (brakeNow !== gpBraking) {
    gpBraking = brakeNow;
    state.do_brake = gpBraking;
    sendState();
  }

  // L1 (button[4]) → headlight toggle on press (rising edge)
  const lightNow = gp.buttons[4]?.pressed ?? false;
  if (lightNow && !gpLightPrev) {
    state.on_headlight = !state.on_headlight;
    btnHeadlight.classList.toggle("active", state.on_headlight);
    sendState();
  }
  gpLightPrev = lightNow;
}

// ── Control logic ─────────────────────────────────────────────────────────────
function applyControls() {
  state.acceleration = targetAccelRaw * 0.1;

  if (currentMode === 'remote') {
    const mag = Math.abs(targetSpeedRaw) * 0.1;

    // Gamepad analog input takes priority when stick is deflected
    if (gpIndex !== null && (gpSpeed !== 0 || gpSteer !== 0)) {
      state.move_speed = gpSpeed * mag;
      state.steer      = gpSteer;
    } else {
      if      (keysDown.has("w")) state.move_speed =  mag;
      else if (keysDown.has("s")) state.move_speed = -mag;
      else                        state.move_speed =  0.0;

      if      (keysDown.has("a")) state.steer = -1.0;
      else if (keysDown.has("d")) state.steer =  1.0;
      else                        state.steer =  0.0;
    }
  } else {
    state.move_speed = targetSpeedRaw * 0.1;
    state.steer = 0.0;
  }

  sendState();
}

// ── Mode selection ────────────────────────────────────────────────────────────
const btnDrive     = document.getElementById("btn-drive");
const btnAutobrake = document.getElementById("btn-autobrake");

btnDrive.addEventListener("click", () => {
  state.do_stop = !state.do_stop;
  btnDrive.classList.toggle("active", !state.do_stop);
  sendState();
});

btnAutobrake.addEventListener("click", () => {
  state.enable_auto_brake = !state.enable_auto_brake;
  btnAutobrake.classList.toggle("active", state.enable_auto_brake);
  sendState();
});

function setMode(name) {
  const btns = document.querySelectorAll('.mode-btn');
  if (currentMode === name) {
    currentMode = null;
    btns.forEach(b => b.classList.remove('active'));
    state.mode = 0;
    kbdHint.classList.remove('visible');
    keysDown.clear();
  } else {
    currentMode = name;
    btns.forEach(b => b.classList.remove('active'));
    document.getElementById(`btn-${name}`).classList.add('active');
    state.mode = name === 'remote' ? 1 : name === 'mode1' ? 2 : 3;
    kbdHint.classList.toggle('visible', name === 'remote');
    if (name !== 'remote') keysDown.clear();
  }
  applyControls();
}

['remote', 'mode1', 'mode2'].forEach(m => {
  document.getElementById(`btn-${m}`).addEventListener('click', () => setMode(m));
});

// ── UI handlers ───────────────────────────────────────────────────────────────
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

const btnBrake = document.getElementById("btn-brake");
const brakeOn  = () => { state.do_brake = true;  btnBrake.classList.add("active");    sendState(); };
const brakeOff = () => { state.do_brake = false; btnBrake.classList.remove("active"); sendState(); };
btnBrake.addEventListener("mousedown",  brakeOn);
btnBrake.addEventListener("mouseup",    brakeOff);
btnBrake.addEventListener("mouseleave", brakeOff);
btnBrake.addEventListener("touchstart", (e) => { e.preventDefault(); brakeOn(); });
btnBrake.addEventListener("touchend",   brakeOff);

const hornOn  = () => { state.play_sound = true;  btnHorn.classList.add("active");    sendState(); };
const hornOff = () => { state.play_sound = false;  btnHorn.classList.remove("active"); sendState(); };
btnHorn.addEventListener("mousedown",  hornOn);
btnHorn.addEventListener("mouseup",    hornOff);
btnHorn.addEventListener("mouseleave", hornOff);
btnHorn.addEventListener("touchstart", (e) => { e.preventDefault(); hornOn(); });
btnHorn.addEventListener("touchend",   hornOff);

// ── Keyboard ──────────────────────────────────────────────────────────────────
document.addEventListener("keydown", (e) => {
  if (e.code === "Space") {
    e.preventDefault();
    if (!state.do_brake) { state.do_brake = true; sendState(); }
    return;
  }
  if (currentMode !== 'remote') return;
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

// ── Radar ─────────────────────────────────────────────────────────────────────
const RADAR_N        = 360;   // LiDARセクター数（1°刻み）
const RADAR_DOTS     = 36;    // 分割線数（10°刻み）
const RADAR_DOT_STEP = 10;    // 分割線間隔（°）
const RADAR_CX       = 100, RADAR_CY = 100, RADAR_R = 88;
let   radarMaxMm     = 2500;  // 最大表示距離 (mm), スライダーで変更
const SVG_NS         = "http://www.w3.org/2000/svg";

function initRadar() {
  const divsG = document.getElementById("radar-divs");
  for (let i = 0; i < RADAR_DOTS; i++) {
    const theta = (i * RADAR_DOT_STEP - 90) * Math.PI / 180;
    const line = document.createElementNS(SVG_NS, "line");
    line.setAttribute("x1", RADAR_CX);
    line.setAttribute("y1", RADAR_CY);
    line.setAttribute("x2", (RADAR_CX + RADAR_R * Math.cos(theta)).toFixed(1));
    line.setAttribute("y2", (RADAR_CY + RADAR_R * Math.sin(theta)).toFixed(1));
    line.setAttribute("class", "radar-div");
    divsG.appendChild(line);
  }
}

function updateRadar(dists) {
  // dists: 360要素 (mm, 0=範囲外), 1°刻み
  const points = [];
  for (let i = 0; i < RADAR_N; i++) {
    const theta = (i - 90) * Math.PI / 180;
    const d = dists[i];
    const r = d > 0 ? Math.min(d / radarMaxMm, 1.0) * RADAR_R : RADAR_R;
    points.push(`${(RADAR_CX + r * Math.cos(theta)).toFixed(1)},${(RADAR_CY + r * Math.sin(theta)).toFixed(1)}`);
  }
  const poly = document.getElementById("radar-poly");
  if (poly) poly.setAttribute("points", points.join(" "));
}

// ── Temperature display ───────────────────────────────────────────────────────
function updateTemp(gaugeEl, dispEl, temp) {
  dispEl.textContent = temp;
  setGauge(gaugeEl.id, temp / 100);
  gaugeEl.style.stroke =
    temp >= 75 ? "#e02020" :
    temp >= 50 ? "#ca8a04" : "#16a34a";
}

// ── IMU display ───────────────────────────────────────────────────────────────
function updateAHI(pitch, roll) {
  const scene = document.getElementById("ahi-scene");
  if (scene) {
    const pitchPx = Math.max(-36, Math.min(36, pitch * 0.72)); // 50° → edge
    scene.setAttribute("transform", `rotate(${roll}, 50, 50) translate(0, ${pitchPx})`);
  }
  const pe = document.getElementById("disp-pitch");
  const re = document.getElementById("disp-roll");
  if (pe) pe.textContent = `${pitch > 0 ? "+" : ""}${pitch.toFixed(0)}°`;
  if (re) re.textContent = `${roll  > 0 ? "+" : ""}${roll.toFixed(0)}°`;
}

function updateGMeter(ax, ay) {
  const dot = document.getElementById("g-dot");
  if (!dot) return;
  const SCALE = 28; // 1g = 28px (range ±1g visible in r=38 circle)
  const cx = Math.max(14, Math.min(86, 50 + ay * SCALE));
  const cy = Math.max(14, Math.min(86, 50 - ax * SCALE));
  dot.setAttribute("cx", cx.toFixed(1));
  dot.setAttribute("cy", cy.toFixed(1));
  const g = Math.sqrt(ax * ax + ay * ay);
  dot.setAttribute("fill", g > 0.6 ? "#e02020" : g > 0.3 ? "#ca8a04" : "#16a34a");
}

// ── Camera WebSocket ──────────────────────────────────────────────────────────
(function () {
  const img = document.getElementById("camera-stream");
  let prevUrl = null;
  let camWs = null;

  function connectCamera() {
    camWs = new WebSocket(`ws://${location.host}/ws/camera`);
    camWs.binaryType = "arraybuffer";

    camWs.onmessage = (e) => {
      const blob = new Blob([e.data], { type: "image/jpeg" });
      const url = URL.createObjectURL(blob);
      const old = prevUrl;
      img.onload = () => { if (old) URL.revokeObjectURL(old); };
      img.src = url;
      prevUrl = url;
    };

    camWs.onclose = () => {
      if (prevUrl) { URL.revokeObjectURL(prevUrl); prevUrl = null; }
      setTimeout(connectCamera, 2000);
    };

    camWs.onerror = () => camWs.close();
  }

  connectCamera();
})();

// ── Radar range slider ────────────────────────────────────────────────────────
const sliderRadarRange = document.getElementById("slider-radar-range");
const radarRangeVal    = document.getElementById("radar-range-val");
sliderRadarRange.addEventListener("input", () => {
  radarMaxMm = parseInt(sliderRadarRange.value);
  radarRangeVal.textContent = `${radarMaxMm / 10} cm`;
});

// ── Init ──────────────────────────────────────────────────────────────────────
speedVal.textContent = `${(targetSpeedRaw * 0.1).toFixed(1)} m/s`;
accelVal.textContent = `${(targetAccelRaw * 0.1).toFixed(1)} m/s²`;
initRadar();
connect();
