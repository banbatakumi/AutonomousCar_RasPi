/* SURGE Mark.2 Web UI
 * TelemetryServer(/ws) に接続し、封筒JSON＋LiDAR binary(タグ0x01)/占有格子(0x02)を
 * 受信して描画。操作は CommandFrame(JSON) で送信する。
 * 実機/SIM共通：UIは UIView 相当のデータ（telemetry/scene/lidar）だけを読む。
 */
"use strict";

const TAG_LIDAR = 0x01;
const TAG_GRID = 0x02;
const MAX_RANGE = 12.0;

const state = {
  ws: null,
  connected: false,
  control: false,
  source: "—",
  frame: null,      // 最新テレメトリ封筒
  scene: null,      // {walls, center_line}
  lidar: null,      // Float32Array 距離[m]（Infinity=範囲外）
  courses: [],      // コース名（sceneには無いので別途要求…今はSIM側起動時のみ）
  manualSpeed: 0,
  manualSteer: 0,
};

const VEHICLE = { maxSpeed: 3.0, maxSteer: 40.0 };
const RATE = { speed: 2.0, steer: 100.0, center: 140.0 };
const keys = new Set();

// ---- DOM ----
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");
const $ = (id) => document.getElementById(id);

// =====================================================================
// WebSocket
// =====================================================================
function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  const url = `${proto}://${location.host}/ws`;
  const ws = new WebSocket(url);
  ws.binaryType = "arraybuffer";
  state.ws = ws;

  ws.onopen = () => {
    state.connected = true;
    $("conn").textContent = "CONNECTED";
    $("conn").className = "badge badge-on";
  };
  ws.onclose = () => {
    state.connected = false;
    $("conn").textContent = "DISCONNECTED";
    $("conn").className = "badge badge-off";
    setTimeout(connect, 1000); // 自動再接続
  };
  ws.onmessage = (ev) => {
    if (typeof ev.data === "string") handleJson(JSON.parse(ev.data));
    else handleBinary(new DataView(ev.data));
  };
}

function handleJson(msg) {
  switch (msg.type) {
    case "telemetry": state.frame = msg; break;
    case "scene":
      state.scene = msg;
      break;
    case "role":
      state.control = !!msg.control;
      state.source = msg.source || "—";
      updateRoleBadge();
      break;
  }
}

function handleBinary(dv) {
  const tag = dv.getUint8(0);
  if (tag === TAG_LIDAR) {
    const n = (dv.byteLength - 1) / 2;
    const arr = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      const mm = dv.getUint16(1 + i * 2, true); // little-endian
      arr[i] = mm === 0 ? Infinity : mm / 1000.0;
    }
    state.lidar = arr;
  } else if (tag === TAG_GRID) {
    decodeGrid(dv);
  }
}

// 占有格子(int8 row-major)を occupied 世界座標点へ展開
function decodeGrid(dv) {
  const meta = state.frame && state.frame.map;
  if (!meta) return;
  const { res, ox, oy, w, h } = meta;
  const cells = [];
  let i = 1; // 先頭はタグ
  for (let cy = 0; cy < h; cy++) {
    for (let cx = 0; cx < w; cx++) {
      if (dv.getInt8(i) === 1) {
        cells.push([ox + (cx + 0.5) * res, oy + (cy + 0.5) * res]);
      }
      i++;
    }
  }
  state.occCells = cells;
  state.occRes = res;
}

function send(name, payload = {}) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(Object.assign({ type: "cmd", name }, payload)));
  }
}

// =====================================================================
// 座標変換（ワールド[m] → キャンバスpx）
// =====================================================================
let TF = { scale: 1, ox: 0, oy: 0, minX: 0, maxY: 0 };

function computeTransform() {
  const W = canvas.width, H = canvas.height;
  const margin = 0.5;
  // 表示範囲は「ロボットが知る世界」基準：SLAM占有格子 → 無ければ推定姿勢周辺
  let minX, minY, maxX, maxY;
  const map = state.frame && state.frame.map;
  if (map) {
    minX = map.ox - margin; minY = map.oy - margin;
    maxX = map.ox + map.w * map.res + margin;
    maxY = map.oy + map.h * map.res + margin;
  } else if (state.frame && state.frame.pose_est) {
    const p = state.frame.pose_est, r = 6.0;
    minX = p.x - r; maxX = p.x + r; minY = p.y - r; maxY = p.y + r;
  } else {
    minX = 0; minY = 0; maxX = 1; maxY = 1;
  }
  const wW = Math.max(maxX - minX, 1e-3), wH = Math.max(maxY - minY, 1e-3);
  const pad = 24;
  const scale = Math.min((W - 2 * pad) / wW, (H - 2 * pad) / wH);
  TF = {
    scale,
    ox: pad + (W - 2 * pad - wW * scale) / 2,
    oy: pad + (H - 2 * pad - wH * scale) / 2,
    minX, maxY,
  };
}

function w2s(x, y) {
  return [TF.ox + (x - TF.minX) * TF.scale, TF.oy + (TF.maxY - y) * TF.scale];
}

// =====================================================================
// 描画
// =====================================================================
function resize() {
  const r = canvas.parentElement.getBoundingClientRect();
  canvas.width = r.width; canvas.height = r.height;
}

function render() {
  ctx.fillStyle = "#0a0a0e";
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  computeTransform();

  const f = state.frame;
  const scene = state.scene;

  // SLAM占有格子＝ロボットが構築した「地図」（白＝壁）
  if (state.occCells && state.occRes) {
    const sz = Math.max(state.occRes * TF.scale, 2);
    ctx.fillStyle = "#dcdce0";
    for (const c of state.occCells) {
      const s = w2s(c[0], c[1]);
      ctx.fillRect(s[0] - sz / 2, s[1] - sz / 2, sz, sz);
    }
  }

  // SLAM抽出の中心線（シアン破線）
  if (scene && scene.slam_center) drawPath(scene.slam_center, "#28d2d2", true);

  // レーシングライン（黄）
  if (scene && scene.racing_line) drawPath(scene.racing_line, "#ebd228", false);

  // LiDAR点（推定姿勢基準）
  if (state.lidar && f) {
    drawLidar(state.lidar, f.pose_est);
  }

  // 進路ターゲット
  if (f && f.drive_mode !== "manual" && f.planner && f.planner.target_point) {
    const color = f.drive_mode === "reactive" ? "#3cc878" : "#ebd228";
    const t = w2s(f.planner.target_point[0], f.planner.target_point[1]);
    const v = w2s(f.pose_est.x, f.pose_est.y);
    ctx.strokeStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(v[0], v[1]); ctx.lineTo(t[0], t[1]); ctx.stroke();
    ctx.beginPath(); ctx.arc(t[0], t[1], 7, 0, Math.PI * 2); ctx.stroke();
  }

  // 車両（推定姿勢）— 運用UIは真値を表示しない（実機相当）
  if (f) drawVehicle(f.pose_est, "#468cec", false);

  updateInfo();
  requestAnimationFrame(render);
}

function drawPath(pts, color, dashed) {
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  for (let i = 0; i < pts.length; i++) {
    if (dashed && i % 2 === 1) continue;
    const a = pts[i], b = pts[(i + 1) % pts.length];
    const sa = w2s(a[0], a[1]), sb = w2s(b[0], b[1]);
    ctx.beginPath(); ctx.moveTo(sa[0], sa[1]); ctx.lineTo(sb[0], sb[1]); ctx.stroke();
  }
}

function drawLidar(dist, pose) {
  const n = dist.length;
  if (!n) return;
  const hd = pose.heading * Math.PI / 180;
  // 360点を角度順に結んだレーダー状ポリゴン（範囲外は最大レンジで閉じる）
  ctx.beginPath();
  for (let i = 0; i < n; i++) {
    let d = dist[i];
    if (!isFinite(d) || d > MAX_RANGE) d = MAX_RANGE;
    const ang = (i * 360 / n) * Math.PI / 180 + hd;
    const s = w2s(pose.x + d * Math.cos(ang), pose.y + d * Math.sin(ang));
    if (i === 0) ctx.moveTo(s[0], s[1]); else ctx.lineTo(s[0], s[1]);
  }
  ctx.closePath();
  ctx.fillStyle = "rgba(235,70,70,0.12)";
  ctx.fill();
  ctx.strokeStyle = "rgba(235,70,70,0.85)";
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

function drawVehicle(pose, color, outline) {
  const L = 0.18 * TF.scale, Wd = 0.10 * TF.scale;
  const [cx, cy] = w2s(pose.x, pose.y);
  const h = pose.heading * Math.PI / 180;
  const tip = [cx + L * Math.cos(h), cy - L * Math.sin(h)];
  const rl = [cx - 0.5 * L * Math.cos(h) - Wd * Math.sin(h), cy + 0.5 * L * Math.sin(h) - Wd * Math.cos(h)];
  const rr = [cx - 0.5 * L * Math.cos(h) + Wd * Math.sin(h), cy + 0.5 * L * Math.sin(h) + Wd * Math.cos(h)];
  ctx.beginPath(); ctx.moveTo(tip[0], tip[1]); ctx.lineTo(rl[0], rl[1]); ctx.lineTo(rr[0], rr[1]); ctx.closePath();
  if (outline) { ctx.strokeStyle = color; ctx.lineWidth = 1; ctx.stroke(); }
  else { ctx.fillStyle = color; ctx.fill(); ctx.strokeStyle = "#c8dcff"; ctx.lineWidth = 1; ctx.stroke(); }
}

function poseDiffers(a, b) {
  return Math.abs(a.x - b.x) > 0.02 || Math.abs(a.y - b.y) > 0.02 ||
         Math.abs(a.heading - b.heading) > 1.0;
}

// =====================================================================
// INFOパネル更新
// =====================================================================
const MODE_LABEL = { manual: "MANUAL", auto: "AUTO(map)", reactive: "REACTIVE" };

function updateInfo() {
  const f = state.frame;
  if (!f) return;
  const v = f.vehicle, p = f.pose_est;
  let lmin = MAX_RANGE;
  if (state.lidar) for (const d of state.lidar) if (d < lmin) lmin = d;

  $("i-mode").textContent = MODE_LABEL[f.drive_mode] || f.drive_mode;
  $("i-speed").textContent = v.speed.toFixed(2) + " m/s";
  $("i-accel").textContent = v.accel.toFixed(2) + " m/s²";
  $("i-steer").textContent = v.steer.toFixed(2) + "°";
  $("i-x").textContent = p.x.toFixed(2) + " m";
  $("i-y").textContent = p.y.toFixed(2) + " m";
  $("i-hd").textContent = p.heading.toFixed(2) + "°";
  $("i-lmin").textContent = lmin.toFixed(2) + " m";
  $("i-locsrc").textContent = p.src;
  $("i-time").textContent = f.t.toFixed(2) + " s";

  // モードボタンのアクティブ表示
  document.querySelectorAll(".mode-btn").forEach((b) =>
    b.classList.toggle("active", b.dataset.mode === f.drive_mode));
  // SIM CTRL 状態
  const paused = f.sim_ctrl ? f.sim_ctrl.paused : false;
  $("c-play").classList.toggle("active", !paused);
  $("c-pause").classList.toggle("active", paused);
  if (f.sim_ctrl) {
    document.querySelectorAll("#mult-row button").forEach((b) =>
      b.classList.toggle("active", parseFloat(b.dataset.mult) === f.sim_ctrl.speed_mult));
  }
}

function updateRoleBadge() {
  $("role").textContent = state.control ? "CONTROL" : "VIEWER";
  $("role").className = "badge " + (state.control ? "badge-ctrl" : "");
  $("src").textContent = state.source.toUpperCase();
  $("claim").style.display = state.control ? "none" : "block";
}

// =====================================================================
// 入力（手動ドライブは絶対値 manual_input を ~20Hz 送信）
// =====================================================================
function inputTick(dt) {
  const f = state.frame;
  if (!f) return;
  if (f.drive_mode !== "manual") {
    // 非manual中はローカル目標を現在指令に同期（切替時に滑らか）
    if (f.command) { state.manualSpeed = f.command.target_speed; state.manualSteer = f.command.target_steer; }
    return;
  }
  let { manualSpeed: sp, manualSteer: st } = state;
  if (keys.has("ArrowUp")) sp += RATE.speed * dt;
  if (keys.has("ArrowDown")) sp -= RATE.speed * dt;
  if (keys.has("ArrowLeft")) st += RATE.steer * dt;
  else if (keys.has("ArrowRight")) st -= RATE.steer * dt;
  else if (Math.abs(st) > 1e-3) {
    const step = Math.sign(st) * Math.min(RATE.center * dt, Math.abs(st));
    st -= step;
  }
  sp = clamp(sp, -VEHICLE.maxSpeed, VEHICLE.maxSpeed);
  st = clamp(st, -VEHICLE.maxSteer, VEHICLE.maxSteer);
  state.manualSpeed = sp; state.manualSteer = st;
  send("manual_input", { speed: sp, steer: st });
}

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

// =====================================================================
// イベント結線
// =====================================================================
function toggleMode(mode) {
  const cur = state.frame ? state.frame.drive_mode : "manual";
  send("set_mode", { mode: cur === mode ? "manual" : mode });
}

document.addEventListener("keydown", (e) => {
  if (["ArrowUp", "ArrowDown", "ArrowLeft", "ArrowRight", " "].includes(e.key)) e.preventDefault();
  if (e.repeat) return;
  switch (e.key) {
    case "a": case "A": toggleMode("auto"); break;
    case "f": case "F": toggleMode("reactive"); break;
    case "m": case "M": send("toggle_mapping"); break;
    case "g": case "G": send("build_racing_line"); break;
    case "r": case "R": send("reset"); break;
    case " ": {
      const paused = state.frame && state.frame.sim_ctrl && state.frame.sim_ctrl.paused;
      send("pause", { value: !paused }); break;
    }
    default: keys.add(e.key);
  }
});
document.addEventListener("keyup", (e) => keys.delete(e.key));

// モードボタン
document.querySelectorAll(".mode-btn").forEach((b) =>
  b.addEventListener("click", () => send("set_mode", { mode: b.dataset.mode })));
$("estop").addEventListener("click", () => send("estop"));
$("map-btn").addEventListener("click", () => send("toggle_mapping"));
$("rl-btn").addEventListener("click", () => send("build_racing_line"));

// SIM CTRL
$("c-play").addEventListener("click", () => send("pause", { value: false }));
$("c-pause").addEventListener("click", () => send("pause", { value: true }));
$("c-reset").addEventListener("click", () => send("reset"));
$("claim").addEventListener("click", () => send("claim_control"));

// 速度倍率ボタン
[0.5, 1.0, 2.0].forEach((m) => {
  const b = document.createElement("button");
  b.textContent = m + "x"; b.dataset.mult = m;
  b.addEventListener("click", () => send("speed_mult", { value: m }));
  $("mult-row").appendChild(b);
});

// 方向パッド（押している間だけキー扱い）
function bindPad(id, key) {
  const el = $(id);
  const down = (e) => { e.preventDefault(); keys.add(key); el.classList.add("held"); };
  const up = (e) => { e.preventDefault(); keys.delete(key); el.classList.remove("held"); };
  el.addEventListener("mousedown", down); el.addEventListener("mouseup", up);
  el.addEventListener("mouseleave", up);
  el.addEventListener("touchstart", down); el.addEventListener("touchend", up);
}
bindPad("d-up", "ArrowUp"); bindPad("d-down", "ArrowDown");
bindPad("d-left", "ArrowLeft"); bindPad("d-right", "ArrowRight");

// コースリスト（?courses= で渡らない場合のフォールバックは無し。scene更新時に未提供なら空）
function renderCourseList(names) {
  const box = $("course-list"); box.innerHTML = "";
  names.forEach((n) => {
    const b = document.createElement("button");
    b.textContent = n;
    b.addEventListener("click", () => send("set_course", { course: n }));
    box.appendChild(b);
  });
}

// =====================================================================
// 起動
// =====================================================================
window.addEventListener("resize", resize);
resize();
updateRoleBadge();
connect();

// 入力＆描画ループ
let lastT = performance.now();
setInterval(() => {
  const now = performance.now();
  inputTick((now - lastT) / 1000);
  lastT = now;
}, 50);
requestAnimationFrame(render);

// コース一覧はサーバ起動時に既知（SIM）。/courses から取得を試みる
fetch("courses.json").then(r => r.ok ? r.json() : null).then((d) => {
  if (d && d.courses) renderCourseList(d.courses);
}).catch(() => {});
