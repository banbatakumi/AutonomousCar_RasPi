// 走行モード定数定義（バックエンドの DriveMode と一致させる）
export const MODES = {
  MANUAL: "manual",
  MAP_BUILDING: "map_building",
  REACTIVE: "reactive",
  AUTONOMOUS: "autonomous",
};

export const MODE_LABELS = {
  [MODES.MANUAL]: "Manual",
  [MODES.MAP_BUILDING]: "MapBuilding",
  [MODES.REACTIVE]: "Reactive",
  [MODES.AUTONOMOUS]: "Autonomous",
};

export const MODE_LIST = [
  MODES.MANUAL,
  MODES.MAP_BUILDING,
  MODES.REACTIVE,
  MODES.AUTONOMOUS,
];

// 車両スペック（UI クランプ用）
export const MAX_SPEED = 3.0; // m/s
export const MAX_STEER = 40.0; // deg
export const SPEED_STEP = 0.1; // m/s
export const STEER_STEP = 5.0; // deg

// API ベースURL（WS と同じホストから REST も叩く）
export function apiBase() {
  const wsUrl = import.meta.env.VITE_WS_URL;
  // VITE_WS_URL があれば そのホストへ。無ければ今開いているページのホストへ。
  if (wsUrl) {
    try {
      const u = new URL(wsUrl);
      const proto = u.protocol === "wss:" ? "https:" : "http:";
      return `${proto}//${u.host}`;
    } catch {
      /* fallthrough */
    }
  }
  return window.location.origin;
}

export async function apiPost(path, body) {
  const res = await fetch(`${apiBase()}${path}`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: body ? JSON.stringify(body) : undefined,
  });
  return res.json();
}

export async function apiGet(path) {
  const res = await fetch(`${apiBase()}${path}`);
  return res.json();
}
