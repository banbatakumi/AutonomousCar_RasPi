"""ml_e2e/check_labels.py — 抽出したラベルが正しいかを、学習の前に確かめる。

    .venv/bin/python -m ml_e2e.check_labels ml_e2e/runs/v1/data            # 数値の表
    .venv/bin/python -m ml_e2e.check_labels ml_e2e/runs/v1/data --draw 24  # 重ね描きも出す

ラベルが間違っていれば、どれだけ学習しても走らない。見るのは2つ:

1. **経路点が壁の内側を通っているか。** 点群（撮影時刻まで車の動きぶん進めたもの）と経路点の最小距離。
   経路が壁を突き抜けていれば、姿勢の時刻・座標変換・ラインの版の選び方のどれかが違う
2. **経路点を Pure Pursuit で追った舵が、記録の舵と合うか。** 手本（`slam2d_route`）も同じ線を追っているので、
   合わなければラベルの向き（左右・前後）か時刻が違う。最適ラインのラベルと通った軌跡のラベルを分けて出す

`--draw N` は標本を等間隔に N 個選び、点群＋経路点の上面図と、経路点を投影した画像を
`<data>/check/` に書く（目で見る確認）。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

from ml_e2e import labels as L  # noqa: E402
from ml_e2e.extract import SRC_FUTURE, SRC_LINE  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402

__all__ = ["scan_points", "clearance", "check_file", "main"]

#: `slam2d_route` の注視距離（`look_k`·v + `look_min` の既定）。手本と同じ狙い方で比べる
_LOOK_K, _LOOK_MIN = 0.45, 0.3
#: 壁との距離を見る経路の長さ [m]（遠くは点群の方が古く・粗い）
_CLEAR_AHEAD_M = 3.0


def scan_points(dist: np.ndarray, seen: np.ndarray, vehicle: Vehicle, *, age: float = 0.0,
                speed: float = 0.0, yaw_rate: float = 0.0) -> np.ndarray:
    """点群1周 → 車体座標の点 `(N, 2)`。`age` 秒ぶん車が進んだ後の車体座標へ移す。

    添字がそのまま車両角 [deg]。欠測セクタ（`sector_seen`）と測距不能（0）は捨てる。
    """
    deg = np.arange(360)
    ok = (dist > 0.02) & seen[((deg - 1) % 360) // 30]
    a = np.radians(deg[ok]) + vehicle.lidar_yaw
    x = vehicle.lidar_x + dist[ok] * np.cos(a)
    y = vehicle.lidar_y + dist[ok] * np.sin(a)
    if age > 0:
        # 車の動き（円弧）の逆を点に掛ける
        dth = yaw_rate * age
        if abs(dth) > 1e-6:
            r = speed / yaw_rate
            mx, my = r * math.sin(dth), r * (1.0 - math.cos(dth))
        else:
            mx, my = speed * age, 0.0
        x, y = x - mx, y - my
        c, s = math.cos(-dth), math.sin(-dth)
        x, y = c * x - s * y, s * x + c * y
    return np.stack([x, y], axis=1)


def clearance(path_xy: np.ndarray, pts: np.ndarray) -> float:
    """経路点と点群の最小距離 [m]。どちらかが空なら NaN。"""
    if len(path_xy) == 0 or len(pts) == 0:
        return float("nan")
    d = np.hypot(path_xy[:, None, 0] - pts[None, :, 0], path_xy[:, None, 1] - pts[None, :, 1])
    return float(d.min())


def _corr(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.corrcoef(a, b)[0, 1]) if len(a) > 2 and np.std(a) > 0 and np.std(b) > 0 else float("nan")


def check_file(npz_path: Path, vehicle: Vehicle, ds: float) -> dict:
    d = np.load(npz_path)
    n = len(d["t"])
    n_ahead = max(1, int(_CLEAR_AHEAD_M / ds))
    clear = np.full(n, np.nan)
    steer_pp = np.zeros(n)
    for i in range(n):
        speed, _steer, yaw_rate = d["state"][i]
        si = int(d["scan_idx"][i, 0])
        pts = scan_points(d["scan_dist"][si], d["scan_seen"][si], vehicle, age=float(d["scan_age"][i]),
                          speed=float(speed), yaw_rate=float(yaw_rate))
        xy = d["path_xy"][i][d["path_valid"][i]]
        clear[i] = clearance(xy[:n_ahead], pts)
        if len(xy):
            steer_pp[i] = L.pursuit_steer(xy, _LOOK_K * float(speed) + _LOOK_MIN, vehicle.wheelbase,
                                          vehicle.max_steer)
    out = {"file": npz_path.name, "n": n}
    for name, src in (("line", SRC_LINE), ("future", SRC_FUTURE)):
        m = d["path_src"] == src
        if not m.any():
            continue
        steer_act = d["state"][m, 1]
        c = clear[m]
        c = c[np.isfinite(c)]
        out[name] = {
            "n": int(m.sum()),
            "clear_p05": float(np.percentile(c, 5)) if len(c) else float("nan"),
            "clear_med": float(np.median(c)) if len(c) else float("nan"),
            "hit_ratio": float((c < vehicle.half_width * 0.5).mean()) if len(c) else float("nan"),
            "steer_corr": _corr(steer_pp[m], steer_act),
            "steer_rms_deg": float(np.degrees(np.sqrt(np.mean((steer_pp[m] - steer_act) ** 2)))),
            "cross_p95": float(np.percentile(np.abs(d["cross"][m]), 95)),
        }
    return out


def draw(data_dir: Path, npz_paths: list[Path], vehicle: Vehicle, count: int) -> Path:
    """標本を等間隔に選び、上面図と画像への投影を並べた PNG を書く。"""
    import cv2

    from raspi.nav.ipm import CameraExtrinsics, _ground_to_pixel_vec, vehicle_camera_intrinsics

    out_dir = data_dir / "check"
    out_dir.mkdir(exist_ok=True)
    per_file = max(1, count // max(1, len(npz_paths)))
    ext = CameraExtrinsics(vehicle.cam_front_x, vehicle.cam_front_y, vehicle.cam_front_z,
                           vehicle.cam_front_pitch, vehicle.cam_front_yaw)
    for p in npz_paths:
        d = np.load(p)
        n = len(d["t"])
        # 端（発進・停止）を避けて内側から選ぶ
        for i in np.linspace(0, n - 1, per_file + 2)[1:-1].astype(int):
            img = cv2.imread(str(data_dir / str(d["image"][i])))
            if img is None:
                continue
            h, w = img.shape[:2]
            intr = vehicle_camera_intrinsics(vehicle, "front", w, h)
            valid = d["path_valid"][i]
            xy = d["path_xy"][i][valid]
            u, v, ok = _ground_to_pixel_vec(xy[:, 0], xy[:, 1], intr, ext)
            for uu, vv, good in zip(np.atleast_1d(u), np.atleast_1d(v), np.atleast_1d(ok)):
                if good and 0 <= uu < w and 0 <= vv < h:
                    cv2.circle(img, (int(uu), int(vv)), 3, (0, 255, 0), -1)
            # 上面図（前が上・左が左）。1m = 60px、車は下から 1/5
            size, ppm = h, 60.0
            top = np.zeros((size, size, 3), np.uint8)

            def px(x, y):
                return int(size / 2 - y * ppm), int(size * 0.8 - x * ppm)

            speed, steer, yaw_rate = d["state"][i]
            si = int(d["scan_idx"][i, 0])
            for x, y in scan_points(d["scan_dist"][si], d["scan_seen"][si], vehicle,
                                    age=float(d["scan_age"][i]), speed=float(speed), yaw_rate=float(yaw_rate)):
                cv2.circle(top, px(x, y), 1, (255, 255, 255), -1)
            for x, y in xy:
                cv2.circle(top, px(x, y), 2, (0, 255, 0), -1)
            cv2.circle(top, px(0, 0), 4, (0, 128, 255), -1)
            txt = (f"src={int(d['path_src'][i])} v={speed:.1f} steer={math.degrees(steer):+.0f}deg "
                   f"cross={float(d['cross'][i]):+.2f}")
            cv2.putText(top, txt, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            cv2.imwrite(str(out_dir / f"{p.stem}_{i:05d}.jpg"), np.hstack([img, top]))
    return out_dir


def main() -> int:
    import json

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("data_dir", type=Path)
    ap.add_argument("--draw", type=int, default=0, help="重ね描きを出す標本の数（既定 0 = 出さない）")
    args = ap.parse_args()

    meta = json.loads((args.data_dir / "dataset.json").read_text())
    ds = float(meta["config"]["ds"])
    vehicle = Vehicle.load()
    paths = sorted(args.data_dir.glob("*.npz"))
    print(f"{'記録':34} {'種類':6} {'枚数':>6} {'壁まで5%':>8} {'中央':>6} {'接触率':>6} "
          f"{'舵の相関':>8} {'舵RMS':>7} {'横偏差95%':>9}")
    for p in paths:
        r = check_file(p, vehicle, ds)
        for name in ("line", "future"):
            if name not in r:
                continue
            s = r[name]
            print(f"{p.stem:34} {name:6} {s['n']:6d} {s['clear_p05']:8.2f} {s['clear_med']:6.2f} "
                  f"{s['hit_ratio'] * 100:5.1f}% {s['steer_corr']:8.3f} {s['steer_rms_deg']:6.1f}° "
                  f"{s['cross_p95']:9.2f}", flush=True)
    if args.draw > 0:
        print(f"# 重ね描き → {draw(args.data_dir, paths, vehicle, args.draw)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
