"""走行ログ（`.mcap`）を、STM32 のステレオ自動停止（`stereo_obstacle.c`）に通して、検出と誤検出を数える。

    python -m tools.fwsim.stereo_replay ~/Downloads/surge_*.mcap
    python -m tools.fwsim.stereo_replay LOG.mcap --min-points 3 --confirm 3 --ext-front 0 --ext-side 0

`/vehicle_state`（速度・実舵角）・`/scan`（LiDAR）・`/scan/stereo` を読み、`io_node` と同じ変換
（`raspi/core/stereo_obs.py`）でフレームを STM32 に渡す。C の判定はファームの `src/control` をそのままコンパイルする
（`tools/fwsim/tests/test_stereo_obstacle.py` と同じ）。**実際には止めない**（ログは `auto_stop` が働いた走行とは限らない）。

出すもの:
- 衝突（前向き加速度 −8 m/s² 未満のスパイク）ごとに、衝突の何秒前から途切れず検出していたか（ステレオ・LiDAR）
- 検出区間の数と、うち LiDAR と同時でない「ステレオ単独」の数・平均長。**作動したら止まるまで解除しないので、
  単独の検出は短くても完全停止になる**
- 走行中のステレオフレームのうち、検出した点を LiDAR が 25cm 以内で裏付けるか（床・自車の誤検出の目安）

LiDAR の判定は `sim/stm32.py`（ファームと突き合わせ済みの予想進路）で同じ停止距離・3 点以上。停止距離は
`vehicle.toml` の制動減速度・MD の遅れ・既定マージン 15cm から。値の既定はファームの `VEHICLE_AUTO_STOP_STEREO_*`。
"""

from __future__ import annotations

import argparse
import ctypes as C
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import sim.stm32 as st  # noqa: E402
from raspi.core.stereo_obs import scan_to_obs  # noqa: E402
from raspi.msgs.types import Scan  # noqa: E402
import tools.fwsim.tests.test_stereo_obstacle as T  # noqa: E402

DECEL, MD_LAG, MARGIN, LIDAR_PERIOD = 2.165, 0.016, 0.15, 0.105
ANG = np.radians(np.arange(360))


def d_stop(v: float) -> float:
    return v * (LIDAR_PERIOD + MD_LAG) + v * v / (2 * DECEL) + MARGIN


def load(path: str):
    from mcap.reader import make_reader

    vs, sc, sts = [], [], []
    for _s, ch, m in make_reader(open(path, "rb")).iter_messages(topics=["/vehicle_state", "/scan", "/scan/stereo"]):
        d = json.loads(m.data)
        {"/vehicle_state": vs, "/scan": sc, "/scan/stereo": sts}[ch.topic].append(d)
    return vs, sorted(sc, key=lambda d: d["t_pub"]), sorted(sts, key=lambda d: d["t_pub"])


def _pts(dist, ox=0.0):
    D = np.asarray(dist)
    ok = D > 0
    return ox + D[ok] * np.cos(ANG[ok]), D[ok] * np.sin(ANG[ok])


def lidar_hit(scan, v, steer) -> bool:
    D = np.asarray(scan["dist"])
    x = st.AUTO_STOP_LIDAR_X_M + D * np.cos(ANG)
    y = D * np.sin(ANG)
    inp = st.auto_stop_path_contains(x, y, True, math.tan(steer) / st.AUTO_STOP_WHEELBASE_M, d_stop(v))
    return int(np.count_nonzero((D > 0) & inp & ~st.auto_stop_inside_body(x, y) & (np.cos(ANG) > 0))) >= 3


def runs(t, mask, gap=0.15):
    out = []
    for k in np.where(mask)[0]:
        if out and t[k] - out[-1][1] <= gap:
            out[-1][1] = t[k]
        else:
            out.append([t[k], t[k]])
    return out


def replay(path: str, a) -> None:
    vs, sc, sts = load(path)
    t0 = vs[0]["t_capture"]
    t = np.array([(s["t_capture"] - t0) / 1e9 for s in vs])
    ob = T.new()
    i_st = i_sc = 0
    last = None
    hit_s, hit_l = [], []
    for s in vs:
        tp = s["t_pub"]
        while i_st < len(sts) and sts[i_st]["t_pub"] <= tp:
            d = sts[i_st]
            o = scan_to_obs(Scan(**{k: v for k, v in d.items() if k in ("dist", "t_capture", "sector_seen", "seq")}), d["t_pub"])
            if o is not None:
                T.LIB.StereoObstacles_Push(C.byref(ob), int((d["t_pub"] - t0) // 1000) & 0xFFFFFFFF, o.age_ms, o.count,
                                           (C.c_int16 * 80)(*o.xy))
            i_st += 1
        while i_sc < len(sc) and sc[i_sc]["t_pub"] <= tp:
            last = sc[i_sc]
            i_sc += 1
        v = abs(s["speed"])
        p = T.params(curvature_1_m=math.tan(s["steer_actual"]) / st.AUTO_STOP_WHEELBASE_M, d_stop_m=d_stop(v), speed_abs_m_s=v,
                     body_front_x_m=st.AUTO_STOP_BODY_FRONT_X_M, body_rear_x_m=st.AUTO_STOP_BODY_REAR_X_M,
                     body_half_width_m=0.09, path_half_width_m=st.AUTO_STOP_HALF_WIDTH_M, min_points=a.min_points,
                     confirm_frames=a.confirm, self_ext_front_m=a.ext_front, self_ext_side_m=a.ext_side)
        now = int((tp - t0) // 1000) & 0xFFFFFFFF
        hit_s.append(bool(T.LIB.StereoObstacles_Evaluate(C.byref(ob), now, True, C.byref(p))) and s["speed"] >= -0.05)
        hit_l.append(last is not None and lidar_hit(last, v, s["steer_actual"]))
    hit_s, hit_l = np.array(hit_s), np.array(hit_l)
    ax = np.array([s["accel"][0] for s in vs])
    cols = []
    for i in np.where(ax < -8)[0]:
        if not cols or t[i] - cols[-1] > 1.0:
            cols.append(t[i])
    print(f"== {Path(path).name}: {t[-1]:.0f}s 最高 {max(abs(s['speed']) for s in vs):.1f}m/s  衝突スパイク {len(cols)} 回")

    def lead(hit, tc):
        m = np.where(hit & (t <= tc + 0.05) & (t >= tc - 4))[0]
        if len(m) == 0:
            return None
        k = len(m) - 1
        while k > 0 and t[m[k]] - t[m[k - 1]] <= 0.3:
            k -= 1
        return tc - t[m[k]]

    for tc in cols:
        ls, ll = lead(hit_s, tc), lead(hit_l, tc)
        f = lambda x: "なし" if x is None else f"{x:.2f}s前"
        print(f"   衝突 {tc:6.2f}s  ステレオ {f(ls):>8}  LiDAR {f(ll):>8}")
    rs, rl = runs(t, hit_s), runs(t, hit_l)
    solo = [(x, y) for x, y in rs if not any(not (y + 0.3 < c or d + 0.3 < x) for c, d in rl)
            and not any(x - 0.1 <= tc <= y + 0.6 for tc in cols)]
    mean = np.mean([y - x for x, y in solo]) if solo else 0.0
    print(f"   ステレオ検出 {len(rs)} 区間（検出フレーム率 {100 * hit_s.mean():.1f}%）  LiDAR {len(rl)} 区間  "
          f"衝突と無関係なステレオ単独 {len(solo)} 区間（平均 {mean:.2f}s）")
    # 点の裏付け（走行中のステレオフレームのうち、検出点が 25cm 以内に LiDAR 点を持つか）
    vt = np.array([s["t_capture"] for s in vs])
    sct = np.array([d["t_capture"] for d in sc])
    n = unc = 0
    for d in sts:
        s = vs[int(np.argmin(abs(vt - d["t_capture"])))]
        v = abs(s["speed"])
        if v < 0.3 or s["speed"] < 0:
            continue
        L = d_stop(v) + v * ((d["t_pub"] - d["t_capture"]) / 1e9 + 0.11)
        sx, sy = _pts(d["dist"])
        ex = (sx < st.AUTO_STOP_BODY_FRONT_X_M + a.ext_front) & (np.abs(sy) < 0.09 + a.ext_side) & (sx > st.AUTO_STOP_BODY_REAR_X_M)
        m = st.auto_stop_path_contains(sx, sy, True, math.tan(s["steer_actual"]) / st.AUTO_STOP_WHEELBASE_M, L) & ~ex
        if m.sum() < a.min_points:
            continue
        n += 1
        lx, ly = _pts(sc[int(np.argmin(abs(sct - d["t_capture"])))]["dist"], st.AUTO_STOP_LIDAR_X_M)
        dm = np.sqrt((sx[m][:, None] - lx[None, :]) ** 2 + (sy[m][:, None] - ly[None, :]) ** 2).min(axis=1)
        unc += (dm < 0.25).mean() < 0.5
    print(f"   走行中の検出フレーム {n} のうち、LiDAR が25cm以内で裏付けない {unc}（{100 * unc / max(n, 1):.0f}%）")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--min-points", type=int, default=5)
    ap.add_argument("--confirm", type=int, default=2)
    ap.add_argument("--ext-front", type=float, default=0.05)
    ap.add_argument("--ext-side", type=float, default=0.04)
    a = ap.parse_args()
    print(f"最小点数 {a.min_points}  確認 {a.confirm} フレーム  自車除外 前+{a.ext_front}m 横+{a.ext_side}m")
    for p in a.logs:
        replay(p, a)
    return 0


if __name__ == "__main__":
    sys.exit(main())
