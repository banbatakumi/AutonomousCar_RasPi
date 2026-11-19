"""推測航法（車速＋ジャイロ）だけで 1〜3 秒進めたときの誤差を、走行記録の ICP の姿勢を正として測る。

    python3 -m raspi.tools.dr_error ~/Downloads/surge_20261108_022016.mcap
    python3 -m raspi.tools.dr_error logs/*.mcap --horizons 1 2 3 --csv /tmp/dr.csv

大会計画（`docs/competition_plan.md` §4 決勝の方針 1.）の「前提の確認」。LiDAR が欠けた間を推測航法で
つなぐとき、何秒・どの速さまでなら道幅の余裕に収まるかを決める材料にする。

## やっていること

1. `/auto/state` の `pose_*`（`slam2d_raceline`・`slam2d_route` の ICP の姿勢）を正とする。姿勢の時刻は
   **使った点群の最新点の時刻**（`slam2d/core/deskew.py` の `t_ref`）なので、`/auto/state` の発行時刻ではなく
   `scan_age_ms` から使った `/scan` を引き当て、その点ごとの時刻の最大を使う
2. 正の姿勢の各時刻から、`/vehicle_state` の `speed`・`yaw_rate` だけを積分して `H` 秒進める
   （ジャイロのゼロ点は記録の静止区間で学んだ値。`raspi/rec/odom.py` の `DeadReckoner` と同じ考え方）
3. `H` 秒後の正の姿勢と比べ、誤差を**その時刻の車体座標の前後・横**と向きに分ける

窓の分類（窓の中の最大値で決める。決勝の方針で挙げた路面を記録の値から見分ける）:

- `slip` … 後輪のスリップ率（STM32 の `tc_slip`）の絶対値の最大が `SLIP_THR` 超（④低μ・空転・ロック）
- `bumpy` … 上下加速度の標準偏差が `BUMP_THR` 超（⑦でこぼこ・段差）
- `normal` … どちらでもない

ICP 自身が怪しい窓（`match_score` が `MIN_SCORE` 未満・見失い・姿勢の跳び）は除く。正の側も誤差を持つので、
**ここで出る値は「推測航法の誤差＋ICP の誤差」の上限**として読む（ICP はシムで 2〜4cm、`sim.slam_bench`）。
"""

from __future__ import annotations

import argparse
import bisect
import csv
import itertools
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.msgs.types import Scan  # noqa: E402
from raspi.nav.se2 import wrap_angle  # noqa: E402
from raspi.nav.deskew import point_times_ns  # noqa: E402

NS = 1_000_000_000

#: これ未満の一致度の正の姿勢は使わない
MIN_SCORE = 0.6
#: 窓の中で後輪スリップ率の絶対値がこれを超えたら `slip`
SLIP_THR = 0.3
#: 窓の中の上下加速度の標準偏差 [m/s²] がこれを超えたら `bumpy`
BUMP_THR = 2.0
#: 窓の平均車速がこれ未満なら数えない [m/s]（止まっている窓は誤差 0 で平均を薄める）
MIN_SPEED = 0.5
#: 正の姿勢が 1 周期（0.1s）でこれ以上跳んだら、その前後は ICP の取り直しとみて使わない [m]
JUMP_M = 0.25
#: 車速の上の分け目 [m/s]
SPEED_BINS = (0.5, 1.5, 2.5, 99.0)




def load(path: Path) -> dict:
    from mcap.reader import make_reader

    vs_t, vs_v, vs_w, vs_az, vs_slip, vs_stop = [], [], [], [], [], []
    scans: dict[int, int] = {}          # scan.t_capture → 最新点の時刻
    states: list[dict] = []
    with open(path, "rb") as f:
        r = make_reader(f)
        for _s, ch, m in r.iter_messages(topics=["/vehicle_state", "/scan", "/auto/state"]):
            d = json.loads(m.data)
            if ch.topic == "/vehicle_state":
                vs_t.append(d["t_capture"])
                vs_v.append(d["speed"])
                vs_w.append(d["yaw_rate"])
                vs_az.append(d["accel"][2])
                sr = d.get("tc_slip") or [0.0, 0.0]
                vs_slip.append(max(abs(sr[0]), abs(sr[1])))
                vs_stop.append(bool(d.get("stopped", False)))
            elif ch.topic == "/scan":
                if d["t_capture"] <= 0:
                    continue
                sc = Scan(**{k: v for k, v in d.items() if k in Scan.__struct_fields__})
                tp = point_times_ns(sc)
                valid = (np.asarray(sc.dist) > 0) & (tp > 0)
                scans[d["t_capture"]] = int(tp[valid].max()) if valid.any() else 0
            else:
                if d.get("phase") not in ("RACE", "LOCATED", "EXPLORE") or not d.get("mode"):
                    continue
                states.append({k: d[k] for k in ("t_pub", "scan_age_ms", "pose_x", "pose_y",
                                                 "pose_yaw", "match_score", "phase", "reason",
                                                 "engaged")})
    order = np.argsort(vs_t)
    vs = {k: np.asarray(v)[order] for k, v in
          (("t", vs_t), ("v", vs_v), ("w", vs_w), ("az", vs_az), ("slip", vs_slip),
           ("stop", vs_stop))}
    # 正の姿勢の時刻を点群の最新点へ合わせる
    scan_keys = sorted(scans)
    truth = []
    for s in states:
        # `scan_age_ms` は plan() を始めた時刻 now から測っているが、`/auto/state` は別の周期
        # （planning_node の送信ループ）で送られるので `t_pub` は now より 0〜80ms 遅い。
        # 使った点群の t_capture = now − age ≤ t_pub − age なので、それ以下で最新の周を採る
        # （次の周は 100ms 後なので、送信の遅れが 100ms 未満なら取り違えない）
        t_scan = s["t_pub"] - int(s["scan_age_ms"] * 1e6)
        j = bisect.bisect_right(scan_keys, t_scan + 1_000_000) - 1
        if j < 0 or scans[scan_keys[j]] <= 0:
            continue
        best = scan_keys[j]
        truth.append((scans[best], s["pose_x"], s["pose_y"], s["pose_yaw"], s["match_score"],
                      s["phase"], s["engaged"]))
    truth.sort()
    # 同じ点群を2回使った周（planning が点群を待たずに回った周）は1つにする
    dedup = []
    for row in truth:
        if dedup and dedup[-1][0] == row[0]:
            continue
        dedup.append(row)
    return {"vs": vs, "truth": dedup}


def gyro_bias(vs: dict) -> float:
    """静止中（止まって 0.5s 以上経った所）のヨーレートの中央値。"""
    t, w, stop = vs["t"], vs["w"], vs["stop"]
    still = np.zeros(len(t), dtype=bool)
    since = None
    for i in range(len(t)):
        if stop[i]:
            since = t[i] if since is None else since
            still[i] = (t[i] - since) > 0.5 * NS
        else:
            since = None
    return float(np.median(w[still])) if still.sum() > 50 else 0.0


@dataclass(frozen=True)
class Model:
    """推測航法の補正。既定値は実機の `slam2d` の予測と同じ（車速×1・ジャイロ×1・横速度 0）。

    - `speed_scale` … 車速の倍率。実機は走行中に点群で学ぶ（`slam2d/core/motion.py` の `SpeedScaleEstimator`）
    - `gyro_scale` … ヨーレートの倍率
    - `slip_c` … 後車軸の横すべり [s]。横速度 vy = −slip_c·v·ω（旋回の外へ流れる。横加速度 v·ω に比例）
    """
    speed_scale: float = 1.0
    gyro_scale: float = 1.0
    slip_c: float = 0.0


#: 補正なし（実機の予測と同じ）
NO_CORRECTION = Model()


def integrate(vs: dict, t0: int, t1: int, bias: float,
              model: Model = NO_CORRECTION) -> tuple[float, float, float]:
    """t0 の車体座標で、t0→t1 の推測航法の移動 (dx, dy, dyaw)。台形則（区間の中点の向きで進める）。"""
    t = vs["t"]
    i0 = max(0, int(np.searchsorted(t, t0)) - 1)
    i1 = min(len(t) - 1, int(np.searchsorted(t, t1)))
    tt = np.clip(t[i0:i1 + 1].astype(np.int64), t0, t1)    # 端を t0・t1 に切る
    v = vs["v"][i0:i1 + 1] * model.speed_scale
    w = (vs["w"][i0:i1 + 1] - bias) * model.gyro_scale
    dt = np.diff(tt) / NS
    vm = 0.5 * (v[1:] + v[:-1])
    wm = 0.5 * (w[1:] + w[:-1])
    yaw_end = np.cumsum(wm * dt)
    ym = yaw_end - 0.5 * wm * dt
    vy = -model.slip_c * vm * wm
    c, s = np.cos(ym), np.sin(ym)
    x = float(np.sum((vm * c - vy * s) * dt))
    y = float(np.sum((vm * s + vy * c) * dt))
    return x, y, float(yaw_end[-1]) if yaw_end.size else 0.0


def windows(data: dict, horizons: list[float], bias: float,
            model: Model = NO_CORRECTION) -> list[dict]:
    vs, truth = data["vs"], data["truth"]
    tt = np.array([r[0] for r in truth], dtype=np.int64)
    xs = np.array([r[1] for r in truth])
    ys = np.array([r[2] for r in truth])
    # 跳び（ICP の取り直し）の前後を印す
    bad = np.array([r[4] < MIN_SCORE or r[5] != "RACE" for r in truth])
    jump = np.zeros(len(truth), dtype=bool)
    for k in range(1, len(truth)):
        dtk = (tt[k] - tt[k - 1]) / NS
        if dtk > 0.3:
            continue
        vmax = float(np.max(np.abs(vs["v"][(vs["t"] >= tt[k - 1]) & (vs["t"] <= tt[k])]), initial=0))
        if math.hypot(xs[k] - xs[k - 1], ys[k] - ys[k - 1]) > vmax * dtk + JUMP_M:
            jump[k - 1] = jump[k] = True
    out = []
    for i, (t0, x0, y0, yaw0, *_rest) in enumerate(truth):
        if bad[i] or jump[i]:
            continue
        for h in horizons:
            t_goal = t0 + int(h * NS)
            j = int(np.searchsorted(tt, t_goal))
            cand = [k for k in (j - 1, j) if 0 <= k < len(tt)]
            if not cand:
                continue
            j = min(cand, key=lambda k: abs(tt[k] - t_goal))
            if abs(tt[j] - t_goal) > 0.06 * NS or j <= i:
                continue
            if bad[i:j + 1].any() or jump[i:j + 1].any():
                continue
            t1 = int(tt[j])
            m = (vs["t"] >= t0) & (vs["t"] <= t1)
            if m.sum() < 20:
                continue
            gaps = np.diff(vs["t"][m])
            if gaps.size and gaps.max() > 0.05 * NS:
                continue
            vmean = float(np.mean(np.abs(vs["v"][m])))
            if vmean < MIN_SPEED:
                continue
            dx, dy, dyaw = integrate(vs, t0, t1, bias, model)
            c, s = math.cos(yaw0), math.sin(yaw0)
            px, py, pyaw = x0 + c * dx - s * dy, y0 + s * dx + c * dy, yaw0 + dyaw
            ex, ey = px - xs[j], py - ys[j]
            yt = truth[j][3]
            ct, st = math.cos(yt), math.sin(yt)
            e_long = ct * ex + st * ey
            e_lat = -st * ex + ct * ey
            dist = float(np.trapezoid(np.abs(vs["v"][m]), vs["t"][m] / NS))
            slip = float(vs["slip"][m].max())
            bump = float(np.std(vs["az"][m]))
            kind = "slip" if slip > SLIP_THR else ("bumpy" if bump > BUMP_THR else "normal")
            out.append({"t0": t0, "t1": t1, "j": j, "h": h, "x0": x0, "y0": y0, "yaw0": yaw0, "v": vmean, "dist": dist,
                        "e_long": e_long, "e_lat": e_lat,
                        "e_yaw_deg": math.degrees(wrap_angle(pyaw - yt)),
                        "slip": slip, "bump": bump, "kind": kind})
    return out


def _errors(vs: dict, truth: list, r: dict, bias: float, model: Model) -> tuple[float, float, float]:
    dx, dy, dyaw = integrate(vs, r["t0"], r["t1"], bias, model)
    c, s = math.cos(r["yaw0"]), math.sin(r["yaw0"])
    _t, xt, yt_, yawt, *_ = truth[r["j"]]
    ex = r["x0"] + c * dx - s * dy - xt
    ey = r["y0"] + s * dx + c * dy - yt_
    ct, st = math.cos(yawt), math.sin(yawt)
    return ct * ex + st * ey, -st * ex + ct * ey, wrap_angle(r["yaw0"] + dyaw - yawt)


def fit(runs: list[tuple[dict, float, list[dict]]], h: float) -> Model:
    """`h` 秒の窓の前後・横・向き（1m を掛けて長さに直す）の誤差の二乗和が最小になる `Model`。

    `normal` の窓だけで当てはめる（滑った・跳ねた窓は推測航法のモデルの外の誤差）。
    """
    from scipy.optimize import least_squares

    items = [(d, b, r) for d, b, rows in runs for r in rows if r["h"] == h and r["kind"] == "normal"]
    if len(items) < 20:
        raise SystemExit(f"当てはめに使える窓が {len(items)} しかない")

    def res(q):
        m = Model(*q)
        out = []
        for d, b, r in items:
            el, ea, ey = _errors(d["vs"], d["truth"], r, b, m)
            out += [el, ea, ey * 1.0]
        return out

    sol = least_squares(res, x0=[1.0, 1.0, 0.0], loss="soft_l1", f_scale=0.05)
    return Model(*map(float, sol.x))


def _p(a, q):
    return float(np.percentile(a, q)) if len(a) else float("nan")


def report(rows: list[dict], horizons: list[float]) -> None:
    print(f"{'H[s]':>4} {'分類':<7} {'車速':<9} {'n':>5} {'前後p50':>8} {'前後p95':>8} "
          f"{'横p50':>7} {'横p95':>7} {'横max':>7} {'向きp95[°]':>10} {'前後/距離p50':>12}")
    for h in horizons:
        for kind in ("all", "normal", "slip", "bumpy"):
            for lo, hi in [(SPEED_BINS[0], SPEED_BINS[-1])] + list(itertools.pairwise(SPEED_BINS)):
                sel = [r for r in rows if r["h"] == h and (kind == "all" or r["kind"] == kind)
                       and lo <= r["v"] < hi]
                if not sel:
                    continue
                el = np.abs([r["e_long"] for r in sel])
                ea = np.abs([r["e_lat"] for r in sel])
                ey = np.abs([r["e_yaw_deg"] for r in sel])
                rel = np.abs([r["e_long"] / r["dist"] for r in sel if r["dist"] > 0.3])
                vb = "全部" if hi == SPEED_BINS[-1] and lo == SPEED_BINS[0] else f"{lo:.1f}-{hi:.1f}" \
                    if hi < 50 else f"{lo:.1f}+"
                print(f"{h:>4.1f} {kind:<7} {vb:<9} {len(sel):>5} {_p(el, 50):>8.3f} {_p(el, 95):>8.3f} "
                      f"{_p(ea, 50):>7.3f} {_p(ea, 95):>7.3f} {ea.max():>7.3f} {_p(ey, 95):>10.2f} "
                      f"{_p(rel, 50) * 100:>11.1f}%")
    # 前後の誤差の符号（車速の倍率のずれ）
    for h in horizons:
        sel = [r for r in rows if r["h"] == h and r["dist"] > 0.3 and r["kind"] == "normal"]
        if sel:
            rel = np.array([r["e_long"] / r["dist"] for r in sel])
            print(f"H={h:.1f}s normal: 前後の誤差 / 走行距離 の中央値 {np.median(rel) * 100:+.2f}%"
                  "（正 = 推測航法が進みすぎ＝車速が大きめ）")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("paths", nargs="+", type=Path)
    ap.add_argument("--horizons", nargs="+", type=float, default=[1.0, 2.0, 3.0])
    ap.add_argument("--model", nargs=3, type=float, metavar=("SPEED_SCALE", "GYRO_SCALE", "SLIP_C"),
                    help="この補正を掛けて測る（別の記録で当てはめた値の検証用）")
    ap.add_argument("--fit", type=float, metavar="H",
                    help="H 秒の窓で補正（Model）を当てはめ、その補正を掛けた結果も出す")
    ap.add_argument("--csv", type=Path, help="窓ごとの値を書き出す")
    a = ap.parse_args()
    model = Model(*a.model) if a.model else NO_CORRECTION
    runs: list[tuple[dict, float, list[dict]]] = []
    for p in a.paths:
        try:
            data = load(p)
        except Exception as e:      # noqa: BLE001  末尾の切れた記録など。直すのは raspi.tools.mcap_repair
            print(f"# {p.name}: 読めない（{type(e).__name__}: {e}）。飛ばす")
            continue
        bias = gyro_bias(data["vs"])
        rows = windows(data, a.horizons, bias, model)
        n_race = sum(1 for r in data["truth"] if r[5] == "RACE")
        print(f"# {p.name}: 正の姿勢 {len(data['truth'])}（RACE {n_race}）・ジャイロのゼロ点 "
              f"{math.degrees(bias):+.3f}°/s・窓 {len(rows)}")
        for r in rows:
            r["file"] = p.name
        runs.append((data, bias, rows))
    rows_all = [r for _d, _b, rows in runs for r in rows]
    if not rows_all:
        print("使える窓がない（slam2d の RACE 段の記録か確かめる）")
        return 1
    print(f"\n## 補正 {model}")
    report(rows_all, a.horizons)
    if a.fit:
        m = fit(runs, a.fit)
        print(f"\n## 当てはめた補正（H={a.fit:.1f}s・normal の窓） {m}")
        print(f"   → --model {m.speed_scale:.4f} {m.gyro_scale:.4f} {m.slip_c:.4f}")
        rows_all = []
        for data, bias, _rows in runs:
            rows_all += windows(data, a.horizons, bias, m)
        report(rows_all, a.horizons)
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows_all[0].keys()))
            w.writeheader()
            w.writerows(rows_all)
        print(f"→ {a.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
