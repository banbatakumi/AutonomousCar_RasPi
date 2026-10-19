"""校正 2・3: ステア。設計は `docs/calibration.md` §4.2・§4.3。

## 2. 静特性（`calib_steer` の記録 → 舵のリンクの換算・中立のずれ・サーボの定常ゲイン）

STM32 はモータ角 x（×リンク比 0.5）を路面舵角に換算して報告・指令する: δ_報告 = g·x + c·x³（`[control] steer_link_*`）。
この換算が実際とずれていると、slam2d の「狙う曲率 → 指令舵角」が狂う。

各弧（舵角を固定した前進・後退。舵が動き終わった後、車速 0.2m/s 以上）で

    曲率 κ = 向きの変化 ÷ 走った距離 = ∫(ヨーレート − バイアス)dt ÷ ∫車速 dt
    実際の舵角 δ = atan(κ·L)

を出し（低速なので横滑りは無視できる）、報告された舵角を記録時の換算の逆で x に戻して

    δ = g·x + c·x³ + δ0

を最小二乗で当てはめる。g・c が新しい換算（STM32 へ）、δ0 が中立のずれ（`steer_offset_rad`。Pi が指令から引く）。
換算を入れ直すと報告値の意味が変わるので、`[dynamics] steer_gain`=1・`steer_gain_cubic`=0・`max_steer`（可動範囲 ±30°
を新しい換算に通した値）を**そろえて**書く。サーボの定常ゲインは、保持中の報告値 ÷ STM32 の参照舵角（`steer_cmd_echo`）。

ジャイロのバイアスは試験の頭の静止（`settle`）で読む。

## 3. 動特性（`calib_steer_dyn` の記録 → 舵の遅れ・ヨーの遅れ）

- サーボ: 参照舵角 `steer_cmd_echo` → 報告舵角 `steer_actual` に「むだ時間＋1次遅れ」（格子で探す）
- ヨー: 報告舵角から出る曲率 tan(δ(x))/L → 実際の曲率（ヨーレート ÷ 車速）に 2 次遅れ（ωn・ζ）
"""

from __future__ import annotations

import math

import numpy as np
from scipy.signal import lfilter

from raspi.auto._calib_common import phase_parts
from raspi.core.control_params import STEER_LINK_X_MAX_RAD, steer_link_road, steer_link_x

from .encoder import _stm_time
from .log import Log
from .result import Result

__all__ = ["analyze_static", "analyze_dynamic", "arc_points", "ArcPoint"]

WHEELBASE_M = 0.23
_SKIP_M = 0.08
_MIN_V = 0.2
_STEER_SETTLED_RAD = 0.006
_SLOPE_MIN = 0.3         # STM32 の `STEERING_LINKAGE_MIN_SLOPE`


class ArcPoint:
    def __init__(self, level: int, direction: int, reported: float, echo: float, kappa: float, dist: float):
        self.level, self.direction, self.reported, self.echo = level, direction, reported, echo
        self.kappa, self.dist = kappa, dist


def _bias(log: Log) -> float:
    names = log.phase_of_samples()
    v = np.abs(log.vs["speed"])
    m = np.array([n == "settle" for n in names]) & (v < 0.005)
    if m.sum() < 20:
        m = v < 0.003
    return float(np.mean(log.vs["yaw_rate"][m])) if m.any() else 0.0


def recorded_link(log: Log) -> tuple[float, float]:
    cp = log.control_params
    return float(cp.get("steer_link_gain", 1.0)), float(cp.get("steer_link_cubic", 0.0))


def arc_points(log: Log) -> list[ArcPoint]:
    t = _stm_time(log)
    od = log.vs["odom_dist"].mean(axis=1)
    steer = log.vs["steer_actual"]
    ds = np.diff(od, prepend=od[0]) * np.cos(steer)
    dt = np.diff(t, prepend=t[0])
    v = np.where(dt > 0, ds / np.where(dt > 0, dt, 1), 0.0)
    b = _bias(log)
    yaw = log.vs["yaw_rate"] - b
    out = []
    segs = log.segments(lambda p: (phase_parts(p) or ("",))[0] in ("out", "back"))
    if not segs:
        segs = _legacy_arcs(log)
    for name, i0, i1 in segs:
        kind, level, direction = phase_parts(name)
        idx = np.arange(i0, i1)
        travelled = np.abs(np.cumsum(ds[idx]))
        idx = idx[(travelled >= _SKIP_M) & (np.abs(v[idx]) >= _MIN_V)]
        if len(idx) < 8:
            continue
        med = float(np.median(steer[idx]))
        idx = idx[np.abs(steer[idx] - med) < _STEER_SETTLED_RAD]
        if len(idx) < 8:
            continue
        dist = float(np.sum(ds[idx]))
        if abs(dist) < 0.05:
            continue
        kappa = float(np.sum(yaw[idx] * dt[idx]) / dist)
        out.append(ArcPoint(level, direction, float(np.mean(steer[idx])), float(np.mean(log.vs["steer_cmd_echo"][idx])),
                            kappa, abs(dist)))
    return out


def _legacy_arcs(log: Log) -> list[tuple[str, int, int]]:
    """区間の名前を持たない旧 planner（`sysid_steer`）の記録: 舵の指令が一定で、低速（0.7m/s 以下）で一定の向きに
    走っている区間を弧として拾う。"""
    cmd_steer = log.cmd_at("target_steer", log.t - 0.15)
    cmd_v = log.cmd_at("target_speed", log.t - 0.15)
    out = []
    i, n, lv = 0, len(log.t), 0
    while i < n:
        j = i
        while (j < n and abs(cmd_steer[j] - cmd_steer[i]) < 1e-4 and np.sign(cmd_v[j]) == np.sign(cmd_v[i])
               and abs(cmd_v[j]) > 0.1):
            j += 1
        if j - i >= 40 and abs(cmd_v[i]) <= 0.7:
            out.append((f"out/{lv}/{'+' if cmd_v[i] > 0 else '-'}", i, j))
            lv += 1
        i = max(j, i + 1)
    return out


def analyze_static(log: Log) -> Result:
    res = Result("校正 2: ステア（舵角→曲率）")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）。残った弧で解析する")
    pts = arc_points(log)
    if len(pts) < 10:
        res.fail(f"使える弧が {len(pts)} 本しかない（`calib_steer` の記録か確かめる）")
        return res
    g0, c0 = recorded_link(log)
    x = np.array([steer_link_x(p.reported, g0, c0) for p in pts])
    delta = np.arctan(np.array([p.kappa for p in pts]) * WHEELBASE_M)
    X = np.column_stack([x, x ** 3, np.ones_like(x)])
    coef, *_ = np.linalg.lstsq(X, delta, rcond=None)
    g, c, d0 = (float(v) for v in coef)
    r = delta - X @ coef
    resid_deg = math.degrees(float(np.sqrt(np.mean(r ** 2))))
    # STM32 は可動範囲の端で傾きが下限を割らないよう cubic を丸める（`Steering_ClampLinkageCubic`）
    c_min = (_SLOPE_MIN - g) / (3 * STEER_LINK_X_MAX_RAD ** 2)
    if c < c_min:
        res.warn(f"当てはめた換算は可動範囲の端で効きが寝すぎる（cubic {c:.3f} < {c_min:.3f}）。STM32 が丸める値にする")
        c = c_min
    res.info(f"弧 {len(pts)} 本（前進 {sum(p.direction > 0 for p in pts)}・後退 {sum(p.direction < 0 for p in pts)}）、"
             f"ジャイロのバイアス {_bias(log):+.4f}rad/s（試験の頭の静止で読んだ値）")
    res.info(f"実際の舵角 = {g:.4f}·x {c:+.4f}·x³ {d0:+.5f}rad（x = モータ角×0.5、記録時の換算は {g0:.4f}·x {c0:+.4f}·x³）")
    if resid_deg > 0.5:
        res.warn(f"当てはめの残差 {resid_deg:.2f}° が大きい（目安 0.5°。路面のむら・タイヤの横滑り・ガタ？）")
    else:
        res.ok(f"当てはめの残差 {resid_deg:.2f}°")
    # 前進と後退の差（同じ段）
    diffs = []
    for lv in sorted({p.level for p in pts}):
        f = [p.kappa for p in pts if p.level == lv and p.direction > 0]
        bk = [p.kappa for p in pts if p.level == lv and p.direction < 0]
        if f and bk:
            diffs.append(math.degrees(math.atan(np.mean(f) * WHEELBASE_M) - math.atan(np.mean(bk) * WHEELBASE_M)))
    if diffs:
        dmax = float(np.max(np.abs(diffs)))
        if dmax > 1.0:
            res.warn(f"同じ舵角の前進と後退で実際の舵角が最大 {dmax:.2f}° 違う（目安 1°。横滑り・ガタが大きい）")
        else:
            res.ok(f"前進と後退で実際の舵角が一致（差 最大 {dmax:.2f}°）")
    for deg in (10.0, 20.0):
        xx = math.radians(deg)
        old = steer_link_road(xx, g0, c0)
        new = g * xx + c * xx ** 3 + d0
        res.info(f"x = {deg:.0f}° のとき: 記録時の換算 {math.degrees(old):.2f}° → 実際 {math.degrees(new):.2f}°（差 "
                 f"{math.degrees(new - old):+.2f}°）")
    # サーボの定常ゲイン（報告 ÷ 参照）。中立付近は比が不安定なので外す
    rep = np.array([p.reported for p in pts])
    ech = np.array([p.echo for p in pts])
    m = np.abs(ech) > 0.05
    servo = float(np.sum(rep[m] * ech[m]) / np.sum(ech[m] ** 2)) if m.sum() >= 4 else 1.0
    res.info(f"サーボの定常ゲイン（保持中の報告舵角 ÷ 参照舵角）{servo:.4f}")
    if abs(servo - 1.0) > 0.08:
        res.warn(f"サーボの定常ゲイン {servo:.3f} が 1 から 8% 以上ずれる（ステアの位置制御が押し負けている）")
    max_steer = steer_link_road(STEER_LINK_X_MAX_RAD, g, c)
    res.values.update({
        "control.steer_link_gain": round(g, 6), "control.steer_link_cubic": round(c, 6),
        "dynamics.steer_offset_rad": round(d0, 6), "dynamics.steer_servo_gain": round(servo, 5),
        "dynamics.steer_gain": 1.0, "dynamics.steer_gain_cubic": 0.0, "max_steer": round(max_steer, 6),
    })
    res.notes.update({"control.steer_link_gain": "x→実際の舵角（STM32 が換算する）", "dynamics.steer_gain": "換算に畳み込んだので 1",
                      "max_steer": "可動範囲 ±30° を新しい換算に通した値", "dynamics.steer_offset_rad": "Pi が指令から引く中立のずれ"})
    xs = np.linspace(-STEER_LINK_X_MAX_RAD, STEER_LINK_X_MAX_RAD, 61)
    res.plot("実際の舵角 [°]（横軸: x [°]）", "当てはめ", np.degrees(xs), np.degrees(g * xs + c * xs ** 3 + d0))
    res.plot("実際の舵角 [°]（横軸: x [°]）", "記録時の換算", np.degrees(xs), np.degrees([steer_link_road(v, g0, c0) for v in xs]))
    res.plot("実際の舵角 [°]（横軸: x [°]）", "実測", np.degrees(x), np.degrees(delta))
    return res


# ── 3. 動特性 ────────────────────────────────────────────────────────────

_FINE = 0.001


def _first_order_delay(t: np.ndarray, u: np.ndarray, d: float, tau: float, gain: float = 1.0) -> np.ndarray:
    """サンプル列 u(t) をむだ時間 d・1次遅れ τ に通した値（サンプルの時刻で）。入力はサンプルの間で線形に補間。"""
    tf = np.arange(t[0], t[-1] + 1e-9, _FINE)
    uf = np.interp(tf - d, t, u, left=u[0])
    if tau > 1e-5:
        a = math.exp(-_FINE / tau)
        uf, _ = lfilter([1 - a], [1, -a], uf, zi=[a * uf[0]])
    return gain * np.interp(t, tf, uf)


def _second_order(t: np.ndarray, u: np.ndarray, wn: float, zeta: float) -> np.ndarray:
    """単位ゲインの 2 次遅れ wn²/(s² + 2ζwn·s + wn²)（双一次変換で離散化。初期値は u の頭で静止）。"""
    from scipy.signal import bilinear, lfiltic
    tf = np.arange(t[0], t[-1] + 1e-9, _FINE)
    uf = np.interp(tf, t, u)
    b, a = bilinear([wn * wn], [1.0, 2 * zeta * wn, wn * wn], fs=1.0 / _FINE)
    zi = lfiltic(b, a, [uf[0]] * 2, [uf[0]] * 2)
    y, _ = lfilter(b, a, uf, zi=zi)
    return np.interp(t, tf, y)


def analyze_dynamic(log: Log, link: tuple[float, float] | None = None, offset: float = 0.0) -> Result:
    """`link` は実際の舵角の換算（校正 2 の値。既定は記録時の換算＝合っているとみなす）。"""
    res = Result("校正 3: ステアの遅れ")
    t = _stm_time(log)
    names = log.phase_of_samples()
    sel = np.array([(phase_parts(n) or ("",))[0] == "zigzag" for n in names])
    if sel.sum() < 100:
        res.fail("舵を振っている区間が見つからない（`calib_steer_dyn` の記録か確かめる）")
        return res
    # サーボ: 区間ごとに当てはめる（区間の間は切れている）
    segs = log.segments(lambda p: (phase_parts(p) or ("",))[0] == "zigzag")
    best = None
    for d in np.arange(0.0, 0.0605, 0.0025):
        for tau in np.arange(0.0, 0.0805, 0.004):
            err = 0.0
            num = den = 0.0
            for _, i0, i1 in segs:
                pr = _first_order_delay(t[i0:i1], log.vs["steer_cmd_echo"][i0:i1], d, tau)
                y = log.vs["steer_actual"][i0:i1]
                num += float(np.dot(pr, y))
                den += float(np.dot(pr, pr))
            gain = num / den if den > 0 else 1.0
            for _, i0, i1 in segs:
                pr = _first_order_delay(t[i0:i1], log.vs["steer_cmd_echo"][i0:i1], d, tau, gain)
                err += float(np.sum((pr - log.vs["steer_actual"][i0:i1]) ** 2))
            if best is None or err < best[0]:
                best = (err, float(d), float(tau), gain)
    _, d_s, tau_s, gain_s = best
    n = sum(i1 - i0 for _, i0, i1 in segs)
    rms = math.degrees(math.sqrt(best[0] / n))
    res.info(f"サーボ: むだ時間 {d_s * 1000:.1f}ms・1次遅れ {tau_s * 1000:.1f}ms・ゲイン {gain_s:.3f}（残差 {rms:.2f}°）")
    # ヨー: 報告舵角 → 運動学の曲率 → 2 次遅れ → 実曲率
    g0, c0 = recorded_link(log)
    lk = link or (g0, c0)
    bias = _bias(log)
    best_y = None
    for wn in np.geomspace(6.0, 150.0, 30):
        for zeta in np.arange(0.4, 1.81, 0.1):
            err = 0.0
            for _, i0, i1 in segs:
                rep = log.vs["steer_actual"][i0:i1]
                x = np.array([steer_link_x(v, g0, c0) for v in rep])
                delta = lk[0] * x + lk[1] * x ** 3 + offset
                k_kin = np.tan(delta) / WHEELBASE_M
                v = log.vs["speed"][i0:i1]
                ok = np.abs(v) > 0.5
                k_meas = np.where(ok, (log.vs["yaw_rate"][i0:i1] - bias) / np.where(ok, v, 1.0), 0.0)
                pr = _second_order(t[i0:i1], k_kin, wn, zeta)
                err += float(np.sum(((pr - k_meas) * ok) ** 2))
            if best_y is None or err < best_y[0]:
                best_y = (err, wn, zeta)
    _, wn, zeta = best_y
    yaw_delay = 2 * zeta / wn
    res.info(f"ヨー: 2 次遅れ ωn {wn:.0f}rad/s・ζ {zeta:.1f}（遅れにして {yaw_delay * 1000:.0f}ms）")
    total = d_s + tau_s + yaw_delay
    res.info(f"舵を送ってから曲率が付くまでの遅れの合計（slam2d の先読み `steer_delay_s`）: {total * 1000:.0f}ms")
    if total > 0.25:
        res.warn(f"遅れの合計 {total * 1000:.0f}ms が大きい（目安 250ms 以下）")
    res.values.update({"dynamics.dead_time_s": round(d_s, 4), "dynamics.tau_steer_s": round(tau_s, 4),
                       "dynamics.yaw_natural_freq_rad_s": round(wn, 2), "dynamics.yaw_damping": round(zeta, 3)})
    _, i0, i1 = segs[0]
    res.plot("舵角 [rad]（横軸: 時刻 [s]）", "参照（STM32 のランプ）", log.t[i0:i1], log.vs["steer_cmd_echo"][i0:i1])
    res.plot("舵角 [rad]（横軸: 時刻 [s]）", "報告", log.t[i0:i1], log.vs["steer_actual"][i0:i1])
    return res
