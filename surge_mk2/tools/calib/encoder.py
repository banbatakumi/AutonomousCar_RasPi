"""校正 1: 前輪エンコーダ（`raspi/auto/calib_encoder.py` の記録 → `[control] enc_*`）。

設計は `docs/calibration.md` §4.1。STM32 の式（`lib/encoder_cal/encoder_cal.h`）:

    θm = wrap(2π·(u − lo)/(hi − lo)),   θm = θ + e(θ),
    e(θ) = h1s·sinθ + h1c·cosθ + h2s·sin2θ + h2c·cos2θ

u は TELEMETRY の `front_angle`÷2π（校正前の生の読み）、θ は真の角度（取付向きの符号のまま）。

## 当てはめ

惰行の区間 k ごとに真の角度を時間の 3 次式 θ = P_k(t) と置き、全区間で共通の (hi − lo, h1s…h2c) と
各区間の P_k を、読みとの角度の差（±π に畳む）の最小二乗で同時に決める。惰行 1 回で車輪は数回転するので、
角度の関数（周期誤差）と時間の関数（減速）は分かれる。外れ値（ADC のグリッチ）に引かれないよう soft-L1 の損失。

## lo は当てはめない（区別できない）

lo をずらすのは角度の原点を回すのと同じで、周期誤差の位相を同じだけ回せば**校正後の角度は定数しか
変わらない**（走行距離・車速は角度の差しか使わない）。だから記録からは決まらない。lo は読みの最小値
（センサが出せる下端）に置き、hi = lo + 当てはめた幅 とする。こうしておくと周期誤差の位相が記録をまたいで
比べられる（同じ車なら同じ値に戻る）。

## 確かめ

- 速さの違う惰行（`calib_encoder` の `speeds`）だけで別々に当てはめた周期誤差が、標準誤差の範囲で一致すること
  （一致しなければ角度の関数でない誤差＝振動・ADC の遅れが混ざっている）
- 残差のばらつきが 0.5° 以下であること（2026-10-18 の実機で 0.2〜0.3°。静止の ADC ノイズ 0.04° より大きいのは走行中の振動）
- 残差に 3・4 次の高調波（STM32 の式が持たない成分）が残っていないこと（実機で 0.1° 以下）
- 記録したときに STM32 に校正が入っていれば、**STM32 が出した校正後の走行距離 `odom_dist`** に残る周期誤差を測る
  （校正が正しく効いていれば測定ノイズ程度まで落ちる）
"""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares

from raspi.auto._calib_common import phase_parts

from .log import Log
from .result import Result

__all__ = ["analyze", "fit_wheel", "cal_angle", "WheelFit", "SIDES", "DIRS"]

TWO_PI = 2.0 * math.pi
#: 左右の並び（`front_angle`・`odom_dist` の添字）と、取付向きの符号（ファーム `DRIVE_FRONT_*_DIR`）
SIDES = ("left", "right")
DIRS = (-1.0, +1.0)
WHEEL_RADIUS_M = 0.030
#: 惰行の区間の頭で捨てる時間 [s]（指令が届いて MD のトルクが抜けるまで）と、末尾で捨てる時間
_TRIM_HEAD_S = 0.08
_TRIM_TAIL_S = 0.03
#: これより遅いサンプルは使わない [m/s]（止まりかけは転がり抵抗が滑らかでない）
_MIN_SPEED = 0.12
#: 1 区間に要る回転 [回]
_MIN_REVS_SEG = 1.5
_MIN_REVS_TOTAL = 10.0
_POLY_DEG = 3
#: STM32 が受け付ける範囲（ファーム `control_params.h`。外れると丸められ、ARM できない）
LO_MAX = 0.2
HI_MIN, HI_MAX = 0.8, 1.0
H_MAX = 0.1
#: 合否の目安 [°]
_RESID_WARN_DEG = 0.5
#: STM32 が直せない 3・4 次の高調波がこれを超えたら知らせる [°]
_HIGHER_WARN_DEG = 0.3
_POST_RESID_OK_DEG = 0.2


def _wrap_pi(a):
    return (a + math.pi) % TWO_PI - math.pi


def ripple(h: np.ndarray, theta) -> np.ndarray:
    """e(θ)。`h` = (h1s, h1c, h2s, h2c)。"""
    return h[0] * np.sin(theta) + h[1] * np.cos(theta) + h[2] * np.sin(2 * theta) + h[3] * np.cos(2 * theta)


def cal_angle(u: np.ndarray, lo: float, hi: float, h) -> np.ndarray:
    """STM32 の `EncoderCal_Angle` と同じ計算（生の読み u → 校正後の角度 [rad]、0〜2π）。"""
    span = hi - lo if hi - lo > 0.5 else 1.0
    h = np.asarray(h, dtype=float)
    theta_m = np.mod((u - lo) / span * TWO_PI, TWO_PI)
    theta = theta_m.copy()
    for _ in range(2):
        theta = theta_m - ripple(h, theta)
    return np.mod(theta, TWO_PI)


def amplitude_deg(h) -> float:
    th = np.linspace(0, TWO_PI, 721)
    return float(np.degrees(np.max(np.abs(ripple(np.asarray(h), th)))))


def _stm_time(log: Log) -> np.ndarray:
    """STM32 の時計 [s]（TELEMETRY の `t_stm_us`。Pi 側の受信の揺れが無い）。無ければ Pi の時刻。"""
    us = log.vs["t_stm_us"]
    if np.all(us > 0) and len(us) > 1:
        d = np.diff(us)
        d = np.where(d < -2 ** 31, d + 2 ** 32, d)
        t = np.concatenate([[0.0], np.cumsum(d)]) * 1e-6
        if np.all(np.diff(t) > 0):
            return t + log.t[0]
    return log.t


def coast_segments(log: Log) -> list[tuple[int, int, float]]:
    """使える惰行の区間 `(先頭, 末尾+1, 惰行に入った車速)`。"""
    t = log.t
    tq = np.abs(log.vs["torque_cmd"]).sum(axis=1)
    v = np.abs(log.vs["speed"])
    out = []
    segs = log.segments(lambda p: (phase_parts(p) or ("",))[0] == "coast")
    if not segs:
        # 区間の名前を持たない旧 planner（`sysid_encoder`）の記録: 理由の文の「惰行」で拾う
        segs = log.segments_by_reason(lambda r: r.startswith("惰行"))
    for name, i0, i1 in segs:
        # 指令が届いてトルクが 0 になってから
        j = i0
        while j < i1 and tq[j] > 1e-6:
            j += 1
        if j >= i1:
            continue
        t_start = t[j] + _TRIM_HEAD_S
        t_end = t[i1 - 1] - _TRIM_TAIL_S
        idx = np.arange(j, i1)
        idx = idx[(t[idx] >= t_start) & (t[idx] <= t_end) & (v[idx] >= _MIN_SPEED) & (tq[idx] <= 1e-6)]
        if len(idx) < 15:
            continue
        out.append((int(idx[0]), int(idx[-1]) + 1, float(v[j])))
    return out


class WheelFit:
    """1輪ぶんの当てはめの結果。"""

    def __init__(self, side: str, lo: float, span: float, h: np.ndarray, h_se: np.ndarray, resid_deg: float,
                 n_points: int, revs: float, span_se: float, higher_deg: float = 0.0):
        self.side, self.lo, self.span, self.h, self.h_se = side, lo, span, h, h_se
        self.resid_deg, self.n_points, self.revs, self.span_se = resid_deg, n_points, revs, span_se
        #: 残差の 3・4 次の高調波の振幅の大きい方 [°]
        self.higher_deg = higher_deg

    @property
    def hi(self) -> float:
        return self.lo + self.span

    @property
    def amp_deg(self) -> float:
        return amplitude_deg(self.h)

    def values(self) -> dict[str, float]:
        s = self.side
        out = {f"control.enc_{s}_lo": round(self.lo, 6), f"control.enc_{s}_hi": round(self.hi, 6)}
        for k, v in zip(("h1s", "h1c", "h2s", "h2c"), self.h):
            out[f"control.enc_{s}_{k}"] = round(float(v), 6)
        return out


def fit_wheel(t: np.ndarray, u: np.ndarray, segs: list[tuple[int, int]], side: str = "",
              lo: float | None = None, _lo_retry: int = 0) -> WheelFit:
    """生の読み `u`（0〜1）の惰行の区間 `segs` から出力の範囲と周期誤差を当てはめる。"""
    if lo is None:
        allu = np.concatenate([u[a:b] for a, b in segs])
        lo = float(np.min(allu))
    # 初期値: 幅 = 観測した範囲、周期誤差 0、各区間の 3 次式は読みをほどいて当てはめたもの
    allu = np.concatenate([u[a:b] for a, b in segs])
    span0 = float(np.max(allu) - lo)
    span0 = span0 if 0.9 < span0 <= 1.0 else 1.0
    taus, polys, meas = [], [], []
    for a, b in segs:
        tt = t[a:b]
        tau = (tt - 0.5 * (tt[0] + tt[-1])) / max(0.5 * (tt[-1] - tt[0]), 1e-3)
        th = np.unwrap(np.mod((u[a:b] - lo) / span0 * TWO_PI, TWO_PI))
        c = np.polyfit(tau, th, _POLY_DEG)
        taus.append(tau)
        polys.append(c)
        meas.append(u[a:b])
    nseg = len(segs)
    npoly = _POLY_DEG + 1

    def unpack(x):
        return x[0], x[1:5], x[5:].reshape(nseg, npoly)

    def resid(x):
        span, h, cs = unpack(x)
        out = []
        for k in range(nseg):
            th = np.polyval(cs[k], taus[k])
            pred = th + ripple(h, th)
            got = (meas[k] - lo) / span * TWO_PI
            out.append(_wrap_pi(got - pred))
        return np.concatenate(out)

    x0 = np.concatenate([[span0], np.zeros(4), np.concatenate(polys)])
    sol = least_squares(resid, x0, loss="soft_l1", f_scale=math.radians(0.5), x_scale="jac")
    r = resid(sol.x)
    sigma = float(np.sqrt(np.mean(r ** 2)))
    J = sol.jac
    try:
        cov = np.linalg.inv(J.T @ J) * sigma ** 2
        se = np.sqrt(np.clip(np.diag(cov), 0, None))
    except np.linalg.LinAlgError:
        se = np.full(len(sol.x), np.nan)
    span, h, cs = unpack(sol.x)
    revs = 0.0
    ths = []
    for k in range(nseg):
        th = np.polyval(cs[k], taus[k])
        revs += abs(th[-1] - th[0]) / TWO_PI
        ths.append(th)
    th = np.concatenate(ths)
    higher = 0.0
    for order in (3, 4):
        X = np.column_stack([np.sin(order * th), np.cos(order * th)])
        c, *_ = np.linalg.lstsq(X, r, rcond=None)
        higher = max(higher, math.degrees(float(np.hypot(*c))))
    fit = WheelFit(side, lo, float(span), np.array(h), np.array(se[1:5]), math.degrees(sigma), len(r), revs,
                   float(se[0]), higher)
    # STM32 の hi は 0.8〜1.0（`control_params.h`）。ADC は 1.0 で飽和するので hi > 1 はあり得ず、外れると ARM が
    # 「範囲外で丸められた」で止まる。lo は読みの最小値に置いたので、惰行が下端まで届かないと真の lo より大きく、
    # lo + 幅 が 1.0 を超える（実機で 1.000655）。lo は角度の原点でしかない（ずらすと周期誤差の位相が回るだけ）ので、
    # hi = 1.0 になる lo へ下げて当てはめ直す
    if fit.hi > HI_MAX and _lo_retry < 2 and HI_MAX - fit.span >= 0.0:
        return fit_wheel(t, u, segs, side, lo=HI_MAX - fit.span, _lo_retry=_lo_retry + 1)
    return fit


def post_cal_ripple_deg(log: Log, side_i: int, segs: list[tuple[int, int]], t: np.ndarray,
                        cal: tuple[float, ...]) -> tuple[float, float]:
    """STM32 の校正後の走行距離 `odom_dist` に残る周期誤差の振幅 [°] と、その標準誤差 [°]。

    各区間で 校正後の角度 φ = odom/R（取付向きの符号）= 3 次式(t) + 周期誤差(θc) を線形に当てはめる。
    θc は記録したときの校正値で直した角度（STM32 と同じ式。周期誤差の角度の引数にだけ使う）。"""
    u = log.vs["front_angle"][:, side_i] / TWO_PI
    phi = log.vs["odom_dist"][:, side_i] / WHEEL_RADIUS_M * DIRS[side_i]
    lo, hi = cal[0], cal[1]
    rows, ys = [], []
    nseg = len(segs)
    npoly = _POLY_DEG + 1
    for k, (a, b) in enumerate(segs):
        tt = t[a:b]
        tau = (tt - 0.5 * (tt[0] + tt[-1])) / max(0.5 * (tt[-1] - tt[0]), 1e-3)
        th = cal_angle(u[a:b], lo, hi, cal[2:6])
        X = np.zeros((b - a, nseg * npoly + 4))
        for d in range(npoly):
            X[:, k * npoly + d] = tau ** d
        X[:, -4:] = np.column_stack([np.sin(th), np.cos(th), np.sin(2 * th), np.cos(2 * th)])
        rows.append(X)
        ys.append(phi[a:b])
    X = np.vstack(rows)
    y = np.concatenate(ys)
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ coef
    sigma = float(np.sqrt(np.sum(r ** 2) / max(1, len(y) - X.shape[1])))
    cov = np.linalg.pinv(X.T @ X) * sigma ** 2
    h = coef[-4:]
    se = float(np.sqrt(np.mean(np.diag(cov)[-4:])))
    return amplitude_deg(h), math.degrees(se) * 2.0


def _recorded_cal(log: Log, side: str) -> tuple[float, ...] | None:
    keys = [f"enc_{side}_{k}" for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c")]
    if not all(k in log.control_params for k in keys):
        return None
    return tuple(float(log.control_params[k]) for k in keys)


def _is_identity(cal: tuple[float, ...]) -> bool:
    return abs(cal[0]) < 1e-9 and abs(cal[1] - 1.0) < 1e-9 and all(abs(x) < 1e-9 for x in cal[2:])


def analyze(log: Log) -> Result:
    res = Result("校正 1: 前輪エンコーダ")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）。残った区間で解析する")
    segs3 = coast_segments(log)
    if len(segs3) < 2:
        res.fail(f"使える惰行の区間が {len(segs3)} 個しかない（`calib_encoder` の記録か、惰行が短すぎないか確かめる）")
        return res
    t = _stm_time(log)
    segs = [(a, b) for a, b, _ in segs3]
    speeds = np.array([v for _, _, v in segs3])
    for i, side in enumerate(SIDES):
        name = "左" if i == 0 else "右"
        u = log.vs["front_angle"][:, i] / TWO_PI
        fit = fit_wheel(t, u, segs, side)
        if fit.revs < _MIN_REVS_TOTAL:
            res.fail(f"{name}: 惰行の回転が {fit.revs:.1f} 回しかない（{_MIN_REVS_TOTAL:.0f} 回以上要る）")
            continue
        se_deg = float(np.degrees(np.sqrt(np.mean(fit.h_se ** 2))))
        res.info(f"{name}: 周期誤差 ±{fit.amp_deg:.2f}°（係数の標準誤差 {se_deg:.3f}°）、出力の範囲 "
                 f"{fit.lo:.4f}〜{fit.hi:.4f}（幅の標準誤差 {fit.span_se:.4f}）、惰行 {len(segs)} 区間・"
                 f"{fit.revs:.0f} 回転・{fit.n_points} 点")
        # `hi` も見る（外れると STM32 が丸め、`control_params_status` が mismatch になって ARM が止まる）。
        # 上側は惰行が下端まで届かず `hi = lo + 幅` が 1.0 を超えるのを当てはめ直しで 1.0 に収めてあるので、
        # 丸めの誤差ぶんだけ許す
        if (fit.lo > LO_MAX or np.max(np.abs(fit.h)) > H_MAX
                or not HI_MIN <= fit.hi <= HI_MAX + 1e-6):
            res.fail(f"{name}: 求めた値が STM32 の受け付ける範囲（lo ≤ {LO_MAX}、{HI_MIN} ≤ hi ≤ {HI_MAX}、"
                     f"|周期誤差の係数| ≤ {H_MAX}rad）を外れる。"
                     "適用すると ARM できない。エンコーダの取り付け・磁石を確かめる")
        if fit.resid_deg > _RESID_WARN_DEG:
            res.warn(f"{name}: 当てはめの残差 {fit.resid_deg:.2f}° が大きい（目安 {_RESID_WARN_DEG}° 以下）。"
                     "角度の関数でない誤差（振動・グリッチ）が多い")
        else:
            res.ok(f"{name}: 当てはめの残差 {fit.resid_deg:.2f}°")
        if fit.higher_deg > _HIGHER_WARN_DEG:
            res.warn(f"{name}: STM32 の式（2 次まで）で直せない 3・4 次の周期誤差が ±{fit.higher_deg:.2f}° 残る")
        # 速さの違う惰行で一致するか
        # 惰行に入った車速で遅い半分と速い半分に分ける
        med = float(np.median(speeds))
        slow = [s for s, v in zip(segs, speeds) if v <= med]
        fast = [s for s, v in zip(segs, speeds) if v > med]
        if len(slow) >= 2 and len(fast) >= 2 and np.ptp(speeds) > 0.2:
            fa, fb = fit_wheel(t, u, slow, side, lo=fit.lo), fit_wheel(t, u, fast, side, lo=fit.lo)
            d = float(np.degrees(np.max(np.abs(fa.h - fb.h))))
            se = float(np.degrees(np.sqrt(np.max(fa.h_se ** 2 + fb.h_se ** 2))))
            v_slow = float(np.mean([v for v in speeds if v <= med]))
            v_fast = float(np.mean([v for v in speeds if v > med]))
            if d > max(4.0 * se, 0.1):
                res.warn(f"{name}: {v_slow:.1f}m/s と {v_fast:.1f}m/s からの惰行で周期誤差の係数が {d:.2f}° 違う"
                         f"（標準誤差 {se:.3f}°）。角度の関数でない誤差が混ざっている")
            else:
                res.ok(f"{name}: {v_slow:.1f}m/s と {v_fast:.1f}m/s からの惰行で周期誤差が一致"
                       f"（係数の差 {d:.2f}°、標準誤差 {se:.3f}°）")
        else:
            res.info(f"{name}: 速さの違う惰行が足りず、速さによる違いは確かめていない")
        res.values.update(fit.values())
        th = np.linspace(0, TWO_PI, 181)
        res.plot("周期誤差 [°]（横軸: 角度 [°]）", name, np.degrees(th), np.degrees(ripple(fit.h, th)))
        cal = _recorded_cal(log, side)
        if cal is not None and not _is_identity(cal):
            post, post_se = post_cal_ripple_deg(log, i, segs, t, cal)
            if post <= max(_POST_RESID_OK_DEG, post_se, 0.15 * fit.amp_deg):
                res.ok(f"{name}: 記録したときの STM32 の校正で、走行距離に残る周期誤差は ±{post:.2f}°（校正が効いている）")
            else:
                res.warn(f"{name}: 記録したときの STM32 の校正の後も、走行距離に周期誤差が ±{post:.2f}° 残る"
                         f"（校正なしなら ±{fit.amp_deg:.2f}°）。この記録の値を入れて録り直す")
    for k in list(res.values):
        res.notes[k] = "生の読みから求めた値（記録時の校正に依らない）"
    return res
