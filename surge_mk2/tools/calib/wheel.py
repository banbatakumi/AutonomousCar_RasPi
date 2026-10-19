"""校正 6: 後輪の空転（`raspi/auto/calib_wheel.py` の記録 → `[control.plant]`）。設計は `docs/calibration.md` §4.6。

## モデル（1輪ごと。周速 w = 車輪の角速度 × 半径）

    T_eff' = (T(t − d) − T_eff)/τ
    w'     = (r/J)·T_eff − (r/J)·T_f·sgn − (b/J)·w

T は TELEMETRY の `torque_cmd`、w は MD が報告した後輪の周速（`wheel_speed[2:4]`）。d・τ は **MD のトルクの遅れと、
MD が報告する速度の遅れの合計**（この記録からは分けられない。TC の一巡にはどちらも入るので、合計でよい）。

(d, τ, b/J) を格子で探し、各点で (r/J, r·T_f/J（向きごと）, 各区間の初速) を線形最小二乗で解く（`speed.py` と同じ
変数射影）。1 区間 = 「回す」＋続く「惰行」（惰行の減速が摩擦÷J を、回す加速が (T − 摩擦)÷J を決める）。
止まりかけ（|w| < 0.05m/s）は静止摩擦が混ざるので使わない。

## 確かめ

- 左右の慣性が 15% 以内でそろう（同じ部品）
- 区間の半分ずつで当てはめた慣性が 10% 以内でそろう
- 残差が MD の速度の雑音程度（25mm/s 以下。シムの雑音で 11mm/s）
"""

from __future__ import annotations

import math

import numpy as np
from scipy.signal import lfilter

from raspi.auto._calib_common import phase_parts

from .encoder import _stm_time
from .log import Log
from .result import Result
from .speed import _FINE_DT, Segment, _fine_input

__all__ = ["analyze", "WheelFit", "fit_wheels"]

R = 0.030
_MIN_RIM = 0.05
_D_GRID = np.arange(0.0, 0.0151, 0.0005)
_TAU_GRID = np.arange(0.0, 0.0121, 0.001)
_BETA_GRID = (0.0, 0.5, 1.0, 2.0, 4.0)


class WheelFit:
    def __init__(self, J, friction_fwd, friction_bwd, damping, d, tau, resid, n, segs, dirs=(1, -1)):
        self.dirs = dirs
        self.J, self.friction_fwd, self.friction_bwd = J, friction_fwd, friction_bwd
        self.damping, self.d, self.tau, self.resid, self.n, self.segs = damping, d, tau, resid, n, segs

    @property
    def friction(self) -> float:
        """区間のあった向きの摩擦の平均。"""
        vals = [f for f, d in ((self.friction_fwd, 1), (self.friction_bwd, -1)) if d in self.dirs]
        return float(np.mean(vals)) if vals else 0.0


def wheel_segments(log: Log, side: int) -> list[Segment]:
    """1輪ぶんの「回す＋惰行」の区間（`Segment.x` に周速を入れる）。"""
    t = _stm_time(log)
    w = log.vs["wheel_speed"][:, 2 + side]
    tq = log.vs["torque_cmd"][:, side]
    names = log.phase_of_samples()
    if not any((phase_parts(n) or ("",))[0] == "spin" for n in names):
        names = _legacy_phases(log)
    out = []
    i = 0
    n = len(t)
    while i < n:
        p = phase_parts(names[i])
        if p is None or p[0] != "spin":
            i += 1
            continue
        cyc, direction = p[1], p[2]
        j = i
        while j < n:
            q = phase_parts(names[j])
            if q is None or q[1] != cyc or q[0] not in ("spin", "coast"):
                break
            j += 1
        idx = np.arange(i, j)
        # 回り始めてから（静止摩擦の区間は捨てる）、惰行が遅くなるまで。制動に入ったサンプルは捨てる
        moving = np.abs(w[idx]) >= _MIN_RIM
        drive_or_zero = (np.sign(tq[idx]) == direction) | (np.abs(tq[idx]) < 1e-6)
        ok = moving & drive_or_zero
        if ok.sum() >= 15:
            first = idx[np.argmax(ok)]
            last = idx[len(ok) - 1 - np.argmax(ok[::-1])]
            sel = np.arange(first, last + 1)
            h0 = max(0, first - 10)
            out.append(Segment(t=t[sel], x=w[sel], tq=tq[sel], t_prev=float(t[first - 1]), tq_hist=tq[h0:first],
                               t_hist=t[h0:first], direction=int(direction)))
        i = j
    return out


def _stuck_spins(log: Log, side: int) -> list[float]:
    """「回す」の区間で周速が `_MIN_RIM` の 2 倍に届かなかった回の、planner が指令したトルク [N·m]（1輪）。"""
    w = np.abs(log.vs["wheel_speed"][:, 2 + side])
    out = []
    for name, i0, i1 in log.segments(lambda p: (phase_parts(p) or ("",))[0] == "spin"):
        cmd = log.cmd_at("target_torque", log.t[i0:i1])
        cmd = cmd[np.abs(cmd) > 1e-4]
        if len(cmd) and np.max(w[i0:i1]) < _MIN_RIM * 2:
            out.append(float(np.median(cmd)))
    return out


def _legacy_phases(log: Log) -> list[str]:
    """区間の名前を持たない旧 planner（`sysid_wheel`、理由の文「回す 0.008N·m 1/6」）の記録の区間名。"""
    import re
    names = []
    idx = np.searchsorted(log.phase_t, log.t, side="right") - 1
    for i in idx:
        r = log.reasons[i] if i >= 0 else ""
        m = re.match(r"回す ([-+0-9.]+)N·m (\d+)/", r)
        names.append(f"spin/{m.group(2)}/{'+' if float(m.group(1)) >= 0 else '-'}" if m else "")
    return names


def _vbasis(seg: Segment, d: float, tau: float, beta: float):
    tf, u = _fine_input(seg, d, tau)
    m = tf >= seg.t[0] - 1e-12
    tf, u = tf[m], u[m]
    dt = _FINE_DT
    a = math.exp(-beta * dt)
    b = (1 - a) / beta if beta > 1e-9 else dt
    vT = lfilter([0.0, b], [1, -a], u)
    v1 = lfilter([0.0, b], [1, -a], np.ones_like(u))
    v0 = np.exp(-beta * (tf - tf[0]))
    return np.interp(seg.t, tf, vT), np.interp(seg.t, tf, v1), np.interp(seg.t, tf, v0)


def _solve(segs, d, tau, beta):
    ns = len(segs)
    rows, ys = [], []
    for k, sg in enumerate(segs):
        vT, v1, v0 = _vbasis(sg, d, tau, beta)
        X = np.zeros((len(sg.t), 3 + ns))
        X[:, 0] = vT                                   # r/J
        X[:, 1 if sg.direction > 0 else 2] = -v1 * sg.direction   # r·Tf/J（向きごと）
        X[:, 3 + k] = v0
        rows.append(X)
        ys.append(sg.x)
    X = np.vstack(rows)
    y = np.concatenate(ys)
    keep = np.any(np.abs(X) > 0, axis=0)
    coef = np.zeros(X.shape[1])
    sol, *_ = np.linalg.lstsq(X[:, keep], y, rcond=None)
    coef[keep] = sol
    return coef, y - X @ coef


def fit_wheels(segs: list[Segment]) -> WheelFit:
    best = None
    for beta in _BETA_GRID:
        for d in _D_GRID:
            for tau in _TAU_GRID:
                _, r = _solve(segs, float(d), float(tau), beta)
                c = float(np.sum(r ** 2))
                if best is None or c < best[0]:
                    best = (c, float(d), float(tau), beta)
    _, d, tau, beta = best
    coef, r = _solve(segs, d, tau, beta)
    a = coef[0]
    J = R / a
    return WheelFit(J=J, friction_fwd=coef[1] / a, friction_bwd=coef[2] / a, damping=beta * J,
                    d=d, tau=tau, resid=float(np.sqrt(np.mean(r ** 2))), n=len(r), segs=len(segs),
                    dirs=tuple(sorted({sg.direction for sg in segs})))


def analyze(log: Log) -> Result:
    res = Result("校正 6: 後輪の空転（車輪の慣性・摩擦・トルクの遅れ）")
    if log.aborted:
        res.fail(f"試験は中止された（{log.aborted}）")
        return res
    fits = []
    for side, name in ((0, "左"), (1, "右")):
        segs = wheel_segments(log, side)
        stuck = _stuck_spins(log, side)
        if stuck:
            res.warn(f"{name}: トルクを掛けても回り出さなかった回が {len(stuck)} 回（{'・'.join(f'{t:+.3f}' for t in stuck)} N·m）。"
                     "静止摩擦・コギングより小さいトルク（その回は使わない）")
        if len(segs) < 4:
            res.fail(f"{name}: 使える区間が {len(segs)} 個しかない（回り出さなかった回が {len(stuck)} 回。後輪が浮いて"
                     "いるか・何かに触れていないか、回すトルクが静止摩擦を超えているか確かめる）")
            return res
        f = fit_wheels(segs)
        fits.append(f)
        # 半分ずつ
        fa, fb = fit_wheels(segs[0::2]), fit_wheels(segs[1::2])
        half = abs(fa.J - fb.J) / f.J
        fr_txt = "・".join(f"{lab} {v * 1000:.2f}" for lab, v, dd in (("前進", f.friction_fwd, 1), ("後退", f.friction_bwd, -1))
                           if dd in f.dirs)
        res.info(f"{name}: 慣性 {f.J * 1e6:.1f}e-6 kg·m²、摩擦 {fr_txt} mN·m、粘性 {f.damping * 1e6:.2f}e-6 N·m·s、遅れ {f.d * 1000:.1f}＋{f.tau * 1000:.1f}ms"
                 f"（残差 {f.resid * 1000:.1f}mm/s、{f.segs} 区間）")
        if half > 0.10:
            res.warn(f"{name}: 区間の半分ずつで当てはめた慣性が {half * 100:.0f}% 違う（目安 10% 以内）")
        else:
            res.ok(f"{name}: 区間の半分ずつで当てはめた慣性の差 {half * 100:.1f}%")
        if f.resid > 0.025:
            res.warn(f"{name}: 残差 {f.resid * 1000:.1f}mm/s が大きい（目安 25mm/s 以下。後輪が何かに触れている？）")
        for k, sg in enumerate(segs[:6]):
            res.plot(f"{name}後輪の周速 [m/s]（横軸: 時刻 [s]）", f"区間{k}", sg.t - sg.t[0], sg.x)
    if any(len(f.dirs) < 2 for f in fits):
        res.warn("前進・後退の片方の向きの区間しかない（摩擦の向きによる違いを確かめていない）")
    lag_l, lag_r = fits[0].d + fits[0].tau, fits[1].d + fits[1].tau
    if abs(lag_l - lag_r) > 0.005:
        res.warn(f"左右の遅れが {lag_l * 1000:.0f}ms・{lag_r * 1000:.0f}ms と食い違う（同じ MD のはず。区間が足りず決まっていない）")
    jl, jr = fits[0].J, fits[1].J
    if abs(jl - jr) / (0.5 * (jl + jr)) > 0.15:
        res.warn(f"左右の慣性が {abs(jl - jr) / (0.5 * (jl + jr)) * 100:.0f}% 違う（目安 15% 以内。同じ部品のはず）")
    else:
        res.ok(f"左右の慣性がそろう（差 {abs(jl - jr) / (0.5 * (jl + jr)) * 100:.1f}%）")
    J = 0.5 * (jl + jr)
    fr = 0.5 * (fits[0].friction + fits[1].friction)
    damp = 0.5 * (fits[0].damping + fits[1].damping)
    d = max(f.d for f in fits)
    tau = max(f.tau for f in fits)
    res.values.update({"control.plant.wheel_inertia_kgm2": float(f"{J:.4g}"),
                       "control.plant.wheel_friction_nm": round(fr, 6),
                       "control.plant.wheel_damping_nms": float(f"{damp:.4g}"),
                       "control.plant.md_delay_s": round(d, 5), "control.plant.md_tau_s": round(tau, 5)})
    res.notes.update({"control.plant.wheel_inertia_kgm2": "左右の平均",
                      "control.plant.md_delay_s": "MD のトルクの遅れ＋MD の速度の遅れ（左右の大きい方）"})
    return res
