"""校正 5: 車速制御（測る `calib_speed` → 決める → 確かめる `calib_speed_check`）。設計は `docs/calibration.md` §4.5。

## 車体のモデル

後輪 2 輪の合計トルク T（TELEMETRY の `torque_cmd` の和＝STM32 が MD へ送った値）→ 車速 v:

    T_eff' = (T(t − d) − T_eff)/τ           （MD のトルクの遅れ: むだ時間 d・1次遅れ τ）
    v'     = G·T_eff − r_± − c·v            （G: 利得、r: 転がり抵抗の減速度（前進・後退で別）、c: 粘性）

G には車輪の回転の慣性も入る（実効の質量）。名目は 1/(車輪半径×質量) = 16.7 (m/s²)/(N·m)。
r には後輪の回転の摩擦も入る（FF が打ち消すべき実効の値）。

**d・τ は MD の遅れだけではない。** トルクが変わってから後輪が滑って前後力が立ち上がるまで（タイヤの前後の
柔らかさ。数 ms、車速と荷重で変わる）も車体からは同じ遅れに見える（シムで、タイヤを硬くすると 3ms 消えた）。
車速の制御が実際に見るのはこの合計なので、PI の設計にはこれを使う。MD だけの遅れは校正 6（後輪の空転）で測る。

## 当てはめ（変数射影）

(d, τ, c) を決めると、走行距離 x(t) は (G, r_+, r_−, 各区間の初速・初期位置) について**線形**になる。
そこで (d, τ, c) だけを外側で探し、内側は線形最小二乗で解く（大域解が一意で、初期値に依らない）。
比べるのは走行距離（前輪の走行距離を舵角で車体の前後へ射影）そのもので、微分しないのでノイズに強い。
時刻は STM32 の時計（`t_stm_us`）。トルクが変わった時刻は TELEMETRY の間隔（10ms）の中のどこかで、
前後のサンプルの中点に置く（計画の周期と TELEMETRY の位相がずれていくので、平均すると偏らない）。

使う区間: トルク直接指令（`torque_mode`）の間で、制動から 50ms 以上離れ、|v| > 0.1m/s、TC が絞っていない所。

## 決める（PI と FF。STM32 の構造はそのまま）

    FF: speed_ff_nm_per_m_s2 = 1/G,   speed_ff_rr_nm = r_+/G,   launch_exit_torque_nm = r_+/G/2
    PI: 一巡伝達 L(s) = G·kp·(s + ωz)/s² · e^(−s·Td)/(1 + s·τ)，Td = d + 車速の推定の遅れ + 制御周期の半分

PI の零点 ωz を交差 ωc の `ZERO_RATIO` 分の 1 に置き、**位相余裕** PM(ωc) = atan(ωc/ωz) − ωc·Td − atan(ωc·τ)
が `PM_TARGET_DEG` になる ωc を解く（ただし `WC_MAX` まで）。|L(jωc)| = 1 から kp、ki = kp·ωz。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.signal import lfilter

from . import loop
from .encoder import _stm_time
from .log import Log
from .result import Result

__all__ = ["PlantFit", "fit_plant", "design_pi", "PiDesign", "analyze", "WHEEL_RADIUS_M", "MASS_KG"]

WHEEL_RADIUS_M = 0.030
MASS_KG = 2.0
G_NOMINAL = 1.0 / (WHEEL_RADIUS_M * MASS_KG)
#: ファームの値（`drive.h`）。1輪あたりのトルクの上限・制御周期
MAX_TORQUE_NM = 0.13
CONTROL_PERIOD_S = 0.0005
#: 車速（カルマンフィルタ）の遅れの見積もり [s]。走行距離を主にするので小さい（`docs/control_constants.md`）
SPEED_EST_LAG_S = 0.001
#: 設計の方針
PM_TARGET_DEG = 60.0
ZERO_RATIO = 4.0
#: 交差角周波数の上限 [rad/s]。上げるほど外乱（TC の絞り・勾配）からの戻りが速いが、車速の推定の雑音
#: （校正後の実機で数 mm/s）がトルクに kp 倍で乗り、TC の介入と干渉しやすい。追従は FF が受け持つので、
#: フィードバックは外乱を 0.1〜0.2 秒で戻せれば足りる
WC_MAX = 15.0
#: 使う区間の条件
_MIN_SPEED = 0.10
_BRAKE_GUARD_S = 0.05
_MIN_SEG_S = 0.25
_FINE_DT = 0.0005


@dataclass
class Segment:
    t: np.ndarray        # TELEMETRY の時刻 [s]（STM32 の時計）
    x: np.ndarray        # 車体の前後の走行距離 [m]（区間の頭を 0）
    tq: np.ndarray       # 後輪 2 輪の合計トルク [N·m]（各サンプルの値）
    t_prev: float        # 区間の頭の 1 つ前のサンプルの時刻（トルクの段の位置の基準）
    tq_hist: np.ndarray  # 区間の頭より前のトルク（遅れの初期値用、時刻と組）
    t_hist: np.ndarray
    direction: int       # +1 前進 / −1 後退


@dataclass
class PlantFit:
    G: float
    rr_fwd: float
    rr_bwd: float
    d: float
    tau: float
    c: float
    resid_mm: float
    n: int
    G_se: float
    rr_se: float
    segments: int

    @property
    def rr(self) -> float:
        """FF に使う転がり抵抗（前進。レースは前進で走る）。"""
        return self.rr_fwd


def body_distance(log: Log) -> np.ndarray:
    """前輪の走行距離（左右の平均）を舵角で車体の前後へ射影して積んだもの [m]（STM32 の車速と同じ射影）。"""
    od = log.vs["odom_dist"].mean(axis=1)
    inc = np.diff(od, prepend=od[0]) * np.cos(log.vs["steer_actual"])
    return np.cumsum(inc)


def plant_segments(log: Log, torque_mode_only: bool = True) -> list[Segment]:
    t = _stm_time(log)
    x = body_distance(log)
    tq = log.vs["torque_cmd"].sum(axis=1)
    v = np.gradient(x, t)
    tm = log.cmd_at("torque_mode", log.t) > 0.5
    tm_prev = log.cmd_at("torque_mode", log.t - _BRAKE_GUARD_S) > 0.5
    br = log.cmd_at("brake", log.t) > 0.5
    br_prev = log.cmd_at("brake", log.t - _BRAKE_GUARD_S) > 0.5
    tc_lim = log.vs["tc_limit_nm"].min(axis=1) < MAX_TORQUE_NM - 1e-3
    ok = (~br) & (~br_prev) & (np.abs(v) > _MIN_SPEED) & (~tc_lim)
    if torque_mode_only:
        ok &= tm & tm_prev
    out: list[Segment] = []
    i = 0
    n = len(t)
    while i < n:
        if not ok[i]:
            i += 1
            continue
        j = i
        while j < n and ok[j] and np.sign(v[j]) == np.sign(v[i]):
            j += 1
        if t[j - 1] - t[i] >= _MIN_SEG_S and i >= 1:
            h0 = max(0, i - 10)
            out.append(Segment(t=t[i:j], x=x[i:j] - x[i], tq=tq[i:j], t_prev=float(t[i - 1]),
                               tq_hist=tq[h0:i], t_hist=t[h0:i], direction=int(np.sign(v[i]))))
        i = j
    return out


def _fine_input(seg: Segment, d: float, tau: float) -> tuple[np.ndarray, np.ndarray]:
    """細かい刻みの時刻と、遅れを通したトルク T_eff。"""
    ts = np.concatenate([seg.t_hist, seg.t])
    vals = np.concatenate([seg.tq_hist, seg.tq])
    # 値が変わった時刻はサンプルの中点
    mids = np.concatenate([[ts[0] - 0.01], 0.5 * (ts[1:] + ts[:-1])])
    tf = np.arange(seg.t[0] - 0.06, seg.t[-1] + 1e-9, _FINE_DT)
    k = np.searchsorted(mids, tf - d, side="right") - 1
    u = vals[np.clip(k, 0, len(vals) - 1)]
    if tau > 1e-5:
        a = math.exp(-_FINE_DT / tau)
        u, _ = lfilter([1 - a], [1, -a], u, zi=[a * u[0]])
    return tf, u


def _basis(seg: Segment, d: float, tau: float, c: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """区間の各サンプルでの、x の成分 (∫∫T_eff 由来, ∫∫1 由来, ∫e^{-ct} 由来)。区間の頭で v, x の積分を 0 から。"""
    tf, u = _fine_input(seg, d, tau)
    m = tf >= seg.t[0] - 1e-12
    tf, u = tf[m], u[m]
    dt = _FINE_DT
    if c > 1e-6:
        a = math.exp(-c * dt)
        b = (1 - a) / c
    else:
        a, b = 1.0, dt
    # v[i] = a·v[i−1] + b·u[i−1]（v[0] = 0）
    vT = lfilter([0.0, b], [1, -a], u)
    v1 = lfilter([0.0, b], [1, -a], np.ones_like(u))
    v0 = np.exp(-c * (tf - tf[0]))
    xT = np.concatenate([[0.0], np.cumsum(vT[:-1]) * dt])
    x1 = np.concatenate([[0.0], np.cumsum(v1[:-1]) * dt])
    x0 = np.concatenate([[0.0], np.cumsum(v0[:-1]) * dt])
    return (np.interp(seg.t, tf, xT), np.interp(seg.t, tf, x1), np.interp(seg.t, tf, x0))


def _solve(segs: list[Segment], d: float, tau: float, c: float):
    """(d, τ, c) を固定した線形最小二乗。未知数の並び [G, r_+, r_−, (x0_k, v0_k)…]。"""
    rows, ys = [], []
    ns = len(segs)
    for k, sg in enumerate(segs):
        xT, x1, xv = _basis(sg, d, tau, c)
        X = np.zeros((len(sg.t), 3 + 2 * ns))
        X[:, 0] = xT
        X[:, 1 if sg.direction > 0 else 2] = -x1 * sg.direction
        X[:, 3 + 2 * k] = 1.0
        X[:, 4 + 2 * k] = xv
        rows.append(X)
        ys.append(sg.x)
    X = np.vstack(rows)
    y = np.concatenate(ys)
    # 片方の向きの区間が無ければ、その転がり抵抗の列は 0 → 落とす
    keep = np.any(np.abs(X) > 0, axis=0)
    coef = np.zeros(X.shape[1])
    sol, *_ = np.linalg.lstsq(X[:, keep], y, rcond=None)
    coef[keep] = sol
    r = y - X @ coef
    return coef, r, X[:, keep], keep


#: 遅れを探す格子 [s]。むだ時間は細かい刻み（`_FINE_DT`）の単位で効くので、連続の最適化では傾きが出ない
_D_GRID = np.arange(0.0, 0.0201, 0.0005)
_TAU_GRID = np.arange(0.0, 0.0151, 0.001)
_C_GRID = (0.0, 0.05, 0.1, 0.2, 0.4)


def fit_plant(segs: list[Segment], fit_viscous: bool = True) -> PlantFit:
    """(d, τ, c) は格子で総当たり（各点で内側の線形最小二乗を解く）。"""
    if not segs:
        raise ValueError("使える区間がない")
    best = None
    for c in (_C_GRID if fit_viscous else (0.0,)):
        for d in _D_GRID:
            for tau in _TAU_GRID:
                _, r, _, _ = _solve(segs, float(d), float(tau), c)
                cost = float(np.sum(r ** 2))
                if best is None or cost < best[0]:
                    best = (cost, float(d), float(tau), c)
    _, d, tau, c = best
    coef, r, Xk, keep = _solve(segs, d, tau, c)
    sigma = float(np.sqrt(np.sum(r ** 2) / max(1, len(r) - Xk.shape[1] - 3)))
    cov = np.linalg.pinv(Xk.T @ Xk) * sigma ** 2
    se_full = np.zeros(len(keep))
    se_full[keep] = np.sqrt(np.clip(np.diag(cov), 0, None))
    return PlantFit(G=float(coef[0]), rr_fwd=float(coef[1]), rr_bwd=float(coef[2]), d=d, tau=tau, c=c,
                    resid_mm=sigma * 1000, n=len(r), G_se=float(se_full[0]),
                    rr_se=float(max(se_full[1], se_full[2])), segments=len(segs))


@dataclass
class PiDesign:
    kp: float
    ki: float
    wc: float
    pm_deg: float
    td: float
    limited_by: str


def phase_margin_deg(wc: float, wz: float, td: float, tau: float) -> float:
    return math.degrees(math.atan2(wc, wz) - wc * td - math.atan(wc * tau))


def design_pi(G: float, d: float, tau: float, pm_target: float = PM_TARGET_DEG,
              ratio: float = ZERO_RATIO, wc_max: float = WC_MAX) -> PiDesign:
    """位相余裕から PI のゲインを決める（`loop.design`。TC・ABS と同じ式）。"""
    td = d + SPEED_EST_LAG_S + 0.5 * CONTROL_PERIOD_S
    dz = loop.design(loop.Plant(gain=G, pole=0.0, delay=td, tau=tau), pm_target, ratio, wc_max)
    return PiDesign(kp=dz.kp, ki=dz.ki, wc=dz.wc, pm_deg=dz.pm_deg, td=td,
                    limited_by="位相余裕" if dz.limited_by == "位相余裕" else "上限 WC_MAX")


def direction_check(segs: list[Segment]) -> tuple[float, float] | None:
    """前進だけ・後退だけで別々に当てはめた G（前進, 後退）。どちらかの区間が 2 つ未満なら None。

    G は車体の性質なので向きに依らないはず。違えば MD のトルクが向きで違う・片方の向きでタイヤが滑った・
    1 区間の中のトルクの段が足りず G と転がり抵抗が分かれていない、のどれか。"""
    a = [s for s in segs if s.direction > 0]
    b = [s for s in segs if s.direction < 0]
    if len(a) < 2 or len(b) < 2:
        return None
    return fit_plant(a, fit_viscous=False).G, fit_plant(b, fit_viscous=False).G


def analyze(log: Log) -> Result:
    res = Result("校正 5: 車速制御（車体の同定と PI・FF の設計）")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）。残った区間で解析する")
    segs = plant_segments(log)
    if len(segs) < 4:
        res.fail(f"使えるトルク直接指令の区間が {len(segs)} 個しかない（`calib_speed` の記録か確かめる）")
        return res
    fit = fit_plant(segs)
    fit0 = fit_plant(segs, fit_viscous=False)
    # 粘性 c は、入れて残差がはっきり減るときだけ採る（入れなくても説明できるなら単純な方）
    use_c = fit.resid_mm < 0.9 * fit0.resid_mm and fit.c > 0.02
    f = fit if use_c else fit0
    res.info(f"車体の利得 G = {f.G:.2f} ±{2 * f.G_se:.2f} (m/s²)/(N·m)（名目 1/(r·m) = {G_NOMINAL:.1f}。"
             f"車輪の回転の慣性・MD のトルク定数の誤差を含む実効の値）")
    res.info(f"転がり抵抗 前進 {f.rr_fwd:.3f}・後退 {f.rr_bwd:.3f} m/s²（±{2 * f.rr_se:.3f}）"
             + (f"、粘性 c = {f.c:.3f} 1/s" if use_c else "（粘性の項は入れても残差が減らないので 0）"))
    res.info(f"トルク→加速度の遅れ: むだ時間 {f.d * 1000:.1f}ms＋1次遅れ {f.tau * 1000:.1f}ms（MD のトルクの遅れと、"
             "後輪が滑って前後力が立ち上がるまでの遅れの合計。分け方は 10ms の TELEMETRY からは確かでない）")
    res.info(f"当てはめの残差 {f.resid_mm:.2f}mm（走行距離。{f.segments} 区間・{f.n} 点）")
    if not 0.75 * G_NOMINAL < f.G < 1.35 * G_NOMINAL:
        res.warn(f"G が名目の {f.G / G_NOMINAL:.2f} 倍。質量・車輪半径・MD のトルクの設定を確かめる")
    if not 0.03 < f.rr_fwd < 1.0:
        res.warn(f"転がり抵抗 {f.rr_fwd:.3f}m/s² がありそうな範囲（0.03〜1.0）の外")
    if f.resid_mm > 3.0:
        res.warn(f"残差 {f.resid_mm:.1f}mm が大きい（目安 3mm 以下）。タイヤが滑った・モデルに無い力がある")
    else:
        res.ok(f"残差 {f.resid_mm:.2f}mm（モデルで説明できている）")
    dc = direction_check(segs)
    if dc is not None:
        diff = abs(dc[0] - dc[1]) / (0.5 * (dc[0] + dc[1]))
        if diff > 0.05:
            res.warn(f"前進だけ・後退だけで当てはめた G が {dc[0]:.2f}・{dc[1]:.2f} で {diff * 100:.1f}% 違う（目安 5% 以内）。"
                     "MD のトルクが向きで違うか、区間の中のトルクの段が足りない")
        else:
            res.ok(f"前進だけ・後退だけで当てはめた G が一致（{dc[0]:.2f}・{dc[1]:.2f}、差 {diff * 100:.1f}%）")
    else:
        res.info("前進・後退の片方の区間が足りず、向きによる違いは確かめていない")
    pi = design_pi(f.G, f.d, f.tau)
    res.info(f"PI の設計: 交差 ωc = {pi.wc:.1f}rad/s（{pi.limited_by}で決まる）・零点 {pi.wc / ZERO_RATIO:.1f}rad/s・"
             f"位相余裕 {pi.pm_deg:.0f}°（遅れ {pi.td * 1000:.1f}ms・{f.tau * 1000:.1f}ms で計算）")
    rec_kp = log.control_params.get("speed_kp")
    rec_ki = log.control_params.get("speed_ki")
    if rec_kp and rec_ki:
        wz = rec_ki / rec_kp
        # 記録したときのゲインの交差を数値で求める
        w = np.logspace(-1, 3, 4000)
        mag = f.G * rec_kp * np.sqrt(w * w + wz * wz) / (w * w) / np.sqrt(1 + (w * f.tau) ** 2)
        wc_rec = float(w[np.argmin(np.abs(mag - 1))])
        res.info(f"記録したときのゲイン kp {rec_kp:.3f}・ki {rec_ki:.3f} は交差 {wc_rec:.1f}rad/s・位相余裕 "
                 f"{phase_margin_deg(wc_rec, wz, pi.td, f.tau):.0f}°")
    res.values.update({
        "dynamics.speed_plant_gain": round(f.G, 4),
        "dynamics.rolling_resistance": round(f.rr_fwd, 4),
        "dynamics.speed_kp": round(pi.kp, 5),
        "dynamics.speed_ki": round(pi.ki, 5),
        "control.speed_ff_nm_per_m_s2": round(1.0 / f.G, 6),
        "control.speed_ff_rr_nm": round(f.rr_fwd / f.G, 6),
        "control.launch_exit_torque_nm": round(f.rr_fwd / f.G / 2.0, 6),
    })
    res.notes.update({
        "dynamics.speed_kp": f"位相余裕 {pi.pm_deg:.0f}°・交差 {pi.wc:.1f}rad/s の設計",
        "dynamics.speed_ki": f"零点 = 交差/{ZERO_RATIO:.0f}",
        "control.speed_ff_nm_per_m_s2": "= 1/G（加速度 1m/s² を出す合計トルク）",
        "control.speed_ff_rr_nm": "= 転がり抵抗/G",
        "control.launch_exit_torque_nm": "= 転がり抵抗/G/2（1輪）",
    })
    for k, sg in enumerate(segs[:12]):
        res.plot("走行距離 [m]（横軸: 時刻 [s]）", f"区間{k}", sg.t - sg.t[0], sg.x)
    return res
