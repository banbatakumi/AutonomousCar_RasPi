"""校正 7: タイヤ（`raspi/auto/calib_tyre.py` の記録）→ TC・ABS の目標スリップ率とゲイン。設計は `docs/calibration.md` §4.7。

## 1. 曲線を測る

前進の片道ごとに TC・ABS の目標スリップ率を振って、全開の発進・最大の制動をした記録。各区間（車速 0.6m/s 以上、
`_CHUNK_S` ずつに区切る）で

- スリップ率: STM32 が測って制御に使っている値（TELEMETRY の `tc_slip`）。駆動は左右の平均（TC は各輪）、制動は
  ロックしかけている側（ABS は左右共通で、その側に合わせる）
- 前後力の合計: 車体の運動から F = m_b·(a + r_b)。a は走行距離を 2 次式で当てはめた加速度、m_b は車体の実効の質量
  （校正 5 の G から後輪の回転の慣性を除いたもの）、r_b は車体の転がり抵抗（校正 5 の値から後輪の摩擦ぶんを除く）

**STM32 が測るスリップ率は、加減速中に真の値より小さく読める**: 後輪の周速は MD の報告とローパスで遅れ L を持ち、
加速度 a の間は周速が a·L だけ古い。スリップ率にすると a·L/v（低速ほど大きい。シムで 0.75m/s・2.5m/s² のとき約 0.02）。
同じ目標スリップ率でも低速ほど真のスリップ率が大きく、力も大きく出る。そこで曲線は真のスリップ率

    λ = λ_STM32 + |a|·L/v,   F = D·sin(C·atan(B·λ))

で当てはめ、L も未知数にする（速さの違う点があるので決まる）。駆動・制動で別々に当てはめる。

## 2. 目標スリップ率

試したスリップ率の範囲で曲線の最大 F_max を取り、**F が F_max の 95% に届く最小の真のスリップ率** λ95。STM32 の目標は
STM32 の測り方の値なので、`V_TARGET` の車速・その力の加速度での読みの遅れを引いて λ95 − a·L/v にする（高速ほど
真の値に近づき、低速では少し山の先まで滑らせる）。山の手前に置けば、
外乱で少し超えても力が落ちず戻れる（山を越えると力が落ち、横のグリップも失う）。山が試した範囲に無ければ★
（目標は範囲の端の 95%。もっと高いスリップ率を試さないと山が分からない）。

## 3. ゲイン

1輪のスリップ速度 u = 周速 − 基準速度 の動きを、目標の点 λ0 で線形化する:

    u' = (r/J)·T − a·u,   a = r²·k/(J·v)       （k = dF_w/dλ、1輪の曲線の傾き）

TC は u の偏差からトルクの上限を作る PI。植物 (r/J)·e^(−s·d)/((s + a)(1 + s·τ)) に対して、位相余裕 `PM_TARGET_DEG`
から交差を決める（`loop.design`。車速の PI と同じ式）。d・τ は校正 6 の遅れ（MD のトルク＋MD の速度の報告＋ファームの
ローパスの合計。TC の一巡にちょうど入る量）。a は車速で変わるので `V_DESIGN` の車速で決め（理由はその注記）、
ほかの車速での位相余裕も出す。

★ 極 a を PI の零点で打ち消して積分器として設計すると（最初に試した形）、kp が 1/6 になり、シムで発進が 3 割遅くなった
（空転の後にトルクの上限を戻すのが遅い）。目標の点ではタイヤの傾きが大きく、a ≈ 200rad/s は交差より上にある。

## 4. 確かめ（シム）

決めた値と今の値を、ホストでコンパイルしたファーム＋測った車輪・タイヤのモデルで「全開の発進」「最大の制動」に
かけて比べる（進んだ距離・止まるまでの距離・スリップ率のばらつき・トルクの暴れ）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import least_squares

from raspi.auto._calib_common import phase_parts

from . import loop, toml_io
from .encoder import _stm_time
from .log import Log
from .result import Result
from .speed import body_distance

__all__ = ["CurvePoint", "Curve", "curve_points", "fit_curve", "pick_target", "design_gains", "analyze", "Consts"]

R = 0.030
GRAVITY = 9.80665
#: 位相余裕の狙い [°]。60° で設計した TC のゲインは、前の版の数値最適化の値（位相余裕 58°）とほぼ同じで、
#: シムの 5 種類の車で発進の性能も同じだった（ABS は旧値が 29° と攻めていて、60° は止まる距離が同じでトルクの暴れが少ない）
PM_TARGET_DEG = 60.0
ZERO_RATIO = 4.0
#: ゲインを決める車速 [m/s]（TC・ABS）。スリップの極 a は車速に反比例し、高速ほど一巡が積分器に近く位相余裕が厳しい。
#: 高い車速で決めるほどゲインは低く、振動しにくいが、空転・ロックの始まりから戻るのが遅い。シム（2026-10-19）で:
#: - TC: 1m/s で決めたゲインに比べ、2m/s では発進が 2% 速くトルクの暴れが 1/2〜1/5（山のあるタイヤ）。3m/s は始まりの
#:   空転が 80ms と長くなる
#: - ABS: 3m/s で決めると制動の始まりのロックから戻るのに 160ms かかり、平均のスリップ率が目標の 2 倍になった
#:   （校正 8 が不合格にする）。1m/s では 50ms
V_DESIGN = {"drive": 2.0, "brake": 1.0}
#: 目標スリップ率の読みの遅れを戻すときの車速 [m/s]（発進から加速していく途中の代表）
V_TARGET = 1.0
_CHUNK_S = 0.12
_MIN_V = 0.6
#: 横の上限の見積もりに使う、後軸の静止荷重の割合と重心高/ホイールベース（実測していない机上の値）
REAR_SHARE = 0.59
HEIGHT_RATIO = 0.2


@dataclass
class Consts:
    """解析に使う、ほかの校正で決まった値。"""

    G: float = 16.7                 #: 校正 5 の利得 [(m/s²)/(N·m)]
    rr: float = 0.22                #: 校正 5 の転がり抵抗 [m/s²]
    J: float = 2.4e-5               #: 校正 6 の後輪の慣性 [kg·m²]
    Tf: float = 0.0025              #: 校正 6 の後輪の摩擦 [N·m]
    d: float = 0.006                #: 校正 6 の遅れ（むだ時間）[s]
    tau: float = 0.005              #: 同 1次遅れ [s]
    mass: float = 2.0

    @property
    def m_body(self) -> float:
        return 1.0 / (R * self.G) - 2.0 * self.J / R ** 2

    @property
    def rr_body(self) -> float:
        return max(0.0, self.rr - 2.0 * self.Tf * self.G)

    @classmethod
    def from_toml(cls, path=toml_io.DEFAULT_TOML) -> "Consts":
        f = toml_io.read_flat(path)
        c = cls()
        return replace(c, G=f.get("dynamics.speed_plant_gain", c.G), rr=f.get("dynamics.rolling_resistance", c.rr),
                       J=f.get("control.plant.wheel_inertia_kgm2", c.J), Tf=f.get("control.plant.wheel_friction_nm", c.Tf),
                       d=f.get("control.plant.md_delay_s", c.d), tau=f.get("control.plant.md_tau_s", c.tau),
                       mass=f.get("mass", c.mass))


@dataclass
class CurvePoint:
    kind: str           #: "drive" / "brake"
    target: float       #: その区間の目標スリップ率（上書きした値。わからなければ nan）
    slip: float         #: 制御されたスリップ率（STM32 の測り方、正の大きさ）
    force: float        #: 前後力の合計 [N]（正の大きさ）
    v: float
    slip_l: float
    slip_r: float
    accel: float = 0.0  #: 車体の加速度の大きさ [m/s²]


@dataclass
class Curve:
    D: float
    B: float
    C: float
    resid: float
    n: int
    #: STM32 のスリップ率の読みの遅れ [s]
    L: float = 0.0

    def measured_to_true(self, lam_fw: float, accel: float, v: float) -> float:
        return lam_fw + abs(accel) * self.L / max(v, 0.3)

    def force(self, lam) -> np.ndarray:
        lam = np.asarray(lam, dtype=float)
        return self.D * np.sin(self.C * np.arctan(self.B * lam))

    def slope(self, lam: float) -> float:
        x = self.B * lam
        return self.D * self.C * self.B * math.cos(self.C * math.atan(x)) / (1 + x * x)


def _override_at(log: Log, t: float, key: str) -> float:
    i = int(np.searchsorted(log.cmd_t, t, side="right")) - 1
    if i < 0:
        return float("nan")
    return float(log.cmd_overrides[i].get(key, float("nan")))


def curve_points(log: Log, c: Consts) -> list[CurvePoint]:
    t = _stm_time(log)
    x = body_distance(log)
    v = np.gradient(x, t)
    slip = log.vs["tc_slip"]
    names = log.phase_of_samples()
    out: list[CurvePoint] = []
    for kind in ("drive", "brake"):
        for _, i0, i1 in log.segments(lambda p, k=kind: (phase_parts(p) or ("",))[0] == k):
            idx = np.arange(i0, i1)
            idx = idx[np.abs(v[idx]) >= _MIN_V]
            # 指令が効き始めた後（駆動はトルクが出ている・制動は負のトルク）
            tq = log.vs["torque_cmd"][idx].mean(axis=1)
            idx = idx[(tq > 1e-3) if kind == "drive" else (tq < -1e-3)]
            if len(idx) < 8:
                continue
            target = _override_at(log, log.t[idx[0]], "tc_slip_target" if kind == "drive" else "abs_slip_target")
            n_chunk = max(1, int((t[idx[-1]] - t[idx[0]]) / _CHUNK_S))
            for part in np.array_split(idx, n_chunk):
                if len(part) < 6:
                    continue
                tt = t[part] - t[part[0]]
                cf = np.polyfit(tt, x[part], 2)
                a = 2 * cf[0]
                vm = float(np.mean(v[part]))
                sgn = 1.0 if vm >= 0 else -1.0
                F = c.m_body * (a + c.rr_body * sgn) * sgn
                sl, sr = float(np.mean(slip[part, 0])), float(np.mean(slip[part, 1]))
                lam = 0.5 * (sl + sr) if kind == "drive" else -min(sl, sr)
                if kind == "brake":
                    F = -F
                out.append(CurvePoint(kind, target, lam, F, abs(vm), sl, sr, abs(a)))
    del names
    return out


def fit_curve(points: list[CurvePoint], fit_lag: bool = True) -> Curve:
    lam = np.array([0.0] + [p.slip for p in points])
    F = np.array([0.0] + [p.force for p in points])
    acc_v = np.array([0.0] + [p.accel / max(p.v, 0.3) for p in points])
    w = np.ones_like(lam)
    w[0] = 3.0                                   # 原点（転がっているだけなら力 0）は確か

    def resid(q):
        D, B, C = q[:3]
        L = q[3] if fit_lag else 0.0
        return w * (D * np.sin(C * np.arctan(B * (lam + acc_v * L))) - F)

    best = None
    for B0 in (5.0, 15.0, 40.0):
        for C0 in (1.0, 1.5):
            x0 = [max(F.max(), 0.5), B0, C0] + ([0.005] if fit_lag else [])
            lb = [0.05, 0.5, 0.3] + ([0.0] if fit_lag else [])
            ub = [50.0, 300.0, 2.0] + ([0.03] if fit_lag else [])
            sol = least_squares(resid, x0, bounds=(lb, ub))
            if best is None or sol.cost < best.cost:
                best = sol
    D, B, C = best.x[:3]
    r = resid(best.x)[1:]
    return Curve(float(D), float(B), float(C), float(np.sqrt(np.mean(r ** 2))), len(points),
                 float(best.x[3]) if fit_lag else 0.0)


def pick_target(curve: Curve, lam_max: float, frac: float = 0.95) -> tuple[float, bool]:
    """(目標スリップ率, 山が試した範囲の中にあったか)。"""
    lam = np.linspace(0, lam_max, 2001)
    Fv = curve.force(lam)
    k_peak = int(np.argmax(Fv))
    has_peak = k_peak < len(lam) - 1
    target = float(lam[np.argmax(Fv >= frac * Fv[k_peak])])
    return target, has_peak


def slip_plant(slope_wheel: float, c: Consts, v: float = 1.0) -> loop.Plant:
    """目標の点で線形化したスリップ速度の植物（モジュールの docstring §3）。"""
    a = R * R * max(slope_wheel, 0.0) / (c.J * max(v, 0.3))
    return loop.Plant(gain=R / c.J, pole=a, delay=c.d, tau=c.tau)


def design_gains(slope_wheel: float, c: Consts, pm: float = PM_TARGET_DEG, v: float = 1.0) -> loop.Design:
    """目標の点の植物に対して、位相余裕から PI を決める（`loop.design`）。"""
    return loop.design(slip_plant(slope_wheel, c, v), pm, ratio=ZERO_RATIO)


def analyze(log: Log, consts: Consts | None = None) -> Result:
    c = consts or Consts.from_toml()
    res = Result("校正 7: タイヤ（スリップ率→前後力）と TC・ABS の設計")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）。残った区間で解析する")
    pts = curve_points(log, c)
    res.info(f"使った値: 車体の実効の質量 {c.m_body:.3f}kg・転がり抵抗 {c.rr_body:.3f}m/s²（校正 5）、後輪の慣性 "
             f"{c.J * 1e6:.1f}e-6kg·m²・遅れ {c.d * 1000:.1f}＋{c.tau * 1000:.1f}ms（校正 6）")
    if not 0.8 * c.mass < c.m_body < 1.2 * c.mass:
        res.warn(f"車体の実効の質量 {c.m_body:.2f}kg が車両の質量 {c.mass:.2f}kg と 2 割以上違う（toml の speed_plant_gain が"
                 "古い？・MD のトルク定数が実際と違う＝指令ほどトルクが出ていない）。前後力はトルク指令の単位で出る"
                 "（TC・ABS の設計はこの単位で閉じるので影響しない。μ はこの比で狂う）")
    for kind, name, key in (("drive", "駆動（TC）", "tc"), ("brake", "制動（ABS）", "abs")):
        p = [q for q in pts if q.kind == kind]
        levels = sorted({round(q.target, 3) for q in p if not math.isnan(q.target)})
        if len(p) < 6 or len(levels) < 4:
            res.fail(f"{name}: 曲線の点が {len(p)} 個・目標の段が {len(levels)} 段しかない（`calib_tyre` の記録か確かめる）")
            continue
        # TC・ABS が目標のスリップ率に付いてきたか（付いてこなければ、上書きが入っていない・路面のグリップが強すぎて
        # 0.13N·m でも滑らない。どちらでも曲線は測れていない）
        held = [np.mean([q.slip for q in p if round(q.target, 3) == lv]) for lv in levels]
        if max(held) - min(held) < 0.05:
            res.fail(f"{name}: 目標スリップ率を振っても実際のスリップ率が {min(held):.3f}〜{max(held):.3f} しか動かない"
                     "（TC・ABS の目標の上書きが STM32 に入っていない？0.13N·m でも滑らない路面？）。曲線を測れていない")
            continue
        cv = fit_curve(p)
        lam_max = max(cv.measured_to_true(q.slip, q.accel, q.v) for q in p)
        target_true, has_peak = pick_target(cv, lam_max)
        a_t = float(cv.force(target_true)) / c.m_body
        target = max(0.02, target_true - a_t * cv.L / V_TARGET)
        res.info(f"{name}: F = {cv.D:.2f}·sin({cv.C:.2f}·atan({cv.B:.1f}·λ)) [N]（左右の合計、λ は真のスリップ率、{cv.n} 点、"
                 f"残差 {cv.resid:.2f}N）。STM32 のスリップ率の読みの遅れ {cv.L * 1000:.1f}ms")
        tab = "・".join(f"{lv:.2f}→{np.mean([q.slip for q in p if round(q.target, 3) == lv]):.3f}" for lv in levels)
        res.info(f"{name}: 目標スリップ率 → 実際に保てた値 {tab}")
        if cv.resid > 0.1 * cv.D:
            res.warn(f"{name}: 曲線の残差 {cv.resid:.2f}N が上限の 1 割を超える（路面のむら・片輪だけの滑り？）")
        if not has_peak:
            res.warn(f"{name}: 試したスリップ率（真の値で〜{lam_max:.2f}）の中に力の山が無い。目標は範囲の端の 95% の点"
                     f"（真の値 {target_true:.3f}）。もっと高いスリップ率を試すと上がるかもしれない")
        else:
            grid = np.linspace(0, lam_max, 2001)
            res.ok(f"{name}: 力の山は真のスリップ率 {float(grid[np.argmax(cv.force(grid))]):.3f}、山の 95% の力が出る "
                   f"{target_true:.3f} を狙う")
        res.info(f"{name}: STM32 の目標 = {target_true:.3f} − 読みの遅れ（{a_t:.1f}m/s²・{V_TARGET:.1f}m/s で "
                 f"{a_t * cv.L / V_TARGET:.3f}）= {target:.3f}")
        # 左右の滑り方の違い（ABS は滑りやすい側に合わせるので、片側が弱いと制動力はその側で決まる）
        dl = np.median([q.slip_l for q in p])
        dr = np.median([q.slip_r for q in p])
        if kind == "brake" and abs(dl - dr) > 0.03:
            weak = "左" if dl < dr else "右"
            res.warn(f"{name}: 同じ制動トルクで{weak}後輪がよく滑る（スリップ率の中央値 左 {dl:+.3f}・右 {dr:+.3f}）。"
                     f"ABS は{weak}に合わせるので制動力は{weak}で決まる（{weak}の荷重・タイヤ・MD を確かめる）")
        slope_w = 0.5 * cv.slope(target_true)
        vd = V_DESIGN[kind]
        dg = design_gains(slope_w, c, v=vd)
        kp, ki, wc = dg.kp, dg.ki, dg.wc
        pms = "・".join(f"{v:.1f}m/s で {loop.evaluate(slip_plant(slope_w, c, v), kp, ki)[0]:.0f}°" for v in (0.6, 1.0, 2.0, 3.0)
                        if abs(v - vd) > 1e-9)
        res.info(f"{name}: 目標の点の傾き（1輪）{slope_w:.1f}N・スリップの極 {slip_plant(slope_w, c, vd).pole:.0f}rad/s（{vd:.1f}m/s で設計）"
                 f" → PI 交差 {wc:.0f}rad/s・位相余裕 {dg.pm_deg:.0f}°（ほかの車速: {pms}）")
        old_kp = log.control_params.get(f"{key}_kp_nm_per_m_s")
        old_ki = log.control_params.get(f"{key}_ki_nm_per_m")
        if old_kp and old_ki:
            pm_old, wc_old = loop.evaluate(slip_plant(slope_w, c, vd), old_kp, old_ki)
            res.info(f"{name}: 記録したときのゲイン kp {old_kp:.3f}・ki {old_ki:.2f} は位相余裕 {pm_old:.0f}°・交差 {wc_old:.0f}rad/s")
        res.values[f"control.{key}_slip_target"] = round(target, 4)
        res.values[f"control.{key}_kp_nm_per_m_s"] = round(kp, 5)
        res.values[f"control.{key}_ki_nm_per_m"] = round(ki, 4)
        res.notes[f"control.{key}_slip_target"] = "山の 95% の力が出るスリップ率"
        res.notes[f"control.{key}_kp_nm_per_m_s"] = f"位相余裕 {PM_TARGET_DEG:.0f}°（交差 {wc:.0f}rad/s）"
        if kind == "drive":
            Fz = c.mass * GRAVITY * REAR_SHARE + c.mass * HEIGHT_RATIO * a_t
            mu = cv.D / Fz
            res.values["control.plant.mu"] = round(mu, 4)
            res.values["control.plant.tyre_b"] = round(cv.B, 3)
            res.values["control.plant.tyre_c"] = round(cv.C, 3)
            if has_peak:
                res.values["limits.a_lat_m_s2"] = round(mu * GRAVITY, 3)
                res.notes["limits.a_lat_m_s2"] = (f"= μ·g（μ = {mu:.2f}。後軸の荷重 {REAR_SHARE:.0%}・重心高/ホイールベース "
                                                  f"{HEIGHT_RATIO} の仮定。走行中の学習の初期値）")
                res.info(f"μ ≈ {mu:.2f}（駆動の上限 {cv.D:.2f}N ÷ 後輪の荷重の見積もり {Fz:.2f}N）→ 横の上限の見積もり "
                         f"{mu * GRAVITY:.2f}m/s²（前後と横の μ が同じ・前後輪の荷重の分担が横力の分担と同じ、を仮定）")
            else:
                # 山が範囲の外なら D は外挿で、μ・横の上限は出せない（2026-10-19 の実機で μ 1.24・12.1m/s² と出た）
                res.warn(f"駆動の力の山が測った範囲に無いので μ（{mu:.2f}）は外挿で当てにならない。横の上限 "
                         "limits.a_lat_m_s2 は書かない（今の値のまま。走行中の学習に任せる）")
        lam_plot = np.linspace(0, max(lam_max, 0.05) * 1.1, 100)
        res.plot(f"{name}: 前後力の合計 [N]（横軸: スリップ率）", "当てはめ", lam_plot, cv.force(lam_plot))
        res.plot(f"{name}: 前後力の合計 [N]（横軸: スリップ率）", "実測（真のスリップ率に直した点）",
                 [cv.measured_to_true(q.slip, q.accel, q.v) for q in p], [q.force for q in p])
    return res
