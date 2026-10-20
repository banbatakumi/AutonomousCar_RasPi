"""校正 7: タイヤ（`raspi/auto/calib_tyre.py` の記録）→ TC・ABS の目標スリップ率。設計は `docs/calibration.md` §4.7。

## 1. 曲線を測る

前進の片道ごとに、TC・ABS を切って**トルクをゆっくり 0 から最大まで上げた**ランプの記録（`dramp`＝駆動・`bramp`＝制動。
手前の `trim` はゼロ点）。制御器に依らず、低いスリップ率から山の先まで 1 本のランプで測れる。ランプの区間を
`_WINDOW_S` ずつ（半分ずつ重ねて）区切り、各窓で

- スリップ率: STM32 が測って制御に使っている値（TELEMETRY の `tc_slip`）の左右別。**`trim`（ほぼ加速していない定速）の
  中央値を引く**（前輪の倍率誤差などによる偏り。0.006〜0.011 ある）。制動は符号を反転して大きさにする
- 前後力の合計: 車体の運動から F = m_b·(a + r_b)。a は走行距離を 2 次式で当てはめた加速度、m_b は車体の実効の質量
  （校正 5 の G から後輪の回転の慣性を除いたもの）、r_b は車体の転がり抵抗
- 前後力の左右の比: 車輪の運動方程式から f = (T − J·ω̇ − T_f)/r（T は各輪のトルク指令、ω̇ は周速の傾き。J・T_f は校正 6）。
  ランプが遅ければ慣性の項は力の 3% 程度で、空転・ロックに入った後も慣性の項ごと数えているので力が出る。
  **各輪の力は、車体の合計を車輪から出した左右の比で配る**（F_L = F·f_L/(f_L + f_R)）。MD のトルク定数や、制動の低い周速での
  トルク抜け（約 20rad/s 以下）など、指令と実際のトルクの差は左右で同じと見て、比には効かない。車輪から出した和と車体の合計の
  比（0.85〜1.15）は照合に使う（外れていれば質量・慣性・摩擦・MD のどれかがずれている）。
  左右別に出るので、**左右の荷重・グリップが違っても、各輪の曲線をそのまま別々に測れる**

**目標スリップ率は各輪の曲線から決め、大きい方（弱い輪）に合わせる**（TC は各輪が同じ目標を使い、ABS は滑りやすい側に
合わせるので、弱い輪の肩まで滑らせないと、その輪の力が 95% に届かない）。

**STM32 が測るスリップ率は、加減速中に真の値より小さく読める**: 後輪の周速は MD の報告とローパスで遅れ L を持ち、
加速度 a の間は周速が a·L だけ古い。スリップ率にすると a·L/v（低速ほど大きい。シムで 0.75m/s・2.5m/s² のとき約 0.02）。
そこで曲線は真のスリップ率

    λ = λ_STM32 + |a|·L/v,   f(λ) = sin(C·atan(B·λ))

で当てはめる。**L は駆動の各輪の曲線で決め（中央値）、駆動・制動の当てはめはその値に固定する**（制動の曲線は L が B・C と
入れ替わって決まらず、探索範囲の上限に張り付く。駆動では実機・シムとも 30〜60ms に決まる。校正 6 の MD の遅れの合計 16ms より
大きいのは、基準速度の遅れなどを含む見かけの値）。左右・駆動・制動でそれぞれ別々に当てはめる。

## 2. 目標スリップ率

試したスリップ率の範囲で曲線の最大 F_max を取り、**F が F_max の 95% に届く最小の真のスリップ率** λ95。STM32 の目標は
STM32 の測り方の値なので、`V_TARGET` の車速・その力の加速度での読みの遅れを引いて λ95 − a·L/v にする（高速ほど
真の値に近づき、低速では少し山の先まで滑らせる）。山の手前に置けば、
外乱で少し超えても力が落ちず戻れる（山を越えると力が落ち、横のグリップも失う）。山が試した範囲に無ければ★
（力が最大トルクまで増え続けた＝この路面ではトルクが先に頭打ち。**前後の力を優先する方針で、最大トルクまで滑らせる**: 目標は
ランプの終わりのスリップ率の中央値、上限 `_NO_PEAK_CAP`）。

## 3. ゲインは決めない

2026-10-20: 位相余裕の式でのゲイン設計（スリップ速度 u' = (r/J)·T − a·u の線形化）は、実機でゲインが高すぎて振動した。
TC・ABS を切ったランプでも制動側の周速は 0.03m/s 揺れ（定速は 0.004）、揺れの一部は制御器の外（MD・タイヤ）にある。
ゲインは校正 8（`calib_tc_gain`）が、交差周波数を振って実機の閉ループで決める。

## 4. 曲線が標準の式に合わないとき

実機のタイヤは、ごく低いスリップ率で力が立ち上がり（約 4N が 0.01 未満）、その後ゆっくり増えて山が見えないことがある。
F = D·sin(C·atan(B·λ)) の係数が探索範囲の端（C = 0.3 など）に張り付いたら、形を仮定せず、真のスリップ率を 0.02 ごとに
区切った力の中央値（3 区間で平滑）から、最大の 95% に届く最小のスリップ率を目標にする（`empirical_peak`）。μ・タイヤの係数
（`control.plant.*`）・横の上限は、式が決まったときだけ書く。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from scipy.optimize import least_squares

from raspi.auto._calib_common import phase_parts

from . import toml_io
from .encoder import _stm_time
from .log import Log
from .result import Result
from .speed import body_distance

__all__ = ["CurvePoint", "Curve", "Empirical", "curve_points", "fit_curve", "estimate_lag", "empirical_peak", "pick_target", "analyze", "Consts"]

R = 0.030
GRAVITY = 9.80665
#: 目標スリップ率の読みの遅れを戻すときの車速 [m/s]（発進から加速していく途中の代表）
V_TARGET = 1.0
#: ランプを区切る窓の長さ [s]（半分ずつ重ねる）と、ゼロ点に使う `trim` の頭で捨てる時間 [s]
_WINDOW_S = 0.1
_TRIM_SKIP_S = 0.05
#: 曲線の点がこれ未満ならやり直し。スリップ率がこの範囲に届いていないと、肩か山の先が測れていない
_MIN_POINTS = 12
_SLIP_LOW_OK = 0.03
_SLIP_HIGH_OK = 0.15
_ACTIVE_FRAC_MAX = 0.05
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
    target: float       #: （旧方式の目標スリップ率。ランプでは nan）
    slip: float         #: スリップ率（STM32 の測り方、ゼロ点を引いた、左右の平均。正の大きさ）
    force: float        #: 前後力の合計 [N]（正の大きさ）
    v: float
    slip_l: float
    slip_r: float
    accel: float = 0.0  #: 車体の加速度の大きさ [m/s²]
    #: 左右の後輪が地面から受けた前後力の大きさ [N]（各輪のトルク指令と周速の変化から。モジュールの docstring §1）
    f_l: float = float("nan")
    f_r: float = float("nan")
    #: 車輪から出した左右の和（車体から出した合計 `force` との照合用）
    f_wheels: float = float("nan")


@dataclass
class Curve:
    D: float
    B: float
    C: float
    resid: float
    n: int
    #: STM32 のスリップ率の読みの遅れ [s]
    L: float = 0.0

    @property
    def degenerate(self) -> bool:
        """係数が探索範囲の端に張り付いた＝曲線の形が決まっていない（山が見えない・計測の窓が狭い）。"""
        return self.C < _C_MIN + 0.01 or self.B > _B_MAX - 10.0

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


_RAMP_LABEL = {"drive": "dramp", "brake": "bramp"}


def _label(name: str) -> str:
    return (phase_parts(name) or ("",))[0]


def curve_points(log: Log, c: Consts) -> tuple[list[CurvePoint], dict[str, float]]:
    """ランプの窓ごとの（スリップ率・前後力）の点と、診断（制御器が働いていた割合・ゼロ点）。"""
    t = _stm_time(log)
    x = body_distance(log)
    v = np.gradient(x, t)
    slip = np.asarray(log.vs["tc_slip"], dtype=float)
    wsp = np.asarray(log.vs["wheel_speed"], dtype=float)[:, 2:4]
    tqc = np.asarray(log.vs["torque_cmd"], dtype=float)
    tcA = np.asarray(log.vs["tc_active"]).astype(bool)
    abA = np.asarray(log.vs["abs_active"]).astype(bool)
    out: list[CurvePoint] = []
    n_active = n_ramp = 0
    offsets: list[float] = []
    for kind in ("drive", "brake"):
        sgn_slip = 1.0 if kind == "drive" else -1.0
        for name, i0, i1 in log.segments(lambda p, k=kind: _label(p) == _RAMP_LABEL[k]):
            leg = (phase_parts(name) or ("", -1))[1]
            # ゼロ点: 同じ片道の trim（定速）の頭を捨てた後の中央値
            trims = [(a, b) for nm, a, b in log.segments(lambda p: _label(p) == "trim")
                     if (phase_parts(nm) or ("", -2))[1] == leg]
            off = np.zeros(2)
            if trims:
                a0, b0 = trims[0]
                keep = np.arange(a0, b0)
                keep = keep[t[keep] - t[a0] >= _TRIM_SKIP_S]
                if len(keep) >= 3:
                    off = np.median(slip[keep], axis=0)
                    offsets.extend(abs(off))
            idx = np.arange(i0, i1)
            n_ramp += len(idx)
            n_active += int(np.sum(tcA[idx] | abA[idx]))
            idx = idx[np.abs(v[idx]) >= _MIN_V]
            if len(idx) < 6:
                continue
            width = max(6, int(_WINDOW_S / max(np.median(np.diff(t[idx])), 1e-3)))
            for st in range(0, max(1, len(idx) - width + 1), max(1, width // 2)):
                part = idx[st:st + width]
                if len(part) < 6:
                    continue
                tt = t[part] - t[part[0]]
                cf = np.polyfit(tt, x[part], 2)
                a = 2 * cf[0]
                vm = float(np.mean(v[part]))
                sgn = 1.0 if vm >= 0 else -1.0
                F = c.m_body * (a + c.rr_body * sgn) * sgn
                if kind == "brake":
                    F = -F
                sl = sgn_slip * float(np.mean(slip[part, 0]) - off[0])
                sr = sgn_slip * float(np.mean(slip[part, 1]) - off[1])
                # 各輪の前後力: F = (T − J·ω̇ − T_f)/r（前進の向きを正。制動は大きさにする）
                fw = []
                for w in (0, 1):
                    dv_dt = float(np.polyfit(tt, wsp[part, w], 1)[0])
                    f = (float(np.mean(tqc[part, w])) - c.J * dv_dt / R - c.Tf * sgn) / R
                    fw.append(-f if kind == "brake" else f)
                if fw[0] + fw[1] < 0.3 or F < 0.3:
                    continue
                # 左右の比は車輪から、大きさは車体から（MD のトルク定数・制動の低い周速でのトルク抜けなど、指令と実際のトルクの
                # 差は左右で同じと見て、比には効かない）。車輪から出した和と車体の合計の比は照合用に残す
                k = F / (fw[0] + fw[1])
                out.append(CurvePoint(kind, float("nan"), 0.5 * (sl + sr), F, abs(vm), sl, sr, abs(a),
                                      k * fw[0], k * fw[1], fw[0] + fw[1]))
    diag = {"active_frac": n_active / n_ramp if n_ramp else 0.0, "offset_max": max(offsets) if offsets else 0.0}
    return out, diag


_C_MIN, _B_MAX, _LAG_MAX = 0.3, 300.0, 0.12
#: 山を読む真のスリップ率の上限。これより先はタイヤが完全に滑っている（ロック・空転）ので、力が増えても目標にはしない
_LAM_CAP = 0.5
#: 目標スリップ率の上限（STM32 の読み）。ピーク付近は文献で 6〜20%（Formula Student で約 18%）。これより先は横力をほぼ失う
_TARGET_CAP = 0.3


def fit_curve(points: list[CurvePoint], lag: float | None = None) -> Curve:
    """F = D·sin(C·atan(B·λ_true)) を当てはめる。λ_true = 読み + |a|/v·L。

    `lag` を与えると読みの遅れ L をその値に固定する。None なら L も未知数にする（初期値を 3 通り試す）。L は駆動の
    曲線では決まる（実機・シムとも 30〜60ms）が、制動の曲線では B・C と入れ替わって決まらない（探索範囲の上限に
    張り付く）。そこで駆動で L を決め（`estimate_lag`）、制動はその値に固定する。
    """
    fit_lag = lag is None
    lam = np.array([0.0] + [p.slip for p in points])
    F = np.array([0.0] + [p.force for p in points])
    acc_v = np.array([0.0] + [p.accel / max(p.v, 0.3) for p in points])
    w = np.ones_like(lam)
    w[0] = 3.0                                   # 原点（転がっているだけなら力 0）は確か

    def resid(q):
        D, B, C = q[:3]
        L = q[3] if fit_lag else lag
        return w * (D * np.sin(C * np.arctan(B * (lam + acc_v * L))) - F)

    best = None
    for B0 in (5.0, 15.0, 40.0):
        for C0 in (1.0, 1.5):
            for L0 in ((0.01, 0.04, 0.08) if fit_lag else (None,)):
                x0 = [max(F.max(), 0.5), B0, C0] + ([L0] if fit_lag else [])
                lb = [0.05, 0.5, _C_MIN] + ([0.0] if fit_lag else [])
                ub = [50.0, _B_MAX, 2.0] + ([_LAG_MAX] if fit_lag else [])
                sol = least_squares(resid, x0, bounds=(lb, ub))
                if best is None or sol.cost < best.cost:
                    best = sol
    D, B, C = best.x[:3]
    r = resid(best.x)[1:]
    L = float(best.x[3]) if fit_lag else float(lag)
    return Curve(float(D), float(B), float(C), float(np.sqrt(np.mean(r ** 2))), len(points), L)


def estimate_lag(sets: list[list[CurvePoint]]) -> tuple[float, bool]:
    """駆動の各輪の曲線から、スリップ率の読みの遅れ L の中央値と、探索範囲の上限に張り付いたか（決まっていないか）。"""
    lags = [fit_curve(pw).L for pw in sets if len(pw) >= _MIN_POINTS]
    if not lags:
        return 0.0, True
    L = float(np.median(lags))
    return L, L > 0.97 * _LAG_MAX


@dataclass
class Empirical:
    """曲線の形を仮定せず、真のスリップ率の区間ごとの力の中央値から読んだ山。"""

    target: float       #: 力が最大の 95% に届く最小の真のスリップ率（区間の中心）
    has_peak: bool      #: 試した範囲の中で、最大より力が落ちていたか
    f_max: float        #: 区間ごとの力（平滑後）の最大 [N]
    lam_max: float      #: 試した真のスリップ率の最大
    n_bins: int         #: 群の数


def empirical_peak(points: list[CurvePoint], lag: float, min_n: int = 3, frac: float = 0.95) -> Empirical | None:
    """真のスリップ率の小さい順に点を同じ数ずつの群に分け、群ごとの力の中央値（3 群で平滑）から山を読む。点が足りなければ None。
    曲線の形を仮定しないので、山が無い・形が標準の式に合わない場合でも目標が出る（山が無ければ範囲の端の 95%）。
    群の幅を揃えない（点の数を揃える）のは、スリップ率の高い側は点がまばらなため。"""
    n = len(points)
    if n < 4 * min_n:
        return None
    lam = np.array([p.slip + abs(p.accel) * lag / max(p.v, 0.3) for p in points])
    F = np.array([p.force for p in points])
    keep = lam <= _LAM_CAP                       # 完全にロック・空転した先は山の議論に含めない
    lam, F = lam[keep], F[keep]
    n = len(lam)
    if n < 4 * min_n:
        return None
    order = np.argsort(lam)
    size = max(min_n, n // 10)
    groups = [order[i:i + size] for i in range(0, n - size + 1, size)]
    if n - len(groups) * size >= min_n:                       # 余りも一つの群にする
        groups.append(order[len(groups) * size:])
    else:
        groups[-1] = np.concatenate([groups[-1], order[len(groups) * size:]])
    cen = np.array([float(np.mean(lam[g])) for g in groups])
    med = np.array([float(np.median(F[g])) for g in groups])
    sm = med.copy()
    sm[1:-1] = (med[:-2] + med[1:-1] + med[2:]) / 3.0
    k = int(np.argmax(sm))
    f_max = float(sm[k])
    has_peak = k < len(sm) - 1 and bool(sm[k + 1:].min() < 0.97 * f_max)
    target = float(cen[int(np.argmax(sm >= frac * f_max))])
    return Empirical(target, has_peak, f_max, float(lam.max()), len(groups))


def pick_target(curve: Curve, lam_max: float, frac: float = 0.95) -> tuple[float, bool]:
    """(目標スリップ率, 山が試した範囲の中にあったか)。"""
    lam = np.linspace(0, lam_max, 2001)
    Fv = curve.force(lam)
    k_peak = int(np.argmax(Fv))
    has_peak = k_peak < len(lam) - 1
    target = float(lam[np.argmax(Fv >= frac * Fv[k_peak])])
    return target, has_peak


def analyze(log: Log, consts: Consts | None = None) -> Result:
    c = consts or Consts.from_toml()
    res = Result("校正 7: タイヤ（スリップ率→前後力）と TC・ABS の目標スリップ率")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）。残った区間で解析する")
    pts, diag = curve_points(log, c)
    res.info(f"使った値: 車体の実効の質量 {c.m_body:.3f}kg・転がり抵抗 {c.rr_body:.3f}m/s²（校正 5）、後輪の慣性 "
             f"{c.J * 1e6:.1f}e-6kg·m²・遅れ {c.d * 1000:.1f}＋{c.tau * 1000:.1f}ms（校正 6）")
    if not 0.8 * c.mass < c.m_body < 1.2 * c.mass:
        res.warn(f"車体の実効の質量 {c.m_body:.2f}kg が車両の質量 {c.mass:.2f}kg と 2 割以上違う（toml の speed_plant_gain が"
                 "古い？・MD のトルク定数が実際と違う＝指令ほどトルクが出ていない）。前後力はトルク指令の単位で出る"
                 "（TC・ABS の設計はこの単位で閉じるので影響しない。μ はこの比で狂う）")
    if diag["active_frac"] > _ACTIVE_FRAC_MAX:
        res.fail(f"ランプの間に TC・ABS が {diag['active_frac']:.0%} の時間働いていた。切る上書きが STM32 に入っていない"
                 "（記録の /diag/link を確かめる）。曲線は制御器の影響を受けている")
    res.info(f"ゼロ点（定速のときのスリップ率の読みの偏り）: 最大 {diag['offset_max']:.3f}")
    # STM32 のスリップ率の読みの遅れ L: 駆動の各輪の曲線で決める（制動では決まらないので、制動もこの値に固定する）。
    # 校正 6 の遅れ（MD のトルク＋速度の報告＋ローパス = {c.d + c.tau}）より大きいのは、基準速度の遅れなどを含む見かけの値
    drive_sets = []
    pd = [q for q in pts if q.kind == "drive"]
    if len(pd) >= _MIN_POINTS:
        drive_sets = [[replace(q, slip=(q.slip_l, q.slip_r)[w], force=(q.f_l, q.f_r)[w]) for q in pd] for w in (0, 1)]
    lag, lag_pinned = estimate_lag(drive_sets)
    if not drive_sets or lag_pinned:
        lag = c.d + c.tau
        res.warn(f"スリップ率の読みの遅れ L が決まらなかった（駆動の曲線の点が足りない、または探索範囲の上限 {_LAG_MAX * 1000:.0f}ms に"
                 f"張り付いた）。校正 6 の遅れの合計 {lag * 1000:.0f}ms を使う")
    else:
        res.info(f"スリップ率の読みの遅れ L = {lag * 1000:.0f}ms（駆動の曲線から。校正 6 の MD の遅れの合計は "
                 f"{(c.d + c.tau) * 1000:.0f}ms。差は基準速度の遅れなどを含む見かけの値。TC・ABS が見るスリップ率はこの分、力より遅れる）")
    targets: dict[str, float] = {}
    plant_ok = True
    d_total = 0.0
    hp_all = True
    a_ts: list[float] = []
    for kind, name, key in (("drive", "駆動（TC）", "tc"), ("brake", "制動（ABS）", "abs")):
        p = [q for q in pts if q.kind == kind]
        if len(p) < _MIN_POINTS:
            res.fail(f"{name}: 曲線の点が {len(p)} 個しかない（{_MIN_POINTS} 個以上要る。`calib_tyre` の記録か、ランプが"
                     "短く切られていないか確かめる）")
            continue
        slips = np.array([max(q.slip_l, q.slip_r) for q in p])
        if slips.max() < _SLIP_HIGH_OK:
            res.warn(f"{name}: スリップ率が {slips.max():.2f} までしか上がっていない（{_SLIP_HIGH_OK} 以上要る）。力の頭打ちが"
                     "見えず、目標は当てにならない（路面のグリップが強すぎる？ランプが直線の端で切られた？）")
        low = np.array([min(q.slip_l, q.slip_r) for q in p])
        if low.min() > _SLIP_LOW_OK:
            res.warn(f"{name}: 最も低いスリップ率が {low.min():.3f}。立ち上がり（肩）が測れていない")
        if kind == "drive":
            # 駆動: 各輪のトルク指令が実際に出る（MD のトルク抜けが無い）ので、各輪の力を車輪から出せる。各輪の曲線を別々に
            # 当てはめ、肩が遠い方（弱い輪）に目標を合わせる
            sets = [(wname, [replace(q, slip=(q.slip_l, q.slip_r)[w], force=(q.f_l, q.f_r)[w]) for q in p])
                    for w, wname in ((0, "左"), (1, "右"))]
        else:
            # 制動: MD は低い周速でトルクが抜ける（指令と実際が違う）ので、車輪から各輪の力は出せない。ABS は左右の滑りやすい方に
            # 合わせる（select-low）ので、車体の合計の力を、滑りやすい方のスリップ率に対して当てはめる（ABS が実際に見る関係）
            sets = [("滑りやすい方", [replace(q, slip=max(q.slip_l, q.slip_r)) for q in p])]
        tts, hps, tgs, ats, plats, cvs = [], [], [], [], [], []
        for idx_w, (wname, pw) in enumerate(sets):
            cv_w = fit_curve(pw, lag=lag)
            emp = empirical_peak(pw, lag)
            lm = max(cv_w.measured_to_true(q.slip, q.accel, q.v) for q in pw)
            unit = "1輪" if kind == "drive" else "左右の合計"
            if cv_w.degenerate and emp is not None:
                # 式が合わない（係数が端に張り付いた）: 形を仮定せず、区間ごとの力の中央値から山を読む
                tt, hp, plateau = emp.target, emp.has_peak, emp.f_max
                plant_ok = False
                res.info(f"{name}・{wname}: 標準の曲線 F = D·sin(C·atan(B·λ)) が合わない（C={cv_w.C:.2f}・B={cv_w.B:.0f} が範囲の端）。"
                         f"群ごとの力から読む（{emp.n_bins} 群、真のスリップ率〜{emp.lam_max:.2f}）: 力の上限 {plateau:.2f}N・"
                         f"95% の力が出る真のスリップ率 {tt:.3f}")
            else:
                tt, hp = pick_target(cv_w, lm)
                plateau = float(cv_w.force(lm))
                if cv_w.degenerate:
                    plant_ok = False
                res.info(f"{name}・{wname}: F = {cv_w.D:.2f}·sin({cv_w.C:.2f}·atan({cv_w.B:.1f}·λ)) [N]（{unit}、λ は真のスリップ率、"
                         f"{cv_w.n} 点、残差 {cv_w.resid:.2f}N）。力の上限 {plateau:.2f}N・95% の力が出る真のスリップ率 {tt:.3f}")
                if cv_w.resid > 0.1 * max(plateau, 1e-9):
                    res.warn(f"{name}・{wname}: 曲線の残差 {cv_w.resid:.2f}N が上限の 1 割を超える（路面のむら・ランプが速すぎる？）")
            at = plateau / (0.5 * c.m_body if kind == "drive" else c.m_body)
            # 目標は**測定したスリップ率のまま**（STM32 が TC・ABS に使う値。遅れの補正をしない）で、力が頭打ちになる点: 群ごとの力の
            # 中央値が最大の 95% に届く最小のスリップ率。ピーク付近を目標にするのは文献どおり（乾いたタイヤは 6〜20%）。
            # 山が見えなくても（力が緩やかに増え続ける）、95% に届く点で頭打ちとみなす。遅れの補正は目標を実際の発進より低く
            # 出し（実機: 補正した版は 0.08〜0.1 で、TC に保たせると加速が 2.9→2.2m/s² に落ちた）、測定値の knee は文献と合う範囲に出る
            emp0 = empirical_peak(pw, 0.0)
            if emp0 is not None:
                tg = max(0.02, min(_TARGET_CAP, emp0.target))
                hp = emp0.has_peak
                tt = emp0.target
                note = "" if emp0.target <= _TARGET_CAP else f"（上限 {_TARGET_CAP} で頭打ち）"
                res.info(f"{name}・{wname}: 測定したスリップ率で、力（群ごとの中央値）が最大 {emp0.f_max:.2f}N の 95% に届くのは "
                         f"{emp0.target:.3f}（{emp0.n_bins} 群、〜{emp0.lam_max:.2f}）{note} → 目標 {tg:.3f}")
                if not hp:
                    res.warn(f"{name}・{wname}: 試したスリップ率（〜{emp0.lam_max:.2f}）の中に力の山（その先で力が落ちる点）は見えない。"
                             f"力が頭打ちになる点（95%）の {tg:.3f} を目標にした。山がもっと先なら、力を少し捨てている")
            elif hp:
                tg = max(0.02, tt - at * lag / V_TARGET)
            else:
                top = sorted(q.slip for q in pw)[-max(3, len(pw) // 10):]
                tg = max(0.02, min(_TARGET_CAP, float(np.median(top))))
                res.warn(f"{name}・{wname}: 点が足りず山を読めない。ランプの終わりのスリップ率 {tg:.3f} を目標にした（低めに出る）")
            tts.append(tt)
            hps.append(hp)
            ats.append(at)
            plats.append(plateau)
            cvs.append(cv_w)
            tgs.append(tg)
        if kind == "drive":
            # TC は各輪が同じ目標を使う。左右の肩が違うとき、弱い輪（大きい方）に合わせると強い輪の保護が緩く、強い輪に合わせると
            # 弱い輪の力を削る。力は肩の付近でほぼ平らなので、**左右の平均**を採る（実機: 左 0.15・右 0.31 → 0.23。校正 8 の実測で TC に
            # 0.2 前後を保たせても加速は 0.3 のときと 7% 以内、横力の余裕は残る）
            target = float(np.mean(tgs))
            if abs(tgs[0] - tgs[1]) < 0.4 * max(tgs):
                res.ok(f"{name}: 目標スリップ率 {target:.3f}（左右の平均。左 {tgs[0]:.3f}・右 {tgs[1]:.3f}）")
            else:
                res.warn(f"{name}: 左右の肩が違う（左 {tgs[0]:.3f}・右 {tgs[1]:.3f}）。平均 {target:.3f} を採る。肩が高い輪は力を少し捨てる"
                         "（左右の荷重・タイヤ・MD を揃えると目標を 1 つにできる）")
            ratio = plats[0] / plats[1] if plats[1] > 1e-9 else float("inf")
            res.info(f"{name}: 力の上限の左右比 左/右 = {ratio:.2f}（左 {plats[0]:.2f}N・右 {plats[1]:.2f}N）")
            if not 0.8 < ratio < 1.25:
                lw = "左" if ratio < 1 else "右"
                res.warn(f"{name}: {lw}後輪の力の上限が相手の {min(ratio, 1 / ratio):.0%}（同じトルクで{lw}がよく滑る。"
                         f"{lw}の荷重・タイヤ・MD を確かめる）")
            d_total = float(sum(plats))
            hp_all = all(hps)
            a_ts = ats
        else:
            target = tgs[0]
            res.ok(f"{name}: 目標スリップ率 {target:.3f}")
        targets[kind] = target
        # 同じ力を出すのに左右で滑り方がどれだけ違うか（ランプの前半: 力が出始めて肩の手前まで）
        sel = [q for q in p if 0.03 < max(q.slip_l, q.slip_r) < 0.4 and q.force > 1.0]
        if sel:
            lr = float(np.median([max(q.slip_l, 1e-3) / max(q.slip_r, 1e-3) for q in sel]))
            if not 0.67 < lr < 1.5:
                lw = "左" if lr > 1 else "右"
                res.warn(f"{name}: 同じトルクで{lw}後輪のスリップ率が {max(lr, 1 / lr):.1f} 倍（中央値）。{lw}の荷重・タイヤ・MD を確かめる"
                         + ("（ABS は滑りやすい方に合わせるので、制動力は" + lw + "で決まる）" if kind == "brake" else ""))
        # 車輪から出した左右の和と、車体から出した合計の照合（駆動だけ。制動は MD のトルク抜けで合わない）
        sel = [q for q in p if max(q.slip_l, q.slip_r) < 0.25 and q.force > 1.0]
        if sel and kind == "drive":
            fr = float(np.median([q.f_wheels / q.force for q in sel]))
            res.info(f"{name}: 車輪から出した力の和 ÷ 車体から出した力 = {fr:.2f}（1 に近いほど J・質量・MD のトルクが合っている）")
            if not 0.85 < fr < 1.15:
                res.warn(f"{name}: 車輪と車体から出した力が {abs(fr - 1):.0%} 食い違う。校正 5・6 の値（質量・慣性・摩擦）か MD の"
                         "トルク定数がずれている。左右の比と目標は影響が小さいが、μ・横の上限は当てにならない")
        res.values[f"control.{key}_slip_target"] = round(target, 4)
        res.notes[f"control.{key}_slip_target"] = "力が最大の 95% に届く測定したスリップ率（STM32 の読み。TC は左右の平均。各輪の上限 0.3）"
        if kind == "drive":
            drive_cvs = cvs
        for (wname, pw), cv_w in zip(sets, cvs):
            lam_plot = np.linspace(0, max(max(cv_w.measured_to_true(q.slip, q.accel, q.v) for q in pw), 0.05) * 1.1, 100)
            title = f"{name}: {'1輪' if kind == 'drive' else '左右の合計'}の前後力 [N]（横軸: スリップ率）"
            if not cv_w.degenerate:
                res.plot(title, f"当てはめ・{wname}", lam_plot, cv_w.force(lam_plot))
            res.plot(title, f"実測・{wname}", [cv_w.measured_to_true(q.slip, q.accel, q.v) for q in pw],
                     [q.force for q in pw])
    if "drive" in targets:
        a_t = float(np.mean(a_ts))
        Fz = c.mass * GRAVITY * REAR_SHARE + c.mass * HEIGHT_RATIO * a_t
        mu = d_total / Fz
        if plant_ok and hp_all:
            res.values["control.plant.mu"] = round(mu, 4)
            res.values["control.plant.tyre_b"] = round(float(np.mean([cv_w.B for cv_w in drive_cvs])), 3)
            res.values["control.plant.tyre_c"] = round(float(np.mean([cv_w.C for cv_w in drive_cvs])), 3)
            res.values["limits.a_lat_m_s2"] = round(mu * GRAVITY, 3)
            res.notes["limits.a_lat_m_s2"] = (f"= μ·g（μ = {mu:.2f}。後軸の荷重 {REAR_SHARE:.0%}・重心高/ホイールベース "
                                              f"{HEIGHT_RATIO} の仮定。走行中の学習の初期値）")
            res.info(f"μ ≈ {mu:.2f}（駆動の上限 {d_total:.2f}N ÷ 後輪の荷重の見積もり {Fz:.2f}N）→ 横の上限の見積もり "
                     f"{mu * GRAVITY:.2f}m/s²（前後と横の μ が同じ・前後輪の荷重の分担が横力の分担と同じ、を仮定）")
        else:
            # 山が範囲の外・曲線の形が決まらないときは D が当てにならない（2026-10-19 の実機で μ 1.24・12.1m/s² と出た）
            res.warn(f"駆動の力の山が測った範囲に無い、または曲線の形が決まらないので μ（≥ {mu:.2f} 程度）・タイヤの係数・横の上限は"
                     "書かない（今の値のまま。走行中の学習に任せる）")
    res.info("ゲイン（kp・ki）はここでは決めない。校正 8（`calib_tc_gain`）が、実機で交差周波数を振って決める")
    return res
