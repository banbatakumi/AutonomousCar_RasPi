"""同定の試験の記録から、車両モデル（`plant.Plant`）のパラメータを求める。

| 試験（`raspi/auto`） | 求めるもの | 関数 |
|---|---|---|
| `sysid_wheel`（後輪を浮かせて回す） | 車輪の慣性・回転の摩擦・MD のトルクの遅れ | `fit_wheel()` |
| `sysid_tyre`（TC・ABS を切って滑らせる） | タイヤの前後力の曲線（μ・荷重移動・傾き・形） | `fit_tyre()` |

TV（ヨー）の同定は持たない（実測でヨーモーメント 0.1N·m →
0.056rad/s、上限 0.15N·m でも旋回中のヨーレートの約3%しか動かせず、同定して最適化する効きが無い。
限界での効きを測るには定常円の試験と横力が飽和するモデルが要る——`PROGRESS.md` 参照）。

`analyze()` がこの順に解析し、前の結果を次に使う（タイヤの力の計算に車輪の慣性が要る）。
求めた値は `config/vehicle.toml` の `[control.plant]` に入れる（`tuning.apply_plant()`）。

## 考え方

どれも**出力誤差**で当てはめる: 記録のトルク指令を車両モデルと同じ式に入れて後輪の回転を計算し、記録との差が最小になるパラメータを探す（`tools/sysid/fit.py` と同じ方針）。
TELEMETRY は 100Hz で、後輪の周速にはファームのローパスが掛かっている。その遅れは
ファームの定数として既知なので、モデル側の出力に同じ遅れを掛けてから比べる。

## ★（警告）

記録がモデルの形から外れている・測りたいところまで届いていないときは `FitResult.warnings` に入る。
GUI はその試験の値を既定で適用しない。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from .plant import Plant
from .record import Rec, load_mcap

__all__ = ["FitResult", "Analysis", "fit_wheel", "fit_tyre", "analyze", "TESTS"]

WHEEL_RADIUS_M = 0.030          # ファームの DRIVE_REAR_WHEEL_RADIUS_M
REAR_TRACK_M = 0.155            # 同 DRIVE_REAR_TRACK_M
#: 記録の後輪の周速の遅れ [s]: ファームのローパス（DRIVE_LPF_K_REAR=0.90 を 2kHz で回す
#: 1次遅れ 4.75ms）＋ MD が報告する角速度の遅れ（約2ms）
REAR_OBS_TAU_S = 0.00675
#: MD の制動が抜ける角速度 [rad/s]（MD 側の実装 -tanh(ω/20)）
BRAKE_FADE_RAD_S = 20.0
_SUB = 20                       # 1件（10ms）を刻む数

#: (内部キー, 表示名, planner の id, 求めるキー)。並び順が解析の順序
TESTS: tuple[tuple[str, str, str, tuple[str, ...]], ...] = (
    ("wheel", "Ⓐ 後輪の空転試験", "sysid_wheel", ("wheel_inertia_kgm2", "wheel_friction_nm", "md_tau_s")),
    ("tyre", "Ⓑ タイヤの前後力試験", "sysid_tyre", ("mu", "load_transfer", "tyre_b", "tyre_c")),
)


@dataclass
class FitResult:
    values: dict[str, float]
    notes: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    def warn(self, text: str) -> None:
        self.warnings.append(text)
        self.notes.append("★" + text)


def _dt(rec: Rec) -> float:
    return float(np.median(np.diff(rec.t)))


#: TELEMETRY の取りこぼしとみなす間隔（中央値の何倍か）。微分（`savgol_filter`）と刻みの積分
#: （`_spin_sim`）は等間隔を前提にしているので、多ければ所見で知らせる
_GAP_FACTOR = 1.5
_GAP_WARN_FRAC = 0.02


def _gap_note(rec: Rec) -> str | None:
    """取りこぼしが多い記録の所見（少なければ None）。"""
    d = np.diff(rec.t)
    if len(d) == 0:
        return None
    frac = float(np.mean(d > _GAP_FACTOR * np.median(d)))
    if frac <= _GAP_WARN_FRAC:
        return None
    return (f"TELEMETRY の取りこぼしが {frac * 100:.0f}%（間隔が中央値の{_GAP_FACTOR}倍超）。解析は等間隔を"
            "前提に微分・積分するので、値がずれる")


def _md_torque(cmd: np.ndarray, omega: np.ndarray, dt: float, tau: float, delay: float) -> np.ndarray:
    """記録のトルク指令（駆動は正、制動は負）→ モータが実際に出したトルク。

    制動は回転を妨げる向きに `tanh(ω/20)` で掛かる。むだ時間 `delay` の後に1次遅れ `tau`。
    """
    target = np.where(cmd >= 0.0, cmd, cmd * np.tanh(omega / BRAKE_FADE_RAD_S))
    t = np.arange(len(cmd)) * dt
    target = np.interp(t - delay, t, target, left=target[0])
    out = np.empty_like(target)
    y = target[0]
    k = 1.0 - math.exp(-dt / max(tau, 1e-6))
    # TELEMETRY の値はその瞬間の指令（0次ホールド）。区間の終わりの値を出す
    for i, x in enumerate(target):
        y += (x - y) * k
        out[i] = y
    return out


def _limited(rec: Rec) -> bool:
    """トルク直接指令の間に、左右のトルク指令が食い違ったか（どちらかの輪が絞られた）。

    試験は左右へ同じトルクを指令する（TC・TV・片輪浮き対策は切ってある）。食い違うのは、それらが
    切れていなくて片輪を絞ったとき。
    """
    m = rec.torque_mode & (np.maximum(rec.torque_left, rec.torque_right) > 0.004)
    if not np.any(m):
        return False
    return bool(np.mean(np.abs(rec.torque_left[m] - rec.torque_right[m]) > 0.002) > 0.05)


# ── Ⓐ 後輪の空転 ──────────────────────────────────────────────────────────

def _spin_sim(cmd: np.ndarray, omega0: float, dt: float, inertia: float, friction: float, tau: float,
              delay: float) -> np.ndarray:
    """トルク指令 → 記録に出るはずの後輪の角速度（ローパス後）。タイヤは路面に触れていない。"""
    h = dt / _SUB
    n_delay = int(round(delay / h))
    buf = [cmd[0]] * (n_delay + 1)
    omega = obs = omega0
    torque = 0.0
    out = np.empty(len(cmd))
    k_md = h / (tau + h)
    k_obs = h / (REAR_OBS_TAU_S + h)
    for i, c in enumerate(cmd):
        for _ in range(_SUB):
            buf.append(c)
            c_delayed = buf.pop(0)
            target = c_delayed if c_delayed >= 0.0 else c_delayed * math.tanh(omega / BRAKE_FADE_RAD_S)
            torque += (target - torque) * k_md
            omega += (torque - friction * math.tanh(omega)) / inertia * h
            obs += (omega - obs) * k_obs
        out[i] = obs
    return out


def fit_wheel(rec: Rec, base: Plant) -> FitResult:
    dt = _dt(rec)
    out = FitResult({})
    per_wheel = []
    for name, speed, cmd in (("左", rec.wheel_left, rec.torque_left), ("右", rec.wheel_right, rec.torque_right)):
        omega = speed / WHEEL_RADIUS_M
        if np.max(np.abs(speed)) < 0.5:
            out.warn(f"{name}後輪がほとんど回っていません（最高 {np.max(np.abs(speed)):.2f}m/s）")
            continue
        # トルクを掛け始める少し前から。指令は次の件まで同じ値が続くとみなす
        active = np.nonzero(np.abs(cmd) > 1e-4)[0]
        i0 = max(0, int(active[0]) - 5)
        c, w = cmd[i0:], omega[i0:]

        def resid(x, c=c, w=w):
            sim = _spin_sim(c, float(w[0]), dt, math.exp(x[0]), x[1], math.exp(x[2]), base.md_delay_s)
            return (sim - w) * WHEEL_RADIUS_M

        best = None
        for j0 in (2e-5, 8e-5):
            r = least_squares(resid, [math.log(j0), 0.002, math.log(0.004)],
                              bounds=([math.log(3e-6), 0.0, math.log(3e-4)],
                                      [math.log(1e-3), 0.02, math.log(0.05)]))
            if best is None or r.cost < best.cost:
                best = r
        rms = float(np.sqrt(np.mean(best.fun ** 2)))
        per_wheel.append((name, math.exp(best.x[0]), best.x[1], math.exp(best.x[2]), rms))
        out.notes.append(f"{name}後輪: 慣性 {math.exp(best.x[0]) * 1e5:.2f}e-5kg·m²・摩擦 {best.x[1] * 1e3:.1f}mN·m・"
                         f"MD の遅れ {math.exp(best.x[2]) * 1e3:.1f}ms（残差 {rms * 100:.1f}cm/s）")
        if rms > 0.15:
            out.warn(f"{name}後輪: 残差が大きい（{rms * 100:.0f}cm/s）——モデルの形（一定の摩擦＋慣性）に合っていません")
    if not per_wheel:
        return out
    if np.max(np.abs(rec.speed)) > 0.15:
        out.warn(f"前輪が回っています（最高 {np.max(np.abs(rec.speed)):.2f}m/s）——後輪が接地していた可能性")
    if np.any(rec.tc_active) or _limited(rec):
        out.warn("TC か片輪浮き対策が介入しています（TC を切れていない、または左右の回り方が大きく違う）")
    inertias = [w[1] for w in per_wheel]
    if len(inertias) == 2 and abs(inertias[0] - inertias[1]) > 0.3 * float(np.mean(inertias)):
        out.warn(f"左右の慣性が3割以上違います（{inertias[0] * 1e5:.2f}e-5 / {inertias[1] * 1e5:.2f}e-5）")
    out.values = {"wheel_inertia_kgm2": float(np.mean(inertias)),
                  "wheel_friction_nm": float(np.mean([w[2] for w in per_wheel])),
                  "md_tau_s": float(np.mean([w[3] for w in per_wheel]))}
    return out


# ── Ⓑ タイヤの前後力 ──────────────────────────────────────────────────────

#: 一定トルクとみなす最短の長さ [s]、頭の捨てる時間 [s]（MD のトルクと車速の推定が落ち着くまで）、トルクの許容差 [N·m]
HOLD_MIN_S = 0.22
HOLD_SKIP_S = 0.10
HOLD_TORQUE_TOL = 0.002
#: 保持区間として使うスリップ率の上限（これより滑っていたら暴走中で、定常ではない）と、区間内のばらつきの上限
HOLD_MAX_KAPPA = 0.6
HOLD_MAX_KAPPA_STD = 0.15
#: 転がり中の後輪周速/車速の校正の残り [スリップ率]、暴走区間の1サンプルのスリップ率の不確かさ、力の不確かさ [N]
SIGMA_K_SCALE = 0.003
SIGMA_K_SAMPLE = 0.03
SIGMA_F_HOLD = 0.12
SIGMA_F_SAMPLE = 0.3
#: B（スリップ剛性）の対数の標準誤差がこれを超えたら「決まっていない」（0.5 ≒ 約1.6倍の幅）
B_MAX_LOG_SE = 0.5
#: 保持区間の点がこれ未満なら B は決まらない
MIN_HOLD_POINTS = 8


def _rear_ratio(rec: Rec, accel: np.ndarray):
    """転がり中（トルクなし・制動なし・ほぼ定速）の 後輪周速/車速 を左右別に。スリップ率の原点の校正に使う。

    後輪の周速は MD の角速度に車輪の半径を掛けたもので、車速は前輪（校正済み）から。左右の半径や
    エンコーダの個体差で 2% ほどずれる（2026-10-18 の実機: 左 0.996・右 1.017）。小スリップ領域では
    このずれがスリップ率そのものと同じ大きさになる。
    """
    v = rec.speed
    roll = (~rec.torque_mode) & (~rec.brake) & (v > 0.3) & (np.abs(accel) < 0.15)
    out = []
    for w in (rec.wheel_left, rec.wheel_right):
        r = w[roll] / v[roll]
        out.append(float(np.mean(r)) if len(r) >= 20 else None)
    return None if None in out else (out[0], out[1])


def _hold_runs(rec: Rec, dt: float):
    """トルク指令が一定の区間 `(輪, 始め, 終わり)`（頭の `HOLD_SKIP_S` を捨てたあと）。"""
    runs = []
    active = rec.torque_mode | rec.brake
    v = rec.speed
    n = len(v)
    skip = int(round(HOLD_SKIP_S / dt))
    for side, cmd in enumerate((rec.torque_left, rec.torque_right)):
        i = 0
        while i < n:
            if not active[i] or abs(cmd[i]) < 0.01 or v[i] < 0.35:
                i += 1
                continue
            j = i
            while (j + 1 < n and active[j + 1] and rec.brake[j + 1] == rec.brake[i] and v[j + 1] >= 0.35
                   and abs(cmd[j + 1] - cmd[i]) < HOLD_TORQUE_TOL):
                j += 1
            if (j - i + 1) * dt >= HOLD_MIN_S and j + 1 - (i + skip) >= 8:
                runs.append((side, i + skip, j + 1))
            i = j + 1
    return runs


def _tyre_data(rec: Rec, base: Plant, ratio, accel: np.ndarray):
    """記録 → 当てはめに使う点: 輪(0=左,1=右)・スリップ率（校正済み）・前後力 [N]・前後加速度・σκ・σF・保持区間か。

    力はすべて**トルク指令の尺度**（トルクから車輪の慣性・摩擦を引いて半径で割った値）。シム（ctrl_tune）は
    トルク指令をそのまま使い、Ⓐ で求めた慣性・摩擦も指令の尺度の値なので、タイヤの力も同じ尺度で表す。

    - **保持区間**（限界の下の一定トルク）は、区間の平均を1輪ごとに1点にする。スリップ率が定常なので平均でき
      （誤差 0.003 前後）、B が決まる
    - **暴走区間**（空転・ロックまで上げた所）は1サンプルずつ。上限（μ）と限界の先の形に効くが、
      スリップ率の誤差が大きい（σκ 0.03）

    左右に同じトルクを掛けても、左右のグリップは同じではない（2026-10-18 の実機で、同じトルクで左だけが
    先に限界に達した）ので、輪ごとの点のまま持つ。
    """
    dt = _dt(rec)
    v = rec.speed
    rl, rr_ = ratio if ratio is not None else (1.0, 1.0)
    W, K, F, A, SK, SF, H = [], [], [], [], [], [], []
    kappas, forces = {}, {}
    for side, (speed, cmd, r) in enumerate(((rec.wheel_left, rec.torque_left, rl), (rec.wheel_right, rec.torque_right, rr_))):
        omega = np.interp(rec.t + REAR_OBS_TAU_S, rec.t, speed) / WHEEL_RADIUS_M
        omega_dot = savgol_filter(omega, 5, 2, deriv=1, delta=dt)
        torque = _md_torque(cmd, omega, dt, base.md_tau_s, base.md_delay_s)
        force = (torque - base.wheel_inertia_kgm2 * omega_dot - base.wheel_friction_nm * np.tanh(omega)) / WHEEL_RADIUS_M
        kappa = (omega * WHEEL_RADIUS_M / r - v) / np.maximum(np.abs(v), base.slip_speed_floor_m_s)
        kappas[side], forces[side] = kappa, force
        active = (rec.torque_mode | rec.brake) & (v > 0.35) & (np.abs(cmd) > 0.03)
        n = int(active.sum())
        W.append(np.full(n, side)); K.append(kappa[active]); F.append(force[active]); A.append(accel[active])
        SK.append(np.full(n, SIGMA_K_SAMPLE)); SF.append(np.full(n, SIGMA_F_SAMPLE)); H.append(np.zeros(n, dtype=bool))
    runs = _hold_runs(rec, dt)
    n_hold = 0
    for side, i0, i1 in runs:
        k = kappas[side][i0:i1]
        if abs(float(np.mean(k))) > HOLD_MAX_KAPPA or float(np.std(k)) > HOLD_MAX_KAPPA_STD:
            continue
        se = float(np.std(k)) / math.sqrt(max((i1 - i0) / 3.0, 1.0))
        W.append(np.array([side])); K.append(np.array([float(np.mean(k))]))
        F.append(np.array([float(np.mean(forces[side][i0:i1]))])); A.append(np.array([float(np.mean(accel[i0:i1]))]))
        SK.append(np.array([math.hypot(se, SIGMA_K_SCALE)])); SF.append(np.array([SIGMA_F_HOLD])); H.append(np.array([True]))
        n_hold += 1
    # 診断: 車体の加速度から出した力（後輪2つの平均）÷ トルク由来の力。1より大きければ MD の実トルクが指令より大きい
    left = [(a, b) for sd, a, b in runs if sd == 0]
    right = [(a, b) for sd, a, b in runs if sd == 1]
    f_body_sum = f_torque_sum = 0.0
    for a0, a1 in left:
        for b0, b1 in right:
            lo, hi = max(a0, b0), min(a1, b1)
            if hi - lo >= max(8, 0.7 * (a1 - a0)):
                a_m = float(np.mean(accel[lo:hi]))
                sgn = 1.0 if float(np.mean(v[lo:hi])) >= 0 else -1.0
                f_body_sum += abs(0.5 * base.mass_kg * (a_m + sgn * base.rolling_decel_m_s2))
                f_torque_sum += abs(0.5 * (float(np.mean(forces[0][lo:hi])) + float(np.mean(forces[1][lo:hi]))))
    rho = f_body_sum / f_torque_sum if f_torque_sum > 1e-6 else 1.0
    cat = np.concatenate
    return cat(W), cat(K), cat(F), cat(A), cat(SK), cat(SF), cat(H), n_hold, rho


def fit_tyre(rec: Rec, base: Plant) -> FitResult:
    out = FitResult({})
    dt = _dt(rec)
    accel = savgol_filter(rec.speed, 11, 2, deriv=1, delta=dt)
    ratio = _rear_ratio(rec, accel)
    if ratio is None:
        out.warn("転がり区間（トルクなし・ほぼ定速）が取れず、後輪周速の個体差を校正できない（スリップ率の原点がずれる）")
    side, kappa, force, accel_s, sig_k, sig_f, is_hold, n_hold, rho = _tyre_data(rec, base, ratio, accel)
    if len(kappa) < 50:
        out.warn("駆動・制動の区間がほとんどありません")
        return out
    runaway = ~is_hold
    drive_max, brake_min = float(np.max(kappa[runaway])), float(np.min(kappa[runaway]))
    both = drive_max > 0.3 and brake_min < -0.3
    is_right = side == 1

    fz0 = base.fz_static_n
    f_roll = base.wheel_friction_nm / WHEEL_RADIUS_M

    def parts(x):
        mu_l, mu_r, lt, log_b, c = x
        mu = np.where(is_right, mu_r, mu_l)
        b = math.exp(log_b)
        # 転がり中でもタイヤは軸受の摩擦ぶんの力を出している。校正は「転がり中を原点」にするので、その分を足す
        ratio_roll = np.minimum(f_roll / np.maximum(mu * fz0, 1e-6), 0.95)
        k_roll = np.tan(np.arcsin(ratio_roll) / c) / b
        k = kappa + k_roll
        fz = np.maximum(fz0 + lt * base.mass_kg * accel_s * 0.5, 0.0)
        th = np.arctan(b * k)
        model = mu * fz * np.sin(c * th)
        dk = mu * fz * c * b * np.cos(c * th) / (1.0 + (b * k) ** 2)
        return model, dk

    def resid(x):
        model, dk = parts(x)
        return (model - force) / np.sqrt(sig_f ** 2 + (dk * sig_k) ** 2)

    # 荷重移動は駆動と制動の上限の差から決まる。片方しか限界に届いていなければ机上値のまま
    lt_lo, lt_hi = (0.0, 0.6) if both else (base.load_transfer - 1e-9, base.load_transfer + 1e-9)
    best = None
    for c0 in (1.05, 1.3, 1.6):
        for b0 in (8.0, 25.0):
            r = least_squares(resid, [base.mu, base.mu, min(max(base.load_transfer, lt_lo), lt_hi), math.log(b0), c0],
                              bounds=([0.1, 0.1, lt_lo, math.log(2.0), 1.0], [2.0, 2.0, lt_hi, math.log(150.0), 1.9]),
                              loss="soft_l1", f_scale=2.0)
            if best is None or r.cost < best.cost:
                best = r
    mu_l, mu_r, lt, log_b, c = best.x
    mu = math.sqrt(mu_l * mu_r)
    b = math.exp(log_b)
    model, _ = parts(best.x)
    rms = float(np.sqrt(np.mean((model - force) ** 2)))
    # B の不確かさ: 当てはめのヤコビアンから（正規化残差の自乗平均が1を超えたぶんは広げる）
    chi2 = float(np.mean(resid(best.x) ** 2))
    cov = np.linalg.pinv(best.jac.T @ best.jac) * max(chi2, 1.0)
    se_log_b = math.sqrt(max(float(cov[3, 3]), 0.0))
    peak_slip = math.tan(math.pi / (2.0 * c)) / b if c > 1.0 else float("inf")
    drop = 1.0 - math.sin(c * math.pi / 2.0) if c > 1.0 else 0.0
    out.notes.append(f"傾き B {b:.1f}（±{(math.exp(se_log_b) - 1) * 100:.0f}%）・形 C {c:.2f}・荷重移動 {lt:.3f}"
                     f"・μ 左 {mu_l:.3f}／右 {mu_r:.3f}（残差 {rms:.2f}N、{len(kappa)}点うち保持区間 {n_hold}、"
                     f"スリップ率 {brake_min:+.2f}〜{drive_max:+.2f}）")
    if max(mu_l, mu_r) / min(mu_l, mu_r) > 1.4:
        out.notes.append(f"左右のグリップが違う（μ の比 {max(mu_l, mu_r) / min(mu_l, mu_r):.2f}）——同じトルクでも弱い輪が先に滑る。"
                         "車両モデルには左右の平均（幾何平均）μ を入れ、最適化は μ×0.6・×1.4 の場面で両側を見る")
    if n_hold:
        out.notes.append(f"力の尺度（診断）: 車体の加速度から出した力 ÷ トルクから逆算した力 = {rho:.2f}"
                         "（1より大きいほど MD の実トルクが指令より大きい）")
    if ratio is not None:
        out.notes.append(f"後輪周速/車速（転がり中）: 左 {ratio[0]:.4f}・右 {ratio[1]:.4f}（スリップ率の原点を合わせた）")
    fz_d, fz_b = fz0 + lt * base.mass_kg * 1.7, fz0 - lt * base.mass_kg * 1.6
    out.notes.append(f"前後力の上限（後輪1つ）: 駆動 左 約{mu_l * fz_d:.1f}N・右 約{mu_r * fz_d:.1f}N、"
                     f"制動 左 約{mu_l * fz_b:.1f}N・右 約{mu_r * fz_b:.1f}N。"
                     + (f"力が最大になるスリップ率 {peak_slip:.2f}、滑らせ切ると {drop * 100:.0f}% 落ちる"
                        if peak_slip < 2.0 else "滑らせても力がほとんど落ちない形"))
    if drive_max <= 0.3:
        out.warn(f"駆動で限界まで滑っていません（スリップ率の最大 {drive_max:.2f}）——上限と形は外挿")
    if brake_min >= -0.3:
        out.warn(f"制動で限界まで滑っていません（スリップ率の最小 {brake_min:.2f}）——上限と形は外挿")
    if np.any(rec.tc_active & rec.torque_mode) or np.any(rec.abs_active) or _limited(rec):
        out.warn("TC・ABS・片輪浮き対策が介入しています（試験が切るはずのものが切れていない。左右のトルク指令が食い違った）")
    if rms > 1.0:
        out.warn(f"残差が大きい（{rms:.1f}N）——車輪の慣性（Ⓐ）が合っていない可能性")
    if n_hold < MIN_HOLD_POINTS:
        out.warn(f"限界の下で一定トルクを保った区間が{n_hold}個しか取れていません（{MIN_HOLD_POINTS}個以上要る）——"
                 "スリップ剛性 B は決まりません（古い手順の記録か、保つ往復が途中で終わった）")
    elif se_log_b > B_MAX_LOG_SE:
        out.warn(f"スリップ剛性 B の不確かさが大きい（±{(math.exp(se_log_b) - 1) * 100:.0f}%）——保持区間のスリップ率が"
                 "小さすぎて原点のずれと区別できない可能性（転がり区間の校正・車速の誤差を確かめる）")
    # 探索範囲の端に張り付いた値は「求まった」のではなく「範囲で止められた」
    # 形 C の下限 1.0 は「ピークがなく、滑らせても力が落ちない」という物理的な端なので警告しない
    # （注記に「滑らせても力がほとんど落ちない形」と出る）。上限は警告する
    for name, val, lo, hi in (("形 C", c, 0.0, 1.9), ("傾き B", b, 2.0, 150.0), ("μ（左）", mu_l, 0.1, 2.0),
                              ("μ（右）", mu_r, 0.1, 2.0)):
        if val <= lo * 1.001 + 1e-9 or val >= hi * 0.999:
            out.warn(f"{name} が探索範囲の端（{max(lo, 1.0) if name == '形 C' else lo:g}〜{hi:g}）に張り付いた（{val:.3g}）——値は信用できない。"
                     "記録がモデルの形（sin(C·atan(B·κ))）から外れている")
    if both and (lt <= 1e-3 or lt >= 0.6 * 0.999):
        out.warn(f"荷重移動が探索範囲の端（0〜0.6）に張り付いた（{lt:.3f}）")
    out.values = {"mu": float(mu), "tyre_b": float(b), "tyre_c": float(c)}
    if both:
        out.values["load_transfer"] = float(lt)
    return out


# ── まとめて ──────────────────────────────────────────────────────────────

_FNS = {"wheel": fit_wheel, "tyre": fit_tyre}


@dataclass
class Analysis:
    results: dict[str, float] = field(default_factory=dict)
    notes: dict[str, list[str]] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    #: ★の出た試験で求めたキー → 試験の表示名
    warned: dict[str, str] = field(default_factory=dict)
    #: 求めた値を入れた車両モデル
    plant: Plant | None = None


def analyze(sources: dict[str, str | Path | Rec], base: Plant) -> Analysis:
    """`sources` は試験キー（`TESTS`）→ mcap のパス（または読み込み済みの `Rec`）。"""
    out = Analysis()
    plant = base
    for key, label, _planner, params in TESTS:
        src = sources.get(key)
        if src is None:
            continue
        try:
            rec = src if isinstance(src, Rec) else load_mcap(src)
            if rec.md_dropouts:
                # 止まった MD は駆動も制動もせず、車輪の値は古いまま固まる（`tools/sysid/analyze` と同じ扱い）
                names = "・".join(dict.fromkeys(n for _, _, n in rec.md_dropouts))
                raise ValueError(f"★{names}のモータドライバが試験中に無応答になった（{rec.md_dropouts[0][0]:.1f}s から）。"
                                 "この記録は使えない——駆動バッテリーと MD の電源を確かめて録り直してください")
            r = _FNS[key](rec, plant)
            gap = _gap_note(rec)
            if gap is not None:
                r.warn(gap)
        except Exception as e:  # noqa: BLE001 - 画面にそのまま出す
            out.errors.append(f"{label}: {e}")
            continue
        vals = {k: v for k, v in r.values.items() if k in params}
        out.results.update(vals)
        out.notes[label] = list(rec.notes) + r.notes
        if r.warnings:
            out.warned.update({k: label for k in vals})
        plant = replace(plant, **vals)
    out.plant = plant
    return out
