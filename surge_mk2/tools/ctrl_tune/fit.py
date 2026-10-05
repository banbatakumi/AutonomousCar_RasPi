"""同定の試験の記録から、車両モデル（`plant.Plant`）のパラメータを求める。

| 試験（`raspi/auto`） | 求めるもの | 関数 |
|---|---|---|
| `sysid_wheel`（後輪を浮かせて回す） | 車輪の慣性・回転の摩擦・MD のトルクの遅れ | `fit_wheel()` |
| `sysid_tyre`（TC・ABS を切って滑らせる） | タイヤの前後力の曲線（μ・荷重移動・傾き・形） | `fit_tyre()` |
| `sysid_yawmoment`（左右のトルク差を入れる） | ヨーの慣性・減衰 | `fit_yaw()` |

`analyze()` がこの順に解析し、前の結果を次に使う（タイヤの力の計算に車輪の慣性が要る）。
求めた値は `config/vehicle.toml` の `[control.plant]` に入れる（`tuning.apply_plant()`）。

## 考え方

どれも**出力誤差**で当てはめる: 記録のトルク指令を車両モデルと同じ式に入れて後輪の回転・
ヨーレートを計算し、記録との差が最小になるパラメータを探す（`tools/sysid/fit.py` と同じ方針）。
TELEMETRY は 100Hz で、後輪の周速・ヨーレートにはファームのローパスが掛かっている。その遅れは
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

__all__ = ["FitResult", "Analysis", "fit_wheel", "fit_tyre", "fit_yaw", "analyze", "TESTS"]

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
    ("yaw", "Ⓒ ヨーモーメント試験", "sysid_yawmoment", ("yaw_inertia_kgm2", "yaw_damping")),
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

    試験は左右へ同じトルクを指令する（TV は切ってある）。食い違うのは TC か片輪浮き対策が
    片輪を絞ったとき。片輪浮き対策の介入は TELEMETRY の旗に無いので、ここで見る。
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

def _tyre_samples(rec: Rec, base: Plant):
    """記録 → (スリップ率, 前後力 [N], 前後加速度) を両輪ぶん。"""
    dt = _dt(rec)
    v = savgol_filter(rec.odom, 9, 2, deriv=1, delta=dt)
    accel = savgol_filter(v, 11, 2, deriv=1, delta=dt)
    active = (rec.torque_mode | rec.brake) & (v > 0.35)
    kappas, forces, accels = [], [], []
    for speed, cmd in ((rec.wheel_left, rec.torque_left), (rec.wheel_right, rec.torque_right)):
        # 記録の周速はローパスで遅れている。その分だけ先の値がその瞬間の値
        omega = np.interp(rec.t + REAR_OBS_TAU_S, rec.t, speed) / WHEEL_RADIUS_M
        omega_dot = savgol_filter(omega, 5, 2, deriv=1, delta=dt)
        torque = _md_torque(cmd, omega, dt, base.md_tau_s, base.md_delay_s)
        force = (torque - base.wheel_inertia_kgm2 * omega_dot
                 - base.wheel_friction_nm * np.tanh(omega)) / WHEEL_RADIUS_M
        kappa = (omega * WHEEL_RADIUS_M - v) / np.maximum(np.abs(v), base.slip_speed_floor_m_s)
        m = active & (np.abs(cmd) > 0.03)
        kappas.append(kappa[m])
        forces.append(force[m])
        accels.append(accel[m])
    return np.concatenate(kappas), np.concatenate(forces), np.concatenate(accels)


def fit_tyre(rec: Rec, base: Plant) -> FitResult:
    out = FitResult({})
    kappa, force, accel = _tyre_samples(rec, base)
    if len(kappa) < 50:
        out.warn("駆動・制動の区間がほとんどありません")
        return out
    drive_max, brake_min = float(np.max(kappa)), float(np.min(kappa))
    both = drive_max > 0.3 and brake_min < -0.3

    def model(x, kappa=kappa, accel=accel):
        mu, lt, log_b, c = x
        fz = np.maximum(base.fz_static_n + lt * base.mass_kg * accel * 0.5, 0.0)
        return mu * fz * np.sin(c * np.arctan(math.exp(log_b) * kappa))

    # 荷重移動は駆動と制動の上限の差から決まる。片方しか限界に届いていなければ机上値のまま
    lt_lo, lt_hi = (0.0, 0.6) if both else (base.load_transfer - 1e-9, base.load_transfer + 1e-9)
    best = None
    for c0 in (1.05, 1.3, 1.6):
        for b0 in (8.0, 25.0):
            r = least_squares(lambda x: model(x) - force,
                              [base.mu, min(max(base.load_transfer, lt_lo), lt_hi), math.log(b0), c0],
                              bounds=([0.1, lt_lo, math.log(2.0), 1.0], [2.0, lt_hi, math.log(80.0), 1.9]),
                              loss="soft_l1", f_scale=0.3)
            if best is None or r.cost < best.cost:
                best = r
    mu, lt, log_b, c = best.x
    rms = float(np.sqrt(np.mean((model(best.x) - force) ** 2)))
    peak_slip = math.tan(math.pi / (2.0 * c)) / math.exp(log_b) if c > 1.0 else float("inf")
    drop = 1.0 - math.sin(c * math.pi / 2.0) if c > 1.0 else 0.0
    out.notes.append(f"μ {mu:.3f}・荷重移動 {lt:.3f}・傾き B {math.exp(log_b):.1f}・形 C {c:.2f}"
                     f"（残差 {rms:.2f}N、{len(kappa)}点、スリップ率 {brake_min:+.2f}〜{drive_max:+.2f}）")
    out.notes.append(f"前後力の上限: 駆動 約{mu * (base.fz_static_n + lt * base.mass_kg * 1.7):.1f}N・"
                     f"制動 約{mu * (base.fz_static_n - lt * base.mass_kg * 1.6):.1f}N（後輪1つ）。"
                     + (f"力が最大になるスリップ率 {peak_slip:.2f}、滑らせ切ると {drop * 100:.0f}% 落ちる"
                        if peak_slip < 2.0 else "滑らせても力がほとんど落ちない形"))
    if drive_max <= 0.3:
        out.warn(f"駆動で限界まで滑っていません（スリップ率の最大 {drive_max:.2f}）——上限と形は外挿")
    if brake_min >= -0.3:
        out.warn(f"制動で限界まで滑っていません（スリップ率の最小 {brake_min:.2f}）——上限と形は外挿")
    if np.any(rec.tc_active & rec.torque_mode) or np.any(rec.abs_active) or _limited(rec):
        out.warn("TC・ABS・片輪浮き対策が介入しています（切れていない、または片輪浮き対策が働いた）")
    if rms > 1.0:
        out.warn(f"残差が大きい（{rms:.1f}N）——車輪の慣性（Ⓐ）が合っていない可能性")
    out.values = {"mu": float(mu), "tyre_b": float(math.exp(log_b)), "tyre_c": float(c)}
    if both:
        out.values["load_transfer"] = float(lt)
    return out


# ── Ⓒ ヨーモーメント ──────────────────────────────────────────────────────

def _yaw_windows(rec: Rec, dt: float) -> list[slice]:
    """ヨーモーメントを入れている区間（前後 0.15s を含む）。"""
    on = np.nonzero(rec.test_moment != 0.0)[0]
    if len(on) == 0:
        return []
    pad = int(round(0.15 / dt))
    out, start, prev = [], int(on[0]), int(on[0])
    for i in on[1:]:
        if i - prev > pad:
            out.append(slice(max(0, start - pad), min(len(rec), prev + pad)))
            start = int(i)
        prev = int(i)
    out.append(slice(max(0, start - pad), min(len(rec), prev + pad)))
    return out


def _yaw_sim(diff_cmd: np.ndarray, v: np.ndarray, r0: float, dt: float, inertia: float, damping: float,
             gyro_tau: float, md_tau: float, md_delay: float) -> np.ndarray:
    """左右のトルク指令の差 → 記録に出るはずのヨーレート（ジャイロのローパス後）。"""
    h = dt / _SUB
    n_delay = int(round(md_delay / h))
    buf = [diff_cmd[0]] * (n_delay + 1)
    yaw = obs = r0
    diff = diff_cmd[0]
    out = np.empty(len(diff_cmd))
    k_obs = h / (gyro_tau + h)
    k_md = h / (md_tau + h)
    arm = REAR_TRACK_M * 0.5 / WHEEL_RADIUS_M
    for i in range(len(diff_cmd)):
        speed = max(abs(v[i]), 0.3)
        for _ in range(_SUB):
            buf.append(diff_cmd[i])
            diff += (buf.pop(0) - diff) * k_md
            yaw += (diff * arm - damping * (yaw - r0) / speed) / inertia * h
            obs += (yaw - obs) * k_obs
        out[i] = obs
    return out


def fit_yaw(rec: Rec, base: Plant) -> FitResult:
    out = FitResult({})
    dt = _dt(rec)
    windows = _yaw_windows(rec, dt)
    if not windows:
        out.warn("ヨーモーメントを入れた区間がありません")
        return out
    pad = int(round(0.15 / dt))
    segs = []
    for w in windows:
        r0 = float(np.mean(rec.yaw_rate[w][:max(pad - 2, 1)]))
        segs.append((rec.torque_right[w] - rec.torque_left[w], rec.speed[w], r0, rec.yaw_rate[w]))

    def resid(x):
        return np.concatenate([_yaw_sim(d, v, r0, dt, math.exp(x[0]), math.exp(x[1]), base.gyro_tau_s,
                                        base.md_tau_s, base.md_delay_s) - y
                               for d, v, r0, y in segs])

    r = least_squares(resid, [math.log(base.yaw_inertia_kgm2), math.log(base.yaw_damping)],
                      bounds=([math.log(1e-3), math.log(0.05)], [math.log(0.5), math.log(50.0)]))
    inertia, damping = math.exp(r.x[0]), math.exp(r.x[1])
    rms = float(np.sqrt(np.mean(r.fun ** 2)))
    # 対数パラメータの標準偏差（ヤコビアンから）。慣性は過渡（10〜20ms）でしか決まらず、
    # 100Hz の記録では確かさが低い
    try:
        cov = np.linalg.inv(r.jac.T @ r.jac) * (2.0 * r.cost / max(len(r.fun) - 2, 1))
        std_inertia, std_damping = math.sqrt(cov[0, 0]), math.sqrt(cov[1, 1])
    except np.linalg.LinAlgError:
        std_inertia = std_damping = float("inf")
    speeds = sorted({round(float(np.median(v)), 1) for _, v, _, _ in segs})
    gain = float(np.median(np.abs(rec.speed[np.nonzero(rec.test_moment != 0.0)[0]]))) / damping
    out.notes.append(f"ヨーの減衰 {damping:.2f}（±{std_damping * 100:.0f}%）・ヨー慣性 {inertia:.4f}kg·m²"
                     f"（±{std_inertia * 100:.0f}%）、残差 {rms:.3f}rad/s、{len(segs)}区間・速度 {speeds}m/s")
    out.notes.append(f"ヨーモーメント 0.1N·m で付くヨーレートは約 {gain * 0.1:.3f}rad/s、"
                     f"時定数 約{inertia / damping * 1e3:.0f}ms×車速[m/s]")
    out.values = {"yaw_damping": float(damping)}
    if std_inertia < 0.5:
        out.values["yaw_inertia_kgm2"] = float(inertia)
    else:
        out.notes.append("ヨー慣性は記録から決まらない（応答が記録の刻みより速い）ので机上値のまま")
    if std_damping > 0.3:
        out.warn(f"ヨーの減衰の確かさが低い（±{std_damping * 100:.0f}%）——ヨーレートの変化がノイズに埋もれています")
    peak = float(np.max(np.abs(np.concatenate([y - r0 for _, _, r0, y in segs]))))
    if peak < 0.03:
        out.warn(f"ヨーレートがほとんど変わっていません（最大 {peak:.3f}rad/s）——TV が有効か確認")
    return out


# ── まとめて ──────────────────────────────────────────────────────────────

_FNS = {"wheel": fit_wheel, "tyre": fit_tyre, "yaw": fit_yaw}


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
            r = _FNS[key](rec, plant)
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
