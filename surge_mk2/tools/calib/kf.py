"""車速カルマンフィルタ（STM32 `src/control/speed_kf.c`）の調整値を、走行の記録から最尤推定する。

    .venv/bin/python -m tools.calib.kf 記録.mcap [記録.mcap ...]          # 結果と検定を表示
    .venv/bin/python -m tools.calib.kf 記録.mcap --apply                  # vehicle.toml の [control] へ書く

## 何を推定するか

`[control]` の `kf_sigma_s_m`（走行距離の測定ノイズ）・`kf_sigma_a_m_s2` と `kf_sigma_a_per_m_s`（加速度の測定ノイズと
その車速による増え方）・`kf_jerk_psd`（加速度の変化の強さ）・`kf_bias_walk_psd`（加速度計のバイアスの変化）・
`kf_lever_arm_m`（旋回で加速度計に乗る −k·ω²）。v0.26 までは記録1本と経験で決めた固定値だった。

## どうやって（真値は要らない）

STM32 と同じ形のフィルタ（状態 [s, v, a, b]、走行距離と加速度の2つの測定）を TELEMETRY（100Hz）の
`odom_dist`・`accel[0]`・`yaw_rate`・`steer_actual` に通し、**残差の尤度** Σ(log S + y²/S) を最小にする調整値を探す
（カルマンフィルタの予測誤差分解。調整値が現実と合っていれば残差は白色で、分散は S そのもの）。STM32 と同じ
ゲート（6σ を超えた残差は測定ノイズを膨らませる）を掛け、尤度にもその分だけ上限を付ける（外れ値に引きずられない）。

- 雑音の強さ（PSD）と1回の測定のばらつき（σ）は、フィルタを回す周期に依らない量なので、100Hz の記録で求めた
  値をそのまま 2kHz（走行距離）・200Hz（加速度）の STM32 に入れてよい。ただし**白色であることが前提**。周期誤差・
  振動のように色の付いた誤差は、周期が違うと効き方が変わる。結果の残差の相関係数 ρ（隣どうし）が 0 から
  大きく外れたら、その測定の値は信用しない（`raspi/tools/kf_consistency.py` と同じ読み方）
- TELEMETRY の `odom_dist` は 0.1mm 刻み（左右平均で σ 0.02mm）。推定した走行距離の σ からこの分を除いて返す
- `kf_lever_arm_m` は旋回が十分に含まれる（ω² のばらつきが `_LEVER_MIN_OMEGA2`）ときだけ推定する

**エンコーダの校正（校正 1）を入れた後の記録で回す。** 校正前は周期誤差が走行距離の残差に色を付けて、σ が水増しされる。
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from . import toml_io

NS = 1e9


def _ranges() -> dict[str, tuple[float, float]]:
    """STM32 が受け付ける範囲（`control_params.h` の表。ファームが無ければ空）。"""
    try:
        from tools.fwsim.fw import param_table
        return {k: (v.lo, v.hi) for k, v in param_table().items()}
    except Exception:                          # noqa: BLE001
        return {}

__all__ = ["KfParams", "Series", "load_series", "nll", "replay", "estimate", "Result", "KF_KEYS", "main"]

#: 書き込むキー（`[control]`。STM32 `control_params.h` の `kf_*`）
KF_KEYS = ("kf_sigma_s_m", "kf_sigma_a_m_s2", "kf_sigma_a_per_m_s", "kf_jerk_psd", "kf_bias_walk_psd", "kf_lever_arm_m")

#: STM32 の定数（`speed_kf.h`）
_GATE_SIGMA = 6.0
_MAX_DT_S = 1.0
#: TELEMETRY の `odom_dist` の刻み [m]。左右平均の量子化の分散 = 2 × (刻み²/12) / 4
_ODOM_LSB_M = 1e-4
_QUANT_VAR = 2.0 * (_ODOM_LSB_M ** 2 / 12.0) / 4.0
#: 旋回で加速度計に乗る項を推定するのに要る ω² のばらつき [(rad/s)²]
_LEVER_MIN_OMEGA2 = 0.3
#: 残差の相関係数 ρ がこれを超えたら、白色の前提が破れている
_RHO_WARN = 0.3


@dataclass
class KfParams:
    sigma_s_m: float = 0.0008
    sigma_a_m_s2: float = 0.6
    sigma_a_per_m_s: float = 0.0
    jerk_psd: float = 100.0
    bias_walk_psd: float = 0.05
    lever_arm_m: float = 0.15

    def as_control(self) -> dict[str, float]:
        return {"kf_sigma_s_m": self.sigma_s_m, "kf_sigma_a_m_s2": self.sigma_a_m_s2,
                "kf_sigma_a_per_m_s": self.sigma_a_per_m_s, "kf_jerk_psd": self.jerk_psd,
                "kf_bias_walk_psd": self.bias_walk_psd, "kf_lever_arm_m": self.lever_arm_m}

    @classmethod
    def from_control(cls, d: dict[str, float]) -> "KfParams":
        base = cls()
        return cls(**{k[3:]: float(d.get(k, getattr(base, k[3:]))) for k in KF_KEYS})


@dataclass
class Series:
    """1本の記録の、フィルタに入れる測定（100Hz）。"""

    t: np.ndarray            #: [s]
    ds: np.ndarray           #: [m] 前のサンプルからの走行距離（左右平均×cos 舵角。STM32 と同じ射影）
    accel: np.ndarray        #: [m/s²] 加速度計の x（重力込みの生の値）
    omega2: np.ndarray       #: [(rad/s)²] ヨーレートの2乗
    speed: np.ndarray        #: [m/s] STM32 の車速（区間の判定・表示用）
    #: 記録したときに STM32 に入っていた調整値（`/diag/link` の `control_params`。無ければ空）
    active: dict[str, float] = field(default_factory=dict)


def build_series(vs_msgs: Sequence[tuple[float, dict]], active: dict[str, float] | None = None) -> Series:
    vs_msgs = sorted(vs_msgs, key=lambda x: x[0])
    t = np.array([x[0] for x in vs_msgs])
    od = np.array([0.5 * (float(x[1]["odom_dist"][0]) + float(x[1]["odom_dist"][1])) for x in vs_msgs])
    steer = np.array([float(x[1].get("steer_actual", 0.0)) for x in vs_msgs])
    ds = np.concatenate([[0.0], np.diff(od) * np.cos(steer[1:])])
    return Series(t=t - t[0], ds=ds,
                  accel=np.array([float((x[1].get("accel") or [0.0])[0]) for x in vs_msgs]),
                  omega2=np.array([float(x[1].get("yaw_rate", 0.0)) ** 2 for x in vs_msgs]),
                  speed=np.array([float(x[1].get("speed", 0.0)) for x in vs_msgs]),
                  active=dict(active or {}))


def load_series(path: str | Path) -> Series:
    from mcap.reader import make_reader

    vs: list[tuple[float, dict]] = []
    params: dict[str, float] = {}
    with open(path, "rb") as f:
        for _s, channel, message in make_reader(f).iter_messages(topics=["/vehicle_state", "/diag/link"]):
            obj = json.loads(message.data)
            if channel.topic == "/vehicle_state":
                vs.append((int(obj.get("t_capture", 0)) / NS, obj))
            else:
                params.update({k: float(v) for k, v in (obj.get("control_params") or {}).items()
                               if k in KF_KEYS and isinstance(v, (int, float))})
    if len(vs) < 100:
        raise ValueError(f"{path}: /vehicle_state が少なすぎます")
    return build_series(vs, params)


# ── フィルタ（STM32 `speed_kf.c` の写し。P は対称なので上三角の10要素で持つ）─────────────

@dataclass
class Replay:
    nll: float
    #: 正規化した残差 z（ゲート前の S で割った値）。走行距離・加速度
    z_s: np.ndarray
    z_a: np.ndarray
    v: np.ndarray
    gated: int


def replay(series: Series, p: KfParams, extra_s_var: float = _QUANT_VAR, keep: bool = False) -> Replay:
    """`series` をフィルタに通した残差の負の対数尤度（とその他）。`extra_s_var` は記録の量子化の分散。"""
    t, ds, acc, om2, spd = series.t, series.ds, series.accel, series.omega2, series.speed
    q, qb = p.jerk_psd, p.bias_walk_psd
    rs = p.sigma_s_m ** 2 + extra_s_var
    s0, s1, k_lev = p.sigma_a_m_s2, p.sigma_a_per_m_s, p.lever_arm_m
    g2 = _GATE_SIGMA * _GATE_SIGMA
    log2pi = math.log(2.0 * math.pi)
    # 状態と共分散（ResetState と同じ初期値）
    x0 = x1 = x2 = x3 = 0.0
    p00, p01, p02, p03 = p.sigma_s_m ** 2, 0.0, 0.0, 0.0
    p11, p12, p13 = 1.0, 0.0, 0.0
    p22, p23 = 10.0, 0.0
    p33 = 25.0
    s_meas = 0.0
    nll = 0.0
    gated = 0
    n = len(t)
    zs = np.zeros(n) if keep else None
    za = np.zeros(n) if keep else None
    vv = np.zeros(n) if keep else None
    for i in range(1, n):
        dt = t[i] - t[i - 1]
        if not dt > 0.0:
            continue
        if dt > _MAX_DT_S:
            x0 = x1 = x2 = x3 = 0.0
            s_meas = 0.0
            p00, p01, p02, p03, p11, p12, p13, p22, p23, p33 = p.sigma_s_m ** 2, 0, 0, 0, 1.0, 0, 0, 10.0, 0, 25.0
            continue
        # ── Predict: x' = F x、P' = F P Fᵀ + Q ──
        h2 = 0.5 * dt * dt
        x0 += x1 * dt + x2 * h2
        x1 += x2 * dt
        # F P Fᵀ（F = I + dt·E01 + h2·E02 + dt·E12）
        a00 = p00 + 2 * dt * p01 + 2 * h2 * p02 + dt * dt * p11 + 2 * dt * h2 * p12 + h2 * h2 * p22
        a01 = p01 + dt * p02 + dt * p11 + (dt * dt + h2) * p12 + dt * h2 * p22
        a02 = p02 + dt * p12 + h2 * p22
        a03 = p03 + dt * p13 + h2 * p23
        a11 = p11 + 2 * dt * p12 + dt * dt * p22
        a12 = p12 + dt * p22
        a13 = p13 + dt * p23
        dt2 = dt * dt
        dt3 = dt2 * dt
        p00 = a00 + q * dt3 * dt2 / 20.0
        p01 = a01 + q * dt2 * dt2 / 8.0
        p02 = a02 + q * dt3 / 6.0
        p03 = a03
        p11 = a11 + q * dt3 / 3.0
        p12 = a12 + q * dt2 / 2.0
        p13 = a13
        p22 = p22 + q * dt
        p33 = p33 + qb * dt
        # ── 走行距離: h = [1, 0, 0, 0] ──
        s_meas += ds[i]
        y = s_meas - x0
        hph = p00
        s_nom = hph + rs
        z2 = y * y / s_nom
        nll += math.log(s_nom) + min(z2, g2) + log2pi
        r = rs
        if z2 > g2:
            r = y * y / g2 - hph
            gated += 1
        s = hph + r
        if keep:
            zs[i] = y / math.sqrt(s_nom)
        if s > 0.0:
            k0, k1, k2, k3 = p00 / s, p01 / s, p02 / s, p03 / s
            x0 += k0 * y
            x1 += k1 * y
            x2 += k2 * y
            x3 += k3 * y
            c0, c1, c2, c3 = p00, p01, p02, p03
            p00 -= k0 * c0
            p01 -= k0 * c1
            p02 -= k0 * c2
            p03 -= k0 * c3
            p11 -= k1 * c1
            p12 -= k1 * c2
            p13 -= k1 * c3
            p22 -= k2 * c2
            p23 -= k2 * c3
            p33 -= k3 * c3
        if abs(s_meas) > 0.25:                    # SPEED_KF_REBASE_M
            x0 -= s_meas
            s_meas = 0.0
        # ── 加速度: h = [0, 0, 1, 1] ──
        z_acc = acc[i] + k_lev * om2[i]
        sig = s0 + s1 * abs(x1)
        ra = sig * sig
        y = z_acc - (x2 + x3)
        ph0, ph1, ph2, ph3 = p02 + p03, p12 + p13, p22 + p23, p23 + p33
        hph = ph2 + ph3
        s_nom = hph + ra
        z2 = y * y / s_nom
        nll += math.log(s_nom) + min(z2, g2) + log2pi
        r = ra
        if z2 > g2:
            r = y * y / g2 - hph
            gated += 1
        s = hph + r
        if keep:
            za[i] = y / math.sqrt(s_nom)
        if s > 0.0:
            k0, k1, k2, k3 = ph0 / s, ph1 / s, ph2 / s, ph3 / s
            x0 += k0 * y
            x1 += k1 * y
            x2 += k2 * y
            x3 += k3 * y
            p00 -= k0 * ph0
            p01 -= k0 * ph1
            p02 -= k0 * ph2
            p03 -= k0 * ph3
            p11 -= k1 * ph1
            p12 -= k1 * ph2
            p13 -= k1 * ph3
            p22 -= k2 * ph2
            p23 -= k2 * ph3
            p33 -= k3 * ph3
        if p00 < 1e-14:
            p00 = 1e-14
        if p11 < 1e-14:
            p11 = 1e-14
        if p22 < 1e-14:
            p22 = 1e-14
        if p33 < 1e-14:
            p33 = 1e-14
        if keep:
            vv[i] = x1
    return Replay(0.5 * nll, zs if keep else np.zeros(0), za if keep else np.zeros(0),
                  vv if keep else np.zeros(0), gated)


def nll(series: Sequence[Series], p: KfParams) -> float:
    return sum(replay(s, p).nll for s in series)


# ── 推定 ─────────────────────────────────────────────────────────────────────

@dataclass
class Result:
    params: KfParams
    values: dict[str, float]
    notes: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def _rho(z: np.ndarray) -> tuple[float, float]:
    z = z[np.abs(z) > 0]
    if z.size < 10:
        return math.nan, math.nan
    nis = float(np.mean(z * z))
    return nis, float(np.mean(z[1:] * z[:-1])) / nis


def estimate(series: Sequence[Series], start: KfParams | None = None,
             fit_lever: bool | None = None) -> Result:
    """記録（1本以上）→ 尤度を最大にする調整値。`start` は探索の初期値（既定は記録時の値か v0.26 の固定値）。"""
    series = list(series)
    if start is None:
        start = KfParams.from_control(series[0].active) if series[0].active else KfParams()
    om2 = np.concatenate([s.omega2 for s in series])
    if fit_lever is None:
        fit_lever = float(np.std(om2)) >= _LEVER_MIN_OMEGA2

    # 正の量は対数で、車速による増え方は 0 以上（√ の2乗）で探す
    def unpack(th: np.ndarray) -> KfParams:
        return KfParams(sigma_s_m=math.exp(th[0]), sigma_a_m_s2=math.exp(th[1]), sigma_a_per_m_s=th[2] ** 2,
                        jerk_psd=math.exp(th[3]), bias_walk_psd=math.exp(th[4]),
                        lever_arm_m=th[5] if fit_lever else start.lever_arm_m)

    th0 = np.array([math.log(max(start.sigma_s_m, 1e-5)), math.log(max(start.sigma_a_m_s2, 1e-2)),
                    math.sqrt(max(start.sigma_a_per_m_s, 0.0)) + 0.1, math.log(max(start.jerk_psd, 1.0)),
                    math.log(max(start.bias_walk_psd, 1e-5)), start.lever_arm_m])
    f0 = nll(series, unpack(th0))

    def obj(th: np.ndarray) -> float:
        v = nll(series, unpack(th))
        return v if math.isfinite(v) else 1e30

    # 初期の単体は各軸に「1桁の数分の1」ずつ広げる（既定の 5% では対数の軸で狭すぎる）
    simplex = np.vstack([th0, th0 + np.diag([0.5, 0.5, 0.3, 0.7, 0.7, 0.05])])
    res = minimize(obj, th0, method="Nelder-Mead",
                   options={"xatol": 1e-3, "fatol": 1e-3, "maxiter": 3000, "maxfev": 3000,
                            "initial_simplex": simplex})
    # Nelder-Mead は局所解に止まりやすいので、見つけた点から測り直す
    res = minimize(obj, res.x, method="Nelder-Mead",
                   options={"xatol": 1e-4, "fatol": 1e-4, "maxiter": 3000, "maxfev": 3000})
    p = unpack(res.x)
    # 推定した σ は記録の量子化を含まない（replay がその分を足して尤度を測った）
    out = Result(params=p, values={k: round(v, 7) for k, v in p.as_control().items()})
    if not fit_lever:
        out.values.pop("kf_lever_arm_m")
        out.notes.append(f"旋回が少ない（ω² のばらつき {float(np.std(om2)):.2f} < {_LEVER_MIN_OMEGA2}）ので kf_lever_arm_m は"
                         f"推定せず {start.lever_arm_m:g} のまま")
    n = sum(len(s.t) for s in series)
    out.notes.append(f"尤度: 初期値 {f0:.1f} → {res.fun:.1f}（{len(series)} 本・{n} サンプル。差 {f0 - res.fun:.1f}）")
    out.notes.append(
        f"推定: σs {p.sigma_s_m * 1000:.3f}mm、σa {p.sigma_a_m_s2:.3f}＋{p.sigma_a_per_m_s:.3f}·|v| m/s²、"
        f"jerk {p.jerk_psd:.1f} m²/s⁵、バイアス {p.bias_walk_psd:.4g} (m/s²)²/s、腕 {p.lever_arm_m:.3f}m")
    zs = np.concatenate([replay(s, p, keep=True).z_s for s in series])
    za = np.concatenate([replay(s, p, keep=True).z_a for s in series])
    for name, z in (("走行距離", zs), ("加速度", za)):
        nis, rho = _rho(z)
        msg = f"検定（{name}）: NIS {nis:.2f}・隣どうしの相関係数 ρ {rho:+.2f}（白色なら 1 と 0）"
        if abs(rho) > _RHO_WARN:
            msg = ("★" + msg + "。残差に色が付いている（周期誤差・振動・モデルに無い動き）。この測定の σ は"
                   "白色の前提で決めた値で、周期の違う STM32 では効き方が変わる")
            out.warnings.append(msg)
        out.notes.append(msg)
    rng = _ranges()
    for key, value in out.values.items():
        lo, hi = rng.get(key, (-float("inf"), float("inf")))
        if not lo <= value <= hi:
            out.warnings.append(f"★{key}={value:.5g} は STM32 の受け付ける範囲 {lo}〜{hi} の外")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mcap", nargs="+")
    ap.add_argument("--apply", action="store_true", help="config/vehicle.toml の [control] へ書く")
    ap.add_argument("--toml", default=str(toml_io.DEFAULT_TOML))
    a = ap.parse_args(argv)
    res = estimate([load_series(p) for p in a.mcap])
    for n in res.notes:
        print(n)
    print()
    for k, v in res.values.items():
        print(f"{k} = {v}")
    if a.apply:
        if res.warnings:
            print("\n★警告があるので書き込みません（記録を確かめてください）")
            return 1
        changed = toml_io.apply({f"control.{k}": v for k, v in res.values.items()}, a.toml)
        print(f"\n{a.toml} を更新: {', '.join(changed) or '変更なし'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
