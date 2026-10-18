"""⑤ 車速PI試験（`raspi/auto/sysid_speed_pi.py`、トルク往復）の記録 → 車体の利得・転がり抵抗・トルクの遅れと、
車速PIのゲインの検討。

    .venv/bin/python -m tools.sysid.speed_pi 記録.mcap

## 何をするか

1. **車体**: トルク指令の区間（加速・揺さぶり・惰行）に、車体のモデル

       T_eff' = (T(t − τd) − T_eff) / τm,   v' = G·T_eff − rr_±

   を出力誤差法で当てはめる（T は STM32 が MD へ送ったトルクの合計 `torque_cmd`、v は前輪の走行距離の微分。
   rr は前進・後退で別）。PI を通さないので、PI のゲインを仮定しない（② 前後運動試験は PI を通した応答に当てはめる）
2. **ゲイン**: `[dynamics]` の狙い（`speed_pi_zeta`・`speed_pi_wn_rad_s`）と求めた G から極配置の式で
   `speed_kp = 2ζωn/G`・`speed_ki = ωn²/G`（`toml_update.speed_pi_values`。G を書き直すと kp・ki も書き直る）
3. **検討**: 測った遅れ（τd・τm と STM32 の制御周期）を入れた一巡伝達関数で位相余裕を出し、位相余裕
   `PM_MIN_DEG` を保てる ωn の上限を示す（狙いは方針なので書き換えない。上げるかは人が決める）

結果の `speed_plant_gain`・`rolling_resistance` は ② の値より優先する（`analyze.TESTS` で ⑤ は ② の後）。
"""

from __future__ import annotations

import argparse
import json
import math
import tomllib
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

from . import fit, toml_update

__all__ = ["Record", "build_record", "load_record", "Plant", "fit_plant", "margins", "wn_max", "analyze", "main"]

#: 位相余裕の下限 [°]（ωn の上限を決める）。45° で閉ループのステップ応答の行き過ぎが約 20%
PM_MIN_DEG = 45.0
#: STM32 の制御周期 [s]（零次ホールドで半周期の遅れ）。`drive.c` は 2kHz
CONTROL_PERIOD_S = 0.0005
#: 測ったむだ時間の不確かさ [s]。トルクが変わった時刻は TELEMETRY（10ms ごと）の間のどこかで、中点とみなすと
#: ±5ms。指令はスキャンごと（10Hz）に変わり TELEMETRY と位相がほぼ固定なので、平均しても消えない（ベンチで
#: 真の遅れ 16ms に対し 12ms と出た）。位相余裕の検討には保守側（長い側）に足して使う
DEAD_TIME_UNCERTAINTY_S = 0.005
#: 使うトルク指令の区間の最短 [s] と、これより遅い所は使わない [m/s]（止まりかけは静止摩擦が混ざる）
_SEG_MIN_S = 0.3
_SEG_MIN_SPEED = 0.08


@dataclass
class Record:
    t: np.ndarray            #: [s]
    odom: np.ndarray         #: [m] 前輪の走行距離の左右平均
    steer: np.ndarray        #: [rad]
    torque: np.ndarray       #: [N·m] 後輪の合計トルク（STM32 が MD へ送った値）
    torque_mode: np.ndarray  #: bool
    #: 記録したときの車速PIのゲイン（`/diag/link`。無ければ None）
    active_pi: tuple[float, float] | None = None


def build_record(vs_msgs: Sequence[tuple[float, dict]], cmds: Sequence[tuple[float, dict]],
                 active_pi: tuple[float, float] | None = None) -> Record:
    vs_msgs = sorted(vs_msgs, key=lambda x: x[0])
    cmds = sorted(cmds, key=lambda x: x[0])
    t0 = vs_msgs[0][0]
    ct = np.array([c[0] for c in cmds])
    n = len(vs_msgs)
    t, od, st, tq = (np.empty(n) for _ in range(4))
    tm = np.zeros(n, dtype=bool)
    for i, (ti, vs) in enumerate(vs_msgs):
        j = int(np.searchsorted(ct, ti, side="right")) - 1
        c = cmds[j][1] if j >= 0 else {}
        t[i] = ti - t0
        o = vs.get("odom_dist") or [0.0, 0.0]
        od[i] = 0.5 * (float(o[0]) + float(o[1]))
        st[i] = float(vs.get("steer_actual", 0.0))
        tc = vs.get("torque_cmd") or [0.0, 0.0]
        tq[i] = float(tc[0]) + float(tc[1])
        tm[i] = bool(c.get("torque_mode")) and not bool(c.get("brake"))
        if tm[i] and abs(tq[i]) < 1e-9 and abs(float(c.get("target_torque", 0.0))) > 1e-9:
            # STM32 が torque_cmd を送らない記録（古いファーム）は指令で代える（Pi→STM32 の遅れが τd に入る）
            tq[i] = 2.0 * float(c["target_torque"])
    return Record(t=t, odom=od, steer=st, torque=tq, torque_mode=tm, active_pi=active_pi)


def load_record(path: str | Path) -> Record:
    from mcap.reader import make_reader

    vs: list[tuple[float, dict]] = []
    cmds: list[tuple[float, dict]] = []
    params: dict[str, float] = {}
    with open(path, "rb") as f:
        for _s, channel, message in make_reader(f).iter_messages(topics=["/vehicle_state", "/cmd", "/diag/link"]):
            obj = json.loads(message.data)
            if channel.topic == "/vehicle_state":
                vs.append((int(obj.get("t_capture", 0)) / fit.NS, obj))
            elif channel.topic == "/cmd":
                cmds.append((int(obj.get("t_pub", 0)) / fit.NS, obj))
            else:
                params.update({k: float(v) for k, v in (obj.get("control_params") or {}).items()
                               if isinstance(v, (int, float))})
    pi = (params["speed_kp"], params["speed_ki"]) if "speed_kp" in params and "speed_ki" in params else None
    return build_record(vs, cmds, pi)


# ── 車体の当てはめ ──────────────────────────────────────────────────────────

@dataclass
class Plant:
    gain: float              #: G [(m/s²)/(N·m)]
    rr_fwd: float            #: [m/s²]
    rr_rev: float
    dead_s: float            #: τd
    lag_s: float             #: τm
    rms: float = 0.0         #: 残差の RMS [m/s]
    n: int = 0

    @property
    def rr(self) -> float:
        return 0.5 * (self.rr_fwd + self.rr_rev)


def segments(rec: Record) -> list[np.ndarray]:
    v = fit._odom_slope(rec.t, rec.odom) * np.cos(rec.steer)
    out = []
    m = np.concatenate([[False], rec.torque_mode & (np.abs(v) >= _SEG_MIN_SPEED), [False]])
    d = np.diff(m.astype(int))
    for a, b in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)):
        idx = np.arange(a, b)
        # 向きが変わったらそこで切る（往復の継ぎ目）
        sg = np.sign(v[idx])
        if np.any(sg != sg[0]):
            idx = idx[:np.flatnonzero(sg != sg[0])[0]]
        if idx.size >= 10 and rec.t[idx[-1]] - rec.t[idx[0]] >= _SEG_MIN_S:
            out.append(idx)
    return out


def _simulate(rec: Record, idx: np.ndarray, p: Plant, v0: float) -> np.ndarray:
    """区間 `idx` の位置（サンプル時刻）をモデルで作る。始めの車速は `v0`、トルクの遅れの状態は定常とする。

    トルクは区分的に一定（MD への指令）なので、区間ごとに厳密に解く（刻んで積分すると、むだ時間が刻みで
    量子化されて当てはめが進まない）。TELEMETRY は 10ms ごとなので、トルクが変わった時刻は前のサンプルとの
    中点とみなす（±5ms。PRBS の切り替えは TELEMETRY と無関係なので偏らない）。"""
    t = rec.t[idx]
    sgn = 1.0 if v0 >= 0 else -1.0
    rr = p.rr_fwd if sgn > 0 else p.rr_rev
    c = sgn * rr
    tau = max(p.lag_s, 1e-5)
    # 区間に効くトルクの切り替え（変わった時刻＋むだ時間）
    tq = rec.torque
    a0 = max(int(idx[0]) - 1 - int(math.ceil((p.dead_s + 0.02) / 0.005)), 1)
    k = np.arange(a0, int(idx[-1]) + 1)
    ch = k[tq[k] != tq[k - 1]]
    sw_t = 0.5 * (rec.t[ch] + rec.t[ch - 1]) + p.dead_s
    sw_u = tq[ch]
    j0 = int(np.searchsorted(sw_t, t[0], side="right")) - 1
    u = sw_u[j0] if j0 >= 0 else tq[max(a0 - 1, 0)]
    te = u
    events = sw_t[(sw_t > t[0]) & (sw_t < t[-1])]
    ev_u = sw_u[(sw_t > t[0]) & (sw_t < t[-1])]
    # サンプル時刻とトルクの切り替えを時刻順に並べて進める
    times = np.concatenate([t, events])
    kind = np.concatenate([np.arange(t.size), -1 - np.arange(events.size)])
    order = np.argsort(times, kind="stable")
    pos = np.empty(t.size)
    tc, v, s = t[0], v0, 0.0
    for o in order:
        tn = times[o]
        d = tn - tc
        if d > 0:
            e = math.exp(-d / tau)
            g = p.gain * (te - u)
            base = p.gain * u - c
            s += v * d + 0.5 * base * d * d + g * tau * (d - tau * (1.0 - e))
            v += base * d + g * tau * (1.0 - e)
            te = u + (te - u) * e
            tc = tn
        if kind[o] >= 0:
            pos[kind[o]] = s
        else:
            u = ev_u[-1 - kind[o]]
    return pos


def fit_plant(rec: Record, base_gain: float = 21.5, base_rr: float = 0.27) -> Plant:
    segs = segments(rec)
    if not segs:
        raise ValueError("トルク指令の区間がありません（⑤ 車速PI試験の記録か確かめてください）")
    v_meas = fit._odom_slope(rec.t, rec.odom) * np.cos(rec.steer)

    def unpack(x: np.ndarray) -> Plant:
        return Plant(gain=x[0], rr_fwd=x[1], rr_rev=x[2], dead_s=x[3], lag_s=x[4])

    # 区間ごとの始めの車速も未知数にする（実測の微分は区間の端で片側の窓になり、そのずれが区間じゅう残る）
    def resid(x: np.ndarray) -> np.ndarray:
        p = unpack(x)
        out = []
        for k, idx in enumerate(segs):
            s = _simulate(rec, idx, p, float(x[5 + k]))
            # 実測と同じ微分（前後の窓の直線の傾き）を掛けてから比べる
            v_sim = fit._odom_slope(rec.t[idx], s) * np.cos(rec.steer[idx])
            t = rec.t[idx]
            keep = (t >= t[0] + fit._ODOM_DIFF_HALF_S) & (t <= t[-1] - fit._ODOM_DIFF_HALF_S)
            out.append((v_sim - v_meas[idx])[keep])
        return np.concatenate(out)

    v0 = [float(v_meas[idx[0]]) for idx in segs]
    x0 = np.array([base_gain, base_rr, base_rr, 0.005, 0.01, *v0])
    lo = [1.0, 0.0, 0.0, 0.0, 1e-4] + [v - 0.3 for v in v0]
    hi = [100.0, 3.0, 3.0, 0.1, 0.3] + [v + 0.3 for v in v0]
    res = least_squares(resid, x0, bounds=(lo, hi), x_scale=[5.0, 0.1, 0.1, 0.005, 0.01] + [0.05] * len(segs),
                        diff_step=1e-3)
    p = unpack(res.x)
    p.rms = float(np.sqrt(np.mean(res.fun ** 2)))
    p.n = int(res.fun.size)
    return p


# ── ゲインの検討 ────────────────────────────────────────────────────────────

def _loop(w: np.ndarray, gain: float, kp: float, ki: float, dead_s: float, lag_s: float) -> np.ndarray:
    s = 1j * w
    return (kp + ki / s) * gain * np.exp(-s * dead_s) / (s * (1.0 + s * lag_s))


def margins(gain: float, kp: float, ki: float, dead_s: float, lag_s: float) -> tuple[float, float]:
    """一巡伝達関数（PI × 車体 G/s × 遅れ）の位相余裕 [°] と交差角周波数 [rad/s]。"""
    w = np.logspace(-1, 4, 4000)
    L = _loop(w, gain, kp, ki, dead_s, lag_s)
    mag = np.abs(L)
    i = int(np.flatnonzero(mag >= 1.0)[-1]) if np.any(mag >= 1.0) else 0
    i = min(i, w.size - 2)
    # |L| = 1 を対数で補間
    f = (math.log(mag[i]) / (math.log(mag[i]) - math.log(mag[i + 1]))) if mag[i] != mag[i + 1] else 0.0
    wc = float(math.exp(math.log(w[i]) + f * (math.log(w[i + 1]) - math.log(w[i]))))
    # 位相は各要素の和で出す（遅れが大きいと −π を越え、複素数の偏角では畳まれる）
    phase = math.atan2(-ki / wc, kp) - 0.5 * math.pi - math.atan(wc * lag_s) - wc * dead_s
    return float(180.0 + math.degrees(phase)), wc


def wn_max(gain: float, zeta: float, dead_s: float, lag_s: float, pm_min: float = PM_MIN_DEG) -> float:
    """減衰比 `zeta` のまま、位相余裕 `pm_min` を保てる ωn の上限 [rad/s]（二分法）。"""
    def pm(wn: float) -> float:
        g = toml_update.speed_pi_values(gain, zeta, wn)
        return margins(gain, g["speed_kp"], g["speed_ki"], dead_s, lag_s)[0]

    lo, hi = 0.1, 500.0
    if pm(hi) >= pm_min:
        return hi
    for _ in range(60):
        mid = math.sqrt(lo * hi)
        if pm(mid) >= pm_min:
            lo = mid
        else:
            hi = mid
    return lo


@dataclass
class Result:
    values: dict[str, float]
    plant: Plant
    notes: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def analyze(rec: Record, toml_path: str | Path = fit.DEFAULT_TOML, base_gain: float | None = None,
            base_rr: float | None = None) -> Result:
    with open(toml_path, "rb") as f:
        dyn = tomllib.load(f).get("dynamics", {})
    g0 = float(base_gain if base_gain is not None else dyn.get("speed_plant_gain", 21.5))
    rr0 = float(base_rr if base_rr is not None else dyn.get("rolling_resistance", 0.27))
    p = fit_plant(rec, g0, rr0)
    out = Result(values={"speed_plant_gain": round(p.gain, 6), "rolling_resistance": round(p.rr, 6)}, plant=p)
    out.notes.append(f"車体の利得 G {p.gain:.3f} (m/s²)/N·m（② の値 {g0:.3f} から {(p.gain / g0 - 1) * 100:+.1f}%）、"
                     f"転がり抵抗 前進 {p.rr_fwd:.3f}・後退 {p.rr_rev:.3f} m/s²（平均 {p.rr:.3f} を使う）")
    out.notes.append(f"トルクの遅れ: むだ時間 {p.dead_s * 1000:.1f}ms・1次遅れ {p.lag_s * 1000:.1f}ms"
                     f"（残差 {p.rms * 1000:.1f}mm/s、{p.n} 点。むだ時間は ±{DEAD_TIME_UNCERTAINTY_S * 1000:.0f}ms 不確か"
                     "で、位相余裕は長い側で見る）")
    if abs(p.rr_fwd - p.rr_rev) > 0.3 * max(p.rr, 0.05):
        out.notes.append("★転がり抵抗が前進と後退で3割以上違う（傾いた床か、駆動系の引きずりが向きで違う）。"
                         "平均を使うが、平らな床で録り直すこと")
    if abs(p.gain / g0 - 1.0) > 0.1:
        msg = (f"★車体の利得が ② 前後運動試験の値と 10% 以上違う（{g0:.2f} → {p.gain:.2f}）。② は PI を通した応答に、"
               "記録時の kp・ki を既知として当てはめている。記録時のゲインが toml と違わないか確かめること")
        out.warnings.append(msg)
        out.notes.append(msg)
    zeta = float(dyn.get(toml_update.PI_DESIGN_KEYS[0], 0.0) or 0.0)
    wn = float(dyn.get(toml_update.PI_DESIGN_KEYS[1], 0.0) or 0.0)
    dead = p.dead_s + 0.5 * CONTROL_PERIOD_S + DEAD_TIME_UNCERTAINTY_S
    if zeta > 0 and wn > 0:
        g = toml_update.speed_pi_values(p.gain, zeta, wn)
        pm, wc = margins(p.gain, g["speed_kp"], g["speed_ki"], dead, p.lag_s)
        top = wn_max(p.gain, zeta, dead, p.lag_s)
        out.notes.append(f"車速PI: 狙い ζ {zeta:.3f}・ωn {wn:.2f}rad/s → speed_kp {g['speed_kp']:.4f}・"
                         f"speed_ki {g['speed_ki']:.4f}（適用すると toml_update が G から書き直す）。位相余裕 {pm:.0f}°"
                         f"（交差 {wc:.1f}rad/s）")
        out.notes.append(f"ωn の上限: 位相余裕 {PM_MIN_DEG:.0f}° を保てるのは ωn ≤ {top:.1f}rad/s（ζ {zeta:.2f} のまま）。"
                         "上げると速度の追従が速くなるが、車速の雑音もトルクに乗る（kp に比例）。上げるなら "
                         "[dynamics] の speed_pi_wn_rad_s を手で書き換えて ② で確かめる")
        if pm < PM_MIN_DEG:
            msg = f"★今の狙いでは位相余裕が {pm:.0f}° しかない（{PM_MIN_DEG:.0f}° 未満）。speed_pi_wn_rad_s を下げること"
            out.warnings.append(msg)
            out.notes.append(msg)
    else:
        out.notes.append("[dynamics] に speed_pi_zeta・speed_pi_wn_rad_s が無いので、ゲインは検討しない")
    if rec.active_pi is not None:
        kp, ki = rec.active_pi
        pm, wc = margins(p.gain, kp, ki, dead, p.lag_s)
        out.notes.append(f"記録したときのゲイン kp {kp:.4f}・ki {ki:.4f} の位相余裕 {pm:.0f}°（交差 {wc:.1f}rad/s）")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mcap")
    ap.add_argument("--toml", default=str(fit.DEFAULT_TOML))
    a = ap.parse_args(argv)
    res = analyze(load_record(a.mcap), a.toml)
    for n in res.notes:
        print(n)
    print()
    for k, v in res.values.items():
        print(f"{k} = {v}")
    print("\n適用は GUI（tools/sysid/gui.py の ⑤）から。speed_kp・speed_ki は G に合わせて書き直る")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
