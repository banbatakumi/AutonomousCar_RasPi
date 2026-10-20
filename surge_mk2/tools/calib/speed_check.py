"""校正 5（確かめる）: `calib_speed_check` の記録 → 車速の追従の合否。設計は `docs/calibration.md` §4.5。

## 何を比べるか

1. **追従**（実測だけで決まる）: STM32 の目標車速（指令をランプさせたもの。記録の指令から同じ規則で作り直す）と
   実際の車速（前輪の走行距離を零位相で微分）の差。区間ごとに
   - ランプの間の誤差の RMS（FF が効いていれば小さい）
   - ランプが終わった後の行き過ぎ
   - 定速の終わりの残りの誤差（積分が効いていれば 0 に近い）
2. **予測との一致**: 記録の指令を、ホストでコンパイルしたファーム＋同定した車（`config/vehicle.toml`）に入れ直した
   車速（`replay`）と実測の差。外れていれば、同定した車体の値か、記録したときの STM32 のゲインが toml と違う

合否の目安（`SPEC`）は、同定した値で設計どおりのゲインのシムが余裕を持って収まる値。
"""

from __future__ import annotations

import numpy as np

from raspi.auto._calib_common import phase_parts
from raspi.msgs.convert import MAX_TARGET_TORQUE_NM

from .encoder import _stm_time
from .log import Log
from .replay import best_latency, car_from_toml
from .result import Result
from .speed import body_distance

__all__ = ["analyze", "SPEC", "ramp_reference", "measured_speed"]

#: 合否の目安 [m/s]
SPEC = {"ramp_rms": 0.06, "overshoot": 0.08, "settle": 0.03, "model_rms": 0.05}
#: 指令が STM32 に効くまでの遅れ [s]（目標車速を作り直すとき。解析は予測との一致から探した値を使う）
_DEFAULT_LATENCY_S = 0.012


def measured_speed(log: Log) -> np.ndarray:
    """前輪の走行距離（車体の前後へ射影）を ±2 サンプルの直線で微分した車速（遅れなし）。"""
    t = _stm_time(log)
    x = body_distance(log)
    v = np.gradient(x, t)
    k = np.array([-2, -1, 0, 1, 2], dtype=float)
    out = v.copy()
    for i in range(2, len(t) - 2):
        tt = t[i - 2:i + 3] - t[i]
        out[i] = float(np.polyfit(tt, x[i - 2:i + 3], 1)[0])
    del k
    return out


def ramp_reference(log: Log, latency: float, v_meas: np.ndarray) -> np.ndarray:
    """STM32 の目標車速（ランプ後）を記録の指令から作り直す（`drive.c` の `Drive_Update` と同じ規則）。

    制動・トルク直接指令の間は目標を実車速に合わせ続ける（`TrackVehicleSpeed`）。"""
    t = log.t
    ref = np.zeros_like(t)
    target = 0.0
    for i in range(len(t)):
        tc = t[i] - latency
        brake = log.cmd_at("brake", tc)[0] > 0.5
        torque = log.cmd_at("torque_mode", tc)[0] > 0.5
        if brake or torque:
            target = float(v_meas[i])
        else:
            sp = float(log.cmd_at("target_speed", tc)[0])
            acc = float(log.cmd_at("accel_limit", tc)[0]) or 3.0
            dt = t[i] - t[i - 1] if i else 0.0
            step = acc * dt
            target += max(-step, min(step, sp - target))
        ref[i] = target
    return ref


def analyze(log: Log, car=None) -> Result:
    """`car` は予測に使う車（既定は `config/vehicle.toml` の同定値から作る）。"""
    res = Result("校正 5: 車速制御（確かめる）")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）")
    v = measured_speed(log)
    names = log.phase_of_samples()
    parts = [phase_parts(n) for n in names]
    legs = sorted({p[1] for p in parts if p is not None and p[0] in ("ramp", "step", "down")})
    if len(legs) < 2:
        res.fail("車速指令の区間が見つからない（`calib_speed_check` の記録か確かめる）")
        return res
    moving = np.array([p is not None and p[0] in ("ramp", "step", "down") for p in parts])
    # 予測（同定した車＋記録したときのゲイン）。遅れは予測と実測が最も合う値
    car = car or car_from_toml()
    lat, pred = best_latency(log, car, v, moving)
    ref = ramp_reference(log, lat, v)
    model_rms = float(np.sqrt(np.mean((pred["speed"][moving] - v[moving]) ** 2)))
    kp = log.control_params.get("speed_kp", float("nan"))
    ki = log.control_params.get("speed_ki", float("nan"))
    res.info(f"記録したときのゲイン kp {kp:.3f}・ki {ki:.3f}、指令が効くまでの遅れ {lat * 1000:.0f}ms（予測との一致から）")
    worst = {"ramp_rms": 0.0, "overshoot": 0.0, "settle": 0.0}
    worst_pred = dict(worst)
    rows = []
    for leg in legs:
        idx_ramp = np.array([i for i, p in enumerate(parts) if p and p[1] == leg and p[0] == "ramp"])
        idx_step = np.array([i for i, p in enumerate(parts) if p and p[1] == leg and p[0] == "step"])
        if len(idx_ramp) < 10:
            continue
        sgn = 1.0 if parts[idx_ramp[0]][2] > 0 else -1.0
        r_ref = ref[idx_ramp] * sgn
        v1 = float(np.max(r_ref))
        ramping = r_ref < v1 - 1e-3
        hold = ~ramping
        m = {}
        mp = {}
        for vv, out in ((v * sgn, m), (pred["speed"] * sgn, mp)):
            e = vv[idx_ramp] - r_ref
            out["ramp_rms"] = float(np.sqrt(np.mean(e[ramping] ** 2))) if ramping.any() else 0.0
            out["overshoot"] = float(max(0.0, np.max(vv[idx_ramp][hold] - v1))) if hold.any() else 0.0
            tail = idx_ramp[hold][-int(0.2 / 0.01):] if hold.sum() > 25 else idx_ramp[hold]
            out["settle"] = float(abs(np.mean(vv[tail] - v1))) if len(tail) else 0.0
        for k in worst:
            worst[k] = max(worst[k], m[k])
            worst_pred[k] = max(worst_pred[k], mp[k])
        acc = float(log.cmd_at("accel_limit", log.t[idx_ramp[0]])[0])
        rows.append(f"片道{leg}（{'前進' if sgn > 0 else '後退'} {acc:.1f}m/s²→{v1:.1f}m/s）: ランプ中 {m['ramp_rms'] * 100:.1f}cm/s・"
                    f"行き過ぎ {m['overshoot'] * 100:.1f}cm/s・定速の残り {m['settle'] * 100:.1f}cm/s"
                    f"（予測 {mp['ramp_rms'] * 100:.1f}・{mp['overshoot'] * 100:.1f}・{mp['settle'] * 100:.1f}）")
        res.plot(f"片道{leg} の車速 [m/s]（横軸: 時刻 [s]）", "実測", log.t[idx_ramp], v[idx_ramp] * 1.0)
        res.plot(f"片道{leg} の車速 [m/s]（横軸: 時刻 [s]）", "目標（ランプ後）", log.t[idx_ramp], ref[idx_ramp])
        res.plot(f"片道{leg} の車速 [m/s]（横軸: 時刻 [s]）", "予測", log.t[idx_ramp], pred["speed"][idx_ramp])
        del idx_step
    for r in rows:
        res.info(r)
    tc = (log.vs["tc_limit_nm"].min(axis=1) < MAX_TARGET_TORQUE_NM - 1e-3) & moving
    if tc.sum() * 0.01 > 0.2:
        res.warn(f"TC が {tc.sum() * 0.01:.2f} 秒ぶん駆動を絞った（グリップの限界に近い。その区間の誤差は制御器ではなくタイヤによる）")
    elif tc.any():
        res.info(f"TC が {tc.sum() * 0.01:.2f} 秒だけ駆動を絞った（発進の瞬間など。短いので評価には効かない）")
    labels = {"ramp_rms": "ランプ中の誤差", "overshoot": "行き過ぎ", "settle": "定速の残りの誤差"}
    for k, lab in labels.items():
        if worst[k] > SPEC[k]:
            res.fail(f"{lab}の最大 {worst[k] * 100:.1f}cm/s が目安 {SPEC[k] * 100:.0f}cm/s を超える"
                     f"（予測 {worst_pred[k] * 100:.1f}cm/s）")
        else:
            res.ok(f"{lab}の最大 {worst[k] * 100:.1f}cm/s（目安 {SPEC[k] * 100:.0f}cm/s 以下、予測 {worst_pred[k] * 100:.1f}cm/s）")
    if model_rms > SPEC["model_rms"]:
        res.warn(f"予測（同定した車＋このゲイン）と実測の差 {model_rms * 100:.1f}cm/s が大きい"
                 f"（目安 {SPEC['model_rms'] * 100:.0f}cm/s）。校正 5 を測り直したか、STM32 に入っていたゲインが toml と同じか確かめる")
    else:
        res.ok(f"予測と実測の差 {model_rms * 100:.1f}cm/s（同定した車で説明できる）")
    return res
