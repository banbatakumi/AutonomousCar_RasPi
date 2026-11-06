"""実機の走行記録（mcap）でシムの車（`sim.vehicle.VehicleModel`）を確かめる。**値は求めない・書かない。**

    .venv/bin/python -m sim.log_check ~/Downloads/surge_20261102_093957.mcap
    .venv/bin/python -m sim.log_check A.mcap B.mcap --json /tmp/check.json

記録の `/cmd`（io_node へ渡った指令）を `latency` 遅らせてシムの車へ開ループで入れ、実測の TELEMETRY と比べる。
車速は `window` 秒ごとに実測へ戻す（開ループの積分のずれを溜めない。PI の積分は記録に無いので 0 から）。

出すもの:

- **舵角**: シムの `steer_actual` − 実測
- **車速**: シム − 実測（車速の帯別・横加速度の帯別。旋回で差が増えるなら旋回の抵抗が無い）
- **ヨー比**: 実測ヨー ÷ 運動学のヨー `v·tan(δ)/L`（実測の舵角・車速から）を、運動学の横加速度の帯別に。
  シムの同じ比も並べる（シムの横の飽和の形・上限が実機と合っているか）
- **摩擦円**: 横加速度 3〜6m/s² の区間のヨー比を、前後加速度の帯別に（シムは前後に使うと横が減る）
- **ヨーの遅れ**: 運動学のヨーを何 ms ずらすと実測に最も近いか（シムの曲率の2次遅れと比べる）
- **前後加速度**: 実測の p99・p1 とシムの上限

**ここで出た差からパラメータを決めない**（コースの床は校正の床と違うので、合わせ込むとシムが会場の床に化ける。
`docs/calibration.md`）。モデルを直したとき「実機に近づいたか」を同じ物差しで判定するための道具。

`/diag/link` の `fw_build_epoch` が 0 の記録（2026-11-01 の `fb65b6f` より前のファーム。前輪エンコーダの鏡像で
車速が壊れている区間がある）は警告する。
"""

from __future__ import annotations

import argparse
import json
import sys
import tomllib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from sim.vehicle import DEFAULT_SPEC_PATH, GRAVITY_MPS2, DriveInput, VehicleModel, VehicleSpec  # noqa: E402

__all__ = ["check", "firmware_build_epoch", "report"]

#: ファームは `steer_rate_limit` = 0 を既定の 0.5rad/s と読む（`sim/stm32.py` の DEFAULT_STEER_RATE_RAD_S）
_DEFAULT_STEER_RATE_RAD_S = 0.5
#: 比べる区間: ARM 中でこれより速い
MOVING_M_S = 0.5
#: ヨー比を取る最小の舵角 [rad]（中立付近は比が発散する）
MIN_STEER_RAD = 0.03
AY_BANDS = ((0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 7), (7, 20))
AX_BANDS = ((-9, -1.5), (-1.5, -0.5), (-0.5, 0.5), (0.5, 1.5), (1.5, 9))
V_BANDS = ((0.5, 1), (1, 2), (2, 3), (3, 5))


def _cmd(log, i: int) -> DriveInput:
    c = log.cmd
    rate = c["steer_rate_limit"][i]
    return DriveInput(armed=bool(c["arm"][i]), brake=bool(c["brake"][i]),
                      torque_mode=bool(c["torque_mode"][i]),
                      target_speed=float(c["target_speed"][i]), target_steer=float(c["target_steer"][i]),
                      accel_limit=float(c["accel_limit"][i]),
                      steer_rate_limit=float(rate) if rate > 0 else _DEFAULT_STEER_RATE_RAD_S,
                      brake_torque=float(c["brake_torque"][i]), target_torque=float(c["target_torque"][i]))


def replay(log, spec: VehicleSpec, control: dict, latency: float = 0.005, window: float = 2.0,
           dt: float = 0.001) -> dict[str, np.ndarray]:
    """記録の指令をシムの車へ入れ、TELEMETRY の各時刻のシムの車速・舵角・ヨーレートを返す。"""
    m = VehicleModel(spec, (0.0, 0.0, 0.0))
    m.set_speed_ff(control.get("speed_ff_nm_per_m_s2", 0.0), control.get("speed_ff_rr_nm", 0.0))
    m.set_launch_exit_torque(control.get("launch_exit_torque_nm", 0.0))
    t, ct = log.t, log.cmd_t
    n = len(t)
    out = {k: np.zeros(n) for k in ("speed", "steer", "yaw_rate")}
    ci, k, now, next_reset = -1, 0, float(t[0]), float(t[0])
    while k < n:
        if now >= next_reset:
            m.speed = float(log.vs["speed"][k])
            if m._speed_ctl is not None:
                m._speed_ctl.reset(v=m.speed, ref=m.speed)
            next_reset += window
        while ci + 1 < len(ct) and ct[ci + 1] + latency <= now:
            ci += 1
            m.apply(_cmd(log, ci))
        m.step(dt)
        now += dt
        while k < n and t[k] <= now:
            out["speed"][k], out["steer"][k], out["yaw_rate"][k] = m.speed, m.steer_actual, m.yaw_rate
            k += 1
    return out


def _err(a: np.ndarray, b: np.ndarray, mask: np.ndarray) -> dict:
    e = (a - b)[mask]
    if not len(e):
        return {"n": 0}
    return {"n": int(len(e)), "bias": float(e.mean()), "rms": float(np.sqrt((e ** 2).mean())),
            "p95_abs": float(np.percentile(np.abs(e), 95))}


def _ratio(num: np.ndarray, den: np.ndarray, mask: np.ndarray, min_n: int = 20) -> dict | None:
    if mask.sum() < min_n:
        return None
    r = num[mask] / den[mask]
    return {"n": int(mask.sum()), "med": float(np.median(r)),
            "p25": float(np.percentile(r, 25)), "p75": float(np.percentile(r, 75))}


def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    return np.convolve(x, np.ones(n) / n, mode="same") if len(x) >= n else x


def check(log, spec: VehicleSpec | None = None, control: dict | None = None, latency: float = 0.005,
          window: float = 2.0) -> dict:
    """シムと実測の差の表（`report()` が文字にする）。"""
    if spec is None:
        spec = VehicleSpec.load()
    if control is None:
        with open(DEFAULT_SPEC_PATH, "rb") as f:
            control = tomllib.load(f).get("control", {})
    sim = replay(log, spec, control, latency, window)
    vs = log.vs
    v, d, r = vs["speed"], vs["steer_actual"], vs["yaw_rate"]
    mov = (vs["armed"] > 0) & (np.abs(v) > MOVING_M_S)
    r_kin = v * np.tan(d) / spec.wheelbase
    ay_kin, ay = v * r_kin, v * r
    steer_ok = mov & (np.abs(d) > MIN_STEER_RAD)
    a_x = _smooth(np.gradient(v, log.t), 10) if len(v) > 1 else np.zeros_like(v)

    res: dict = {"duration_s": float(log.t[-1] - log.t[0]) if len(log.t) else 0.0,
                 "moving_samples": int(mov.sum()), "latency_s": latency, "window_s": window,
                 "sim_lat_limit_m_s2": spec.mu * GRAVITY_MPS2}
    res["steer"] = _err(sim["steer"], d, mov)
    res["speed"] = _err(sim["speed"], v, mov)
    res["speed_by_v"] = {f"{lo}-{hi}": _err(sim["speed"], v, mov & (np.abs(v) >= lo) & (np.abs(v) < hi))
                         for lo, hi in V_BANDS}
    res["speed_by_ay"] = {f"{lo}-{hi}": _err(sim["speed"], v, mov & (np.abs(ay) >= lo) & (np.abs(ay) < hi))
                          for lo, hi in AY_BANDS[:5]}
    res["yaw_ratio_by_ay_kin"] = {}
    for lo, hi in AY_BANDS:
        mk = steer_ok & (np.abs(ay_kin) >= lo) & (np.abs(ay_kin) < hi)
        res["yaw_ratio_by_ay_kin"][f"{lo}-{hi}"] = {"real": _ratio(r, r_kin, mk),
                                                    "sim": _ratio(sim["yaw_rate"], r_kin, mk)}
    band = steer_ok & (np.abs(ay_kin) >= 3) & (np.abs(ay_kin) < 6)
    res["yaw_ratio_by_ax"] = {f"{lo}..{hi}": {"real": _ratio(r, r_kin, band & (a_x >= lo) & (a_x < hi)),
                                              "sim": _ratio(sim["yaw_rate"], r_kin, band & (a_x >= lo) & (a_x < hi))}
                              for lo, hi in AX_BANDS}
    res["ay_real_p99"] = float(np.percentile(np.abs(ay[mov]), 99)) if mov.any() else 0.0
    res["ay_sim_p99"] = float(np.percentile(np.abs(sim["speed"] * sim["yaw_rate"])[mov], 99)) if mov.any() else 0.0
    # ヨーの遅れ: 運動学のヨー（遅れなし）を何サンプルずらすと実測・シムに最も近いか
    dt_s = float(np.median(np.diff(log.t))) if len(log.t) > 1 else 0.01
    res["yaw_lag_ms"] = {name: _best_lag(r_kin, y, mov) * dt_s * 1000
                         for name, y in (("real", r), ("sim", sim["yaw_rate"]))}
    res["ax_real_p99"] = float(np.percentile(a_x[mov], 99)) if mov.any() else 0.0
    res["ax_real_p1"] = float(np.percentile(a_x[mov], 1)) if mov.any() else 0.0
    res["sim_drive_limit_m_s2"] = spec.drive_accel_m_s2 - spec.rolling_resistance
    res["sim_decel_limit_m_s2"] = spec.speed_decel_m_s2 + spec.rolling_resistance
    res["tc_active_pct"] = float(np.mean(vs["tc_active"][mov] > 0) * 100) if mov.any() else 0.0
    res["abs_active_pct"] = float(np.mean(vs["abs_active"][mov] > 0) * 100) if mov.any() else 0.0
    return res


def _best_lag(x: np.ndarray, y: np.ndarray, mask: np.ndarray, max_lag: int = 15) -> int:
    best, best_lag = np.inf, 0
    for lag in range(0, max_lag + 1):
        a, b, m = (x[:len(x) - lag], y[lag:], mask[lag:]) if lag else (x, y, mask)
        if m.sum() < 20:
            continue
        e = float(np.mean((a[m] - b[m]) ** 2))
        if e < best:
            best, best_lag = e, lag
    return best_lag


def firmware_build_epoch(path: str | Path) -> int | None:
    """`/diag/link` の `fw_build_epoch`（0 = build_info を持たない古いファーム。読めなければ None）。"""
    from mcap.reader import make_reader
    try:
        with open(path, "rb") as f:
            for _s, _ch, msg in make_reader(f).iter_messages(topics=["/diag/link"]):
                obj = json.loads(msg.data)
                if "fw_build_epoch" in obj:
                    return int(obj.get("fw_build_epoch") or 0)
    except Exception:
        return None
    return None


def _fmt_err(e: dict) -> str:
    if not e.get("n"):
        return "—"
    return f"n={e['n']:6d} 偏り {e['bias']:+.3f} rms {e['rms']:.3f} p95|e| {e['p95_abs']:.3f}"


def _fmt_ratio(r: dict | None) -> str:
    return "—" if r is None else f"{r['med']:.2f} [{r['p25']:.2f}, {r['p75']:.2f}]"


def report(res: dict) -> str:
    """`check()` の結果を読める表にする。"""
    L = [f"  {res['duration_s']:.0f}s・走行 {res['moving_samples']} サンプル"
         f"（指令の遅れ {res['latency_s'] * 1000:.0f}ms・車速は {res['window_s']:.1f}s ごとに実測へ戻す）",
         f"  舵角 シム−実 [rad]   {_fmt_err(res['steer'])}",
         f"  車速 シム−実 [m/s]   {_fmt_err(res['speed'])}"]
    for k, e in res["speed_by_v"].items():
        L.append(f"    車速 {k:>7} m/s      {_fmt_err(e)}")
    for k, e in res["speed_by_ay"].items():
        L.append(f"    実|a_y| {k:>5} m/s²   {_fmt_err(e)}")
    L.append(f"  ヨー比（÷運動学 v·tanδ/L、中央 [p25, p75]）  シムの横の上限 {res['sim_lat_limit_m_s2']:.2f} m/s²")
    L.append("    運動学 a_y        実機                    シム")
    for k, r in res["yaw_ratio_by_ay_kin"].items():
        if r["real"] is None:
            continue
        L.append(f"    {k:>6} m/s²  n={r['real']['n']:5d} {_fmt_ratio(r['real']):>20}   {_fmt_ratio(r['sim']):>20}")
    L.append("  摩擦円: 運動学 a_y 3〜6 のヨー比を前後加速度の帯別に")
    for k, r in res["yaw_ratio_by_ax"].items():
        if r["real"] is None:
            continue
        L.append(f"    a_x {k:>10}  n={r['real']['n']:5d} {_fmt_ratio(r['real']):>20}   {_fmt_ratio(r['sim']):>20}")
    L.append(f"  |a_y| p99: 実 {res['ay_real_p99']:.2f}  シム {res['ay_sim_p99']:.2f} m/s²")
    L.append(f"  ヨーの遅れ（運動学から）: 実 {res['yaw_lag_ms']['real']:.0f}ms  シム {res['yaw_lag_ms']['sim']:.0f}ms")
    L.append(f"  前後加速度（100ms 平滑）: 実 p99 {res['ax_real_p99']:+.2f} p1 {res['ax_real_p1']:+.2f}"
             f"  シムの上限 +{res['sim_drive_limit_m_s2']:.2f} / −{res['sim_decel_limit_m_s2']:.2f} m/s²")
    L.append(f"  実機の介入: TC {res['tc_active_pct']:.1f}%  ABS {res['abs_active_pct']:.1f}%（シムは動特性なし）")
    return "\n".join(L)


def main(argv: list[str] | None = None) -> int:
    from tools.calib.log import load_mcap

    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+", help="実機の mcap")
    ap.add_argument("--latency", type=float, default=0.005, help="指令が STM32 に効くまでの遅れ [s]")
    ap.add_argument("--window", type=float, default=2.0, help="車速を実測へ戻す間隔 [s]")
    ap.add_argument("--vehicle", default=str(DEFAULT_SPEC_PATH), help="vehicle.toml")
    ap.add_argument("--json", default=None, metavar="FILE", help="結果を JSON で書く")
    args = ap.parse_args(argv)

    spec = VehicleSpec.load(args.vehicle)
    with open(args.vehicle, "rb") as f:
        control = tomllib.load(f).get("control", {})
    out = {}
    for p in args.logs:
        print(f"== {Path(p).name}")
        epoch = firmware_build_epoch(p)
        if epoch == 0:
            print("  ★ fw_build_epoch = 0（fb65b6f より前のファーム）。前輪エンコーダの鏡像で車速が壊れた区間があり得る")
        try:
            log = load_mcap(p)
        except Exception as e:  # 途中で切れた mcap（フッタ無し）など
            print(f"  読めない: {type(e).__name__}: {e}")
            continue
        res = check(log, spec, control, args.latency, args.window)
        res["fw_build_epoch"] = epoch
        out[str(p)] = res
        print(report(res))
    if args.json:
        Path(args.json).write_text(json.dumps(out, ensure_ascii=False, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
