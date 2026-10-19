"""ステア・遅延の校正の試験を、真値の分かっている Python のシム（`sim/vehicle.py` の `VehicleModel`）で走らせる。

車体モデルの舵: 指令 → STM32 のランプ（`steer_cmd_echo`）→ サーボ（むだ時間・1次遅れ・定常ゲイン）→ 報告される
舵角 `steer_actual` → 実際に効く舵角 δ = steer_gain·d + steer_gain_cubic·d³ + steer_offset_rad（d は報告値）→
曲率 tan(δ)/L → ヨーの 2 次遅れ。実機の「STM32 のリンクの換算が実際とずれている」状態を steer_gain・cubic・offset で表す。

周期は `simcar` と同じ（1ms で車体、TELEMETRY 10ms、planner 約 10Hz、指令は `latency_s` 遅れて効く）。
"""

from __future__ import annotations

import random
from dataclasses import replace

from raspi.core.control_params import load_control_params
from raspi.msgs.types import Scan, VehicleState
from sim.vehicle import DEFAULT_SPEC_PATH, DriveInput, VehicleModel, VehicleSpec

from .log import Log, build

__all__ = ["truth_spec", "run", "LATENCY_S"]

LATENCY_S = 0.012
PLAN_PERIOD_S = 0.0997
NS = 1e9


def truth_spec(**over) -> VehicleSpec:
    """真値の車（`config/vehicle.toml` を土台に、ステアの癖を `over` で与える）。"""
    base = VehicleSpec.load(DEFAULT_SPEC_PATH)
    base = replace(base, mu=1.2, steer_servo_hysteresis_rad=0.0, steer_link_hysteresis_rad=0.0,
                   steer_link_deadband_rad=0.0, understeer_gradient=0.0, yaw_relaxation_m=0.0,
                   steer_rate_limit_rad_s=0.0, yaw_rate_filter_s=0.0)
    return replace(base, **over)


def run(planner, spec: VehicleSpec, *, seed: int = 0, latency_s: float = LATENCY_S, max_s: float = 150.0,
        yaw_noise: float = 0.01, yaw_bias: float = 0.004, steer_noise: float = 0.0005,
        steer_rate_limit: float = 7.0, params: dict[str, float] | None = None) -> Log:
    rng = random.Random(seed)
    cp = {**load_control_params(strict=False), **(params or {})}
    veh = VehicleModel(spec, (0.0, 0.0, 0.0))
    planner.reset()
    set_engaged = getattr(planner, "set_engaged", None)
    on_vs = getattr(planner, "on_vehicle_state", None)
    ms = 0.001
    vs = None
    vs_rows, cmd_rows, state_rows, scans = [], [], [], []
    pending: list[tuple[float, DriveInput]] = []
    next_plan = 0.0
    seq = 0
    done_at = None
    path: list[tuple[float, float, float]] = []
    for k in range(int(max_s / ms)):
        now = k * ms
        while pending and pending[0][0] <= now + 1e-12:
            veh.apply(pending.pop(0)[1])
        veh.step(ms)
        if k % 50 == 0:
            path.append((veh.x, veh.y, veh.yaw))
        if k % 10 == 9:
            t = now + ms
            vs = VehicleState(t_capture=int(t * NS), t_pub=int(t * NS), speed=veh.speed_measured,
                              yaw_rate=veh.yaw_rate_measured + yaw_bias + rng.gauss(0, yaw_noise),
                              steer_actual=veh.steer_actual + rng.gauss(0, steer_noise), steer_cmd_echo=veh.steer_ref,
                              odom_dist=list(veh.odom_front), wheel_speed=[veh.speed] * 4, armed=True, mode=2,
                              t_stm_us=int(t * 1e6) & 0xFFFFFFFF, md_status=[0, 0, 0])
            if on_vs is not None:
                on_vs(vs)
            vs_rows.append((t, {f: getattr(vs, f) for f in (
                "speed", "yaw_rate", "steer_actual", "steer_cmd_echo", "odom_dist", "wheel_speed", "armed",
                "t_stm_us", "md_status")}))
        if now >= next_plan - 1e-12:
            if set_engaged is not None:
                set_engaged(True)
            # スキャンは「最後のセクタを受信した時刻」= 今。planner はそれを受けて判断する
            scan = Scan(seq=seq, t_capture=int(now * NS), t_pub=int(now * NS))
            scans.append((seq, now, now))
            st = planner.plan(scan, vs, {}, PLAN_PERIOD_S)
            seq += 1
            di = DriveInput(armed=True, brake=st.brake or not st.ready, torque_mode=st.torque_mode,
                            target_speed=0.0 if (st.brake or not st.ready) else st.target_speed,
                            target_steer=st.target_steer, accel_limit=st.accel_limit or 3.0,
                            steer_rate_limit=steer_rate_limit, brake_torque=st.brake_torque,
                            target_torque=st.target_torque)
            pending.append((now + latency_s, di))
            cmd_rows.append((now, {"target_speed": di.target_speed, "target_steer": di.target_steer,
                                   "accel_limit": di.accel_limit, "steer_rate_limit": steer_rate_limit,
                                   "brake": di.brake, "brake_torque": di.brake_torque, "torque_mode": di.torque_mode,
                                   "target_torque": di.target_torque, "arm": True, "fw_overrides": {}}))
            state_rows.append((now, {"mode": getattr(planner, "id", ""), "engaged": True, "phase": st.phase,
                                     "reason": st.reason}))
            if done_at is None and (st.reason.startswith("完了") or st.reason.startswith("中止")):
                done_at = now
            next_plan += PLAN_PERIOD_S
        if done_at is not None and now - done_at > 0.5:
            break
    log = build(vs_rows, cmd_rows, state_rows, cp, scans, source="simsteer")
    if done_at is None:
        log.reasons.append("（シム: planner が完了しなかった）")
    path.append((veh.x, veh.y, veh.yaw))
    #: 真の軌跡 (x, y, yaw)（50ms ごと。原点が置いた位置）。試験が置いた位置へ戻るかの確かめ用
    log.truth_path = path
    return log
