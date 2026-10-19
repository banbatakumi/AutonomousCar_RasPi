"""真値の分かっている車で校正の試験を走らせ、実機と同じ形の記録（`Log`）を作る。

**解析が正しいかを確かめるためのもの**（`tools/calib/tests`）。車は STM32 のファーム
（`src/control` をホストでコンパイルしたもの）＋後輪・タイヤ・車体のモデル（ファーム側
`host/host_sim.c`）。planner（`raspi/auto/calib_*.py`）を実機と同じ周期で閉ループにする:

| 段 | 周期・遅れ |
|---|---|
| ファーム | 2kHz（`DT`） |
| TELEMETRY | 100Hz。planner の `on_vehicle_state()` もこの周期 |
| planner の `plan()` | 約 10Hz（`PLAN_PERIOD_S`。TELEMETRY とは位相がずれていく） |
| planner → ファーム | `LATENCY_S`（実機の測定で約 10ms）。指令はファームの周期（2kHz）の単位で効く |

ステアは理想（指令した路面舵角がそのまま出る）。ステアの校正は Python のシム（`sim/vehicle.py`）で
確かめる（`tools/calib/simsteer.py`）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields, replace

import numpy as np

from raspi.msgs.types import Scan, VehicleState
from tools.fwsim.fw import IN, MODE_BRAKE, MODE_DISARM, MODE_SPEED, MODE_TORQUE, OUT, Firmware, HostConfig, HostPlant

from .log import Log, build

__all__ = ["Car", "run", "DT", "LATENCY_S", "firmware"]

DT = 0.0005
SUBSTEPS = 20
TELEMETRY_STEPS = 20
#: planner の周期 [s]。TELEMETRY（10ms）の整数倍からわざと外す（実機の LiDAR は STM32 の時計と同期していない）
PLAN_PERIOD_S = 0.0997
LATENCY_S = 0.01
NS = 1e9

#: fw_overrides のうち機能の ON/OFF（それ以外のキーは調整値の上書き）
_ENABLE_KEYS = ("tc_enable", "abs_enable", "wheel_lift_guard_enable", "tv_enable")


@dataclass(frozen=True)
class Car:
    """シムの車の真値。名前は `host_sim.c` の HostPlant と同じ（`enc_*` 以外）。

    既定値は 2026-10 の実機の同定に近い値（真値として使うだけなので、厳密である必要はない）。"""

    mass_kg: float = 2.0
    wheel_inertia_kgm2: float = 2.4e-5
    wheel_friction_nm: float = 0.003
    wheel_damping_nms: float = 0.0
    rolling_decel_m_s2: float = 0.27
    mu: float = 0.45
    fz_static_n: float = 5.75
    load_transfer: float = 0.2
    lateral_transfer: float = 0.13
    rear_lateral_share: float = 0.59
    tyre_b: float = 15.0
    tyre_c: float = 1.1
    slip_speed_floor_m_s: float = 0.3
    md_delay_s: float = 0.0015
    md_tau_s: float = 0.0045
    md_brake_fade_rad_s: float = 20.0
    md_speed_tau_s: float = 0.002
    yaw_inertia_kgm2: float = 0.015
    yaw_damping: float = 1.6
    steer_gain: float = 1.0
    steer_gain_cubic: float = 0.0
    lateral_accel_max_m_s2: float = 4.4
    front_noise_rad_s: float = 2.2
    rear_noise_rad_s: float = 0.5
    gyro_noise_rad_s: float = 0.01
    gyro_tau_s: float = 0.0035
    gyro_bias_rad_s: float = 0.0
    front_scale_error: float = 0.0
    seed: int = 0
    #: 前輪エンコーダの真の特性（左右）。意味は `[control]` の `enc_*` と同じ（`lib/encoder_cal`）
    enc_left: tuple[float, float, float, float, float, float] = (0.0011, 0.9980, -0.005, 0.0004, 0.0026, 0.0156)
    enc_right: tuple[float, float, float, float, float, float] = (0.0012, 0.9988, 0.0235, 0.0061, -0.0295, 0.0119)
    #: エンコーダの出力 u（0〜1）のノイズの標準偏差。12bit の ADC を32回平均した値の実測（静止で約 0.022mm＝0.04°）相当
    front_adc_noise: float = 1.2e-4

    def to_c(self) -> HostPlant:
        kw = {f.name: getattr(self, f.name) for f in fields(self)
              if f.name not in ("enc_left", "enc_right", "front_adc_noise", "seed")}
        # reserved bit3: 前輪エンコーダを左右別の真の特性（enc_true）で作る
        p = HostPlant(**kw, front_ripple=0.0, seed=self.seed, reserved=8)
        for i, v in enumerate(self.enc_left + self.enc_right):
            p.enc_true[i] = v
        p.front_adc_noise = self.front_adc_noise
        return p

    def enc_params(self) -> dict[str, float]:
        """この車のエンコーダを正しく直す `[control]` の `enc_*`（テスト用の正解）。"""
        out = {}
        for side, e in (("left", self.enc_left), ("right", self.enc_right)):
            for k, v in zip(("lo", "hi", "h1s", "h1c", "h2s", "h2c"), e):
                out[f"enc_{side}_{k}"] = float(v)
        return out


_FW: Firmware | None = None


def firmware() -> Firmware:
    global _FW
    if _FW is None:
        _FW = Firmware()
    return _FW


def _vehicle_state(o: np.ndarray, t_ns: int, steer: float) -> VehicleState:
    flags = int(o[OUT["flags"]])
    return VehicleState(
        t_capture=t_ns, t_pub=t_ns, speed=float(o[OUT["fw_speed"]]), yaw_rate=float(o[OUT["fw_yaw_rate"]]),
        steer_actual=steer, steer_cmd_echo=steer,
        wheel_speed=[float(o[OUT["fw_speed"]])] * 2 + [float(o[OUT["fw_rear_left"]]), float(o[OUT["fw_rear_right"]])],
        odom_dist=[float(o[OUT["front_odom_left"]]), float(o[OUT["front_odom_right"]])],
        front_angle=[float(o[OUT["front_raw_left"]]) % (2 * math.pi), float(o[OUT["front_raw_right"]]) % (2 * math.pi)],
        torque_cmd=[float(o[OUT["cmd_left"]]), float(o[OUT["cmd_right"]])],
        tc_slip=[float(o[OUT["fw_slip_left"]]), float(o[OUT["fw_slip_right"]])],
        tc_limit_nm=[float(o[OUT["tc_limit_left"]]), float(o[OUT["tc_limit_right"]])],
        abs_limit_nm=float(o[OUT["abs_limit"]]), accel_kf=float(o[OUT["accel"]]),
        accel=[float(o[OUT["accel"]]), 0.0, 9.80665],
        md_status=[0, 0, 0], armed=True, mode=2,
        tc_active=bool(flags & 1), abs_active=bool(flags & 2), tv_active=bool(flags & 4))


@dataclass
class Truth:
    """記録と同じ時刻で並んだ真値（テストの答え合わせ用）。"""

    t: np.ndarray
    speed: np.ndarray
    distance: np.ndarray
    wheel_left: np.ndarray
    wheel_right: np.ndarray
    force_left: np.ndarray
    force_right: np.ndarray
    kappa_left: np.ndarray
    kappa_right: np.ndarray
    torque_left: np.ndarray
    torque_right: np.ndarray


def run(planner, car: Car = Car(), params: dict[str, float] | None = None, *, lifted: bool = False,
        max_s: float = 120.0, mode: str | None = None) -> tuple[Log, Truth]:
    """`planner` を `car` と閉ループで回し、記録と真値を返す。

    `params` は `[control]` の調整値の上書き（`vehicle.toml` の値の上に重ねる）。`lifted` は後輪を両方とも
    浮かせた置き方（路面の μ をほぼ 0 にする。車体は動かない）。planner が「完了」か「中止」と
    言ってから 0.5s で終える。
    """
    fw = firmware()
    base = dict(params or {})
    cfg = HostConfig(dt_s=DT, substeps=SUBSTEPS, tc_enabled=1, abs_enabled=1, wheel_lift_guard_enabled=1,
                     tv_enabled=0, imu_ready=1, initial_speed_m_s=0.0)
    sess = fw.session(car.to_c(), cfg, base)
    planner.reset()
    set_engaged = getattr(planner, "set_engaged", None)
    on_vs = getattr(planner, "on_vehicle_state", None)
    mu = 1e-4 if lifted else 1.0
    row = np.zeros(len(IN))
    row[[IN["mode"], IN["mu_left"], IN["mu_right"], IN["front_scale"]]] = (MODE_DISARM, mu, mu, 1.0)
    row[IN["test_moment"]] = float("nan")
    pending: list[tuple[int, np.ndarray, dict]] = []      # (効き始めるファームの周期の番号, 入力, 上書き)
    applied_ov: dict[str, float] = {}
    vs_rows: list[tuple[float, dict]] = []
    cmd_rows: list[tuple[float, dict]] = []
    state_rows: list[tuple[float, dict]] = []
    truth_rows: list[np.ndarray] = []
    vs = None
    done_at = None
    n_max = int(max_s / (DT * TELEMETRY_STEPS))
    period = DT * TELEMETRY_STEPS
    next_plan = 0.0
    plan_seq = 0

    def apply(ov: dict[str, float]) -> None:
        nonlocal applied_ov
        if ov == applied_ov:
            return
        en = {key: ov.get(key, 1.0 if key != "tv_enable" else 0.0) != 0.0 for key in _ENABLE_KEYS}
        sess.set_enables(tc=en["tc_enable"], abs_=en["abs_enable"], lift=en["wheel_lift_guard_enable"],
                         tv=en["tv_enable"])
        for key in set(applied_ov) | set(ov):
            if key in _ENABLE_KEYS:
                continue
            if key in ov:
                sess.set_param(key, ov[key])
            else:
                sess.set_param(key, base.get(key, fw.base_params()[key]))
        applied_ov = ov

    for k in range(n_max):
        t = k * period
        # この TELEMETRY の間に来る planner の判断（直前の TELEMETRY の車両状態で判断する）
        while next_plan < t + period - 1e-12:
            if set_engaged is not None:
                set_engaged(True)
            st = planner.plan(Scan(seq=plan_seq, t_capture=int(next_plan * NS), t_pub=int(next_plan * NS)), vs, {},
                              PLAN_PERIOD_S)
            plan_seq += 1
            new = (pending[-1][1] if pending else row).copy()
            if not st.ready or st.brake:
                new[IN["mode"]] = MODE_BRAKE
                new[IN["value"]] = st.brake_torque if (st.ready and st.brake_torque > 0) else 0.13
            elif st.torque_mode:
                new[IN["mode"]], new[IN["value"]] = MODE_TORQUE, st.target_torque
            else:
                new[IN["mode"]], new[IN["value"]] = MODE_SPEED, st.target_speed
            new[IN["accel_limit"]] = st.accel_limit
            new[IN["steer"]] = st.target_steer
            ov = dict(st.fw_overrides) if st.ready else {}
            pending.append((int(round((next_plan + LATENCY_S) / DT)), new, ov))
            cmd_rows.append((next_plan, {"target_speed": st.target_speed if not st.brake else 0.0,
                                         "target_steer": st.target_steer, "accel_limit": st.accel_limit,
                                         "brake": bool(new[IN["mode"]] == MODE_BRAKE),
                                         "brake_torque": st.brake_torque,
                                         "torque_mode": bool(st.torque_mode and st.ready and not st.brake),
                                         "target_torque": st.target_torque, "arm": True, "fw_overrides": ov}))
            state_rows.append((next_plan, {"mode": mode or getattr(planner, "id", ""), "engaged": True,
                                           "phase": st.phase, "reason": st.reason}))
            if done_at is None and (st.reason.startswith("完了") or st.reason.startswith("中止")):
                done_at = k
            next_plan += PLAN_PERIOD_S
        # ファームを TELEMETRY 1 件ぶん進める。指令が効く周期で区切る
        pos = k * TELEMETRY_STEPS
        end = pos + TELEMETRY_STEPS
        out = None
        while pos < end:
            if pending and pending[0][0] <= pos:
                _, row, ov = pending.pop(0)
                apply(ov)
                continue
            nxt = min(end, pending[0][0]) if pending else end
            out = sess.step(np.tile(row, (nxt - pos, 1)))
            pos = nxt
        t_s = t + period
        vs = _vehicle_state(out[-1], int(t_s * NS), float(row[IN["steer"]]))
        if on_vs is not None:
            on_vs(vs)
        vs_rows.append((t_s, {f: getattr(vs, f) for f in (
            "speed", "yaw_rate", "steer_actual", "steer_cmd_echo", "wheel_speed", "odom_dist", "front_angle",
            "torque_cmd", "tc_slip", "tc_limit_nm", "abs_limit_nm", "accel_kf", "accel", "md_status",
            "armed", "tc_active", "abs_active")}))
        truth_rows.append(out[-1])
        if done_at is not None and k - done_at >= 50:
            break
    sess.close()
    log = build(vs_rows, cmd_rows, state_rows, {**fw.base_params(), **base}, source="sim")
    tr = np.array(truth_rows)
    t0 = vs_rows[0][0]
    truth = Truth(t=np.array([r[0] - t0 for r in vs_rows]), speed=tr[:, OUT["speed"]], distance=tr[:, OUT["distance"]],
                  wheel_left=tr[:, OUT["wheel_left"]], wheel_right=tr[:, OUT["wheel_right"]],
                  force_left=tr[:, OUT["force_left"]], force_right=tr[:, OUT["force_right"]],
                  kappa_left=tr[:, OUT["kappa_left"]], kappa_right=tr[:, OUT["kappa_right"]],
                  torque_left=tr[:, OUT["torque_left"]], torque_right=tr[:, OUT["torque_right"]])
    if done_at is None:
        log.reasons.append("（シム: planner が完了しなかった）")
    return log, truth


def with_seed(car: Car, seed: int) -> Car:
    return replace(car, seed=seed)
