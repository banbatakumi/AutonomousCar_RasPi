"""システム同定ベンチ — 真値の分かっているシムで各試験を閉ループで回し、解析が復元できるか見る。

    .venv/bin/python -m tools.sysid.bench            # 既定の真値・シード0
    .venv/bin/python -m tools.sysid.bench --seeds 3

## なぜ要るか

`tools/sysid/fit.py` は実機ログでしか確かめられておらず、**解析が正しい値を返すかを
正解と突き合わせたことが無かった**（実際、IMUの95パーセンタイルは上振れ、エコーの
ランプはシムから抜けていた）。ここでは `raspi/auto/sysid_*.py` のプランナーを
実機と同じ経路で動かし、その結果を `fit.py` に通して真値と比べる。

## 実機の経路をどこまで再現しているか

| 段 | 周期・遅れ | 実機での出どころ |
|---|---|---|
| LiDAR | 10Hz。最終セクタの受信＋1.5ms、Scan の publish まで2〜6ms | `bus_bridge.on_frame` |
| planning_node | 新しいスキャンが来たときだけ `plan()`、`auto/cmd` は新しい判断で即時＋50Hz | `planning_node._replan` |
| telemetry_node | 5msごとにバスを見て中身の変化で即時、ほか50Hz で `/cmd`（GUIのレート制限・制動トルクを載せる） | `_on_auto_cmd`・`_merge_auto` |
| io_node | 中身の変化でループ1周以内に即時、ほか100Hz で COMMAND、UART 1.2ms。`target_speed` は3.0m/s・加速度の上限は3.0で切る | `io_node` |
| STM32 | TELEMETRY 100Hz（`Pipeline.telemetry_ms`）。速度・ヨーレート・舵角にノイズとバイアス | `sim/stm32.py` と同程度 |

各周期の位相は乱数で決める。**車両は `sim/vehicle.py` そのもの**なので、ここで
復元できるのは「解析がシムのモデル構造に対して正しい」ことまで（実機がこの構造から
外れている分は、`fit.py` が出す残差で見る）。

## 必要なスペース

各試験で車体外形（`footprint`）が掃いた範囲の外接矩形を出す。
"""

from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, replace

import numpy as np

from raspi.auto.registry import PLANNERS
from raspi.msgs.types import Scan, VehicleState
from sim.stm32 import ABS_MIN_SPEED_M_S
from sim.vehicle import GRAVITY_MPS2, DriveInput, SteerServo, VehicleModel, VehicleSpec, brake_decel
from tools.sysid import fit
from tools.sysid.fit import Cmd, Log, ScanStamp, build_log

__all__ = ["TRUTH", "Pipeline", "Noise", "REALISTIC", "run_test", "run_all", "Space",
           "write_mcap", "reproduction_error"]

#: 既定の真値。`vehicle.toml` の旧手順の値とは**わざとずらしてある**（既定値に
#: たまたま一致して「復元できた」ように見えるのを避ける）
TRUTH = replace(
    VehicleSpec.load(),
    tau_steer_s=0.055, dead_time_s=0.018, steer_rate_limit_rad_s=5.0,
    # 速度はファームの PI（`SpeedController`）。Kp/Ki・出力上限・ランプはファームの定数、
    # 車体の利得・転がり抵抗・加減速の上限が同定の対象。加速の上限2.8・減速の上限2.5は
    # ファームのランプ3.0より低い＝観測できる側（観測できない真値は `test_fit` で別に見る）
    speed_kp=0.25, speed_ki=0.5, speed_torque_max_nm=0.30, speed_ramp_max_m_s2=3.0,
    speed_plant_gain=22.0, rolling_resistance=0.3,
    drive_accel_m_s2=2.8, drive_fade_speed_m_s=1.3, drive_top_speed_m_s=4.5,
    brake_decel_m_s2=3.9, speed_decel_m_s2=2.5,
    # 制動トルク→減速度の傾き（名目 33 とわざとずらす）。0.04・0.08N·m の段は直線、0.12・0.15 は
    # グリップ（3.9）で頭打ち
    brake_decel_per_nm=40.0,
    steer_servo_gain=0.98,
    mu=0.47, steer_gain=0.93, steer_offset_rad=0.012, understeer_gradient=0.025,
    steer_gain_cubic=-0.25, steer_servo_hysteresis_rad=math.radians(0.4),
    steer_link_hysteresis_rad=math.radians(0.8), steer_link_deadband_rad=math.radians(0.6),
    # STM32 の speed のローパス（ファームの定数: 1段・9.95ms。解析はオドメトリを使うので自己点検だけに効く）
    speed_filter_s=0.00995, speed_filter_order=1,
    # IMU のローパス（MPU6050 の DLPF 20Hz 設定で約8ms）とタイヤの緩和長
    yaw_rate_filter_s=0.01, yaw_relaxation_m=0.02,
    control_latency_s=0.0,
)

#: 加減速の上限が無い真値（ランプとPIだけで加速が決まる）。上限・減衰を誤って検出しないかを見る
TRUTH_NOCAP = replace(TRUTH, drive_accel_m_s2=0.0, drive_fade_speed_m_s=0.0, drive_top_speed_m_s=0.0,
                      speed_decel_m_s2=0.0)
TRUTHS = {"default": TRUTH, "nocap": TRUTH_NOCAP}

#: 比べるときの許容誤差（相対。`None` は絶対誤差 `ABS_TOL`）
REL_TOL = {
    "tau_steer_s": 0.15, "dead_time_s": None, "steer_rate_limit_rad_s": 0.15,
    "speed_plant_gain": 0.1, "rolling_resistance": None, "drive_accel_m_s2": 0.08,
    "drive_fade_speed_m_s": 0.25, "drive_top_speed_m_s": 0.25, "brake_decel_m_s2": 0.08,
    "speed_decel_m_s2": 0.1, "steer_servo_gain": None, "brake_decel_per_nm": 0.1,
    "yaw_natural_freq_rad_s": None, "yaw_damping": None,
    "mu": 0.06, "steer_gain": 0.05, "steer_offset_rad": None, "understeer_gradient": None,
    "control_latency_s": None, "steer_gain_cubic": None, "steer_servo_hysteresis_rad": None,
    "steer_link_hysteresis_rad": None, "steer_link_deadband_rad": None,
    "yaw_rate_filter_s": None, "yaw_relaxation_m": None,
}
#: mu が下限として返ったとき、これ以上の横加速度 [m/s²] を示していれば可（2.0m/s・舵30°の
#: 最小旋回でも約4.8m/s²。学習の速度域では頭打ちしない）
_MU_LOWER_BOUND_MIN = 4.5
#: 表示するが判定しない（測った範囲の外への外挿・効かなければ決まらない量。挙動
#: `speed_step@max_err`・`speed_top@max_err`・`servo_step@max_err_deg`・`yaw_resp@rms` で判定する）
_REFERENCE_ONLY = {"drive_fade_speed_m_s", "drive_top_speed_m_s", "steer_rate_limit_rad_s",
                   "steer_servo_hysteresis_rad", "drive_accel_m_s2", "speed_decel_m_s2",
                   "yaw_natural_freq_rad_s", "yaw_damping", "yaw_rate_filter_s", "yaw_relaxation_m"}
ABS_TOL = {"dead_time_s": 0.006, "steer_offset_rad": 0.004, "understeer_gradient": 0.012,
           "control_latency_s": 0.006, "steer_gain_cubic": 0.12,
           "steer_servo_hysteresis_rad": math.radians(0.15),
           "steer_link_hysteresis_rad": math.radians(0.25),
           "steer_link_deadband_rad": math.radians(0.25),
           "yaw_rate_filter_s": 0.008, "yaw_relaxation_m": 0.012,
           "rolling_resistance": 0.1, "steer_servo_gain": 0.01}


@dataclass
class Pipeline:
    scan_rx_s: float = 0.0015           # 最終点の計測 → 最終セクタ受信
    scan_proc_s: tuple = (0.002, 0.006)  # 受信 → Scan publish
    plan_s: tuple = (0.001, 0.004)       # Scan publish → plan() 完了
    uart_s: float = 0.0008               # COMMAND / TELEMETRY の片道（1Mbps で TELEMETRY 81B が 0.81ms）
    gui_brake_torque: float = 0.15       # GUI 既定（DEFAULT_BRAKE_TORQUE_NM）
    pi_max_speed: float = 3.0            # io_node --max-speed
    pi_max_accel: float = 3.0            # io_node が STM32 の LIMITS（DRIVE_MAX_ACCEL_M_S2）で切る
    #: 新しい判断を各段がその場で中継する（2026-09-26〜の実装）。False で以前の定期送信だけ
    immediate: bool = True
    bus_poll_s: float = 0.001            # telemetry_node がバスを見る周期（`BUS_POLL_S`、2026-09-27 に 5→1ms）
    #: io_node が `/cmd` に気づくまで（UART とバスを同じ poller で待つので、届けばすぐ起きる。
    #: 2026-09-27 までは UART の1バイトを最大5ms待ってからバスを見ていた）
    io_loop_s: float = 0.0005
    telemetry_ms: int = 10               # TELEMETRY の周期 [ms]（100Hz。2026-09-25 までのファームは 20）


@dataclass
class Noise:
    #: 前輪エンコーダの角度のノイズ [rad]（ADC を32スキャン平均した後、1制御周期ごと）。
    #: **`speed` とオドメトリは同じ角度から作る**（ファームと同じ: 角度の差分→ローパス＝speed、
    #: 角度の差分の積算＝odom_dist）。平均化前のノイズ±100mV（≒±124LSB）を σ≈40LSB とみて /√32
    #: ≈ 7LSB ≈ 0.01rad。★実機未確認（`sensor_check` の所見で分かる）
    enc_angle_sigma: float = 0.01
    yaw_sigma: float = 0.01
    yaw_bias: float = 0.006
    #: 後輪モータの周速のノイズ [m/s]（MD側でフィルタ済み。`SensorGuard` の照合に使う）
    rear_sigma: float = 0.02
    steer_sigma: float = 0.0015
    #: 以下は「実機らしさ」（既定0＝理想）。`REALISTIC` 参照
    drop_prob: float = 0.0          # TELEMETRY の取りこぼし
    clock_offset_s: float = 0.0     # STM32→Pi の時刻換算のずれ（t_capture が遅れる向き）


#: 実機でありそうな記録の不完全さ: 取りこぼし2%、時刻同期のずれ3ms。
#: yaw_rate のローパスとタイヤの緩和長は**真値（`VehicleSpec`）の側で持つ**。speed は
#: エンコーダの角度からファームと同じ計算で作る（ローパスの時定数は真値の `speed_filter_s`）
REALISTIC = Noise(drop_prob=0.02, clock_offset_s=0.003)


@dataclass
class Space:
    width: float
    height: float

    def __str__(self) -> str:
        return f"{self.width:.2f}m × {self.height:.2f}m"


def _scan(seq: int) -> Scan:
    return Scan(dist=[4.0] * 360, sector_seen=[True] * 12, seq=seq)


def _q(x: float, lsb: float) -> float:
    return round(x / lsb) * lsb


def run_test(planner_id: str, truth: VehicleSpec, *, params: dict | None = None,
             seed: int = 0, pipe: Pipeline | None = None, noise: Noise | None = None,
             max_s: float = 180.0, pre_s: float = 0.2) -> tuple[Log, Space, list[float]]:
    """プランナー1つを閉ループで最後まで回す → `(ログ, 掃いた範囲, 真の遅延[s]の列)`。

    `pre_s` だけ停止したまま記録してから試験を始める。ジャイロのバイアスを読む静止区間は
    プランナー自身が試験の頭に作る（`_sysid_common.SETTLE_S`）ので、ここは短くしてある。
    """
    pipe = pipe or Pipeline()
    noise = noise or Noise()
    rng = random.Random(seed)
    cls = PLANNERS[planner_id]
    planner = cls()
    on_vs = getattr(planner, "on_vehicle_state", None)
    # 手順（`SETTINGS`）は GUI から変えられないが、ベンチ・テストでは差し替えて試す
    p = {s.key: s.default for s in cls.params}
    for k, v in (params or {}).items():
        if k in getattr(planner, "settings", {}):
            planner.settings[k] = v
        else:
            p[k] = v

    veh = VehicleModel(truth, (0.0, 0.0, 0.0))
    ms = 1e-3
    # 各周期の位相 [ms]。周期はLiDAR 100・TELEMETRY 10（100Hz、ファームの RAS_TELEMETRY_INTERVAL_US）・
    # auto/cmd 20・/cmd 20・COMMAND 10
    tlm_ms = pipe.telemetry_ms
    ph_scan, ph_tlm, ph_auto, ph_pump, ph_io = (rng.randrange(n) for n in (100, tlm_ms, 20, 20, 10))

    vs_log: list[tuple[float, dict]] = []
    cmds: list[Cmd] = []
    scans: list[ScanStamp] = []
    vs_queue: list[tuple[float, VehicleState]] = []     # (Piに届く時刻, VehicleState)
    vs_latest: VehicleState | None = None
    plan_queue: list[tuple[float, int]] = []            # (plan()する時刻, スキャンseq)
    cmd_queue: list[tuple[float, DriveInput, int]] = []  # (STM32着, 入力, 反応元seq)
    latencies: list[float] = []
    scan_meas_t: dict[int, float] = {}                  # seq → 最終点を計測した時刻

    state = None          # 最新の AutoState
    state_seq = -1        # その state を作ったスキャン
    auto_cmd: tuple[dict, int] | None = None
    last_cmd: tuple[Cmd, bool, int] | None = None
    last_plan_t = None
    last_applied_steer = None
    planned = False                     # この ms に plan() したか（即時送信のきっかけ）
    relay_at = io_at = None             # 即時中継の予定時刻（telemetry_node・io_node）
    relayed_auto = None                 # telemetry_node が最後に中継した auto/cmd の中身
    sent_key = None                     # io_node が最後に送った COMMAND の中身
    seq = 0
    done_at = None
    corners = np.array(truth.footprint)
    xmin = ymin = math.inf
    xmax = ymax = -math.inf


    # 前輪エンコーダ（ファームと同じ計算。1msごと＝ファームの2kHzを1kHzで近似。ローパスの時定数は同じ）
    r_w = truth.wheel_radius
    enc_prev = 0.0                      # 前回の計測角 [rad]
    enc_accum = 0.0                     # 累積角（= odom_dist / 車輪半径）
    omega_f = 0.0                       # ローパス後の角速度
    a_spd = 1.0 - math.exp(-ms / truth.speed_filter_s) if truth.speed_filter_s > 0 else 1.0

    for k in range(int(max_s / ms)):
        now = k * ms
        started = now >= pre_s

        # ── STM32: 届いた COMMAND を受理 ──
        while cmd_queue and cmd_queue[0][0] <= now:
            _, di, src = cmd_queue.pop(0)
            if last_applied_steer is not None and abs(di.target_steer - last_applied_steer) > 0.02 \
                    and src in scan_meas_t:
                latencies.append(now - scan_meas_t[src])
            last_applied_steer = di.target_steer
            veh.apply(di)
        veh.step(ms)
        enc = (veh.odom_front[0] + veh.odom_front[1]) / 2.0 / r_w + rng.gauss(0, noise.enc_angle_sigma)
        gap = enc - enc_prev
        enc_prev = enc
        enc_accum += gap
        omega_f += (gap / ms - omega_f) * a_spd

        if k % 10 == 0 and started:                     # 掃いた範囲（10msごと）
            c, s_ = math.cos(veh.yaw), math.sin(veh.yaw)
            wx = veh.x + corners[:, 0] * c - corners[:, 1] * s_
            wy = veh.y + corners[:, 0] * s_ + corners[:, 1] * c
            xmin, xmax = min(xmin, wx.min()), max(xmax, wx.max())
            ymin, ymax = min(ymin, wy.min()), max(ymax, wy.max())

        if k % tlm_ms == ph_tlm and rng.random() >= noise.drop_prob:   # ── TELEMETRY ──
            vs_dict = {
                # STM32 の speed: 角度の差分→ローパス→車体中心線方向へ射影
                "speed": _q(omega_f * r_w * math.cos(veh.steer_actual), 0.001),
                # STM32 の yaw_rate は IMU のローパス後（真値の yaw_rate_filter_s）
                "yaw_rate": _q(veh.yaw_rate_measured + noise.yaw_bias + rng.gauss(0, noise.yaw_sigma), 0.001),
                "steer_actual": _q(veh.steer_actual + rng.gauss(0, noise.steer_sigma), 0.0001),
                "steer_cmd_echo": _q(veh.steer_ref, 0.0001),
                "odom_dist": [_q(enc_accum * r_w, 0.0001)] * 2,
                # 後輪モータの周速（前輪と独立。真の車速にノイズ）
                "wheel_speed": [0.0, 0.0] + [_q(veh.speed + rng.gauss(0, noise.rear_sigma), 0.001)] * 2,
                "accel": [veh.accel_x, 0.0, 0.0],
                # ABS（★v0.15）: 制動がグリップで頭打ちになっている間（`sim/stm32.py` と同じ判定）
                "abs_active": bool(veh.cmd.brake and abs(veh.speed) >= ABS_MIN_SPEED_M_S
                                   and brake_decel(veh.spec, veh.cmd, veh.speed)[1]),
            }
            t_cap = now + noise.clock_offset_s          # 時刻同期のずれ（Pi時刻に換算した値がずれる）
            vs_log.append((t_cap, vs_dict))
            vs_queue.append((now + pipe.uart_s, VehicleState(
                t_capture=int(round(t_cap * 1e9)),
                speed=vs_dict["speed"], yaw_rate=vs_dict["yaw_rate"],
                steer_actual=vs_dict["steer_actual"], steer_cmd_echo=vs_dict["steer_cmd_echo"],
                odom_dist=vs_dict["odom_dist"], wheel_speed=vs_dict["wheel_speed"],
                abs_active=vs_dict["abs_active"], armed=True)))
        while vs_queue and vs_queue[0][0] <= now:
            vs_latest = vs_queue.pop(0)[1]
            if on_vs is not None:                     # planning_node と同じく全サンプルを渡す
                on_vs(vs_latest)

        if k % 100 == ph_scan:                          # ── LiDAR: 1周完了 ──
            seq += 1
            t_done = now + pipe.scan_rx_s
            t_pub = t_done + rng.uniform(*pipe.scan_proc_s)
            scans.append(ScanStamp(t_done=t_done, t_pub=t_pub, seq=seq))
            scan_meas_t[seq] = now
            plan_queue.append((t_pub + rng.uniform(*pipe.plan_s), seq))

        while plan_queue and plan_queue[0][0] <= now:   # ── planning_node._replan ──
            _, sq = plan_queue.pop(0)
            planner.set_engaged(started)
            dt = (now - last_plan_t) if last_plan_t is not None else 0.0
            last_plan_t = now
            state = planner.plan(_scan(sq), vs_latest, p, dt)
            state_seq = sq
            planned = True
            if started and state.reason.startswith("完了") and done_at is None:
                done_at = now

        def auto_from_state():
            if not started:
                return ({"arm": False}, state_seq)
            if not state.ready or state.brake:
                return ({"arm": True, "brake": True, "target_speed": 0.0,
                         "target_steer": state.target_steer,
                         "brake_torque": state.brake_torque if state.ready else 0.0}, state_seq)
            return ({"arm": True, "target_speed": state.target_speed,
                     "target_steer": state.target_steer, "accel_limit": state.accel_limit}, state_seq)

        # ── auto/cmd: 50Hz の定期送信＋新しい判断はその場で（planning_node.run）──
        if state is not None and (k % 20 == ph_auto or (pipe.immediate and planned)):
            auto_cmd = auto_from_state()
            if pipe.immediate and auto_cmd[0] != relayed_auto:
                # telemetry_node はバスを `bus_poll_s` ごとに見て、中身が変わっていれば即中継
                relay_at = now + rng.uniform(0.0, pipe.bus_poll_s)
        planned = False

        relay_now = relay_at is not None and now >= relay_at
        if (k % 20 == ph_pump or relay_now) and auto_cmd is not None:  # ── telemetry_node: /cmd ──
            if relay_now:
                relay_at = None
            d, src = auto_cmd
            relayed_auto = d
            # 加速度の上限は GUI の値と planner の値の小さい方（`telemetry_node._merge_auto`）
            accel = truth.cmd_accel_limit_m_s2
            if d.get("accel_limit", 0.0) > 0:
                accel = min(accel, d["accel_limit"]) if accel > 0 else d["accel_limit"]
            # 制動トルクは planner の指定があればそれ、無ければ GUI の値（`_merge_auto`）
            brake_torque = d["brake_torque"] if d.get("brake_torque", 0.0) > 0 else pipe.gui_brake_torque
            c = Cmd(t=now, target_speed=d.get("target_speed", 0.0),
                    target_steer=d.get("target_steer", 0.0), brake=d.get("brake", False),
                    accel_limit=accel,
                    steer_rate_limit=truth.cmd_steer_rate_limit_rad_s,
                    brake_torque=brake_torque)
            cmds.append(c)
            last_cmd = (c, bool(d.get("arm", False)), src)
            key = (c.target_speed, c.target_steer, c.brake, c.accel_limit, c.brake_torque, last_cmd[1])
            if pipe.immediate and key != sent_key:
                # io_node はループ1周以内に中身の変化に気づいて即送る
                io_at = now + rng.uniform(0.0, pipe.io_loop_s)

        io_now = io_at is not None and now >= io_at
        if (k % 10 == ph_io or io_now) and last_cmd is not None:    # ── io_node: COMMAND ──
            if io_now:
                io_at = None
            c, arm, src = last_cmd
            sent_key = (c.target_speed, c.target_steer, c.brake, c.accel_limit, c.brake_torque, arm)
            v_cap = pipe.pi_max_speed
            di = DriveInput(armed=arm, brake=c.brake,
                            target_speed=max(-v_cap, min(v_cap, c.target_speed)),
                            target_steer=c.target_steer,
                            accel_limit=min(c.accel_limit, pipe.pi_max_accel) if c.accel_limit > 0
                            else pipe.pi_max_accel,
                            steer_rate_limit=c.steer_rate_limit, brake_torque=c.brake_torque)
            cmd_queue.append((now + pipe.uart_s, di, src))

        if done_at is not None and now > done_at + 1.0 and abs(veh.speed) < 1e-3:
            break

    if done_at is None:
        raise RuntimeError(f"{planner_id}: {max_s}s 以内に完了しなかった（{state.reason if state else ''}）")
    return build_log(vs_log, cmds, scans), Space(xmax - xmin, ymax - ymin), latencies


def run_all(truth: VehicleSpec = TRUTH, seed: int = 0, verbose: bool = True,
            noise: Noise | None = None, pipe: Pipeline | None = None) -> dict:
    """4試験を回して解析し、`{key: (真値, 推定値)}` と各試験のスペースを返す。"""
    base = replace(VehicleSpec.load(), steer_gain=1.0, steer_offset_rad=0.0)
    got: dict[str, float] = {}
    notes: dict[str, list[str]] = {}
    spaces: dict[str, Space] = {}
    true_latency: list[float] = []

    logs: dict[str, Log] = {}

    def step(test: str, planner_id: str, fn) -> None:
        nonlocal base
        if test not in logs:
            log, space, lat = run_test(planner_id, truth, seed=seed, noise=noise, pipe=pipe)
            logs[test] = log
            spaces[test] = space
            true_latency.extend(lat if test == "latency" else [])
        r = fn(logs[test])
        got.update(r.values)
        notes[test] = r.notes
        base = replace(base, **{k: v for k, v in r.values.items() if hasattr(base, k)})

    # GUI（`tools/sysid/gui.py`）と同じ順序。舵の効き・中立ずれ・アンダーステアは
    # 3試験の記録をまとめて最後に当てはめる
    step("steer", "sysid_steer", fit.fit_steer)
    step("accel", "sysid_accel", lambda lg: fit.fit_accel(lg, base))
    step("corner", "sysid_corner", lambda lg: fit.fit_corner(lg, base))
    step("latency", "sysid_latency", fit.fit_latency)
    r = fit.fit_geometry({k: logs[k] for k in ("accel", "steer", "corner")}, base)
    got.update(r.values)
    notes["geometry"] = r.notes

    truth_vals = {k: getattr(truth, k) for k in got}
    # 物理上限がCOMMANDのランプより遅い（=観測できる）ときだけ真値と比べる
    if truth.steer_rate_limit_rad_s >= truth.cmd_steer_rate_limit_rad_s:
        truth_vals["steer_rate_limit_rad_s"] = 0.0
    truth_vals["control_latency_s"] = float(np.median(true_latency)) if true_latency else 0.0

    out = {k: (truth_vals[k], got[k]) for k in got}
    # 速度・舵・ヨーは「挙動が一致するか」で見る。加減速の上限・物理上限などは、効かない真値
    # では決まらない（決まらなくてよい）うえ、減衰が始まる・0になる速度は2mの直線で出せる
    # 速度（約2m/s）より上の外挿なので、値そのものは判定しない
    spec_got = replace(truth, **{k: got[k] for k in _SPEED_KEYS if k in got})
    out["speed_step@max_err"] = (0.0, _speed_step_err(truth, spec_got))
    out["speed_top@max_err"] = (0.0, _speed_top_err(truth, spec_got))
    out["brake@max_err"] = (0.0, _brake_err(truth, spec_got))
    spec_got = replace(truth, **{k: got[k] for k in _BEHAVIOR_KEYS if k in got})
    out["servo_step@max_err_deg"] = (0.0, _servo_step_err(truth, spec_got))
    spec_got = replace(truth, **{k: got[k] for k in _BEHAVIOR_KEYS + _GEO_KEYS if k in got})
    out["yaw_resp@rms"] = (0.0, _yaw_resp_err(truth, spec_got))
    if verbose:
        print(f"== seed {seed} ==")
        for k, (tv, gv) in out.items():
            ok = _within(k, tv, gv)
            mark = "参考" if k in _REFERENCE_ONLY else ("OK " if ok else "NG ")
            print(f"  {mark} {k:26s} 真値 {tv:9.4f}  推定 {gv:9.4f}")
        for test, sp in spaces.items():
            print(f"  スペース {test:8s} {sp}")
        for test, ns in notes.items():
            for n in ns:
                print(f"  [{test}] {n}")
    return {"values": out, "spaces": spaces, "notes": notes}


def write_mcap(log: Log, path, t0_ns: int = 5_000_000_000) -> None:
    """ベンチの記録を、実機と同じロガー（`McapLog`）・同じトピックで mcap に書く。

    `speed` は STM32 が送る値（ローパス後、`speed_filtered`）、`odom_dist` は前輪の累積距離。
    """
    from raspi.msgs.types import DriveCmd, Scan as ScanMsg
    from raspi.rec.mcap_log import McapLog

    def ns(t: float) -> int:
        return t0_ns + int(round(t * 1e9))

    with McapLog(path, t0_mono_ns=t0_ns, t0_unix_ns=1_700_000_000_000_000_000) as w:
        for x in log.samples:
            w.write("/vehicle_state", VehicleState(
                t_capture=ns(x.t), speed=x.speed_filtered, yaw_rate=x.yaw_rate,
                steer_actual=x.steer_actual, steer_cmd_echo=x.steer_cmd_echo,
                odom_dist=[x.odom, x.odom], armed=True, tc_active=x.tc_active, abs_active=x.abs_active,
                wheel_speed=[0.0, 0.0, x.wheel_speed_rear, x.wheel_speed_rear]))
        for c in log.cmds:
            w.write("/cmd", DriveCmd(
                t_capture=ns(c.t), t_pub=ns(c.t), target_speed=c.target_speed,
                target_steer=c.target_steer, brake=c.brake, accel_limit=c.accel_limit,
                steer_rate_limit=c.steer_rate_limit, brake_torque=c.brake_torque))
        for sc in log.scans:
            w.write("/scan", ScanMsg(t_capture=ns(sc.t_done - 0.1), t_pub=ns(sc.t_pub),
                                     seq=sc.seq, sector_t_ns=[ns(sc.t_done)] * 12))


def reproduction_error(truth: VehicleSpec, model: VehicleSpec, seed: int = 0,
                       duration_s: float = 30.0) -> dict[str, float]:
    """同じ指令列を真値とモデルのシムに入れたときの応答の食い違い（RMS）。

    指令は学習中の車に近いもの: 0.1s ごとに舵の目標を最大 ±0.15rad 動かし、速度の目標を
    0.5〜2.0m/s で時々変える。COMMAND のレート制限と制御遅延も実機と同じく入れる。
    位置は発散するので比べず、入力だけで決まる応答（速度・舵角・ヨーレート）と、
    1秒ごとに揃え直したときの向きのずれを比べる。
    """
    rng = random.Random(seed)
    vt, vm = VehicleModel(truth, (0, 0, 0)), VehicleModel(model, (0, 0, 0))
    steer, speed = 0.0, 1.0
    dt = 0.005
    rows = []
    head_err = []
    yaw_t = yaw_m = 0.0
    lat_t = int(round(truth.control_latency_s / dt))
    lat_m = int(round(model.control_latency_s / dt))
    queue: list[DriveInput] = []
    for k in range(int(duration_s / dt)):
        if k % 20 == 0:                      # 0.1s ごとの判断
            steer = max(-truth.max_steer, min(truth.max_steer, steer + rng.uniform(-0.15, 0.15)))
            if rng.random() < 0.1:
                speed = rng.uniform(0.5, 2.0)
        cmd = DriveInput(armed=True, target_speed=speed, target_steer=steer,
                         accel_limit=truth.cmd_accel_limit_m_s2,
                         steer_rate_limit=truth.cmd_steer_rate_limit_rad_s)
        queue.append(cmd)
        vt.apply(queue[max(0, len(queue) - 1 - lat_t)])
        vm.apply(queue[max(0, len(queue) - 1 - lat_m)])
        vt.step(dt)
        vm.step(dt)
        yaw_t += vt.yaw_rate * dt
        yaw_m += vm.yaw_rate * dt
        if k % 200 == 199:                   # 1s ごとに向きのずれを記録して揃え直す
            head_err.append(yaw_m - yaw_t)
            yaw_t = yaw_m = 0.0
        rows.append((vt.speed - vm.speed, vt.steer_actual - vm.steer_actual,
                     vt.yaw_rate - vm.yaw_rate, vt.speed_measured - vm.speed_measured))
    a = np.array(rows)
    return {"speed_rms": float(np.sqrt(np.mean(a[:, 0] ** 2))),
            "steer_rms_deg": math.degrees(float(np.sqrt(np.mean(a[:, 1] ** 2)))),
            "yaw_rate_rms": float(np.sqrt(np.mean(a[:, 2] ** 2))),
            "speed_obs_rms": float(np.sqrt(np.mean(a[:, 3] ** 2))),
            "heading_1s_rms_deg": math.degrees(float(np.sqrt(np.mean(np.square(head_err)))))}


_SPEED_KEYS = ("speed_plant_gain", "rolling_resistance", "drive_accel_m_s2", "drive_fade_speed_m_s",
               "drive_top_speed_m_s", "speed_decel_m_s2", "brake_decel_m_s2", "brake_decel_per_nm")
_GEO_KEYS = ("steer_gain", "steer_gain_cubic", "steer_offset_rad", "understeer_gradient",
             "steer_link_hysteresis_rad", "steer_link_deadband_rad", "yaw_rate_filter_s",
             "yaw_relaxation_m", "yaw_natural_freq_rad_s", "yaw_damping")
#: 挙動の許容誤差: 速度の最大差 [m/s]、ヨーレートの RMS 差 [rad/s]
_SPEED_STEP_TOL = 0.06
#: 最高速付近（測った範囲より上）の速度の最大差 [m/s]。外挿なので段より緩い
_SPEED_TOP_TOL = 0.15
_YAW_RESP_TOL = 0.04


def _speed_step_err(truth: VehicleSpec, got: VehicleSpec) -> float:
    """測った速度域（〜2.0m/s）での速度指令の並び（段・全開加速・速度指令0の減速）を真値と
    同定値のシムに入れたときの、車速の差の最大 [m/s]。COMMAND の加速度の上限はファームと同じ3.0。"""
    seq = [(0.4, 1.0), (1.2, 1.0), (0.4, 1.0), (3.0, 0.6), (0.0, 1.5), (1.0, 1.0), (0.0, 1.0)]
    vms = [VehicleModel(sp, (0, 0, 0)) for sp in (truth, got)]
    worst = 0.0
    dt = 0.002
    for target, dur in seq:
        for vm in vms:
            vm.apply(DriveInput(armed=True, target_speed=target, accel_limit=3.0))
        for _ in range(int(dur / dt)):
            for vm in vms:
                vm.step(dt)
            worst = max(worst, abs(vms[0].speed - vms[1].speed))
    return worst


def _speed_top_err(truth: VehicleSpec, got: VehicleSpec) -> float:
    """静止から目標3.0m/sで4秒走らせたときの車速の差の最大 [m/s]。

    `_speed_step_err` は約2m/sまでしか走らないので、最高速付近の加速度の減衰（2mの直線で出せる
    速度より上への外挿）の誤検出を見逃した: 上限の無い真値で解析が約2.3m/sで0になる減衰を返し、
    それが学習・シムの車速を黙って頭打ちにするところだった（2026-09-26 の第三者検証）。
    `max_speed` を上げる計画の範囲（3.0m/s）で挙動が一致するかを見る。"""
    vms = [VehicleModel(sp, (0, 0, 0)) for sp in (truth, got)]
    for vm in vms:
        vm.apply(DriveInput(armed=True, target_speed=3.0, accel_limit=3.0))
    worst = 0.0
    dt = 0.002
    for _ in range(int(4.0 / dt)):
        for vm in vms:
            vm.step(dt)
        worst = max(worst, abs(vms[0].speed - vms[1].speed))
    return worst


#: 制動の挙動の許容誤差 [m/s]（速度の最大差）
_BRAKE_TOL = 0.06


def _brake_err(truth: VehicleSpec, got: VehicleSpec) -> float:
    """2.0m/s から制動トルク 0.03・0.06・0.10・最大で止めたときの車速の差の最大 [m/s]。
    試験の段（0.04・0.08・0.12・0.15）と違う強さでも挙動が合うか（任意の強さのブレーキ）を見る。"""
    worst = 0.0
    dt = 0.002
    for nm in (0.03, 0.06, 0.10, 0.0):
        vms = [VehicleModel(sp, (0, 0, 0)) for sp in (truth, got)]
        for vm in vms:
            vm.speed = 2.0
            vm.apply(DriveInput(armed=True, brake=True, brake_torque=nm))
        for _ in range(int(2.0 / dt)):
            for vm in vms:
                vm.step(dt)
            worst = max(worst, abs(vms[0].speed - vms[1].speed))
    return worst


def _yaw_resp_err(truth: VehicleSpec, got: VehicleSpec, seed: int = 3) -> float:
    """舵の指令の並び（1.0m/sで±15°のステップ、0.3m/sで±30°の掃引）を真値と同定値のシムに入れた
    ときの、観測されるヨーレートの差の RMS [rad/s]（舵・リンク・ヨーの遅れをまとめて見る）。"""
    rng = random.Random(seed)
    vms = [VehicleModel(sp, (0, 0, 0)) for sp in (truth, got)]
    dt = 0.002
    diffs = []
    steer = 0.0
    for k in range(int(12.0 / dt)):
        t = k * dt
        if t < 6.0:
            speed = 1.0
            if k % int(0.4 / dt) == 0:
                steer = rng.uniform(-0.26, 0.26)
        else:
            speed = 0.3
            steer = 0.5 * math.sin(2 * math.pi * (t - 6.0) / 3.0)
        for vm in vms:
            vm.apply(DriveInput(armed=True, target_speed=speed, target_steer=steer,
                                accel_limit=3.0, steer_rate_limit=7.0))
            vm.step(dt)
        if t > 1.0:
            diffs.append(vms[0].yaw_rate_measured - vms[1].yaw_rate_measured)
    return float(np.sqrt(np.mean(np.square(diffs))))


_BEHAVIOR_KEYS = ("dead_time_s", "tau_steer_s", "steer_rate_limit_rad_s", "steer_servo_hysteresis_rad",
                  "steer_servo_gain")


def _servo_step_err(truth: VehicleSpec, got: VehicleSpec) -> float:
    """COMMAND のランプ（7rad/s）を通した参照のステップ列を真値と同定値のサーボに入れたときの、
    実舵角の差の最大 [deg]。同じ舵角に左右両方から近づく並び（ヒステリシスが効く）。"""
    seq = [0.0, 0.52, -0.52, 0.1, 0.2, 0.15, -0.05, 0.05, 0.0, -0.3, 0.3]
    servos = [SteerServo(sp.dead_time_s, sp.tau_steer_s, sp.steer_rate_limit_rad_s,
                         hysteresis_rad=sp.steer_servo_hysteresis_rad, gain=sp.steer_servo_gain)
              for sp in (truth, got)]
    ref = 0.0
    dt = 0.001
    worst = 0.0
    for target in seq:
        for _ in range(600):
            ref += max(-7.0 * dt, min(7.0 * dt, target - ref))
            a, b = (sv.step(ref, dt) for sv in servos)
            worst = max(worst, abs(a - b))
    return math.degrees(worst)


def _within(key: str, truth: float, got: float) -> bool:
    if key == "servo_step@max_err_deg":
        # 参照が 7rad/s でランプしている最中は、むだ時間が 1ms ずれるだけで 0.4° 開く。
        # むだ時間 2ms（テレメトリ周期20msの1/10、学習の判断周期の1/50）のずれ＋0.2° までを可とする
        return got <= math.degrees(7.0 * 0.002) + 0.2
    if key == "mu" and got < truth and got * GRAVITY_MPS2 > _MU_LOWER_BOUND_MIN:
        # 滑り出さなかったときの下限（`fit_corner` の所見）。学習の速度域では頭打ちしない値なら可
        return True
    if key in _REFERENCE_ONLY:
        return True
    if key == "speed_step@max_err":
        return got <= _SPEED_STEP_TOL
    if key == "speed_top@max_err":
        return got <= _SPEED_TOP_TOL
    if key == "brake@max_err":
        return got <= _BRAKE_TOL
    if key == "yaw_resp@rms":
        return got <= _YAW_RESP_TOL
    rel = REL_TOL.get(key)
    if rel is None:
        return abs(got - truth) <= ABS_TOL.get(key, 0.0)
    if truth == 0.0:
        return abs(got) < 1e-9
    return abs(got - truth) <= rel * abs(truth)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--truth", choices=sorted(TRUTHS), default="default",
                    help="真値（nocap: 加減速の上限なし＝ランプとPIだけで決まる車）")
    args = ap.parse_args()
    n_bad = 0
    for sd in range(args.seeds):
        r = run_all(truth=TRUTHS[args.truth], seed=sd)
        n_bad += sum(not _within(k, tv, gv) for k, (tv, gv) in r["values"].items())
    print(f"許容誤差外: {n_bad}")
    return 1 if n_bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
