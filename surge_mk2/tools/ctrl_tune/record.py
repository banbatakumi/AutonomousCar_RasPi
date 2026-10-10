"""同定の試験の記録（`Rec`）。実機の mcap から読むか、車両モデルで planner を回して作る。

`Rec` は TELEMETRY（100Hz）の時系列と、その時刻に効いていた指令。解析（`fit.py`）はこれだけを見る。

## 車両モデルで作る（`simulate()`）

`raspi/auto/sysid_*.py` の planner を、ホストでコンパイルしたファーム＋車両モデル
（`fw.Firmware.session()`）と閉ループで回す。planner の判断は LiDAR の周期（10Hz）、
`on_vehicle_state()` は TELEMETRY の周期（100Hz）、ファームは 2kHz。planner → ファームの
遅れは `LATENCY_S`（実機の計測で約10ms）。**真値の分かっている車で試験→解析を通して、
解析が真値を復元できるかを確かめる**ためのもの（`tools/sysid/bench.py` と同じ位置づけ）。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from raspi.msgs.types import Scan, VehicleState

from .fw import IN, MODE_BRAKE, MODE_DISARM, MODE_SPEED, MODE_TORQUE, OUT, Firmware, HostConfig
from .plant import Plant
from .scenarios import DT, SUBSTEPS

__all__ = ["Rec", "load_mcap", "simulate", "LATENCY_S"]

NS = 1e9
TELEMETRY_STEPS = 20        # 2kHz の制御周期 20 回 = TELEMETRY 1件（100Hz）
PLAN_EVERY = 10             # TELEMETRY 10件 = planner の判断1回（LiDAR 10Hz）
LATENCY_S = 0.01


@dataclass
class Rec:
    """列はすべて同じ長さ（TELEMETRY の件数）。"""

    t: np.ndarray               # [s]
    speed: np.ndarray           # STM32 の speed（前輪。ローパス後）
    odom: np.ndarray            # 前輪の累積走行距離（左右の平均）
    wheel_left: np.ndarray      # 後輪の周速 [m/s]
    wheel_right: np.ndarray
    torque_left: np.ndarray     # torque_cmd [N·m]（駆動は正、制動は負）
    torque_right: np.ndarray
    yaw_rate: np.ndarray
    steer: np.ndarray
    brake: np.ndarray           # 指令（bool）
    torque_mode: np.ndarray
    tc_active: np.ndarray
    abs_active: np.ndarray
    #: その時刻の指令が TC・ABS を切っていたか（fw_overrides）
    tc_off: np.ndarray
    abs_off: np.ndarray
    notes: list[str] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.t)


def _build(vs_rows: list[tuple[float, dict]], cmd_rows: list[tuple[float, dict]]) -> Rec:
    if not vs_rows:
        raise ValueError("記録に /vehicle_state がありません")
    vs_rows.sort(key=lambda x: x[0])
    cmd_rows.sort(key=lambda x: x[0])
    t0 = vs_rows[0][0]
    cmd_t = np.array([t for t, _ in cmd_rows]) if cmd_rows else np.zeros(0)
    cols: dict[str, list] = {k: [] for k in Rec.__dataclass_fields__ if k != "notes"}
    for t, vs in vs_rows:
        i = int(np.searchsorted(cmd_t, t, side="right")) - 1
        c = cmd_rows[i][1] if i >= 0 else {}
        ov = c.get("fw_overrides") or {}
        ws = vs.get("wheel_speed", [0.0] * 4)
        od = vs.get("odom_dist", [0.0, 0.0])
        tq = vs.get("torque_cmd", [0.0, 0.0])
        cols["t"].append(t - t0)
        cols["speed"].append(float(vs.get("speed", 0.0)))
        cols["odom"].append(0.5 * (float(od[0]) + float(od[1])))
        cols["wheel_left"].append(float(ws[2]))
        cols["wheel_right"].append(float(ws[3]))
        cols["torque_left"].append(float(tq[0]))
        cols["torque_right"].append(float(tq[1]))
        cols["yaw_rate"].append(float(vs.get("yaw_rate", 0.0)))
        cols["steer"].append(float(vs.get("steer_actual", 0.0)))
        cols["brake"].append(bool(c.get("brake", False)))
        cols["torque_mode"].append(bool(c.get("torque_mode", False)))
        cols["tc_active"].append(bool(vs.get("tc_active", False)))
        cols["abs_active"].append(bool(vs.get("abs_active", False)))
        cols["tc_off"].append(ov.get("tc_enable", 1.0) == 0.0)
        cols["abs_off"].append(ov.get("abs_enable", 1.0) == 0.0)
    return Rec(**{k: np.asarray(v) for k, v in cols.items()})


#: MCAP の先頭（マジック＋ヘッダレコード: profile ""・library "surge-mk2"）。先頭の欠けた記録を
#: 読むときに補う。ヘッダの中身は読み出しに使われない
_MCAP_HEAD = (b"\x89MCAP0\r\n" + b"\x01" + (17).to_bytes(8, "little")
              + (0).to_bytes(4, "little") + (9).to_bytes(4, "little") + b"surge-mk2")


def _read_headless(path: str | Path) -> tuple[list[tuple[float, dict]], list[tuple[float, dict]]]:
    """先頭の欠けた mcap（マジック・ヘッダと最初のスキーマ・チャンネルが無い）を読む。

    2026-10-05 まで、ブラウザが `/ws/record` を繋ぐより先に logger_node が書き出した先頭の塊が
    捨てられることがあった（`telemetry_node._mcap_pump`）。メッセージ本体は残っていて、スキーマ・
    チャンネルはファイル末尾の要約にもあるので、2回舐めれば読める（1回目でチャンネル、2回目でメッセージ）。
    """
    import io

    from mcap.records import Channel, Message
    from mcap.stream_reader import StreamReader

    data = _MCAP_HEAD + Path(path).read_bytes()
    channels = {r.id: r.topic for r in StreamReader(io.BytesIO(data)).records if isinstance(r, Channel)}
    vs_rows: list[tuple[float, dict]] = []
    cmd_rows: list[tuple[float, dict]] = []
    for r in StreamReader(io.BytesIO(data)).records:
        if not isinstance(r, Message):
            continue
        topic = channels.get(r.channel_id)
        if topic == "/vehicle_state":
            obj = json.loads(r.data)
            vs_rows.append(((obj.get("t_capture") or r.log_time) / NS, obj))
        elif topic == "/cmd":
            obj = json.loads(r.data)
            cmd_rows.append(((obj.get("t_pub") or r.log_time) / NS, obj))
    return vs_rows, cmd_rows


def load_mcap(path: str | Path) -> Rec:
    """GUI の「システム同定」タブで録った mcap → `Rec`。時刻はメッセージの中の単調時刻。"""
    from mcap.reader import make_reader

    with open(path, "rb") as f:
        headless = f.read(8) != _MCAP_HEAD[:8]
    if headless:
        vs_rows, cmd_rows = _read_headless(path)
        rec = _build(vs_rows, cmd_rows)
        rec.notes.append("記録の先頭（MCAP のヘッダ）が欠けていたので、残りから読み出しました")
        return rec
    vs_rows = []
    cmd_rows = []
    with open(path, "rb") as f:
        for _schema, channel, message in make_reader(f).iter_messages(topics=["/cmd", "/vehicle_state"]):
            obj = json.loads(message.data)
            if channel.topic == "/vehicle_state":
                vs_rows.append(((obj.get("t_capture") or message.log_time) / NS, obj))
            else:
                cmd_rows.append(((obj.get("t_pub") or message.log_time) / NS, obj))
    return _build(vs_rows, cmd_rows)


def _vehicle_state(o: np.ndarray, t_ns: int) -> VehicleState:
    flags = int(o[OUT["flags"]])
    return VehicleState(
        t_capture=t_ns, speed=float(o[OUT["fw_speed"]]), yaw_rate=float(o[OUT["fw_yaw_rate"]]),
        wheel_speed=[float(o[OUT["fw_speed"]])] * 2 + [float(o[OUT["fw_rear_left"]]),
                                                       float(o[OUT["fw_rear_right"]])],
        odom_dist=[float(o[OUT["front_odom"]])] * 2,
        torque_cmd=[float(o[OUT["cmd_left"]]), float(o[OUT["cmd_right"]])],
        tc_slip=[float(o[OUT["fw_slip_left"]]), float(o[OUT["fw_slip_right"]])],
        tc_limit_nm=[float(o[OUT["tc_limit_left"]]), float(o[OUT["tc_limit_right"]])],
        armed=True, mode=2, tc_active=bool(flags & 1), abs_active=bool(flags & 2),
        tv_active=bool(flags & 4))


def simulate(fw: Firmware, plant: Plant, planner, params: dict[str, float] | None = None,
             max_s: float = 90.0, lifted: bool = False) -> Rec:
    """`planner`（`raspi/auto` の `Planner`）を車両モデルと閉ループで回し、記録を返す。

    `lifted` は後輪を両方とも浮かせた状態（路面μをほぼ 0 にし、車体は動かない）。
    planner が「完了」と言ってから 0.3s で終える。
    """
    cfg = HostConfig(dt_s=DT, substeps=SUBSTEPS, tc_enabled=1, abs_enabled=1, wheel_lift_guard_enabled=1,
                     tv_enabled=1, imu_ready=1, initial_speed_m_s=0.0)
    sess = fw.session(plant.to_c(), cfg, params)
    planner.reset()
    set_engaged = getattr(planner, "set_engaged", None)
    on_vs = getattr(planner, "on_vehicle_state", None)
    mu = 1e-4 if lifted else 1.0
    row = np.zeros(len(IN))
    row[[IN["mode"], IN["mu_left"], IN["mu_right"], IN["front_scale"]]] = (MODE_DISARM, mu, mu, 1.0)
    row[IN["test_moment"]] = 0.0
    pending: list[tuple[int, np.ndarray, dict]] = []      # (効き始める TELEMETRY の番号, 入力, 上書き)
    applied_ov: dict[str, float] = {}
    vs_rows: list[tuple[float, dict]] = []
    cmd_rows: list[tuple[float, dict]] = []
    vs = None
    done_at = None
    lat = max(1, int(round(LATENCY_S / (DT * TELEMETRY_STEPS))))
    n_max = int(max_s / (DT * TELEMETRY_STEPS))
    for k in range(n_max):
        t = k * DT * TELEMETRY_STEPS
        if k % PLAN_EVERY == 0:
            if set_engaged is not None:
                set_engaged(True)
            st = planner.plan(Scan(seq=k // PLAN_EVERY), vs, {}, DT * TELEMETRY_STEPS * PLAN_EVERY)
            new = row.copy()
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
            pending.append((k + lat, new, ov))
            cmd_rows.append((t, {"brake": bool(new[IN["mode"]] == MODE_BRAKE),
                                 "torque_mode": bool(st.torque_mode and not st.brake),
                                 "fw_overrides": ov}))
            if st.reason.startswith("完了") or st.reason.startswith("中止"):
                done_at = done_at if done_at is not None else k
        while pending and pending[0][0] <= k:
            _, row, ov = pending.pop(0)
            if ov != applied_ov:
                sess.set_enables(tc=ov.get("tc_enable", 1.0) != 0.0, abs_=ov.get("abs_enable", 1.0) != 0.0,
                                 lift=ov.get("wheel_lift_guard_enable", 1.0) != 0.0,
                                 tv=ov.get("tv_enable", 1.0) != 0.0)
                applied_ov = ov
        out = sess.step(np.tile(row, (TELEMETRY_STEPS, 1)))
        vs = _vehicle_state(out[-1], int((t + DT * TELEMETRY_STEPS) * NS))
        if on_vs is not None:
            on_vs(vs)
        vs_rows.append((t + DT * TELEMETRY_STEPS, {
            "speed": vs.speed, "odom_dist": vs.odom_dist, "wheel_speed": vs.wheel_speed,
            "torque_cmd": vs.torque_cmd, "yaw_rate": vs.yaw_rate, "steer_actual": float(row[IN["steer"]]),
            "tc_active": vs.tc_active, "abs_active": vs.abs_active}))
        if done_at is not None and k - done_at >= 30:
            break
    sess.close()
    rec = _build(vs_rows, cmd_rows)
    if done_at is None:
        rec.notes.append("planner が完了しませんでした")
    return rec
