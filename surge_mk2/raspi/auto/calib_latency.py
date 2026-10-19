"""校正 4: 遅延（スキャン完了 → STM32 がそれに反応した指令を受理するまで、`[dynamics] control_latency_s`）。
解析は `tools/calib/latency.py`、設計は `docs/calibration.md` §4.4。

止まったまま、**新しいスキャンが来るたびに**舵角の符号を反転する。どのスキャンに反応した指令かを解析が取り違え
ないよう、舵角にスキャンの番号の下 3 bit を刻む（`±(振幅 + 0.002rad × (seq mod 8))`。0.002rad は COMMAND の
分解能 0.0001rad より十分大きい）。Pi 側の処理（ScanAssembler → planning_node → telemetry_node の中継）・io_node の
送信・UART がすべて入る。推論（E2E モデル）の時間はこの上に乗る。

止まったまま舵を小さく振る（±8°）ので、ステアのモータへの負担は小さい。その場でできる。約 15 秒。
"""

from __future__ import annotations

import math

from ..msgs.types import AutoState, Scan, VehicleState
from ._calib_common import TestGate, settle_state
from .base import Planner

__all__ = ["CalibLatency", "CODE_STEP_RAD", "CODE_MOD", "encode"]

CODE_STEP_RAD = 0.002
CODE_MOD = 8


def encode(amplitude_rad: float, sign: float, scan_seq: int) -> float:
    """反転後の舵角。スキャン番号を `CODE_STEP_RAD` 刻みで刻む。"""
    return math.copysign(amplitude_rad + CODE_STEP_RAD * (scan_seq % CODE_MOD), sign)


class CalibLatency(Planner):
    id = "calib_latency"
    name = "校正 4: 遅延"
    description = "止まったまま、LiDARの1周ごとに舵を±8°で反転させる（15秒）。スキャンから指令が効くまでの遅れを測る。その場でできる"
    category = "sysid"
    stats = ()
    params = ()

    SETTINGS = {"amplitude_deg": 8.0, "duration_s": 15.0}

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self._gate = TestGate()
        self._t = 0.0
        self._sign = 1.0

    def reset(self) -> None:
        self._gate.reset()
        self._t = 0.0
        self._sign = 1.0

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def plan(self, scan: Scan, vs: VehicleState | None, p: dict[str, float], dt: float) -> AutoState:
        s = self.settings
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        st.target_speed = 0.0
        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._t, self._sign = 0.0, 1.0
        if not engaged:
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        if self._gate.settling(dt):
            return settle_state(st)
        self._t += dt
        if self._t >= s["duration_s"]:
            st.reason = "完了"
            st.phase = "done"
            return st
        self._sign = -self._sign
        st.target_steer = encode(math.radians(s["amplitude_deg"]), self._sign, scan.seq if scan is not None else 0)
        st.phase = "flip"
        st.reason = f"計測中 {self._t:.0f}/{s['duration_s']:.0f}s"
        return st
