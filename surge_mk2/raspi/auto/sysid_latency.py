"""システム同定 — 遅延試験（`config/vehicle.toml` `[dynamics] control_latency_s`）。

**LiDARスキャンが完了してから、それに反応した指令をSTM32が受理するまで**の遅れを測る。
Pi側の処理（ScanAssembler→planning_node→auto/cmd→telemetry_nodeの中継）と
io_nodeのCOMMAND送信周期・UARTがすべて含まれる。

## なぜ要るか

学習環境（`ml_lidar/env.py`）は実ノードを通さず、スキャンが揃った瞬間に行動を
車両モデルへ渡していた（遅れ0）。実機では数十msの遅れがあり、10Hzで判断する
方策にとっては操舵の時定数より効きうる。測った値の分だけ学習環境が指令を遅らせる。

## 仕組み

停止したまま、**新しいスキャンが来るたびに**（`planning_node`は新しいスキャンが
来たときだけ`plan()`を呼ぶ）舵角の符号を反転させる。どのスキャンに反応した指令かを
解析側で取り違えないよう、舵角にスキャンの`seq`の下3bitを刻んでおく
（`amplitude + 0.002rad*(seq%8)`。0.002radはCOMMANDの分解能0.0001radより十分大きい）。
解析（`tools/sysid/fit.py`の`fit_latency()`）は、スキャンの最終セクタ受信時刻から
`steer_cmd_echo`が動き始めた時刻（STM32時刻をPi時刻へ換算済み）までを測る。

**これで測れるのは推論がほぼ0の場合の遅れ。** E2Eモデルの推論時間はこの上に乗る
（Pi 5で数ms程度）ので、`fit_latency()`にE2E走行のログを渡せばPi側の区間
（推論込み）を別に読める。

## 試験開始/中止を押すまで進まない

`TestGate`（`_sysid_common.py`）参照。
"""

from __future__ import annotations

import math

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import TestGate, settle_state
from .base import Planner

__all__ = ["SysIdLatency", "LATENCY_CODE_STEP_RAD", "LATENCY_CODE_MOD"]

#: 舵角に刻むスキャン番号の刻み [rad] と周期
LATENCY_CODE_STEP_RAD = 0.002
LATENCY_CODE_MOD = 8


class SysIdLatency(Planner):
    id = "sysid_latency"
    name = "システム同定: 遅延"
    description = "止まったまま、LiDARの1周ごとに舵を±8°で反転させる（15秒）。その場でできる"
    category = "sysid"
    stats = ()

    #: 試験の手順。**GUIからは変えられない**（`_sysid_common.py`「手順を固定した理由」）
    SETTINGS: dict[str, float] = {
        # ±8°（据え切りなのでステアMDの負担を小さく）を15s（約150回反転）
        "amplitude_deg": 8.0, "duration_s": 15.0,
    }
    params = ()

    def __init__(self) -> None:
        #: 手順（`SETTINGS`の写し。テスト・ベンチだけが差し替える）
        self.settings = dict(self.SETTINGS)
        self._gate = TestGate()
        self._t = 0.0
        self._sign = 1.0

    def reset(self) -> None:
        self._gate.reset()
        self._t = 0.0
        self._sign = 1.0

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    @staticmethod
    def encode(amplitude_rad: float, sign: float, scan_seq: int) -> float:
        """反転後の舵角。スキャン番号を`LATENCY_CODE_STEP_RAD`刻みで刻む。"""
        code = scan_seq % LATENCY_CODE_MOD
        return math.copysign(amplitude_rad + LATENCY_CODE_STEP_RAD * code, sign)

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        p = {**self.settings, **p}
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        st.target_speed = 0.0

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._t = 0.0
            self._sign = 1.0
        if not engaged:
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.reason = "ARM待ち（Enterを押してください）"
            return st

        if self._gate.settling(dt):
            return settle_state(st)

        self._t += dt
        if self._t >= p["duration_s"]:
            st.reason = "完了"
            return st

        self._sign = -self._sign
        seq = scan.seq if scan is not None else 0
        st.target_steer = self.encode(math.radians(p["amplitude_deg"]), self._sign, seq)
        st.reason = f"計測中 {self._t:.0f}/{p['duration_s']:.0f}s"
        return st
