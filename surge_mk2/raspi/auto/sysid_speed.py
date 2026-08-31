"""システム同定 — 加速試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

直進で `target_speed` を前後交互にステップ入力し、`VehicleState.speed` の
応答から `tau_speed_s` を測る。`target_speed`/`target_steer`/`brake` の
3つだけで組んであり、`planning_node.py` の `_cmd_from()` が自律走行に許して
いる自由度（速度・舵・制動の有無だけ）の範囲内で完結する。

## 前進⇄後退の交互ステップにしてある（2026-08-31、バンビの指摘）

最初の実装は「前進→停止→前進→停止…」で、停止区間は速度0で待つだけだった。
これだと車体は毎サイクル前進し続け、必要な走行スペースが繰り返し回数ぶん
伸びる。**前進⇄後退を交互に**すれば車体はほぼ同じ場所を往復するだけになり、
オープンスペースが小さくて済む。速度指令モードの一次遅れ
（`sim/vehicle.py`の`_next_speed()`）は符号に関わらず対称な式なので、
後退方向のステップも前進と同じ`tau_speed_s`を測る材料として使える
（`tools/sysid/fit.py`の`fit_speed()`も両symbolのステップを使うように
変更済み）。

## 惰行区間（`rolling_resistance`）は測らない

`rolling_resistance` は `sim/vehicle.py` の `torque_mode`／unarmed（惰行）分岐
でしか参照されないが、`sim/gym_env.py` の学習ループは常に
`armed=True, target_speed=...` でシムを駆動するため、測っても `ml_lidar` の
シム精度には反映されない（実測はしたが対象外、という判断。検討の経緯は
`docs/architecture.md` の実測手順コメントを参照）。

## 試験開始/中止を押すまでステップは進まない（2026-08-31、実機での不具合修正）

試験開始を押す前から勝手にステップが進む・試験中止を押しても止まらない・
ARM保持中に試験開始し直せない、という3つの不具合があった。`TestGate`
（`_sysid_common.py`）で`engaged`（試験開始/中止の状態）を直接ゲートに使う。
詳しい経緯はそちらのdocstring参照。
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import TestGate
from .base import ParamSpec, Planner

__all__ = ["SysIdSpeed"]


class SysIdSpeed(Planner):
    id = "sysid_speed"
    name = "システム同定: 加速"
    description = "直進でtarget_speedを前後交互にステップ入力し、tau_speed_sを測る"
    category = "sysid"
    stats = ()

    params = (
        ParamSpec(key="target_speed", label="目標速度", min=0.1, max=1.0, step=0.05,
                  default=0.5, unit="m/s",
                  note="前進・後退それぞれこの速度までステップさせる。速度によって"
                       "tau_speed_sの測定値が変わるようなら実車に未知の加減速上限が"
                       "ある可能性が高いので、複数の速度で録って見比べること"),
        ParamSpec(key="hold_s", label="1ステップの保持時間", min=0.5, max=5.0, step=0.1,
                  default=2.0, unit="s",
                  note="tau_speed_s（既定0.35s）の数倍は欲しい"),
        ParamSpec(key="cycles", label="繰り返し回数", min=1, max=8, step=1,
                  default=3, unit="回",
                  note="前進→後退を1組として繰り返す回数。往復するだけなので前進のみの"
                       "設計より必要な走行スペースが小さい"),
    )

    def __init__(self) -> None:
        self._gate = TestGate()
        self._t = 0.0
        self._step_i = 0

    def reset(self) -> None:
        self._gate.reset()
        self._t = 0.0
        self._step_i = 0

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        st.target_steer = 0.0

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._t = 0.0
            self._step_i = 0
        if not engaged:
            st.target_speed = 0.0
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.target_speed = 0.0
            st.reason = "ARM待ち（Enterを押してください）"
            return st

        n_steps = int(p["cycles"]) * 2         # 前進→後退 を1組として cycles 回

        if self._step_i >= n_steps:
            st.target_speed = 0.0
            st.reason = "完了"
            return st

        self._t += dt
        if self._t >= p["hold_s"]:
            self._t = 0.0
            self._step_i += 1
            if self._step_i >= n_steps:
                st.target_speed = 0.0
                st.reason = "完了"
                return st

        forward = self._step_i % 2 == 0
        st.target_speed = p["target_speed"] if forward else -p["target_speed"]
        phase = "前進" if forward else "後退"
        st.reason = f"{phase} {self._step_i // 2 + 1}/{int(p['cycles'])}"
        return st
