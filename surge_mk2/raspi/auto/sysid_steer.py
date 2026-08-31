"""システム同定 — ステア試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

停止したまま舵角だけをステップ入力し、`VehicleState.steer_actual`（TELEMETRY）
の応答から `tau_steer_s`・`dead_time_s`・`steer_rate_limit_rad_s`（サーボの
物理的な最大角速度、新規パラメータ）を測る。ログの解析は Mac 側の
`tools/sysid/fit.py` が行う——このプランナーはステップ入力を送るだけで、
時定数の推定はしない。

## 速度は常に0

舵の応答（むだ時間→一次遅れ→レート制限、`sim/vehicle.py` の `step()`）は
`target_speed` と無関係。**車両を走らせる必要が無い**ので、3つのシステム
同定試験の中で最もリスクが低い（台の上でも実施できる）。

## 振幅は`max_steer`まで振ってよい（2026-08-31、バンビの指摘）

`steer_rate_limit_rad_s`（サーボの物理的な最大角速度）は振幅が大きいほど
正しく測れる——小さい振幅では一次遅れの自然な速度がレート上限に届かず、
頭打ちが一度も起きないまま終わってしまう（実測: シムで振幅8°→25°まで
振ったところ、レート上限の推定値は2.14→6.00（真値）まで単調に改善した）。
`target_steer`は`plan()`で`self.vehicle.max_steer`にクランプする
（`sim/vehicle.py`の`step()`と同じ安全網）ので、ParamSpecの上限を
`max_steer`ちょうどにしても危険は増えない——他のplanner
（`follow_the_gap.py`等）が`max_steer`専用のParamSpecを持たず
`self.vehicle.max_steer`を直接参照する設計（2026-08-28全廃）と同じ考え方。

## 試験開始/中止を押すまでステップは進まない（2026-08-31、実機での不具合修正）

試験開始を押す前から勝手にステップが進む・試験中止を押しても止まらない・
ARM保持中に試験開始し直せない、という3つの不具合があった。`TestGate`
（`_sysid_common.py`）で`engaged`（試験開始/中止の状態）を直接ゲートに使う。
詳しい経緯はそちらのdocstring参照。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import TestGate
from .base import ParamSpec, Planner

__all__ = ["SysIdSteer"]


class SysIdSteer(Planner):
    id = "sysid_steer"
    name = "システム同定: ステア"
    description = "停止したまま舵角をステップ入力し、tau_steer_s・dead_time_s・ステアレート上限を測る"
    category = "sysid"
    stats = ()

    params = (
        ParamSpec(key="amplitude_deg", label="振幅", min=5.0, max=30.0, step=1.0,
                  default=30.0, unit="°",
                  note="±この角度でステップ入力する（実際の舵角は車両物理限界のmax_steerで"
                       "頭打ちにする）。steer_rate_limit_rad_sは振幅が大きいほど正しく測れるので、"
                       "上限いっぱい(30°)まで振るのが望ましい。小さめの振幅(8〜10°程度)でも"
                       "一度録っておくとtau_steer_s/dead_time_sの確認になる"),
        ParamSpec(key="hold_s", label="保持時間", min=0.5, max=5.0, step=0.1,
                  default=1.5, unit="s",
                  note="1ステップぶんの保持時間。tau_steer_s（既定0.12s）の数十倍は欲しい"),
        ParamSpec(key="cycles", label="繰り返し回数", min=1, max=10, step=1,
                  default=4, unit="回",
                  note="+振幅→-振幅を1往復として繰り返す回数。多いほどフィッティングが安定する"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
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
        st.target_speed = 0.0

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._t = 0.0
            self._step_i = 0
        if not engaged:
            st.target_steer = 0.0
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.target_steer = 0.0
            st.reason = "ARM待ち（Enterを押してください）"
            return st

        self._t += dt
        hold = p["hold_s"]
        n_steps = int(p["cycles"]) * 2 + 1     # 中立→+A→-A→+A→-A→...

        if self._step_i >= n_steps:
            st.target_steer = 0.0
            st.reason = "完了"
            return st

        if self._t >= hold:
            self._t = 0.0
            self._step_i += 1
            if self._step_i >= n_steps:
                st.target_steer = 0.0
                st.reason = "完了"
                return st

        amp = min(math.radians(p["amplitude_deg"]), self.vehicle.max_steer)
        if self._step_i == 0:
            target = 0.0
        else:
            target = amp if self._step_i % 2 == 1 else -amp
        st.target_steer = target
        st.reason = f"ステップ {self._step_i}/{n_steps - 1}（{math.degrees(target):+.0f}°）"
        return st
