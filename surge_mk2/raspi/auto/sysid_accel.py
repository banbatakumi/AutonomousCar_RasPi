"""システム同定 — 加減速試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

直進で `target_speed` への加速→急制動を繰り返し、`VehicleState.accel`（x成分。
縦加速度）の応答から `drive_accel_m_s2`（最大加速度）・`brake_decel_m_s2`
（最大減速度）を測る。ログの解析は Mac 側の `tools/sysid/fit.py` の
`fit_accel()` が行う——このプランナーはステップ入力を送るだけで、値の
推定はしない（他の`sysid_*`プランナーと同じ設計）。

## なぜこの試験が要るか

`sim/vehicle.py` の摩擦円連成（`a_lat_max = sqrt((mu*g)^2 - a_long^2)`）が
使う縦加速度の上限は、これまで`MAX_BRAKE_TORQUE_NM`（後輪モータのハードウェア
仕様値）からの逆算に頼っていた。しかし`mu`は`sysid_corner`で実測されている
のに対し、こちらは未実測のまま——実際にタイヤが発生できる加減速度（グリップ
限界を含む）は、モータのトルク仕様とは別物になりうる（`sim/vehicle.py`の
モジュールdocstring「縦加速度の上限」参照）。`mu`と同じ土俵で実測することで、
摩擦円連成が実車に対してどこまで正しく効いているかを検証できる。

## TC（トラクションコントロール）介入込みの値を測る

実機STM32にはTC/TVが実装済み（`docs/architecture.md`）で、`torque_cmd`は
「TC適用後の最終指令値」——ここで測る`accel_x`は、モータのトルク仕様値
ではなく**TCが実際に許した範囲での加減速度**になる（シムはタイヤモデルを
持たずTC自体は再現していないので、その結果だけを実測値として折り込む
狙い）。`tools/sysid/fit.py`の`fit_accel()`は`VehicleState.tc_active`を見て、
加速・制動の両フェーズで実際にTCが介入したかを確認する（介入していなければ
「タイヤが滑り出す本当の上限」を測れていない可能性が高いのでエラーにする）。

**TCのゲイン（`DRIVE_TC_CUT_GAIN`/`DRIVE_TC_RECOVER_RATE`）は実機で調整中**
（`docs/uart_protocol.md` v0.13の変更履歴参照）。測定後にゲインを再調整したら、
`drive_accel_m_s2`/`brake_decel_m_s2`も変わりうるので測り直すこと。

## 直進のみ・後輪駆動なので`sysid_speed`の延長

`target_steer`は常に0（直進）。加速→制動の繰り返しは`sysid_speed`の
前進⇄後退ステップと似た構成だが、こちらは「速度応答の時定数」ではなく
「実際に出た加速度そのもの」を見るため、`brake=True`による急制動フェーズを
明示的に挟む。

## 試験開始/中止を押すまでステップは進まない

他の`sysid_*`プランナーと同じ`TestGate`（`_sysid_common.py`）を使う。
詳しい経緯はそちらのdocstring参照。
"""

from __future__ import annotations

from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import TestGate
from .base import ParamSpec, Planner

__all__ = ["SysIdAccel"]


class SysIdAccel(Planner):
    id = "sysid_accel"
    name = "システム同定: 加減速"
    description = "直進で加速→急制動を繰り返し、drive_accel_m_s2・brake_decel_m_s2を測る"
    category = "sysid"
    stats = ()

    params = (
        ParamSpec(key="target_speed", label="目標速度", min=0.3, max=3.0, step=0.1,
                  default=1.5, unit="m/s",
                  note="この速度まで加速してから急制動する。速いほど加速度の頭打ちを"
                       "踏みやすいが、加速・惰行・制動距離ぶんのオープンスペースが要る"),
        ParamSpec(key="accel_hold_s", label="加速フェーズの保持時間", min=0.5, max=5.0, step=0.1,
                  default=2.0, unit="s",
                  note="目標速度に達してからもこの時間だけ走り続ける（速度指令の"
                       "一次遅れが収束しきるまでの余裕）"),
        ParamSpec(key="brake_hold_s", label="制動フェーズの保持時間", min=0.5, max=5.0, step=0.1,
                  default=1.5, unit="s",
                  note="完全停止した後もこの時間だけ制動を維持する"),
        ParamSpec(key="cycles", label="繰り返し回数", min=1, max=8, step=1,
                  default=4, unit="回",
                  note="加速→制動を1組として繰り返す回数。多いほどフィッティングが安定する"),
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

        n_steps = int(p["cycles"]) * 2   # 加速→制動 を1組として cycles 回

        if self._step_i >= n_steps:
            st.target_speed = 0.0
            st.reason = "完了"
            return st

        accelerating = self._step_i % 2 == 0
        hold = p["accel_hold_s"] if accelerating else p["brake_hold_s"]

        self._t += dt
        if self._t >= hold:
            self._t = 0.0
            self._step_i += 1
            if self._step_i >= n_steps:
                st.target_speed = 0.0
                st.reason = "完了"
                return st
            accelerating = self._step_i % 2 == 0

        if accelerating:
            st.target_speed = p["target_speed"]
        else:
            st.brake = True
        phase = "加速" if accelerating else "制動"
        st.reason = f"{phase} {self._step_i // 2 + 1}/{int(p['cycles'])}"
        return st
