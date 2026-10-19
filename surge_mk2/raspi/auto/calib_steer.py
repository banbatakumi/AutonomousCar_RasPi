"""校正 2: ステアの静特性（舵角 → 実際の曲率）。解析は `tools/calib/steer.py`、設計は `docs/calibration.md` §4.2。

低速（`speed`）で舵角を固定して弧を前進し、同じ舵角のまま後退して元の点へ戻る。これを舵角の段 `LEVELS` で、
−最大 → +最大 → −最大の順に繰り返す（同じ舵角に両側から近づくので、ガタ・ヒステリシスが打ち消し合う）。
低速なのでタイヤの横滑りは無視でき、**曲率 = ヨーレート ÷ 車速 = tan(実際の舵角)/ホイールベース** から実際の
舵角が出る。前進と後退で同じ弧をたどるので、置いた場所から大きく動かない。

舵は後退の終わり（起点の `switch_m` 手前、まだ転がっている間）に次の段へ切り替える（止まった車で舵を切らない＝
据え切りでステアのモータを熱くしない）。区間の名前は `out/<段>/+`（前進）・`back/<段>/-`（後退）・
`switch/<段>/-`（舵の切り替え中。解析は使わない）。

## 最後に向きを戻す

舵を切り替える間（数 cm）に進んだ分は同じ弧をたどり直さないので、段ごとに向きが少しずつ残る（シムで全体で約 4°）。
最後に、ジャイロで積算した向き ψ（バイアスは試験の頭の静止で読む）を `align_m` の弧で打ち消す舵で前進し、
まっすぐ後退して戻る（`align/…`。解析は使わない）。

必要なスペース: 約 0.8m × 0.8m（最大舵角の弧 `arc_m` ぶん）。約 45 秒。
"""

from __future__ import annotations

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
import math

from ._calib_common import DeadReckon, SensorGuard, TestGate, abort_state, settle_state
from .base import Planner

__all__ = ["CalibSteer", "levels"]

#: 舵角の段（最大舵角に対する割合）。0 も含める（中立のずれは当てはめの定数項で決まる）
FRACTIONS = (-1.0, -0.6, -0.25, 0.0, 0.25, 0.6, 1.0)


def levels(max_steer: float) -> list[float]:
    """−最大 → +最大 → −最大 の順の舵角 [rad]（端は 1 回ずつ）。"""
    up = [f * max_steer for f in FRACTIONS]
    return up + up[-2::-1]


class CalibSteer(Planner):
    id = "calib_steer"
    name = "校正 2: ステア（舵角→曲率）"
    description = ("低速で舵角を固定した弧を前進・同じ舵角で後退して戻る、を舵角の段（-最大〜+最大を往復）で繰り返し、"
                   "舵角と実際の曲率の関係を測る。約0.8m四方。約45秒")
    category = "sysid"
    stats = ()
    params = ()

    SETTINGS = {
        "speed": 0.45, "accel": 1.0,
        #: 弧の長さ [m]（前進ぶん。後退で同じだけ戻る）
        "arc_m": 0.4,
        #: 後退の終わりで舵を次の段へ切り替える、起点からの残り [m]
        "switch_m": 0.08,
        #: 片道の時間の上限 [s]（オドメトリが止まっても走り続けない）
        "leg_timeout_s": 4.0,
        #: 最後に向きを戻す弧の長さ [m]
        "align_m": 0.3,
    }

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self._gate = TestGate()
        self._guard = SensorGuard()
        self.vehicle = Vehicle.load()
        self._reset_state()

    def _reset_state(self) -> None:
        self._k = 0
        self._stage = "out"
        self._stage_t = 0.0
        self._origin: float | None = None
        self._guard.reset()
        self._dr = DeadReckon()
        self._align_steer = 0.0

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        self._guard.update(vs)
        self._dr.update(vs)

    def plan(self, scan: Scan, vs: VehicleState | None, p: dict[str, float], dt: float) -> AutoState:
        s = self.settings
        st = AutoState(mode=self.id, planner=self.name)
        st.ready = True
        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._reset_state()
        if not engaged:
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.reason = "ARM待ち（Enterを押してください）"
            return st
        assert vs is not None
        if self._guard.fault is not None:
            return abort_state(st, self._guard.fault, vs.steer_actual)
        if self._gate.settling(dt):
            self._dr.stationary = True
            return settle_state(st)
        self._dr.stationary = False
        lv = levels(self.vehicle.max_steer)
        if self._k >= len(lv):
            return self._align(st, vs, dt)
        odom = 0.5 * (vs.odom_dist[0] + vs.odom_dist[1])
        if self._origin is None:
            self._origin = odom
        pos = odom - self._origin
        self._stage_t += dt
        timeout = self._stage_t >= s["leg_timeout_s"]
        if self._stage == "out" and (pos >= s["arc_m"] or timeout):
            self._stage, self._stage_t = "back", 0.0
        elif self._stage == "back" and (pos <= s["switch_m"] or timeout):
            self._stage, self._stage_t = "switch", 0.0
        elif self._stage == "switch" and (pos <= 0.0 or timeout):
            self._k += 1
            self._stage, self._stage_t = "out", 0.0
            if self._k >= len(lv):
                # 弧は置いた位置から `align_m` 先まで（今は後退の行き過ぎで置いた位置より手前にいる）。後退で置いた位置へ戻る
                self._stage = "align_out"
                L = self.vehicle.wheelbase
                kappa = -self._dr.yaw / max(0.1, s["align_m"] - pos)
                self._align_steer = max(-self.vehicle.max_steer, min(self.vehicle.max_steer, math.atan(kappa * L)))
                return self._align(st, vs, 0.0)
        steer = lv[self._k] if self._stage != "switch" else lv[min(self._k + 1, len(lv) - 1)]
        if self._k + 1 >= len(lv) and self._stage == "switch":
            steer = 0.0            # 最後の段の後は、向きを戻す弧の前にまっすぐへ
        sign = 1.0 if self._stage == "out" else -1.0
        st.target_speed = sign * s["speed"]
        st.accel_limit = s["accel"]
        st.target_steer = steer
        st.phase = f"{self._stage}/{self._k}/{'+' if sign > 0 else '-'}"
        st.reason = (f"{'前進' if sign > 0 else '後退'} 舵 {steer * 57.2958:+.1f}° "
                     f"{self._k + 1}/{len(lv)}{'（舵を切り替え中）' if self._stage == 'switch' else ''}")
        return st

    def _align(self, st: AutoState, vs: VehicleState, dt: float) -> AutoState:
        """最後に向きを戻す: ψ を打ち消す弧で `align_m` 前進 → まっすぐ後退して戻る（モジュールの docstring）。"""
        s = self.settings
        odom = 0.5 * (vs.odom_dist[0] + vs.odom_dist[1])
        pos = odom - (self._origin if self._origin is not None else odom)
        self._stage_t += dt
        timeout = self._stage_t >= s["leg_timeout_s"]
        if self._stage == "align_out" and (pos >= s["align_m"] or timeout):
            self._stage, self._stage_t = "align_back", 0.0
        elif self._stage == "align_back" and (pos <= 0.05 or timeout):
            self._stage = "done"
        if self._stage == "done":
            st.reason = "完了"
            st.phase = "done"
            st.brake = True
            st.brake_torque = 0.05
            return st
        forward = self._stage == "align_out"
        st.target_speed = s["speed"] if forward else -s["speed"]
        st.accel_limit = s["accel"]
        # 後退も同じ舵だと向きが戻ってしまうので、まっすぐ（前進の弧の頭の向き＝ψ を打ち消した向き）で戻る
        st.target_steer = self._align_steer if forward else 0.0
        st.phase = f"align/{self._k}/{'+' if forward else '-'}"
        st.reason = f"向きを戻す（{math.degrees(self._dr.yaw):+.1f}°）"
        return st
