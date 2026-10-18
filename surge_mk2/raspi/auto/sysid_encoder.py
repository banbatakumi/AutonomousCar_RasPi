"""システム同定 — エンコーダ校正試験（前輪エンコーダの周期誤差・出力の範囲）。

直線を前進・後退で往復し、トルク0で惰行する区間の前輪エンコーダの生の読みから、1回転の周期誤差と出力の範囲を
求める。解析は `tools/sysid/encoder_cal.py`、結果は `config/vehicle.toml` の `[control]` の `enc_*`（STM32 へ
`CONFIG_SET` で入る。プロトコル v0.27）。車輪の半径は実測値のまま（この試験では求めない）。

## 1回の片道（前進・後退とも同じ）

1. 加速して `speeds[サイクル]` に乗せ、`settle_s` だけ定速で走る
2. **トルク 0 で惰行**する（`coast_s`、車速が `coast_min_speed` を下回るか直線の端が近づくまで）。
   周期誤差・出力の範囲はこの区間だけで測る: 定速の巡航は車速PIが**エンコーダの読み（周期誤差を含む）**に
   合わせてトルクを揺らすので、本当の回転にも同じ周期のうねりが乗り、周期誤差が小さく出る（0.5m/s・±2.3° で
   真の動きが誤差の約4割ぶん打ち消す見込み）。惰行中は何も読み返さないので、真の回転は転がり抵抗だけで滑らかに減速する
3. 制動して止まる
4. `hold_s` だけ止まってから折り返す

前進と後退で同じ直線の上を往復する（舵は置いた向きの直線へ戻る弱いフィードバック。惰行中も同じ）。

## なぜ速さを2つ使うか

周期誤差は角度の関数で、速度に依らないはず。速さの違う往復で係数が一致することを解析が確かめる
（一致しなければ、角度の関数でない誤差＝振動・ADC の遅れが混ざっている）。

## 必要なスペース

直線 `run_length_m`（2m）＋車長。
"""

from __future__ import annotations

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import DeadReckon, SensorGuard, TestGate, abort_state, settle_state, stop_distance
from .base import Planner

__all__ = ["SysIdEncoder"]

#: 経過時間の比較の余裕 [s]
_EPS = 1e-6


class SysIdEncoder(Planner):
    id = "sysid_encoder"
    name = "システム同定: ⓪ エンコーダ校正"
    description = ("直線を往復し、トルク0で惰行する区間で前輪エンコーダの周期誤差・出力の範囲を測る（4往復）。"
                   "直線2m＋車長。約35秒")
    category = "sysid"
    stats = ()

    SETTINGS: dict[str, float] = {
        "cycles": 4, "run_length_m": 2.0,
        # 往復ごとの惰行に入る車速 [m/s]（サイクルの番号で交互に使う）
        "speed_a": 0.6, "speed_b": 1.0,
        "accel": 1.0,
        # 定速に乗せてから惰行に入るまで [s]（PI の過渡が収まるまで）
        "settle_s": 0.3,
        # 惰行の長さの上限 [s] と、これを下回ったら惰行をやめる車速 [m/s]（遅いと1回転が長く材料が減る）
        "coast_s": 1.5, "coast_min_speed": 0.25,
        "hold_s": 0.5,
        # 置いた向きの直線へ戻る舵の上限 [rad]
        "keep_steer": 0.05,
    }
    params = ()

    _RUN, _COAST, _STOP, _HOLD = range(4)
    _LABELS = {_RUN: "定速", _COAST: "惰行", _STOP: "停止", _HOLD: "静止"}
    _STOPPED_M_S = 0.02
    #: 目標にこれより近ければ「定速に乗った」[m/s]
    _AT_SPEED_M_S = 0.03
    _STOP_TIMEOUT_S = 3.0
    #: 片道の時間の上限 [s]（オドメトリが止まっても走り続けない）
    _LEG_TIMEOUT_S = 12.0

    def __init__(self) -> None:
        self.settings = dict(self.SETTINGS)
        self._gate = TestGate()
        self._guard = SensorGuard()
        self._dr = DeadReckon()
        self.vehicle = Vehicle.load()
        self._reset_state()

    def _reset_state(self) -> None:
        self._t = 0.0
        self._leg_t = 0.0
        self._leg = 0                   # 片道の番号（偶数 = 前進、奇数 = 後退）
        self._phase = self._RUN
        self._at_speed = 0.0             # 定速に乗ってからの時間 [s]
        self._leg_start: float | None = None
        self._origin_set = False
        self._guard.reset()
        self._dr = DeadReckon()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def set_engaged(self, engaged: bool) -> None:
        self._gate.set_engaged(engaged)

    def on_vehicle_state(self, vs: VehicleState) -> None:
        self._guard.update(vs)
        self._dr.update(vs)

    @staticmethod
    def _odom(vs: VehicleState) -> float:
        return (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0

    def _enter(self, phase: int) -> None:
        self._phase = phase
        self._t = 0.0

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        p = {**self.settings, **p}
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
            return abort_state(st, self._guard.fault)
        if self._gate.settling(dt):
            self._dr.stationary = True
            return settle_state(st)
        if not self._origin_set:
            self._dr.reset_origin()            # 置いた位置・向き＝走る直線
            self._origin_set = True

        n_legs = 2 * int(p["cycles"])
        if self._leg >= n_legs:
            self._dr.stationary = True
            st.reason = "完了"
            return st
        if self._leg_start is None:
            self._leg_start = self._odom(vs)
            self._leg_t = 0.0

        self._t += dt
        self._leg_t += dt
        forward = self._leg % 2 == 0
        sign = 1.0 if forward else -1.0
        speed = p["speed_a"] if (self._leg // 2) % 2 == 0 else p["speed_b"]
        traveled = sign * (self._odom(vs) - self._leg_start)
        v = abs(vs.speed)
        accel_now = p["accel"] if self._phase == self._RUN else 0.0
        out_of_road = traveled + stop_distance(v, True, accel_now) >= p["run_length_m"]
        if self._phase == self._RUN and v >= speed - self._AT_SPEED_M_S:
            self._at_speed += dt

        if self._phase in (self._RUN, self._COAST):
            if out_of_road or self._leg_t >= self._LEG_TIMEOUT_S - _EPS:
                self._enter(self._STOP)
            elif self._phase == self._RUN and self._at_speed >= p["settle_s"] - _EPS:
                self._enter(self._COAST)
            elif self._phase == self._COAST and (self._t >= p["coast_s"] - _EPS
                                                 or v < p["coast_min_speed"]):
                self._enter(self._STOP)
        elif self._phase == self._STOP:
            if v <= self._STOPPED_M_S or self._t >= self._STOP_TIMEOUT_S - _EPS:
                self._enter(self._HOLD)
        elif self._phase == self._HOLD and self._t >= p["hold_s"] - _EPS:
            self._leg += 1
            self._leg_start = None
            self._at_speed = 0.0
            self._enter(self._RUN)
            if self._leg >= n_legs:
                self._dr.stationary = True
                st.reason = "完了"
                return st
            forward = self._leg % 2 == 0
            sign = 1.0 if forward else -1.0

        self._dr.stationary = self._phase == self._HOLD
        keep = p["keep_steer"]
        L = self.vehicle.wheelbase
        if self._phase == self._RUN:
            st.target_speed = sign * speed
            st.accel_limit = p["accel"]
            st.target_steer = self._dr.lane_keep_steer(forward, keep, L)
        elif self._phase == self._COAST:
            st.torque_mode = True
            st.target_torque = 0.0
            st.target_steer = self._dr.lane_keep_steer(forward, keep, L)
        else:
            st.brake = True
            st.target_steer = 0.0
        st.reason = (f"{self._LABELS[self._phase]}（{'前進' if forward else '後退'} {speed:.1f}m/s から）"
                     f" {self._leg // 2 + 1}/{int(p['cycles'])}")
        return st
