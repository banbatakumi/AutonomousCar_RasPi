"""校正の試験（`raspi/auto/calib_*.py`）が共有する部品。設計は `docs/calibration.md`。

**catalog には登録しない**（`_` 始まりのファイル名は `raspi/auto/registry.py` の対象外）。

## 区間の名前（`AutoState.phase`）

解析（`tools/calib`）は記録の区間を `phase` で切り分ける。値を推測で区切らない（速度や指令から
区間を推定すると、実機の遅れ・雑音で境目がずれる）。直線の試験の名前は `"<種類>/<片道の番号>/<向き>"`
（例 `"coast/3/-"`。向きは `+` 前進・`-` 後退）。`phase_parts()` で分解する。

## 手順は GUI から変えられない

速度・時間・回数は「測りたい量が見える」「解析の前提を満たす」「直線 2m に収まる」を同時に満たす組み合わせで
決めてある。各 planner の `SETTINGS` に固定し、変えるときはコードを直して `tools/calib/tests` で確かめる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ..proto.generated.packets import MDS_COMM_OK, MDS_VOLTAGE_OOR
from .base import Planner

__all__ = ["MD_NAMES", "md_down", "SensorGuard", "ABORT_BRAKE_TORQUE_NM", "abort_state", "DeadReckon",
           "settle_state", "SETTLE_S", "stop_distance", "TestGate", "Step", "StraightTest", "phase_parts"]

#: `md_status` の並び（`uart_protocol.md` §5.5）
MD_NAMES = ("左後輪", "右後輪", "ステア")


def md_down(md_status: list[int]) -> list[str]:
    """無応答のモータドライバの名前。`md_status` が 0（まだ状態が来ていない）は故障と見なさない。"""
    return [MD_NAMES[i] for i, s in enumerate(md_status[:3]) if s and not (s & MDS_COMM_OK)]


def phase_parts(phase: str) -> tuple[str, int, int] | None:
    """`"<種類>/<片道>/<向き>"` → (種類, 片道の番号, +1/-1)。形が違えば None。"""
    parts = phase.split("/")
    if len(parts) != 3 or parts[2] not in ("+", "-"):
        return None
    try:
        return parts[0], int(parts[1]), 1 if parts[2] == "+" else -1
    except ValueError:
        return None


class SensorGuard:
    """前輪の速度を独立な後輪モータの周速と照合し、前輪エンコーダが止まったまま走り続けるのを防ぐ。

    試験の「止まる」「直線が尽きた」の判定は前輪由来の `speed`・`odom_dist` を見る。エンコーダが止まると
    速度 0・距離が進まないと読み続け、直線を超えて走る。**後輪が回っているのに前輪がそれより大きく遅い**
    状態が `PERSIST_S` 続いたら異常とする。モータドライバの無応答（`MDS_COMM_OK` が落ちた）も見る
    （黙った MD の車輪速度は古い値のまま固まる）。

    後輪をわざと空転させる試験（タイヤ）は `wheel_check=False`。"""

    TOLERANCE_M_S = 0.2
    TOLERANCE_FRAC = 0.3
    PERSIST_S = 0.3

    def __init__(self, wheel_check: bool = True) -> None:
        self.wheel_check = wheel_check
        self.fault: str | None = None
        self._bad_since_ns: int | None = None

    def reset(self) -> None:
        self.fault = None
        self._bad_since_ns = None

    def update(self, vs: VehicleState) -> None:
        """ARM 済みのサンプルだけで判定する（ARM 前の MD は起動待ちで状態が来ていない）。"""
        if self.fault is not None:
            return
        if not vs.armed:
            self._bad_since_ns = None
            return
        down = md_down(vs.md_status)
        if down:
            volt = any(s & MDS_VOLTAGE_OOR for s in vs.md_status[:3])
            self.fault = (f"{'・'.join(down)}のモータドライバが無応答"
                          f"{'（電圧異常）' if volt else ''}。駆動バッテリーの残量と MD の電源を確かめる")
            return
        if not self.wheel_check or len(vs.wheel_speed) < 4:
            return
        front = vs.speed
        rear = 0.5 * (vs.wheel_speed[2] + vs.wheel_speed[3])
        opposite = front * rear < 0 and min(abs(front), abs(rear)) > self.TOLERANCE_M_S
        slow = abs(rear) - abs(front) > self.TOLERANCE_M_S + self.TOLERANCE_FRAC * abs(front)
        if not (slow or opposite):
            self._bad_since_ns = None
            return
        if self._bad_since_ns is None:
            self._bad_since_ns = vs.t_capture
        elif vs.t_capture - self._bad_since_ns >= self.PERSIST_S * 1e9:
            cause = ("後輪の空転か前輪エンコーダの不調" if abs(front) >= 0.25
                     else "前輪エンコーダの不調？前輪がほぼ止まっている")
            self.fault = f"前輪の速度 {front:+.2f}m/s が後輪 {rear:+.2f}m/s と食い違う（{cause}）"


#: 試験を止めるときの制動トルク [N·m]（1輪あたり）。0.12N·m 以上は ABS なしで後輪がロックする。
#: 0.08N·m はロックせず、1m/s 以上で約 3m/s² で止まる
ABORT_BRAKE_TORQUE_NM = 0.08


def abort_state(st: AutoState, why: str, steer: float = 0.0) -> AutoState:
    """試験を中止して制動する指令（舵は `steer` を保つ。`ready=False` にすると最大制動・舵 0 に読み替わる）。"""
    st.ready = True
    st.brake = True
    st.brake_torque = ABORT_BRAKE_TORQUE_NM
    st.target_speed = 0.0
    st.target_steer = steer
    st.fw_overrides = {}
    st.reason = f"中止: {why}"
    st.phase = "abort"
    return st


class DeadReckon:
    """前輪オドメトリの増分と `yaw_rate` から平面上の位置・向きを積算する（置いた直線へ戻るためだけに使う）。

    ジャイロのバイアスは**止まっている間のサンプル**（planner が `stationary` で教える）の平均で推定して
    引く（0.006rad/s を放っておくと 1 分で 20° ずれる）。"""

    def __init__(self) -> None:
        self.x = self.y = self.yaw = 0.0
        self._t_ns: int | None = None
        self._odom: float | None = None
        self._bias_sum = 0.0
        self._bias_n = 0
        self.stationary = False

    @property
    def bias(self) -> float:
        return self._bias_sum / self._bias_n if self._bias_n else 0.0

    def reset_origin(self) -> None:
        self.x = self.y = self.yaw = 0.0

    def update(self, vs: VehicleState) -> None:
        t = vs.t_capture
        odom = (vs.odom_dist[0] + vs.odom_dist[1]) / 2.0
        if self._t_ns is None or t <= self._t_ns:
            self._t_ns = t
            self._odom = odom
            return
        dt = min((t - self._t_ns) / 1e9, 0.1)
        ds = (odom - self._odom) * math.cos(vs.steer_actual)
        self._t_ns = t
        self._odom = odom
        if self.stationary:
            self._bias_sum += vs.yaw_rate
            self._bias_n += 1
            return
        self.yaw += (vs.yaw_rate - self.bias) * dt
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

    def lane_keep_steer(self, forward: bool, limit: float, wheelbase: float) -> float:
        """原点を通る元の向きの直線へ戻る舵角（弱いフィードバック、±`limit`）。"""
        k, ky = 1.5, 1.0
        kappa = -k * (self.yaw + ky * self.y) if forward else k * (self.yaw - ky * self.y)
        return max(-limit, min(limit, math.atan(kappa * wheelbase)))


#: 試験開始直後に止まっている時間 [s]（ジャイロのバイアスを読む。解析もここを静止の基準に使う）
SETTLE_S = 1.0


def settle_state(st: AutoState) -> AutoState:
    st.ready = True
    st.brake = False
    st.target_speed = 0.0
    st.target_steer = 0.0
    st.reason = "静止（ジャイロのバイアスを計測）"
    st.phase = "settle"
    return st


#: 停止距離の見積もりの減速度 [m/s²] と、判断してから効き始めるまでの遅れ [s]。測る対象に頼らない保守側の値
_STOP_DECEL_M_S2 = 2.0
_STOP_LATENCY_S = 0.12


def stop_distance(v: float, accel: float = 0.0, decel: float = _STOP_DECEL_M_S2) -> float:
    """速度 `v` から止まりきるまでの距離の見積もり [m]（保守側）。`accel` は今の加速度（遅れの間も加速する）。"""
    v = abs(v)
    a = max(0.0, accel)
    v_peak = v + a * _STOP_LATENCY_S
    return v * _STOP_LATENCY_S + 0.5 * a * _STOP_LATENCY_S ** 2 + v_peak * v_peak / (2.0 * decel)


class TestGate:
    """試験の状態機械を「試験開始」から「中止」までの間だけ進める。

    `planning_node` は engage していなくても `plan()` を呼ぶ。時間で進む試験はそれでは勝手に進むので、
    `planning_node` がダックタイピングで呼ぶ `set_engaged()` の立ち上がりを「試験開始」とみなす。
    ARM（人が保持）は表示だけに使う（ソフトから ARM を奪わない）。"""

    def __init__(self) -> None:
        self._engaged = False
        self._just_started = False
        self._settle_t = 0.0

    def reset(self) -> None:
        self._engaged = False
        self._just_started = False
        self._settle_t = 0.0

    def settling(self, dt: float) -> bool:
        if self._settle_t >= SETTLE_S:
            return False
        self._settle_t += dt
        return True

    def set_engaged(self, engaged: bool) -> None:
        if engaged and not self._engaged:
            self._just_started = True
            self._settle_t = 0.0
        self._engaged = engaged

    def tick(self, vs: VehicleState | None) -> tuple[bool, bool, bool]:
        """`(engaged, armed, just_started)`。"""
        armed = bool(vs and vs.armed)
        just_started = self._just_started
        self._just_started = False
        return self._engaged, armed, just_started


@dataclass
class Step:
    """直線の試験の 1 周期ぶんの指令（`StraightTest.leg_step` が返す）。

    `kind`: `"speed"`（車速指令。`value` は進む向きの車速 [m/s]、正）、`"torque"`（トルク直接指令。`value` は
    進む向きの1輪あたりのトルク [N·m]、負なら逆向き）、`"brake"`（`value` は1輪あたりの制動トルク [N·m]）。
    `label` は区間の名前の種類（`phase` の先頭）。`done` なら片道の手順は終わり（止まって折り返す）。"""

    kind: str = "speed"
    value: float = 0.0
    label: str = "run"
    accel_limit: float = 0.0
    overrides: dict[str, float] = field(default_factory=dict)
    done: bool = False
    note: str = ""
    #: 手順が自分で止めている最中（土台は通路の端で打ち切らない。制動や強い減速そのものを測る試験）
    stopping: bool = False
    #: 置いた直線へ戻る舵に上乗せする舵角 [rad]（舵を振る試験）
    steer_offset: float = 0.0


class StraightTest(Planner):
    """置いた向きの直線を前進・後退で往復する試験の土台。

    片道 `leg`（偶数 = 前進、奇数 = 後退）ごとに、サブクラスの `leg_step()` が手順を返す。土台が受け持つのは:

    - 試験開始 → `SETTLE_S` 静止 → 片道 × `leg_count()` → 完了
    - **直線の端**: 走ってよいのは置いた位置から前へ `run_length_m` の通路（後退は置いた位置まで）。
      端までの残りが停止距離を切ったら、その片道はそこで制動して終える（片道の起点から測ると、前進と後退の
      距離の差が往復ごとに積み重なって通路から出ていく）
    - 片道の終わり: 制動（`stop_torque_nm`、ABS は入れたまま）→ 止まったら `hold_s` 静止（ジャイロのバイアスを読む）
    - 置いた直線へ戻る弱い舵（`DeadReckon`。前輪オドメトリとジャイロ）
    - 異常（前輪エンコーダの停止・MD の無応答）で中止

    区間の名前は `"<種類>/<片道>/<向き>"`（`phase_parts`）。止まる・静止は `"stop"`・`"hold"`。
    """

    category = "sysid"
    stats = ()
    params = ()
    #: 直線の長さ [m]（片道の起点から）
    run_length_m = 2.0
    hold_s = 0.6
    stop_torque_nm = 0.06
    keep_steer_rad = 0.05
    #: 片道の時間の上限 [s]（オドメトリが止まっても走り続けない）
    leg_timeout_s = 12.0
    #: 後輪をわざと滑らせる試験は False（前輪と後輪の照合をしない）
    wheel_check = True
    #: 直線の端で止まりきれるかの見積もりに使う減速度 [m/s²]（土台の止め方 `stop_torque_nm` で出る減速度の保守側）
    stop_decel_m_s2 = 2.0
    _STOPPED_M_S = 0.02
    _STOP_TIMEOUT_S = 3.0
    #: 停止の待ちが尽きたとき、これを超える速度で動いていれば中止する [m/s]
    _STILL_MOVING_M_S = 0.1

    def __init__(self) -> None:
        self.settings = dict(getattr(self, "SETTINGS", {}))
        self._gate = TestGate()
        self._guard = SensorGuard(wheel_check=self.wheel_check)
        self.vehicle = Vehicle.load()
        self._reset_state()

    # ── サブクラスが書くもの ───────────────────────────────────────────────

    def leg_count(self) -> int:
        raise NotImplementedError

    def leg_step(self, leg: int, t: float, vs: VehicleState, traveled: float) -> Step:
        """片道 `leg` の、開始から `t` 秒・起点から `traveled` m（進む向きを正）の時点の指令。

        `self.room` に、進む向きの通路の残り [m] が入っている（自分で強く止める試験が使う）。"""
        raise NotImplementedError

    def leg_title(self, leg: int) -> str:
        return f"{leg // 2 + 1}/{self.leg_count() // 2}"

    def keep_limit(self, leg: int) -> float:
        """片道 `leg` で、置いた直線へ戻る舵の上限 [rad]。"""
        return self.keep_steer_rad

    def leg_stop_decel(self, leg: int) -> float:
        """片道 `leg` で、直線の端の手前で止まりきれるかの見積もりに使う減速度 [m/s²]。"""
        return self.stop_decel_m_s2

    # ── 土台 ──────────────────────────────────────────────────────────────

    def _reset_state(self) -> None:
        self._leg = 0
        self._leg_t = 0.0
        self._leg_start: float | None = None
        self._stage = "leg"            # leg / stop / hold
        self._stage_t = 0.0
        self._origin_set = False
        self._origin_odom = 0.0
        self._guard.reset()
        self._dr = DeadReckon()
        self._last_accel = 0.0
        self._last_v: float | None = None
        self.room = self.run_length_m

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

    def _phase(self, kind: str) -> str:
        return f"{kind}/{self._leg}/{'+' if self._leg % 2 == 0 else '-'}"

    def plan(self, scan: Scan, vs: VehicleState | None, p: dict[str, float], dt: float) -> AutoState:
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
            self._dr.reset_origin()
            self._origin_odom = self._odom(vs)
            self._origin_set = True
        if self._leg >= self.leg_count():
            self._dr.stationary = True
            st.reason = "完了"
            st.phase = "done"
            return st
        if self._leg_start is None:
            self._leg_start = self._odom(vs)
            self._leg_t = 0.0
        forward = self._leg % 2 == 0
        sign = 1.0 if forward else -1.0
        traveled = sign * (self._odom(vs) - self._leg_start)
        v = abs(vs.speed)
        if self._last_v is not None and dt > 0:
            self._last_accel = (v - self._last_v) / dt
        self._last_v = v
        self._stage_t += dt
        self._leg_t += dt
        L = self.vehicle.wheelbase
        steer = self._dr.lane_keep_steer(forward, self.keep_limit(self._leg), L)
        self._dr.stationary = False

        if self._stage == "leg":
            self.room = (self.run_length_m - (self._odom(vs) - self._origin_odom) if forward
                         else self._odom(vs) - self._origin_odom)
            step = self.leg_step(self._leg, self._leg_t, vs, traveled)
            pos = self._odom(vs) - self._origin_odom
            room = self.run_length_m - pos if forward else pos
            self.room = room
            # 手順が自分で制動している間は通路の端で打ち切らない（止まるのが手順そのもの）
            end = step.kind != "brake" and not step.stopping and stop_distance(v, self._last_accel, self.leg_stop_decel(self._leg)) >= room
            if step.done or end or self._leg_t >= self.leg_timeout_s:
                self._stage, self._stage_t = "stop", 0.0
            else:
                st.phase = self._phase(step.label)
                st.target_steer = steer + step.steer_offset
                st.fw_overrides = dict(step.overrides)
                if step.kind == "speed":
                    st.target_speed = sign * step.value
                    st.accel_limit = step.accel_limit
                elif step.kind == "torque":
                    st.torque_mode = True
                    st.target_torque = sign * step.value
                elif step.kind == "brake":
                    st.brake = True
                    st.brake_torque = step.value
                else:
                    raise ValueError(step.kind)
                st.reason = f"{step.note or step.label}（{'前進' if forward else '後退'} {self.leg_title(self._leg)}）"
                return st
        if self._stage == "stop":
            if v <= self._STOPPED_M_S:
                self._stage, self._stage_t = "hold", 0.0
            elif self._stage_t >= self._STOP_TIMEOUT_S:
                if v > self._STILL_MOVING_M_S:
                    # 止まらないまま静止（ジャイロのバイアス）や次の向きの片道へ進まない
                    return abort_state(st, f"制動しても止まらない（{vs.speed:+.2f}m/s）", steer)
                self._stage, self._stage_t = "hold", 0.0     # ごくゆっくりの這い（エンコーダの分解能）は止まったと見なす
            else:
                st.brake = True
                st.brake_torque = self.stop_torque_nm
                st.target_steer = steer
                st.phase = self._phase("stop")
                st.reason = f"停止（{self.leg_title(self._leg)}）"
                return st
        # hold
        self._dr.stationary = True
        st.brake = True
        st.brake_torque = self.stop_torque_nm
        st.target_steer = 0.0
        st.phase = self._phase("hold")
        st.reason = f"静止（{self.leg_title(self._leg)}）"
        if self._stage_t >= self.hold_s:
            self._leg += 1
            self._leg_start = None
            self._stage, self._stage_t = "leg", 0.0
        return st
