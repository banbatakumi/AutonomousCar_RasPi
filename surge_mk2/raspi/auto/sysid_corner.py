"""システム同定 — 旋回グリップ試験（`config/vehicle.toml` `[dynamics]` の実測手順）。

固定舵角のまま `target_speed` を**一定の割合で連続して上げて**円旋回させ、`yaw_rate`・`speed`
から `mu`（横加速度 `speed*yaw_rate` が頭打ちになった値 / g。`tools/sysid/fit.py` の
`fit_corner()`）を測る。限界より下の区間の曲率は、大舵角での舵の効きと、速度とともに曲率が
落ちる度合い（アンダーステア勾配）の材料として `fit_geometry()` が他の試験と合わせて使う
（速度の幅があるのはこの試験だけ）。

3試験の中で唯一グリップ限界まで走らせるためリスクが最も高い。既存の安全機構
（人間のARM保持・STM32の`auto_stop`・GUIのdisengage）に加えて、**周囲クリアランスを
LiDARで見ながら速度を上げる**（`scan_window()`を流用）。

## 速度は段ではなく連続して上げる（2026-09-27）

以前は速度を0.11m/s刻みの段（各2.5s）で上げ、各段の後半の定常値を使っていた。バンビの
「連続的に速度を変える方が連続的に値が取れる」を受けて、`ramp_m_s2`（0.06m/s²）で上げ続ける
形にした（自動車の定常円旋回試験の、舵角を固定して速度をゆっくり上げる方法と同じ考え方）。
横加速度と速度の関係がどの速度でも取れ、限界の検出も段の刻みに縛られない。上げる速さは
ヨーの遅れ（0.1s程度）に対して十分遅く、縦の加速度（0.06m/s²）は横（限界で約4.5m/s²）に比べて
無視できる（摩擦円を縦に食わない）。

## 限界に達したら、舵を保ったままゆっくり止める（2026-09-27、実機の記録を受けて）

速度・横加速度・後輪の滑り率を `CORNER_WINDOW_S` の区間ごとにまとめ、`corner_limit()`
（`_sysid_common.py`。解析側と同じ判定）が限界を示したらすぐ終える。限界は **後輪が滑った**
（滑り率が `REAR_SLIP_LIMIT` を超えた）か **横加速度が頭打ち**の早い方。この車は後輪駆動なので、
実機では前輪の頭打ちより先に後輪が滑った（1.45m/s・約5.0m/s²、同時に曲率が増えた＝オーバー
ステア）。以前は頭打ちの判定だけで、しかも検出後も1秒速度を上げ続けたので、後輪が空転して
`SensorGuard` が中止し、最大制動＋舵0で**旋回中に後輪がロック**した。

止め方: 舵はそのまま、目標速度0を `accel_limit` 1.0m/s² でゆっくり（制動は後輪がロックする）。
周囲クリアランスでの停止・中止も、舵を保ってロックしない強さ（`ABORT_BRAKE_TORQUE_NM`）で制動する。
`v_max` は「そこまでに限界が来なければ諦める」上限。

## 舵角は`max_steer`まで振ってよい

舵角を大きくするほど旋回半径が小さくなり、**より低い速度でグリップ限界に到達できる**
（スペースも小さく、安全にもつながる）。`target_steer`は`self.vehicle.max_steer`で
クランプする。左右の特性差を見たいときは `direction` を変えてもう一度録る。

## 試験開始/中止を押すまで進まない

`TestGate`（`_sysid_common.py`）参照。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import AutoState, Scan, VehicleState
from ._sysid_common import (ABORT_BRAKE_TORQUE_NM, SensorGuard, TestGate, abort_state, corner_limit,
                            corner_windows, rear_slip, settle_state)
from .base import ParamSpec, Planner, scan_window

__all__ = ["SysIdCorner"]


class SysIdCorner(Planner):
    id = "sysid_corner"
    name = "システム同定: 旋回グリップ"
    description = "舵30°の円を0.3m/sから一定の割合で速くしながら回り、滑り出したら止まる。約2m四方が要る。約30〜45秒"
    category = "sysid"
    stats = ("nearest",)

    #: 試験の手順。**GUIからは変えられない**（`_sysid_common.py`「手順を固定した理由」）。
    #: 旋回方向だけは左右差を見るために選べる（`params`）
    SETTINGS: dict[str, float] = {
        # 30°（最大舵角）: 半径が最小で、低い速度（mu=0.45なら約1.33m/s）で限界に届く
        "steer_deg": 30.0,
        # 0.3m/sから0.06m/s²で上げ続ける（段の頃の平均 0.11m/s÷2.5s≒0.045m/s² とほぼ同じ）。
        # 限界を検出したら confirm_s だけ続けて止める。3.0m/s（io_node の上限）は「そこまでに
        # 限界が来なければ諦める」上限（2.0m/sだと、摩擦係数が大きい・大舵角で舵の効きが
        # 落ちている車で滑り出す約2.2m/s に届かなかった。限界の直前の円の大きさは速度によらず
        # ほぼ最小旋回半径なので、上限を上げてもスペースは増えない）
        "v_start": 0.3, "v_max": 3.0, "ramp_m_s2": 0.06,
        # 限界に達した後、目標速度0へ下げる速さ（舵は保つ。制動は後輪がロックする）
        "stop_decel_m_s2": 1.0,
        # 走り出しの過渡（速度が目標に追いつくまで）は頭打ちの判定に使わない
        "warmup_s": 1.5,
        # 全周のLiDAR最近傍がこれを切ったら止まる
        "margin_m": 0.3,
    }
    params = (
        ParamSpec(key="direction", label="旋回方向", min=-1.0, max=1.0, step=2.0,
                  default=1.0, unit="",
                  note="+1=左回り、-1=右回り。左右の特性差を見たいときは両方録る"),
    )

    def __init__(self) -> None:
        #: 手順（`SETTINGS`の写し。テスト・ベンチだけが差し替える）
        self.settings = dict(self.SETTINGS)
        self.vehicle = Vehicle.load()
        self._gate = TestGate()
        #: 前輪エンコーダの不調で段を上げ続けないための照合（`SensorGuard`）
        self._guard = SensorGuard()
        self._reset_state()

    def _reset_state(self) -> None:
        self._t = 0.0
        self._done_reason = ""
        #: 走り出してからの (経過時間, 速さ, 横加速度, 後輪の滑り率)。`on_vehicle_state` が全サンプルで足す
        self._t_s: list[float] = []
        self._v_s: list[float] = []
        self._a_s: list[float] = []
        self._slip_s: list[float] = []
        self._t0_ns: int | None = None
        self._running = False
        #: 限界を検出したか
        self._at_limit = False
        self._checked_until = 0.0
        self._guard.reset()

    def reset(self) -> None:
        self._gate.reset()
        self._reset_state()

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """`planning_node`が全サンプルで呼ぶ（ダックタイピング）。前輪と後輪の速度を照合し、
        走っている間の速さと横加速度を溜める（頭打ちの判定の材料）。"""
        self._guard.update(vs)
        if not self._running:
            return
        if self._t0_ns is None:
            self._t0_ns = vs.t_capture
        self._t_s.append((vs.t_capture - self._t0_ns) / 1e9)
        self._v_s.append(abs(vs.speed))
        self._a_s.append(abs(vs.speed * vs.yaw_rate))
        rear = 0.5 * (vs.wheel_speed[2] + vs.wheel_speed[3]) if len(vs.wheel_speed) >= 4 else vs.speed
        self._slip_s.append(rear_slip(vs.speed, rear))

    def set_engaged(self, engaged: bool) -> None:
        """`planning_node.py`がplan()の直前に呼ぶ（ダックタイピング）。"""
        self._gate.set_engaged(engaged)

    def _check_limit(self, p: dict[str, float]) -> None:
        """溜めたサンプルで限界を判定する（解析 `fit_corner()` と同じ区間・同じ判定）。
        判定は `CORNER_WINDOW_STEP_S` ごとに1回（毎周だと全区間を作り直して重い）。"""
        if self._at_limit or not self._t_s or self._t_s[-1] < self._checked_until:
            return
        self._checked_until = self._t_s[-1] + 0.25
        i0 = next((i for i, t in enumerate(self._t_s) if t >= p["warmup_s"]), len(self._t_s))
        t, v = self._t_s[i0:], self._v_s[i0:]
        wa = corner_windows(t, v, self._a_s[i0:])
        ws = corner_windows(t, v, self._slip_s[i0:])
        k, why = corner_limit([(v_, a, sl) for (v_, a, _, _), (_, sl, _, _) in zip(wa, ws)])
        if k is not None:
            self._at_limit = True
            self._done_reason = f"完了（グリップ限界: {why}）"

    def _stop_state(self, st: AutoState, p: dict[str, float], steer: float) -> AutoState:
        """終わった後: 舵を保ったまま目標速度0へゆっくり（制動しない）。"""
        st.ready = True
        st.target_speed = 0.0
        st.target_steer = steer
        st.accel_limit = p["stop_decel_m_s2"]
        st.reason = self._done_reason
        return st

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        p = {**self.settings, **p}
        st = AutoState(mode=self.id, planner=self.name)

        engaged, armed, just_started = self._gate.tick(vs)
        if just_started:
            self._reset_state()
        if not engaged:
            st.ready = True
            st.reason = "試験開始を押してください"
            return st
        if not armed:
            st.ready = True
            st.reason = "ARM待ち（Enterを押してください）"
            return st

        steer = math.copysign(min(math.radians(p["steer_deg"]), self.vehicle.max_steer), p["direction"])
        if self._guard.fault is not None:
            return abort_state(st, self._guard.fault, steer)
        if self._gate.settling(dt):
            return settle_state(st)

        # ── 安全: 全周のクリアランスを見る。切れたら速度を上げている途中でも止まる ──
        # 舵は保ち、後輪がロックしない強さで制動する（最大制動は旋回中に後輪をロックさせる）
        w = scan_window(scan, 360.0, max(1.0, p["margin_m"] * 4))
        nearest = min(w.dist) if w.dist else 0.0
        st.nearest = nearest
        if nearest < p["margin_m"]:
            st.ready = True
            st.brake = True
            st.brake_torque = ABORT_BRAKE_TORQUE_NM
            st.target_steer = steer
            st.reason = f"周囲クリアランス不足（最近傍 {nearest * 100:.0f}cm）で停止"
            return st

        if self._done_reason:
            return self._stop_state(st, p, steer)

        self._running = True
        self._t += dt
        self._check_limit(p)
        speed = min(p["v_start"] + p["ramp_m_s2"] * self._t, p["v_max"])
        if not self._done_reason and speed >= p["v_max"]:
            self._done_reason = "完了（上限速度まで限界が来なかった。v_maxを上げて録り直す）"
        if self._done_reason:
            return self._stop_state(st, p, steer)

        st.target_steer = steer
        st.target_speed = speed
        st.ready = True
        st.reason = f"旋回 {speed:.2f}m/s"
        return st
