"""E2E 融合 — LiDAR＋前方カメラ＋車両状態から学習したモデルが出す**前方の経路点**を追って走る。

推論は `raspi/nodes/e2e_fusion_node.py`（独立プロセス）が行い、ここはその結果（`E2EFusionPlan`）を
走行指令にする。学習は `ml_e2e/`（実機の記録の模倣）。地図も自己位置も使わない。

## 経路点を追う

モデルが出すのは経路点だけ（舵・速度の直接出力は持たない）。それを **Pure Pursuit で追う**
（`raspi/nav/purepursuit.follow`）。

- **推論の遅れを幾何で消せる。** 経路点は画像の撮影時刻の車体座標。撮影から今までに車が動いたぶんを
  推測航法（車速とヨーレートの積分）で求め、経路を「今の車」から見た位置へ移してから追う。舵を直接出す
  モデルだと撮影時刻の舵になり、遅れのぶんだけ古い（`cam_e2e` はラベルを時間でずらして補った）
- **点群の合間も追える**（`tick`、50Hz）。推論は 10Hz だが、その間も経路の上の位置は進む
- **速度の上限を物理から掛けられる。** 経路の曲率と `Vehicle` の横の限界から出した速度（`raceline.speed_profile`）を
  モデルの速度に被せる。手本に速度の無いコース（人がゆっくり走っただけ）は、こちらだけで走る

## 止まる条件（`ready=False`）

推論ノードが出せない（モデル無し・入力途絶）、値が有限でない、経路の不確かさ `b` が注視点までに `b_max` を
超える（どこを走るかモデルが決められない）、経路が車の後ろへ向いている。固定距離の停止は持たない
（学習データに無い状況の挙動は保証できない。STM32 の `auto_stop` と、人が触れた瞬間の解除で受ける。
`e2e_lidar.py`・`cam_e2e.py` と同じ）。

## 速度

    経路の各点の上限 = min(最高速度, モデルが出した各点の速度)      ← `use_model_speed` が 1 のとき
    → `speed_profile`（横の限界 → 減速 → 加速）で今の車速から引き直す
    → 経路の終わりの速度: モデルの速度を使うならその最後の点の値、使わないなら「どんなコーナーでも曲がれる速度」

速度指令は `slam2d_raceline` と同じく、STM32 の加速度フィードフォワードが有効なら `speedctl.ref_command`、
無効なら `speedctl.SpeedFF`（速度 PI の遅れの前倒し）。
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np

from ..core.vehicle import Vehicle
from ..msgs.types import TOPIC_E2E_FUSION_PLAN, AutoState, E2EFusionPlan, VehicleState
from ..nav import raceline as rl_mod
from ..nav import speedctl
from ..nav.purepursuit import PursuitConfig, follow, nearest_index
from .base import CMD_HZ, ParamSpec, Planner, bool_param

__all__ = ["E2EFusion", "PoseTrack"]

NS = 1_000_000_000
#: 推測航法の履歴を持つ長さ [s]（経路の基準時刻へさかのぼれる長さ）
_HISTORY_S = 1.0
#: 推測航法で積分する1サンプルの最大 [s]（車両状態が途切れた後に大きく跳ばない）
_MAX_DT_S = 0.1
#: 経路の基準時刻がこれより古ければ使わない [s]
_PLAN_MAX_AGE_S = 0.4
#: 追う経路の最短の長さ [m]
_MIN_PATH_M = 0.6


class PoseTrack:
    """車速とヨーレートを積分した姿勢（原点は任意）と、その短い履歴。

    経路点は撮影時刻の車体座標で届くので、「撮影時刻の姿勢」と「今の姿勢」の両方が要る。
    絶対位置は要らない（1秒以内の相対的な動きだけ使う）ので、ジャイロのゼロ点の補正はしない。
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.x = self.y = self.yaw = 0.0
        self._t = 0
        self._hist: deque[tuple[int, float, float, float]] = deque()

    def update(self, vs: VehicleState) -> None:
        t = int(vs.t_capture)
        if not t or t <= self._t:
            return
        if self._t:
            dt = min((t - self._t) / NS, _MAX_DT_S)
            v = 0.0 if vs.stopped else float(vs.speed)
            w = float(vs.yaw_rate)
            mid = self.yaw + 0.5 * w * dt
            self.x += v * math.cos(mid) * dt
            self.y += v * math.sin(mid) * dt
            self.yaw += w * dt
        self._t = t
        self._hist.append((t, self.x, self.y, self.yaw))
        while self._hist and t - self._hist[0][0] > _HISTORY_S * NS:
            self._hist.popleft()

    @property
    def pose(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    @property
    def t(self) -> int:
        return self._t

    def at(self, t: int) -> tuple[float, float, float] | None:
        """時刻 `t` の姿勢（履歴を線形補間）。履歴より前なら `None`、後なら最新。"""
        h = self._hist
        if not h or t < h[0][0]:
            return None
        if t >= h[-1][0]:
            return h[-1][1:]
        # 履歴は 100 件ほど。新しい側から探す（基準時刻はたいてい 0.1s 以内）
        for i in range(len(h) - 1, 0, -1):
            t0, x0, y0, a0 = h[i - 1]
            if t0 <= t:
                t1, x1, y1, a1 = h[i]
                f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return x0 + f * (x1 - x0), y0 + f * (y1 - y0), a0 + f * (a1 - a0)
        return h[0][1:]


class E2EFusion(Planner):
    id = "e2e_fusion"
    name = "E2E 融合（LiDAR＋カメラ）"
    description = ("LiDAR・前方カメラ・車両状態から学習したモデルが出す前方の経路を追う。"
                   "地図も自己位置も使わない（記録を録って学習したコースを走る）")

    input_topic = TOPIC_E2E_FUSION_PLAN
    stale_ms = 300
    stats = ()

    params = (
        ParamSpec(key="max_speed", label="最高速度", min=0.2, max=5.0, step=0.1, default=1.0, unit="m/s",
                  note="これより速くは走らない。初めてのモデル・初めてのコースは低くして試す"),
        bool_param(key="use_model_speed", label="モデルの速度を使う", default=1.0,
                   note="ON: モデルが経路の各点に出した速度（手本の速度）を上限にする。"
                        "OFF: 経路の曲がり具合と車の限界だけで決める（速度の手本が無いコース向け）"),
        ParamSpec(key="b_max", hidden=True, label="経路の不確かさの上限", min=0.1, max=3.0, step=0.05,
                  default=0.8, unit="m", note="注視点までの経路点の不確かさがこれを超えたら走らない"),
        ParamSpec(key="look_k", hidden=True, label="注視距離の速度係数", min=0.1, max=1.5, step=0.05,
                  default=0.45, unit="s"),
        ParamSpec(key="look_min", hidden=True, label="注視距離の下限", min=0.2, max=1.5, step=0.05,
                  default=0.4, unit="m"),
        ParamSpec(key="speed_preview", hidden=True, label="速度の先読み", min=0.0, max=0.5, step=0.05,
                  default=0.1, unit="s"),
        ParamSpec(key="v_min", hidden=True, label="最低速度", min=0.1, max=1.5, step=0.05, default=0.4,
                  unit="m/s"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self.track = PoseTrack()
        self._sff = speedctl.SpeedFF()
        self._sff_cfg = speedctl.config_from_vehicle(
            self.vehicle, cmd_delay_s=0.5 / CMD_HZ + self._cmd_latency())
        self._sff_t = 0
        #: `tick()` が使う、今追っている経路と設定
        self._ctx: tuple[rl_mod.RaceLine, dict[str, float]] | None = None
        #: 今の経路の上で最後に居た点の添字。**最寄りの点はこの近くだけで探す**——経路が折り返して
        #: 自分の近くへ戻ってくる（ヘアピン・周回）と、全体から探した最寄りが経路の終わりに飛ぶ
        self._idx = 0

    def reset(self) -> None:
        self.track.reset()
        self._sff.reset()
        self._sff_t = 0
        self._ctx = None
        self._idx = 0

    def _cmd_latency(self) -> float:
        v = self.vehicle
        return v.control_latency_s if v.control_latency_s > 1e-4 else 0.02

    def on_vehicle_state(self, vs: VehicleState) -> None:
        """全サンプル（100Hz）で推測航法を進める。"""
        self.track.update(vs)

    # ── 経路 ──

    def build_path(self, plan: E2EFusionPlan, p: dict[str, float], v_now: float
                   ) -> tuple[rl_mod.RaceLine | None, str]:
        """推論結果 → 推測航法の座標の経路（速度つき）。作れなければ `(None, 理由)`。"""
        k = len(plan.path_x)
        if k < 3 or not (len(plan.path_y) == len(plan.path_v) == len(plan.path_b) == k) or plan.ds <= 0:
            return None, "経路点の形が不正"
        body = np.column_stack([plan.path_x, plan.path_y]).astype(np.float64)
        v_model = np.asarray(plan.path_v, dtype=np.float64)
        b = np.asarray(plan.path_b, dtype=np.float64)
        if not (np.isfinite(body).all() and np.isfinite(v_model).all() and np.isfinite(b).all()):
            return None, "推論結果が不正（NaN/Inf）"

        # 不確かさが上限を超える手前までを使う
        over = np.flatnonzero(b > p["b_max"])
        n_ok = int(over[0]) if len(over) else k
        need = max(_MIN_PATH_M, p["look_k"] * max(v_now, 0.0) + p["look_min"])
        if n_ok * plan.ds < need:
            return None, (f"経路に自信が無い（{n_ok * plan.ds:.1f}m 先で不確かさ {b[min(n_ok, k - 1)]:.2f}m。"
                          f"{need:.1f}m 先まで要る）")
        body, v_model = body[:n_ok], v_model[:n_ok]
        if body[min(n_ok - 1, 3), 0] <= 0.0:
            return None, "経路が後ろを向いている"

        # 撮影時刻の姿勢で推測航法の座標へ。先頭に撮影時刻の車の位置を足す（経路は車から始まる）
        pose0 = self.track.at(int(plan.t_capture))
        if pose0 is None:
            return None, "経路の基準時刻が推測航法の履歴より古い"
        c, s = math.cos(pose0[2]), math.sin(pose0[2])
        pts = np.vstack([[0.0, 0.0], body])
        xy = np.column_stack([pose0[0] + c * pts[:, 0] - s * pts[:, 1],
                              pose0[1] + s * pts[:, 0] + c * pts[:, 1]])
        kappa = rl_mod.curvature(xy, closed=False)
        seg = np.hypot(*np.diff(xy, axis=0).T)
        length = float(seg.sum())

        veh = self.vehicle
        v_max = p["max_speed"]
        v_min = min(p["v_min"], v_max)
        if p["use_model_speed"] > 0:
            cap = np.minimum(v_max, np.concatenate([[v_model[0]], v_model]))
            v_end = float(cap[-1])
        else:
            cap = None
            # 経路の先が見えていない。どんなコーナーが来ても曲がれる速度で終える
            v_end = min(v_max, math.sqrt(veh.a_lat_max / max(veh.kappa_max, 1e-6)))
        v = rl_mod.speed_profile(xy, kappa, v_max=v_max, v_min=v_min, a_lat=veh.a_lat_max,
                                 a_accel=veh.a_accel_max(), a_brake=veh.a_decel_max, closed=False,
                                 v_start=max(v_now, v_min), v_end=max(v_end, v_min), v_cap=cap)
        return rl_mod.RaceLine(xy=xy, v=np.asarray(v, dtype=np.float64), kappa=kappa, length=length,
                               closed=False), ""

    def _pursuit(self, p: dict[str, float]) -> PursuitConfig:
        veh = self.vehicle
        return PursuitConfig(wheelbase=veh.wheelbase, max_steer=veh.max_steer, lookahead_k=p["look_k"],
                             lookahead_min=p["look_min"], delay_s=veh.steer_delay_s + self._cmd_latency(),
                             speed_preview_s=p["speed_preview"], steer_map=veh.steer_map)

    def _command(self, path: rl_mod.RaceLine, p: dict[str, float], vs: VehicleState
                 ) -> tuple[float, float, float, float]:
        """今の姿勢で経路を追う。`(舵, 目標速度, ランプ, 横偏差)`。"""
        pose = self.track.pose
        pp = follow(path, pose, vs.speed, vs.steer_actual, self._pursuit(p), hint=self._idx)
        self._idx = pp.index
        j = nearest_index(path, pose[0], pose[1], pp.index, window=4)
        pos = speedctl.arc_index(path, j, pose[0], pose[1])
        t = int(vs.t_capture)
        dt = (t - self._sff_t) / NS if self._sff_t and t else 0.0
        self._sff_t = t
        if self.vehicle.speed_ff_firmware:
            lead = 0.5 / CMD_HZ + self._cmd_latency() + self.vehicle.speed_ff_filter_s
            speed, ramp = speedctl.ref_command(path, pos, vs.speed, lead_s=lead,
                                               catch_s=self.vehicle.speed_loop_tau_s)
        else:
            speed, ramp = self._sff.command(path, pos, vs.speed, dt, self._sff_cfg), 0.0
        return pp.steer, min(speed, p["max_speed"]), ramp, pp.cross_track

    # ── 判断 ──

    def plan(self, plan: E2EFusionPlan, vs: VehicleState | None, p: dict[str, float],
             dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)
        self._ctx = None
        if vs is None:
            st.reason = "車両状態がまだ届いていない"
            return st
        if not plan.ready:
            st.reason = plan.reason or "推論結果が無い"
            return st
        age = (int(vs.t_capture) - int(plan.t_capture)) / NS
        if age > _PLAN_MAX_AGE_S:
            st.reason = f"経路が古い（{age * 1e3:.0f}ms 前の入力）"
            return st

        path, why = self.build_path(plan, p, vs.speed)
        if path is None:
            st.reason = why
            return st

        self._idx = 0                      # 新しい経路は車の位置から始まる
        steer, speed, ramp, cross = self._command(path, p, vs)
        self._ctx = (path, p)

        st.ready = True
        st.target_steer = st.heading = steer
        st.target_speed = speed
        st.accel_limit = ramp
        st.cross_track = cross
        inputs = "＋".join(n for n, ok in (("画像", plan.has_image), ("点群", plan.has_lidar)) if ok)
        st.reason = (f"経路 {path.length:.1f}m を追従（{inputs}・推論 {plan.infer_ms:.0f}ms・"
                     f"遅れ {age * 1e3:.0f}ms）")
        return st

    def tick(self, vs: VehicleState | None) -> tuple[float, float | None, float] | None:
        """点群の合間（50Hz）に、同じ経路を今の姿勢で追い直す。"""
        if self._ctx is None or vs is None:
            return None
        path, p = self._ctx
        steer, speed, ramp, _ = self._command(path, p, vs)
        return steer, speed, ramp
