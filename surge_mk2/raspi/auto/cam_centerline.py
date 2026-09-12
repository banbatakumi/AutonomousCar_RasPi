"""Cam Centerline — カメラの走行可能領域の中心線を Pure Pursuit で追従する。

`cam_perception_node.py` が前方カメラのセグメンテーションマスクから作る
`CamPath`（前方複数距離ごとの回廊中心・幅の点列）を使う。`ftg_cam` が
「車両原点から見た角度ごとの距離」という1次元表現（`scan/cam`）に潰した後の
値を使うのに対し、こちらはマスクの2次元的な広がり（回廊の幅・中心が前方距離
ごとにどう変わるか）を保ったまま使う（`raspi/nav/drivable_path.py` の docstring参照）。

## 幅の閾値はここで初めて効く

`CamPath.widths` は生の測定値のまま届く（`CamPath` の docstring）。ここで
`min_width_m`（車幅＋余裕、GUI 調整可能）を下回った最初の距離で経路を打ち切り、
それより手前だけを使う——`FollowTheGap` が `Scan.dist`（生の距離配列）に
`gap_min` を適用するのと同じ役割分担（`Planner` の約束4）。

## 目標点の選び方・速度は `FollowTheGap`/`LineTrace` と同じ式

狙点は `Ld = look_k*v + look_min`（`free_ahead` で頭打ち）の位置を中心線から
補間した点。舵角は `nav.purepursuit.steer_for_target()`。速度は前方余裕
（`free_ahead`）ベースの減速＋実舵角から求めた曲率による横加速度上限——
`follow_the_gap.py` の⑥をそのまま踏襲する（センサが違うだけで速度則を
変える理由が無い）。

## 経路が短すぎたら止まる

`LineTrace` の「白線を見失ったら止まる」と同じ思想。最初の点（`path.xs[0]`）
から既に幅が足りない、または `CamPath` 自体が届いていない（`seen=False`）
周期は `ready=False`——直前の舵のまま惰行させず止める。

## 緊急停止は持たない（2026-09-12）

`follow_the_gap.py` と同じ理由で、正面距離しきい値によるハード停止（旧
`stop_dist`）は撤去し STM32 の `auto_stop` に任せる。前方余裕による減速
（⑥）はそのまま残る——緊急停止ではなく走行そのものの一部のため。
"""

from __future__ import annotations

import math

import numpy as np

from ..core.vehicle import Vehicle
from ..msgs.types import TOPIC_CAM_PATH, AutoState, CamPath, VehicleState
from ..nav.purepursuit import steer_for_target
from .base import ParamSpec, Planner

__all__ = ["CamCenterline"]


class CamCenterline(Planner):
    id = "cam_centerline"
    name = "中心線追従（カメラ）"
    description = ("カメラの走行可能領域セグメンテーションから前方複数距離の回廊中心を作り、"
                   "Pure Pursuitで追従する。scan/camを経由しないぶんマスクの形状を直接使う")

    #: `cam_perception_node.py` が publish する中心線点列を使う
    input_topic = TOPIC_CAM_PATH
    #: カメラ側の推論ケイデンスに余裕を持たせる（`follow_the_gap_cam.py` と同じ理由）
    stale_ms = 500
    #: 地図もLiDARも使わないので `nearest`/`gap` は書かない
    stats = ("free_ahead", "valid_ratio")

    params = (
        ParamSpec(key="min_width_m", label="最小通行幅", min=0.15, max=1.5, step=0.01,
                  default=0.40, unit="m",
                  note="左右合計の空き幅がこれを下回った距離より先は使わない。"
                       "車幅＋余裕から決める"),
        ParamSpec(key="look_k", label="前方注視の速度係数", min=0.0, max=2.0, step=0.05,
                  default=0.7, unit="s",
                  note="Ld = 係数×速度 + 最小値。上げると滑らかだがコーナーで曲がりきれなくなる"),
        ParamSpec(key="look_min", label="前方注視の最小値", min=0.15, max=1.5, step=0.05,
                  default=0.35, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
        ParamSpec(key="slow_dist", label="減速を始める距離", min=0.3, max=5.0, step=0.05,
                  default=1.5, unit="m",
                  note="free_ahead がこれ以下で最高速度から最低速度へ線形に落とす。"
                       "0m（接触寸前）で最低速度になる"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=3.0, step=0.01,
                  default=0.30, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(key="min_speed", label="最低速度", min=0.0, max=1.0, step=0.01,
                  default=0.10, unit="m/s",
                  note="減速しきってもこれ以下にはしない。0 にすると詰まった所で動けなくなる"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m/s²",
                  note="★実車未計測の暫定値。実際に切る舵角から曲率 κ=tan(δ)/L を求め、"
                       "v ≤ sqrt(これ/κ) で速度を抑える（`FollowTheGap` と同じ式）"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.10, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが反応が鈍る"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._steer = 0.0

    def reset(self) -> None:
        self._steer = 0.0

    def plan(self, path: CamPath, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        if not path.seen or not path.xs:
            st.reason = "走行可能領域のマスクが届いていない"
            return st                      # ready=False ＝ 制動

        xs = np.asarray(path.xs)
        ys = np.asarray(path.ys)
        widths = np.asarray(path.widths)
        ok = widths >= p["min_width_m"]

        if not ok[0]:
            st.reason = (f"直前（{xs[0]:.2f}m）の通行幅が足りない"
                         f"（{widths[0]:.2f}m ＜ {p['min_width_m']:.2f}m）")
            return st                      # ready=False ＝ 制動

        # ── 幅が足りなくなった最初の距離で打ち切る（それ以遠は使わない） ──
        k = len(xs) if ok.all() else int(np.argmin(ok))
        xs, ys = xs[:k], ys[:k]
        free_ahead = float(xs[-1])
        st.free_ahead = free_ahead
        st.valid_ratio = k / len(path.xs)
        st.ready = True

        # ── 狙点。free_ahead で頭打ちにした Ld の位置を中心線から補間する ──
        max_steer = self.vehicle.max_steer
        v_now = vs.speed if vs is not None else 0.0
        ld = min(free_ahead, p["look_k"] * v_now + p["look_min"])
        y_target = float(np.interp(ld, xs, ys))
        eta = math.atan2(y_target, ld)
        st.heading = eta
        st.target_x = ld
        st.target_y = y_target

        target = steer_for_target(eta, ld, self.vehicle.wheelbase, max_steer)
        # 時間ベースの1次遅れ。フレームレートに依存させない（`follow_the_gap.py`と同じ）
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── 速度。前方余裕による減速 ＋ 曲率ベースの横加速度上限（⑥と同じ式） ──
        slow_d = max(p["slow_dist"], 1e-3)
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)
        ratio = max(0.0, min(1.0, free_ahead / slow_d))
        v = v_min + (v_max - v_min) * ratio

        kappa = abs(math.tan(target) / self.vehicle.wheelbase)
        v_curve = math.sqrt(p["a_lat_max"] / kappa) if kappa > 1e-6 else math.inf
        st.target_speed = max(v_min, min(v, v_curve, v_max))

        st.reason = (f"中心線 {math.degrees(eta):+.0f}°・{ld:.2f}m 先へ・"
                     f"前方 {free_ahead:.2f}m")
        return st
