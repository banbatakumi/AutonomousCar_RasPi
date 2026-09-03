"""Cam E2E — 前方カメラの画像から操舵を直接回帰する模倣学習モデルで走る。

`ml_cam_e2e/train.py`（教師あり学習、人間の運転ログから直接ステアを回帰）で
学習したモデルを `raspi/nodes/cam_e2e_node.py` が推論し、その結果
（`CamE2ECmd`）を読むだけの薄い Planner。`cam_centerline.py`/`ftg_cam` が
セグメンテーション＋IPM（幾何変換）で回廊を作るのに対し、**ここは幾何変換を
一切経由しない**——低いカメラ高さでIPMの深度誤差が拡大する問題（実車で
「奥行きを掴むのが難しい」と分かった件）を、画像から操舵を直接学習する
ことで迂回するのがこの Planner の存在理由（`docs/development.md` §12.3）。

## 速度は回帰しない。LiDAR前方距離ベースの別ロジック

学習対象は操舵だけ（`ml_cam_e2e/dataset.py` 参照）。速度は
`cam_centerline.py` と同じ「前方余裕による減速＋曲率ベースの横加速度上限」
（⑥）をそのまま踏襲する——速度則を変える理由がセンサの違いだけでは無いため。

## 独立の安全策（`stop_dist`）

模倣学習は「学習データに無いパターン」への挙動を原理的に保証できない
（`e2e_lidar.py` と同じ理由）。`cam_e2e_node.py` が同梱する LiDAR 前方距離
（`CamE2ECmd.lidar_front_dist`）を見て、**正面の余裕が閾値を切ったら
モデル出力を無視して止める。** 低い壁には効かない（そもそもLiDARが
見えない）が、それ以外の一般障害物への最後の砦として残す。
`lidar_seen=False`（LiDARが届いていない）も同じ扱いにする——「分からなければ
止まる」という他の Planner と同じ安全側の判断。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import TOPIC_CAM_E2E_CMD, AutoState, CamE2ECmd, VehicleState
from .base import ParamSpec, Planner

__all__ = ["CamE2E"]


class CamE2E(Planner):
    id = "cam_e2e"
    name = "E2Eカメラ（模倣学習）"
    description = ("前方カメラの画像から操舵を直接回帰する模倣学習モデルで走る。"
                   "IPMなどの幾何変換を経由しない。LiDAR前方距離を独立の安全策として使う")

    #: `cam_e2e_node.py` が publish する推論結果を使う
    input_topic = TOPIC_CAM_E2E_CMD
    #: カメラ側の推論ケイデンスに余裕を持たせる（`cam_centerline.py` と同じ理由）
    stale_ms = 500
    #: 地図もギャップ探索も持たないので `nearest`/`gap` は書かない
    stats = ("free_ahead", "valid_ratio")

    params = (
        ParamSpec(key="stop_dist", label="停止する前方距離", min=0.1, max=1.5, step=0.01,
                  default=0.35, unit="m",
                  note="★独立した安全策。モデルの判断を経由せず、LiDARで見た正面の"
                       "余裕がこれを切ったら無条件で停止する"),
        ParamSpec(key="slow_dist", label="減速を始める距離", min=0.3, max=5.0, step=0.05,
                  default=1.5, unit="m",
                  note="前方余裕がこれ以下で最高速度から最低速度へ線形に落とす"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=1.5, step=0.01,
                  default=0.30, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(key="min_speed", label="最低速度", min=0.0, max=1.0, step=0.01,
                  default=0.10, unit="m/s",
                  note="減速しきってもこれ以下にはしない。0 にすると詰まった所で動けなくなる"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m/s²",
                  note="★実車未計測の暫定値。実際に切る舵角から曲率 κ=tan(δ)/L を求め、"
                       "v ≤ sqrt(これ/κ) で速度を抑える（`cam_centerline.py` と同じ式）"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.10, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが反応が鈍る"
                       "（`e2e_lidar.py`/`cam_centerline.py` と同じ仕組み）"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._steer = 0.0

    def reset(self) -> None:
        self._steer = 0.0

    def plan(self, cmd: CamE2ECmd, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)

        if not cmd.ready:
            st.reason = "モデル未選択、または推論に失敗"
            return st                      # ready=False ＝ 制動

        free_ahead = cmd.lidar_front_dist if cmd.lidar_seen else 0.0
        st.free_ahead = free_ahead
        st.valid_ratio = 1.0 if cmd.lidar_seen else 0.0

        stop_d = p["stop_dist"]
        if not cmd.lidar_seen or free_ahead <= stop_d:
            # ★独立した安全策。モデルの判断を経由しない「事実」としての正面の余裕
            # （またはそもそも LiDAR が届いていない）で無条件停止する
            st.ready = True
            st.brake = True
            st.target_speed = 0.0
            st.target_steer = self._steer
            st.reason = ("LiDARが届いていない" if not cmd.lidar_seen else
                        f"前方 {free_ahead * 100:.0f}cm で停止"
                        f"（安全策・停止距離 {stop_d * 100:.0f}cm）")
            return st

        st.ready = True

        # ── 操舵。モデル出力（正規化値）を契約の max_steer で物理量に戻す ──
        # `model_max_steer` はエクスポート時の契約値（`ml_cam_e2e/export_onnx.py` が
        # 書く `max_steer`）。0 のまま届くことは無い想定だが、念のため車両限界に倒す
        model_max_steer = cmd.model_max_steer if cmd.model_max_steer > 0 else self.vehicle.max_steer
        steer_norm = max(-1.0, min(1.0, cmd.steer_norm))
        raw_steer = steer_norm * model_max_steer
        max_steer = self.vehicle.max_steer
        target = max(-max_steer, min(max_steer, raw_steer))

        # 時間ベースの1次遅れ（`cam_centerline.py`/`e2e_lidar.py` と同じ式）
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── 速度。前方余裕による減速 ＋ 曲率ベースの横加速度上限（`cam_centerline.py`⑥と同じ式） ──
        slow_d = max(p["slow_dist"], stop_d + 0.01)
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)
        ratio = min(1.0, (free_ahead - stop_d) / (slow_d - stop_d))
        v = v_min + (v_max - v_min) * ratio

        kappa = abs(math.tan(target) / self.vehicle.wheelbase)
        v_curve = math.sqrt(p["a_lat_max"] / kappa) if kappa > 1e-6 else math.inf
        st.target_speed = max(v_min, min(v, v_curve, v_max))

        st.reason = (f"モデル出力 {math.degrees(target):+.0f}°・"
                     f"前方 {free_ahead:.2f}m")
        return st
