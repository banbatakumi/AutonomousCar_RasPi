"""Cam E2E — 前方カメラの画像から操舵を直接回帰する模倣学習モデルで走る。

`ml_cam_e2e/train.py`（教師あり学習、人間の運転ログから操舵を回帰）で
学習したモデルを `raspi/nodes/cam_e2e_node.py` が推論し、その結果
（`CamE2ECmd`）を読むだけの薄い Planner。`cam_centerline.py`/`ftg_cam` が
セグメンテーション＋IPM（幾何変換）で回廊を作るのに対し、**ここは幾何変換を
一切経由しない**——低いカメラ高さでIPMの深度誤差が拡大する問題（実車で
「奥行きを掴むのが難しい」と分かった件）を、画像から操作を直接学習する
ことで迂回するのがこの Planner の存在理由（`docs/development.md` §12.3）。

## カメラだけで走る。速度は舵から決める（2026-10-07）

LiDAR は見ない。速度はモデルが出した舵角の曲率 `κ=tan(δ)/L` から
`v ≤ sqrt(a_lat_max/κ)` で決める（`cam_centerline.py`/`follow_the_gap.py` と
同じ式）。直線では `max_speed`、舵を切るほど遅くなる。

2026-10-06 の時点では速度も人の運転から学んでいたが、やめた。手本は舵に
集中してゆっくり走った方が質が上がり、そうすると速度のラベルは「遅い一定値」
になって情報が無く、しかも手本の遅さがそのまま実車の上限になるため。
**モデルの `speed_norm` はここでは使わない**（出力の契約には残してある）。

★ 舵が切れてから遅くなるので、コーナー手前の減速は遅れる。足りなければ
先の経路を出すモデルへ進む（`docs/development.md` §12.3）。

## 緊急停止は持たない

模倣学習は「学習データに無いパターン」への挙動を原理的に保証できない
（`e2e_lidar.py` と同じ理由）。そこを受けるのは STM32 の `auto_stop`
（速度に応じて伸びる動的停止距離。`follow_the_gap.py` docstring参照。
GUI の設定がそのまま通るので、このモードでも ON にしておけば効く）と、
人が舵・スロットル・ブレーキに触れた瞬間の解除。planner 側に固定距離の
停止を重ねて持たない方針は他のモードと同じ（2026-09-12）。
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
    description = ("前方カメラの画像だけから操舵を直接回帰する模倣学習モデルで走る。"
                   "速度は舵の切れ具合から決める。IPMなどの幾何変換もLiDARも使わない")

    #: `cam_e2e_node.py` が publish する推論結果を使う
    input_topic = TOPIC_CAM_E2E_CMD
    #: カメラ側の推論ケイデンスに余裕を持たせる（`cam_centerline.py` と同じ理由）
    stale_ms = 500
    #: 地図もギャップ探索も距離も持たないので判断欄に出す数値が無い
    stats = ()

    params = (
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=3.0, step=0.01,
                  default=0.30, unit="m/s",
                  note="直線で出す速度。初めてのモデルは低くして試す。"
                       "★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(key="min_speed", label="最低速度", min=0.0, max=1.0, step=0.01,
                  default=0.10, unit="m/s",
                  note="舵をいっぱいに切ってもこれ以下にはしない"),
        ParamSpec(key="a_lat_max", label="旋回時の横加速度上限", min=0.5, max=8.0, step=0.1,
                  default=3.0, unit="m/s²",
                  note="★実車未計測の暫定値。モデルが出した舵角から曲率 κ=tan(δ)/L を求め、"
                       "v ≤ sqrt(これ/κ) で速度を抑える（`FollowTheGap` と同じ式）。"
                       "下げるほどコーナーで遅くなる"),
        ParamSpec(key="steer_gain", label="舵の倍率", min=0.5, max=2.0, step=0.05,
                  default=1.0, unit="",
                  note="モデルが出した舵角に掛ける。回帰は平均へ寄って舵が浅くなりがちなので、"
                       "コーナーで膨らむなら上げる"),
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

        # **NaN/Inf はここで弾く（issue #1）。** `max(-1, min(1, nan))` は Python の
        # 仕様で常に +1 を返すため、`steer_norm` が NaN だと「最大舵角」に
        # 化ける——下流の `isfinite` チェックは通過してしまう（NaN のまま
        # 伝播しないため）ので、ここで潰す必要がある。`speed_norm` は使わないが、
        # 片方だけ壊れた推論結果の舵を信じる理由も無いので一緒に見る
        if not (math.isfinite(cmd.steer_norm) and math.isfinite(cmd.speed_norm)):
            st.reason = "推論結果が不正（NaN/Inf）"
            return st                      # ready=False ＝ 制動

        # 契約値（`ml_cam_e2e/export_onnx.py` が書く）が無いと物理量に戻せない。
        # `cam_e2e_node.load_model()` が読み込み時に弾くので通常は来ない
        if not (math.isfinite(cmd.model_max_steer) and cmd.model_max_steer > 0):
            st.reason = "モデルの契約値（max_steer）が不正"
            return st                      # ready=False ＝ 制動

        st.ready = True

        # ── 操舵。モデル出力（正規化値）を契約の max_steer で物理量に戻す ──
        steer_norm = max(-1.0, min(1.0, cmd.steer_norm))
        raw_steer = steer_norm * cmd.model_max_steer * p["steer_gain"]
        max_steer = self.vehicle.max_steer
        target = max(-max_steer, min(max_steer, raw_steer))

        # 時間ベースの1次遅れ（`cam_centerline.py`/`e2e_lidar.py` と同じ式）
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer

        # ── 速度。モデルの出力は使わず、舵の曲率から決める ──
        # 平滑化する前の `target` で見る（`cam_centerline.py` と同じ）。
        # 平滑化後の舵だと、切り始めの減速がさらに `steer_tau` ぶん遅れる
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)
        kappa = abs(math.tan(target) / self.vehicle.wheelbase)
        v_curve = math.sqrt(p["a_lat_max"] / kappa) if kappa > 1e-6 else math.inf
        st.target_speed = max(v_min, min(v_curve, v_max))

        limited = "（旋回で減速）" if v_curve < v_max else ""
        st.reason = (f"モデル出力 舵 {math.degrees(target):+.0f}°・"
                     f"速度 {st.target_speed:.2f}m/s{limited}")
        return st
