"""Line Trace — カメラで検出した白線を Pure Pursuit で追従する。

`line_perception_node.py` が前方カメラの白線を検出し、地面座標に逆投影した
2点（近傍・遠方）を `LineScan` として publish する。ここでは遠方点を
（無ければ近傍点を）Pure Pursuit の目標点として `follow_the_gap.py` と同じ
`nav.purepursuit.steer_for_target()` に渡すだけで、白線を追う舵角が出る——
白線の追従も「自分（後輪車軸）から見た目標点の方位と距離」という Pure
Pursuit の入力形に落とし込める点は、ギャップの中央を狙うのと同じ構造なので、
独自の制御則（横偏差＋方位誤差の PID 等）を新設せずに済む。

## 白線を見失ったら止まる

`ready=False` は必ず制動に読み替えられる（`Planner` の約束2・3）。白線が
画角の外に出た・かすれて消えた等で `seen=False`（または検出割合が低すぎる）
周期は、直前の舵角のまま惰行させず**止める**。コースアウトの方が停止よりずっと
危険なため。

## 地図もLiDARも使わない

`ftg`/`ftg_cam` と違って障害物回避は一切しない。白線の上を辿るだけの
最小構成——STM32 側の超音波 `auto_stop`（20cm）と組み合わせて使うことを
前提にしている（`DriveCmd.auto_stop` は既定 False のままなので、有効化は
別途 GUI 側の対応が要る。★今後の課題）。
"""

from __future__ import annotations

import math

from ..core.vehicle import Vehicle
from ..msgs.types import TOPIC_LINE_CAM, AutoState, LineScan, VehicleState
from ..nav.purepursuit import steer_for_target
from .base import ParamSpec, Planner

__all__ = ["LineTrace"]


class LineTrace(Planner):
    id = "line_trace"
    name = "ライントレース（カメラ）"
    description = "カメラで検出した白線をPure Pursuitで追従する。地図もLiDARも使わない"

    #: `line_perception_node.py` が publish する擬似目標点を使う
    input_topic = TOPIC_LINE_CAM
    #: カメラ側の推論ケイデンスに余裕を持たせる（`follow_the_gap_cam.py` と同じ理由）
    stale_ms = 500

    params = (
        ParamSpec(key="min_coverage", label="最小検出割合", min=0.0, max=0.2, step=0.005,
                  default=0.01, unit="",
                  note="ROI内で白と判定した画素の割合がこれ未満なら「見失った」として停止する"),
        ParamSpec(key="look_k", label="前方注視の速度係数", min=0.0, max=2.0, step=0.05,
                  default=0.7, unit="s",
                  note="Ld = 係数×速度 + 最小値。上げると滑らかだがコーナーで曲がりきれなくなる"),
        ParamSpec(key="look_min", label="前方注視の最小値", min=0.15, max=1.5, step=0.05,
                  default=0.35, unit="m",
                  note="低速時の注視距離。小さすぎると舵が振動する"),
        ParamSpec(key="max_speed", label="最高速度", min=0.05, max=1.5, step=0.01,
                  default=0.30, unit="m/s",
                  note="★io_node の --max-speed を超えても Pi 側で切り捨てられるだけ"),
        ParamSpec(key="min_speed", label="最低速度", min=0.0, max=1.0, step=0.01,
                  default=0.10, unit="m/s",
                  note="旋回中でもこれ以下にはしない。0 にすると急カーブで詰まって動けなくなる"),
        ParamSpec(key="max_steer", label="最大舵角", min=0.1, max=0.524, step=0.005,
                  default=0.50, unit="rad",
                  note="★io_node の --max-steer を超えても切り捨てられるだけ"),
        ParamSpec(key="turn_slow", label="旋回時の減速", min=0.0, max=1.0, step=0.05,
                  default=0.60, unit="",
                  note="舵角いっぱいで速度をこの割合ぶん落とす。1.0 で全舵時に停止"),
        ParamSpec(key="steer_tau", label="舵の平滑化", min=0.0, max=0.5, step=0.01,
                  default=0.10, unit="s",
                  note="舵指令の1次遅れの時定数。0 で平滑化なし。上げると滑らかだが反応が鈍る"),
    )

    def __init__(self) -> None:
        self.vehicle = Vehicle.load()
        self._steer = 0.0

    def reset(self) -> None:
        # **モード切替・disengage のたびに呼ばれる。** 残しておくと、次に engage
        # した瞬間に前回の舵の続きから動き出す
        self._steer = 0.0

    def plan(self, line: LineScan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        st = AutoState(mode=self.id, planner=self.name)
        st.valid_ratio = line.coverage

        if not line.seen or line.coverage < p["min_coverage"]:
            st.reason = f"白線を見失った（検出割合 {line.coverage * 100:.1f}%）"
            return st                      # ready=False ＝ 制動

        # ── 目標点はまず遠方帯。無ければ近傍帯（`LineScan` の docstring） ──
        if line.far_seen:
            tx, ty = line.far_x, line.far_y
        else:
            tx, ty = line.near_x, line.near_y

        dist = math.hypot(tx, ty)
        if dist < 1e-3:
            st.reason = "目標点が近すぎる（車両の真下付近）"
            return st                      # ready=False ＝ 制動

        eta = math.atan2(ty, tx)
        st.heading = eta
        st.target_x = tx
        st.target_y = ty

        # ── Pure Pursuit。`follow_the_gap.py` の⑤と同じ式 ──
        max_steer = p["max_steer"]
        v_now = vs.speed if vs is not None else 0.0
        ld = min(dist, p["look_k"] * v_now + p["look_min"])
        target = steer_for_target(eta, ld, self.vehicle.wheelbase, max_steer)
        tau = p["steer_tau"]
        alpha = 1.0 if tau <= 1e-3 or dt <= 0 else 1.0 - math.exp(-dt / tau)
        self._steer += (target - self._steer) * alpha
        st.target_steer = self._steer
        st.ready = True

        # ── 速度。旋回が大きいほど落とす（`follow_the_gap.py` の⑥と同じ考え方） ──
        v_max = p["max_speed"]
        v_min = min(p["min_speed"], v_max)
        turn = abs(st.target_steer) / max_steer if max_steer > 0 else 0.0
        v = v_max - (v_max - v_min) * p["turn_slow"] * min(1.0, turn)
        st.target_speed = max(v_min, v)

        st.reason = (f"白線 {math.degrees(eta):+.0f}°・{dist:.2f}m 先へ"
                     f"（検出割合 {line.coverage * 100:.0f}%）")
        return st
