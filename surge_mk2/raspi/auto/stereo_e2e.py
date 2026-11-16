"""ステレオ E2E — 前方ステレオの左右の画像＋視差の帯＋車両状態から学習したモデルが出す**前方の経路点**を追って走る。

**LiDAR を使わない。** 推論は `raspi/nodes/stereo_e2e_node.py`（独立プロセス）、学習は `ml_stereo_e2e/`
（LiDAR で走った `slam2d_route` の記録を手本にした模倣）。経路点の受け取り方・推測航法でのずらし方・速度の決め方・
止まる条件は `e2e_fusion.py` と同じなので継承する（入力の `E2EFusionPlan` も同じ型）。違いは:

- 入力のトピックが `stereo_e2e/plan`
- **追従の既定が MPC**（`nav/mpc.py`。経路の曲率を打ち消す舵を基準に、先の経路まで見て舵を最適化する）
- `E2EFusionPlan.has_lidar` は「視差を使ったか」の意味（推論ノードは視差の無い周は推論しないので、
  `ready` の経路では常に視差を使っている。表示だけ「視差」にする）

固定距離の停止（ステレオの障害物での減速）は持たない（2026-11-16 のバンビの判断。STM32 の `auto_stop` と、人が触れた瞬間の解除で受ける）。
"""

from __future__ import annotations

import msgspec

from ..msgs.types import TOPIC_STEREO_E2E_PLAN
from .base import ParamSpec
from .e2e_fusion import E2EFusion

__all__ = ["StereoE2E"]


def _with_default(params: tuple[ParamSpec, ...], key: str, default: float) -> tuple[ParamSpec, ...]:
    out = tuple(msgspec.structs.replace(p, default=default) if p.key == key else p for p in params)
    if not any(p.key == key for p in out):
        raise KeyError(key)
    return out


class StereoE2E(E2EFusion):
    id = "stereo_e2e"
    name = "ステレオ E2E（カメラだけ）"
    description = ("前方ステレオの左右の画像と視差から学習したモデルが出す前方の経路を MPC で追う。"
                   "LiDAR・地図・自己位置を使わない（記録を録って学習したコースを走る）")

    input_topic = TOPIC_STEREO_E2E_PLAN
    second_input = "視差"
    params = _with_default(E2EFusion.params, "ctrl_mode", 1.0)
