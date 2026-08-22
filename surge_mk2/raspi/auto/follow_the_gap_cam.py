"""Follow the Gap（カメラ）— `FollowTheGap` をカメラ由来の擬似スキャンで走らせる。

**`plan()` は一切上書きしない。** これは狙って作った制約で、「ギャップ探索の
ロジックはセンサを問わない」ことをコードそのもので示す。カメラ側で新しいのは
`cam_perception_node.py`（走行可能／不可能のセグメンテーション結果を
`raspi.nav.ipm` と `raspi.nav.grid.OccGrid.raycast()` で `Scan` と同じ形の
配列に変換する）だけで、ギャップの選び方・安全バブル・速度の決め方は
`follow_the_gap.py` から一切変えていない。

## `fov_deg` だけ上書きする理由

`FollowTheGap` の既定 `fov_deg=180`（LiDAR の前方ほぼ全周）のままだと、
カメラが実際に見ている範囲（`hfov`≒66°）の外は常に `sector_seen=False`
（＝壁）になり、視野の大半が塞がって `MIN_SEEN_RATIO`（0.6）を割り、
**常に「点群の欠測が多すぎる」で止まる planner** になってしまう。
GUI のスライダ上限もカメラが物理的に見える範囲までに制限しておく
（それ以上に広げても意味が無いことをスライダ自体が伝える）。

## `stale_ms` を伸ばす理由

`Planner.stale_ms` の既定 300ms は LiDAR の 10Hz を前提にした値
（`planning_node.py` の `SCAN_STALE_NS` の由来と同じ）。カメラ側の推論
（`cam_perception_node.py`）は Pi 5 の CPU で回すため LiDAR ほどの
ケイデンスを出せない前提で、余裕を持たせてある。実測後に調整する。
"""

from __future__ import annotations

from ..msgs.types import TOPIC_SCAN_CAM
from .base import ParamSpec
from .follow_the_gap import FollowTheGap

__all__ = ["FollowTheGapCam"]


class FollowTheGapCam(FollowTheGap):
    id = "ftg_cam"
    name = "Follow the Gap（カメラ）"
    description = ("カメラの走行可能／不可能セグメンテーションを距離配列に変換し、"
                   "FTGのギャップ探索をそのまま使う。LiDARが見えない低い壁向け")

    input_topic = TOPIC_SCAN_CAM
    stale_ms = 500

    #: `fov_deg` だけカメラの実視野に合わせて狭める。他は `FollowTheGap` と同じ
    params = tuple(
        ParamSpec(key="fov_deg", label="視野角", min=20, max=70, step=5,
                  default=60, unit="°",
                  note="カメラの実視野（hfov≈66°）より広げても見えない範囲。"
                       "cam_perception_node が publish する sector_seen の"
                       "範囲と合わせて調整する")
        if s.key == "fov_deg" else s
        for s in FollowTheGap.params
    )
