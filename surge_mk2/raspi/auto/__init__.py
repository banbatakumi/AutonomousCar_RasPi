"""自動運転アルゴリズム（`docs/architecture.md` §8）。

**アルゴリズムだけを置く。** バス・WS・GUI のことは何も知らない純粋な計算に
してあり、`Scan` と `VehicleState` を渡すと `AutoState` が返る。テストで
合成した点群を流し込めるのはこのため（`raspi/tests/test_auto.py`）。

配線は `raspi/nodes/planning_node.py` の仕事。
"""

from .base import (ParamSpec, Planner, ScanWindow, min_filter, scan_window,
                   sector_of_deg, wrap_deg)
from .disparity_extender import DisparityExtender
from .e2e_lidar import E2ELidar
from .follow_the_gap import FollowTheGap
from .follow_the_gap_cam import FollowTheGapCam
from .gap_pursuit import DisparityPursuit
from .line_trace import LineTrace
from .raceline import RaceLine
from .registry import PLANNERS, catalog, make_planner, merged_params

__all__ = [
    "Planner", "ParamSpec", "ScanWindow", "min_filter", "scan_window",
    "sector_of_deg", "wrap_deg",
    "DisparityExtender", "E2ELidar", "FollowTheGap", "FollowTheGapCam", "DisparityPursuit",
    "RaceLine", "LineTrace",
    "PLANNERS", "catalog", "make_planner", "merged_params",
]
