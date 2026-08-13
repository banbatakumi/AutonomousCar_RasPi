"""自動運転アルゴリズム（`docs/architecture.md` §8）。

**アルゴリズムだけを置く。** バス・WS・GUI のことは何も知らない純粋な計算に
してあり、`Scan` と `VehicleState` を渡すと `AutoState` が返る。テストで
合成した点群を流し込めるのはこのため（`raspi/tests/test_auto.py`）。

配線は `raspi/nodes/planning_node.py` の仕事。
"""

from .base import ParamSpec, Planner, sector_of_deg, wrap_deg
from .follow_the_gap import FollowTheGap
from .registry import PLANNERS, catalog, make_planner, merged_params

__all__ = [
    "Planner", "ParamSpec", "sector_of_deg", "wrap_deg",
    "FollowTheGap",
    "PLANNERS", "catalog", "make_planner", "merged_params",
]
