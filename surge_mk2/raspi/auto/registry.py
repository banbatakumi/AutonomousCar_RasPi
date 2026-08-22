"""自動運転モードの一覧。**GUI の選択肢はここから自動で生える。**

新しい走らせ方を足す手順は2つだけ:

    1. `raspi/auto/<名前>.py` に `Planner` のサブクラスを書く
    2. 下の `PLANNERS` に 1 行足す

GUI・WS・planning_node のどれも触らなくてよい。モード選択もパラメータの
スライダも `catalog()` が返す宣言から組み立てられる（`gui/src/components/AutoPanel.tsx`）。

**id は保存キーでもある。** `config/auto.json` と `localStorage` がこの文字列で
パラメータを覚えているので、後から変えると設定が黙って既定値に戻る。
"""

from __future__ import annotations

from .base import Planner
from .disparity_extender import DisparityExtender
from .follow_the_gap import FollowTheGap
from .follow_the_gap_cam import FollowTheGapCam
from .gap_pursuit import DisparityPursuit
from .raceline import RaceLine

__all__ = ["PLANNERS", "catalog", "make_planner", "merged_params"]

#: id → クラス。**並び順がそのまま GUI の並び順**
PLANNERS: dict[str, type[Planner]] = {
    FollowTheGap.id: FollowTheGap,
    FollowTheGapCam.id: FollowTheGapCam,
    DisparityExtender.id: DisparityExtender,
    DisparityPursuit.id: DisparityPursuit,
    RaceLine.id: RaceLine,
}


def catalog() -> list[dict]:
    """GUI に渡す宣言一式（JSON にそのまま落ちる形）。"""
    return [
        {
            "id": cls.id,
            "name": cls.name,
            "description": cls.description,
            "params": [
                {"key": s.key, "label": s.label, "min": s.min, "max": s.max,
                 "step": s.step, "default": s.default, "unit": s.unit, "note": s.note}
                for s in cls.params
            ],
        }
        for cls in PLANNERS.values()
    ]


def make_planner(mode: str) -> Planner | None:
    """id から planner を作る。**知らない id は None**（例外にしない）。

    古い `config/auto.json` や、planner を消したあとの GUI から知らない id が
    来ても、planning_node が落ちるのではなく「自動運転しない」に倒れるべき。
    """
    cls = PLANNERS.get(mode)
    return cls() if cls is not None else None


def merged_params(mode: str, user: dict[str, float]) -> dict[str, float]:
    """既定値にユーザー設定を重ねてクランプしたもの。知らない id なら空 dict。"""
    cls = PLANNERS.get(mode)
    return cls.merged(user) if cls is not None else {}
