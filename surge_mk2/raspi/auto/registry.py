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
from .cam_centerline import CamCenterline
from .cam_e2e import CamE2E
from .disparity_extender import DisparityExtender
from .e2e_lidar import E2ELidar
from .follow_object import FollowObject
from .follow_the_gap import FollowTheGap
from .follow_the_gap_cam import FollowTheGapCam
from .gap_pursuit import DisparityPursuit
from .line_trace import LineTrace
from .park_to_point import ParkToPoint
from .slam2d_raceline import Slam2dRaceLine
from .slam2d_route import Slam2dRoute
from .calib_encoder import CalibEncoder
from .calib_latency import CalibLatency
from .calib_speed import CalibSpeed
from .calib_speed_check import CalibSpeedCheck
from .calib_steer import CalibSteer
from .calib_steer_dyn import CalibSteerDyn
from .calib_traction_check import CalibTractionCheck
from .calib_tyre import CalibTyre
from .calib_wheel import CalibWheel

__all__ = ["PLANNERS", "catalog", "make_planner", "merged_params"]

#: id → クラス。**並び順がそのまま GUI の並び順**
PLANNERS: dict[str, type[Planner]] = {
    FollowTheGap.id: FollowTheGap,
    FollowTheGapCam.id: FollowTheGapCam,
    CamCenterline.id: CamCenterline,
    CamE2E.id: CamE2E,
    DisparityExtender.id: DisparityExtender,
    DisparityPursuit.id: DisparityPursuit,
    Slam2dRaceLine.id: Slam2dRaceLine,
    Slam2dRoute.id: Slam2dRoute,
    LineTrace.id: LineTrace,
    E2ELidar.id: E2ELidar,
    FollowObject.id: FollowObject,
    ParkToPoint.id: ParkToPoint,
    #: システム同定タブ専用（`Planner.category == "sysid"`）。自動運転タブのモード選択には出さない
    #: （`AutoPanel.tsx` 側のフィルタ）。車両の校正（`docs/calibration.md`）の試験で、並びは校正の順番
    CalibEncoder.id: CalibEncoder,
    CalibSteer.id: CalibSteer,
    CalibSteerDyn.id: CalibSteerDyn,
    CalibLatency.id: CalibLatency,
    CalibSpeed.id: CalibSpeed,
    CalibSpeedCheck.id: CalibSpeedCheck,
    CalibWheel.id: CalibWheel,
    CalibTyre.id: CalibTyre,
    CalibTractionCheck.id: CalibTractionCheck,
}


def catalog() -> list[dict]:
    """GUI に渡す宣言一式（JSON にそのまま落ちる形）。"""
    return [
        {
            "id": cls.id,
            "name": cls.name,
            "description": cls.description,
            "category": cls.category,
            "params": [
                {"key": s.key, "label": s.label, "min": s.min, "max": s.max,
                 "step": s.step, "default": s.default, "unit": s.unit, "note": s.note,
                 "group": s.group}
                for s in cls.params if not s.hidden
            ],
            "stats": list(cls.stats),
            #: 地図パネル・地図ライブラリを出すか（SLAM 地図を持つ planner）、
            #: 経由点エディタと経路切替ボタンを出すか（`slam2d_route`）
            "map_ui": bool(getattr(cls, "map_ui", False)),
            "routes_ui": bool(getattr(cls, "routes_ui", False)),
        }
        for cls in PLANNERS.values() if cls.listed
    ]


def make_planner(mode: str) -> Planner | None:
    """id から planner を作る。**知らない id は None**（例外にしない）。

    古い `config/auto.json` や、planner を消したあとの GUI から知らない id が
    来ても、planning_node が落ちるのではなく「自動運転しない」に倒れるべき。
    """
    cls = PLANNERS.get(mode)
    return cls() if cls is not None else None


def merged_params(mode: str, user: dict[str, float], hidden: bool = False) -> dict[str, float]:
    """既定値にユーザー設定を重ねてクランプしたもの。知らない id なら空 dict。

    `hidden` は `Planner.merged` を参照（GUI から隠した項目も上書きするか。`sim.bench` 用）。"""
    cls = PLANNERS.get(mode)
    return cls.merged(user, hidden) if cls is not None else {}
