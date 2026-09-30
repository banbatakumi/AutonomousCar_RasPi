"""自己位置推定・地図・経路生成の部品（`docs/architecture.md` §14 Phase 3/4）。

**ここは純粋計算だけ。** バスも WebSocket も GUI も設定ファイルも知らない
（`raspi/auto/__init__.py` の約束と同じ）。入力は `raspi/msgs/types.py` の
メッセージと numpy 配列、出力も数値だけ。

    deskew      点群のモーションスキュー補正（走りながら測った1周を1瞬に直す）
    grid        占有格子。ヒット/ミスの回数を数え、動く物を壁として確定させない
    raceline    最小曲率最適化と速度プロファイル（車体非依存の共通ライブラリ）

**SLAM 本体は `slam2d/`（車体非依存ライブラリ）に統合済み。** ここにあった
自作の `scanmatch`/`slam`（旧 `auto/raceline.py` 用）は学習用の基礎基盤で、
現行の `slam2d_raceline`/`slam2d_route` planner には使われておらず削除した。
"""

from __future__ import annotations

from . import centerline, obstacles
from .centerline import Centerline
from .deskew import Points, deskew
from .drivable_path import extract_centerline
from .grid import OccGrid, dilate, pack_trinary
from .ipm import (CameraExtrinsics, CameraIntrinsics, camera_intrinsics,
                  ground_to_pixel, pixel_to_ground, project_mask_to_grid,
                  project_seen_to_grid)
from .obstacles import Obstacle
from .purepursuit import Pursuit, PursuitConfig, follow
from .raceline import RaceLine, optimize

__all__ = [
    "CameraExtrinsics",
    "CameraIntrinsics",
    "Centerline",
    "Obstacle",
    "OccGrid",
    "Points",
    "Pursuit",
    "PursuitConfig",
    "RaceLine",
    "camera_intrinsics",
    "centerline",
    "deskew",
    "dilate",
    "extract_centerline",
    "follow",
    "ground_to_pixel",
    "obstacles",
    "optimize",
    "pack_trinary",
    "pixel_to_ground",
    "project_mask_to_grid",
    "project_seen_to_grid",
]
