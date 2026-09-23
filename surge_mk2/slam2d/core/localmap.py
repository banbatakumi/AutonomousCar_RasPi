"""局所地図 — 直近のキーフレームの点群だけで作る、位置合わせ専用の地図。

地図作成中のフレーム間の位置合わせ（`core/frontend.py`）は、**全体の地図ではなく
直近の数mぶんの点群**に対して行う。理由は2つ:

1. **周回して戻ってきたとき、古い壁と新しい壁が混ざらない。** 全体の地図に
   合わせると、ドリフトした分だけずれた「前の周の壁」と「今の周の壁」の両方に
   点が引っ張られ、壁が二重になった地図に対して中途半端な姿勢が出る。
   その姿勢でまた地図を焼くので二重の壁が太り続ける。周回のずれを正すのは
   ループ閉じ（`backend/`）の役目で、フロントエンドはなめらかに（小さく）
   ドリフトさせておく方が正しく直せる（Cartographer の local SLAM と同じ分担）
2. **ヒット回数の閾値に依存しない。** `OccGrid.wall_mask()`は動く物を消すために
   `hits >= min_hits`を要求するので、走り出した直後は壁がまだ立たない。旧実装は
   これで照合点が足りず「見失い → 地図を更新しない → 永久に見失う」に落ちていた
   （`sim/slam_bench.py`の course1 で開始直後から99.8%見失い）。ここは点を
   そのまま使うので、2周目のスキャンから照合できる

窓は「走行距離で直近`window_m`」ぶんのキーフレーム。地図の格子は車の周り
`radius`の正方形の面要素地図（`core/surfmap.py`）で、車がその1/4以上動いたら作り直す（キーフレームが増えた
ときも作り直す）。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .surfmap import SurfaceMap
from .types import Pose2D

__all__ = ["LocalMapConfig", "LocalMap"]


@dataclass(frozen=True, slots=True)
class LocalMapConfig:
    resolution: float = 0.025
    #: 格子の半径[m]。LiDAR の照合距離（`FrontendConfig.match_range`）より少し広く
    radius: float = 8.5
    #: 直近どれだけの走行距離ぶんのキーフレームを持つか[m]
    window_m: float = 3.0
    #: 走行距離に関わらず最低これだけは持つ（止まって回頭しているとき等）
    min_keyframes: int = 3
    #: 同上、上限（回頭が続いてキーフレームが溜まりすぎないように）
    max_keyframes: int = 60
    #: 対応を付ける距離の上限[m]
    max_dist: float = 0.4


class LocalMap:
    """直近のキーフレーム点群から距離場を作る。"""

    def __init__(self, config: LocalMapConfig = LocalMapConfig()) -> None:
        self.config = config
        self.clear()

    def clear(self) -> None:
        #: (姿勢, 世界座標のx, 世界座標のy, 走行距離)
        self._kfs: deque[tuple[Pose2D, np.ndarray, np.ndarray, float]] = deque()
        self._path = 0.0
        self._last_pose: Pose2D | None = None
        self._field: SurfaceMap | None = None
        self._center: tuple[float, float] | None = None
        self._dirty = True

    def __len__(self) -> int:
        return len(self._kfs)

    def add(self, pose: Pose2D, px: np.ndarray, py: np.ndarray) -> None:
        """キーフレーム1つぶんの点（車体座標、壁に当たった点だけ）を足す。"""
        if self._last_pose is not None:
            self._path += math.hypot(pose.x - self._last_pose.x, pose.y - self._last_pose.y)
        self._last_pose = pose
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        wx = pose.x + c * px - s * py
        wy = pose.y + s * px + c * py
        self._kfs.append((pose, wx.astype(np.float32), wy.astype(np.float32), self._path))
        cfg = self.config
        while len(self._kfs) > cfg.min_keyframes and (
                len(self._kfs) > cfg.max_keyframes
                or self._path - self._kfs[0][3] > cfg.window_m):
            self._kfs.popleft()
        self._dirty = True

    def reset_to(self, poses_points: list[tuple[Pose2D, np.ndarray, np.ndarray]]) -> None:
        """ループ閉じ後など、補正済みの姿勢で窓を作り直す（点は車体座標）。"""
        self.clear()
        for pose, px, py in poses_points:
            self.add(pose, px, py)

    def field(self, around: Pose2D) -> SurfaceMap | None:
        """`around`の周りの面要素地図。キーフレームが無ければ None。"""
        if not self._kfs:
            return None
        cfg = self.config
        moved = (self._center is None
                 or math.hypot(around.x - self._center[0], around.y - self._center[1])
                 > cfg.radius * 0.25)
        if self._field is None or self._dirty or moved:
            self._rebuild((around.x, around.y))
        return self._field

    def _rebuild(self, center: tuple[float, float]) -> None:
        cfg = self.config
        xs = np.concatenate([k[1] for k in self._kfs]).astype(np.float64)
        ys = np.concatenate([k[2] for k in self._kfs]).astype(np.float64)
        self._field = SurfaceMap.from_points(xs, ys, center=center, radius=cfg.radius,
                                             resolution=cfg.resolution, max_dist=cfg.max_dist)
        self._center = center
        self._dirty = False
