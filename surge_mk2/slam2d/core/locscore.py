"""自己位置の候補の採点 — 他車などの**遮り物に強い**一致度。

`core/register.py`の`RegisterResult.inlier_all`・`rms`は、点群の**全点**が地図の壁に
乗っていることを前提にした尺度で、地図に無い他車（小さな障害物）が走査線を遮ると
崩れる。他車の表面の点は壁から数cm〜数十cm離れた空きセルに落ちるので、当たり率は
下がり、残差のRMSは（壁から`max_dist`以内に落ちた他車の点が混じって）桁で跳ねる。
真の姿勢でも点数が潰れる（`sim.slam_bench --locate --locate-cars 3`、
真の姿勢の旧点数 0.00〜0.07、他車なしでは 0.85〜0.92）。

## 他車は「壁より手前で遮る」だけ

遮り物の点は、**センサからその点までの光路が地図の上で空き**で、終点が壁より手前の
空きセルに落ちる。誤った姿勢の点は、これに加えて**光路が壁を突き抜ける**
（地図の壁の向こう側が見えている）か、壁の近くを外れる。そこで点を3つに分ける:

- `hit`   : 終点が壁から`inlier_dist`以内（地図と一致）
- `cross` : 光路が壁を横切る（地図と矛盾。他車では起きない）
- `shadow`: 光路は空きで、終点が壁から離れた既知の空きセル（他車などの遮り物の候補）

採点は`hit`を**遮り物を除いた点数**で割る。遮り物に見える点は`shadow_cap`までしか
中立に扱わない（誤った姿勢は点の大半が空きに落ちて、中立に逃げ得をするため）。
`cross`は減点し、`hit`の絶対量（全点に対する割合）が`min_hit`に届かない候補は断る。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .grid import OccGrid
from .types import Pose2D, ScanPoints

__all__ = ["LocateFit", "LocateScorer"]


class LocateFit(NamedTuple):
    n: int           #: 採点した点数
    hit: float       #: 壁に乗った点の割合（全点に対して）
    cross: float     #: 光路が壁を横切った点の割合
    shadow: float    #: 遮り物らしい点の割合
    rms: float       #: `hit`の点の残差のRMS [m]
    near: float = 0.0  #: 壁のすぐ脇（`inlier_dist`超〜`shadow_dist`以内）に外れた点の割合。ずれの兆候


class LocateScorer:
    """1つの地図に対する採点器。地図ごとの前処理（壁までの距離場）を持つので、候補をまたいで使い回す。"""

    def __init__(self, grid: OccGrid, *, inlier_dist: float = 0.05, shadow_dist: float = 0.15,
                 margin: float = 0.12, max_range: float = 8.0) -> None:
        import cv2           # Pi には scipy が無い（`surfmap.py` と同じく cv2 を使う）

        self._res = grid.resolution
        self._origin = grid.origin
        wall = grid.wall_mask()
        self._dist = cv2.distanceTransform((~wall).astype(np.uint8), cv2.DIST_L2, 5) * self._res
        self._known = grid.seen > 0
        # 光路の判定は壁セルそのまま（太らせると、壁すれすれを掠める光線が壁を横切ったことになる）。
        # 細い壁をすり抜けないよう、光路は半セル刻みで調べる（`fit`）
        self._blocked = wall
        self._inlier = inlier_dist
        self._shadow = shadow_dist
        self._margin = margin
        self._max_range = max_range

    def _cells(self, wx: np.ndarray, wy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        c = np.floor((wx - self._origin[0]) / self._res).astype(np.int64)
        r = np.floor((wy - self._origin[1]) / self._res).astype(np.int64)
        h, w = self._dist.shape
        ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        return np.where(ok, r, 0), np.where(ok, c, 0), ok

    def fit(self, pts: ScanPoints, pose: Pose2D, sensor: tuple[float, float],
            max_points: int = 240) -> LocateFit:
        px, py = pts.x[pts.hit], pts.y[pts.hit]
        d0 = np.hypot(px - sensor[0], py - sensor[1])
        keep = d0 <= self._max_range
        px, py = px[keep], py[keep]
        if px.size > max_points:
            take = np.linspace(0, px.size - 1, max_points).astype(np.int64)
            px, py = px[take], py[take]
        n = px.size
        if n == 0:
            return LocateFit(0, 0.0, 0.0, 0.0, math.inf)
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        wx = pose.x + c * px - s * py
        wy = pose.y + s * px + c * py
        ox = pose.x + c * sensor[0] - s * sensor[1]
        oy = pose.y + s * sensor[0] + c * sensor[1]

        r, cc, ok = self._cells(wx, wy)
        known = ok & self._known[r, cc]
        d = np.where(known, self._dist[r, cc], np.inf)
        hit = d <= self._inlier

        # 光路: センサから終点の手前`margin`までを刻み、壁のセルを踏んだら矛盾
        rng = np.hypot(wx - ox, wy - oy)
        step = 0.5 * self._res
        steps = max(1, int(math.ceil((self._max_range - self._margin) / step)))
        t = (np.arange(steps) * step)[None, :]
        lim = (rng - self._margin)[:, None]
        valid = t < lim
        ux, uy = ((wx - ox) / np.maximum(rng, 1e-9))[:, None], ((wy - oy) / np.maximum(rng, 1e-9))[:, None]
        sx, sy = ox + ux * t, oy + uy * t
        rr, cs, sok = self._cells(sx, sy)
        cross = (self._blocked[rr, cs] & sok & valid).any(axis=1) & ~hit
        shadow = known & (d > self._shadow) & ~cross
        rms = float(np.sqrt(np.mean(d[hit] ** 2))) if hit.any() else math.inf
        near = known & (d > self._inlier) & (d <= self._shadow) & ~cross
        return LocateFit(n, float(hit.mean()), float(cross.mean()), float(shadow.mean()), rms,
                         float(near.mean()))
