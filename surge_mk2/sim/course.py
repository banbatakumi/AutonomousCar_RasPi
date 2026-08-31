"""コース — PNG の占有格子・レイキャスト・車体外形の衝突判定。

PNG と同名の JSON がペアになる。PNG は「見た目」ではなく**占有格子そのもの**で、
輝度が `wall_threshold` 未満のピクセルが壁。

    sim/courses/oval.png
    sim/courses/oval.json   {"resolution": 0.02, "origin": [0,0],
                             "start": [1.0, 1.0, 0.0], "wall_threshold": 128}

## 座標系

世界座標は `docs/architecture.md` §5.1 に従う（x = 前/右、y = 左/上、反時計回りが正）。
**画像の行は上下反転して格納する**（画像は行0が上、世界は y が上に増える）ので、
`grid[row, col]` の row は `origin` からの +y 方向のセル数になる。

## レイキャストは numpy でまとめて撃つ

12m ÷ 0.02m = 最大 600 セル。これを Python の for で 360 本回すと 120Hz に間に合わない。
「全レイ × 全ステップ」の座標を一度に作り、`argmax` で最初の当たりを取る。
30 本 × 1200 ステップ = 36,000 点で、120Hz でも 4.3M サンプル/秒に収まる。
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

__all__ = ["Course", "list_courses", "DEFAULT_COURSE_DIR"]

DEFAULT_COURSE_DIR = Path(__file__).resolve().parent / "courses"

#: レイを進める刻み幅を解像度の何倍にするか。1.0 だと斜めの壁をすり抜ける
_STEP_RATIO = 0.5


@dataclass
class Course:
    name: str
    path: Path
    resolution: float                 #: [m/px]
    origin: tuple[float, float]       #: 画像左下ピクセルの世界座標 [m]
    start: tuple[float, float, float] #: 初期姿勢 x[m], y[m], yaw[rad]
    grid: np.ndarray                  #: bool [H, W]。True = 壁。行0 = y 最小
    #: センターライン方式で作った場合の中心線 `(N,3)`（x, y, 向き）。PNG 由来なら None。
    #: **Phase 3 の経路追従の正解データとしてそのまま使える**
    centerline: np.ndarray | None = None
    #: 道幅 [m]（センターライン方式のみ）。スカラーならコース全体で一定、
    #: `centerline`と同じ長さの配列なら位置ごとに変わる
    #: （`sim/random_course.py`の`narrow`/`obstacle`アーキタイプ）
    width: float | np.ndarray | None = None
    #: 孤立した円盤障害物の中心座標 (K,3): x, y, 半径[m]。
    #: `sim/random_course.py`の`obstacle`アーキタイプのみが持つ
    obstacles: np.ndarray | None = None

    # ── 読み込み ──

    @classmethod
    def load(cls, path: str | Path) -> "Course":
        """`.png`（＋同名 `.json`）でも、`path` を持つ `.json` 単体でも読める。

        判別は **JSON に `"path"`（区間の並び）があるかどうか**の1点だけ。
        あればセンターラインから生成し、無ければ PNG を占有格子として読む。
        """
        p = Path(path)
        meta: dict = {}
        if p.suffix == ".json":
            meta = json.loads(p.read_text(encoding="utf-8"))
        else:
            side = p.with_suffix(".json")
            if side.exists():
                meta = json.loads(side.read_text(encoding="utf-8"))

        if "path" in meta:
            from .track import build
            t = build(meta)
            return cls(name=meta.get("name", p.stem), path=p,
                       resolution=t["resolution"], origin=t["origin"],
                       start=t["start"], grid=np.ascontiguousarray(t["grid"]),
                       centerline=t["centerline"], width=t["width"])

        from PIL import Image
        img = Image.open(p).convert("L")
        # 画像は行0が上。世界座標は y が上に増えるので上下反転して持つ
        arr = np.flipud(np.asarray(img, dtype=np.uint8))
        grid = arr < int(meta.get("wall_threshold", 128))

        origin = tuple(meta.get("origin", (0.0, 0.0)))
        start = tuple(meta.get("start", (0.0, 0.0, 0.0)))
        return cls(
            name=meta.get("name", p.stem),
            path=p,
            resolution=float(meta.get("resolution", 0.02)),
            origin=(float(origin[0]), float(origin[1])),
            start=(float(start[0]), float(start[1]), float(start[2])),
            grid=np.ascontiguousarray(grid),
        )

    # ── 寸法 ──

    @property
    def size_m(self) -> tuple[float, float]:
        h, w = self.grid.shape
        return w * self.resolution, h * self.resolution

    def occupied(self, x: float, y: float) -> bool:
        """1点の占有判定。**画像の外は壁とみなす**（コースの外に出られると困る）。"""
        col = int((x - self.origin[0]) / self.resolution)
        row = int((y - self.origin[1]) / self.resolution)
        h, w = self.grid.shape
        if not (0 <= col < w and 0 <= row < h):
            return True
        return bool(self.grid[row, col])

    # ── レイキャスト ──

    def raycast(self, ox: float, oy: float, angles: np.ndarray,
                max_range: float) -> np.ndarray:
        """`angles`（世界座標の絶対角 [rad]）方向の距離 [m] を返す。

        当たらなかったレイは **0.0**（プロトコルの「無効」と同じ表現）。
        `dist=0` を「距離0の障害物」と読んではいけないのは実機と同じ。
        """
        step = self.resolution * _STEP_RATIO
        n_steps = max(1, int(max_range / step))
        t = np.arange(1, n_steps + 1, dtype=np.float32) * step      # (S,)

        cos = np.cos(angles).astype(np.float32)[:, None]            # (R,1)
        sin = np.sin(angles).astype(np.float32)[:, None]
        cols = ((ox - self.origin[0]) / self.resolution
                + cos * t[None, :] / self.resolution).astype(np.int32)
        rows = ((oy - self.origin[1]) / self.resolution
                + sin * t[None, :] / self.resolution).astype(np.int32)

        h, w = self.grid.shape
        inside = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        # 画像外は壁扱い。np.where で添字を潰してから引くと fancy index 1回で済む
        occ = self.grid[np.where(inside, rows, 0), np.where(inside, cols, 0)]
        occ = np.where(inside, occ, True)

        hit = occ.any(axis=1)
        first = occ.argmax(axis=1)                 # 当たらない行は 0 を返すので hit で潰す
        return np.where(hit, t[first], 0.0).astype(np.float64)

    def ray(self, ox: float, oy: float, angle: float, max_range: float) -> float:
        """レイ1本。超音波用。"""
        return float(self.raycast(ox, oy, np.array([angle]), max_range)[0])

    # ── 衝突判定 ──

    def body_samples(self, footprint: list[list[float]]) -> np.ndarray:
        """車体外形を埋めるサンプル点（車体座標）を作る。**1回だけ呼んで使い回す。**

        毎周期ポリゴンを走査し直す必要はない。回転と平行移動だけを毎回やる。

        **内部の格子だけでは足りない。** 交差数判定は輪郭上の点を「外」と判定するので、
        車体の縁ちょうど 1 セルぶんが判定から抜け落ちる（前端 0.31m が 0.30m 扱いになる）。
        衝突判定が甘い方向にずれるので、輪郭も別に刻んで足す。
        """
        poly = np.asarray(footprint, dtype=np.float64)
        step = self.resolution * _STEP_RATIO
        xs = np.arange(poly[:, 0].min(), poly[:, 0].max() + step, step)
        ys = np.arange(poly[:, 1].min(), poly[:, 1].max() + step, step)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        pts = np.column_stack((gx.ravel(), gy.ravel()))
        inner = pts[_points_in_poly(pts, poly)]
        return np.vstack((inner, _outline(poly, step)))

    def collides(self, x: float, y: float, yaw: float,
                 body: np.ndarray) -> bool:
        """車体外形が壁に重なっているか。`body` は `body_samples()` の戻り値。"""
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + body[:, 0] * c - body[:, 1] * s
        wy = y + body[:, 0] * s + body[:, 1] * c
        cols = ((wx - self.origin[0]) / self.resolution).astype(np.int32)
        rows = ((wy - self.origin[1]) / self.resolution).astype(np.int32)
        h, w = self.grid.shape
        inside = (cols >= 0) & (cols < w) & (rows >= 0) & (rows < h)
        if not inside.all():
            return True                          # 画像の外にはみ出した = 壁
        return bool(self.grid[rows, cols].any())


def _outline(poly: np.ndarray, step: float) -> np.ndarray:
    """ポリゴンの各辺を `step` 間隔で刻んだ点列。"""
    out = []
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        seg = float(np.hypot(*(b - a)))
        k = max(2, int(seg / step) + 1)
        t = np.linspace(0.0, 1.0, k)[:, None]
        out.append(a + (b - a) * t)
    return np.vstack(out)


def _points_in_poly(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """交差数判定（ray casting）。凸凹を問わない。"""
    inside = np.zeros(len(pts), dtype=bool)
    px, py = pts[:, 0], pts[:, 1]
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        straddles = (y1 > py) != (y2 > py)
        with np.errstate(divide="ignore", invalid="ignore"):
            xin = (x2 - x1) * (py - y1) / (y2 - y1) + x1
        inside ^= straddles & (px < xin)
    return inside


def list_courses(directory: str | Path = DEFAULT_COURSE_DIR) -> list[Path]:
    """コースの一覧。名前順。

    PNG と、**PNG を伴わない JSON**（センターライン方式）を返す。
    PNG のサイドカー JSON を二重に並べないための区別。
    """
    d = Path(directory)
    if not d.is_dir():
        return []
    pngs = sorted(d.glob("*.png"))
    stems = {p.stem for p in pngs}
    tracks = sorted(j for j in d.glob("*.json") if j.stem not in stems)
    return pngs + tracks
