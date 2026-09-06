"""壁モード — 頂点/円弧ループをそのまま壁として描き、中心線は副産物として導出する。

中心線モード（`sim/track.py`）は中心線+道幅から壁を自動生成するが、壁モードは逆に
壁そのものを自由な本数・形状で描く。そのぶん中心線は幾何学的に決まらないので、
壁で囲まれた自由空間を細線化（スケルトン化）して1本のループを抜き出すしかない。

`sim/raceline.py::compute_raceline_offsets` は `width=None` のコースを、
`course.raycast_batch()` で中心線の法線方向に実測した壁までの距離から非対称な
道幅として扱えるよう既に対応済み（2026-09-05追加）。壁モードのコースは
`width=None` を返すことでそのままこの経路に乗る——道幅そのものを保存する必要が
そもそも無い。

## 骨格化からの中心線抽出は保存時にしか走らせない

反復するスケルトン化＋`raspi/nav/centerline.py::build()`（「測る→真ん中へ寄せる」を
数回反復）は数十〜数百msのオーダー。編集中は壁の形が毎フレーム変わりうるので、
エディタは**保存時に1回だけ**これを実行して `centerline` をJSONへ焼き込む。
`Course.load()`（→`build()`、本モジュール下部）はグリッドだけ壁ループから作り直し、
`centerline`/`start` はJSONの値をそのまま使う（ロードのたびに反復処理を走らせない）。

## スケルトン化は自前実装（Zhang-Suen）

骨格化には通常 `scikit-image` の `skeletonize` を使うが、この用途のためだけに
比較的重い依存を増やしたくない（`sim/requirements.txt` の「numpyとpillowだけで
足りる」という既存の最小主義に合わせる）。1反復あたりの判定を numpy でベクトル化
すれば十分速い（保存時に1回だけ走る処理なので、多少のコストは許容できる）。
"""

from __future__ import annotations

import math
from pathlib import Path

import networkx as nx
import numpy as np

from .sketch import Loop, path_to_loop, sample_loop
from .track import stamp_discs

__all__ = ["WallExtractionError", "rasterize_walls", "derive_centerline", "build"]

#: 骨格化の最大反復回数（安全弁。まともなZhang-Suenならこれよりずっと早く収束する）
_MAX_THIN_ITERS = 10000
#: `raspi.nav.centerline.build()` に渡す既定の点間隔 [m]
_DEFAULT_STEP_M = 0.10
#: 同・反復回数
_DEFAULT_ITERS = 5


class WallExtractionError(Exception):
    """壁の形状から中心線を導出できないときに送出する。エディタはこれを捕まえて
    保存を止め、メッセージをそのままユーザーに見せる。
    """


def rasterize_walls(loops: list[Loop], thickness: float, resolution: float,
                    margin: float = 0.4) -> tuple[np.ndarray, tuple[float, float]]:
    """壁ループの列 → 占有格子。

    **初期値は全部「壁なし」。** 中心線モードの `rasterize()`（全部壁で埋めてから
    中心線沿いに掘る）とは逆に、壁のストロークだけを円盤スタンプでOR演算していく
    （`sim/track.py::rasterize()` の `divider` スタンプと同じ考え方）。
    """
    if not loops:
        raise WallExtractionError("壁がありません")
    pts_list = [sample_loop(loop, resolution)[:, :2] for loop in loops]
    xy = np.concatenate(pts_list, axis=0)

    half = thickness / 2.0
    pad = half + margin
    x0, y0 = float(xy[:, 0].min() - pad), float(xy[:, 1].min() - pad)
    w = int(math.ceil((float(xy[:, 0].max()) + pad - x0) / resolution))
    h = int(math.ceil((float(xy[:, 1].max()) + pad - y0) / resolution))

    grid = np.zeros((h, w), dtype=bool)
    r = max(1, int(round(half / resolution)))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disc = (xx * xx + yy * yy) <= r * r

    for pts in pts_list:
        cols = np.round((pts[:, 0] - x0) / resolution).astype(np.int64)
        rows = np.round((pts[:, 1] - y0) / resolution).astype(np.int64)
        for c, rw in zip(cols, rows):
            grid[rw - r:rw + r + 1, c - r:c + r + 1] |= disc
    return grid, (x0, y0)


def _flood_fill_free(grid: np.ndarray, origin: tuple[float, float], resolution: float,
                     start_xy: tuple[float, float]) -> np.ndarray:
    """`start_xy` から4連結でつながる自由空間（壁ではないセル）だけのマスク。

    `scipy.ndimage.label` で全連結成分を一括ラベル付けし、スタート地点のラベルだけ
    残す——手書きのスタックより速く単純。
    """
    from scipy import ndimage

    h, w = grid.shape
    col = int(round((start_xy[0] - origin[0]) / resolution))
    row = int(round((start_xy[1] - origin[1]) / resolution))
    if not (0 <= row < h and 0 <= col < w) or grid[row, col]:
        raise WallExtractionError("スタート地点が壁の中（またはコース範囲外）にあります")

    free = ~grid
    labeled, _ = ndimage.label(free, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))
    label = labeled[row, col]
    return labeled == label


def _thin_zhang_suen(mask: np.ndarray) -> np.ndarray:
    """Zhang-Suen 細線化。`mask`（True=前景）を1px幅の骨格へ削る。

    1反復 = 8近傍パターンによる2段階の削除判定。両方とも削除対象が無くなるまで
    反復する。各段の判定は配列全体を一度にベクトル化して行う。
    """
    img = mask.astype(np.uint8)
    for _ in range(_MAX_THIN_ITERS):
        changed = False
        for step in (0, 1):
            padded = np.pad(img, 1)
            p2 = padded[0:-2, 1:-1]
            p3 = padded[0:-2, 2:]
            p4 = padded[1:-1, 2:]
            p5 = padded[2:, 2:]
            p6 = padded[2:, 1:-1]
            p7 = padded[2:, 0:-2]
            p8 = padded[1:-1, 0:-2]
            p9 = padded[0:-2, 0:-2]
            ring = (p2, p3, p4, p5, p6, p7, p8, p9, p2)
            b = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            a = np.zeros_like(img, dtype=np.int32)
            for u, v in zip(ring[:-1], ring[1:]):
                a += ((u == 0) & (v == 1)).astype(np.int32)
            common = (img == 1) & (b >= 2) & (b <= 6) & (a == 1)
            if step == 0:
                cond = common & (p2 * p4 * p6 == 0) & (p4 * p6 * p8 == 0)
            else:
                cond = common & (p2 * p4 * p8 == 0) & (p2 * p6 * p8 == 0)
            if cond.any():
                img[cond] = 0
                changed = True
        if not changed:
            break
    return img.astype(bool)


def _prune_to_single_cycle(skel: np.ndarray) -> list[tuple[int, int]]:
    """骨格ピクセルをグラフ化し、袋小路の枝（次数1の葉）を刈り取って
    単一の単純ループだけを残し、順序付きピクセル列にして返す。
    """
    ys, xs = np.nonzero(skel)
    if len(ys) < 3:
        raise WallExtractionError("自由空間が細すぎて骨格を抽出できませんでした")

    coords = set(zip(ys.tolist(), xs.tolist()))
    g = nx.Graph()
    g.add_nodes_from(coords)
    for r, c in coords:
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (r + dr, c + dc)
            if nb in coords:
                g.add_edge((r, c), nb)
        for dr, dc in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            nb = (r + dr, c + dc)
            if nb not in coords:
                continue
            # 直交の「肘」のどちらかが既にあるなら対角の近道は張らない。L字コーナーで
            # 肘1つ+対角が三角形の余分な分岐を作る（実測: 矩形リングの角で発生）、
            # 8連結グラフ化の定番の罠。両方の肘が無いとき（本当に対角でしか
            # つながっていない隙間）だけ対角を張る
            if (r + dr, c) in coords or (r, c + dc) in coords:
                continue
            g.add_edge((r, c), nb)

    changed = True
    while changed:
        changed = False
        dead = [n for n in g.nodes if g.degree(n) <= 1]
        if dead:
            g.remove_nodes_from(dead)
            changed = True

    if g.number_of_nodes() == 0:
        raise WallExtractionError(
            "壁で囲まれた閉じたループが見つかりませんでした（壁が閉じていない可能性があります）")

    components = list(nx.connected_components(g))
    if len(components) != 1:
        raise WallExtractionError(
            f"閉じたループが{len(components)}個に分かれています。"
            "1つのコースにつき壁で囲まれたループは1つにしてください")

    sub = g.subgraph(components[0])
    if any(d != 2 for _, d in sub.degree()):
        raise WallExtractionError(
            "壁の形状が複雑すぎます（分岐や交差を検出しました）。ループをシンプルにしてください")

    start = next(iter(sub.nodes))
    order = [start]
    prev, cur = None, start
    while True:
        nxt = next(n for n in sub.neighbors(cur) if n != prev)
        if nxt == start:
            break
        order.append(nxt)
        prev, cur = cur, nxt
    return order


class _GridRaycastAdapter:
    """`raspi.nav.centerline.measure()` が要求する
    `grid.raycast(ox, oy, angles, max_range)` 契約（当たらなければ `max_range` を
    返す）に、`Course.raycast_batch()`（当たらなければ `0.0` を返す）を合わせる
    だけの薄いラッパー。
    """

    def __init__(self, course) -> None:
        self._course = course

    def raycast(self, ox, oy, angles, max_range: float) -> np.ndarray:
        angles = np.asarray(angles, dtype=np.float64)
        origins = np.column_stack([np.broadcast_to(ox, angles.shape),
                                   np.broadcast_to(oy, angles.shape)])
        d = self._course.raycast_batch(origins, angles, max_range)
        return np.where(d <= 0.0, max_range, d)


def derive_centerline(grid: np.ndarray, origin: tuple[float, float], resolution: float,
                      start_xy: tuple[float, float], *,
                      max_width: float | None = None,
                      step: float = _DEFAULT_STEP_M,
                      iters: int = _DEFAULT_ITERS) -> np.ndarray:
    """壁を描いた占有格子から中心線 `(N,3)`（x, y, 向き）を導出する。

    導出できない形状（壁が閉じていない・分岐がある・ループが複数ある等）なら
    `WallExtractionError`。**保存時にのみ呼ぶこと**（モジュール docstring 参照）。
    """
    from raspi.nav.centerline import build as _refine

    from .course import Course

    free = _flood_fill_free(grid, origin, resolution, start_xy)
    skel = _thin_zhang_suen(free)
    order = _prune_to_single_cycle(skel)

    x0, y0 = origin
    rough_xy = np.array([[x0 + c * resolution, y0 + r * resolution] for r, c in order])

    if max_width is None:
        max_width = max(grid.shape) * resolution
    tmp_course = Course(name="<wall-preview>", path=Path("<wall-preview>"),
                        resolution=resolution, origin=origin,
                        start=(0.0, 0.0, 0.0), grid=grid)
    adapter = _GridRaycastAdapter(tmp_course)
    cl = _refine(adapter, rough_xy, step=step, max_width=max_width, iters=iters)

    # normal = (-sin(yaw), cos(yaw))（`raspi/nav/centerline.py::normals()`の規約）
    # なので yaw = atan2(-nx, ny)
    yaw = np.arctan2(-cl.normal[:, 0], cl.normal[:, 1])
    return np.column_stack([cl.xy[:, 0], cl.xy[:, 1], yaw])


def build(meta: dict) -> dict:
    """壁モードのコースJSON（`walls` を持つもの）→ `Course` に渡す材料。

    `centerline`/`start` はエディタが保存時に焼き込んだ値をそのまま使う
    （骨格化はここでは実行しない）。グリッドだけ壁ループから毎回作り直す
    （決定的で軽い処理なので、JSONに焼き込む必要が無い）。
    """
    res = float(meta.get("resolution", 0.02))
    thickness = float(meta.get("wall_thickness", 0.03))
    margin = float(meta.get("margin", 0.4))
    loops = [path_to_loop(tuple(w["origin"]), w["path"]) for w in meta["walls"]]
    grid, origin = rasterize_walls(loops, thickness, res, margin)

    obstacles_in = meta.get("obstacles")
    obstacles = None
    if obstacles_in:
        stamp_discs(grid, origin, res, obstacles_in)
        obstacles = np.asarray(obstacles_in, dtype=np.float64)

    centerline = np.asarray(meta["centerline"], dtype=np.float64)
    start = tuple(float(v) for v in meta.get("start", centerline[0]))

    return {"grid": grid, "resolution": res, "origin": origin,
            "start": start, "centerline": centerline, "width": None,
            "obstacles": obstacles}
