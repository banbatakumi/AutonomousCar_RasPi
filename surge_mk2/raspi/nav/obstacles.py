"""動的障害物 — 凍結した地図に**無いもの**を見つける。

地図を作る段では「動く物を壁として焼かない」（`grid.py` の hits/misses）。
ここは走る段の話で、**凍結した地図と今の点群の差**を取る。

## 判定は「空きだと確信しているセルに点が落ちた」だけ

    候補 = known_free なセル に落ちた点  −  壁のすぐ近く  −  飽和点

**「未知」のセルを空き扱いにしない**のが誤検出を出さない最大のコツ。1周目に
見えなかった場所（壁の陰、コースの外側）は unknown のままで、そこを通るたびに
「障害物だ」と言い出す planner は使い物にならない。

## 壁の近くは見ない

自己位置は数 cm〜数十 cm ずれる。壁の点がその隣の空きセルに落ちるのは
**日常的に起きる**。壁を `wall_pad` [m] 太らせた領域の中は、何が落ちても
動的障害物と呼ばない。太らせるぶん、壁際に立つ本物の障害物は見逃すが、
そちらは経路が壁際を通らない限り問題にならない。

**セル数ではなく距離で指定する。** 格子を 5cm から 2.5cm に細かくしたとき、
セル数のままだと膨張幅が黙って半分になり、**壁が動的障害物として検出されて
「前方 0.0m に障害物」で止まり続けた**。

## 1周期では信じない

`confirm()` に前の周期の結果を渡すと、**近い位置に2周期連続で出たものだけ**を
返す。10Hz なので 200ms の確認遅延が乗るが、その代わり単発のノイズで
急ブレーキを踏まなくなる。3 m/s なら 60cm 進むので、`stop_dist` はその
ぶんを含めて決めること。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .deskew import Points
from .grid import OccGrid, dilate

__all__ = ["Obstacle", "detect", "confirm", "blocking"]

#: クラスタを切る点間距離 [m]。これより離れていたら別の物体
_SPLIT_M = 0.18


class Obstacle(NamedTuple):
    x: float                               #: 重心 [m]（map フレーム）
    y: float
    r: float                               #: 半径 [m]。点群の広がりから出す
    n: int                                 #: 構成する点の数


def detect(grid: OccGrid, pts: Points, pose: tuple[float, float, float], *,
           wall_pad: float = 0.15, min_points: int = 3,
           max_obstacles: int = 8) -> list[Obstacle]:
    """凍結地図に無いものを拾う。**`grid.frozen` でなければ何も返さない。**

    地図を作っている最中は「地図に無い」が当たり前なので、判定に意味が無い。
    """
    if not grid.frozen or len(pts) == 0:
        return []
    hit = pts.hit
    if not hit.any():
        return []

    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x0 + pts.x[hit] * c - pts.y[hit] * s
    wy = y0 + pts.x[hit] * s + pts.y[hit] * c

    col, row = grid.to_cell(wx, wy)
    ok = grid.inside(col, row)
    pad = max(1, int(round(wall_pad / grid.resolution)))
    free = grid.known_free_mask() & ~dilate(grid.wall_mask(), pad)
    cand = np.zeros(wx.size, dtype=bool)
    cand[ok] = free[row[ok], col[ok]]
    if not cand.any():
        return []

    idx = np.flatnonzero(cand)
    px, py = wx[idx], wy[idx]
    # 点は方位順に並んでいるので、隣同士の距離が開いたところで切ればよい
    gap = np.hypot(np.diff(px), np.diff(py)) > _SPLIT_M
    starts = np.concatenate([[0], np.flatnonzero(gap) + 1, [idx.size]])

    out: list[Obstacle] = []
    for a, b in zip(starts[:-1], starts[1:]):
        if b - a < min_points:
            continue
        cx, cy = float(px[a:b].mean()), float(py[a:b].mean())
        r = float(np.hypot(px[a:b] - cx, py[a:b] - cy).max())
        out.append(Obstacle(cx, cy, max(r, grid.resolution), int(b - a)))

    out.sort(key=lambda o: -o.n)
    return out[:max_obstacles]


def confirm(now: list[Obstacle], prev: list[Obstacle], *,
            tol: float = 0.35) -> list[Obstacle]:
    """**2周期連続で同じ場所に出たものだけ**を残す。単発のノイズを落とす。"""
    if not prev:
        return []
    out = []
    for o in now:
        if any(math.hypot(o.x - p.x, o.y - p.y) <= tol + o.r + p.r for p in prev):
            out.append(o)
    return out


def blocking(obstacles: list[Obstacle], path_xy: np.ndarray, index: int,
             *, half_width: float, ahead_m: float, step: float,
             skip_m: float = 0.35) -> tuple[Obstacle | None, float]:
    """これから通る帯に入っている障害物と、そこまでの距離 [m]。

    経路上の `index` から `ahead_m` 先までの点だけを見る。**後ろや、1周先の
    同じ場所に居る物で止まらない**ため（閉ループなので添字は巡回する）。

    :param skip_m: 自車の足元ぶん、手前を飛ばす距離 [m]

    ★ **自分の真横や後ろを数えてはいけない。** `index` は自車が今いる場所なので、
    そこから数えると**自車のすぐ横にある壁**（＝地図の誤差で動的障害物と誤検出
    されたもの）が「前方 0.0m の障害物」になり、走り出した瞬間に止まる。
    実測でこれに引っ掛かった（`+138°`＝左後方の物で停止していた）。

    戻り値の距離は経路に沿った弧長の近似（点数 × 間隔）。**車体前端ではなく
    base_link からの距離**なので、停止判定に使う側で前オーバーハングを足すこと。
    """
    if not obstacles or len(path_xy) == 0:
        return None, math.inf
    n = len(path_xy)
    span = max(1, int(ahead_m / max(step, 1e-3)))
    span = min(span, n - 1)
    skip = min(span, max(0, int(skip_m / max(step, 1e-3))))
    idx = (index + np.arange(skip, span + 1)) % n
    seg = path_xy[idx]

    best: Obstacle | None = None
    best_d = math.inf
    for o in obstacles:
        d2 = (seg[:, 0] - o.x) ** 2 + (seg[:, 1] - o.y) ** 2
        k = int(np.argmin(d2))
        if math.sqrt(float(d2[k])) <= half_width + o.r:
            d = (k + skip) * step
            if d < best_d:
                best, best_d = o, d
    return best, best_d
