"""ランダムな閉ループコースの生成 — RL 訓練のコース多様性確保用。

`sim/editor.py` のような「直線/円弧を手でつないで閉じる」方式は自動生成に向かない
（総回頭が360の倍数になるよう組み合わせを選ぶ必要があり、機械的に閉じるのが難しい）。
代わりに**円周上のランダム半径点列**を作り、**角だけを小さい半径で丸める**
（辺の途中は直線のまま残す）ことで、直線区間つきの閉ループを作る。円周上の
関数として作っているので、始点と終点は常にぴったり閉じる（`close_gap()`のような
補正が要らない）。

`sim/track.py` の `build()`（JSON の `"path"` セグメント列から中心線を作る経路）は
使わず、`rasterize()` だけを再利用して `Course` を直接組み立てる。

## ★ 角だけを丸める理由（直線区間を残すため）

最初の実装は多角形全体を一様に`smooth_loop()`していたため、平滑化の窓（辺の
長さの大部分をカバーする大きさ）が辺そのものを消してしまい、**直線区間がほぼ
無い・コースの見た目が毎回似たような丸っこい形になる**という問題があった
（2026-08-28、バンビの指摘で判明）。角の近傍だけを丸める窓（`fillet_m`、辺の
長さよりずっと小さい）にすることで、辺の中間は直線のまま残る。

## 形状の多様性

角の数（`n_ctrl`）・半径の基準値・半径のばらつき具合をコースごとに乱数で決め、
さらに `_add_chicanes()` で局所的なS字の蛇行（`fuji`コースのシケイン区間に近い
性格）を数箇所ランダムに挿入する。蛇行は窓の両端で変位0になるように作るので、
**ループの閉じ方には一切影響しない**（法線方向の局所的な足し算で済む）。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from raspi.nav.centerline import resample_loop, smooth_loop

from .course import Course
from .track import rasterize

__all__ = ["generate_random_course"]


def generate_random_course(rng: np.random.Generator, *, name: str = "rand",
                           width: float = 1.0, resolution: float = 0.02,
                           fillet_m: float = 0.25, final_step: float = 0.1) -> Course:
    """ランダムな閉ループの中心線から `Course` を組み立てて返す。

    最小旋回半径の厳密な保証はしない。学習コースの多様性確保が目的であって
    コース品質の完璧さは求めない——きつすぎる区間があっても、その区間の学習が
    単に難しくなるだけで、閉じない・自己交差するよりは実害が小さい。

    :param fillet_m: 角を丸める半径のだいたいの目安 [m]。辺の長さより
        十分小さくすること（大きくすると直線区間が消える。上記docstring参照）
    """
    n_ctrl = int(rng.integers(6, 13))                  # 角の数。少ないほど直線が際立つ
    base_r = float(rng.uniform(1.4, 3.5))
    jaggedness = float(rng.uniform(0.15, 0.6))          # 半径のばらつき（コースごとに変える）
    radius_range = (base_r * (1 - jaggedness), base_r * (1 + jaggedness))

    angles = np.linspace(0.0, 2 * math.pi, n_ctrl, endpoint=False)
    radii = rng.uniform(radius_range[0], radius_range[1], n_ctrl)
    poly = np.column_stack((radii * np.cos(angles), radii * np.sin(angles)))

    edge_len = 2 * math.pi * base_r / n_ctrl
    corner_step = min(0.05, edge_len / 20)              # 角の丸めを解像度良く効かせる刻み
    xy = resample_loop(poly, corner_step)
    fillet_window = max(1, int(round(fillet_m / corner_step)))
    xy = smooth_loop(xy, fillet_window)                 # ★角だけ丸める（辺は直線のまま残る）

    xy = _add_chicanes(rng, xy, n=int(rng.integers(0, 3)))

    xy = resample_loop(xy, final_step)
    nxt = np.roll(xy, -1, axis=0)
    yaw = np.arctan2(nxt[:, 1] - xy[:, 1], nxt[:, 0] - xy[:, 0])
    centerline = np.column_stack((xy, yaw))

    grid, origin = rasterize(centerline, width, resolution)
    start = (float(centerline[0, 0]), float(centerline[0, 1]), float(centerline[0, 2]))

    return Course(name=name, path=Path(f"<random:{name}>"), resolution=resolution,
                 origin=origin, start=start, grid=np.ascontiguousarray(grid),
                 centerline=centerline, width=width)


def _add_chicanes(rng: np.random.Generator, xy: np.ndarray, *, n: int,
                  window_m: float = 1.2, amplitude_range: tuple[float, float] = (0.25, 0.6)
                  ) -> np.ndarray:
    """局所的なS字の蛇行を `n` 箇所ランダムに挿入する。**窓の両端で変位0**にして
    あるので、挿入してもループの閉じ方（始点=終点）は崩れない。"""
    if n <= 0 or len(xy) < 12:
        return xy
    out = xy.copy()
    N = len(xy)
    seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
    avg_step = float(seg.mean()) if seg.mean() > 1e-9 else 0.05
    half_w = max(3, int(round(window_m / avg_step / 2)))

    for _ in range(n):
        # ★法線は挿入前の座標(base)から計算する。out を直接参照すると、
        # 直前に変位させた隣接点を使って次の点の法線を出すことになり、
        # 変位が変位を呼んでねじれる（実際にこれで自己交差する蛇行が出た）
        base = out.copy()
        center = int(rng.integers(0, N))
        amp = float(rng.uniform(*amplitude_range)) * float(rng.choice([-1.0, 1.0]))
        for k in range(-half_w, half_w + 1):
            idx = (center + k) % N
            t = (k + half_w) / (2 * half_w)
            bump = amp * math.sin(math.pi * t)          # 窓の両端(t=0,1)で0
            a, b = base[(idx - 1) % N], base[(idx + 1) % N]
            tangent = b - a
            norm = math.hypot(*tangent)
            if norm < 1e-9:
                continue
            nx, ny = -tangent[1] / norm, tangent[0] / norm
            out[idx, 0] = base[idx, 0] + bump * nx
            out[idx, 1] = base[idx, 1] + bump * ny
    return out
