"""閉ループ経路ユーティリティ（Phase2 経路追従用）。

経路はすべて閉ループ（最後の点が最初の点に戻る）として扱う。
- resample_closed : 等間隔リサンプリング
- nearest_index   : 最近傍点インデックス
- lookahead_point : 現在位置から弧長 Ld だけ前方の点
- path_curvature  : 各点の曲率（外接円の逆数）
"""
from __future__ import annotations

import numpy as np


def resample_closed(path: np.ndarray, spacing: float) -> np.ndarray:
    """閉ループ経路を等間隔（約 spacing[m]）でリサンプルする。

    入力 path: shape(N,2)。出力 shape(M,2)（始点≒終点は重複させない）。
    """
    pts = np.asarray(path, dtype=float)
    if len(pts) < 2:
        return pts.copy()

    # 閉ループにするため終点に始点を追加
    loop = np.vstack([pts, pts[0]])
    seg = np.diff(loop, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    total = cum[-1]
    if total < 1e-9:
        return pts.copy()

    n = max(int(round(total / spacing)), 3)
    targets = np.linspace(0.0, total, n, endpoint=False)
    xs = np.interp(targets, cum, loop[:, 0])
    ys = np.interp(targets, cum, loop[:, 1])
    return np.column_stack([xs, ys])


def nearest_index(path: np.ndarray, point: tuple[float, float]) -> int:
    """point に最も近い経路点のインデックスを返す。"""
    pts = np.asarray(path, dtype=float)
    d = np.hypot(pts[:, 0] - point[0], pts[:, 1] - point[1])
    return int(np.argmin(d))


def lookahead_point(
    path: np.ndarray,
    position: tuple[float, float],
    lookahead_dist: float,
    start_idx: int | None = None,
) -> tuple[np.ndarray, int]:
    """現在位置から経路に沿って弧長 lookahead_dist 前方の点を返す（閉ループ）。

    戻り値: (target_point shape(2,), target_index)
    """
    pts = np.asarray(path, dtype=float)
    n = len(pts)
    if n == 0:
        return np.array(position, dtype=float), 0
    if start_idx is None:
        start_idx = nearest_index(path, position)

    acc = 0.0
    idx = start_idx
    for _ in range(n):
        nxt = (idx + 1) % n
        seg = pts[nxt] - pts[idx]
        seg_len = float(np.hypot(seg[0], seg[1]))
        if acc + seg_len >= lookahead_dist:
            remain = lookahead_dist - acc
            t = remain / seg_len if seg_len > 1e-9 else 0.0
            return pts[idx] + t * seg, nxt
        acc += seg_len
        idx = nxt
    # 一周しても届かない場合は最遠点
    return pts[(start_idx + n // 2) % n], (start_idx + n // 2) % n


def path_curvature(path: np.ndarray) -> np.ndarray:
    """各点の曲率 κ [1/m] を返す（閉ループ、外接円の逆数）。"""
    pts = np.asarray(path, dtype=float)
    n = len(pts)
    if n < 3:
        return np.zeros(n)

    prev = np.roll(pts, 1, axis=0)
    nxt = np.roll(pts, -1, axis=0)

    a = np.hypot(*(pts - prev).T)
    b = np.hypot(*(nxt - pts).T)
    c = np.hypot(*(nxt - prev).T)

    # 三角形の符号付き面積 ×2
    area2 = np.abs(
        (pts[:, 0] - prev[:, 0]) * (nxt[:, 1] - prev[:, 1])
        - (pts[:, 1] - prev[:, 1]) * (nxt[:, 0] - prev[:, 0])
    )
    denom = a * b * c
    kappa = np.where(denom > 1e-9, 2.0 * area2 / denom, 0.0)
    return kappa


def extract_loop(traj: np.ndarray, start_xy: tuple[float, float],
                 leave: float = 0.8, return_radius: float = 0.45) -> np.ndarray:
    """走行軌跡から1周分を抽出する。

    スタート地点を一度 leave[m] 以上離れてから、再び return_radius[m] 以内に
    戻ってきた点までを1周とみなして切り出す。
    """
    traj = np.asarray(traj, dtype=float)
    if len(traj) < 3:
        return traj
    d0 = np.hypot(traj[:, 0] - start_xy[0], traj[:, 1] - start_xy[1])
    left_idx = np.argmax(d0 > leave) if np.any(d0 > leave) else None
    if left_idx is None or left_idx == 0 and d0[0] <= leave:
        return traj
    after = np.where(d0[left_idx:] < return_radius)[0]
    if len(after) == 0:
        return traj
    end = left_idx + after[0]
    return traj[: end + 1]


def path_normals(path: np.ndarray) -> np.ndarray:
    """各点の左向き単位法線ベクトル shape(N,2) を返す（閉ループ）。"""
    pts = np.asarray(path, dtype=float)
    nxt = np.roll(pts, -1, axis=0)
    prev = np.roll(pts, 1, axis=0)
    tang = nxt - prev
    norm = np.hypot(tang[:, 0], tang[:, 1])
    norm = np.where(norm > 1e-9, norm, 1.0)
    tx = tang[:, 0] / norm
    ty = tang[:, 1] / norm
    # 左法線 = 接線を +90° 回転
    return np.column_stack([-ty, tx])
