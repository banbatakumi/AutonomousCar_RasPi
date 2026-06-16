"""経路（パス）処理ユーティリティ。

Pure Pursuit などの経路追従で共通利用する。閉ループ経路の等間隔リサンプル、
最近傍点探索、先読み点（lookahead point）探索を提供する。

経路は shape(N,2) の numpy 配列（ワールド座標 [m]）で表現する。
"""

from __future__ import annotations

import numpy as np


def resample_closed(waypoints, spacing: float = 0.05) -> np.ndarray:
    """閉ループのウェイポイント列を等間隔にリサンプルする。

    Args:
        waypoints: shape(M,2) のコーナー点列（最後の点と最初の点を自動で結ぶ）。
        spacing: リサンプル間隔 [m]。

    Returns:
        shape(N,2) の等間隔点列（閉ループ、終点は始点に戻らない）。
    """
    pts = np.asarray(waypoints, dtype=np.float64)
    if pts.shape[0] < 2:
        return pts.copy()

    # 閉ループ化（末尾に始点を追加して一周ぶんの線分を作る）
    loop = np.vstack([pts, pts[0]])
    seg = np.diff(loop, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    total = float(seg_len.sum())
    if total < 1e-9:
        return pts.copy()

    n = max(int(round(total / spacing)), 2)
    # 0..total を n 等分（終点 total は始点と重なるので除外）
    targets = np.linspace(0.0, total, n, endpoint=False)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])

    out = np.empty((n, 2), dtype=np.float64)
    for i, dist in enumerate(targets):
        # dist がどの線分に属するか
        k = int(np.searchsorted(cum, dist, side="right") - 1)
        k = min(max(k, 0), len(seg) - 1)
        local = (dist - cum[k]) / seg_len[k] if seg_len[k] > 1e-9 else 0.0
        out[i] = loop[k] + local * seg[k]
    return out


def nearest_index(path: np.ndarray, x: float, y: float) -> int:
    """点(x,y)に最も近い経路点のインデックスを返す。"""
    dx = path[:, 0] - x
    dy = path[:, 1] - y
    return int(np.argmin(dx * dx + dy * dy))


def lookahead_point(path: np.ndarray, x: float, y: float,
                    lookahead: float, start_idx: int | None = None
                    ) -> tuple[np.ndarray, int]:
    """最近傍点から経路に沿って lookahead 進んだ先読み点を返す（閉ループ）。

    Returns:
        (point shape(2,), target_index)
    """
    n = path.shape[0]
    if start_idx is None:
        start_idx = nearest_index(path, x, y)

    acc = 0.0
    idx = start_idx
    prev = path[start_idx]
    for step in range(1, n + 1):
        nxt_idx = (start_idx + step) % n
        nxt = path[nxt_idx]
        d = float(np.hypot(nxt[0] - prev[0], nxt[1] - prev[1]))
        if acc + d >= lookahead:
            # prev→nxt 上で lookahead に達する点を線形補間
            remain = lookahead - acc
            ratio = remain / d if d > 1e-9 else 0.0
            pt = prev + ratio * (nxt - prev)
            return pt, nxt_idx
        acc += d
        prev = nxt
        idx = nxt_idx
    # 一周しても届かない（経路が短い）→ 最遠点
    return path[idx], idx


def extract_loop(points, start_xy=None, leave_radius: float = 0.8,
                 return_radius: float = 0.45) -> np.ndarray:
    """走行軌跡から1周ぶんの閉ループを抽出する。

    開始点から一度 leave_radius 以上離れ、その後 return_radius 以内へ戻った
    最初の時点までを1周とみなす。戻ってこない場合は全体を返す。
    """
    pts = np.asarray(points, dtype=np.float64)
    if len(pts) < 4:
        return pts
    start = np.asarray(start_xy if start_xy is not None else pts[0], dtype=np.float64)

    d = np.hypot(pts[:, 0] - start[0], pts[:, 1] - start[1])
    left = np.argmax(d > leave_radius)
    if d[left] <= leave_radius:
        return pts  # 一度も離れていない
    # left 以降で初めて return_radius 内へ
    for i in range(left, len(pts)):
        if d[i] < return_radius:
            return pts[:i + 1]
    return pts


def path_curvature(path: np.ndarray) -> np.ndarray:
    """各点での経路曲率 [1/m] を返す（閉ループ、3点円近似）。"""
    n = path.shape[0]
    kappa = np.zeros(n)
    for i in range(n):
        p0 = path[(i - 1) % n]
        p1 = path[i]
        p2 = path[(i + 1) % n]
        a = np.hypot(*(p1 - p0))
        b = np.hypot(*(p2 - p1))
        c = np.hypot(*(p2 - p0))
        # 三角形面積（外積）
        area = abs((p1[0] - p0[0]) * (p2[1] - p0[1])
                   - (p2[0] - p0[0]) * (p1[1] - p0[1])) / 2.0
        denom = a * b * c
        kappa[i] = (4.0 * area / denom) if denom > 1e-9 else 0.0
    return kappa
