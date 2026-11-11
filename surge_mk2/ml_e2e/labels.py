"""ml_e2e/labels.py — 経路点のラベルを作る幾何（torch も記録の形式も知らない純粋関数）。

経路点は**車体座標（x=前・y=左）で、一定の弧長間隔 `ds` の `K` 点**。`k` 番目は経路に沿って
`(k+1)·ds` 先の点。形（どこを通るか）と速さ（各点での速度）を別に持つ。

ラベルの出どころは2つ:

- `path_from_line`: 地図の最適ラインを、地図上の姿勢で車体座標へ移したもの。**車が線から外れていても
  線へ戻る経路が付く**（ずれからの復帰を教えられる）。速度はラインの計画速度
- `path_from_future`: 車がその後に実際に通った軌跡（推測航法）。地図も自己位置も要らない。
  速度はそこを通ったときの実速度
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["PathLabel", "advance_pose", "to_body", "resample_line", "path_from_line",
           "path_from_future", "pursuit_steer", "is_closed"]


class PathLabel:
    """経路点のラベル。`xy` は `(K, 2)`、`v` は `(K,)`、`valid` は `(K,)` の bool。"""

    __slots__ = ("xy", "v", "valid", "cross")

    def __init__(self, xy: np.ndarray, v: np.ndarray, valid: np.ndarray, cross: float = 0.0) -> None:
        self.xy = xy
        self.v = v
        self.valid = valid
        #: 線からの横偏差 [m]（左が正）。`path_from_future` では 0
        self.cross = cross


def advance_pose(pose: tuple[float, float, float], od0: tuple[float, float, float],
                 od1: tuple[float, float, float]) -> tuple[float, float, float]:
    """地図上の姿勢 `pose`（推測航法が `od0` だった時刻のもの）を、推測航法が `od1` になった時刻まで進める。

    推測航法の原点と地図の原点は別なので、動いた量を `od0` の向きで測って `pose` の向きへ掛け直す。
    """
    dx, dy = od1[0] - od0[0], od1[1] - od0[1]
    c, s = math.cos(-od0[2]), math.sin(-od0[2])
    bx, by = c * dx - s * dy, s * dx + c * dy            # od0 の車体座標での移動
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * bx - s * by, pose[1] + s * bx + c * by, pose[2] + (od1[2] - od0[2]))


def to_body(xy: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    """世界座標の点 `(N, 2)` を、`pose` の車体座標へ。"""
    d = xy - np.array([pose[0], pose[1]])
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)


def is_closed(xy: np.ndarray) -> bool:
    """折れ線が閉じた周回か（終点と始点の間が点の間隔の数倍以内）。"""
    step = np.hypot(*np.diff(xy, axis=0).T)
    return bool(np.hypot(*(xy[0] - xy[-1])) < 3.0 * float(np.median(step)))


def resample_line(xy: np.ndarray, v: np.ndarray | None, step: float = 0.05
                  ) -> tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
    """最適ラインを `step` 間隔に引き直す。`(xy, v, 向き, 閉じているか)`。

    閉じた線は終点から始点へつないでから引き直す（最後の点は始点の手前で終わる）。
    `v` が無ければ NaN。
    """
    closed = is_closed(xy)
    pts = np.vstack([xy, xy[:1]]) if closed else xy
    vv = None if v is None or len(v) != len(xy) else (np.append(v, v[0]) if closed else v)
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(s[-1] / step))
    q = np.linspace(0.0, s[-1], n, endpoint=not closed)
    out = np.stack([np.interp(q, s, pts[:, 0]), np.interp(q, s, pts[:, 1])], axis=1)
    ov = np.interp(q, s, vv) if vv is not None else np.full(n, np.nan)
    nxt = np.roll(out, -1, axis=0) if closed else np.vstack([out[1:], 2 * out[-1] - out[-2]])
    head = np.arctan2(nxt[:, 1] - out[:, 1], nxt[:, 0] - out[:, 0])
    return out, ov, head, closed


def path_from_line(line: tuple[np.ndarray, np.ndarray, np.ndarray, bool], pose: tuple[float, float, float],
                   k: int, ds: float, *, step: float = 0.05, max_cross: float = 1.0,
                   max_head_deg: float = 70.0) -> PathLabel | None:
    """引き直した最適ライン `line`（`resample_line` の戻り値）から、`pose` の先の経路点を作る。

    車に最も近く**向きも合う**点を起点にする（折り返しで隣り合う区間や交差に飛ばないため）。
    線から `max_cross` 以上離れている・向きの合う点が無いなら `None`。
    """
    xy, v, head, closed = line
    dh = (head - pose[2] + np.pi) % (2 * np.pi) - np.pi
    d2 = (xy[:, 0] - pose[0]) ** 2 + (xy[:, 1] - pose[1]) ** 2
    d2 = np.where(np.abs(dh) <= math.radians(max_head_deg), d2, np.inf)
    i0 = int(np.argmin(d2))
    if not np.isfinite(d2[i0]) or d2[i0] > max_cross ** 2:
        return None
    hop = ds / step
    idx = i0 + np.round(hop * np.arange(1, k + 1)).astype(int)
    n = len(xy)
    if closed:
        valid = np.ones(k, dtype=bool)
        idx %= n
    else:
        valid = idx < n
        idx = np.minimum(idx, n - 1)
    body = to_body(xy[idx], pose)
    # 線の左に居れば正（線の向きに対して）
    ex, ey = pose[0] - xy[i0, 0], pose[1] - xy[i0, 1]
    cross = -math.sin(head[i0]) * ex + math.cos(head[i0]) * ey
    return PathLabel(body.astype(np.float32), v[idx].astype(np.float32), valid, float(cross))


def path_from_future(od_x: np.ndarray, od_y: np.ndarray, od_yaw: np.ndarray, od_dist: np.ndarray,
                     od_t: np.ndarray, speed_t: np.ndarray, speed: np.ndarray, t: int,
                     k: int, ds: float, *, max_s: float = 15.0) -> PathLabel | None:
    """時刻 `t` の車体座標で、その後に通った軌跡の `ds` おきの点を作る。

    `od_dist`（走行距離の累積）が `(k+1)·ds` 進んだ時刻の位置を引く。記録の終わり・`max_s` 秒以内に
    届かない点は `valid=False`。1点も届かない・後退していたなら `None`。
    """
    tt = float(t)
    otf = od_t.astype(np.float64)
    if tt < otf[0] or tt > otf[-1]:
        return None
    i0 = int(np.searchsorted(od_t, t))
    i1 = int(np.searchsorted(od_t, t + int(max_s * 1e9)))
    if i1 - i0 < 3:
        return None
    d0 = float(np.interp(tt, otf, od_dist))
    dist = od_dist[i0:i1] - d0
    # 距離が単調に増える所までを使う（後退・止まって押し戻された所で切る）
    back = np.flatnonzero(np.diff(dist) < -1e-3)
    end = int(back[0]) + 1 if len(back) else len(dist)
    dist = dist[:end]
    if len(dist) < 3:
        return None
    want = ds * np.arange(1, k + 1)
    valid = want <= dist[-1]
    if not valid.any():
        return None
    sl = slice(i0, i0 + end)
    x = np.interp(want, dist, od_x[sl])
    y = np.interp(want, dist, od_y[sl])
    tk = np.interp(want, dist, otf[sl])
    pose = (float(np.interp(tt, otf, od_x)), float(np.interp(tt, otf, od_y)),
            float(np.interp(tt, otf, od_yaw)))
    body = to_body(np.stack([x, y], axis=1), pose)
    v = np.interp(tk, speed_t.astype(np.float64), speed)
    return PathLabel(body.astype(np.float32), v.astype(np.float32), valid, 0.0)


def pursuit_steer(path_xy: np.ndarray, lookahead: float, wheelbase: float, max_steer: float) -> float:
    """経路点（車体座標）を Pure Pursuit で追う舵角 [rad]。ラベルと記録の舵が整合するかの確認用。

    原点から `lookahead` 以上離れた最初の点を狙う。式は `raspi.nav.purepursuit.steer_for_target` と同じ
    `δ = atan(2L·sin η / ld)`。
    """
    r = np.hypot(path_xy[:, 0], path_xy[:, 1])
    far = np.flatnonzero(r >= lookahead)
    j = int(far[0]) if len(far) else len(r) - 1
    ld = float(r[j])
    if ld < 1e-6:
        return 0.0
    eta = math.atan2(float(path_xy[j, 1]), float(path_xy[j, 0]))
    return max(-max_steer, min(max_steer, math.atan(2.0 * wheelbase * math.sin(eta) / ld)))
