"""走った跡 → コースの中心線と道幅。

レーシングラインの最適化は「中心線 `c_i`・法線 `n_i`・左右の余裕 `w_i`」の3つを
入力に取る。ここはそれを地図から作る。

## 初期値は SLAM の軌跡

1周目は Follow the Gap で走っているので、軌跡は**道の真ん中を通っていない**
（FTG は視野の開き方でギャップの中央を決めるので、コーナーでは内側に寄る）。
それでも初期値として使えるのは、**実行可能な範囲を決めるのは `c_i` ではなく
`(n_i, w_i)` だから**。中心線が多少ずれていても、そこから左右へ測った幅が
正しければ最適化の答えは変わらない。

**本当に効く弱点は法線の向き。** `c_i` が壁に対して斜めに走っていると法線も
斜めに当たり、道幅を過大に測る（＝壁にめり込む解が実行可能に見える）。
だから「幅を測る → 真ん中へ寄せる」を数回繰り返す。1回 1ms 以下なので惜しまない。

## 距離変換（EDT）は使わない

中心線を「壁からの距離が極大になる尾根」として求める手もあるが、厳密な EDT は
400×400 で数百 ms 掛かり、近似すると 2 割近い誤差が出る。**法線方向にレイを
撃って測る方が直接的で速く、`scipy` も要らない。**
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

from .grid import OccGrid

__all__ = ["Centerline", "build", "resample_loop", "smooth_loop", "close_gap"]


class Centerline(NamedTuple):
    """閉ループの中心線。**添字は巡回**（`i-1`, `i+1` は mod N）。"""

    xy: np.ndarray                         #: (N, 2) [m]
    normal: np.ndarray                     #: (N, 2) 単位法線。**左が正**（y=左に合わせる）
    w_left: np.ndarray                     #: (N,) 左の壁までの余裕 [m]（車体半幅を引く前）
    w_right: np.ndarray                    #: (N,) 右の壁まで [m]
    step: float                            #: 点の間隔 [m]

    def __len__(self) -> int:
        return int(self.xy.shape[0])


def close_gap(xy: np.ndarray) -> np.ndarray:
    """始点と終点のずれを**軌跡全体へ線形に配分して**閉じる。

    スキャンマッチだけだとループクロージャが無いので、1周して戻ってきたとき
    始点と数 cm〜数十 cm ずれる。終点だけを始点に貼り付けると、その1箇所に
    折れ目ができて曲率が跳ね、レーシングラインがそこだけ暴れる。
    **ずれを1周かけて薄く配る**のが、軽量なポーズグラフ最適化の代わりになる。
    """
    if len(xy) < 3:
        return xy
    gap = xy[0] - xy[-1]
    w = np.linspace(0.0, 1.0, len(xy))[:, None]
    return xy + gap * w


def resample_loop(xy: np.ndarray, step: float) -> np.ndarray:
    """閉ループを弧長 `step` [m] で等間隔に取り直す。

    SLAM の軌跡は速度によって点の密度がばらつく（止まれば同じ場所に溜まる）。
    曲率は隣接点の2階差分で測るので、**間隔が揃っていないと曲率が速度の関数に
    化ける**。等間隔にしてから最適化に渡す。
    """
    if len(xy) < 3:
        return xy
    loop = np.vstack([xy, xy[:1]])                 # 終点→始点の区間も入れて閉じる
    seg = np.hypot(*np.diff(loop, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    n = max(3, int(round(total / step)))
    # **最後の点は入れない。** 始点と重なって間隔 0 の区間ができ、法線が壊れる
    target = np.linspace(0.0, total, n, endpoint=False)
    return np.column_stack([np.interp(target, s, loop[:, 0]),
                            np.interp(target, s, loop[:, 1])])


def smooth_loop(xy: np.ndarray, window: int) -> np.ndarray:
    """巡回の移動平均。**SLAM のがたつきを落とす。**

    曲率は2階差分なので、5cm の位置ノイズが 20cm 間隔の点に乗ると曲率が
    桁で暴れる。最適化に渡す前にここで均しておく。
    """
    if window <= 1 or len(xy) < window:
        return xy
    k = np.ones(window) / window
    n = len(xy)
    pad = window
    out = np.empty_like(xy)
    for j in range(2):
        wrapped = np.concatenate([xy[-pad:, j], xy[:, j], xy[:pad, j]])
        out[:, j] = np.convolve(wrapped, k, mode="same")[pad:pad + n]
    return out


def tangents(xy: np.ndarray) -> np.ndarray:
    """巡回の中央差分による単位接ベクトル。"""
    d = np.roll(xy, -1, axis=0) - np.roll(xy, 1, axis=0)
    n = np.hypot(d[:, 0], d[:, 1])
    n[n < 1e-9] = 1.0
    return d / n[:, None]


def normals(xy: np.ndarray) -> np.ndarray:
    """左向きの単位法線。接ベクトルを **+90° 回した**もの（y=左が正の規約）。"""
    t = tangents(xy)
    return np.column_stack([-t[:, 1], t[:, 0]])


def measure(grid: OccGrid, xy: np.ndarray, nrm: np.ndarray,
            max_width: float) -> tuple[np.ndarray, np.ndarray]:
    """各点から左右の壁までの距離 [m]。**壁の穴は `raycast` が塞いでくれる。**

    左右 2N 本を**一度に**撃つ。点ごとに呼ぶと 278 点で 130ms 掛かり、
    BUILD 段の予算（1周期 15ms）を1発で使い切る。
    """
    ang = np.arctan2(nrm[:, 1], nrm[:, 0])
    both = grid.raycast(np.concatenate([xy[:, 0], xy[:, 0]]),
                        np.concatenate([xy[:, 1], xy[:, 1]]),
                        np.concatenate([ang, ang + np.pi]), max_width)
    n = len(xy)
    return both[:n], both[n:]


def lateral_offset(cl: Centerline, xy: np.ndarray) -> np.ndarray:
    """`xy` の各点が中心線からどれだけ横にずれているか [m]。**左が正。**

    最適化を複数回まわすと、2回目以降の `α` は「1回目の経路からのずれ」に
    なってしまう。**人間が読む数字は最後まで中心線基準**でないと、
    「アウト側に振っているか」が判断できない。
    """
    d2 = ((xy[:, None, 0] - cl.xy[None, :, 0]) ** 2
          + (xy[:, None, 1] - cl.xy[None, :, 1]) ** 2)
    j = np.argmin(d2, axis=1)
    d = xy - cl.xy[j]
    return d[:, 0] * cl.normal[j, 0] + d[:, 1] * cl.normal[j, 1]


def build(grid: OccGrid, traj: np.ndarray, *, step: float = 0.10,
          max_width: float = 3.0, smooth: int = 5, iters: int = 3) -> Centerline:
    """軌跡と地図から中心線を作る。

    :param traj: `(N, 2)` または `(N, 3)` の走行軌跡。**1周ぶん**を渡すこと
    :param iters: 「幅を測る → 真ん中へ寄せる」の反復回数（docstring 参照）
    """
    xy = np.asarray(traj, dtype=np.float64)[:, :2]
    xy = resample_loop(close_gap(xy), step)
    xy = smooth_loop(xy, smooth)

    nrm = normals(xy)
    left, right = measure(grid, xy, nrm, max_width)
    for _ in range(max(0, iters - 1)):
        # 左右の壁の真ん中へ寄せる。**寄せてから測り直す**ことで法線の傾きも直る
        xy = xy + nrm * ((left - right) / 2.0)[:, None]
        xy = smooth_loop(resample_loop(xy, step), smooth)
        nrm = normals(xy)
        left, right = measure(grid, xy, nrm, max_width)

    return Centerline(xy=xy, normal=nrm, w_left=left, w_right=right, step=step)
