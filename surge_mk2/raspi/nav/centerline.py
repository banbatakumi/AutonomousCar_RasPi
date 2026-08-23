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

## ★ 壁の小さな穴が中心線を暴れさせる

`measure()` のレイは `OccGrid.raycast()`（既定 `fill=1` セルまで穴を埋める）を
使うが、それでも埋め切れない穴（数セル分、`min_hits` を満たせなかった箇所。
`raspi/nav/grid.py` の `_END_BACKOFF` docstring 参照）を抜けると、**本来の
壁ではなく奥の別の壁**に当たり、その1点だけ幅が数十cm〜`max_width`へ跳ね上がる。

`build()` の反復は「幅の中間へ寄せる」ので、この1点の異常値をそのまま使うと
**その1点だけ大きく横へ飛ぶ**（GUI で中心線が局所的に暴れて見える症状）。
しかも異常に測った幅がそのまま `w_left`/`w_right` として `nav/raceline.py` の
最適化に渡ると、**壁の穴の分だけ実際には無い余白がある**とレーシングラインが
誤認し、そこだけ壁側へ寄った経路を引きかねない。

そこで**近傍の中央値より明らかに大きい値だけ**を中央値へクランプする
（`_declutter()`）。**小さい方へは倒さない**——本当に道が狭い場所を
誤って広げるのは避けたいので、「穴を抜けて広く見えすぎた」側だけを直す。
これは穴そのものを塞ぐわけではない（`raycast` の結果はそのまま）ので、
コース出口のような**本物の広い区間**は連続して広いままで中央値も高く保たれ、
クランプされない。

## ★★ ヘアピン・シケインでは「穴」が無くても暴れる（知覚エイリアシング）

`_declutter()` は**孤立した1点の異常値**には効くが、`fuji` コース（急な
0.5m 半径のシケイン区間）を `sim.bench` で実測したところ、**穴が1つも
無くても**その区間だけ幅の測定が数点にわたって暴れることを確認した
（例: `left/right` が `0.63/1.73 → 0.43/1.63 → 2.41/0.04 → 2.26/1.95`
と隣接点ごとに大きく反転）。近傍の複数点が同時に暴れるので、中央値
そのものが汚染されて `_declutter()` では捉えられない。

原因は穴ではなく**法線方向の推定の脆さ**。急カーブでは中央差分
（`tangents()`）で作る接ベクトルがわずかに傾いただけで、その法線が
本来の壁（数十cm先）ではなく**通路に沿って何mも先**まで抜けてしまう
（浅い角度でレイが壁を舐める、`nav/grid.py` の `_END_BACKOFF` と同じ
現象が中心線側でも起きる）。シケインでは通路の先が別の区間（あるいは
反対側の壁）なので、抜けた先で拾う距離が点ごとにばらつき、幅が
無関係な値へ跳ねる。

これを幅の異常値検出だけで直すのは筋が悪い（何が「正しい壁」かを
区別する情報が無い）。代わりに**1回の反復で動かせる量に上限を掛ける**
（`_MAX_SHIFT_M`）。1回の測定がどれだけ暴れても、中心線の点が
一気に1m以上テレポートすることはなくなり、`iters` 回の反復で
（暴れが収まった後続の測定に助けられながら）緩やかに真ん中へ寄っていく。
GUI で見えた「点線が一直線に飛ぶ」症状は主にこのテレポートが原因なので、
これで大幅に緩和されるはず——ただし**急カーブ内の測定精度そのものが
上がるわけではない**（下記「未解決」参照）。
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


#: `_declutter()` の窓幅 [点]。奇数。step=0.10m なら 0.5m ぶんの近傍を見る
_SPIKE_WINDOW = 5
#: 近傍の中央値よりこれ以上大きければ「穴を抜けた」とみなす [m]
_SPIKE_JUMP = 0.5
#: `build()` の反復1回で動かしてよい量の上限 [m]（上の docstring「ヘアピン・
#: シケインでは穴が無くても暴れる」参照）。**小さくしすぎない**——本当に
#: 中心へ寄せたい量（道幅のズレの半分、数十cm）まで削ると収束しない
_MAX_SHIFT_M = 0.15


def _median_filter_loop(x: np.ndarray, k: int) -> np.ndarray:
    """巡回配列の移動中央値。"""
    n = len(x)
    if n < k:
        return x.copy()
    pad = k // 2
    wrapped = np.concatenate([x[-pad:], x, x[:pad]])
    windows = np.lib.stride_tricks.sliding_window_view(wrapped, k)
    return np.median(windows, axis=1)


def _declutter(width: np.ndarray) -> np.ndarray:
    """近傍の中央値より明らかに大きい値だけを中央値へクランプする（上の docstring）。"""
    if len(width) < _SPIKE_WINDOW:
        return width
    med = _median_filter_loop(width, _SPIKE_WINDOW)
    spike = width > med + _SPIKE_JUMP
    out = width.copy()
    out[spike] = med[spike]
    return out


def measure(grid: OccGrid, xy: np.ndarray, nrm: np.ndarray,
            max_width: float) -> tuple[np.ndarray, np.ndarray]:
    """各点から左右の壁までの距離 [m]。**壁の穴は `raycast` が塞いでくれる。**

    左右 2N 本を**一度に**撃つ。点ごとに呼ぶと 278 点で 130ms 掛かり、
    BUILD 段の予算（1周期 15ms）を1発で使い切る。

    塞ぎ切れなかった穴を抜けて奥の壁を拾った異常値は `_declutter()` で
    近傍の中央値へクランプする（上の docstring「壁の小さな穴が中心線を
    暴れさせる」）。
    """
    ang = np.arctan2(nrm[:, 1], nrm[:, 0])
    both = grid.raycast(np.concatenate([xy[:, 0], xy[:, 0]]),
                        np.concatenate([xy[:, 1], xy[:, 1]]),
                        np.concatenate([ang, ang + np.pi]), max_width)
    n = len(xy)
    return _declutter(both[:n]), _declutter(both[n:])


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
        # 左右の壁の真ん中へ寄せる。**寄せてから測り直す**ことで法線の傾きも直る。
        # ★ 1回で動かす量は `_MAX_SHIFT_M` に上限を掛ける——急カーブで法線が
        # 通路の先まで抜けた1回の暴れが、そのまま1m以上のテレポートに
        # ならないようにする（上の docstring「ヘアピン・シケインでは穴が
        # 無くても暴れる」参照）
        shift = np.clip((left - right) / 2.0, -_MAX_SHIFT_M, _MAX_SHIFT_M)
        xy = xy + nrm * shift[:, None]
        xy = smooth_loop(resample_loop(xy, step), smooth)
        nrm = normals(xy)
        left, right = measure(grid, xy, nrm, max_width)

    return Centerline(xy=xy, normal=nrm, w_left=left, w_right=right, step=step)
