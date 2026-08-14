"""スキャン対スキャン — 連続する2周の点群を**格子を介さず**合わせる。

## なぜ要るのか

`nav/scanmatch.py` は点群を**地図**（2.5cm 格子）に照合する。地図に焼いた時点で
壁の位置が 2.5cm に量子化されているので、**1回の推定精度の下限がそこで決まる**。
実測で格子を 5cm → 2.5cm にすると誤差が 52cm → 31cm に下がったのが、
「量子化で情報を捨てている」ことの証拠だった。

ここは点どうしを直接合わせるので量子化に縛られない。得られるのは
**1周期ぶんの相対移動**（絶対位置ではない）で、それを地図との照合の初期値にする。

## 点対線の ICP（PLICP）

点対点で合わせると、平らな壁に沿った方向に「引っ掛かり」が無いのに
無理やり対応点を作ってしまい、壁沿いに滑る。**点と、相手の局所的な壁の線との
距離**を最小化すれば、壁に垂直な方向だけが拘束され、沿う方向は自由に残る。
これは**正しい**振る舞い: 平行な壁しか見えない区間で進行方向が決まらないのは
幾何の事実で、無理に決めると嘘の値が出る（実測で 5m 走って 4.5m ずれた）。

    誤差 = Σ ( n_i · ( R(θ)p_i + t − q_i ) )²        n_i は q_i まわりの壁の法線

θ が小さいとして線形化すると 3 変数の最小二乗になり、3×3 を解くだけで済む。

## 対応点は「方位」で引く

前の周の点は**方位順に並んでいる**ので、変換後の点の方位から相手を O(1) で
引ける（極座標スキャンマッチングの発想）。全点に対して KD 木を組むより速く、
numpy で一度に処理できる。

## ★ 観測できない方向は「勝手に残る」のではなく、明示的に押さえる

平行な壁の区間では進行方向が決まらない（corridor problem）。当初「ICP は
観測できる方向にだけ動くので、決まらない方向は初期値がそのまま残る」と
考えたが、**それは誤り**。測距ノイズがあると、拘束の無い方向でも最小二乗解は
毎回ふらつき、それが積み上がる。オーバルコースの実測で
**s2s なし 11.6cm → s2s あり 84.8cm** と悪化した。

正しくは、**推測航法の初期値へ引き戻す項を最小二乗に足す**（ティホノフ正則化）:

    最小化  Σ ( n_i · ( R(θ)p_i + t − q_i ) )²  +  λ_t‖t − t₀‖² + λ_r(θ − θ₀)²

こうすると、

- 壁が拘束する方向 … データ側の情報量が大きいので **ICP が勝つ**（量子化に縛られない精度）
- 拘束の無い方向  … データ側の情報量がほぼ 0 なので **初期値が残る**（推測航法の値）

が自動的に成り立つ。

**`λ` は点数に比例させてはいけない。** 事前分布は「1本の拘束」であって
データ点と同じ数だけあるわけではない。比例させると点が多いほど初期値に
張り付き、実測で 0.10m の移動が 0.038m にしか出なくなった。正しくは

    λ_t = (点の残差のばらつき / 推測航法の並進の不確かさ)²
    λ_r = (点の残差のばらつき / (推測航法の回頭の不確かさ × 腕の長さ))²

という**絶対値**で、これは「事前分布はデータ何行ぶんの重みか」を表す。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .deskew import Points

__all__ = ["Delta", "match_scans"]

#: 対応点として認める距離 [m]。**これより離れた相手は別の壁**とみなして捨てる。
#: 1周期の移動（10Hz・1m/s で 10cm）より大きく、通路の幅より小さく取る
MAX_PAIR_M = 0.30
#: 法線を推定するのに使う近傍の点数（片側）
_NORMAL_SPAN = 3
#: 参照する点が少なすぎたら諦める
MIN_POINTS = 40
#: 点の残差のばらつき [m]。LD06 の測距 σ（1cm + 0.5%×距離）から
RESID_SIGMA = 0.013
#: 推測航法の 1 周期ぶんの不確かさ。**これが事前分布の広がり**。
#: 前進は `speed` から（倍率誤差 2% で 0.45m/s なら 1mm 以下）、
#: 回頭はジャイロから（バイアス 0.02°/s なら 0.002°）。**どちらも小さいので
#: 事前分布は強いが、それでもデータ 300 点には敵わない**（＝観測できる方向では
#: ICP が勝つ）。0 にすると拘束の無い方向で流れる（実測 11.6cm → 84.8cm）
PRIOR_XY = 0.005                           #: [m]
PRIOR_YAW = math.radians(0.3)
#: 回転の列 `n × p` の代表的な大きさ [m]。腕の長さぶん効くので換算に要る
LEVER = 2.0


class Delta(NamedTuple):
    """1周期ぶんの相対移動（前の周の車体座標での値）。"""

    dx: float
    dy: float
    dyaw: float
    #: 対応が付いた点の割合 0〜1。**低いなら信じない**
    inlier: float
    #: 対応点の残差の中央値 [m]。大きいなら形が合っていない
    residual: float
    ok: bool                               #: 使ってよいか

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self.dx, self.dy, self.dyaw)


def _normals(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """各点まわりの局所的な壁の向きから法線を作る `(N, 2)`。

    前後 `_NORMAL_SPAN` 点を結んだ向きの垂線を使う。点群は方位順に並んでいるので、
    **隣り合う添字は空間的にも隣**（1周を跨ぐところだけ巻き込む）。
    """
    n = x.size
    k = min(_NORMAL_SPAN, max(1, n // 4))
    tx = np.roll(x, -k) - np.roll(x, k)
    ty = np.roll(y, -k) - np.roll(y, k)
    d = np.hypot(tx, ty)
    d[d < 1e-9] = 1.0
    # 接線を 90° 回して法線に
    return np.column_stack([-ty / d, tx / d])


def _bearing_index(x: np.ndarray, y: np.ndarray, bins: int = 720) -> np.ndarray:
    """方位 → その方位にいちばん近い点の添字。無ければ −1。

    同じ方位に複数の点が来たら**近い方**を採る（手前の壁が奥を隠す、が正しい）。
    """
    idx = np.full(bins, -1, dtype=np.int64)
    best = np.full(bins, np.inf)
    b = ((np.arctan2(y, x) / (2 * math.pi) + 1.0) * bins).astype(np.int64) % bins
    r = np.hypot(x, y)
    # 近い順に入れると、後から来た遠い点で上書きされない
    order = np.argsort(r)
    for i in order:
        bi = b[i]
        if r[i] < best[bi]:
            best[bi] = r[i]
            idx[bi] = i
    return idx


def match_scans(prev: Points, cur: Points,
                guess: tuple[float, float, float] = (0.0, 0.0, 0.0), *,
                iters: int = 8, max_pair: float = MAX_PAIR_M,
                prior_xy: float = PRIOR_XY, prior_yaw: float = PRIOR_YAW) -> Delta:
    """`cur` を `prev` に合わせる相対移動を返す。

    :param guess: 推測航法から作った初期値 `(dx, dy, dyaw)`（前の周の車体座標）
    :param iters: ICP の反復回数
    :param prior_xy: 推測航法の並進の不確かさ [m]。**大きいほど ICP を信じる**
    :param prior_yaw: 同 回頭 [rad]

    **観測できない方向は `guess` に留まる**（上の docstring の正則化）。
    """
    px, py = prev.x[prev.hit], prev.y[prev.hit]
    cx, cy = cur.x[cur.hit], cur.y[cur.hit]
    if px.size < MIN_POINTS or cx.size < MIN_POINTS:
        return Delta(*guess, 0.0, math.inf, False)

    nrm = _normals(px, py)
    bins = 720
    lut = _bearing_index(px, py, bins)

    tx, ty, th = guess
    inlier = 0.0
    residual = math.inf

    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        qx = cx * c - cy * s + tx
        qy = cx * s + cy * c + ty

        # 方位で相手を引く。前後のビンも見て、いちばん近いものを選ぶ
        b = ((np.arctan2(qy, qx) / (2 * math.pi) + 1.0) * bins).astype(np.int64) % bins
        best_j = np.full(qx.size, -1, dtype=np.int64)
        best_d = np.full(qx.size, np.inf)
        for off in (-2, -1, 0, 1, 2):
            j = lut[(b + off) % bins]
            ok = j >= 0
            if not ok.any():
                continue
            jj = np.where(ok, j, 0)
            d = np.hypot(qx - px[jj], qy - py[jj])
            better = ok & (d < best_d)
            best_d = np.where(better, d, best_d)
            best_j = np.where(better, jj, best_j)

        use = (best_j >= 0) & (best_d <= max_pair)
        inlier = float(use.sum()) / qx.size
        if use.sum() < MIN_POINTS:
            return Delta(tx, ty, th, inlier, residual, False)

        j = best_j[use]
        nx, ny = nrm[j, 0], nrm[j, 1]
        ex = qx[use] - px[j]
        ey = qy[use] - py[j]
        err = nx * ex + ny * ey                    # 点と壁の線との距離（符号付き）
        residual = float(np.median(np.abs(err)))

        # θ が小さいとして線形化: 残差 = n·t + θ (n × p) + n·(p−q)
        # 変数は (dtx, dty, dth) の3つだけなので 3×3 を解けばよい
        rot = nx * (-qy[use]) + ny * qx[use]
        a = np.column_stack([nx, ny, rot])
        ata = a.T @ a
        atb = a.T @ (-err)

        # ★ 初期値へ引き戻す項。**拘束の無い方向だけで効く**（データ側の
        #   情報量が 0 に近いので、そこでは小さな λ でも支配的になる）。
        #   **点数に比例させない**（事前分布は1本の拘束。上の docstring）
        lam_t = (RESID_SIGMA / max(prior_xy, 1e-6)) ** 2
        lam_r = (RESID_SIGMA / max(prior_yaw * LEVER, 1e-6)) ** 2
        lam = np.array([lam_t, lam_t, lam_r])
        dev = np.array([tx - guess[0], ty - guess[1], _wrap(th - guess[2])])
        ata[0, 0] += lam[0]
        ata[1, 1] += lam[1]
        ata[2, 2] += lam[2]
        atb -= lam * dev
        try:
            dtx, dty, dth = np.linalg.solve(ata + np.eye(3) * 1e-9, atb)
        except np.linalg.LinAlgError:
            break

        # 1回の補正が大きすぎるときは刻む（線形化が破れている）
        step = min(1.0, 0.15 / max(1e-6, math.hypot(dtx, dty)))
        tx += float(dtx) * step
        ty += float(dty) * step
        th += float(dth) * step
        if abs(dtx) < 1e-4 and abs(dty) < 1e-4 and abs(dth) < 1e-5:
            break

    ok = inlier >= 0.5 and residual < max_pair * 0.5
    return Delta(tx, ty, _wrap(th), inlier, residual, ok)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
