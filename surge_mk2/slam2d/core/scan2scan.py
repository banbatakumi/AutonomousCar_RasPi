"""スキャン対スキャン — 連続する2周の点群を**格子を介さず**合わせる。

`raspi/nav/scan2scan.py`から移植。`core/scanmatch.py`は点群を占有格子
（量子化された地図）に照合するが、ここは点どうしを直接合わせるので量子化に
縛られない。得られるのは**1周期ぶんの相対移動**（絶対位置ではない）で、
それを地図との照合の初期値として使う設計を想定している。

## 点対線のICP（PLICP）

点対点で合わせると、平らな壁に沿った方向に「引っ掛かり」が無いのに無理やり
対応点を作ってしまい、壁沿いに滑る。**点と、相手の局所的な壁の線との距離**を
最小化すれば、壁に垂直な方向だけが拘束され、沿う方向は自由に残る。これは
**正しい**振る舞い: 平行な壁しか見えない区間で進行方向が決まらないのは幾何の
事実で、無理に決めると嘘の値が出る。

    誤差 = Σ ( n_i · ( R(θ)p_i + t − q_i ) )²        n_i は q_i まわりの壁の法線

θが小さいとして線形化すると3変数の最小二乗になり、3×3を解くだけで済む。

## 対応点は「方位」で引く

前の周の点は方位順に並んでいるので、変換後の点の方位から相手をO(1)で引ける
（極座標スキャンマッチングの発想）。KD木を組むより速く、numpyで一度に処理できる。

## ★ 観測できない方向は「勝手に残る」のではなく、明示的に押さえる

平行な壁の区間では進行方向が決まらない（corridor problem）。「ICPは観測
できる方向にだけ動くので、決まらない方向は初期値がそのまま残る」というのは
誤り——測距ノイズがあると、拘束の無い方向でも最小二乗解は毎回ふらつき、
それが積み上がる。

正しくは、**初期値（`guess`）へ引き戻す項を最小二乗に足す**（ティホノフ正則化）:

    最小化  Σ ( n_i · ( R(θ)p_i + t − q_i ) )²  +  λ_t‖t − t₀‖² + λ_r(θ − θ₀)²

こうすると、壁が拘束する方向はデータ側の情報量が大きいのでICPが勝ち
（量子化に縛られない精度）、拘束の無い方向はデータ側の情報量がほぼ0なので
初期値が残る（`guess`の値）、が自動的に成り立つ。

**`λ`は点数に比例させてはいけない。** 事前分布は「1本の拘束」であって
データ点と同じ数だけあるわけではない。正しくは

    λ_t = (点の残差のばらつき / guessの並進の不確かさ)²
    λ_r = (点の残差のばらつき / (guessの回頭の不確かさ × 腕の長さ))²

という**絶対値**で、これは「事前分布はデータ何行ぶんの重みか」を表す。

★ `IcpConfig`の既定値（`RESID_SIGMA`・`PRIOR_XY`・`PRIOR_YAW`）はLD06の測距誤差
と「ジャイロ+速度」推測航法の精度という特定センサ構成での実測値であり、
他構成では要再チューニング。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .types import Pose2D, ScanPoints, wrap_angle

__all__ = ["Delta", "IcpConfig", "match_scans"]

#: 対応点として認める距離[m]。これより離れた相手は別の壁とみなして捨てる。
#: 1周期の移動量より大きく、通路の幅より小さく取る
MAX_PAIR_M = 0.30
#: 法線を推定するのに使う近傍の点数（片側）
_NORMAL_SPAN = 3
#: 参照する点が少なすぎたら諦める
MIN_POINTS = 40
#: 点の残差のばらつき[m]。★特定センサ（LD06、測距σ=1cm+0.5%×距離）の実測値
RESID_SIGMA = 0.013
#: guessの1周期ぶんの不確かさ（事前分布の広がり）。
#: ★「ジャイロ+速度」推測航法での実測値。他の推測航法では要再チューニング
PRIOR_XY = 0.005                           #: [m]
PRIOR_YAW = math.radians(0.3)
#: 回転の列 `n × p` の代表的な大きさ[m]。腕の長さぶん効くので換算に要る
LEVER = 2.0


@dataclass(frozen=True, slots=True)
class IcpConfig:
    """`match_scans()`のパラメータ一式。既定値は上のモジュールdocstring参照。"""

    iters: int = 8
    max_pair: float = MAX_PAIR_M
    prior_xy: float = PRIOR_XY
    prior_yaw: float = PRIOR_YAW
    resid_sigma: float = RESID_SIGMA
    lever: float = LEVER
    min_points: int = MIN_POINTS


class Delta(NamedTuple):
    """1周期ぶんの相対移動（前の周の車体座標での値）。"""

    dx: float
    dy: float
    dyaw: float
    #: 対応が付いた点の割合 0〜1。低いなら信じない
    inlier: float
    #: 対応点の残差の中央値[m]。大きいなら形が合っていない
    residual: float
    ok: bool                               #: 使ってよいか

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self.dx, self.dy, self.dyaw)


def _normals(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """各点まわりの局所的な壁の向きから法線を作る `(N, 2)`。

    前後`_NORMAL_SPAN`点を結んだ向きの垂線を使う。点群は方位順に並んでいるので、
    隣り合う添字は空間的にも隣（1周を跨ぐところだけ巻き込む）。
    """
    n = x.size
    k = min(_NORMAL_SPAN, max(1, n // 4))
    tx = np.roll(x, -k) - np.roll(x, k)
    ty = np.roll(y, -k) - np.roll(y, k)
    d = np.hypot(tx, ty)
    d[d < 1e-9] = 1.0
    # 接線を90°回して法線に
    return np.column_stack([-ty / d, tx / d])


def _bearing_index(x: np.ndarray, y: np.ndarray, bins: int = 720) -> np.ndarray:
    """方位 → その方位にいちばん近い点の添字。無ければ−1。

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


def match_scans(prev: ScanPoints, cur: ScanPoints,
                guess: Pose2D = Pose2D(0.0, 0.0, 0.0), *,
                config: IcpConfig = IcpConfig()) -> Delta:
    """`cur`を`prev`に合わせる相対移動を返す。

    :param guess: 推測航法から作った初期値（前の周の車体座標）

    **観測できない方向は`guess`に留まる**（上のdocstringの正則化）。
    """
    px, py = prev.x[prev.hit], prev.y[prev.hit]
    cx, cy = cur.x[cur.hit], cur.y[cur.hit]
    if px.size < config.min_points or cx.size < config.min_points:
        return Delta(guess.x, guess.y, guess.yaw, 0.0, math.inf, False)

    nrm = _normals(px, py)
    bins = 720
    lut = _bearing_index(px, py, bins)

    tx, ty, th = guess.x, guess.y, guess.yaw
    inlier = 0.0
    residual = math.inf

    for _ in range(config.iters):
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

        use = (best_j >= 0) & (best_d <= config.max_pair)
        inlier = float(use.sum()) / qx.size
        if use.sum() < config.min_points:
            return Delta(tx, ty, th, inlier, residual, False)

        j = best_j[use]
        nx, ny = nrm[j, 0], nrm[j, 1]
        ex = qx[use] - px[j]
        ey = qy[use] - py[j]
        err = nx * ex + ny * ey                    # 点と壁の線との距離（符号付き）
        residual = float(np.median(np.abs(err)))

        # θが小さいとして線形化: 残差 = n·t + θ(n×p) + n·(p−q)
        # 変数は(dtx, dty, dth)の3つだけなので3×3を解けばよい
        rot = nx * (-qy[use]) + ny * qx[use]
        a = np.column_stack([nx, ny, rot])
        ata = a.T @ a
        atb = a.T @ (-err)

        # ★ 初期値へ引き戻す項。**拘束の無い方向だけで効く**（データ側の
        #   情報量が0に近いので、そこでは小さなλでも支配的になる）。
        #   **点数に比例させない**（事前分布は1本の拘束。上のdocstring）
        lam_t = (config.resid_sigma / max(config.prior_xy, 1e-6)) ** 2
        lam_r = (config.resid_sigma / max(config.prior_yaw * config.lever, 1e-6)) ** 2
        lam = np.array([lam_t, lam_t, lam_r])
        dev = np.array([tx - guess.x, ty - guess.y, wrap_angle(th - guess.yaw)])
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

    ok = inlier >= 0.5 and residual < config.max_pair * 0.5
    return Delta(tx, ty, wrap_angle(th), inlier, residual, ok)
