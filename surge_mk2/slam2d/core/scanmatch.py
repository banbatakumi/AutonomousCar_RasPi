"""相関スキャンマッチ — 地図に対して今の点群がいちばん合う姿勢を探す。

`raspi/nav/scanmatch.py`から移植。候補姿勢を**まとめて**評価し、得点の最大を
採る（Olson方式）。粗い格子から細かい格子へ3段で絞る。

## なぜ山登り（RMHC）ではないのか

CoreSLAM/BreezySLAMは候補を1つずつ乱数で変異させて登る。C言語なら速いが、
Pythonから回すと1候補ごとに関数呼び出しが挟まって割に合わない。**候補を配列に
して一括評価する方がnumpyと噛み合い、しかも決定論的**（乱数シードで答えが
変わらないので、同じログを流せば同じ地図になる。デバッグでこれが効く）。

## 得点は「壁の点がどれだけ壁に乗ったか」

`OccGrid.score_map()`（壁の周りをなだらかに盛った地図）を点の位置で引いて
平均する。0〜1に正規化してあるので、**閾値で「見失った」を判定できる**。

**飽和点や「空きを彫るだけ」の点は使わない。** 終端に壁が無いのだから、
壁の地図と照合しても意味が無い（`hit`がTrueのものだけを見る）。

## ★ 得点だけで選ぶと、まっすぐな通路で必ず壊れる（corridor problem）

道幅の狭い直線区間はどこを切っても同じ形をしている。**進行方向の位置は
点群からは決まらない**。それでも候補ごとの得点はノイズで僅かに違うので、
素直に最大値を採ると毎周期でたらめな方向へ滑る。しかも**その姿勢で地図を
焼く**ので、ずれた壁が地図に増えて次はもっと滑る。

なので**推測航法（`guess`）からの隔たりに罰則を掛ける**:

    有効得点 = 一致度 − prior_w × ( (Δx² + Δy²)/σxy² + (Δθ/σθ)² )

観測できる方向（壁が近づく向き）では一致度の差が罰則より十分大きいので
マッチが勝ち、観測できない方向では罰則が勝って**推測航法のまま進む**という
自然な解になる。**罰則の強さは「推測航法がどれだけ信用できるか」で決まる**
——`MatcherConfig`の既定値は特定センサ構成（ジャイロ+速度の相補フィルタ的
推測航法、100msで数mm精度）での実測に基づくものであり、他の構成（車輪
オドメトリのみ等、精度が劣る推測航法）では探索範囲・罰則とも再チューニング
が要る。

**「見失った」の判定に使う`score`は罰則を引く前の生の一致度**を返す
（罰則を引いた値で判定すると、正しく合っているのに遠くへ来ただけで
「見失った」になる）。

## ★★ まだ見ていない場所に落ちた点を「外れ」と数えてはいけない

得点を全点数で割ると、地図を作りながら走る間ずっと後ろへ引っ張られる
（前方の点は未探査領域に入りやすく、そこは地図が空なので0点になる。
姿勢を後ろへずらせば既知の壁に乗る点が増えて得点が上がる、というバイアス
が生まれる）。なので**未探査領域に落ちる点は、照合そのものから外す**。

判定は**初期姿勢で1回だけ**行い、以降どの候補もその同じ点の集合で採点する。
候補ごとに分母を数え直すと今度は逆向きに壊れる（残った少数の点だけが
よく合う姿勢が最高得点になる）。**分母は固定でなければ比較にならない。**
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .grid import OccGrid
from .types import Pose2D, ScanPoints, wrap_angle

__all__ = ["MatchResult", "Stage", "MatcherConfig", "DEFAULT_STAGES", "match", "score_at"]


class Stage(NamedTuple):
    """探索1段ぶん。範囲と刻みは**両端を含む**ように使う。"""

    trans: float                           #: 並進の探索半径 [m]
    trans_step: float                      #: 並進の刻み [m]
    rot: float                             #: 回転の探索半径 [rad]
    rot_step: float                        #: 回転の刻み [rad]


#: 粗 → 中 → 細。**範囲は「1周期で推測航法がどれだけ外れうるか」**で決める。
#:
#: ★ この既定値は「ジャイロ+速度の相補フィルタ的推測航法（100msで数mm精度）」
#: という特定センサ構成での実測に基づく参考値であり、そのまま他構成に
#: 使い回すべきではない。推測航法の精度が劣るなら広げる必要がある。
DEFAULT_STAGES: tuple[Stage, ...] = (
    Stage(0.08, 0.02, math.radians(4.0), math.radians(1.0)),
    Stage(0.02, 0.01, math.radians(1.0), math.radians(0.25)),
    Stage(0.008, 0.004, math.radians(0.25), math.radians(0.05)),
)

#: 事前分布の広がり。「1周期で推測航法がこれくらい外れるのは不思議ではない」
#: という値。推測が良ければ狭く取れる。広すぎると通路で一致度がほぼ平坦な
#: ときに罰則が効かなくなる。★ 上と同じく特定センサ構成での参考値
PRIOR_XY = 0.05                            #: [m]
PRIOR_YAW = math.radians(2.0)
#: 罰則の重み。一致度（0〜1）と同じ単位。★ 特定センサ構成での参考値。
#: 上げすぎるとSLAMが「何もしない」ものになる（マッチが効かなくなる）
PRIOR_W = 0.02

#: 照合に使う点の上限。間引いても得点はほとんど変わらない（隣り合う点は
#: ほぼ同じセルを引く）ので、上限を切って最悪ケースの計算時間を固定する
MAX_POINTS = 240
#: 既知セルに落ちる点がこの割合を切ったら探索しない。
#: 照合できる地図がまだ無いということなので、オドメトリのまま進む
MIN_KNOWN_RATIO = 0.25


@dataclass(frozen=True, slots=True)
class MatcherConfig:
    """`match()`のパラメータ一式。既定値は`DEFAULT_STAGES`等のdocstring参照。"""

    stages: tuple[Stage, ...] = DEFAULT_STAGES
    prior_xy: float = PRIOR_XY
    prior_yaw: float = PRIOR_YAW
    prior_w: float = PRIOR_W
    max_points: int = MAX_POINTS
    min_known_ratio: float = MIN_KNOWN_RATIO


class MatchResult(NamedTuple):
    x: float
    y: float
    yaw: float
    score: float                           #: 0〜1。低いのに「合っている」扱いにしない
    #: 探索したか。地図がまだ空なら False（初期値をそのまま返している）
    searched: bool


def _offsets(radius: float, step: float) -> np.ndarray:
    """`-radius … +radius` を `step` 刻みで。**必ず 0 を含む。**

    0を含まないと、どの候補も現在の推定より悪いときに姿勢が勝手に動く。
    """
    n = int(math.floor(radius / step + 1e-9))
    return np.arange(-n, n + 1, dtype=np.float64) * step


def match(grid: OccGrid, pts: ScanPoints, guess: Pose2D, *,
          config: MatcherConfig = MatcherConfig()) -> MatchResult:
    """`guess`の周りを探して、いちばん**有効得点**の高い姿勢を返す。

    :param pts: 脱スキュー済みの点群（共通フレーム座標）。**`hit`の点だけを使う**
    :param guess: 推測航法から作った初期姿勢
    """
    gx, gy, gyaw = guess
    sx = pts.x[pts.hit]
    sy = pts.y[pts.hit]
    if sx.size == 0:
        return MatchResult(gx, gy, gyaw, 0.0, False)

    if sx.size > config.max_points:
        take = np.linspace(0, sx.size - 1, config.max_points).astype(np.int64)
        sx, sy = sx[take], sy[take]

    score_map = grid.score_map()
    if not score_map.any():
        # 地図がまだ空。探索しても全候補0点で、原点付近へ吸い寄せられる
        return MatchResult(gx, gy, gyaw, 0.0, False)

    # ★ 初期姿勢で未探査領域に落ちる点を捨てる（docstringの「まだ見ていない場所」）
    c0, s0 = math.cos(gyaw), math.sin(gyaw)
    col, row = grid.to_cell(gx + sx * c0 - sy * s0, gy + sx * s0 + sy * c0)
    ok = grid.inside(col, row)
    keep = np.zeros(sx.size, dtype=bool)
    keep[ok] = grid.seen[row[ok], col[ok]] > 0
    if keep.sum() < max(1, int(config.min_known_ratio * sx.size)):
        return MatchResult(gx, gy, gyaw, 0.0, False)
    sx, sy = sx[keep], sy[keep]

    best = (gx, gy, gyaw)
    best_raw = _score(grid, score_map, sx, sy, gx, gy, gyaw)
    best_eff = best_raw                     # guess自身は罰則ゼロ

    prior_xy = config.prior_xy
    prior_yaw = config.prior_yaw
    prior_w = config.prior_w

    for st in config.stages:
        cx = best[0] + _offsets(st.trans, st.trans_step)
        cy = best[1] + _offsets(st.trans, st.trans_step)
        cyaw = best[2] + _offsets(st.rot, st.rot_step)
        # 罰則は候補ごとに決まるので、(X, Y)の外積で一度に作れる
        pen_x = ((cx - gx) / max(prior_xy, 1e-6)) ** 2
        pen_y = ((cy - gy) / max(prior_xy, 1e-6)) ** 2

        for yaw in cyaw:
            pen_yaw = (wrap_angle(yaw - gyaw) / max(prior_yaw, 1e-9)) ** 2
            penalty = prior_w * (pen_x[:, None] + pen_y[None, :] + pen_yaw)
            c, s = math.cos(yaw), math.sin(yaw)
            rx = sx * c - sy * s
            ry = sx * s + sy * c
            # **xとyを分けて(X,N)と(Y,N)を作り、最後に(X,Y,N)で組む。**
            # 候補を(X*Y, N)に展開してから引くと、同じ列・同じ行の計算を
            # 何度も繰り返すことになる
            fx = (cx[:, None] + rx[None, :] - grid.origin[0]) / grid.resolution - 0.5
            fy = (cy[:, None] + ry[None, :] - grid.origin[1]) / grid.resolution - 0.5
            totals = _bilinear(score_map, fx, fy) / sx.size    # (X, Y) 生の一致度
            eff = totals - penalty

            k = int(np.argmax(eff))
            v = float(eff.reshape(-1)[k])
            if v > best_eff:
                best_eff = v
                best_raw = float(totals.reshape(-1)[k])
                best = (float(cx[k // totals.shape[1]]),
                        float(cy[k % totals.shape[1]]), float(yaw))

    # **返す得点は罰則を引く前**（「見失った」の判定に使うため。docstring参照）
    return MatchResult(best[0], best[1], wrap_angle(best[2]), best_raw, True)


def score_at(grid: OccGrid, pts: ScanPoints, pose: Pose2D, *,
             config: MatcherConfig = MatcherConfig()) -> float:
    """`pose`1点だけの一致度（罰則なしの生スコア）。

    `match()`の探索本体からは独立に、特定の姿勢の当てはまりの良さだけを
    知りたいときに使う。`core/confidence.py`が局所曲率（＝観測の確からしさ）
    を測るために、`match()`が返した最良解の周辺でこれを何度も呼ぶ。
    """
    sx = pts.x[pts.hit]
    sy = pts.y[pts.hit]
    if sx.size == 0:
        return 0.0
    if sx.size > config.max_points:
        take = np.linspace(0, sx.size - 1, config.max_points).astype(np.int64)
        sx, sy = sx[take], sy[take]
    score_map = grid.score_map()
    if not score_map.any():
        return 0.0
    return _score(grid, score_map, sx, sy, pose.x, pose.y, pose.yaw)


def _bilinear(sm: np.ndarray, fx: np.ndarray, fy: np.ndarray) -> np.ndarray:
    """`sm`を**セル中心基準の実数座標**で引き、点ごとの値を候補ごとに合計する。

    `fx`は(X, N)、`fy`は(Y, N)。戻りは(X, Y)。

    ## なぜ最近傍ではなく補間するのか

    最近傍で引くと得点がセル幅刻みの**階段**になる。同じセルの中では候補が
    何mm違っても得点が1ビットも動かないので、細かい段の探索が完全に無意味
    になり、平坦部のどれが選ばれるかは配列の並び順で決まってしまう。

    補間すると得点がセル間で連続になり、サブセルの位置に意味が出る。
    引く回数は4倍になるが、階段のまま細かく探すより安い。
    """
    h, w = sm.shape
    x0 = np.floor(fx).astype(np.int32)
    y0 = np.floor(fy).astype(np.int32)
    tx = (fx - x0).astype(np.float32)[:, None, :]              # (X, 1, N)
    ty = (fy - y0).astype(np.float32)[None, :, :]              # (1, Y, N)

    okx = (x0 >= 0) & (x0 < w - 1)
    oky = (y0 >= 0) & (y0 < h - 1)
    cc = np.where(okx, x0, 0)
    rr = np.where(oky, y0, 0)
    ok = okx[:, None, :] & oky[None, :, :]

    r0 = rr[None, :, :]
    c0 = cc[:, None, :]
    v00 = sm[r0, c0]
    v01 = sm[r0, c0 + 1]
    v10 = sm[r0 + 1, c0]
    v11 = sm[r0 + 1, c0 + 1]
    val = ((v00 * (1 - tx) + v01 * tx) * (1 - ty)
           + (v10 * (1 - tx) + v11 * tx) * ty)
    return np.where(ok, val, 0.0).sum(axis=2)


def _score(grid: OccGrid, score_map: np.ndarray,
           sx: np.ndarray, sy: np.ndarray,
           x: float, y: float, yaw: float) -> float:
    """1姿勢ぶんの一致度。**探索と同じ物差し**でないとguessだけ甘くなる。"""
    c, s = math.cos(yaw), math.sin(yaw)
    fx = (x + sx * c - sy * s - grid.origin[0]) / grid.resolution - 0.5
    fy = (y + sx * s + sy * c - grid.origin[1]) / grid.resolution - 0.5
    return float(_bilinear(score_map, fx[None, :], fy[None, :])[0, 0] / sx.size)
