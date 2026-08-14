"""相関スキャンマッチ — 地図に対して今の点群がいちばん合う姿勢を探す。

候補姿勢を**まとめて**評価し、得点の最大を採る（Olson 方式）。粗い格子から
細かい格子へ 3 段で絞る。

## なぜ山登り（RMHC）ではないのか

CoreSLAM/BreezySLAM は候補を 1 つずつ乱数で変異させて登る。C で書くなら速いが、
Python から回すと 1 候補ごとに関数呼び出しが挟まって割に合わない。
**候補を配列にして一括評価する方が numpy と噛み合い、しかも決定論的**（乱数シードで
答えが変わらないので、同じログを流せば同じ地図になる。デバッグでこれが効く）。

## 探索の初期値は等速度モデル

`Slam` が**自分の推定姿勢の履歴**から「たぶんここに居る」を作って渡す
（車輪もジャイロも使わない。`nav/slam.py` の冒頭）。粗探索の範囲は
**10Hz で予測がどれだけ外れうるか**から決めてある: 2 m/s² の加速で 2cm、
2 rad/s² の角加速度で 0.6°。±20cm / ±8° あれば足りる。

## 得点は「壁の点がどれだけ壁に乗ったか」

`OccGrid.score_map()`（壁の周りをなだらかに盛った地図）を点の位置で引いて平均する。
0〜1 に正規化してあるので、**閾値で「見失った」を判定できる**。壁の裾を作って
あるおかげで、粗い刻みでも正しい方向へ落ちる。

**飽和点や「空きを彫るだけ」の点は使わない。** 終端に壁が無いのだから、
壁の地図と照合しても意味が無い（`Points.hit` が True のものだけを見る）。

## ★ 得点だけで選ぶと、まっすぐな通路で必ず壊れる

道幅 0.9m の周回コースは、直線区間ではどこを切っても同じ形をしている。
**進行方向の位置は点群からは決まらない**（英語で言う corridor problem）。
それでも候補ごとの得点はノイズで 0.001 くらい違うので、素直に最大値を採ると
毎周期でたらめな方向へ数 cm 滑る。しかも**その姿勢で地図を焼く**ので、
ずれた壁が地図に増えて次はもっと滑る。実測では 60 秒で 2.8m ずれた。

なので**推測航法（ジャイロ＋`speed`）からの隔たりに罰則を掛ける**:

    有効得点 = 一致度 − prior_w × ( (Δx² + Δy²)/σxy² + (Δθ/σθ)² )

観測できる方向（壁が近づく向き）では一致度の差が罰則より十分大きいのでマッチが
勝ち、観測できない方向では罰則が勝って**推測航法のまま進む**という自然な解になる。

**罰則の強さは「推測航法がどれだけ信用できるか」で決まる。** ジャイロと `speed`
が入って推測が 100ms で数 mm の精度になったので、以前より強くしてある。
**「見失った」の判定に使う `score` は罰則を引く前の生の一致度**を返す
（罰則を引いた値で判定すると、正しく合っているのに遠くへ来ただけで「見失った」になる）。

## ★★ まだ見ていない場所に落ちた点を「外れ」と数えてはいけない

得点を**全点数**で割ると、地図を作りながら走る間ずっと**後ろへ引っ張られる**。
前へ進むほど前方の点は未探査領域に入り、そこは地図が空なので 0 点になる。
姿勢を少し後ろへずらせば、それらの点が既に地図のある壁に乗って得点が上がる
——から、マッチは毎周期こつこつ後退させる。実測では 70 秒で前方に合計
−0.38m・回頭に −11.9° 引かれ、それがそのまま最終誤差になった。

なので**未探査領域に落ちる点は、照合そのものから外す**。

判定は**初期姿勢で1回だけ**行い、以降どの候補もその同じ点の集合で採点する。
候補ごとに分母を数え直すと今度は逆向きに壊れる: 姿勢を前へずらすほど点が
未探査へ抜けて分母が減り、**残った少数の点だけがよく合う姿勢**が最高得点に
なる（実測では 1.0m 進むべきところを 1.93m 進んだ）。**分母は固定でなければ
比較にならない。**

残った点が少なすぎる（`MIN_KNOWN_RATIO` 未満）ときは、まだ照合できる地図が
無いということなので探索せず、オドメトリのまま進む。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .deskew import Points
from .grid import OccGrid

__all__ = ["MatchResult", "Stage", "DEFAULT_STAGES", "match"]


class Stage(NamedTuple):
    """探索1段ぶん。範囲と刻みは**両端を含む**ように使う。"""

    trans: float                           #: 並進の探索半径 [m]
    trans_step: float                      #: 並進の刻み [m]
    rot: float                             #: 回転の探索半径 [rad]
    rot_step: float                        #: 回転の刻み [rad]


#: 粗 → 中 → 細。**範囲は「1周期で推測航法がどれだけ外れうるか」**で決める。
#:
#: ジャイロ（バイアス補正済み）と `speed` があるので、100ms の予測は実測で
#: 数 mm・0.05° の精度がある（`sim.bench` で推測航法のみ 1.9cm/60秒）。
#: **広く探すほど滑る余地を与えるだけ**なので、±8cm / ±4° まで絞ってある。
#: 以前 ±20cm にしていたときは、道幅 0.42m の車線で毎周期こつこつ滑って
#: 4.7m 走る間に 2.8m ずれた
DEFAULT_STAGES: tuple[Stage, ...] = (
    Stage(0.08, 0.02, math.radians(4.0), math.radians(1.0)),
    Stage(0.02, 0.01, math.radians(1.0), math.radians(0.25)),
    Stage(0.008, 0.004, math.radians(0.25), math.radians(0.05)),
)

#: 事前分布の広がり。**「1周期で推測航法がこれくらい外れるのは不思議ではない」**。
#: 推測が良いので狭く取る。広いと、通路で一致度がほぼ平坦なときに罰則が効かない
PRIOR_XY = 0.05                            #: [m]
PRIOR_YAW = math.radians(2.0)
#: 罰則の重み。一致度（0〜1）と同じ単位。σ ぶん離れて 0.02 の損。
#:
#: **上げすぎると SLAM が「何もしない」ものになる。** 実測（部屋の中で 15cm
#: ずらして 20 周期繰り返す）:
#:
#:     prior_w = 0.05 → 3.004（**1mm も戻らない**）
#:     prior_w = 0.02 → 3.174（正しく収束）  ※真値 3.15
#:
#: 0.05 のとき「シムの誤差が一番小さい」ように見えたのは、**マッチが効いて
#: いなかったから**（＝推測航法そのもの）。ドリフトを直せなければ SLAM の
#: 意味が無いので、罰則は「滑りを抑える」最小限に留める。滑りの抑制は
#: 探索範囲（±8cm）の方に主に担わせている
PRIOR_W = 0.02

#: 照合に使う点の上限。**間引いても得点はほとんど変わらない**（隣り合う点は
#: ほぼ同じセルを引く）ので、上限を切って最悪ケースの計算時間を固定する
MAX_POINTS = 240
#: 既知セルに落ちる点がこの割合を切ったら探索しない。
#: **照合できる地図がまだ無い**ということなので、オドメトリのまま進む
MIN_KNOWN_RATIO = 0.25


class MatchResult(NamedTuple):
    x: float
    y: float
    yaw: float
    score: float                           #: 0〜1。低いのに走っているなら疑う
    #: 探索したか。地図がまだ空なら False（初期値をそのまま返している）
    searched: bool


def _offsets(radius: float, step: float) -> np.ndarray:
    """`-radius … +radius` を `step` 刻みで。**必ず 0 を含む。**

    0 を含まないと、どの候補も現在の推定より悪いときに姿勢が勝手に動く。
    """
    n = int(math.floor(radius / step + 1e-9))
    return np.arange(-n, n + 1, dtype=np.float64) * step


def match(grid: OccGrid, pts: Points, guess: tuple[float, float, float], *,
          stages: tuple[Stage, ...] = DEFAULT_STAGES,
          prior_xy: float = PRIOR_XY, prior_yaw: float = PRIOR_YAW,
          prior_w: float = PRIOR_W) -> MatchResult:
    """`guess` の周りを探して、いちばん**有効得点**の高い姿勢を返す。

    :param pts: 脱スキュー済みの点群（base_link 座標）。**`hit` の点だけを使う**
    :param guess: オドメトリから作った初期姿勢 (x, y, yaw)
    :param prior_w: オドメトリからの隔たりに掛ける罰則（0 で罰則なし）
    """
    gx, gy, gyaw = guess
    sx = pts.x[pts.hit]
    sy = pts.y[pts.hit]
    if sx.size == 0:
        return MatchResult(gx, gy, gyaw, 0.0, False)

    if sx.size > MAX_POINTS:
        take = np.linspace(0, sx.size - 1, MAX_POINTS).astype(np.int64)
        sx, sy = sx[take], sy[take]

    score_map = grid.score_map()
    if not score_map.any():
        # 地図がまだ空。**探索しても全候補 0 点で、原点付近へ吸い寄せられる**
        return MatchResult(gx, gy, gyaw, 0.0, False)

    # ★ 初期姿勢で未探査領域に落ちる点を捨てる（docstring の「まだ見ていない場所」）
    c0, s0 = math.cos(gyaw), math.sin(gyaw)
    col, row = grid.to_cell(gx + sx * c0 - sy * s0, gy + sx * s0 + sy * c0)
    ok = grid.inside(col, row)
    keep = np.zeros(sx.size, dtype=bool)
    keep[ok] = grid.seen[row[ok], col[ok]] > 0
    if keep.sum() < max(1, int(MIN_KNOWN_RATIO * sx.size)):
        return MatchResult(gx, gy, gyaw, 0.0, False)
    sx, sy = sx[keep], sy[keep]

    best = (gx, gy, gyaw)
    best_raw = _score(grid, score_map, sx, sy, gx, gy, gyaw)
    best_eff = best_raw                     # guess 自身は罰則ゼロ

    for st in stages:
        cx = best[0] + _offsets(st.trans, st.trans_step)
        cy = best[1] + _offsets(st.trans, st.trans_step)
        cyaw = best[2] + _offsets(st.rot, st.rot_step)
        # 罰則は候補ごとに決まるので、(X, Y) の外積で一度に作れる
        pen_x = ((cx - gx) / max(prior_xy, 1e-6)) ** 2
        pen_y = ((cy - gy) / max(prior_xy, 1e-6)) ** 2

        for yaw in cyaw:
            pen_yaw = (_wrap(yaw - gyaw) / max(prior_yaw, 1e-9)) ** 2
            penalty = prior_w * (pen_x[:, None] + pen_y[None, :] + pen_yaw)
            c, s = math.cos(yaw), math.sin(yaw)
            rx = sx * c - sy * s
            ry = sx * s + sy * c
            # **x と y を分けて (X,N) と (Y,N) を作り、最後に (X,Y,N) で組む。**
            # 候補を (X*Y, N) に展開してから引くと、同じ列・同じ行の計算を
            # 何度も繰り返すことになる（X=Y=7 なら 7 倍の無駄）
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

    # **返す得点は罰則を引く前**（「見失った」の判定に使うため。docstring 参照）
    return MatchResult(best[0], best[1], _wrap(best[2]), best_raw, True)


def _bilinear(sm: np.ndarray, fx: np.ndarray, fy: np.ndarray) -> np.ndarray:
    """`sm` を**セル中心基準の実数座標**で引き、点ごとの値を候補ごとに合計する。

    `fx` は (X, N)、`fy` は (Y, N)。戻りは (X, Y)。

    ## なぜ最近傍ではなく補間するのか

    最近傍で引くと得点が 5cm 刻みの**階段**になる。同じセルの中では候補が
    何 mm 違っても得点が 1 ビットも動かないので、細かい段の探索（±1cm を 5mm
    刻み）が完全に無意味になり、平坦部のどれが選ばれるかは配列の並び順で決まる。
    実測では**まっすぐ走っているのに毎周期 −0.02° ずつ回頭が引かれ**、
    1周で 12° ずれた。

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
    """1 姿勢ぶんの一致度。**探索と同じ物差し**でないと guess だけ甘くなる。"""
    c, s = math.cos(yaw), math.sin(yaw)
    fx = (x + sx * c - sy * s - grid.origin[0]) / grid.resolution - 0.5
    fy = (y + sx * s + sy * c - grid.origin[1]) / grid.resolution - 0.5
    return float(_bilinear(score_map, fx[None, :], fy[None, :])[0, 0] / sx.size)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
