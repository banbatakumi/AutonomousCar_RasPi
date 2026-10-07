"""レーシングライン — 最小曲率最適化と速度プロファイル。**これがアウトインアウト。**

## なぜ「最小曲率」がアウトインアウトになるのか

中心線の各点 `c_i` を法線方向へ `α_i` だけずらした点を `p_i = c_i + α_i·n_i` とし、

    J(α) = ‖D p_x‖² + ‖D p_y‖² + λ‖α‖²         D は巡回2階差分

を、`α_i` が走路の内側に収まる範囲で最小化する。第1項は経路の曲がりの総量で、
**これを減らそうとすると、コーナーの手前で外へ開き、頂点で内へ付き、
立ち上がりで外へ逃げる**。曲率が一番小さく済むのがその通り方だからで、
アウトインアウトを「そう走れ」と教えているわけではない。**結果として出てくる。**

`λ` は中心線へ引き戻す重み。**これが「最短経路寄りにするか、曲率最小寄りに
するか」のつまみ**そのものになる（上げるほど中心線寄り、下げるほど振れ幅が大きい）。

## 解き方 — 直接解 + アクティブセット

`H` の条件数は 10⁵ オーダーになる（2階差分作用素の低周波モードの固有値が
`(2π/N)⁴` で潰れるため）。**射影ガウス・ザイデルや射影勾配法は使えない。**
あれらは高周波を均すスムーザであって、低周波モードのソルバではない。実測でも
20万反復して誤差が桁で改善しなかった。

N=400 なら稠密 `np.linalg.solve` が 1ms 以下で終わるので、素直に直接解く。
箱制約は**アクティブセット**（範囲を出た点を境界に固定して、残りを解き直す）で、
通常 3〜8 回の反復で収まる。

## 速度は曲率から決めて、前後にならす

    ① v = min(v_max, sqrt(a_lat_eff / |κ|))  横 G の上限
    ② 後ろ向きパス                          その先で減速が間に合う上限
    ③ 前向きパス                            そこまでで加速できる上限

②③ は閉ループなので2周させて端をつなぐ。**②を先にやる**のは、
「止まれない速度で入る」方が「加速が足りない」より危険だから。

### ★ 速度に使う曲率は弦 `SPEED_CHORD_M` で測る（`speed_kappa`）

経路の点は 0.1m 刻みで、3点の外接円で測ると**1点の折れが半径を半分に見せる**。
多角形の壁のコース（toyota2）では最小曲率の解でも κ が 1→4 /m と跳ね、その1点の
ために手前から減速していた。車は前方注視（0.3m〜）より細かい折れをなぞらないので、
速度は弦 0.4m の外接円で測った曲率で決める。**経路の形そのものは変えない。**

### ★ 摩擦円（`friction_circle`）

横Gを使っている所では前後に使える加速度が減る: `a_x·sqrt(1-(v²κ/a_lat)²)`。
これを見込まないと、コーナーの中で加減速する速度を出してしまい、`a_lat` を
グリップの限界より大きく下げておくしかなかった。

### ★ 曲がれる限界に近い所は横Gの余裕を削る（`kappa_max`）

舵が限界近くに張り付くと、経路からのずれを直す余地が無い。半径 0.38m
（限界 0.40m）のヘアピンを横Gだけで決めた 0.7m/s で入ったら、遅延のぶん外へ膨らんで
58cm 外れた（toyota2 の近道、2026-09-29）。以前は曲率が `0.6×κmax` を超えた区間を
一律 `v_min` にしていたが、それだと半径 0.66m 以下のコーナーがすべて最低速度になる
（toyota2 のコーナーはほとんどこれに当たり、0.35m/s まで落ちていた）。
今は `κ/κmax` が `tight_ratio` を超えた所から、横Gを限界で `tight_floor` 倍まで
連続に下げる。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .centerline import (Centerline, measure, normals, normals_open,
                         resample_loop, resample_open, tangents, tangents_open)
from .grid import OccGrid
from .local_map import _edt, footprint_circles

__all__ = ["RaceLine", "optimize", "curvature", "speed_profile", "min_curvature_alpha",
           "chord_curvature", "speed_kappa", "retime"]

#: 速度を決める曲率を測る弦の長さ [m]（モジュール docstring）
SPEED_CHORD_M = 0.4
#: 曲がれる限界に対するこの割合から横Gの余裕を削り始める（モジュール docstring）
TIGHT_RATIO = 0.6
#: 曲がれる限界ちょうどで使う横Gの割合。★ 0.3 は限界を幾何の 40cm と見ていた頃の値。限界を
#: 同定値（46cm）から出し、経路の半径もそこまで開くようにしたので 0.6 に上げた（2026-10-06。
#: sim.bench の toyota・toyota2・実コースの再現で衝突0、ラップは変更前の ±1%。1.0 は実コースの
#: 再現でヘアピン出口の壁に当たった）
TIGHT_FLOOR = 0.6


class RaceLine(NamedTuple):
    xy: np.ndarray                         #: (N, 2) [m]
    v: np.ndarray                          #: (N,) 目標速度 [m/s]
    kappa: np.ndarray                      #: (N,) 曲率 [1/m]。左旋回が正
    length: float                          #: 1周の長さ [m]（開いた経路なら始点→終点の長さ）
    #: False なら開いた経路（今の位置→ゴール、`nav/route.py`）。**添字は巡回しない**
    closed: bool = True

    def __len__(self) -> int:
        return int(self.xy.shape[0])


def _cyclic_d2(n: int) -> np.ndarray:
    """巡回2階差分 `D`。**閉ループの境界条件はこれだけで完結する。**"""
    d = np.zeros((n, n))
    i = np.arange(n)
    d[i, i] = -2.0
    d[i, (i - 1) % n] = 1.0
    d[i, (i + 1) % n] = 1.0
    return d


def _open_d2(n: int) -> np.ndarray:
    """開いた列の2階差分（両端の行は無い）。"""
    d = np.zeros((max(0, n - 2), n))
    i = np.arange(max(0, n - 2))
    d[i, i] = 1.0
    d[i, i + 1] = -2.0
    d[i, i + 2] = 1.0
    return d


def min_curvature_alpha(c: np.ndarray, nrm: np.ndarray,
                        lo: np.ndarray, hi: np.ndarray, *,
                        lam: float = 0.1, step: float = 0.10,
                        max_iter: int = 30, closed: bool = True,
                        weights: np.ndarray | None = None,
                        warm: np.ndarray | None = None) -> np.ndarray:
    """箱制約つき最小曲率問題を解いて `α` を返す。

    :param lo: 各点の下限（右側の余裕。**負の値**）
    :param hi: 各点の上限（左側の余裕）
    :param lam: 中心線へ引き戻す重み。**最短経路とのブレンドつまみ**
    :param step: 点の間隔 [m]。`lam` の正規化に使う（下記）
    :param weights: 各点の曲率の重み（`None` = 一様）。遅い所ほど重くすると最小時間に近づく
        （`optimize` の `time_iters`）
    :param warm: 前回の解。境界に居た点を固定した状態から始める（制約を少し締めた・重みを
        少し変えただけの解き直しで、反復が数回で済む）

    ## `lam` は `step⁴` で正規化する

    曲率の項 `‖D p‖²` は点の間隔 `ds` に対して `(κ·ds²)²` で効くので、
    刻みを変えると同じ `lam` の意味が 4 乗で変わる。**内部で `lam·step⁴` を
    使う**ことで、GUI のスライダを触らずに刻みだけ変えても走りが変わらない。

    ## 一度境界に固定した点は、勾配が内側を向いたら**解放する**

    固定しっぱなしにすると、最初の1回の解が大きく振れたときに全点が境界に
    貼り付いて、そのまま answer になる。**中心線より曲がった経路が「最適解」
    として出てくる**（実測した）。KKT 条件（下限にいる点の勾配は 0 以上、
    上限にいる点は 0 以下）を見て、破っている点を自由集合へ戻す。
    """
    n = len(c)
    d2 = _cyclic_d2(n) if closed else _open_d2(n)
    if weights is None:
        dtd = d2.T @ d2
    else:
        w = np.asarray(weights, dtype=np.float64)
        w = w if closed else w[1:-1]            # 開いた列の行 i は点 i+1 の曲がり
        dtd = d2.T @ (w[:, None] * d2)
    nx, ny = nrm[:, 0], nrm[:, 1]

    m = (nx[:, None] * dtd * nx[None, :]
         + ny[:, None] * dtd * ny[None, :]
         + (lam * step ** 4) * np.eye(n))
    b = nx * (dtd @ c[:, 0]) + ny * (dtd @ c[:, 1])
    tol = 1e-12 * max(1.0, float(np.abs(b).max()))

    # 上下限が一致した点（両側から締められた点、`_BodyCheck.tighten`）は等式として
    # 固定したまま解放しない。解放すると「解放→はみ出す→固定」を反復の上限まで往復した
    eq = (hi - lo) <= 1e-9
    if warm is None:
        alpha = np.zeros(n)
        at_lo = eq.copy()
        at_hi = np.zeros(n, dtype=bool)
    else:
        alpha = np.clip(np.asarray(warm, dtype=np.float64), lo, hi)
        at_lo = eq | (alpha <= lo + 1e-9)
        at_hi = ~at_lo & (alpha >= hi - 1e-9)
    alpha[eq] = lo[eq]
    for _ in range(max_iter):
        fixed = at_lo | at_hi
        free = ~fixed
        if free.any():
            rhs = -(b[free] + m[np.ix_(free, fixed)] @ alpha[fixed])
            sub = m[np.ix_(free, free)]
            try:
                alpha[free] = np.linalg.solve(sub, rhs)
            except np.linalg.LinAlgError:
                # 退化した（同じ点が並んでいる等）。**例外で走行を止めない**
                alpha[free] = np.linalg.lstsq(sub, rhs, rcond=None)[0]

        below = alpha < lo - 1e-12
        above = alpha > hi + 1e-12
        if below.any() or above.any():
            alpha = np.clip(alpha, lo, hi)
            at_lo |= below
            at_hi |= above
            continue

        # 実行可能。**境界に貼り付いている点を解放できるか**を KKT で見る
        g = 2.0 * (m @ alpha + b)
        release = ((at_lo & (g < -tol)) | (at_hi & (g > tol))) & ~eq
        if not release.any():
            break
        at_lo[release] = False
        at_hi[release] = False
    return np.clip(alpha, lo, hi)


def curvature(xy: np.ndarray, closed: bool = True) -> np.ndarray:
    """3点から曲率 [1/m]。**左旋回が正**（反時計回りが正の規約に合わせる）。

    2階差分をそのまま使わず外接円から出すのは、点の間隔が完全には揃わない
    （再サンプル後でも端で数 % ずれる）ため。外接円なら間隔に依存しない。
    """
    a = np.roll(xy, 1, axis=0)
    b = xy
    c = np.roll(xy, -1, axis=0)
    ab = np.hypot(*(b - a).T)
    bc = np.hypot(*(c - b).T)
    ca = np.hypot(*(a - c).T)
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) \
        - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    den = ab * bc * ca
    k = np.where(den > 1e-9, 2.0 * cross / np.maximum(den, 1e-9), 0.0)
    if not closed and len(k) >= 3:
        k[0], k[-1] = k[1], k[-2]          # 端は巡回した反対端とつながっていない
    return k


def chord_curvature(xy: np.ndarray, k: int, closed: bool = True) -> np.ndarray:
    """`k` 点離れた3点の外接円の曲率 [1/m]（**絶対値**）。弦は約 `2k` 点ぶん。"""
    if closed:
        a, c = np.roll(xy, k, axis=0), np.roll(xy, -k, axis=0)
    else:
        idx = np.arange(len(xy))
        a, c = xy[np.clip(idx - k, 0, len(xy) - 1)], xy[np.clip(idx + k, 0, len(xy) - 1)]
    b = xy
    ab = np.hypot(*(b - a).T)
    bc = np.hypot(*(c - b).T)
    ca = np.hypot(*(a - c).T)
    cr = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    den = ab * bc * ca
    return np.where(den > 1e-9, np.abs(2.0 * cr) / np.maximum(den, 1e-9), 0.0)


def speed_kappa(xy: np.ndarray, closed: bool = True,
                chord: float = SPEED_CHORD_M) -> np.ndarray:
    """速度を決めるための曲率 [1/m]（絶対値、弦 `chord` で測る。モジュール docstring）。"""
    xy = np.asarray(xy, dtype=np.float64)
    if len(xy) < 3:
        return np.zeros(len(xy))
    seg = np.hypot(*np.diff(xy, axis=0).T)
    step = float(np.median(seg)) if len(seg) else 0.1
    k = max(1, int(round(chord / 2.0 / max(step, 1e-6))))
    k = min(k, max(1, (len(xy) - 1) // 2))
    kap = chord_curvature(xy, k, closed)
    if not closed:
        kap[:k] = kap[k]
        kap[-k:] = kap[-k - 1]
    return kap


def _lateral_limit(kappa: np.ndarray, *, v_max: float, a_lat: float,
                   kappa_max: float | None) -> np.ndarray:
    """横Gで決まる速度の上限（曲がれる限界の近くは横Gを削る、モジュール docstring）。"""
    k = np.abs(kappa)
    a = np.full(len(k), a_lat)
    if kappa_max:
        r = k / kappa_max
        a = a * np.clip((1.0 - r) / (1.0 - TIGHT_RATIO), TIGHT_FLOOR, 1.0)
    return np.minimum(v_max, np.sqrt(a / np.maximum(k, 1e-6)))


def _next_v(v: float, a_x: float, kappa: float, a_lat: float, ds: float,
            friction_circle: bool) -> float:
    """速度 `v` の点から距離 `ds` だけ加速（減速）したときに届く速度。"""
    if friction_circle:
        ay = v * v * kappa
        a_x = a_x * math.sqrt(max(0.0, 1.0 - (ay / a_lat) ** 2))
    return math.sqrt(v * v + 2.0 * a_x * ds)


def speed_profile(xy: np.ndarray, kappa: np.ndarray, *,
                  v_max: float, v_min: float, a_lat: float,
                  a_accel: float, a_brake: float, closed: bool = True,
                  v_start: float | None = None, v_end: float | None = None,
                  kappa_max: float | None = None,
                  friction_circle: bool = True) -> np.ndarray:
    """曲率から目標速度 [m/s]。閉ループとして前後パスを2周ならす。

    :param kappa: **速度を決める曲率**（ふつうは `speed_kappa(xy)`）
    :param kappa_max: 車の曲がれる限界の曲率 [1/m]。与えると限界の近くで横Gを削る

    `closed=False`（開いた経路）は1往復だけで、**終点は `v_end`**（既定 `v_min`、
    止まる直前の這う速度）まで減速し、始点は `v_start`（今の速度）から加速する。
    """
    n = len(xy)
    k = np.abs(np.asarray(kappa, dtype=np.float64))
    v = np.maximum(_lateral_limit(k, v_max=v_max, a_lat=a_lat, kappa_max=kappa_max), v_min)
    if n == 0:
        return v
    fc = friction_circle
    if not closed:
        ds = np.hypot(*np.diff(xy, axis=0).T) if n >= 2 else np.zeros(0)
        v[-1] = min(v[-1], v_min if v_end is None else v_end)
        for i in range(n - 2, -1, -1):
            v[i] = min(v[i], _next_v(v[i + 1], a_brake, k[i + 1], a_lat, ds[i], fc))
        if v_start is not None:
            # 今の速度より速い値から始めても、実際にはそこから加速するしかない
            v[0] = min(v[0], max(v_start, v_min))
            for i in range(1, n):
                v[i] = min(v[i], _next_v(v[i - 1], a_accel, k[i - 1], a_lat, ds[i - 1], fc))
        return v

    ds = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)      # i → i+1 の距離
    for _ in range(2):
        # ② 後ろ向き（i の速度は i+1 で止まれる範囲に抑える）
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], _next_v(v[j], a_brake, k[j], a_lat, ds[i], fc))
        # ③ 前向き（i の速度は i-1 から加速できる範囲に抑える）
        for i in range(n):
            j = (i - 1) % n
            v[i] = min(v[i], _next_v(v[j], a_accel, k[j], a_lat, ds[j], fc))
    return np.maximum(v, v_min)


def _travel_time(xy: np.ndarray, v: np.ndarray, closed: bool) -> float:
    """速度プロファイルどおりに走ったときの時間 [s]（閉路なら1周）。"""
    v = np.maximum(v, 1e-3)
    if closed:
        ds = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
        return float(np.sum(2.0 * ds / (v + np.roll(v, -1))))
    ds = np.hypot(*np.diff(xy, axis=0).T)
    return float(np.sum(2.0 * ds / (v[:-1] + v[1:])))


def retime(rl: RaceLine, *, v_max: float, v_min: float, a_lat: float, a_accel: float,
           a_brake: float, kappa_max: float | None = None,
           v_start: float | None = None, v_end: float | None = None) -> RaceLine:
    """経路の形はそのままで、速度プロファイルだけを今の設定で作り直す。

    ★ 速度は地図を作ったとき（BUILD）の設定で焼き込まれ、保存した地図から走ると
    そのまま使われていた。**GUI で最高速度や加速度を変えても効かなかった。**
    planner は設定が変わるたびにこれを呼ぶ。
    """
    v = speed_profile(rl.xy, speed_kappa(rl.xy, rl.closed), v_max=v_max, v_min=v_min,
                      a_lat=a_lat, a_accel=a_accel, a_brake=a_brake, closed=rl.closed,
                      v_start=v_start, v_end=v_end, kappa_max=kappa_max)
    # `_replace` は使えない（`RaceLine.__len__` が点の数を返すので、NamedTuple の
    # 要素数の検査に引っかかる）
    return RaceLine(**{**rl._asdict(), "v": v})


def body_allowance(kappa: np.ndarray, front: float, rear: float) -> np.ndarray:
    """曲がっているときに**車体の角が経路より外へ張り出す量** [m]。

    経路が守っているのは「base_link（後輪車軸中心）が壁から何m離れているか」
    だけで、車体は長い（前オーバーハング30cm）。半径Rで曲がると前の外側の角は
    `sqrt((R+w)² + f²) − (R+w) ≈ f²/(2R)` だけ外へ、後ろの内側の角は
    `r²/(2R)` だけ内へ張り出す。半径0.6mのコーナーなら前だけで7.5cmで、
    車体半幅+安全余裕（既定14cm）の半分を食う。

    ★ これを見込まないと、**地図も自己位置も正しいのに車が壁を擦る**
    （実測: `normal`コースの`slam2d_raceline`で RACE 中に111回の衝突。
    安全余裕を5cm→12cmに上げると0回になった）。
    """
    k = np.abs(np.asarray(kappa, dtype=np.float64))
    return 0.5 * (front ** 2 + rear ** 2) * k


#: 車体外形の検査で制約を締め直す回数の上限（`_BodyCheck`）
BODY_ITERS = 8
#: 車が曲がり切れない所（`kappa_max` 超え）の曲率の重みを上げて解き直す回数の上限（`optimize`）
KAPPA_ITERS = 8


class _BodyCheck:
    """地図の壁と**車体外形**の距離を経路の各点で測る（`optimize` の `footprint`）。

    ★ 道幅は中心線の法線方向へ撃ったレイで測るので、**薄い仕切り壁の先端**が
    見えない（レイは先端の横を素通りする）。そこを後輪軸の余裕だけで通ると、
    30cm 前に張り出した車体の前端が先端をかすめる。toyota2 の上の仕切りの先端で
    2m/s・横偏差数cmで毎回ぶつかった（sim.bench、2026-09-29）。車体を円で覆い
    （`local_map.footprint_circles`）、壁の距離場で直接確かめる。
    """

    def __init__(self, grid, footprint, closed: bool) -> None:
        wall = getattr(grid, "base_wall_mask", grid.wall_mask)()
        self.res = float(grid.resolution)
        self.origin = (float(grid.origin[0]), float(grid.origin[1]))
        self.edt = _edt(~wall, self.res)
        self.circles, self.r = footprint_circles(footprint)
        self.closed = closed

    def clearance(self, xy: np.ndarray, offset: np.ndarray | None = None) -> np.ndarray:
        """各点に車体を置いたときの壁までの余裕 [m]（負 = 食い込み）。

        向きは `xy` の接線。`offset` を与えると位置だけずらして測る（向きは同じ）。
        """
        t = tangents(xy) if self.closed else tangents_open(xy)
        p = xy if offset is None else xy + offset
        c, s = t[:, 0:1], t[:, 1:2]
        ox, oy = self.circles[None, :, 0], self.circles[None, :, 1]
        px = p[:, 0:1] + ox * c - oy * s
        py = p[:, 1:2] + ox * s + oy * c
        col = np.floor((px - self.origin[0]) / self.res).astype(np.int64)
        row = np.floor((py - self.origin[1]) / self.res).astype(np.int64)
        h, w = self.edt.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        v = self.edt[np.clip(row, 0, h - 1), np.clip(col, 0, w - 1)]
        return np.where(inside, v, 0.0).min(axis=1) - self.r

    def tighten(self, c: np.ndarray, nrm: np.ndarray, a: np.ndarray,
                lo: np.ndarray, hi: np.ndarray, margin: float) -> bool:
        """余裕が `margin` に足りない点の箱制約を、壁から離れる側へ締める。締めたら True。

        ★ 開いた経路の両端（今の車の位置・止まる場所）は動かさない。締めると `lo > hi` に
        なって「真ん中で諦める」に落ち、壁際から走り出す・壁際に止まる経路で、始点が車の
        位置から（終点が停止点から）数cm ずれた。
        """
        xy = c + nrm * a[:, None]
        clr = self.clearance(xy)
        short = clr < margin - 1e-3
        if not self.closed:
            short[[0, -1]] = False
        bad = np.nonzero(short)[0]
        if not len(bad):
            return False
        d = 0.02
        plus = self.clearance(xy, nrm * d)
        minus = self.clearance(xy, -nrm * d)
        need = margin - clr
        for i in bad:
            if plus[i] >= minus[i]:            # 左（+法線）へ逃げると広がる
                lo[i] = max(lo[i], a[i] + need[i])
            else:
                hi[i] = min(hi[i], a[i] - need[i])
            if lo[i] > hi[i]:                  # 両側が詰まっている。真ん中で諦める
                lo[i] = hi[i] = 0.5 * (lo[i] + hi[i])
        return True


def optimize(grid: OccGrid, cl: Centerline, *, half_width: float, margin: float,
             lam: float = 0.1, v_max: float = 1.0, v_min: float = 0.2,
             a_lat: float = 2.0, a_accel: float = 1.0, a_brake: float = 1.5,
             passes: int = 2, max_width: float = 3.0,
             front_overhang: float = 0.0, rear_overhang: float = 0.0,
             v_start: float | None = None, v_end: float | None = None,
             kappa_max: float | None = None, footprint=None,
             time_iters: int = 0) -> RaceLine:
    """中心線からレーシングラインを作る。

    :param half_width: 車体半幅 [m]。`config/vehicle.toml` の外形から取る
    :param margin: それに足す安全余裕 [m]
    :param front_overhang: base_link から車体前端まで [m]。コーナーで車体の角が
        経路の外へ張り出すぶんを余裕に足す（`body_allowance`）。0 なら見込まない
    :param rear_overhang: 同 後端まで [m]
    :param passes: 「最適化 → 法線と幅を測り直す」の反復回数
    :param kappa_max: 車の曲がれる限界の曲率 [1/m]。限界の近くで横Gを削る（`speed_profile`）
    :param footprint: 車体外形（`config/vehicle.toml` の `footprint`）。与えると、車体と
        地図の壁の距離が `margin` を切る点の制約を締めて解き直す（`_BodyCheck`）
    :param time_iters: 最小時間へ寄せる再重み付けの回数（0 = 最小曲率のまま、下記）

    `passes > 1` にしているのは、最適化で経路が動くと**法線の向きも変わる**ため。
    斜めに当たる法線は道幅を過大に測り、**壁にめり込む解が実行可能に見える**。
    2周目以降は出てきた経路を中心線とみなして**地図を撃ち直す**（幅を補間で
    付け替えると、傾いた法線の誤りがそのまま残る）。

    ## ★ 悪くなったパスは返さない

    この反復は収束を保証しない。経路が壁に寄る → そこで測った余裕が小さくなる →
    次の解がさらに寄る、という往復に入ることがあり、実測で**中心線より曲がった
    経路**が出た（`passes=3`, `lam=0.03`）。曲率エネルギー `Σκ²` が最小だった
    パスを採る。最悪でも中心線そのものより悪くはならない。

    ## 最小時間へ寄せる（`time_iters`）

    最小曲率は「曲がりの総量」を一様に減らすが、ラップタイムに効くのは**遅い所**の
    曲がり。速度プロファイルを出し、曲率の重みを `v_max / v` にして解き直す（反復
    再重み付け）。重みは前回と半分ずつ混ぜる（混ぜないと2つの解を往復した）。
    **どの反復の解も見積もりのラップタイムで比べ、最短のものを採る**ので、最小曲率の
    解より遅くはならない。toyota2 では、曲がれる限界に近いヘアピンの半径が開いて
    最低速度が 0.66→1.5m/s になり、見積もりが 13.68→12.9s（−5%）になった。

    ## ★ 車が曲がり切れる半径に開く（`kappa_max`、`KAPPA_ITERS`）

    速度は `v_min` で下支えされるので、見積もりの時間には「曲がり切れない」ことの損が
    出ない。壁の先端を回るヘアピンで半径 37cm（車の限界 46cm）の線が最短として選ばれ、
    実車は舵が張り付いたまま出口で 14〜20cm 膨らんで壁まで 0〜2cm だった（2026-10-06）。
    最小時間の反復の後、`kappa_max` を超えた所の重みを上げて解き直し、超えた量の
    小さい解を優先して採る。道幅が足りず開けない所は、いちばん緩い解になる。
    """
    closed = cl.closed
    keep = half_width + margin + body_allowance(curvature(cl.xy, closed), front_overhang,
                                                rear_overhang)
    c = cl.xy
    nrm = cl.normal
    left, right = cl.w_left, cl.w_right

    spd = dict(v_max=v_max, v_min=v_min, a_lat=a_lat, a_accel=a_accel, a_brake=a_brake,
               closed=closed, v_start=v_start, v_end=v_end, kappa_max=kappa_max)

    body = _BodyCheck(grid, footprint, closed) if footprint is not None else None

    def score(xy: np.ndarray) -> tuple[int, float, float]:
        """小さいほど良い。★ 車体外形の余裕が足りない解は、足りる解より必ず後ろ。
        そのうえで `time_iters` なら見積もりの走行時間、そうでなければ Σκ²。"""
        short = int(body is not None and float(body.clearance(xy).min()) < margin - 0.01)
        ks = speed_kappa(xy, closed)
        # ★ 車が曲がり切れない解は、曲がり切れる解より必ず後ろ（超えた量の小さい順）。
        # 速度は `v_min` で下支えされるので、時間だけで比べると超えた分が見えない
        over = max(0.0, float(ks.max()) - kappa_max) if kappa_max and len(ks) else 0.0
        if time_iters > 0:
            t = _travel_time(xy, speed_profile(xy, ks, **spd), closed)
            return short, over, t
        return short, over, float((curvature(xy, closed) ** 2).sum())

    best = cl.xy                                   # 何も改善しなければ中心線のまま
    best_energy = score(cl.xy)

    for k in range(max(1, passes)):
        hi = np.maximum(left - keep, 0.0)
        lo = -np.maximum(right - keep, 0.0)
        if not closed:
            # 始点（今の車の位置）と終点（止まる場所）は動かさない
            lo[[0, -1]] = 0.0
            hi[[0, -1]] = 0.0
        w = None
        a = None
        for it in range(1 + max(0, time_iters) + (KAPPA_ITERS if kappa_max else 0)):
            a = min_curvature_alpha(c, nrm, lo, hi, lam=lam, step=cl.step, closed=closed,
                                    weights=w, warm=a)
            if body is not None:
                for _ in range(BODY_ITERS):
                    if not body.tighten(c, nrm, a, lo, hi, margin):
                        break
                    a = min_curvature_alpha(c, nrm, lo, hi, lam=lam, step=cl.step,
                                            closed=closed, weights=w, warm=a)
            xy = c + nrm * a[:, None]
            energy = score(xy)
            if energy < best_energy:
                best, best_energy = xy, energy
            if it < time_iters:
                v = speed_profile(xy, speed_kappa(xy, closed), **spd)
                w_new = float(v.max()) / np.maximum(v, 0.1)
                w_new /= w_new.mean()
                w = w_new if w is None else 0.5 * (w + w_new)
                continue
            # 曲がり切れない所が残っていれば、そこの曲率の重みを超えた割合の2乗で上げて
            # 解き直す（道幅が許すぶんだけ外へ膨らんで半径が開く）。無ければ終わり
            ks = speed_kappa(xy, closed)
            if not kappa_max or float(ks.max()) <= kappa_max:
                break
            w = (np.ones(len(c)) if w is None else w) * np.maximum(1.0, ks / kappa_max) ** 2
            w = w / w.mean()
        if k + 1 >= max(1, passes):
            break
        c = resample_loop(xy, cl.step) if closed else resample_open(xy, cl.step)
        nrm = normals(c) if closed else normals_open(c)
        left, right = measure(grid, c, nrm, max_width)
        keep = half_width + margin + body_allowance(curvature(c, closed), front_overhang,
                                                    rear_overhang)

    xy = best
    kap = curvature(xy, closed)
    v = speed_profile(xy, speed_kappa(xy, closed), v_max=v_max, v_min=v_min, a_lat=a_lat,
                      a_accel=a_accel, a_brake=a_brake, closed=closed,
                      v_start=v_start, v_end=v_end, kappa_max=kappa_max)
    seg = np.hypot(*((np.roll(xy, -1, axis=0) - xy) if closed else np.diff(xy, axis=0)).T)
    return RaceLine(xy=xy, v=v, kappa=kap, length=float(seg.sum()), closed=closed)
