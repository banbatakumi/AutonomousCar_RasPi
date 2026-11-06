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

### 曲がれる限界の近くでも横Gは削らない

曲率が車の限界に近い所の横Gを経験則で削る方式は使わない。次の3つで足りるため
（sim.bench でグリップを −10% にした車でも衝突 0）:
①経路そのものを「その速さで出せる曲率 − 追従の余裕」までしか曲げない（`optimize` の `kappa_max`、`slam2d_raceline.kappa_limit`）
②追従の姿勢を「今」へ進めて 50Hz で舵を更新する
③速度指令を前倒ししてコーナーへ速すぎる速度で入らない。
削ると舵が上限に張り付いたとき経路からのずれを直せず、遅延のぶん外へ膨らむ（toyota2 の近道）。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .centerline import (Centerline, measure, normals, normals_open,
                         resample_loop, resample_open, tangents, tangents_open)
from .grid import OccGrid, dilate, unknown_wall_mask
from .local_map import _edt, footprint_circles
from .native import addr as _addr, f64 as _f64, lib as _NATIVE_LIB

#: C の内側ループ（`nav/native.py`）。`None` なら Python 版で回す（テストが差し替えて両方を比べる）
_NATIVE = _NATIVE_LIB

__all__ = ["RaceLine", "optimize", "curvature", "speed_profile", "min_curvature_alpha",
           "sight_distance", "sight_speed",
           "chord_curvature", "speed_kappa", "retime", "refine_time", "tight_sections"]

#: 速度を決める曲率を測る弦の長さ [m]（モジュール docstring）
SPEED_CHORD_M = 0.4
#: 前後の上限の包絡（`speed_profile` の `env`）を持つ横の使い具合 u = 横加速度 ÷ 横の限界 の点。間は直線でつなぐ
ENV_U = (0.0, 0.5, 1.0)


def env_at(vals, u: float) -> float:
    """包絡 `vals`（`ENV_U` の各点の前後の上限 [m/s²]）の、横の使い具合 `u` での値（0〜1 の外は端の値）。"""
    if u <= 0.0:
        return vals[0]
    if u >= 1.0:
        return vals[-1]
    if u < 0.5:
        return vals[0] + (vals[1] - vals[0]) * (u / 0.5)
    return vals[1] + (vals[2] - vals[1]) * ((u - 0.5) / 0.5)

class RaceLine(NamedTuple):
    xy: np.ndarray                         #: (N, 2) [m]
    v: np.ndarray                          #: (N,) 目標速度 [m/s]
    kappa: np.ndarray                      #: (N,) 曲率 [1/m]。左旋回が正
    length: float                          #: 1周の長さ [m]（開いた経路なら始点→終点の長さ）
    #: False なら開いた経路（今の位置→ゴール、`nav/route.py`）。**添字は巡回しない**
    closed: bool = True
    #: True なら、道が狭くて車体外形の壁との余裕（`margin`）を確保できない所が残っている
    #: （`optimize` が余裕を満たす解を見つけられなかった）。線は返すが、呼び出し側は
    #: 利用者に知らせるか速度を抑える
    short: bool = False
    #: True なら、**未知のセルも壁とみなすと**車体外形の余裕（`margin`）が足りない所がある。
    #: 道幅のレイと車体の検査は未知を通れる所として扱う（壁の切れ目の先が未観測だと余裕を過大に
    #: 測りうる）が、道路グラフは未知を壁として扱う。線は変えず、知らせるためだけの印
    unknown_short: bool = False

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

    ## 帯行列で解く（`_min_curvature_alpha_band`）

    行列は巡回5重対角（2階差分の `DᵀWD`）なので、C があれば帯のまま O(n) で解く。密行列だと
    行列積と `solve` が n³ で、Pi（参照 BLAS）では magp2（510点）の1本に 10 秒以上かかった。
    """
    if _NATIVE is not None and len(c) >= _BAND_MIN_N:
        return _min_curvature_alpha_band(c, nrm, lo, hi, lam=lam, step=step, max_iter=max_iter,
                                         closed=closed, weights=weights, warm=warm)
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


#: これより点が少なければ帯行列にせず密行列で解く（巡回の角が帯と重なる）
_BAND_MIN_N = 16
#: 自由な点がこれより少ない部分問題は密行列で解く（`band_solve` は 6 点以上）
_BAND_MIN_FREE = 8


def _band_matvec(t0: np.ndarray, t1: np.ndarray, t2: np.ndarray, x: np.ndarray) -> np.ndarray:
    """対称な巡回5重対角行列（対角 `t0`、`t1[i]` = (i, i+1)、`t2[i]` = (i, i+2)）と `x` の積。"""
    return (t0 * x + t1 * np.roll(x, -1) + np.roll(t1 * x, 1)
            + t2 * np.roll(x, -2) + np.roll(t2 * x, 2))


def _min_curvature_alpha_band(c: np.ndarray, nrm: np.ndarray, lo: np.ndarray, hi: np.ndarray, *,
                              lam: float, step: float, max_iter: int, closed: bool,
                              weights: np.ndarray | None, warm: np.ndarray | None) -> np.ndarray:
    """`min_curvature_alpha` の帯行列版（手順は密行列版と同じ、行列の持ち方と解き方だけが違う）。

    `DᵀWD` は、点 p を中心とする2階差分（重み `wc[p]`）の和なので、帯は
    対角 `wc[i-1] + 4wc[i] + wc[i+1]`・1つ隣 `−2(wc[i] + wc[i+1])`・2つ隣 `wc[i+1]`。
    開いた列は両端の点を中心とする差分が無い（`wc` の両端が 0）ので、同じ式で巡回の角が 0 になる。

    自由な点だけの部分行列は、自由な点を順に並べても帯幅2のままで、巡回の結合は最初の2点と
    最後の2点の間にしか出ない（固定した点を抜くと間が詰まるだけ）。これを C の `band_solve`
    （末尾2行を縁にした矢形の Cholesky）で解く。
    """
    n = len(c)
    if weights is None:
        wc = np.ones(n)
    else:
        wc = np.asarray(weights, dtype=np.float64).copy()
    if not closed:
        wc[0] = wc[-1] = 0.0                    # 開いた列の差分は点 1〜n−2 を中心とする
    wn = np.roll(wc, -1)
    t0 = np.roll(wc, 1) + 4.0 * wc + wn
    t1 = -2.0 * (wc + wn)
    t2 = wn.copy()
    nx, ny = nrm[:, 0], nrm[:, 1]
    m0 = (nx * nx + ny * ny) * t0 + lam * step ** 4
    m1 = (nx * np.roll(nx, -1) + ny * np.roll(ny, -1)) * t1
    m2 = (nx * np.roll(nx, -2) + ny * np.roll(ny, -2)) * t2
    b = nx * _band_matvec(t0, t1, t2, c[:, 0]) + ny * _band_matvec(t0, t1, t2, c[:, 1])
    tol = 1e-12 * max(1.0, float(np.abs(b).max()))

    def get(i: int, j: int) -> float:
        """行列の (i, j) 要素（i, j は点の番号）。"""
        d = (j - i) % n
        if d == 0:
            return float(m0[i])
        if d == 1:
            return float(m1[i])
        if d == 2:
            return float(m2[i])
        if d == n - 1:
            return float(m1[j])
        if d == n - 2:
            return float(m2[j])
        return 0.0

    def solve(free_idx: np.ndarray, rhs: np.ndarray) -> np.ndarray:
        nf = len(free_idx)
        if nf >= _BAND_MIN_FREE:
            g1 = np.diff(free_idx)
            d1 = np.where(g1 == 1, m1[free_idx[:-1]], np.where(g1 == 2, m2[free_idx[:-1]], 0.0))
            d2 = np.where(free_idx[2:] - free_idx[:-2] == 2, m2[free_idx[:-2]], 0.0)
            f0, f1, fl2, fl1 = (int(free_idx[0]), int(free_idx[1]), int(free_idx[-2]),
                                int(free_idx[-1]))
            gl = f0 + n - fl1
            c10 = float(m1[fl1]) if gl == 1 else float(m2[fl1]) if gl == 2 else 0.0
            c20 = float(m2[fl2]) if f0 + n - fl2 == 2 else 0.0
            c11 = float(m2[fl1]) if f1 + n - fl1 == 2 else 0.0
            d0 = _f64(m0[free_idx])
            d1, d2, r = _f64(d1), _f64(d2), _f64(rhs)
            x = np.empty(nf)
            if _NATIVE.band_solve(nf, _addr(d0), _addr(d1), _addr(d2), c20, c10, c11,
                                  _addr(r), _addr(x)) == 0:
                return x
        # 自由な点が少ない・正定値として解けなかった（数値的に退化）。部分行列を作って密に解く
        sub = np.array([[get(int(i), int(j)) for j in free_idx] for i in free_idx])
        try:
            return np.linalg.solve(sub, rhs)
        except np.linalg.LinAlgError:
            # 退化した（同じ点が並んでいる等）。**例外で走行を止めない**
            return np.linalg.lstsq(sub, rhs, rcond=None)[0]

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
            # 固定した点からの寄与 `m[free, fixed] @ alpha[fixed]`
            fixed_part = _band_matvec(m0, m1, m2, np.where(fixed, alpha, 0.0))
            idx = np.nonzero(free)[0]
            alpha[free] = solve(idx, -(b[free] + fixed_part[free]))

        below = alpha < lo - 1e-12
        above = alpha > hi + 1e-12
        if below.any() or above.any():
            alpha = np.clip(alpha, lo, hi)
            at_lo |= below
            at_hi |= above
            continue

        g = 2.0 * (_band_matvec(m0, m1, m2, alpha) + b)
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
    if _NATIVE is not None and len(xy) >= 3:
        xy = _f64(xy)
        out = np.empty(len(xy))
        _NATIVE.curvature(len(xy), _addr(xy), int(closed), _addr(out))
        return out
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
    if _NATIVE is not None and len(xy) >= 3:
        xy = _f64(xy)
        out = np.empty(len(xy))
        _NATIVE.chord_curvature(len(xy), _addr(xy), int(k), int(closed), _addr(out))
        return out
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
    if _NATIVE is not None:
        xy = _f64(xy)
        out = np.empty(len(xy))
        _NATIVE.speed_kappa(len(xy), _addr(xy), int(closed), float(chord), _addr(out))
        return out
    seg = np.hypot(*np.diff(xy, axis=0).T)
    step = float(np.median(seg)) if len(seg) else 0.1
    k = max(1, int(round(chord / 2.0 / max(step, 1e-6))))
    k = min(k, max(1, (len(xy) - 1) // 2))
    kap = chord_curvature(xy, k, closed)
    if not closed:
        kap[:k] = kap[k]
        kap[-k:] = kap[-k - 1]
    return kap


def _lateral_limit(kappa: np.ndarray, *, v_max: float, a_lat: float) -> np.ndarray:
    """横Gで決まる速度の上限 `min(v_max, sqrt(a_lat/|κ|))`。"""
    return np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-6)))


def _next_v(v: float, a_x: float, kappa: float, a_lat: float, ds: float,
            friction_circle: bool) -> float:
    """速度 `v` の点から距離 `ds` だけ加速（減速）したときに届く速度。"""
    if friction_circle:
        ay = v * v * kappa
        a_x = a_x * math.sqrt(max(0.0, 1.0 - (ay / a_lat) ** 2))
    return math.sqrt(v * v + 2.0 * a_x * ds)


def speed_profile(xy: np.ndarray, kappa: np.ndarray, *,
                  v_max: float, v_min: float, a_lat,
                  a_accel: float, a_brake: float, closed: bool = True,
                  v_start: float | None = None, v_end: float | None = None,
                  friction_circle: bool = True, a_grip=None,
                  v_cap: np.ndarray | None = None,
                  env: tuple | None = None) -> np.ndarray:
    """曲率から目標速度 [m/s]。閉ループとして前後パスを2周ならす。

    :param kappa: **速度を決める曲率**（ふつうは `speed_kappa(xy)`）
    :param v_cap: 点ごとの速度の上限 [m/s]（見通しで決まる上限など、`sight_speed`）。前後のパスの
        前に掛けるので、そこへ向けた減速・そこからの加速もプロファイルに入る
    :param a_lat: 横加速度の上限 [m/s²]。スカラーか、点ごとの配列（`nav/grip.py` の学習値）
    :param a_grip: 摩擦円の半径 [m/s²]（車の横グリップの限界）。`None` なら `a_lat`。配列も可。
        `a_lat` を限界より下げて走る設定（「横グリップを使う割合」）でも、前後に残る力は
        限界に対する割合で決まる——`a_lat` で円を引くと頂点で前後が 0 になり、
        コーナーの中で加減速できない前提のプロファイルになる

    :param env: `(加速の包絡, 減速の包絡)`。それぞれ `ENV_U` の各点（横の使い具合 u = 横加速度 ÷ 横の限界
        `a_grip`）で出せる前後の加速度 [m/s²]（走行中に学習した実測の g-g 図、`nav/grip.py`）。与えると、
        摩擦円と `a_accel`・`a_brake` の代わりに、その点の u での包絡の値で加減速する

    `closed=False`（開いた経路）は1往復だけで、**終点は `v_end`**（既定 `v_min`、
    止まる直前の這う速度）まで減速し、始点は `v_start`（今の速度）から加速する。
    """
    if np.ndim(a_lat) or (a_grip is not None and np.ndim(a_grip)) or env is not None:
        # 点ごとの上限（走行中に学習した横グリップ、`nav/grip.py`）。スカラーの経路は下のまま
        # （`refine_time` が数千回呼ぶので、そちらに分岐や配列の読み出しを足さない）
        return _speed_profile_var(xy, kappa, v_max=v_max, v_min=v_min, a_lat=a_lat,
                                  a_accel=a_accel, a_brake=a_brake, closed=closed,
                                  v_start=v_start, v_end=v_end, friction_circle=friction_circle,
                                  a_grip=a_grip, v_cap=v_cap, env=env)
    n = len(xy)
    k = np.abs(np.asarray(kappa, dtype=np.float64))
    v = _lateral_limit(k, v_max=v_max, a_lat=a_lat)
    if n == 0:
        return v
    if v_cap is not None:
        v = np.minimum(v, np.maximum(np.asarray(v_cap, dtype=np.float64), 0.05))
    # `v_min` は直線の下限。横の限界（見通しの上限も）で決まる速度が `v_min` より低い所は、
    # その速度を採る（`v_min` で持ち上げると曲がり切れない）
    floor = np.minimum(v, v_min)
    if v_cap is not None:
        v_min = min(v_min, float(v.min()))
    # ここから下の `a_lat` は摩擦円の半径としてだけ使う
    inv_grip2 = _inv_grip2(a_lat, a_grip, friction_circle)
    if _NATIVE is not None:
        # 下の Python のループと同じ演算（`nav/_native.c` の `sp_*`、ビット一致）
        vc = _f64(v).copy()
        kc = _f64(k)
        xyc = _f64(xy)
        dsc = np.empty(n)
        _NATIVE.seg_len(n, _addr(xyc), int(closed), _addr(dsc))
        if not closed:
            vc[-1] = min(vc[-1], v_min if v_end is None else v_end)
            _NATIVE.sp_open_back(n, _addr(vc), _addr(kc), _addr(dsc), float(a_brake), inv_grip2)
            if v_start is not None:
                vc[0] = min(vc[0], max(v_start, v_min))
                _NATIVE.sp_open_fwd(n, _addr(vc), _addr(kc), _addr(dsc), float(a_accel), inv_grip2)
            return vc
        _NATIVE.sp_closed(n, _addr(vc), _addr(kc), _addr(dsc), float(a_brake), float(a_accel),
                          inv_grip2)
        return np.maximum(vc, floor)
    sqrt = math.sqrt
    # ★ numpy の配列を1点ずつ読むと遅い（要素の取り出しが 1 回 100ns 前後）。この関数は
    #   ラインの仕上げ（`refine_time`）が数千回呼ぶので、リストに直して回す（約5倍）
    vl = v.tolist()
    kl = k.tolist()

    def reach(vj: float, kj: float, a_x: float, d: float, inv: float = inv_grip2) -> float:
        """速度 `vj`（曲率 `kj`）の点から距離 `d` だけ加速（減速）したときに届く速度。"""
        if inv:
            ay = vj * vj * kj
            r = 1.0 - ay * ay * inv
            a_x = a_x * sqrt(r) if r > 0.0 else 0.0
        return sqrt(vj * vj + 2.0 * a_x * d)

    if not closed:
        ds = np.hypot(*np.diff(xy, axis=0).T).tolist() if n >= 2 else []
        vl[-1] = min(vl[-1], v_min if v_end is None else v_end)
        for i in range(n - 2, -1, -1):
            lim = reach(vl[i + 1], kl[i + 1], a_brake, ds[i])
            if lim < vl[i]:
                vl[i] = lim
        if v_start is not None:
            # 今の速度より速い値から始めても、実際にはそこから加速するしかない
            vl[0] = min(vl[0], max(v_start, v_min))
            for i in range(1, n):
                lim = reach(vl[i - 1], kl[i - 1], a_accel, ds[i - 1])
                if lim < vl[i]:
                    vl[i] = lim
        return np.asarray(vl)

    ds = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T).tolist()      # i → i+1 の距離
    for _ in range(2):
        # ② 後ろ向き（i の速度は i+1 で止まれる範囲に抑える）
        for i in range(n - 1, -1, -1):
            j = i + 1 if i + 1 < n else 0
            lim = reach(vl[j], kl[j], a_brake, ds[i])
            if lim < vl[i]:
                vl[i] = lim
        # ③ 前向き（i の速度は i-1 から加速できる範囲に抑える）
        for i in range(n):
            j = i - 1 if i > 0 else n - 1
            lim = reach(vl[j], kl[j], a_accel, ds[j])
            if lim < vl[i]:
                vl[i] = lim
    return np.maximum(np.asarray(vl), floor)


def _inv_grip2(a_lat: float, a_grip, friction_circle: bool = True) -> float:
    """摩擦円の半径の2乗の逆数（`speed_profile` のスカラー版。0 なら摩擦円を使わない）。"""
    grip = max(a_lat, a_grip) if a_grip else a_lat
    return 1.0 / (grip * grip) if friction_circle and grip > 1e-9 else 0.0


#: C で仕上げの評価をまとめて回す（`optimize` の `cost`）点の数の上限。これより多いと numpy の総和の
#: 手順が変わる（`_native.c` の `pairwise_sum`）
_FUSED_MAX_N = 8192


def _speed_profile_var(xy: np.ndarray, kappa: np.ndarray, *, v_max: float, v_min: float,
                       a_lat, a_accel: float, a_brake: float, closed: bool,
                       v_start: float | None, v_end: float | None, friction_circle: bool,
                       a_grip, v_cap: np.ndarray | None, env: tuple | None = None) -> np.ndarray:
    """`speed_profile` の、横の上限 `a_lat`・摩擦円の半径 `a_grip` が**点ごとの配列**の版と、前後の上限の
    包絡 `env` の版。式は `speed_profile` と同じで、点 `j` から出る加減速にその点の半径（包絡なら、その点の
    横の使い具合での包絡の値）を使う。
    """
    n = len(xy)
    k = np.abs(np.asarray(kappa, dtype=np.float64))
    lat = np.maximum(np.broadcast_to(np.asarray(a_lat, dtype=np.float64), (n,)), 1e-6)
    v = np.minimum(v_max, np.sqrt(lat / np.maximum(k, 1e-6)))
    if n == 0:
        return v
    if v_cap is not None:
        v = np.minimum(v, np.maximum(np.asarray(v_cap, dtype=np.float64), 0.05))
    floor = np.minimum(v, v_min)                       # `speed_profile` と同じ（横の限界が低い所は持ち上げない）
    if v_cap is not None:
        v_min = min(v_min, float(v.min()))
    grip = lat if a_grip is None else np.maximum(
        lat, np.broadcast_to(np.asarray(a_grip, dtype=np.float64), (n,)))
    ig = (1.0 / (grip * grip) if friction_circle else np.zeros(n)).tolist()
    sqrt = math.sqrt
    vl = v.tolist()
    kl = k.tolist()

    if env is not None:
        e_acc, e_dec = tuple(env[0]), tuple(env[1])
        gl = grip.tolist()

        def reach(j: int, a_x: float, d: float, acc: bool) -> float:
            vj = vl[j]
            a_x = env_at(e_acc if acc else e_dec, vj * vj * kl[j] / gl[j])
            return sqrt(vj * vj + 2.0 * max(a_x, 0.0) * d)
    else:
        def reach(j: int, a_x: float, d: float, acc: bool) -> float:
            vj = vl[j]
            g = ig[j]
            if g:
                ay = vj * vj * kl[j]
                r = 1.0 - ay * ay * g
                a_x = a_x * sqrt(r) if r > 0.0 else 0.0
            return sqrt(vj * vj + 2.0 * a_x * d)

    if not closed:
        ds = np.hypot(*np.diff(xy, axis=0).T).tolist() if n >= 2 else []
        vl[-1] = min(vl[-1], v_min if v_end is None else v_end)
        for i in range(n - 2, -1, -1):
            lim = reach(i + 1, a_brake, ds[i], False)
            if lim < vl[i]:
                vl[i] = lim
        if v_start is not None:
            vl[0] = min(vl[0], max(v_start, v_min))
            for i in range(1, n):
                lim = reach(i - 1, a_accel, ds[i - 1], True)
                if lim < vl[i]:
                    vl[i] = lim
        return np.asarray(vl)

    ds = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T).tolist()
    for _ in range(2):
        for i in range(n - 1, -1, -1):
            lim = reach(i + 1 if i + 1 < n else 0, a_brake, ds[i], False)
            if lim < vl[i]:
                vl[i] = lim
        for i in range(n):
            j = i - 1 if i > 0 else n - 1
            lim = reach(j, a_accel, ds[j], True)
            if lim < vl[i]:
                vl[i] = lim
    return np.maximum(np.asarray(vl), floor)


def _travel_time(xy: np.ndarray, v: np.ndarray, closed: bool) -> float:
    """速度プロファイルどおりに走ったときの時間 [s]（閉路なら1周）。"""
    v = np.maximum(v, 1e-3)
    if closed:
        ds = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
        return float(np.sum(2.0 * ds / (v + np.roll(v, -1))))
    ds = np.hypot(*np.diff(xy, axis=0).T)
    return float(np.sum(2.0 * ds / (v[:-1] + v[1:])))


def retime(rl: RaceLine, *, v_max: float, v_min: float, a_lat, a_accel: float,
           a_brake: float, v_start: float | None = None, v_end: float | None = None,
           a_grip=None, v_cap: np.ndarray | None = None, env: tuple | None = None) -> RaceLine:
    """経路の形はそのままで、速度プロファイルだけを今の設定で作り直す。

    ★ 速度は地図を作ったとき（BUILD）の設定で焼き込まれ、保存した地図から走ると
    そのまま使われていた。**GUI で最高速度や加速度を変えても効かなかった。**
    planner は設定が変わるたびにこれを呼ぶ。
    """
    v = speed_profile(rl.xy, speed_kappa(rl.xy, rl.closed), v_max=v_max, v_min=v_min,
                      a_lat=a_lat, a_accel=a_accel, a_brake=a_brake, closed=rl.closed,
                      v_start=v_start, v_end=v_end, a_grip=a_grip, v_cap=v_cap,
                      env=env)
    # `_replace` は使えない（`RaceLine.__len__` が点の数を返すので、NamedTuple の
    # 要素数の検査に引っかかる）
    return RaceLine(**{**rl._asdict(), "v": v})


#: 見通しを測る間隔と上限 [m]（`sight_distance`）
SIGHT_STRIDE_M = 0.15
#: ★ 最高速度 5m/s を「見えている距離の中で止まれる」には 7.05m 要る（学習なしの上限・空走 0.25s で。
#:   5.2 のままだと、見通しがどれだけ利いていても 4.1m/s で頭打ちになる）。2026-11-03 に 5.2 → 7.2
SIGHT_MAX_M = 7.2


def sight_distance(grid, xy: np.ndarray, closed: bool = True, *, max_m: float = SIGHT_MAX_M,
                   stride: float = SIGHT_STRIDE_M, half_width: float = 0.0) -> np.ndarray:
    """経路の各点から、**経路の先がどこまで見えているか** [m]（経路に沿った長さ、`max_m` で頭打ち）。

    点 i から、経路上を `stride` ずつ先の点へ向けてレイを撃ち、壁に遮られずに届いた最も遠い点
    までの弧長。壁の先端を回るヘアピンでは、頂点の手前で 1m を切る。LiDAR は経路の上に居る
    とみなす（取付位置のずれ 数cm は見ない）。地図ごと・経路ごとに 1 回測れば足りる
    （点 350 × 24 本 × 3 で、Mac で 20ms 前後）。

    :param half_width: これが正なら、先の点の**左右 `half_width` の点も**見えていることを求める
        （車体が通る帯の両端）。経路の点そのものは見えていても、内側の壁の陰に帯の端が隠れて
        いることがある——そこに居る障害物は、帯の端が見えるまで見えない（sim.bench の toyota で、
        経路の点は 1.6m 手前から見えるのに、内側へ 14cm 寄った障害物は 1.4m 手前まで見えなかった）。
        壁の中に落ちる端は見えなくてよい（そこに物は居ない）
    """
    n = len(xy)
    if n < 3:
        return np.full(n, max_m)
    seg = np.hypot(*((np.roll(xy, -1, axis=0) - xy) if closed else np.diff(xy, axis=0)).T)
    step = max(float(np.median(seg)), 1e-3)
    k_stride = max(1, int(round(stride / step)))
    res = float(grid.resolution)
    sight = np.zeros(n)
    alive = np.ones(n, dtype=bool)
    idx = np.arange(n)
    offsets = (0.0, half_width, -half_width) if half_width > 0.0 else (0.0,)
    nrm = normals(xy) if closed else normals_open(xy)
    ox, oy = float(grid.origin[0]), float(grid.origin[1])
    # 壁を 2 セル太らせた版で「端が壁の中か」を見る（壁際の端は、壁すれすれのレイが届かない）
    wall = dilate(grid.wall_mask(), 2)
    for m in range(1, max(1, int(round(max_m / stride))) + 1):
        j = idx + m * k_stride
        if closed:
            j = j % n
        else:
            alive &= j < n
            j = np.minimum(j, n - 1)
        if not alive.any():
            break
        for o in offsets:
            tgt = xy[j] + nrm[j] * o
            d = tgt - xy
            dist = np.hypot(d[:, 0], d[:, 1])
            ang = np.arctan2(d[:, 1], d[:, 0])
            hit = grid.raycast(xy[:, 0], xy[:, 1], ang, float(dist.max()) + 2.0 * res, step=res)
            seen = hit >= dist - 1.5 * res
            if o != 0.0:
                col = np.clip(((tgt[:, 0] - ox) / res).astype(np.int64), 0, wall.shape[1] - 1)
                row = np.clip(((tgt[:, 1] - oy) / res).astype(np.int64), 0, wall.shape[0] - 1)
                seen |= wall[row, col]                 # 帯の端が壁の中。そこに物は居ない
            alive &= seen
        sight[alive] = m * k_stride * step
    return np.minimum(sight, max_m)


def sight_speed(sight: np.ndarray, *, a_brake: float, react_s: float, reserve: float) -> np.ndarray:
    """見えている距離の中で止まれる速度 [m/s]: `v·react_s + v²/(2·a_brake) ≤ sight − reserve`。

    見通しの利かないコーナーの先に障害物が居ても、見えた時点で止まり切れる速度。`reserve` は
    車体前端までの距離と、障害物の手前に残す間隔。
    """
    d = np.maximum(np.asarray(sight, dtype=np.float64) - reserve, 0.0)
    a = max(a_brake, 1e-3)
    return -a * react_s + np.sqrt((a * react_s) ** 2 + 2.0 * a * d)


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


def tight_sections(xy: np.ndarray, closed: bool, kappa_limit: float) -> tuple[float, float]:
    """車が曲がり切れない区間の `(長さ [m], いちばんきつい所の 曲率/上限)`。無ければ `(0, ≤1)`。

    道幅が足りず半径を開けなかった所が残っていないかを人に知らせるのに使う（速度と同じ
    弦 `SPEED_CHORD_M` の曲率で測る）。
    """
    if len(xy) < 3 or kappa_limit <= 0.0:
        return 0.0, 0.0
    ks = speed_kappa(xy, closed)
    seg = np.hypot(*((np.roll(xy, -1, axis=0) - xy) if closed else np.diff(xy, axis=0)).T)
    step = float(np.median(seg)) if len(seg) else 0.0
    over = ks > kappa_limit * 1.01
    return float(over.sum()) * step, float(ks.max()) / kappa_limit


#: `refine_time` が試すこぶの幅（標準偏差、点の数）。粗い方から細かい方へ
REFINE_WIDTHS = (32, 16, 8, 4)
#: 同じく、こぶの高さの初期値と、これを下回ったら次の幅へ移る値 [m]
REFINE_AMP0 = 0.08
REFINE_AMP_MIN = 0.006
#: `refine_time` が幅を1巡して縮んだ割合がこれ未満なら、次の巡回をしない
REFINE_STOP = 0.002
#: 仕上げの間、3点で測った曲率（0.1m 刻みの折れ）は上限のこの倍まで（弦の曲率だけを見ると、
#: 弦より細かい折れを作って速度を稼ぐ線が出る）
REFINE_KINK = 1.15


def _loop_mean(xy: np.ndarray, w: int) -> np.ndarray:
    """閉じた点列の、前後 `w` 点の移動平均（`refine_time` の「引き締める」動きの行き先）。"""
    k = 2 * w + 1
    pad = np.concatenate([xy[-w:], xy, xy[:w]])
    c = np.cumsum(np.vstack([np.zeros((1, 2)), pad]), axis=0)
    return (c[k:] - c[:-k]) / k


def refine_time(xy: np.ndarray, cost, *, step: float, max_evals: int = 6000,
                widths: tuple[int, ...] = REFINE_WIDTHS, cycles: int = 1,
                tighten: bool = True) -> np.ndarray:
    """閉じた経路を、`cost`（小さいほど良い）が下がる向きへ**直接**動かす（決定的なパターン探索）。

    ## なぜ要るか

    最小曲率の解は「曲がりの総量」を減らすので、コーナーでは必ず道幅いっぱいに大回りする。
    しかし速度が最高速度で頭打ちの所（3m/s・横加速度 3.7m/s² なら半径 2.4m より緩い所）では、
    半径をそれ以上開いても速くならず、**遠回りしたぶんだけ遅い**。重みを変えて解き直す
    （`optimize` の `time_iters`）のは曲率の配分を変えるだけで、この損は消えない。
    見積もりのラップ時間そのものを目的にすると、同じ制約のまま toyota2 で −14%・toyota −7.5%・
    チケイン −4%（経路は 2〜6% 短くなる。`kappa_max` を限界の 0.9 倍にした比較）。

    ## やり方

    経路の法線方向へ、幅 `w` 点・高さ `amp` のガウス形のこぶを足して `cost` が下がれば採る。
    中心を `w` 点おきに1周ずらし、±の両方を試す。1周して1つも採れなければ高さを半分にし、
    `REFINE_AMP_MIN` を切ったら幅を半分にする。こぶは滑らかなので折れた線にはならない。
    制約（車体と壁の余裕・曲率の上限）は `cost` の側が「破った解は必ず後ろ」にして守らせる
    （`optimize` の `score`）ので、**破った解は1度も採らない**。乱数は使わない。

    幅を変えるたびに点を等間隔に打ち直す（動かすと間隔が崩れ、弦の曲率の意味が変わる）。

    ## ★ こぶだけでは S 字で止まる → 「引き締める」動きと、幅の巡回（`tighten`・`cycles`）

    こぶは1か所を法線方向へ押すだけなので、S 字の2つの頂点を**同時に**内へ寄せて真っすぐ抜ける
    線には届かない（片方だけ動かすと、もう片方の曲率が上限を超えて採れない）。出発点の中心線が
    少し違うだけで別の線に止まり、実機の地図では見積もりが 10.2s にも 8.7s にもなった
    （`graph_margin` を変えると経路が変わる、とバンビが気づいた現象）。
    そこで各中心で、こぶの前に「その周り `w` 点を移動平均へ寄せる」（＝たるみを取って短くする）
    動きも試す。細かい幅まで終えたら粗い幅へ戻ってもう一巡する（細かい幅で空いた余地を使う）。

    :param cost: `cost(xy) -> 比較できる値`（タプルなら辞書式）
    :param max_evals: `cost` を呼ぶ回数の上限（計算時間の上限。`optimize` の評価は C 版で1回 Pi 約0.15ms、
        Python 版で Pi 約1.7ms）
    :param cycles: 幅の列を粗い方からやり直す回数の上限（1巡して何も採れなければそこで終わる）
    """
    n = len(xy)
    if n < 16:
        return xy
    best = cost(xy)
    evals = 1
    usable = [w for w in widths if w * 4 <= n]
    mark = best                                # この巡回を始めたときの最良
    for k, w in enumerate([w for _ in range(max(1, cycles)) for w in usable]):
        if k and k % len(usable) == 0:
            # 1巡終えた。縮みが 0.2% に届かなければ収束したとみなして終わる（時間の節約）
            t0, t1 = mark[-1], best[-1]
            if best[:-1] == mark[:-1] and t1 > t0 * (1.0 - REFINE_STOP):
                break
            mark = best
        # 中心 0 のこぶの形。中心 c のこぶはこれを c だけ回したものと要素ごとに同じ値になる
        # （`d = (i − c + n//2) % n − n//2` は i − c だけで決まる）。exp を中心ごとに計算し直さない
        d = (np.arange(n) + n // 2) % n - n // 2
        g0 = np.exp(-0.5 * (d / w) ** 2)
        pull = None
        amp = REFINE_AMP0
        while amp >= REFINE_AMP_MIN and evals < max_evals:
            improved = False
            nrm = normals(xy)
            for c in range(0, n, w):
                g = np.roll(g0, c)[:, None]
                # 候補は試す直前に作る（採れたら残りは作らない）。引き締めの行き先（移動平均）は
                # 線が変わったときだけ計算し直す
                for kind in ((0, 1, 2) if tighten else (1, 2)):
                    if kind == 0:
                        if pull is None:
                            pull = _loop_mean(xy, w) - xy
                        # 引き締め: 寄せる割合は高さ `amp` に合わせて小さくしていく
                        q = xy + (amp / REFINE_AMP0) * g * pull
                    elif kind == 1:
                        bump = amp * g * nrm
                        q = xy + bump
                    else:
                        q = xy - bump
                    cq = cost(q)
                    evals += 1
                    if cq < best:
                        xy, best, improved = q, cq, True
                        pull = None
                        break
                if evals >= max_evals:
                    break
            if not improved:
                amp *= 0.5
        q = resample_loop(xy, step)
        cq = cost(q)
        evals += 1
        # 打ち直しで見積もりがわずかに動く。制約を破らず、悪化が小さければ採る
        if cq[:-1] <= best[:-1] and cq[-1] <= best[-1] * 1.002:
            xy, best = q, cq
            n = len(xy)
        if evals >= max_evals:
            break
    return xy


class _Band:
    """経路が通ってよい帯（中心線の各点から法線方向に `lo`〜`hi`）を格子に塗ったもの。

    ★ `refine_time` の仕上げは車体と**本当の壁**の距離しか見ない。それだけだと、壁に当たらない
    限りどこへでも線を動かせるので、分岐のあるコースでは**選んだ周回の回廊を出て隣の枝・近道へ
    ずれ込む**（実機の地図で、探索を長く回すと見積もり 10.06→8.69s の「別の周回」になった。
    経由点で選んだ道から外れる）。最小曲率の段階が使ったのと同じ帯——道幅は
    回廊（`CorridorGrid`）のレイで測ってある——の外へ出た線は採らない。
    """

    def __init__(self, grid, c: np.ndarray, nrm: np.ndarray, lo: np.ndarray, hi: np.ndarray,
                 closed: bool, pad: float = 0.05) -> None:
        import cv2
        self.res = float(grid.resolution)
        self.origin = (float(grid.origin[0]), float(grid.origin[1]))
        h, w = grid.wall_mask().shape
        a = c + nrm * (lo - pad)[:, None]
        b = c + nrm * (hi + pad)[:, None]

        def cell(p: np.ndarray) -> np.ndarray:
            return np.floor((p - np.asarray(self.origin)) / self.res).astype(np.int32)

        mask = np.zeros((h, w), dtype=np.uint8)
        n = len(c)
        ca, cb = cell(a), cell(b)
        for i in range(n if closed else n - 1):
            j = (i + 1) % n
            cv2.fillPoly(mask, [np.array([ca[i], cb[i], cb[j], ca[j]], dtype=np.int32)], 1)
        self.mask = cv2.dilate(mask, np.ones((3, 3), np.uint8)).astype(bool)
        self._mask8 = np.ascontiguousarray(self.mask.view(np.uint8))

    def inside(self, xy: np.ndarray) -> bool:
        if _NATIVE is not None:
            xy = _f64(xy)
            h, w = self.mask.shape
            return bool(_NATIVE.all_inside(len(xy), _addr(xy), _addr(self._mask8), h, w,
                                           self.origin[0], self.origin[1], self.res))
        col = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(np.int64)
        row = np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(np.int64)
        h, w = self.mask.shape
        if bool(((col < 0) | (col >= w) | (row < 0) | (row >= h)).any()):
            return False
        return bool(self.mask[row, col].all())


#: 車体外形の検査で制約を締め直す回数の上限（`_BodyCheck`）
BODY_ITERS = 8
#: 車が曲がり切れない所（`kappa_max` 超え）の曲率の重みを上げて解き直す回数の上限（`optimize`）
KAPPA_ITERS = 8


class _BodyCheck:
    """地図の壁と**車体外形**の距離を経路の各点で測る（`optimize` の `footprint`）。

    ★ 道幅は中心線の法線方向へ撃ったレイで測るので、**薄い仕切り壁の先端**が
    見えない（レイは先端の横を素通りする）。そこを後輪軸の余裕だけで通ると、
    30cm 前に張り出した車体の前端が先端をかすめる。toyota2 の上の仕切りの先端で
    2m/s・横偏差数cmで毎回ぶつかった（sim.bench）。車体を円で覆い
    （`local_map.footprint_circles`）、壁の距離場で直接確かめる。
    """

    def __init__(self, grid, footprint, closed: bool, extra_wall: np.ndarray | None = None) -> None:
        """:param extra_wall: 壁に加えて壁とみなすセル（未知を壁にしたときの余裕を測る、`unknown_short`）"""
        wall = getattr(grid, "base_wall_mask", grid.wall_mask)()
        if extra_wall is not None:
            wall = wall | extra_wall
        self.res = float(grid.resolution)
        self.origin = (float(grid.origin[0]), float(grid.origin[1]))
        self.edt = _edt(~wall, self.res)
        self.circles, self.r = footprint_circles(footprint)
        self.closed = closed
        # C 版（`clearance`）に渡す写し。距離場が float32 でなければ（cv2 の無い環境）numpy で測る
        self._edt32 = (np.ascontiguousarray(self.edt) if self.edt.dtype == np.float32 else None)
        self._cx = _f64(self.circles[:, 0])
        self._cy = _f64(self.circles[:, 1])

    def clearance(self, xy: np.ndarray, offset: np.ndarray | None = None) -> np.ndarray:
        """各点に車体を置いたときの壁までの余裕 [m]（負 = 食い込み）。

        向きは `xy` の接線。`offset` を与えると位置だけずらして測る（向きは同じ）。
        """
        if _NATIVE is not None and self._edt32 is not None and len(xy):
            # 下の numpy と同じ演算（`nav/_native.c` の `body_min_edt`、ビット一致）
            xy = _f64(xy)
            off = None if offset is None else _f64(np.broadcast_to(offset, xy.shape))
            out = np.empty(len(xy), dtype=np.float32)
            h, w = self._edt32.shape
            _NATIVE.body_min_edt(len(xy), _addr(xy), None if off is None else _addr(off),
                                 int(self.closed), len(self._cx), _addr(self._cx), _addr(self._cy),
                                 _addr(self._edt32), h, w, self.origin[0], self.origin[1],
                                 self.res, _addr(out))
            return out - self.r
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
             time_iters: int = 0, a_grip: float | None = None,
             refine_evals: int = 0, refine_cycles: int = 4,
             refine_starts: int = 1, refine_parallel: bool = False) -> RaceLine:
    """中心線からレーシングラインを作る。

    :param half_width: 車体半幅 [m]。`config/vehicle.toml` の外形から取る
    :param margin: それに足す安全余裕 [m]
    :param front_overhang: base_link から車体前端まで [m]。コーナーで車体の角が
        経路の外へ張り出すぶんを余裕に足す（`body_allowance`）。0 なら見込まない
    :param rear_overhang: 同 後端まで [m]
    :param passes: 「最適化 → 法線と幅を測り直す」の反復回数
    :param kappa_max: 経路に許す曲率の上限 [1/m]（車の曲がれる限界 × 余裕。下の「車が曲がり
        切れる半径に開く」）。これを超える線は、超えない線より必ず後ろに並べる
    :param footprint: 車体外形（`config/vehicle.toml` の `footprint`）。与えると、車体と
        地図の壁の距離が `margin` を切る点の制約を締めて解き直す（`_BodyCheck`）
    :param time_iters: 最小時間へ寄せる再重み付けの回数（0 = 最小曲率のまま、下記）
    :param refine_evals: 見積もりのラップ時間を直接下げる仕上げの評価回数の上限（0 = しない、
        `refine_time`）。閉じた経路で `time_iters > 0` のときだけ。出発点1つあたりの上限
    :param refine_starts: 仕上げを始める出発点の数（下の「出発点を変えて仕上げる」）。時間は
        この数に比例する
    :param refine_parallel: 2本目以降の出発点の仕上げを fork した子プロセスで同時に回す
        （`_refine_forked`。結果は逐次と同じ）。コアをもう1つ使うので、走行中の引き直しでは使わない

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

    速度は `v_min` で下支えされる（横の限界が `v_min` より低い所を除く）ので、見積もりの時間には「曲がり切れない」ことの損が
    出ない。壁の先端を回るヘアピンで半径 37cm（車の限界 46cm）の線が最短として選ばれ、
    実車は舵が張り付いたまま出口で 14〜20cm 膨らんで壁まで 0〜2cm だった。
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
               closed=closed, v_start=v_start, v_end=v_end, a_grip=a_grip)

    body = _BodyCheck(grid, footprint, closed) if footprint is not None else None

    def score(xy: np.ndarray) -> tuple[int, float, float]:
        """小さいほど良い。★ 車体外形の余裕が足りない解は、足りる解より必ず後ろ。
        そのうえで `time_iters` なら見積もりの走行時間、そうでなければ Σκ²。"""
        short = int(body is not None and float(body.clearance(xy).min()) < margin - 0.01)
        ks = speed_kappa(xy, closed)
        # ★ 車が曲がり切れない解は、曲がり切れる解より必ず後ろ（超えた量の小さい順）。
        # 速度は `v_min` で下支えされる（横の限界が低い所を除く）ので、時間だけで比べると超えた分が見えない
        over = max(0.0, float(ks.max()) - kappa_max) if kappa_max and len(ks) else 0.0
        if time_iters > 0:
            t = _travel_time(xy, speed_profile(xy, ks, **spd), closed)
            return short, over, t
        return short, over, float((curvature(xy, closed) ** 2).sum())

    best = cl.xy                                   # 何も改善しなければ中心線のまま
    best_energy = score(cl.xy)
    #: 制約を満たした解すべて（仕上げの出発点の候補、`refine_starts`）
    feasible: list[np.ndarray] = []

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
            if energy[0] == 0:
                feasible.append(xy)
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
    # ★ 車体外形の余裕が足りない線（道が狭くて QP が諦めた所がある）は仕上げない。仕上げは
    #   「制約を破る解は必ず後ろ」の辞書式で比べるので、始めから破っていると破ったままの中で
    #   時間だけを縮めにいく（＝壁を無視して近道する）
    if (refine_evals > 0 and time_iters > 0 and closed and len(xy) >= 16
            and best_energy[0] == 0):
        kink = REFINE_KINK * kappa_max if kappa_max else math.inf
        # 最後のパスの中心線と道幅（車体外形で締める前の幅）から、通ってよい帯を作る
        band = _Band(grid, c, nrm, -np.maximum(right - keep, 0.0), np.maximum(left - keep, 0.0),
                     closed)
        seen: list[tuple[int, float, float]] = []      # これまでの最良（`refine_time` の `best` と同じ）

        def cost(q: np.ndarray) -> tuple[int, float, float]:
            """`score` と同じ並び。**安い検査から順に**行い、最良に負けると分かった時点でやめる
            （曲率 0.03ms → 車体外形 0.1ms → 速度プロファイル 0.2ms。試す解の半分以上は制約で落ちる）。"""
            ref = seen[0] if seen else None
            if fused is not None and len(q) <= _FUSED_MAX_N:
                return cost_c(q, ref)
            ks = speed_kappa(q, True)
            over = max(0.0, float(ks.max()) - kappa_max) if kappa_max else 0.0
            over = round(max(over, float(np.abs(curvature(q, True)).max()) - kink, 0.0), 4)
            if ref is not None and over > ref[1]:
                return 0, over, math.inf               # 余裕が足りていても、曲率で負ける
            if not band.inside(q):
                return 1, over, math.inf               # 選んだ道の帯を出た
            if float(body.clearance(q).min()) < margin - 0.01 if body is not None else False:
                return 1, over, math.inf
            out = 0, over, _travel_time(q, speed_profile(q, ks, **spd), True)
            if ref is None or out < ref:
                seen[:] = [out]
            return out

        # 上の `cost` を C で（`_native.c` の `refine_kappa`・`refine_eval`。結果はビット一致）。
        # 点ごとの横の限界（学習値の配列）や cv2 の無い環境の距離場では使わない
        fused = None
        if (_NATIVE is not None and np.ndim(a_lat) == 0
                and (a_grip is None or np.ndim(a_grip) == 0)
                and (body is None or body._edt32 is not None)):
            bh, bw = band.mask.shape
            if body is not None:
                eh, ew = body._edt32.shape
                bargs = (_addr(body._edt32), eh, ew, body.origin[0], body.origin[1], body.res,
                         len(body._cx), _addr(body._cx), _addr(body._cy), float(np.float32(body.r)),
                         margin - 0.01)
            else:
                bargs = (None, 0, 0, 0.0, 0.0, 1.0, 0, None, None, 0.0, 0.0)
            fused = (_addr(band._mask8), bh, bw, band.origin[0], band.origin[1], band.res, *bargs,
                     float(v_max), float(v_min), float(a_lat), float(a_accel), float(a_brake),
                     _inv_grip2(a_lat, a_grip))
            stats = np.empty(2)
            t_out = np.empty(1)
            a_stats, a_t = _addr(stats), _addr(t_out)
            # 点ごとの値の写し（前回の評価の点列と値。C が変わった点の周りだけ計算し直す）。
            # 点の数が変わったら（打ち直し）作り直す
            cache: list = [0, 0, 0, 0, 0, None]   # [点の数, 曲率の領域, 有効か・k, 車体の領域, 有効か]（アドレス）

        def cost_c(q: np.ndarray, ref) -> tuple[int, float, float]:
            q = _f64(q)
            n_q = len(q)
            if cache[0] != n_q:
                arrs = (np.empty(8 * n_q), np.zeros(2, np.int64), np.empty(3 * n_q),
                        np.zeros(1, np.int64))
                cache[:] = [n_q, *map(_addr, arrs), arrs]     # 最後は配列そのもの（解放させない）
            _, kc, meta, bc, bmeta, _arrs = cache
            a_q = _addr(q)
            _NATIVE.refine_kappa(n_q, a_q, SPEED_CHORD_M, kc, meta, a_stats)
            over = max(0.0, float(stats[0]) - kappa_max) if kappa_max else 0.0
            over = round(max(over, float(stats[1]) - kink, 0.0), 4)
            if ref is not None and over > ref[1]:
                return 0, over, math.inf
            if _NATIVE.refine_eval(n_q, a_q, kc, bc, bmeta, *fused, a_t) != 0:
                return 1, over, math.inf               # 帯の外か、車体の余裕が足りない
            out = 0, over, float(t_out[0])
            if ref is None or out < ref:
                seen[:] = [out]
            return out

        # ★ 出発点を変えて仕上げ、いちばん良いものを採る（`refine_starts`）。局所探索なので
        #   出発点で止まる線が変わる。2本目以降は「最良の解から形がいちばん離れた」解——
        #   ふつうは最小時間へ寄せる前の最小曲率の線（大回り）——から始める
        def apart(a: np.ndarray, b: np.ndarray) -> float:
            """線 `a` の点から線 `b` までの距離の最大 [m]（点の数が違ってもよい）。"""
            d = np.hypot(a[::4, None, 0] - b[None, :, 0], a[::4, None, 1] - b[None, :, 1])
            return float(d.min(axis=1).max())

        starts = [xy]
        pool = [f for f in feasible if f is not xy]
        while len(starts) < max(1, refine_starts) and pool:
            far = max(pool, key=lambda f: min(apart(f, s) for s in starts))
            if min(apart(far, s) for s in starts) < 0.03:
                break                                  # どれも同じような線
            starts.append(far)
            pool = [f for f in pool if f is not far]
        def run(s0: np.ndarray) -> np.ndarray:
            seen.clear()
            return refine_time(s0, cost, step=cl.step, max_evals=refine_evals, cycles=refine_cycles)

        done = (_refine_forked(run, starts) if refine_parallel and len(starts) >= 2 and _can_fork()
                else None)
        if done is None:
            done = [run(s0) for s0 in starts]
        results = []
        for q in done:
            seen.clear()
            results.append((cost(q), len(results), q))
        xy = min(results)[2]
    kap = curvature(xy, closed)
    v = speed_profile(xy, speed_kappa(xy, closed), **spd)
    seg = np.hypot(*((np.roll(xy, -1, axis=0) - xy) if closed else np.diff(xy, axis=0)).T)
    short = bool(body is not None and float(body.clearance(xy).min()) < margin - 0.01)
    return RaceLine(xy=xy, v=v, kappa=kap, length=float(seg.sum()), closed=closed, short=short,
                    unknown_short=short or _unknown_short(grid, footprint, closed, xy, margin))


def _can_fork() -> bool:
    """出発点の仕上げを fork で並べてよいか。Linux で、スレッドが自分だけのプロセス（経路のワーカー）に限る
    （スレッドのあるプロセスを fork すると、他のスレッドが持っていたロックが子で解けない。macOS は
    fork した子でシステムのライブラリが落ちることがある）。"""
    import sys
    import threading
    return sys.platform.startswith("linux") and threading.active_count() == 1


def _refine_forked(run, starts: list[np.ndarray]) -> list[np.ndarray] | None:
    """`run(s)` を、`starts[0]` は自分で、残りは fork した子で同時に回す。子が1つでも失敗したら `None`
    （呼び出し側が逐次で回し直す）。子は親（経路のワーカー）が殺されたら一緒に終わる。"""
    import os
    import pickle
    kids: list[tuple[int, int]] = []
    try:
        for s0 in starts[1:]:
            r, w = os.pipe()
            pid = os.fork()
            if pid == 0:                                # 子
                code = 1
                try:
                    os.close(r)
                    try:
                        import ctypes
                        import signal
                        ctypes.CDLL(None).prctl(1, int(signal.SIGKILL))     # PR_SET_PDEATHSIG
                    except Exception:                   # noqa: BLE001
                        pass
                    data = pickle.dumps(run(s0), protocol=pickle.HIGHEST_PROTOCOL)
                    with os.fdopen(w, "wb") as f:
                        f.write(data)
                    code = 0
                finally:
                    os._exit(code)
            os.close(w)
            kids.append((pid, r))
    except OSError:
        for pid, r in kids:
            os.close(r)
            os.waitpid(pid, 0)
        return None
    out: list[np.ndarray] | None = [run(starts[0])]
    for pid, r in kids:
        with os.fdopen(r, "rb") as f:
            data = f.read()
        _, status = os.waitpid(pid, 0)
        if out is not None and status == 0 and data:
            out.append(pickle.loads(data))
        else:
            out = None
    return out


def _unknown_short(grid, footprint, closed: bool, xy: np.ndarray, margin: float) -> bool:
    """未知セルも壁とみなすと、車体外形の余裕が `margin` を切る所があるか（線は変えない）。"""
    if footprint is None or not (hasattr(grid, "unknown_wall_mask") or hasattr(grid, "trinary")):
        return False
    try:
        extra = (grid.unknown_wall_mask() if hasattr(grid, "unknown_wall_mask")
                 else unknown_wall_mask(grid))
        return bool(float(_BodyCheck(grid, footprint, closed, extra).clearance(xy).min())
                    < margin - 0.01)
    except Exception:                      # noqa: BLE001  警告のための測定で線を作れなくしない
        return False
