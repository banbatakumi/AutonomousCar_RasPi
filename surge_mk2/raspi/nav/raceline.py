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

    ① v = min(v_max, sqrt(a_lat / |κ|))     横 G の上限
    ② 後ろ向きパス                          その先で減速が間に合う上限
    ③ 前向きパス                            そこまでで加速できる上限

②③ は閉ループなので2周させて端をつなぐ。**②を先にやる**のは、
「止まれない速度で入る」方が「加速が足りない」より危険だから。
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np

from .centerline import Centerline, lateral_offset, measure, normals, resample_loop
from .grid import OccGrid

__all__ = ["RaceLine", "optimize", "curvature", "speed_profile", "min_curvature_alpha"]


class RaceLine(NamedTuple):
    xy: np.ndarray                         #: (N, 2) [m]
    v: np.ndarray                          #: (N,) 目標速度 [m/s]
    kappa: np.ndarray                      #: (N,) 曲率 [1/m]。左旋回が正
    alpha: np.ndarray                      #: (N,) 中心線からの横ずれ [m]。左が正
    s: np.ndarray                          #: (N,) 始点からの弧長 [m]
    length: float                          #: 1周の長さ [m]

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


def min_curvature_alpha(c: np.ndarray, nrm: np.ndarray,
                        lo: np.ndarray, hi: np.ndarray, *,
                        lam: float = 0.1, step: float = 0.10,
                        max_iter: int = 30) -> np.ndarray:
    """箱制約つき最小曲率問題を解いて `α` を返す。

    :param lo: 各点の下限（右側の余裕。**負の値**）
    :param hi: 各点の上限（左側の余裕）
    :param lam: 中心線へ引き戻す重み。**最短経路とのブレンドつまみ**
    :param step: 点の間隔 [m]。`lam` の正規化に使う（下記）

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
    d2 = _cyclic_d2(n)
    dtd = d2.T @ d2
    nx, ny = nrm[:, 0], nrm[:, 1]

    m = (nx[:, None] * dtd * nx[None, :]
         + ny[:, None] * dtd * ny[None, :]
         + (lam * step ** 4) * np.eye(n))
    b = nx * (dtd @ c[:, 0]) + ny * (dtd @ c[:, 1])
    tol = 1e-12 * max(1.0, float(np.abs(b).max()))

    alpha = np.zeros(n)
    at_lo = np.zeros(n, dtype=bool)
    at_hi = np.zeros(n, dtype=bool)
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
        release = (at_lo & (g < -tol)) | (at_hi & (g > tol))
        if not release.any():
            break
        at_lo[release] = False
        at_hi[release] = False
    return np.clip(alpha, lo, hi)


def curvature(xy: np.ndarray) -> np.ndarray:
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
    return np.where(den > 1e-9, 2.0 * cross / np.maximum(den, 1e-9), 0.0)


def speed_profile(xy: np.ndarray, kappa: np.ndarray, *,
                  v_max: float, v_min: float, a_lat: float,
                  a_accel: float, a_brake: float) -> np.ndarray:
    """曲率から目標速度 [m/s]。閉ループとして前後パスを2周ならす。"""
    n = len(xy)
    ds = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)      # i → i+1 の距離
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(np.abs(kappa), 1e-6)))
    v = np.maximum(v, v_min)

    for _ in range(2):
        # ② 後ろ向き（i の速度は i+1 で止まれる範囲に抑える）
        for i in range(n - 1, -1, -1):
            j = (i + 1) % n
            v[i] = min(v[i], float(np.sqrt(v[j] ** 2 + 2.0 * a_brake * ds[i])))
        # ③ 前向き（i の速度は i-1 から加速できる範囲に抑える）
        for i in range(n):
            j = (i - 1) % n
            v[i] = min(v[i], float(np.sqrt(v[j] ** 2 + 2.0 * a_accel * ds[j])))
    return np.maximum(v, v_min)


def optimize(grid: OccGrid, cl: Centerline, *, half_width: float, margin: float,
             lam: float = 0.1, v_max: float = 1.0, v_min: float = 0.2,
             a_lat: float = 2.0, a_accel: float = 1.0, a_brake: float = 1.5,
             passes: int = 2, max_width: float = 3.0) -> RaceLine:
    """中心線からレーシングラインを作る。

    :param half_width: 車体半幅 [m]。`config/vehicle.toml` の外形から取る
    :param margin: それに足す安全余裕 [m]
    :param passes: 「最適化 → 法線と幅を測り直す」の反復回数

    `passes > 1` にしているのは、最適化で経路が動くと**法線の向きも変わる**ため。
    斜めに当たる法線は道幅を過大に測り、**壁にめり込む解が実行可能に見える**。
    2周目以降は出てきた経路を中心線とみなして**地図を撃ち直す**（幅を補間で
    付け替えると、傾いた法線の誤りがそのまま残る）。

    ## ★ 悪くなったパスは返さない

    この反復は収束を保証しない。経路が壁に寄る → そこで測った余裕が小さくなる →
    次の解がさらに寄る、という往復に入ることがあり、実測で**中心線より曲がった
    経路**が出た（`passes=3`, `lam=0.03`）。曲率エネルギー `Σκ²` が最小だった
    パスを採る。最悪でも中心線そのものより悪くはならない。
    """
    keep = half_width + margin
    c = cl.xy
    nrm = cl.normal
    left, right = cl.w_left, cl.w_right

    best = cl.xy                                   # 何も改善しなければ中心線のまま
    best_energy = float((curvature(cl.xy) ** 2).sum())

    for k in range(max(1, passes)):
        hi = np.maximum(left - keep, 0.0)
        lo = -np.maximum(right - keep, 0.0)
        a = min_curvature_alpha(c, nrm, lo, hi, lam=lam, step=cl.step)
        xy = c + nrm * a[:, None]
        energy = float((curvature(xy) ** 2).sum())
        if energy < best_energy:
            best, best_energy = xy, energy
        if k + 1 >= max(1, passes):
            break
        c = resample_loop(xy, cl.step)
        nrm = normals(c)
        left, right = measure(grid, c, nrm, max_width)

    xy = best
    # **α は最後まで「元の中心線からの横ずれ」**として返す。2周目以降の
    # 解そのものは 1 周目の経路が基準になっており、人間が読む数字にならない
    alpha = lateral_offset(cl, xy)
    kap = curvature(xy)
    v = speed_profile(xy, kap, v_max=v_max, v_min=v_min, a_lat=a_lat,
                      a_accel=a_accel, a_brake=a_brake)
    seg = np.hypot(*(np.roll(xy, -1, axis=0) - xy).T)
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    return RaceLine(xy=xy, v=v, kappa=kap, alpha=alpha, s=s, length=float(seg.sum()))
