"""点群を面要素地図に Gauss-Newton で合わせる — 推測航法を事前分布に持つ点対線ICP。

`core/scanmatch.py`（相関スキャンマッチ）は候補姿勢を格子で総当たりして最良を
採る。広い範囲から正解を拾えるので**初期値が悪いとき**には強いが、

- 答えが探索格子に量子化される（最終段4mm・0.05°）
- 評価関数（`score_map()`）が壁の周り2セルだけを盛った階段状の場で、その形が
  そのまま偏りになる
- 推測航法との融合を「得点 − 罰則」で近似しているので、罰則の重みが実際の
  推測航法の不確かさと対応しない

という理由で**推測航法の誤差を正しく正せない**（`sim/slam_bench.py`の実測:
車速の倍率誤差2%だけで ATE が 1.2cm→6cm、ジャイロのゼロ点ずれ0.5°/s だけで
複雑コースの ATE が 11cm）。

ここでは次の最小二乗を Gauss-Newton で解く（毎反復で対応を引き直す＝ICP）:

    min  Σ_i ρ( r_i / σ )²  +  (T ⊖ T_pred)ᵀ Ω_pred (T ⊖ T_pred)

- `r_i` は点と、最寄りの壁の面要素との距離（`core/surfmap.py`）。平面なら
  法線方向の距離（点対平面）、角や柱なら点対点
- `ρ` は Huber。動く物・地図の誤差・未観測の壁で外れる点を弱める
- 第2項は推測航法の事前分布。`Ω_pred`は運動モデルが出す予測共分散の逆

**観測できる方向（壁に垂直）では点群側の情報が桁違いに大きいので点群が勝ち、
観測できない方向（通路の進行方向）では点群側の情報がほぼ0なので推測航法が
残る**——方向ごとの利得を手で決める必要が無い（Cartographer の Ceres スキャン
マッチャ・Censi の PL-ICP と同じ考え方）。

戻り値の`info`は点群側だけの情報行列（`J^T W J / σ²`）。点どうしの相関を無視
すると過信になるので、`info_scale`で割り引いてから返す（ポーズグラフの
エッジ重みに使う）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .surfmap import SurfaceMap
from .types import Cov3, Pose2D, wrap_angle

__all__ = ["RegisterConfig", "RegisterResult", "SearchResult", "register", "search",
           "inlier_ratio"]


@dataclass(frozen=True, slots=True)
class RegisterConfig:
    #: 点1つの残差の標準偏差[m]。LiDAR の測距誤差（LD06 で1〜2cm）と地図自体の
    #: ぼけを合わせた値。**距離に比例する分**（`sigma_rel`）と合成して点ごとに
    #: 重みを変える: σ_i = sqrt(sigma² + (sigma_rel × 距離)²)
    sigma: float = 0.02
    #: 距離に比例する測距誤差の割合。LD06 は遠いほど誤差が伸びる（データシートの
    #: 確度±45mm は近距離の値で、実機では距離に比例する成分が乗る）。
    #: **これが無いと、遠い点ほどノイズが大きいのに同じ重みで効き、回頭の
    #: 推定が長距離のノイズに振り回される**（`sim/slam_bench.py` circuit_chicane_a
    #: の harsh 条件で、測距ノイズを下げるだけで向きのドリフトが25.6°→5.0°に
    #: 減ったのが発端）
    sigma_rel: float = 0.01
    #: Huber の閾値（σの何倍から線形にするか）
    huber_k: float = 2.0
    #: 反復回数の上限と収束判定
    max_iter: int = 12
    tol_xy: float = 1e-4
    tol_yaw: float = 2e-5
    #: 1反復の並進の上限[m]。線形化が破れる大きな一歩を刻む
    max_step: float = 0.10
    #: 照合に使う点の上限（等間隔に間引く）
    max_points: int = 360
    #: 「当たっている」とみなす距離[m]（`inlier`の判定）
    inlier_dist: float = 0.05
    #: 点群側の情報行列の割引率（点どうしの相関を無視した過信の補正）。
    #: 返す`info`だけに掛け、最適化そのものには影響しない
    info_scale: float = 0.1


class RegisterResult(NamedTuple):
    pose: Pose2D
    #: 点群側の情報行列（`info_scale`で割引済み、世界座標）。ポーズグラフの重みに使う
    info: Cov3
    #: 照合に使えた点のうち、壁から`inlier_dist`以内に来た点の割合（0〜1）。
    #: **照合できた点が1つも無ければ0**
    inlier: float
    #: 照合に使えた点（既知の領域に落ち、`max_dist`以内に壁があった点）の数
    used: int
    #: 使えた点の残差の RMS [m]
    rms: float
    iterations: int


def _subsample(px, py, n):
    px = np.asarray(px, dtype=np.float64)
    py = np.asarray(py, dtype=np.float64)
    if px.size > n:
        take = np.linspace(0, px.size - 1, n).astype(np.int64)
        px, py = px[take], py[take]
    return px, py


def _point_sigma(px: np.ndarray, py: np.ndarray, config: RegisterConfig) -> np.ndarray:
    """点ごとの残差の標準偏差[m]（距離に比例する成分を合成）。"""
    r = np.hypot(px, py)
    return np.sqrt(config.sigma ** 2 + (config.sigma_rel * r) ** 2)


def _normal_equations(smap: SurfaceMap, px, py, x, y, yaw, k: np.ndarray, inv_s2: np.ndarray):
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * px - s * py
    wy = y + s * px + c * py
    a = smap.associate(wx, wy)
    h = np.zeros((3, 3))
    b = np.zeros(3)
    ok = a.valid
    if not ok.any():
        return h, b, a
    ax = -s * px - c * py                       # ∂wx/∂yaw
    ay = c * px - s * py                        # ∂wy/∂yaw
    ex = wx - a.qx
    ey = wy - a.qy
    # Huber: しきい値も点ごとのσに比例させる（遠い点は緩く見る）
    w = np.where(a.dist <= k, 1.0, k / np.maximum(a.dist, 1e-12)) * inv_s2

    pl = ok & a.planar
    if pl.any():
        nx, ny = a.nx[pl], a.ny[pl]
        j = np.stack([nx, ny, nx * ax[pl] + ny * ay[pl]], axis=1)
        r = nx * ex[pl] + ny * ey[pl]
        jw = j * w[pl][:, None]
        h += jw.T @ j
        b += jw.T @ r
    pt = ok & ~a.planar
    if pt.any():
        ww = w[pt]
        axp, ayp = ax[pt], ay[pt]
        exp_, eyp = ex[pt], ey[pt]
        # 行 [1,0,ax] と [0,1,ay] の2本ぶん
        h[0, 0] += ww.sum()
        h[1, 1] += ww.sum()
        h[0, 2] += (ww * axp).sum()
        h[1, 2] += (ww * ayp).sum()
        h[2, 2] += (ww * (axp * axp + ayp * ayp)).sum()
        h[2, 0] = h[0, 2]
        h[2, 1] = h[1, 2]
        b[0] += (ww * exp_).sum()
        b[1] += (ww * eyp).sum()
        b[2] += (ww * (axp * exp_ + ayp * eyp)).sum()
    return h, b, a


def register(smap: SurfaceMap, px: np.ndarray, py: np.ndarray, guess: Pose2D, *,
             prior: Pose2D | None = None, prior_info: Cov3 | None = None,
             config: RegisterConfig = RegisterConfig()) -> RegisterResult:
    """点群`(px, py)`（車体座標）を`smap`に合わせた世界姿勢を返す。

    :param guess: 反復の初期値
    :param prior: 事前分布の平均（推測航法の予測）。None なら事前分布なし
    :param prior_info: 事前分布の情報行列（予測共分散の逆、世界座標）
    """
    px, py = _subsample(px, py, config.max_points)
    x, y, yaw = float(guess.x), float(guess.y), float(guess.yaw)
    if prior is not None:
        yaw = prior.yaw + wrap_angle(yaw - prior.yaw)
    use_prior = prior is not None and prior_info is not None
    sig = _point_sigma(px, py, config)
    inv_s2 = 1.0 / (sig ** 2)
    k = config.huber_k * sig

    h_pts = np.zeros((3, 3))
    it = 0
    for it in range(1, config.max_iter + 1):
        h, b, _a = _normal_equations(smap, px, py, x, y, yaw, k, inv_s2)
        h_pts = h
        if use_prior:
            e = np.array([x - prior.x, y - prior.y, wrap_angle(yaw - prior.yaw)])
            h = h + prior_info
            b = b + prior_info @ e
        if not np.any(h):
            break
        try:
            delta = -np.linalg.solve(h + np.eye(3) * 1e-9, b)
        except np.linalg.LinAlgError:
            break
        step = math.hypot(delta[0], delta[1])
        if step > config.max_step:
            delta *= config.max_step / step
        x += float(delta[0])
        y += float(delta[1])
        yaw += float(delta[2])
        if (abs(delta[0]) < config.tol_xy and abs(delta[1]) < config.tol_xy
                and abs(delta[2]) < config.tol_yaw):
            break

    h_pts, _b, a = _normal_equations(smap, px, py, x, y, yaw, k, inv_s2)
    used = int(a.valid.sum())
    if used:
        d = a.dist[a.valid]
        inl = float((d <= config.inlier_dist).mean())
        rms = float(np.sqrt(np.mean(d * d)))
    else:
        inl, rms = 0.0, math.inf
    return RegisterResult(Pose2D(x, y, wrap_angle(yaw)), h_pts * config.info_scale,
                          inl, used, rms, it)


def inlier_ratio(smap: SurfaceMap, px: np.ndarray, py: np.ndarray, pose: Pose2D, *,
                 inlier_dist: float = 0.05) -> tuple[float, int]:
    """`pose`で既知の領域に落ちた点のうち、壁から`inlier_dist`以内の割合と、その母数。"""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    wx = pose.x + c * px - s * py
    wy = pose.y + s * px + c * py
    a = smap.associate(wx, wy)
    known = smap.known_at(wx, wy)
    n = int(known.sum())
    if n == 0:
        return 0.0, 0
    return float((a.valid & (a.dist <= inlier_dist)).sum()) / n, n


class SearchResult(NamedTuple):
    pose: Pose2D
    score: float                  #: 0〜1。`exp(-d²/2σ²)`の点平均（外れ・未知の点は0点）
    #: 最良解から`ambiguity_dist`以上離れた候補の中での最高点。対称・繰り返し形状で
    #: 1位と並ぶなら、その解は信用できない
    second: float


def search(smap: SurfaceMap, px: np.ndarray, py: np.ndarray, center: Pose2D, *,
           trans: float, rot: float, trans_step: float, rot_step: float,
           sigma: float = 0.03, max_points: int = 120,
           ambiguity_dist: float = 0.25, ambiguity_yaw: float = math.radians(20.0)) -> SearchResult:
    """`center`の周り（±`trans`[m]・±`rot`[rad]）を総当たりで探す。

    Gauss-Newton（`register()`）の引き込み範囲を超えてずれたとき（見失いからの
    復帰・ループ閉じ）の初期値を作るためのもの。得点は滑らかな尤度
    `exp(-d²/2σ²)` の点平均で、**分母は点数で固定**（未知・外れの点も0点
    として数える。候補ごとに分母を変えると、少数の点だけがよく合う姿勢が勝つ）。
    """
    px, py = _subsample(px, py, max_points)
    n_pts = max(1, px.size)
    nt = int(math.floor(trans / trans_step + 1e-9))
    nr = int(math.floor(rot / rot_step + 1e-9))
    offs = np.arange(-nt, nt + 1) * trans_step
    dyaws = np.arange(-nr, nr + 1) * rot_step
    inv2s2 = 1.0 / (2.0 * sigma * sigma)

    scores = np.zeros((dyaws.size, offs.size, offs.size))
    for k, dy in enumerate(dyaws):
        yaw = center.yaw + dy
        c, s = math.cos(yaw), math.sin(yaw)
        rx = c * px - s * py
        ry = s * px + c * py
        wx = (center.x + offs)[:, None, None] + rx[None, None, :]
        wy = (center.y + offs)[None, :, None] + ry[None, None, :]
        wx, wy = np.broadcast_arrays(wx, wy)
        a = smap.associate(wx, wy)
        like = np.where(a.valid, np.exp(-np.minimum(a.dist, 10.0) ** 2 * inv2s2), 0.0)
        scores[k] = like.sum(axis=2) / n_pts

    flat = int(np.argmax(scores))
    k, i, j = np.unravel_index(flat, scores.shape)
    best = float(scores[k, i, j])
    pose = Pose2D(center.x + offs[i], center.y + offs[j], wrap_angle(center.yaw + dyaws[k]))

    # 2位: 最良解から**離れた**候補の中での最高点（同じ山の裾は数えない）。
    # 離れている＝位置が`ambiguity_dist`以上違う、または向きが`ambiguity_yaw`以上違う
    # （正方形の部屋のように、同じ位置でも90°回すと同じ形に見えることがある）
    ii, jj = np.meshgrid(offs, offs, indexing="ij")
    far_xy = np.hypot(ii - offs[i], jj - offs[j]) >= ambiguity_dist
    far_yaw = np.abs(dyaws - dyaws[k]) >= ambiguity_yaw
    far = far_xy[None, :, :] | far_yaw[:, None, None]
    second = float(scores[far].max()) if far.any() else 0.0
    return SearchResult(pose, best, second)
