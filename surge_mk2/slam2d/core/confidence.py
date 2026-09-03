"""スキャンマッチの点対応から「どの方向が観測できているか」を情報行列として推定する。

`raspi/nav/slam.py`は、コーナーのないループ（oval等）で自己位置が全体的に
回転してしまう問題への対策として、進行方向・横方向で**固定の比率**
（`match_gain_fwd_ratio`）でマッチ結果のブレンドを弱める、という手を使って
いた。これは正しい方向性だが、比率が固定なので「今この瞬間、実際にどちらの
方向が観測できているか」を測っているわけではない。

## 点対応ベースのFisher情報（既定, `_estimate_information_analytic`）

各点について最も近い壁セルとの対応を取り、標準的な点対応ICPのFisher情報
（`J^T J / sigma_pt^2`の総和）で情報行列を作る。密に対応が取れた方向は
点数ぶんだけ鋭く（実際の到達精度どおり）、対応が疎/不在の方向
（corridor problem＝直進方向の壁ぞいでは観測できない）は正しく弱く出る。
実測（下記「旧手法」と同条件、独立ノイズ60回試行、再現性も確認済み）では
無補正のままで実際のばらつきと概ね一致した（`ConfidenceConfig.confidence_scale`
のdocstring参照）——旧手法のような約40〜50倍の過小評価は解消された。

## 旧手法（`_estimate_information_curvature`、既定では未使用）

`core/scanmatch.score_at()`を使い、`match()`が見つけた最良解の周りの一致度
曲面を有限差分で2階微分してHessianを作り、その負値（＝情報行列）を固有値
分解していた。**この一致度曲面（`grid.score_map()`）は粗い探索を安定させる
ために意図的に壁の周りへ半径`radius`セルまでなだらかに盛った尤度場であり、
差分刻み幅をどれだけ細かくしてもこの「盛りの広がり」より鋭い曲率は出ない**
——実測（オーバル直線区間、真の姿勢を固定して独立ノイズ40回試行）で、実際の
広域探索結果のばらつきは位置0.5cm程度なのに対し、旧手法は約22cm相当の
不確かさ（情報行列としては約40〜50分の1）を示唆しており、実際の到達精度を
著しく過小評価することが判明した（`h_xy`を5mm〜40cmで振っても改善せず、
差分刻みの選び方の問題ではなく尤度場の滑らかさ自体が頭打ちの原因と特定）。
ループ拘束（`backend/loop_detection.py`）はこの過小評価された情報行列を
そのままポーズグラフへ渡していたため、本来高精度なはずのループ拘束が
「弱い拘束」として扱われ、蓄積誤差を十分に正せていなかったと考えられる。
フォールバック（対応点が少なすぎる場合）・退行時の切り戻し用に残してある。

## なぜ位置(x,y)と向き(yaw)を分離しないか

点対応ベースのFisher情報は自然に3x3の結合行列（x,y,yawの相関込み）になる
——`J_i`が最初から3列（∂/∂x, ∂/∂y, ∂/∂yaw）を持つため、位置とyawを別々に
固有値分解する必要がない（旧手法は曲率の単位が違う量を混在させないために
分離していたが、Fisher情報の結合はその問題を持たない）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .grid import OccGrid
from .scanmatch import MatcherConfig, score_at
from .types import Cov3, Pose2D, ScanPoints, wrap_angle

__all__ = ["ConfidenceConfig", "estimate_information", "fuse_gaussians"]


@dataclass(frozen=True, slots=True)
class ConfidenceConfig:
    #: Trueなら点対応ベースのFisher情報（既定）、Falseなら旧・曲率ベース
    #: （`grid.score_map()`の有限差分）に切り替える。退行時にすぐ戻せるよう
    #: 残してある（上のモジュールdocstring参照）
    analytic: bool = True
    #: 点1つあたりの対応誤差の標準偏差[m]。`scan2scan.py`の`RESID_SIGMA`
    #: （LD06実測）と同じ値を既定にしてある。壁セル自体の量子化誤差
    #: （一様分布、標準偏差`resolution/sqrt(12)`）を二乗和で合成して使う
    resid_sigma: float = 0.013
    #: 最近傍壁セルを探す窓の半径[セル]。スキャンマッチが既に収束済みの点群を
    #: 前提にした固定窓（遠くの壁を拾いにいかない）
    search_cells: int = 4
    #: 対応が取れた点がこれ未満なら（対応スカスカ＝地図がまだ薄い等）旧・
    #: 曲率ベースにフォールバックする
    min_points: int = 8
    #: Fisher情報に掛ける経験的な補正係数（0〜1）。既定は補正なし——実測
    #: （オーバル直線区間、真の姿勢を固定して独立ノイズ60回試行、シード
    #: を変えて再現性も確認済み）で、無補正の点対応Fisher情報は広域探索結果の
    #: 実際のばらつきと概ね一致した（実測std/推定sigma比: x=1.41, y=1.53,
    #: yaw=0.93。旧・曲率ベース手法は同条件で約40〜50倍もの過小評価だった）。
    #: シーンによってはズレが大きくなる可能性があるので、追加の実測校正が
    #: 必要になった場合に備えて残してある調整口
    confidence_scale: float = 1.0

    #: 位置方向の有限差分の刻み幅[m]。Noneなら`grid.resolution * 2.0`を使う（旧手法用）
    h_xy: float | None = None
    h_yaw: float = math.radians(0.5)
    #: 固有値のクリップ範囲。0にクリップすると後段の逆行列計算が壊れるので
    #: 下限を設ける。上限は数値的な暴走を防ぐ
    min_eigenvalue: float = 1e-6
    max_eigenvalue: float = 1e6
    matcher: MatcherConfig = MatcherConfig()


def estimate_information(grid: OccGrid, pts: ScanPoints, pose: Pose2D, *,
                         config: ConfidenceConfig = ConfidenceConfig()) -> Cov3:
    """`pose`（`match()`の最良解）での観測の信頼度を3x3の情報行列で返す。

    既定は点対応ベースのFisher情報（`config.analytic=True`）。対応点が
    `config.min_points`未満なら旧・曲率ベースにフォールバックする
    （上のモジュールdocstring参照）。
    """
    if config.analytic:
        info = _estimate_information_analytic(grid, pts, pose, config)
        if info is not None:
            return info
    return _estimate_information_curvature(grid, pts, pose, config)


def _nearest_wall_points(grid: OccGrid, wx: np.ndarray, wy: np.ndarray, *,
                         search_cells: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`(wx, wy)`（世界座標の点群）それぞれについて、最も近い壁セル中心の座標を返す。

    見つからない点は`found`がFalse。ベクトル化のため探索窓は固定半径
    （`search_cells`セル）——スキャンマッチが既に収束済みの点群を前提にしており、
    遠くの壁を拾いにいかない（`match()`の探索そのものとは別の目的）。
    """
    wall = grid.wall_mask()
    h, w = wall.shape
    col, row = grid.to_cell(wx, wy)
    n = wx.size
    best_dist2 = np.full(n, np.inf)
    best_wx = np.zeros(n)
    best_wy = np.zeros(n)
    found = np.zeros(n, dtype=bool)

    for dr in range(-search_cells, search_cells + 1):
        for dc in range(-search_cells, search_cells + 1):
            rr, cc = row + dr, col + dc
            ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
            occ = np.zeros(n, dtype=bool)
            occ[ok] = wall[rr[ok], cc[ok]]
            if not occ.any():
                continue
            cwx, cwy = grid.to_world(cc, rr)
            d2 = (cwx - wx) ** 2 + (cwy - wy) ** 2
            better = occ & (d2 < best_dist2)
            best_dist2[better] = d2[better]
            best_wx[better] = cwx[better]
            best_wy[better] = cwy[better]
            found[better] = True
    return best_wx, best_wy, found


def _local_wall_tangents(grid: OccGrid, qx: np.ndarray, qy: np.ndarray, *,
                         window_cells: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """対応先の壁セル`(qx, qy)`ごとに、近傍壁セル群のPCA主成分（接線方向）を返す。

    3本目の戻り値`enough`は近傍壁セルが3個以上あった点（＝平面/直線とみなせる点）
    を示す。**なぜ点対応だけでは足りないか**: `_nearest_wall_points`が返す
    対応は「今このposeでの最近傍」でしかなく、壁に沿って滑らせても毎回別の
    壁セルが最近傍になり続けるため、点対応(point-to-point)のFisher情報は
    corridor problem（壁沿い方向の不定性）を表現できない。近傍壁セルの主成分
    方向（接線）に垂直な法線だけを残差に使う(point-to-plane)ことで、壁に
    正対する方向だけが拘束されるという正しい構造になる。
    """
    wall = grid.wall_mask()
    h, w = wall.shape
    col, row = grid.to_cell(qx, qy)
    n = qx.size
    sx = np.zeros(n); sy = np.zeros(n)
    sxx = np.zeros(n); syy = np.zeros(n); sxy = np.zeros(n)
    cnt = np.zeros(n)
    res = grid.resolution

    for dr in range(-window_cells, window_cells + 1):
        for dc in range(-window_cells, window_cells + 1):
            rr, cc = row + dr, col + dc
            ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
            occ = np.zeros(n, dtype=bool)
            occ[ok] = wall[rr[ok], cc[ok]]
            if not occ.any():
                continue
            ox, oy = dc * res, dr * res
            sx[occ] += ox; sy[occ] += oy
            sxx[occ] += ox * ox; syy[occ] += oy * oy; sxy[occ] += ox * oy
            cnt[occ] += 1.0

    enough = cnt >= 3.0
    safe_cnt = np.maximum(cnt, 1.0)
    mx, my = sx / safe_cnt, sy / safe_cnt
    cxx = sxx / safe_cnt - mx * mx
    cyy = syy / safe_cnt - my * my
    cxy = sxy / safe_cnt - mx * my

    cov = np.zeros((n, 2, 2))
    cov[:, 0, 0], cov[:, 1, 1], cov[:, 0, 1], cov[:, 1, 0] = cxx, cyy, cxy, cxy
    _eigvals, eigvecs = np.linalg.eigh(cov)     # 昇順。最大固有値=接線方向は[...,1]
    tx, ty = eigvecs[:, 0, 1], eigvecs[:, 1, 1]
    return tx, ty, enough


def _estimate_information_analytic(grid: OccGrid, pts: ScanPoints, pose: Pose2D,
                                    config: ConfidenceConfig) -> Cov3 | None:
    """点対応ベースのFisher情報（`J^T J / sigma_pt^2`の総和）。

    壁セルの近傍が3点以上ある対応点は**点対平面(point-to-plane)**残差
    （壁の法線方向の1自由度）、孤立した対応点（角・障害物等）は**点対点**
    残差（2自由度）を使う——corridor problemのような壁沿いの不定性は前者が、
    角のような真に2D拘束がある特徴は後者が正しく表現する（上の
    `_local_wall_tangents`docstring参照）。

    対応点が`config.min_points`未満なら`None`を返し、呼び出し側で曲率ベースに
    フォールバックさせる。
    """
    px = pts.x[pts.hit]
    py = pts.y[pts.hit]
    if px.size == 0:
        return None

    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    wx = pose.x + px * c - py * s
    wy = pose.y + px * s + py * c

    qx, qy, found = _nearest_wall_points(grid, wx, wy, search_cells=config.search_cells)
    if int(found.sum()) < config.min_points:
        return None

    px, py = px[found], py[found]
    wx, wy = wx[found], wy[found]
    qx, qy = qx[found], qy[found]

    tx, ty, enough = _local_wall_tangents(grid, qx, qy, window_cells=config.search_cells)
    # 法線 = 接線を90度回転。符号はどちらでも(n・J)^2で消えるので揃える必要はない
    nx, ny = -ty, tx

    # ∂(wx,wy)/∂yaw = (a_i, b_i)（各点の姿勢に対するヤコビ行列の3列目）
    a = -px * s - py * c
    b = px * c - py * s

    sigma_pt2 = config.resid_sigma ** 2 + (grid.resolution ** 2) / 12.0

    info = np.zeros((3, 3))
    # 点対平面: J_i = [n_x, n_y, n_x*a_i + n_y*b_i]（1x3）。H += J_i^T J_i / sigma_pt2
    if enough.any():
        jnx, jny = nx[enough], ny[enough]
        jnt = jnx * a[enough] + jny * b[enough]
        J = np.stack([jnx, jny, jnt], axis=1)          # (m, 3)
        info += (J.T @ J) / sigma_pt2

    # 点対点(孤立点=角・障害物): J_i = [[1,0,a_i],[0,1,b_i]]（2x3）。H += J_i^T J_i / sigma_pt2
    if (~enough).any():
        ai, bi = a[~enough], b[~enough]
        m = ai.size
        info[0, 0] += m / sigma_pt2
        info[1, 1] += m / sigma_pt2
        info[0, 2] += float(np.sum(ai)) / sigma_pt2
        info[2, 0] += float(np.sum(ai)) / sigma_pt2
        info[1, 2] += float(np.sum(bi)) / sigma_pt2
        info[2, 1] += float(np.sum(bi)) / sigma_pt2
        info[2, 2] += float(np.sum(ai * ai + bi * bi)) / sigma_pt2

    info *= config.confidence_scale    # 点間相関を無視した過信の経験的補正（上のdocstring参照）

    # 対応の外れ値・退化方向での数値不安定を防ぐため固有値をクリップする
    # （旧手法と同じ安全策）
    eigvals, eigvecs = np.linalg.eigh(info)
    eigvals = np.clip(eigvals, config.min_eigenvalue, config.max_eigenvalue)
    return eigvecs @ np.diag(eigvals) @ eigvecs.T


def _estimate_information_curvature(grid: OccGrid, pts: ScanPoints, pose: Pose2D,
                                    config: ConfidenceConfig) -> Cov3:
    """`pose`周りの局所曲率（`grid.score_map()`の有限差分）から3x3の情報行列を作る。

    戻り値はx,yの2x2ブロックとyawの1x1ブロックからなるブロック対角行列
    （xy-yaw間の相関は扱わない）。**モジュールdocstring「旧手法」参照**——
    真の到達精度を大幅に過小評価する既知の弱点があり、既定では使わない。
    """
    h_xy = config.h_xy if config.h_xy is not None else grid.resolution * 2.0
    h_yaw = config.h_yaw

    def s(dx: float, dy: float, dyaw: float) -> float:
        p = Pose2D(pose.x + dx, pose.y + dy, wrap_angle(pose.yaw + dyaw))
        return score_at(grid, pts, p, config=config.matcher)

    s0 = s(0.0, 0.0, 0.0)

    sxp, sxm = s(h_xy, 0.0, 0.0), s(-h_xy, 0.0, 0.0)
    syp, sym = s(0.0, h_xy, 0.0), s(0.0, -h_xy, 0.0)
    spp, spm = s(h_xy, h_xy, 0.0), s(h_xy, -h_xy, 0.0)
    smp, smm = s(-h_xy, h_xy, 0.0), s(-h_xy, -h_xy, 0.0)

    hxx = (sxp - 2.0 * s0 + sxm) / h_xy ** 2
    hyy = (syp - 2.0 * s0 + sym) / h_xy ** 2
    hxy = (spp - spm - smp + smm) / (4.0 * h_xy ** 2)

    # 一致度は最良解で極大（＝凹関数）のはずなので、負のHessianが正定値の
    # 情報行列になる。測距ノイズ・格子の量子化で厳密には成り立たないことも
    # あるため、固有値をクリップして安定させる
    h_xy_mat = np.array([[hxx, hxy], [hxy, hyy]])
    eigvals, eigvecs = np.linalg.eigh(-h_xy_mat)
    eigvals = np.clip(eigvals, config.min_eigenvalue, config.max_eigenvalue)
    info_xy = eigvecs @ np.diag(eigvals) @ eigvecs.T

    syawp, syawm = s(0.0, 0.0, h_yaw), s(0.0, 0.0, -h_yaw)
    h_yawyaw = (syawp - 2.0 * s0 + syawm) / h_yaw ** 2
    info_yaw = float(np.clip(-h_yawyaw, config.min_eigenvalue, config.max_eigenvalue))

    info = np.zeros((3, 3))
    info[:2, :2] = info_xy
    info[2, 2] = info_yaw
    return info


def fuse_gaussians(mean_pred: Pose2D, info_pred: Cov3,
                   mean_obs: Pose2D, info_obs: Cov3) -> Pose2D:
    """情報フィルタでの2ガウス分布の融合。

    `raspi/nav/slam.py`の固定利得ブレンド（`_blend_xy`）を一般化したもの。
    情報行列（共分散の逆）どうしはそのまま足せる（`Lambda_post = Lambda_pred
    + Lambda_obs`）。これにより、`info_obs`がほぼ0の方向（corridor problemで
    観測できない方向）では自然に`mean_pred`側が支配的になり、`info_obs`が
    大きい方向では観測側が支配的になる——**方向ごとの利得を手で決める必要が
    なくなる**のがこの定式化の利点。

    `yaw`は周期量なので、`mean_obs`の`yaw`を`mean_pred`に一番近い等価角へ
    補正してから線形代数に載せる（±πを跨ぐと台無しになるため）。
    """
    obs_yaw = mean_pred.yaw + wrap_angle(mean_obs.yaw - mean_pred.yaw)
    pred_vec = np.array([mean_pred.x, mean_pred.y, mean_pred.yaw])
    obs_vec = np.array([mean_obs.x, mean_obs.y, obs_yaw])

    # `info_post`を正則化するのに単位行列を足すだけだと、両方の情報が0の
    # ケースで解が原点(0,0,0)に吸い寄せられてしまう（正則化項がpredではなく
    # 原点を向いてしまうため）。**正則化はpred_vecへ向ける**ことで、
    # 「情報が無ければpredのまま」という直感通りのフォールバックにする
    eps = 1e-9
    reg = np.eye(3) * eps
    info_post = info_pred + info_obs + reg
    b = info_pred @ pred_vec + info_obs @ obs_vec + reg @ pred_vec
    try:
        x = np.linalg.solve(info_post, b)
    except np.linalg.LinAlgError:
        return mean_pred
    return Pose2D(float(x[0]), float(x[1]), wrap_angle(float(x[2])))
