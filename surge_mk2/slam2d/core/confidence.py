"""スキャンマッチの局所曲率から「どの方向が観測できているか」を情報行列として推定する。

`raspi/nav/slam.py`は、コーナーのないループ（oval等）で自己位置が全体的に
回転してしまう問題への対策として、進行方向・横方向で**固定の比率**
（`match_gain_fwd_ratio`）でマッチ結果のブレンドを弱める、という手を使って
いた。これは正しい方向性だが、比率が固定なので「今この瞬間、実際にどちらの
方向が観測できているか」を測っているわけではない。

ここでは`core/scanmatch.score_at()`を使い、`match()`が見つけた最良解の周りの
一致度曲面を有限差分で2階微分してHessianを作り、その負値（＝情報行列）を
固有値分解する。固有値の大小がそのまま「その方向にどれだけ強く拘束されて
いるか」を表す——corridor problemの直進方向では一致度がほぼ平坦（Hessianの
その方向の固有値がほぼ0）になり、壁に正対する方向では鋭く尖る
（固有値が大きい）。

## なぜ位置(x,y)と向き(yaw)を別々に扱うか

x,y[m]とyaw[rad]は単位が異なる量なので、3x3行列をまとめて固有値分解すると
「1mのズレ」と「1radのズレ」を同列に比較することになり、物理的な意味を
持たない。位置の2x2ブロックだけを固有値分解し、yawは独立した1次元として
扱う（`raspi/nav/slam.py`が位置とyawの利得を別々のパラメータに分けていた
のと同じ考え方）。
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
    #: 位置方向の有限差分の刻み幅[m]。Noneなら`grid.resolution * 2.0`を使う。
    #:
    #: ★ `grid.score_map()`の既定`radius=2`セルと同じスケールに合わせてある。
    #: **刻み幅が減衰スケールより小さいと、尤度場の勾配がほぼ線形に見える範囲
    #: しか差分に入らず、2階微分（曲率）がゼロ近くに出て「観測できる方向」を
    #: 「情報が無い」と誤判定する。** 実測（水平な壁1枚、壁に垂直な方向が
    #: 本来強く観測できるはずのシーン）で、`resolution*0.5`では壁に垂直な
    #: 方向の情報量が壁に沿う方向より小さく出る逆転が起きることを確認した
    h_xy: float | None = None
    h_yaw: float = math.radians(0.5)
    #: 固有値のクリップ範囲。0にクリップすると後段の逆行列計算が壊れるので
    #: 下限を設ける。上限は数値的な暴走を防ぐ
    min_eigenvalue: float = 1e-6
    max_eigenvalue: float = 1e6
    matcher: MatcherConfig = MatcherConfig()


def estimate_information(grid: OccGrid, pts: ScanPoints, pose: Pose2D, *,
                         config: ConfidenceConfig = ConfidenceConfig()) -> Cov3:
    """`pose`（`match()`の最良解）周りの局所曲率から3x3の情報行列を作る。

    戻り値はx,yの2x2ブロックとyawの1x1ブロックからなるブロック対角行列
    （xy-yaw間の相関は扱わない。上のモジュールdocstring参照）。
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
    # ケースで解が原点(0,0,0)に吸い寄せられてしまう（正則化項がpredでは
    # なく原点を向いてしまうため）。**正則化はpred_vecへ向ける**ことで、
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
