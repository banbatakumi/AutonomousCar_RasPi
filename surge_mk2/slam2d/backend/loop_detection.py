"""ループ検出 — 過去に通った場所へ戻ってきたかを調べ、相対姿勢の拘束を作る。

## 古いキーフレームだけで作った部分地図に照合する

新しいキーフレームの点群を、**時間的に離れた過去のキーフレームだけ**から作った
部分地図（候補の前後`submap_half_m`ぶん）に照合する。

旧実装は「今まさに作っている全体の地図」に照合していた。その地図の現在地
付近は**直前の数秒で焼いた壁（今の推定と整合している）**が大半を占めるので、
照合は今の推定をほぼそのまま追認し、「ドリフトした関係」がループ拘束として
グラフに入っていた（`sim/slam_bench.py`: ループ閉じの後で地図の精度が
0.72→0.26 に**悪化**するコースがあった）。古い点だけに合わせれば、
「今の周の自分」と「前の周の自分」のずれそのものが測れる。

## 手順

1. 走行距離で`min_path_gap`以上前のキーフレームのうち、今の推定位置から
   `radius + radius_per_m × (走った距離)`以内のもの（ドリフトは走った距離に
   比例して広がる）の中で、最も近いものを候補にする
2. 候補の前後`submap_half_m`[m]のキーフレームの点で距離場を作る
3. 総当たり（粗→細）→ Gauss-Newton（事前分布なし）で合わせる
4. 合格条件を**全部**満たしたものだけ拘束にする:
   - 総当たりの1位が2位（1位から`ambiguity_dist`以上離れた候補）より十分高い
     （対称・繰り返し形状での取り違えを弾く）
   - 照合できた点が`min_used`以上、当たり率`min_inlier`以上、RMS`max_rms`以下

拘束の情報行列は位置合わせのヘッセ行列（割引済み）。通路のように1方向しか
決まらない照合では、その方向だけが強い拘束になる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple, Sequence

import numpy as np

from ..core.surfmap import SurfaceMap
from ..core.frontend import Keyframe, inflate_info, rotate_info
from ..core.register import RegisterConfig, register, search
from ..core.types import Cov3, Pose2D, between, wrap_angle

__all__ = ["LoopDetectorConfig", "LoopCandidate", "find_loop_closure", "build_submap"]


@dataclass(frozen=True, slots=True)
class LoopDetectorConfig:
    #: 候補にする過去のキーフレームは、走行距離でこれ以上前のもの[m]
    min_path_gap: float = 6.0
    #: 候補を探す半径[m]。走った距離に比例して広げる（ドリフトは距離に比例する）
    radius: float = 1.0
    radius_per_m: float = 0.05
    #: 候補を探す半径の上限[m]。これ以上ずれていたら、そもそも同定できない
    radius_max: float = 4.0
    #: 部分地図に使う候補の前後の走行距離[m]
    submap_half_m: float = 2.0
    resolution: float = 0.025
    submap_radius: float = 8.5
    #: 総当たりの範囲（粗い段）。**候補までの距離ぶんは必ず探す**——
    #: 候補が見つかるということは、それだけずれている可能性があるということ
    search_trans: float = 0.6
    search_trans_per_dist: float = 1.3
    search_trans_max: float = 3.5
    #: 回頭のずれの探索範囲。ジャイロの倍率誤差は1周ぶんの回頭に比例して
    #: 効く（3%なら1周360°で11°）ので、広めに取る
    search_rot: float = math.radians(30.0)
    #: 合格条件
    min_score: float = 0.45
    ambiguity_ratio: float = 0.9
    ambiguity_dist: float = 0.3
    min_used: int = 60
    min_inlier: float = 0.7
    max_rms: float = 0.03
    #: 再訪とみなす向きの差の上限[rad]。周回コースは毎周同じ向きで通るので、
    #: 逆向き・直交する姿勢での一致は形の取り違えを疑う
    yaw_tolerance: float = math.radians(60.0)
    #: 「今の推定からこれ以上動かす拘束は信じない」上限[m]。ドリフトは走った
    #: 距離に比例して育つので、距離に応じて許容を広げる。
    #: ★ これが無いと、平行な車線のような**自信満々の取り違え**（実測:
    #: course2 で当たり率0.99・得点0.92のまま1.42m ずれた拘束が1本入り、
    #: 地図の精度が0.56→0.36 に崩れた）を止められない
    max_correction: float = 0.3
    max_correction_per_m: float = 0.04
    register: RegisterConfig = RegisterConfig()
    #: 拘束に足す「モデル化していない誤差」（`core/frontend.inflate_info`）
    sigma_xy: float = 0.02
    sigma_yaw: float = math.radians(0.3)


class LoopCandidate(NamedTuple):
    src: int                               #: 過去のキーフレームindex
    dst: int                               #: 現在のキーフレームindex
    delta: Pose2D                          #: srcから見たdstの相対姿勢
    score: float
    #: `delta`の情報行列（**srcの座標系**）
    information: Cov3
    inlier: float
    #: 照合で動いた量（今の推定からの補正量）[m]。ドリフトの大きさの目安
    correction: float


def build_submap(keyframes: Sequence[Keyframe], indices: Sequence[int], center: Pose2D, *,
                 resolution: float, radius: float, max_dist: float = 0.4) -> SurfaceMap:
    """`indices`のキーフレームの点（今の推定姿勢で世界座標へ）から面要素地図を作る。"""
    xs, ys = [], []
    for i in indices:
        kf = keyframes[i]
        c, s = math.cos(kf.pose.yaw), math.sin(kf.pose.yaw)
        xs.append(kf.pose.x + c * kf.hx - s * kf.hy)
        ys.append(kf.pose.y + s * kf.hx + c * kf.hy)
    if not xs:
        xs, ys = [np.zeros(0)], [np.zeros(0)]
    return SurfaceMap.from_points(np.concatenate(xs).astype(np.float64),
                                  np.concatenate(ys).astype(np.float64),
                                  center=(center.x, center.y), radius=radius,
                                  resolution=resolution, max_dist=max_dist)


def find_loop_closure(keyframes: Sequence[Keyframe], new_index: int, *,
                      config: LoopDetectorConfig = LoopDetectorConfig(),
                      exclude: set[int] | None = None) -> LoopCandidate | None:
    """`keyframes[new_index]`が過去のキーフレームの近くに戻ってきたかを調べる。"""
    cur = keyframes[new_index]
    if cur.hx.size < config.min_used:
        return None
    best_i, best_d = -1, math.inf
    for i in range(new_index):
        kf = keyframes[i]
        gap = cur.path - kf.path
        if gap < config.min_path_gap:
            break                              # path は単調増加
        if exclude and i in exclude:
            continue
        if abs(wrap_angle(kf.pose.yaw - cur.pose.yaw)) > config.yaw_tolerance:
            continue
        d = math.hypot(kf.pose.x - cur.pose.x, kf.pose.y - cur.pose.y)
        limit = min(config.radius + config.radius_per_m * gap, config.radius_max)
        if d <= limit and d < best_d:
            best_i, best_d = i, d
    if best_i < 0:
        return None

    src = keyframes[best_i]
    idx = [j for j in range(new_index)
           if abs(keyframes[j].path - src.path) <= config.submap_half_m
           and cur.path - keyframes[j].path >= config.min_path_gap * 0.5]
    field = build_submap(keyframes, idx, src.pose, resolution=config.resolution,
                         radius=config.submap_radius)
    if field.empty:
        return None

    # 探索は粗→中→細の3段。粗い段の範囲は「候補までの距離」に応じて広げ、
    # 刻みと尤度の幅（sigma）を範囲に合わせて粗くする（Cartographer の
    # multi-resolution 探索と同じ考え方——粗い段は「山のふもと」を見つける
    # のが仕事で、精度は後段が出す）。計算量は 範囲²×角度×点数 で決まるので、
    # 範囲を広げるときは必ず刻みも粗くすること
    trans = min(max(config.search_trans, config.search_trans_per_dist * best_d),
                config.search_trans_max)
    step = max(0.08, trans / 8.0)
    coarse = search(field, cur.hx, cur.hy, cur.pose, trans=trans,
                    rot=config.search_rot, trans_step=step,
                    rot_step=math.radians(3.0), sigma=max(0.06, step * 0.8),
                    max_points=64, ambiguity_dist=max(config.ambiguity_dist, step * 2))
    if coarse.score < config.min_score * 0.6:
        return None
    if coarse.second >= config.ambiguity_ratio * coarse.score:
        return None
    mid = search(field, cur.hx, cur.hy, coarse.pose, trans=step * 1.5,
                 rot=math.radians(4.0), trans_step=max(0.03, step / 4.0),
                 rot_step=math.radians(1.0), sigma=0.06, max_points=120)
    fine = search(field, cur.hx, cur.hy, mid.pose, trans=0.06, rot=math.radians(1.5),
                  trans_step=0.015, rot_step=math.radians(0.4), sigma=0.03, max_points=160)
    if fine.score < config.min_score:
        return None
    r = register(field, cur.hx, cur.hy, fine.pose, config=config.register)
    if r.used < config.min_used or r.inlier < config.min_inlier or r.rms > config.max_rms:
        return None

    gap = cur.path - src.path
    corr = math.hypot(r.pose.x - cur.pose.x, r.pose.y - cur.pose.y)
    if corr > config.max_correction + config.max_correction_per_m * gap:
        return None                     # ドリフトの見積もりに対して動かしすぎ
    delta = between(src.pose, r.pose)
    info = rotate_info(inflate_info(r.info, sigma_xy=config.sigma_xy,
                                    sigma_yaw=config.sigma_yaw), src.pose.yaw)
    return LoopCandidate(best_i, new_index, delta, fine.score, info, r.inlier, corr)
