"""経路上の障害物を横へ避ける一時経路（Frenet 座標の横ずらし）。

今のレーシングラインの各点を、障害物の手前から後ろまで**法線方向へ滑らかにずらす**。
点の数と添字は元の経路と同じなので、追従（`purepursuit.follow` の `hint`）・周回の
数え方・経路の乗り換えがそのまま使える。F1TENTH の ForzaETH race stack の
「相手の周りにスプラインを引いてレーシングラインへ戻す」のと同じ考え方。

    d(s) = D · smoothstep((s − s_in) / L)      進入（0 → D）
         = D                                    保持（車体が障害物の横を抜けるまで）
         = D · smoothstep((s_out − s) / L)      戻り（D → 0）

smoothstep は5次（端で1階・2階微分が0）で、足す曲率の最大は `5.77·|D|/L²`。
進入長 `L` はこれが `kappa_frac` の枠に収まる長さから始め、できた線の曲率を直接確かめる。

★ 地図に障害物を書き込んで `route.build_raceline` で引き直す方法は採らない。
中心線の初期値が道路グラフの骨格なので障害物の真上を通り、1回に動かせる量も
`centerline._MAX_SHIFT_M` で頭打ちになって抜け出せない。`_BodyCheck.tighten` は
両側が詰まると真ん中で諦めて実行不能なまま通す。Pi では数百ms〜1s かかる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .centerline import normals, normals_open, tangents_open
from .local_map import footprint_circles
from .obstacles import Obstacle
from .raceline import RaceLine, curvature

__all__ = ["AvoidConfig", "AvoidPlan", "plan_offset", "cap_speed", "forward_order",
           "clearance_along", "clearance_to_points", "blend_in"]

#: 5次 smoothstep の2階微分の最大 `max|f''| = 10/√3`
_SMOOTH_D2 = 10.0 / math.sqrt(3.0)
#: 進入長を伸ばして試す倍率（曲率が収まらなければ次を試す）
_LEN_STEPS = (1.0, 1.4, 2.0)
#: 元の経路がすでに `kappa_frac` の枠を超えている所で、さらに急にしてよい量（車の限界に対する割合）
_KAPPA_SLACK = 0.1


@dataclass(frozen=True)
class AvoidConfig:
    half_width: float
    front: float                           #: base_link から車体前端 [m]
    rear: float                            #: base_link から車体後端 [m]
    kappa_max: float                       #: 車の曲がれる限界の曲率 [1/m]
    footprint: tuple = ()
    margin: float = 0.10                   #: 障害物の縁から車体側面までの余裕 [m]
    clear: float = 0.05                    #: 車体外形と壁・障害物の距離の合格線 [m]
    len_min: float = 0.8                   #: 進入・戻りの長さの下限 [m]
    #: 曲がれる曲率のうち回避に使ってよい割合（止まってからはもっと使ってよい）
    kappa_frac: float = 0.6


class AvoidPlan(NamedTuple):
    path: RaceLine                         #: 元の経路と同じ点数・添字。速度は呼び出し側で付ける
    window: np.ndarray                     #: ずらした点の添字（進む順）
    offset: float                          #: 横ずらしの量 [m]（左が正）
    obstacle: Obstacle | None              #: 避けている障害物（経路へ戻るだけなら None）
    clearance: float                       #: 窓の中の車体外形の最小余裕 [m]


def forward_order(path: RaceLine, index: int) -> tuple[np.ndarray, np.ndarray]:
    """`index` から進む順の添字と、そこまでの弧長 [m]。閉じた経路は1周ぶん。"""
    n = len(path)
    if path.closed:
        order = (index + np.arange(n)) % n
    else:
        order = np.arange(max(0, index), n)
    xy = path.xy[order]
    seg = np.hypot(*np.diff(xy, axis=0).T) if len(xy) >= 2 else np.zeros(0)
    return order, np.concatenate([[0.0], np.cumsum(seg)])


def _smooth(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * t * (10.0 - 15.0 * t + 6.0 * t * t)


def _body_obstacle_clearance(xy: np.ndarray, circles: np.ndarray, rc: float,
                             obstacles: list[Obstacle]) -> float:
    """車体（円被覆）を `xy` の各点に接線向きで置いたときの、障害物との最小余裕 [m]。"""
    if not obstacles or len(xy) == 0:
        return math.inf
    t = tangents_open(xy)
    c, s = t[:, 0:1], t[:, 1:2]
    px = xy[:, 0:1] + circles[None, :, 0] * c - circles[None, :, 1] * s
    py = xy[:, 1:2] + circles[None, :, 0] * s + circles[None, :, 1] * c
    best = math.inf
    for o in obstacles:
        d = np.hypot(px - o.x, py - o.y) - rc - o.r
        best = min(best, float(d.min()))
    return best


def clearance_along(xy: np.ndarray, footprint, obstacles: list[Obstacle]) -> float:
    """経路の点列 `xy` に車体を置いたときの、障害物との最小余裕 [m]（負 = 当たる）。"""
    circles, rc = footprint_circles(footprint)
    return _body_obstacle_clearance(xy, circles, rc, obstacles)


def clearance_to_points(xy: np.ndarray, footprint, pts: np.ndarray) -> float:
    """経路の点列 `xy` に車体を置いたときの、点群 `pts` (N, 2) との最小余裕 [m]（負 = 当たる）。

    障害物の重心と半径（`Obstacle.x/y/r`）は見えている面だけから出すので、近づいて見る角度が
    変わると 10cm 近く動く。避けている途中の「まだ当たるか」は、今見えている点で直接測る。
    """
    if len(xy) == 0 or len(pts) == 0:
        return math.inf
    circles, rc = footprint_circles(footprint)
    t = tangents_open(xy)
    c, s = t[:, 0:1], t[:, 1:2]
    px = xy[:, 0:1] + circles[None, :, 0] * c - circles[None, :, 1] * s
    py = xy[:, 1:2] + circles[None, :, 0] * s + circles[None, :, 1] * c
    d = np.hypot(px[:, :, None] - pts[None, None, :, 0], py[:, :, None] - pts[None, None, :, 1])
    return float(d.min()) - rc


def plan_offset(path: RaceLine, index: int, target: Obstacle, obstacles: list[Obstacle],
                body, cfg: AvoidConfig, *, v_now: float,
                ahead_m: float = 8.0) -> AvoidPlan | None:
    """`target` を避ける一時経路。どちらの側でも車体が通らなければ `None`。

    :param index: 車の今いる経路の添字
    :param body: 壁との余裕を測る `raceline._BodyCheck`（`closed=False` で作ったもの）
    :param obstacles: 余裕を確かめる障害物すべて（`target` を含む）
    """
    order, s_fwd = forward_order(path, index)
    near = s_fwd <= ahead_m
    if not near.any():
        return None
    d2 = (path.xy[order[near], 0] - target.x) ** 2 + (path.xy[order[near], 1] - target.y) ** 2
    k = int(np.argmin(d2))
    j = int(order[k])
    s_o = float(s_fwd[k])
    nrm = normals(path.xy) if path.closed else normals_open(path.xy)
    d_o = float(np.dot([target.x - path.xy[j, 0], target.y - path.xy[j, 1]], nrm[j]))

    need = target.r + cfg.half_width + cfg.margin
    hold_a = s_o - target.r - cfg.front
    hold_b = s_o + target.r + cfg.rear
    circles, rc = footprint_circles(cfg.footprint or (
        (cfg.front, cfg.half_width), (cfg.front, -cfg.half_width),
        (-cfg.rear, -cfg.half_width), (-cfg.rear, cfg.half_width)))

    # 障害物の左を通る（+）と右を通る（−）。ずらす量の小さい側から試す
    cands = [(d_o + need, +1), (d_o - need, -1)]
    cands.sort(key=lambda c: abs(c[0]))
    kap = np.abs(path.kappa) if len(path.kappa) == len(path) else np.abs(curvature(path.xy, path.closed))
    best: AvoidPlan | None = None
    k_soft = cfg.kappa_frac * cfg.kappa_max
    for offset, _side in cands:
        if abs(offset) < 1e-3:
            continue
        # 進入長は「まっすぐな道なら足す曲率が `kappa_frac` の枠に収まる長さ」から始め、できた線の
        # 曲率を直接確かめて、だめなら長くして試す。
        # ★ 以前は「障害物の前後 3m の元の曲率の最大」＋「足す曲率の最大」が枠に収まるかで
        #   見積もっていた。ラインは曲がれる限界まで使って引くので、コーナーの多いコースでは
        #   どこでも枠が残らず、横へ避ける経路が1度も引けなかった（実機 2026-10-08・1周 12m、
        #   sim.bench の toyota も同じ）。元の曲率と足す曲率は同じ場所・同じ向きに重なるとは限らない
        base_len = max(cfg.len_min, 0.6 * max(v_now, 0.0),
                       math.sqrt(_SMOOTH_D2 * abs(offset) / max(k_soft, 1e-6)))
        for mul in _LEN_STEPS:
            length = base_len * mul
            s_in = hold_a - length
            if s_in < 0.0:
                # もう進入に使える距離が足りない（止まってから `kappa_frac` を上げて引き直す）
                break
            s_out = hold_b + length
            if path.closed and s_out >= 0.9 * path.length:
                break
            if not path.closed and s_out > s_fwd[-1]:
                break
            win = (s_fwd >= s_in) & (s_fwd <= s_out)
            sw = s_fwd[win]
            d = np.where(sw < hold_a, _smooth((sw - s_in) / length),
                         np.where(sw > hold_b, _smooth((s_out - sw) / length), 1.0)) * offset
            idx = order[win]
            xy = path.xy.copy()
            xy[idx] += nrm[idx] * d[:, None]
            # 窓の前後1点を足して、継ぎ目の向きも含めて車体を確かめる
            ext = order[max(0, int(np.argmax(win)) - 1):
                        min(len(order), int(np.nonzero(win)[0][-1]) + 2)]
            wxy = xy[ext]
            clr = min(float(body.clearance(wxy).min()) if body is not None else math.inf,
                      _body_obstacle_clearance(wxy, circles, rc, obstacles))
            if clr < cfg.clear:
                break                      # 長くしても横の余裕は変わらない
            kappa = curvature(xy, path.closed)
            # 枠（`kappa_frac`）を超えてよいのは元の経路がすでに超えている所だけで、そこでも
            # 元より `_KAPPA_SLACK` までしか急にしない。車の限界は、元が超えている所（最低速度で
            # 通す所）を除いて超えない。カーブの内側へずらすと元の曲率そのものが `κ/(1−κd)` に
            # 増えるので、ここは見積もりではなくできた線で見る
            k_new, k_old = np.abs(kappa[ext]), kap[ext]
            limit = np.minimum(np.maximum(k_soft, k_old + _KAPPA_SLACK * cfg.kappa_max),
                               np.maximum(cfg.kappa_max, k_old + 1e-6))
            if bool((k_new > limit).any()):
                continue
            rl = RaceLine(xy=xy, v=path.v.copy(), kappa=kappa, length=path.length,
                          closed=path.closed)
            best = AvoidPlan(rl, idx, offset, target, clr)
            break
        if best is not None:
            break
    return best


def blend_in(path: RaceLine, index: int, d0: float, length: float, body=None) -> AvoidPlan:
    """経路から横に `d0` [m] ずれた車が、`length` [m] かけて滑らかに経路へ戻る一時経路。

    Hybrid A* で抜けた直後は経路から数cm〜10cmずれている。Pure Pursuit でそのまま乗りに
    行くと舵を大きく切り、壁寄りを通る区間で車体の角が壁に当たった（sim.bench の toyota）。
    """
    order, s_fwd = forward_order(path, index)
    win = s_fwd <= length
    idx = order[win]
    nrm = normals(path.xy) if path.closed else normals_open(path.xy)
    xy = path.xy.copy()
    xy[idx] += nrm[idx] * (d0 * _smooth((length - s_fwd[win]) / length))[:, None]
    clr = float(body.clearance(xy[idx]).min()) if body is not None and len(idx) >= 2 else math.inf
    rl = RaceLine(xy=xy, v=path.v.copy(), kappa=curvature(xy, path.closed),
                  length=path.length, closed=path.closed)
    return AvoidPlan(rl, idx, d0, None, clr)


def cap_speed(path: RaceLine, window: np.ndarray, v_cap: float, a_brake: float,
              a_accel: float | None = None) -> RaceLine:
    """窓の中の速度を `v_cap` に抑え、窓の手前は `a_brake` で減速し切れるように下げる。

    `a_accel` を与えると、窓の後ろも `a_accel` で加速し切れる速度に抑える（与えないと窓の
    出口の1点で元の速度へ跳び、摩擦円で見込んだ加速度を超える指令になる）。
    """
    v = path.v.copy()
    v[window] = np.minimum(v[window], v_cap)
    n = len(path)
    first = int(window[0])
    vi = float(v[first])
    i = first
    for _ in range(n - len(window)):
        j = (i - 1) % n if path.closed else i - 1
        if j < 0:
            break
        ds = float(np.hypot(*(path.xy[i] - path.xy[j])))
        lim = math.sqrt(vi * vi + 2.0 * a_brake * ds)
        if v[j] <= lim:
            break
        v[j] = lim
        vi, i = lim, j
    if a_accel is not None:
        i = int(window[-1])
        vi = float(v[i])
        for _ in range(n - len(window)):
            j = (i + 1) % n if path.closed else i + 1
            if j >= n:
                break
            ds = float(np.hypot(*(path.xy[j] - path.xy[i])))
            lim = math.sqrt(vi * vi + 2.0 * a_accel * ds)
            if v[j] <= lim:
                break
            v[j] = lim
            vi, i = lim, j
    return RaceLine(**{**path._asdict(), "v": v})
