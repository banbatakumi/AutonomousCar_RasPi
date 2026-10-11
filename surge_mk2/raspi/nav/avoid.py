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

__all__ = ["AvoidConfig", "AvoidPlan", "plan_offset", "plan_elastic", "cap_speed", "speed_cap",
           "forward_order",
           "clearance_along", "clearance_to_points", "blend_in"]

#: 5次 smoothstep の2階微分の最大 `max|f''| = 10/√3`
_SMOOTH_D2 = 10.0 / math.sqrt(3.0)
#: 進入長を伸ばして試す倍率（曲率が収まらなければ次を試す）
_LEN_STEPS = (1.0, 1.4, 2.0)
#: 元の経路がすでに `kappa_frac` の枠を超えている所で、さらに急にしてよい量（車の限界に対する割合）
_KAPPA_SLACK = 0.1
#: 必要最小のずらし量に足して試す量 [m]（`plan_offset` の `score` を与えたとき）。大きくずらすと
#: 障害物との余裕が増えて速く通れるが、足す曲率も増える——どれが速いかは `score` が決める
_EXTRA_OFFSETS = (0.0, 0.08, 0.16)
#: 余裕が合格線 `clear` からこれだけ増えたら、避ける間の速度の抑えを外す [m]（`speed_cap`）
CAP_FREE_M = 0.15


def speed_cap(clearance: float, clear: float, v_tight: float, v_free: float) -> float:
    """避けている間の速度の上限 [m/s]。**余裕で決める**。

    余裕が合格線 `clear` ちょうどなら `v_tight`（設定「避ける間の速度」）、`CAP_FREE_M` 以上
    広ければ `v_free`（抑えない）、その間は線形。余裕によらず一律 `v_tight` にすると、
    道幅が十分あって 20cm 空けて通れる障害物でも落としすぎる（toyota で 1 周 +10〜20%）。
    曲率で決まる速度は別に `raceline.retime` が付ける。
    """
    if v_free <= v_tight:
        return v_free
    f = max(0.0, min(1.0, (clearance - clear) / CAP_FREE_M))
    return v_tight + f * (v_free - v_tight)


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
                ahead_m: float = 8.0, score=None,
                diag: list[str] | None = None,
                room: tuple[np.ndarray, np.ndarray] | None = None) -> AvoidPlan | None:
    """`target` を避ける一時経路。どちらの側でも車体が通らなければ `None`。

    :param index: 車の今いる経路の添字
    :param body: 壁との余裕を測る `raceline._BodyCheck`（`closed=False` で作ったもの）
    :param obstacles: 余裕を確かめる障害物すべて（`target` を含む）
    :param score: `score(plan) -> float`（小さいほど良い。ふつうは所要時間）。与えると、通れる
        避け方（左右 × ずらし量 `_EXTRA_OFFSETS`）を全部作って**いちばん良いもの**を返す。
        `None` なら、ずらし量の小さい側から試して最初に通ったもの
    :param diag: リストを渡すと、試した避け方ごとに「なぜ使えなかったか」を足す（表示・調査用）
    :param room: 経路の各点から左・右へずらせる量 [m]（`(左, 右)`、どちらも 0 以上。経路と同じ長さ）。
        与えると、横ずらしを**その場所で通れる範囲に切り詰める**（下記）

    ## ★ 道幅に合わせて切り詰める（`room`）

    ずらしの形は「進入 → 保持 → 戻り」で、進入と戻りは速いほど長い（3m/s で 1.8m ずつ）。
    ラップ時間で仕上げたラインはコーナーの頂点ごとに壁から `line_margin` ぎりぎりを通るので、
    進入か戻りの途中にその頂点があると、そこで壁に入って**どの避け方も使えなかった**
    （道幅 1.45m の toyota でも毎回止まって Hybrid A* に落ちた）。各点のずらしを
    通れる範囲に切り詰めてから短い移動平均でならす——頂点では元のラインのまま、障害物の横では
    必要なだけずれる。できた線は車体外形・障害物・曲率で確かめる（切り詰めは候補を
    増やすだけで、合格の基準は緩めない）。
    """
    def note(offset: float, why: str) -> None:
        if diag is not None:
            diag.append(f"{'左' if offset > 0 else '右'}{abs(offset) * 100:.0f}cm:{why}")

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
    extras = _EXTRA_OFFSETS if score is not None else (0.0,)
    cands = [(d_o + side * (need + e), side) for side in (+1, -1) for e in extras]
    cands.sort(key=lambda c: abs(c[0]))
    best_score = math.inf
    kap = np.abs(path.kappa) if len(path.kappa) == len(path) else np.abs(curvature(path.xy, path.closed))
    best: AvoidPlan | None = None
    k_soft = cfg.kappa_frac * cfg.kappa_max
    for offset, _side in cands:
        if abs(offset) < 1e-3:
            continue
        # 進入長は「まっすぐな道なら足す曲率が `kappa_frac` の枠に収まる長さ」から始め、できた線の
        # 曲率を直接確かめて、だめなら長くして試す。
        # ★ 「障害物の前後 3m の元の曲率の最大」＋「足す曲率の最大」が枠に収まるかの見積もりは使わない。
        #   ラインは曲がれる限界まで使って引くので、コーナーの多いコースではどこでも枠が残らず、
        #   横へ避ける経路が1度も引けない。元の曲率と足す曲率は同じ場所・同じ向きに重なるとは限らない
        base_len = max(cfg.len_min, 0.6 * max(v_now, 0.0),
                       math.sqrt(_SMOOTH_D2 * abs(offset) / max(k_soft, 1e-6)))
        for mul in _LEN_STEPS:
            length = base_len * mul
            s_in = hold_a - length
            if s_in < 0.0:
                # もう進入に使える距離が足りない（止まってから `kappa_frac` を上げて引き直す）
                note(offset, f"進入{length:.1f}mに距離不足({hold_a:.1f}m)")
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
            if room is not None:
                d = np.clip(d, -room[1][idx], room[0][idx])
                if len(d) >= 5:            # 切り詰めの角をならす（両端は 0 のまま）
                    pad = np.concatenate([[0.0, 0.0], d, [0.0, 0.0]])
                    d = np.convolve(pad, np.full(5, 0.2), mode="valid")
                    # ならすと頂点の両隣から少し染み出すので、もう一度切り詰める（連続なまま）
                    d = np.clip(d, -room[1][idx], room[0][idx])
            xy = path.xy.copy()
            xy[idx] += nrm[idx] * d[:, None]
            # 窓の前後1点を足して、継ぎ目の向きも含めて車体を確かめる
            ext = order[max(0, int(np.argmax(win)) - 1):
                        min(len(order), int(np.nonzero(win)[0][-1]) + 2)]
            wxy = xy[ext]
            clr = _body_obstacle_clearance(wxy, circles, rc, obstacles)
            if body is not None:
                # ★ 壁との余裕は「合格線」か「元の経路のその点の余裕」の小さい方を満たせばよい。
                #   ラインは壁から `line_margin` ぎりぎり（許容込みで約4cm）を通る所があり、
                #   合格線（5cm）を一律に求めると、まだほとんどずらしていない進入の入口で
                #   落ちて、どの避け方も使えなかった（sim.bench の toyota）
                wall = body.clearance(wxy)
                base = body.clearance(path.xy[ext])
                if bool((wall < np.minimum(cfg.clear, base - 0.02)).any()):
                    note(offset, f"壁まで{float(wall.min()) * 100:.0f}cm")
                    break                  # 長くしても横の余裕は変わらない
                moved = np.abs(np.einsum("ij,ij->i", wxy - path.xy[ext], nrm[ext])) > 0.03
                if moved.any():
                    clr = min(clr, float(wall[moved].min()))
            if clr < cfg.clear:
                note(offset, f"余裕{clr * 100:.0f}cm")
                break
            kappa = curvature(xy, path.closed)
            # 枠（`kappa_frac`）を超えてよいのは元の経路がすでに超えている所だけで、そこでも
            # 元より `_KAPPA_SLACK` までしか急にしない。車の限界は、元が超えている所（最低速度で
            # 通す所）を除いて超えない。カーブの内側へずらすと元の曲率そのものが `κ/(1−κd)` に
            # 増えるので、ここは見積もりではなくできた線で見る
            k_new, k_old = np.abs(kappa[ext]), kap[ext]
            limit = np.minimum(np.maximum(k_soft, k_old + _KAPPA_SLACK * cfg.kappa_max),
                               np.maximum(cfg.kappa_max, k_old + 1e-6))
            if bool((k_new > limit).any()):
                note(offset, f"曲率(進入{length:.1f}m)")
                continue
            rl = RaceLine(xy=xy, v=path.v.copy(), kappa=kappa, length=path.length,
                          closed=path.closed)
            plan = AvoidPlan(rl, idx, offset, target, clr)
            if score is None:
                return plan
            sc = float(score(plan))
            if sc < best_score:
                best, best_score = plan, sc
            break                          # この量では短い進入がいちばん速い。次の量へ
    return best


#: `plan_elastic` が試すこぶの幅（標準偏差、点の数）と、高さの初期値・下限 [m]
_ELASTIC_WIDTHS = (12, 6, 3)
_ELASTIC_AMP0 = 0.06
_ELASTIC_AMP_MIN = 0.006
#: 計算している間に車が進むぶん、ずらし始めを先へ置く時間 [s]
ELASTIC_LEAD_S = 0.35


def plan_elastic(path: RaceLine, index: int, target: Obstacle, obstacles: list[Obstacle],
                 body, cfg: AvoidConfig, *, v_now: float, room, speed: dict,
                 v_tight: float, ahead_m: float = 8.0, max_evals: int = 500) -> AvoidPlan | None:
    """`target` を避ける一時経路を、**壁と障害物を制約にして窓の中の線を引き直して**作る。

    ## なぜ `plan_offset` だけでは足りないか

    `plan_offset` のずらしは「進入 → 保持 → 戻り」の決まった形で、変えられるのは量と長さだけ。
    ラップ時間で仕上げたラインはコーナーの頂点ごとに壁ぎりぎりを通るので、障害物がコーナーの
    近くにあると、進入か戻りのどこかで壁・曲率に掛かってどの形も通らない（toyota で道幅 1.45m
    あっても「止まる → Hybrid A*」に落ち、1周 +6 秒）。ここでは形を決め打ちにせず、
    `plan_offset` の形を出発点（左右 1 つずつ）にして、ラインの仕上げ（`raceline.refine_time`）と
    同じやり方——こぶを足す・たるみを取る——で窓の中の点を動かす。

    比べる順（小さいほど良い）は `(制約を破った量, 曲率の超過, 窓を抜ける時間)`。**破った量を
    連続量にしてある**ので、出発点が壁・障害物に掛かっていても、まず破った量を 0 にする向きへ
    動く（`refine_time` の 0/1 の印では、掛かったまま時間だけを縮めに行ってしまう）。
    制約と速度の付け方は `plan_offset`・`speed_cap` と同じ。窓の両端 3 点は動かさない。

    重い（評価 1 回 約 0.15ms × `max_evals`）ので**別プロセスで回す**（`slam2d_raceline`）。
    引数と戻り値は pickle できるものだけ。計算の間に車が進むので、ずらし始めは
    `ELASTIC_LEAD_S` 秒ぶん先に置く。

    :param speed: `raceline.speed_profile` へ渡す速度の設定（`v_max`・`a_lat` など）
    :param v_tight: 余裕が合格線ちょうどのときの速度 [m/s]（`speed_cap`）
    """
    from .raceline import _travel_time, speed_kappa, speed_profile

    if not path.closed or body is None:
        return None
    order, s_fwd = forward_order(path, index)
    near = s_fwd <= ahead_m
    d2 = (path.xy[order[near], 0] - target.x) ** 2 + (path.xy[order[near], 1] - target.y) ** 2
    k = int(np.argmin(d2))
    j = int(order[k])
    s_o = float(s_fwd[k])
    nrm = normals(path.xy)
    d_o = float(np.dot([target.x - path.xy[j, 0], target.y - path.xy[j, 1]], nrm[j]))
    need = target.r + cfg.half_width + cfg.margin
    hold_a = s_o - target.r - cfg.front
    hold_b = s_o + target.r + cfg.rear
    k_soft = cfg.kappa_frac * cfg.kappa_max
    length = 2.0 * max(cfg.len_min, 0.6 * max(v_now, 0.0),
                       math.sqrt(_SMOOTH_D2 * need / max(k_soft, 1e-6)))
    s_in = max(hold_a - length, max(v_now, 0.0) * ELASTIC_LEAD_S)
    s_out = hold_b + length
    if hold_a - s_in < 0.5 * cfg.len_min or s_out >= 0.9 * path.length:
        return None
    pos = np.nonzero((s_fwd >= s_in) & (s_fwd <= s_out))[0]
    if len(pos) < 12:
        return None
    lo_i, hi_i = int(pos[0]), int(pos[-1])
    ext = order[max(0, lo_i - 2):min(len(order), hi_i + 3)]      # 前後 2 点（動かさない）を含む
    off = lo_i - max(0, lo_i - 2)                                 # `ext` の中での窓の始まり
    m = len(pos)
    sw = s_fwd[pos]
    base = path.xy[ext].copy()
    nr = nrm[ext]
    # 窓の両端 3 点は元の経路のまま（継ぎ目で折れない）
    taper = np.zeros(len(ext))
    taper[off:off + m] = np.clip(np.minimum(np.arange(m), np.arange(m)[::-1]) / 3.0 - 1.0 / 3.0,
                                 0.0, 1.0)
    circles, rc = footprint_circles(cfg.footprint or (
        (cfg.front, cfg.half_width), (cfg.front, -cfg.half_width),
        (-cfg.rear, -cfg.half_width), (-cfg.rear, cfg.half_width)))
    base_wall = body.clearance(base)
    wall_need = np.minimum(cfg.clear, base_wall - 0.02)
    k_old = np.abs(curvature(path.xy, True))[ext]
    k_lim = np.minimum(np.maximum(k_soft, k_old + _KAPPA_SLACK * cfg.kappa_max),
                       np.maximum(cfg.kappa_max, k_old + 1e-6))
    v_in = min(float(path.v[ext[0]]), max(v_now, 0.3) + 0.3)
    v_out = float(path.v[ext[-1]])
    v_free = float(speed["v_max"])
    idx_w = np.arange(len(ext))

    def measure(q: np.ndarray) -> tuple[float, float, float, float]:
        """`(破った量, 曲率の超過, 時間, 余裕)`。"""
        wall = body.clearance(q)
        obs = _body_obstacle_clearance(q, circles, rc, obstacles)
        viol = float(np.maximum(wall_need - wall, 0.0).max()) + max(0.0, cfg.clear - obs)
        kap = np.abs(curvature(q, False))
        over = float(np.maximum(kap - k_lim, 0.0)[1:-1].max())
        moved = np.abs(np.einsum("ij,ij->i", q - base, nr)) > 0.03
        clr = min(obs, float(wall[moved].min()) if moved.any() else math.inf)
        if viol > 0.0 or over > 0.0:
            return round(viol, 3), round(over, 3), math.inf, clr
        sp = dict(speed, v_max=min(v_free, speed_cap(clr, cfg.clear, v_tight, v_free)))
        v = speed_profile(q, speed_kappa(q, False), closed=False, v_start=v_in, v_end=v_out,
                          **{k_: sp[k_] for k_ in ("v_max", "v_min", "a_lat", "a_accel", "a_brake",
                                                   "a_grip") if k_ in sp})
        return 0.0, 0.0, _travel_time(q, v, False), clr

    def search(q: np.ndarray, budget: int):
        best = measure(q)
        evals = 1
        for w in _ELASTIC_WIDTHS:
            amp = _ELASTIC_AMP0
            while amp >= _ELASTIC_AMP_MIN and evals < budget:
                improved = False
                for c in range(off + 1, off + m - 1, w):
                    g = (np.exp(-0.5 * ((idx_w - c) / w) ** 2) * taper)[:, None]
                    kk = 2 * w + 1
                    pad = np.vstack([np.repeat(q[:1], w, axis=0), q, np.repeat(q[-1:], w, axis=0)])
                    cs = np.cumsum(np.vstack([np.zeros((1, 2)), pad]), axis=0)
                    mean = (cs[kk:] - cs[:-kk]) / kk
                    for cand in (q + (amp / _ELASTIC_AMP0) * g * (mean - q),
                                 q + amp * g * nr, q - amp * g * nr):
                        cq = measure(cand)
                        evals += 1
                        if cq[:3] < best[:3]:
                            q, best, improved = cand, cq, True
                            break
                    if evals >= budget:
                        break
                if not improved:
                    amp *= 0.5
        return q, best

    results = []
    for side in (+1.0, -1.0):
        offset = d_o + side * need
        seg_in = max(hold_a - s_in, 1e-3)
        seg_out = max(s_out - hold_b, 1e-3)
        d = np.where(sw < hold_a, _smooth((sw - s_in) / seg_in),
                     np.where(sw > hold_b, _smooth((s_out - sw) / seg_out), 1.0)) * offset
        if room is not None:
            d = np.clip(d, -room[1][order[pos]], room[0][order[pos]])
        seed = base.copy()
        seed[off:off + m] += nr[off:off + m] * (d * taper[off:off + m])[:, None]
        q, best = search(seed, max_evals // 2)
        if best[0] == 0.0 and best[1] == 0.0:
            results.append((best[2], len(results), q, best[3]))
    if not results:
        return None
    _, _, q, clr = min(results)
    xy = path.xy.copy()
    xy[ext] = q
    shift = np.einsum("ij,ij->i", q - base, nr)
    peak = float(shift[int(np.argmax(np.abs(shift)))])
    rl = RaceLine(xy=xy, v=path.v.copy(), kappa=curvature(xy, True), length=path.length,
                  closed=True)
    return AvoidPlan(rl, order[pos], peak, target, clr)


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
