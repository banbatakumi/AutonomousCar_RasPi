"""障害物・他車を追い抜く一時経路（Frenet 格子 × 時刻つきの当たり判定）。

レーシングラインに対する横ずれ `d(s)` の候補をまとめて作り、障害物の**その時刻の位置**と
重ならないものの中から、窓を抜ける時間がいちばん短いものを選ぶ。Werling ら (2010) の
Frenet 格子、TUM の graph-based local planner (Stahl 2019)、ForzaETH の Predictive Spliner
(Baumann 2024) と同じ考え方。

    d(s) = 5次多項式  今の状態 (d0, d0', d0'') → (dT, 0, 0)        0 ≤ s < L1   （入る）
         = dT                                                     L1 ≤ s < S2  （並ぶ）
         = dT · (1 − smoothstep((s − S2) / L3))                   S2 ≤ s       （戻る）

## `avoid.plan_offset` との違い

- **今の横ずれから出発する。** 避けている途中・戻る途中でも毎周期引き直せる（前の計画の上に
  居れば前の計画の状態から出発するので、同じ候補を選べば形は変わらない）
- **時刻で判定する。** 候補の速度（元の経路の速度・曲率で決まる速度・今の速度から加減速できる
  速度）から各点を通る時刻を出し、障害物をその時刻の位置（`s + u·t`、`d + w·t`）に置いて重なりを
  見る。静止物は `u = w = 0` で同じ式。同方向の遅い車は「抜く間に相手が進む」ぶん並ぶ区間が伸び、
  抜く前に追いつかない候補（相手が先へ逃げる）は塞がれない
- **形を決め打ちにしない。** 入る長さ・並ぶ長さ・戻る長さを格子で振り、曲率と道幅（`room`）で
  捨てる。曲がれる限界まで使う短い入り方も候補に入る（どれを選ぶかは時間で決まる）
- **避ける間に速度を抑えない。** 速度は曲率と摩擦だけで決まる（安全は横・前後の余裕で持つ）

障害物は Frenet 上の矩形（経路に沿った範囲 × 横の範囲）で扱う。LiDAR に見えるのは手前の面だけ
なので、奥（経路の先の側）へ見えない厚みを足す: 動く物は車の長さ `opp_length`、静止物は見えている幅。
動く物は時間とともに位置の不確かさが増えるとして、範囲を `u_sigma·t`・`w_sigma·t` ずつ広げる。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

import numpy as np

from .avoid import AvoidPlan
from .centerline import normals
from .obstacles import MOVING_MIN_SPEED, Obstacle
from .raceline import RaceLine, curvature

__all__ = ["OvertakeConfig", "Track", "Cand", "OvertakeResult", "frenet_state", "frenet_tracks",
           "plan", "eval_cand", "build"]

#: 格子の点の間隔 [m]（経路の点 0.1m の 2〜3 つおき）。速いときは粗くする（車体の長さ 0.37m より
#: 細かければ、静止物を飛び越す判定にはならない）
_STEP = 0.2
_STEP_FAST = 0.3
#: 見る先の長さ: `max(_H_MIN, v·_H_T)`、上限 `_H_MAX` [m]
_H_MIN = 6.0
_H_T = 2.5
_H_MAX = 12.0
#: 障害物を数える経路からの横の範囲 [m]（これより外は道の外か別の道）
_LAT_MAX = 1.5
#: 戻る区間の始まりを振る間隔 [m]
_S2_STEP = 0.4
#: 前の計画の上に居るとみなす横のずれ [m]（超えたら今の姿勢から出発する）
_ON_PLAN_M = 0.08
#: 左右を入れ替える候補に足す時間 [s]（判断のばたつきを抑える）
_SWITCH_COST = 0.15
#: 見る先の終わりで経路から外れている量に掛ける時間 [s/m]（戻れるなら戻る）
_END_COST = 0.4
#: 横ずれの大きさに掛ける時間 [s/m]（同じ時間なら小さくずれる）
_LAT_COST = 0.05
#: 前の計画と同じ形の候補を選び直すときの値引き [s]
_KEEP_BONUS = 0.05
#: 経路の上に居るとみなす横ずれ [m]（ここから外れていなければ「戻る」計画は要らない）
_ON_LINE_M = 0.02
#: 元の線が曲がれる限界を超えている所で、さらに急にしてよい量（限界に対する割合）
_KAPPA_SLACK = 0.1
#: 障害物を経路の枝へ射影するとき、距離の谷から前後に見る範囲 [m]（`frenet_tracks`）
_BRANCH_M = 0.8
#: 予測を伸ばす時間の上限 [s]
_T_PRED_MAX = 3.0


@dataclass(frozen=True)
class OvertakeConfig:
    half_width: float                      #: 車体の半幅 [m]
    front: float                           #: base_link から車体前端 [m]
    rear: float                            #: base_link から車体後端 [m]
    kappa_max: float                       #: 経路に許す曲率の上限 [1/m]
    a_brake: float                         #: 減速度 [m/s²]
    a_accel: float                         #: 加速度 [m/s²]
    margin_lat: float = 0.10               #: 障害物の横の余裕 [m]
    margin_long: float = 0.10              #: 障害物の前後の余裕 [m]
    #: 動く物（他車）の長さ [m]。後ろ面しか見えないので、奥へこれだけ足す
    opp_length: float = 0.40
    #: 動く物の位置の不確かさが時間とともに増える速さ [m/s]（前後・横）と、その上限 [m]。
    #: 横は道の中で走っている車なので大きくは動かない（上限を小さく）
    u_sigma: float = 0.15
    w_sigma: float = 0.04
    u_grow_max: float = 0.40
    w_grow_max: float = 0.06
    #: 抜いた動く相手の前へ戻るときに空ける時間 [s]（相手の速さ × これを前後の余裕に足す）。
    #: 曲がりながら走る相手の速度は直線の回帰で弦の分だけ遅く出る（ヘアピンで 1.5 → 約 1.0m/s）ので、
    #: 抜いた直後に前へ戻ると後ろから当たられた（sim.bench --mover）
    cutin_headway: float = 0.3


class Track(NamedTuple):
    """Frenet 上の障害物（車の今の位置を s=0 とした経路に沿った範囲と、経路からの横の範囲）。"""

    s_lo: float
    s_hi: float
    d_lo: float
    d_hi: float
    u: float                               #: 経路に沿った速度 [m/s]（負 = 逆走）
    w: float                               #: 経路から横へ離れる速度 [m/s]（左が正）
    moving: bool
    src: Obstacle


class Cand(NamedTuple):
    """1 つの候補（`s` は出発点 `s0` からの距離）。`s0` は経路の始点からの弧長 [m]。"""

    s0: float
    d0: float
    d1: float
    d2: float
    dT: float
    L1: float
    S2: float                              #: 戻り始め（`inf` = 見る先の中では戻らない）
    L3: float

    def at(self, x: float) -> tuple[float, float, float]:
        d, d1, d2 = _profile(np.array([x]), *(np.array([v]) for v in self[1:]))
        return float(d[0, 0]), float(d1[0, 0]), float(d2[0, 0])


class OvertakeResult(NamedTuple):
    #: "clear" = 経路のまま通れる / "avoid" = `plan` に乗る / "blocked" = 通れる候補が無い
    status: str
    plan: AvoidPlan | None
    cand: Cand | None
    #: 使える候補を良い順に（`cand` が先頭）。呼び出し側が直交座標で確かめ、落ちたら次を使う
    ranked: list
    #: 経路のままだと当たる障害物のうち最初のもの（"blocked" のとき、付いていく相手）
    lead: Track | None
    n_cands: int
    why: str
    #: 選んだのが前の計画をそのまま続ける候補か（形は前と同じ。作り直さなくてよい）
    kept: bool = False


def _quintic(d0, d1, d2, dT, L):
    """(d0, d1, d2) → (dT, 0, 0) を長さ L で結ぶ5次多項式の 3〜5 次の係数（配列）。"""
    h0 = dT - (d0 + d1 * L + 0.5 * d2 * L * L)
    h1 = -(d1 + d2 * L)
    h2 = -d2
    c3 = (10.0 * h0 - 4.0 * h1 * L + 0.5 * h2 * L * L) / L ** 3
    c4 = (-15.0 * h0 + 7.0 * h1 * L - h2 * L * L) / L ** 4
    c5 = (6.0 * h0 - 3.0 * h1 * L + 0.5 * h2 * L * L) / L ** 5
    return c3, c4, c5


def _profile(x, d0, d1, d2, dT, L1, S2, L3):
    """候補 (C,) を点 x (K,) で評価した `(d, d', d'')`（どれも (C, K)）。"""
    x = np.asarray(x, dtype=float)[None, :]
    d0, d1, d2, dT, L1, S2, L3 = (np.asarray(a, dtype=float)[:, None] for a in (d0, d1, d2, dT, L1, S2, L3))
    c3, c4, c5 = _quintic(d0, d1, d2, dT, L1)
    h2 = 0.5 * d2
    # 入る区間（ホーナー法）。L1 より先は端の値 (dT, 0, 0) になるよう、x を L1 で止める
    xe = np.minimum(x, L1)
    d = d0 + xe * (d1 + xe * (h2 + xe * (c3 + xe * (c4 + xe * c5))))
    dd = d1 + xe * (d2 + xe * (3 * c3 + xe * (4 * c4 + xe * 5 * c5)))
    ddd = d2 + xe * (6 * c3 + xe * (12 * c4 + xe * 20 * c5))
    # 戻る区間: S2 から L3 で dT → 0。入る区間の先（端の値 dT, 0, 0）を上書きする
    with np.errstate(invalid="ignore"):
        u = (x - S2) / L3
    back = (u > 0.0) & (x >= L1)
    if back.any():
        u = np.clip(np.nan_to_num(u, nan=0.0), 0.0, 1.0)
        sm = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
        sm1 = 30.0 * u * u * (1.0 - u) ** 2 / L3
        sm2 = u * (60.0 + u * (-180.0 + 120.0 * u)) / (L3 * L3)
        d = np.where(back, dT * (1.0 - sm), d)
        dd = np.where(back, -dT * sm1, dd)
        ddd = np.where(back, -dT * sm2, ddd)
    return d, dd, ddd


def frenet_state(path: RaceLine, index: int, x: float, y: float, yaw: float):
    """車の (経路の始点からの弧長 s, 横ずれ d, d' = tan(向きのずれ))。"""
    n = len(path)
    j, j2 = index % n, (index + 1) % n
    t = path.xy[j2] - path.xy[j]
    seg = max(float(np.hypot(*t)), 1e-9)
    t = t / seg
    dx, dy = x - path.xy[j, 0], y - path.xy[j, 1]
    along = dx * t[0] + dy * t[1]
    d = -dx * t[1] + dy * t[0]
    psi = (yaw - math.atan2(t[1], t[0]) + math.pi) % (2 * math.pi) - math.pi
    return along, d, math.tan(max(-1.2, min(1.2, psi)))


def frenet_tracks(path: RaceLine, order: np.ndarray, s_rel: np.ndarray, nrm: np.ndarray,
                  obstacles: list[Obstacle], cfg: OvertakeConfig) -> list[Track]:
    """障害物を Frenet の矩形にする。`order`・`s_rel` は車の少し後ろから見る先までの経路の添字と
    車からの弧長（負 = 後ろ）。経路から遠い物・窓の外の物は落とす。

    ★ 経路が折り返している所（ヘアピン）では、1 つの障害物が窓の中の 2 つの枝のどちらにも近い。
    いちばん近い点へ射影すると反対側の枝に乗り、線の上の少し先に居る相手が「足元の右 0.9m」に
    見えて、抜いた直後に戻った線で当たった（sim.bench --mover）。重心から経路までの距離の谷ごとに、
    その枝の近く（±`_BRANCH_M`）だけで射影して別々の矩形にする（どちらの枝の上にも居るものとして避ける）。
    """
    out: list[Track] = []
    xy = path.xy[order]
    tn = np.column_stack([nrm[order, 1], -nrm[order, 0]])           # 接線
    K = len(order)
    step = max(float(np.median(np.diff(s_rel))) if K > 1 else 0.1, 1e-3)
    half = max(2, int(round(_BRANCH_M / step)))
    for o in obstacles:
        pts = o.pts if o.pts is not None and len(o.pts) else None
        if pts is None:
            a = np.linspace(0.0, 2 * math.pi, 9)[:-1]
            pts = np.column_stack([o.x + o.r * np.cos(a), o.y + o.r * np.sin(a)])
        cx, cy = float(pts[:, 0].mean()), float(pts[:, 1].mean())
        dc = np.hypot(xy[:, 0] - cx, xy[:, 1] - cy)
        # 距離の谷（両隣以下、端は除く）で、経路から `_LAT_MAX` 以内のもの
        mins = [k for k in range(1, K - 1)
                if dc[k] <= dc[k - 1] and dc[k] <= dc[k + 1] and dc[k] < _LAT_MAX + 0.3]
        taken: list[int] = []
        for m in sorted(mins, key=lambda k: dc[k]):
            if any(abs(m - q) <= half for q in taken):
                continue
            taken.append(m)
            lo, hi = max(0, m - half), min(K, m + half + 1)
            sub = np.arange(lo, hi)
            d2 = ((pts[:, None, :] - xy[None, sub, :]) ** 2).sum(axis=2)
            k = sub[np.argmin(d2, axis=1)]
            if np.all((k == 0) | (k == K - 1)):
                continue
            rel = pts - xy[k]
            s = s_rel[k] + (rel * tn[k]).sum(axis=1)
            d = (rel * nrm[order[k]]).sum(axis=1)
            if float(np.abs(d).min()) > _LAT_MAX:
                continue
            out.append(_track(o, s, d, tn[m], nrm[order[m]], cfg))
    return out


def _track(o: Obstacle, s: np.ndarray, d: np.ndarray, t, n, cfg: OvertakeConfig) -> Track:
    """射影した点の Frenet 座標 `(s, d)` と、その場所の接線 `t`・法線 `n` から矩形を作る。"""
    s_lo, s_hi = float(s.min()), float(s.max())
    d_lo, d_hi = float(d.min()), float(d.max())
    u = float(o.vx * t[0] + o.vy * t[1])
    w = float(o.vx * n[0] + o.vy * n[1])
    moving = o.speed >= MOVING_MIN_SPEED
    if not moving:
        u = w = 0.0
    elif abs(w) < 0.3:
        w = 0.0                            # 道の中を走る車の横の速度は小さい。推定の揺れを落とす
    # 見えない奥行き（車から遠い側。前の物は経路の先、後ろの物はさらに後ろ）。
    # 動く物は車の長さ、静止物は見えている幅ぶん
    width = d_hi - d_lo
    depth = cfg.opp_length if moving else min(max(width, 0.05), 0.30)
    if 0.5 * (s_lo + s_hi) >= 0.0:
        s_hi = max(s_hi, s_lo + depth)
    else:
        s_lo = min(s_lo, s_hi - depth)
    # 点が少ないと横幅を小さく見積もる（遠くの物）。最低 10cm にする
    if width < 0.10:
        c = 0.5 * (d_lo + d_hi)
        d_lo, d_hi = c - 0.05, c + 0.05
    return Track(s_lo, s_hi, d_lo, d_hi, u, w, moving, o)


def _speeds(kap_c, v_cap, a_lat, ds, v_now, cfg: OvertakeConfig):
    """候補 (C, K) の速度: 曲率で決まる上限 → 後ろから減速 → 今の速度から加速。
    今の速度から減速し切れない候補は `feasible=False`。"""
    with np.errstate(divide="ignore"):
        v = np.minimum(v_cap[None, :], np.sqrt(a_lat[None, :] / np.maximum(np.abs(kap_c), 1e-6)))
    K = v.shape[1]
    for k in range(K - 2, -1, -1):
        v[:, k] = np.minimum(v[:, k], np.sqrt(v[:, k + 1] ** 2 + 2.0 * cfg.a_brake * ds[k]))
    ok = v[:, 0] >= v_now - 0.3
    v[:, 0] = np.minimum(v[:, 0], max(v_now, 0.0))
    for k in range(K - 1):
        v[:, k + 1] = np.minimum(v[:, k + 1], np.sqrt(v[:, k] ** 2 + 2.0 * cfg.a_accel * ds[k]))
    return np.maximum(v, 0.05), ok


def _collide(x, d, dd, t, tracks: list[Track], cfg: OvertakeConfig, start_ok: np.ndarray,
             d_start: float = 0.0):
    """候補ごとに当たる障害物があるか (C,) と、当たる最初の障害物の番号 (C,)（無ければ -1）。"""
    C = d.shape[0]
    hit = np.zeros(C, dtype=bool)
    first = np.full(C, -1)
    #: 前後で重なる間の、横の間隔の最小 [m]（余裕を引いた後。負 = 重なる）。通れる候補が無いときに
    #: 「いちばん離れていられる」候補を選ぶのに使う
    gap = np.full(C, np.inf)
    swing = cfg.half_width + cfg.front * np.abs(dd)              # 向きのずれで角が横へ出るぶん
    # 点と点の間（0.2〜0.3m）でも横ずれは変わる。前後の点との間で取りうる範囲を車体の横の範囲にする
    d_up = np.maximum(d, np.maximum(np.roll(d, 1, axis=1), np.roll(d, -1, axis=1)))
    d_dn = np.minimum(d, np.minimum(np.roll(d, 1, axis=1), np.roll(d, -1, axis=1)))
    d_up[:, 0], d_dn[:, 0] = np.maximum(d[:, 0], d[:, 1]), np.minimum(d[:, 0], d[:, 1])
    d_up[:, -1], d_dn[:, -1] = np.maximum(d[:, -1], d[:, -2]), np.minimum(d[:, -1], d[:, -2])
    car_lo, car_hi = x[None, :] - cfg.rear, x[None, :] + cfg.front
    tt = np.minimum(t, _T_PRED_MAX)
    for j, tr in enumerate(tracks):
        grow_s = np.minimum(cfg.u_sigma * tt, cfg.u_grow_max) if tr.moving else 0.0
        grow_d = np.minimum(cfg.w_sigma * tt, cfg.w_grow_max) if tr.moving else 0.0
        m_lat = cfg.margin_lat
        if tr.s_lo - cfg.margin_long < cfg.front and tr.s_hi + cfg.margin_long > -cfg.rear:
            # ★ もう並んでいる相手。横の余裕を今の間隔まで縮める（今より近づかなければよい）。
            #   縮めないと、余裕の内側に並んだ時点で全候補が初手から重なり「通れない」になる
            gap0 = max(tr.d_lo - (d_start + cfg.half_width), (d_start - cfg.half_width) - tr.d_hi)
            if gap0 >= 0.0:
                m_lat = min(m_lat, max(gap0 - 0.01, 0.0))
                grow_d = np.minimum(grow_d, max(gap0 - 0.01 - m_lat, 0.0)) if tr.moving else 0.0
        s_lo = tr.s_lo + tr.u * tt - grow_s - cfg.margin_long
        s_hi = tr.s_hi + tr.u * tt + grow_s + cfg.margin_long + cfg.cutin_headway * max(tr.u, 0.0)
        d_lo = tr.d_lo + tr.w * tt - grow_d - m_lat
        d_hi = tr.d_hi + tr.w * tt + grow_d + m_lat
        s_ov = (car_hi > s_lo) & (car_lo < s_hi) & start_ok
        lat = np.maximum(d_lo - (d_up + swing), (d_dn - swing) - d_hi)
        gap = np.minimum(gap, np.where(s_ov, lat, np.inf).min(axis=1))
        over = s_ov & (lat < 0.0)
        h = over.any(axis=1)
        first = np.where(h & (first < 0), j, first)
        hit |= h
    return hit, first, gap


def eval_cand(cands: np.ndarray, x, kr, v_cap, a_lat, ds, room_l, room_r, v_now,
              tracks, cfg: OvertakeConfig, d_start: float):
    """候補 (C, 8) をまとめて評価する。戻り値 `(使えるか, 時間のコスト, 当たる最初の障害物, d)`。"""
    d, dd, ddd = _profile(x, *cands[:, 1:].T)
    one = 1.0 - kr[None, :] * d
    kap_c = kr[None, :] / np.maximum(one, 0.2) + ddd
    # 道幅: 出発点が `room` の外に居るなら、最初の 1m は出発点の横ずれまで許す（戻っていく途中）
    fade = np.clip(1.0 - x / 1.0, 0.0, 1.0)
    lim_l = np.maximum(room_l, max(d_start, 0.0) * fade + 0.005)
    lim_r = np.maximum(room_r, max(-d_start, 0.0) * fade + 0.005)
    in_room = ((d <= lim_l[None, :]) & (-d <= lim_r[None, :])).all(axis=1)
    # Frenet の曲率は近似なので（直交座標で測ると数 % 大きい）、限界の 95% までにする。元の線が
    # すでに限界を超えている所（道幅が足りず半径を開けられないヘアピン）は、元の曲率に限界の
    # `_KAPPA_SLACK` まで上乗せしてよい（`avoid.plan_offset` と同じ。+0.001 しか許さないと、
    # そこでは少しでも横へずれた候補がすべて落ち、他車の後ろを 1 周近く付いて回った）
    k_lim = np.maximum(0.95 * cfg.kappa_max, np.abs(kr) + _KAPPA_SLACK * cfg.kappa_max)
    k_ok = ((np.abs(kap_c) <= np.where(np.abs(kr) > 0.95 * cfg.kappa_max, k_lim, 0.95 * cfg.kappa_max)[None, :]).all(axis=1)
            & (one > 0.2).all(axis=1))
    v, v_ok = _speeds(kap_c, v_cap, a_lat, ds, v_now, cfg)
    dt = 2.0 * ds[None, :] / (v[:, :-1] + v[:, 1:])
    t = np.concatenate([np.zeros((len(cands), 1)), np.cumsum(dt, axis=1)], axis=1)
    hit, first, gap = _collide(x, d, dd, t, tracks, cfg, np.ones((1, len(x)), dtype=bool), d_start)
    ok = in_room & k_ok & v_ok & ~hit
    cost = t[:, -1] + _END_COST * np.abs(d[:, -1]) + _LAT_COST * np.abs(d).max(axis=1)
    fails = {"道幅": int((~in_room).sum()), "曲率": int((~k_ok).sum()), "減速": int((~v_ok).sum()),
             "重なる": int(hit.sum())}
    return ok, cost, first, d, fails, np.where(in_room & k_ok, gap, -np.inf)


def _lengths(delta: float, v: float, kappa_budget: float, kappa_max: float) -> list[float]:
    """横に `delta` ずれる長さの候補 [m]（短い順）。

    最短は**車の限界の曲率**で決まる長さ（カーブの外へ出る向きなら元の曲率が減るので、
    `kappa_budget`＝限界から元の曲率を引いた残り、より短く入れる）。その上に `kappa_budget` で
    決まる長さとその倍、速さに合わせた長さ。使えるかは `eval_cand` が曲率で確かめる。
    """
    dl = max(abs(delta), 1e-3)
    l_phys = max(0.3, math.sqrt(5.77 * dl / max(0.95 * kappa_max, 1e-3)))
    l_bud = max(0.3, math.sqrt(5.77 * dl / max(kappa_budget, 1e-3)))
    ls = {round(l_phys, 2), round(l_bud, 2), round(1.5 * l_bud, 2), round(2.5 * l_bud, 2),
          round(max(l_bud, 0.8 * max(v, 0.5)), 2)}
    return sorted(ls)


def plan(path: RaceLine, index: int, pose: tuple[float, float, float], v_now: float,
         v_base: np.ndarray, a_lat, room: tuple[np.ndarray, np.ndarray],
         obstacles: list[Obstacle], cfg: OvertakeConfig, *, prev: Cand | None = None,
         s_cum: np.ndarray | None = None) -> OvertakeResult:
    """今の位置から先の一時経路を引く（閉じた経路だけ）。

    :param index: 車の今いる経路の添字
    :param v_base: 元の経路の速度（点ごと。見通しの上限などを掛けたもの）
    :param a_lat: 横加速度の上限（スカラーか点ごと）
    :param room: 経路の各点から左・右へずらせる量 [m]（壁と車体の余裕を引いたもの）
    :param prev: 前の周期に選んだ候補（その上に居れば、その状態から出発する）
    :param s_cum: 経路の始点からの弧長（点ごと。省略すると作る）
    """
    n = len(path)
    if s_cum is None:
        seg = np.hypot(*np.diff(np.vstack([path.xy, path.xy[:1]]), axis=0).T)
        s_cum = np.concatenate([[0.0], np.cumsum(seg[:-1])])
    L = path.length
    v0 = max(v_now, 0.0)
    H = min(_H_MAX, max(_H_MIN, v0 * _H_T))
    nrm = normals(path.xy)
    along, d_act, d1_act = frenet_state(path, index, *pose)
    s_car = (float(s_cum[index]) + along) % L

    # 見る先の点（0.2m おき）と、障害物を Frenet に直すための少し後ろからの点
    step_n = max(1, int(round((_STEP if v0 < 1.5 else _STEP_FAST) / max(L / n, 1e-3))))
    back = int(round(1.0 / max(L / n, 1e-3)))
    span = int(math.ceil(H / max(L / n, 1e-3))) + 2
    full = (index - back + np.arange(back + span + 1)) % n
    s_full = ((s_cum[full] - s_car + 0.5 * L) % L) - 0.5 * L
    tracks = frenet_tracks(path, full, s_full, nrm, obstacles, cfg)
    # ★ 車より後ろの物は数えない。後ろから速い車が来ても、こちらが止まって避けることはない
    #   （数えると予測で全候補が重なり、止まったところへ突っ込まれた。sim.bench --mover）
    tracks = [tr for tr in tracks if tr.s_hi > -cfg.rear and tr.s_lo < H + 1.0]

    # 出発点: 前の計画の上に居ればその状態、外れていれば今の姿勢
    start = (d_act, d1_act, 0.0)
    x_prev = None
    if prev is not None:
        x_prev = (s_car - prev.s0) % L
        if x_prev < 0.5 * L:
            dp, d1p, d2p = prev.at(x_prev)
            if abs(dp - d_act) < _ON_PLAN_M:
                start = (dp, d1p, d2p)
            else:
                x_prev = None
        else:
            x_prev = None
    if not tracks and abs(start[0]) < _ON_LINE_M:
        return OvertakeResult("clear", None, None, [], None, 0, "")

    sel = (index + np.arange(0, span, step_n)) % n
    x = ((s_cum[sel] - s_car) % L)
    x = x[x <= H + 1e-6]
    sel = sel[:len(x)]
    if len(x) < 4:
        return OvertakeResult("clear", None, None, [], None, 0, "")
    ds = np.maximum(np.diff(x), 1e-3)
    kr = path.kappa[sel] if len(path.kappa) == n else curvature(path.xy, True)[sel]
    v_cap = np.asarray(v_base, dtype=float)[sel]
    al = np.broadcast_to(np.asarray(a_lat, dtype=float), (n,))[sel] if np.ndim(a_lat) else \
        np.full(len(sel), float(a_lat))
    room_l, room_r = room[0][sel], room[1][sel]

    d0, d1, d2 = start
    k_budget = max(0.3 * cfg.kappa_max, cfg.kappa_max - float(np.abs(kr).max()))
    k_budget = min(k_budget, max(0.5, float(al.min()) / max(v0, 0.5) ** 2))
    rows: list[tuple] = []

    def add(dT: float, L1s, S2s, L3s) -> None:
        for L1 in L1s:
            for S2 in S2s:
                if S2 < L1:
                    continue
                for L3 in L3s:
                    rows.append((s_car, d0, d1, d2, dT, L1, S2, L3))

    # 経路へ戻る（＝経路のまま）。並ぶ区間は無い
    for L1 in _lengths(d0, v0, k_budget, cfg.kappa_max):
        add(0.0, [L1], [L1], [1.0])
    targets: list[float] = []
    for tr in tracks:
        for side in (+1, -1):
            base = tr.d_hi if side > 0 else tr.d_lo
            for e in ((0.07, 0.14) if tr.moving else (0.0, 0.08)):
                targets.append(base + side * (cfg.half_width + cfg.margin_lat + 0.01 + e))
    if prev is not None and x_prev is not None and abs(prev.dT) > 1e-3:
        targets.append(prev.dT)
    seen: list[float] = []
    for dT in targets:
        if any(abs(dT - q) < 0.01 for q in seen):
            continue
        seen.append(dT)
        L1s = _lengths(dT - d0, v0, k_budget, cfg.kappa_max)[:5]
        L3s = _lengths(dT, v0, k_budget, cfg.kappa_max)[2:4]
        # 戻り始めは、いちばん手前の障害物の手前より前にしても意味が無い（避ける前に戻ってしまう）
        s_first = max(0.0, min(tr.s_lo for tr in tracks)) if tracks else 0.0
        for L1 in L1s:
            a = max(L1, s_first)
            S2s = [a + k * _S2_STEP for k in range(int(max(H - a, 0.0) / _S2_STEP) + 1)] + [math.inf]
            add(dT, [L1], S2s, L3s)
    # 前の計画をそのまま続ける候補（同じ多項式になる）
    n_before = len(rows)
    if prev is not None and x_prev is not None:
        L1r = prev.L1 - x_prev
        if L1r > 0.05:
            rows.append((s_car, d0, d1, d2, prev.dT, L1r, prev.S2 - x_prev, prev.L3))
        elif prev.S2 - x_prev > 0.0:
            rows.append((s_car, d0, d1, d2, prev.dT, 0.05, prev.S2 - x_prev, prev.L3))
        elif prev.S2 + prev.L3 - x_prev > 0.05:
            L1r = prev.S2 + prev.L3 - x_prev
            rows.append((s_car, d0, d1, d2, 0.0, L1r, L1r, 1.0))
    cands = np.array(rows, dtype=float)

    ok, cost, first, dmat, fails, gap = eval_cand(cands, x, kr, v_cap, al, ds, room_l, room_r, v0, tracks, cfg, d0)
    # 経路のまま（今ほぼ経路の上・戻る候補のどれか）が通れるなら避けない
    line = cands[:, 4] == 0.0
    if abs(d0) < _ON_LINE_M and (ok & line).any():
        return OvertakeResult("clear", None, None, [], None, len(cands), "")
    if prev is not None:
        side_prev = math.copysign(1.0, prev.dT) if abs(prev.dT) > 1e-3 else 0.0
        side = np.sign(cands[:, 4])
        cost = cost + np.where((side != 0) & (side_prev != 0) & (side != side_prev), _SWITCH_COST, 0.0)
        if len(rows) > n_before:
            cost[-1] -= _KEEP_BONUS
    lead = None
    hit_line = first[line]
    if (hit_line >= 0).any():
        lead = tracks[int(hit_line[hit_line >= 0][0])]
    if not ok.any():
        # 通れる候補が無い。道幅と曲率を満たす候補のうち、相手との横の間隔がいちばん大きいものを
        # 逃げ道として返す（並んでいる相手の側へ元の線で戻っていかないため）。速度は呼び出し側が抑える
        k = int(np.argmax(gap))
        ranked = [Cand(*cands[k])] if np.isfinite(gap[k]) or gap[k] == np.inf else []
        return OvertakeResult("blocked", None, ranked[0] if ranked else None, ranked, lead, len(cands),
                              f"候補 {len(cands)} 本すべて不可（"
                              + "・".join(f"{k}{v}" for k, v in fails.items() if v) + "）")
    order = np.argsort(np.where(ok, cost, np.inf))
    ranked = [Cand(*cands[int(i)]) for i in order[:min(6, int(ok.sum()))]]
    kept = len(rows) > n_before and int(order[0]) == len(rows) - 1
    return OvertakeResult("avoid", None, ranked[0], ranked, lead, len(cands), "", kept)


def build(path: RaceLine, index: int, cand: Cand, s_cum: np.ndarray, obstacle: Obstacle | None,
          clearance: float = math.inf) -> AvoidPlan:
    """候補を経路の点へ当てはめた一時経路（点の数と添字は元の経路と同じ）。"""
    n = len(path)
    L = path.length
    nrm = normals(path.xy)
    if not math.isfinite(cand.S2):
        # 見る先の中では戻らない候補。経路の点へ当てはめるときは、見る先の終わりから滑らかに戻す
        # （段になって 0 へ落ちると、そこで曲率が跳ねる）。次の周期に引き直すので、ここは通らない
        cand = cand._replace(S2=max(cand.L1, _H_MAX), L3=max(cand.L3, 2.5))
    s_end = cand.L1 if cand.dT == 0.0 else cand.S2 + cand.L3
    s_end = min(s_end, 0.9 * L)
    back = 10
    span = int(math.ceil(s_end / max(L / n, 1e-3))) + 2
    idx = (index - back + np.arange(back + span)) % n
    x = (s_cum[idx] - cand.s0 + 0.5 * L) % L - 0.5 * L
    d, _, _ = _profile(np.maximum(x, 0.0), *(np.array([v]) for v in cand[1:]))
    d = d[0]
    xy = path.xy.copy()
    xy[idx] += nrm[idx] * d[:, None]
    rl = RaceLine(xy=xy, v=path.v.copy(), kappa=curvature(xy, True), length=path.length, closed=True)
    # 窓は車の位置から先（後ろの点も今の横ずれでずらすが、確かめ・戻ったかの判定には使わない）
    keep = np.abs(d[back:]) > 1e-4
    ahead = idx[back:]
    win = ahead[keep] if keep.any() else ahead[:1]
    return AvoidPlan(rl, win, float(cand.dT if cand.dT != 0.0 else cand.d0), obstacle, clearance)

