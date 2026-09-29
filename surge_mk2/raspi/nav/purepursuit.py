"""Pure Pursuit — レーシングラインを追う。

`docs/architecture.md` §5.2 の自転車モデル（**base_link = 後輪車軸の中心**）が
そのまま使える。Pure Pursuit の標準式が後輪基準で書かれているのが、
原点をそこに置いた理由のひとつ。

    δ = atan( 2·L·sin(η) / Ld )      L=ホイールベース, η=目標点への方位, Ld=前方注視距離

## lookahead は速度に比例させる

`Ld = k_v·v + Ld_min`。固定にすると、低速では大回りし、高速では蛇行が発散する
（§14「低速では動くのに、速度を上げた瞬間に蛇行が発散する」）。

## 遅延補償 — 「今どこか」ではなく「舵が効き始めるときどこか」

§14 の実測要求どおり、ステアリングは 50〜200ms 遅れ、UART 往復と Python の処理で
さらに 30〜50ms 乗る。合計 150ms は 3 m/s なら **45cm 先**。現在位置で目標点を
選ぶと、その 45cm ぶん常に手遅れの舵を切る。

**判断する場所を `t_delay` 秒後の予測位置へ進める。** 予測は今の速度と実舵角を
使った自転車モデルで、`config/vehicle.toml` の `[dynamics]` が既定値を与える。

## 速度は舵の注視点ではなく、すぐ先のプロファイルから読む（`speed_preview_s`）

舵の注視点（遅延予測 + `k·v + min`）の速度を指令すると、2m/s で約1.5m・0.75s 先の
速度を今出すことになり、**コーナーの手前で早く減速しすぎ、立ち上がりでは早く
踏みすぎる**。速度は遅延予測の位置から `v·speed_preview_s` だけ先の値にする
（速度ループの遅れを埋めるぶんだけ先を見る）。`None` なら従来どおり注視点の速度。

## 曲率フィードフォワード — 注視区間で平均した曲率を、足元の曲率に差し替える（`ff_gain`）

Pure Pursuit は「今の位置から注視点へ届く円弧」の曲率を出すので、経路の上に
**ぴったり乗っていても**、注視区間（3m/s で約1.6m）で平均した曲率で曲がる。
曲率が変わり続ける S 字では、これが**内側を切る定常的な偏り**になった（toyota2・
3m/s で ±15cm、横G を上げると衝突）。「車が経路の点 i に経路の向きで居たら
Pure Pursuit が出す曲率」`κ_ideal` を求め、その点の経路の曲率 `κ_path` との差を足す:

    κ = κ_pp + g·(κ_path − κ_ideal)

経路上にいれば `κ = κ_path`（偏りなし）で、ずれの修正は Pure Pursuit のまま残る。

## 横偏差は「評価」であって「入力」ではない

`cross_track` は Pure Pursuit の計算には入らない（純粋に目標点への方位だけで
決まる）。**走りの良し悪しを人間が読むための数字**として返している。
これが増え続けるなら lookahead か遅延補償が合っていない。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .raceline import RaceLine

__all__ = ["Pursuit", "PursuitConfig", "follow", "steer_for_target"]


class PursuitConfig(NamedTuple):
    wheelbase: float = 0.23                #: L [m]（実測確定）
    max_steer: float = 0.524               #: [rad] = 30°（リンク比 0.5 実測確定）
    lookahead_k: float = 0.8               #: `Ld = k·v + min` の k [s]
    lookahead_min: float = 0.35            #: [m]
    delay_s: float = 0.15                  #: 遅延補償の時間 [s]
    #: 速度を読む先読み時間 [s]（遅延予測の位置から）。`None` = 舵の注視点の速度
    speed_preview_s: float | None = None
    #: 曲率フィードフォワードの重み（0 = 従来の Pure Pursuit、モジュール docstring）
    ff_gain: float = 0.0
    #: フィードフォワードの曲率を測る弦 [m]
    ff_chord: float = 0.4


class Pursuit(NamedTuple):
    steer: float                           #: 路面舵角 [rad] 反時計回り正
    speed: float                           #: 目標速度 [m/s]
    index: int                             #: 追っている経路点の添字
    cross_track: float                     #: 横偏差 [m]。**左にずれていれば正**
    target: tuple[float, float]            #: 目標点 [m]（GUI に描く）
    lookahead: float                       #: 使った前方注視距離 [m]
    #: 開いた経路の終点までの残り [m]（弧長）。閉じた経路では `inf`
    remaining: float = math.inf


def _predict(x: float, y: float, yaw: float, v: float, steer: float,
             wheelbase: float, dt: float) -> tuple[float, float, float]:
    """自転車モデルで `dt` 秒先を予測する。**遅延補償の中身。**"""
    if dt <= 0.0 or abs(v) < 1e-3:
        return x, y, yaw
    dyaw = v / wheelbase * math.tan(steer) * dt
    if abs(dyaw) < 1e-6:
        return x + v * dt * math.cos(yaw), y + v * dt * math.sin(yaw), yaw
    r = v * dt / dyaw
    return (x + r * (math.sin(yaw + dyaw) - math.sin(yaw)),
            y - r * (math.cos(yaw + dyaw) - math.cos(yaw)),
            yaw + dyaw)


def steer_for_target(eta: float, lookahead: float, wheelbase: float, max_steer: float) -> float:
    """狙点までの方位 `eta`[rad]・距離 `lookahead`[m] を舵角[rad]に変換する。

    Pure Pursuit の標準式 δ = atan(2L·sinη / Ld)。**この式自体は自己位置を
    要らない** — 自分（後輪車軸）を原点とした目標点の方位と距離さえあれば
    決まる。`follow()` が `pose` を使うのは、地図座標系の経路点列から
    「今の自分から見た目標点」を切り出す前段のためで、この式のためではない。

    `lookahead` は正の値を仮定（呼び出し側で下限を保証すること。ここでは
    0 除算だけ避ける）。
    """
    steer = math.atan2(2.0 * wheelbase * math.sin(eta), max(lookahead, 1e-3))
    return max(-max_steer, min(max_steer, steer))


def _ff_correction(path: RaceLine, i: int, goal, step: float, chord: float) -> float:
    """`κ_path − κ_ideal`（モジュール docstring「曲率フィードフォワード」）。"""
    n = len(path)
    h = max(1, int(round(chord / 2.0 / max(step, 1e-6))))

    def pt(j):
        return path.xy[j % n] if path.closed else path.xy[min(max(j, 0), n - 1)]

    a, b, c = pt(i - h), pt(i), pt(i + h)
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    k_path = 2.0 * cross / (ab * bc * ca) if ab * bc * ca > 1e-9 else 0.0
    t = pt(i + 1) - pt(i - 1)
    yaw = math.atan2(t[1], t[0])
    eta = math.atan2(goal[1] - b[1], goal[0] - b[0]) - yaw
    eta = (eta + math.pi) % (2.0 * math.pi) - math.pi
    k_ideal = 2.0 * math.sin(eta) / max(math.dist(goal, b), 1e-3)
    return k_path - k_ideal


def nearest_index(path: RaceLine, x: float, y: float, hint: int = -1,
                  window: int = 0) -> int:
    """いちばん近い経路点。`hint` の周りだけ探せば速いが、**既定は全探索**。

    全探索でも 400 点なら 10μs 程度。`window` を使うのは、8の字のように
    経路が自分と交差するコースで**別の周回の点に飛び移るのを防ぎたい**とき。
    """
    xy = path.xy
    if hint >= 0 and window > 0:
        n = len(path)
        idx = hint + np.arange(-window, window + 1)
        idx = idx % n if path.closed else idx[(idx >= 0) & (idx < n)]
        if len(idx):
            w = xy[idx]
            return int(idx[np.argmin((w[:, 0] - x) ** 2 + (w[:, 1] - y) ** 2)])
    return int(np.argmin((xy[:, 0] - x) ** 2 + (xy[:, 1] - y) ** 2))


def follow(path: RaceLine, pose: tuple[float, float, float], v_now: float,
           steer_now: float, cfg: PursuitConfig, *, hint: int = -1) -> Pursuit:
    """1周期ぶんの追従。

    :param pose: 現在の base_link 姿勢（map フレーム）
    :param v_now: 現在の車速 [m/s]。lookahead と遅延補償に使う
    :param steer_now: 現在の**実**舵角 [rad]（指令値ではない）
    """
    px, py, pyaw = _predict(*pose, v_now, steer_now, cfg.wheelbase, cfg.delay_s)

    i = nearest_index(path, px, py, hint, window=len(path) // 4 if hint >= 0 else 0)

    # 横偏差は**現在位置**で測る（予測位置で測ると自分の予測を評価してしまう）
    j = nearest_index(path, pose[0], pose[1], hint,
                      window=len(path) // 4 if hint >= 0 else 0)
    n = len(path)
    if path.closed:
        tx = path.xy[(j + 1) % n] - path.xy[j]
    else:
        jj = min(j, n - 2)
        tx = path.xy[jj + 1] - path.xy[jj]
    to_car = np.array([pose[0], pose[1]]) - path.xy[j]
    nrm = np.hypot(*tx)
    cross = float((tx[0] * to_car[1] - tx[1] * to_car[0]) / nrm) if nrm > 1e-9 else 0.0

    # `cfg.lookahead_k * abs(v_now) >= 0` なので、この式は常に既に
    # `cfg.lookahead_min` 以上（`max()` で下限を張る必要はない）
    ld = cfg.lookahead_k * abs(v_now) + cfg.lookahead_min
    # 経路上を弧長 `ld` ぶん進んだ点。**直線距離ではなく弧長**で取る
    # （ヘアピンでは直線距離だと「出口の点」が近くに見えてショートカットする）
    if path.closed:
        step = path.length / n
        k = (i + max(1, int(round(ld / step)))) % n
        goal = path.xy[k]
        remaining = math.inf
    else:
        # 開いた経路: 終点の先は**終点の接線方向へ延ばした点**を狙う。終点そのものを
        # 狙うと、近づくほど注視距離が縮んで舵が暴れる
        step = path.length / max(1, n - 1)
        remaining = max(0.0, (n - 1 - i) * step)
        k = min(n - 1, i + max(1, int(round(ld / step))))
        goal = path.xy[k]
        over = ld - remaining
        if over > 0.0 and n >= 2:
            t = path.xy[-1] - path.xy[-2]
            t = t / max(float(np.hypot(*t)), 1e-9)
            goal = path.xy[-1] + t * over

    dx, dy = goal[0] - px, goal[1] - py
    eta = math.atan2(dy, dx) - pyaw
    eta = (eta + math.pi) % (2.0 * math.pi) - math.pi
    dist = math.hypot(dx, dy)
    if cfg.ff_gain > 0.0 and n >= 5:
        kap = 2.0 * math.sin(eta) / max(dist, 1e-3)
        kap += cfg.ff_gain * _ff_correction(path, i, goal, step, cfg.ff_chord)
        steer = math.atan(cfg.wheelbase * kap)
        steer = max(-cfg.max_steer, min(cfg.max_steer, steer))
    else:
        steer = steer_for_target(eta, dist, cfg.wheelbase, cfg.max_steer)

    kv = k
    if cfg.speed_preview_s is not None:
        m = int(round(max(0.0, v_now) * cfg.speed_preview_s / max(step, 1e-6)))
        kv = (i + m) % n if path.closed else min(n - 1, i + m)
    return Pursuit(steer=steer, speed=float(path.v[kv]), index=i,
                   cross_track=cross, target=(float(goal[0]), float(goal[1])),
                   lookahead=ld, remaining=remaining)
