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

__all__ = ["Pursuit", "PursuitConfig", "follow"]


class PursuitConfig(NamedTuple):
    wheelbase: float = 0.25                #: L [m]
    max_steer: float = 0.60                #: [rad]
    lookahead_k: float = 0.8               #: `Ld = k·v + min` の k [s]
    lookahead_min: float = 0.35            #: [m]
    delay_s: float = 0.15                  #: 遅延補償の時間 [s]


class Pursuit(NamedTuple):
    steer: float                           #: 路面舵角 [rad] 反時計回り正
    speed: float                           #: 目標速度 [m/s]
    index: int                             #: 追っている経路点の添字
    cross_track: float                     #: 横偏差 [m]。**左にずれていれば正**
    target: tuple[float, float]            #: 目標点 [m]（GUI に描く）
    lookahead: float                       #: 使った前方注視距離 [m]


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


def nearest_index(path: RaceLine, x: float, y: float, hint: int = -1,
                  window: int = 0) -> int:
    """いちばん近い経路点。`hint` の周りだけ探せば速いが、**既定は全探索**。

    全探索でも 400 点なら 10μs 程度。`window` を使うのは、8の字のように
    経路が自分と交差するコースで**別の周回の点に飛び移るのを防ぎたい**とき。
    """
    d2 = (path.xy[:, 0] - x) ** 2 + (path.xy[:, 1] - y) ** 2
    if hint >= 0 and window > 0:
        n = len(path)
        idx = (hint + np.arange(-window, window + 1)) % n
        return int(idx[np.argmin(d2[idx])])
    return int(np.argmin(d2))


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
    tx = path.xy[(j + 1) % len(path)] - path.xy[j]
    to_car = np.array([pose[0], pose[1]]) - path.xy[j]
    nrm = np.hypot(*tx)
    cross = float((tx[0] * to_car[1] - tx[1] * to_car[0]) / nrm) if nrm > 1e-9 else 0.0

    ld = max(cfg.lookahead_min, cfg.lookahead_k * abs(v_now) + cfg.lookahead_min)
    # 経路上を弧長 `ld` ぶん進んだ点。**直線距離ではなく弧長**で取る
    # （ヘアピンでは直線距離だと「出口の点」が近くに見えてショートカットする）
    step = path.length / len(path)
    k = (i + max(1, int(round(ld / step)))) % len(path)
    goal = path.xy[k]

    dx, dy = goal[0] - px, goal[1] - py
    eta = math.atan2(dy, dx) - pyaw
    eta = (eta + math.pi) % (2.0 * math.pi) - math.pi
    dist = max(1e-3, math.hypot(dx, dy))
    steer = math.atan2(2.0 * cfg.wheelbase * math.sin(eta), dist)
    steer = max(-cfg.max_steer, min(cfg.max_steer, steer))

    return Pursuit(steer=steer, speed=float(path.v[k]), index=i,
                   cross_track=cross, target=(float(goal[0]), float(goal[1])),
                   lookahead=ld)
