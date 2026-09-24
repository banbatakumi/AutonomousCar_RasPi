"""ml_lidar/obstacles.py — 手続きコースへ静的な円柱障害物を置く。**置くたびに通れることを確かめる。**

## なぜ通過可能性を検証してから置くのか

解けない配置が学習分布に混ざると、方策は「衝突するくらいなら止まる」を覚える。
course3のヘアピン（前進では2.0m/sで通れない、PROGRESS.md 2026-09-18節）でv23が
その場で停止し続けたのが実例で、衝突罰だけの報酬では「止まる」が合理的な答えになる。
道幅と障害物径からの**隙間の幾何計算だけでは不十分**——隙間が車幅より広くても、
直前のコーナーから横移動しきれない配置は通れない。そこで障害物の手前から先までを
**前進のみ・最小旋回半径つきの格子探索**で実際に抜けられるか確かめ、抜けられなければ
置き直す（それでもだめならその1個は置かない）。

## 探索の仮定（＝何を「通れる」とみなすか）

- 前進のみ（方策に後退は無い。`env.py`の`_to_physical()`参照）
- 旋回半径は`course_gen`の中心線と同じ下限（舵角限界×`_RADIUS_MARGIN`）。これは
  **低速でしか使えない半径**なので、「通れる」は「減速すれば通れる」の意味
  （`course_gen.vehicle_min_turn_radius_m()`のdocstring参照）
- 車体は`raspi.nav.local_map.footprint_circles`の5×3円被覆（前後1.1cm・左右1.8cm
  外へはみ出す＝安全側）に`_CLEARANCE_M`を足して壁までの距離場と照合する
- 検証区間は障害物の`_CHECK_BEFORE_M`手前（中心線上・中心線の向き）から
  `_CHECK_AFTER_M`先まで。障害物どうしは`_MIN_SPACING_M`以上離すので、区間に入る
  障害物は常に1個

## `Course`を直接書き換える

`Course.grid`へ円盤をOR演算で刻む（`sim.track.stamp_discs`、コースエディタの障害物と
同じ関数）。LiDAR（`sim.lidar.VirtualLidar`）も衝突判定（`Course.collides`）も格子
だけを見るので、刻めば両方に反映される。`raycast()`のキャッシュ`_padded_grid`は
刻んだあとに捨てる。**呼び出し側は使い回しのコース（evalの固定コース）を渡さないこと。**
"""

from __future__ import annotations

import heapq
import math

import cv2
import numpy as np

from raspi.nav.local_map import footprint_circles
from sim import track
from sim.course import Course
from sim.vehicle import VehicleSpec

from .course_gen import _RADIUS_MARGIN, vehicle_min_turn_radius_m

__all__ = ["PassabilityChecker", "add_obstacles", "passable"]

#: 障害物の半径レンジ [m]。下限はLiDARの角度分解能（1°≒10cm先で1.7mm、3m先で5cm）で
#: 数点は当たる太さ、上限は道幅の下限0.8mでも片側に車幅ぶんの隙間を残せる太さ
_RADIUS_RANGE_M = (0.05, 0.20)
#: 障害物どうしの最小弧長間隔 [m]。`_CHECK_BEFORE_M + _CHECK_AFTER_M`より長くして、
#: 検証区間に障害物が2個入らないようにする（入ると1個ずつの検証が組み合わせを見逃す）
_MIN_SPACING_M = 2.4
#: スタート地点の前後に置かない範囲 [m]。スタート姿勢が埋まる・発進直後に避けようのない
#: 位置に来るのを防ぐ
_START_CLEAR_AHEAD_M = 1.5
_START_CLEAR_BEHIND_M = 0.8
#: 1個あたりの置き直し回数の上限
_MAX_PLACE_TRY = 8
#: 通過可能性を確かめる区間 [m]（障害物の弧長位置から見て手前・先）
_CHECK_BEFORE_M = 1.2
_CHECK_AFTER_M = 1.0
#: 円被覆に加える余裕 [m]
_CLEARANCE_M = 0.02
#: 格子探索のプリミティブ（1本の弧長 [m]・片側の舵段数）と重複判定の格子
_STEP_M = 0.10
_N_STEER = 3
_XY_RES_M = 0.05
_YAW_RES_RAD = math.radians(10.0)
#: ゴールで許す中心線の向きとの差
_GOAL_YAW_TOL_RAD = math.radians(60.0)
#: 展開の上限（超えたら「通れない」扱い＝安全側）
_MAX_EXPANSIONS = 8000


class PassabilityChecker:
    """1本のコースについて、障害物の前後を前進で抜けられるかを判定する。

    距離場は`update()`で作り直す（障害物を刻んだら呼ぶ）。
    """

    def __init__(self, course: Course, spec: VehicleSpec | None = None) -> None:
        if course.centerline is None:
            raise ValueError("centerlineを持たないコースには障害物を置けない")
        spec = spec or VehicleSpec.load()
        self.course = course
        self._circles, self._circle_r = footprint_circles(spec.footprint)
        self._kappa_max = 1.0 / (vehicle_min_turn_radius_m(spec) * _RADIUS_MARGIN)
        xy = course.centerline[:, :2]
        seg = np.hypot(*np.diff(np.vstack([xy, xy[:1]]), axis=0).T)
        self._arc = np.concatenate([[0.0], np.cumsum(seg[:-1])])
        self.total_length = float(seg.sum())
        self._edt: np.ndarray | None = None
        self.update()

    def update(self) -> None:
        free = (~self.course.grid).astype(np.uint8)
        self._edt = cv2.distanceTransform(free, cv2.DIST_L2, 5) * self.course.resolution

    def body_clear(self, x: np.ndarray, y: np.ndarray, yaw: np.ndarray) -> np.ndarray:
        """姿勢の配列ごとに、車体の円被覆がどれも壁から`_CLEARANCE_M`以上離れているか。"""
        x, y, yaw = (np.atleast_1d(np.asarray(v, dtype=np.float64)) for v in (x, y, yaw))
        c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        cx, cy = self._circles[:, 0][None, :], self._circles[:, 1][None, :]
        wx = x[:, None] + cx * c - cy * s
        wy = y[:, None] + cx * s + cy * c
        res = self.course.resolution
        col = np.floor((wx - self.course.origin[0]) / res).astype(np.int64)
        row = np.floor((wy - self.course.origin[1]) / res).astype(np.int64)
        h, w = self._edt.shape
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        d = np.zeros(col.shape)
        d[inside] = self._edt[row[inside], col[inside]]
        # 格子のセル中心と点のずれ（最大 res/√2）ぶんを保守側に引く
        ok = inside & (d - res * 0.71 >= self._circle_r + _CLEARANCE_M)
        return ok.all(axis=1)

    def _window(self, s0: float, length: float) -> tuple[np.ndarray, np.ndarray]:
        """弧長`s0`から`length`ぶんの中心線点のインデックスと、`s0`起点の相対弧長。"""
        rel = (self._arc - s0) % self.total_length
        m = rel <= length
        idx = np.nonzero(m)[0]
        order = np.argsort(rel[idx])
        return idx[order], rel[idx][order]

    def pose_at(self, s: float) -> tuple[float, float, float]:
        i = int(np.argmin(np.abs(((self._arc - s) + self.total_length / 2) % self.total_length
                                 - self.total_length / 2)))
        x, y, yaw = self.course.centerline[i]
        return float(x), float(y), float(yaw)

    def search(self, s_obs: float) -> list[tuple[float, float, float]] | None:
        """障害物（弧長`s_obs`）の手前から先まで前進で抜ける経路。無ければ`None`。

        A*（g=走行距離、h=残りの弧長）。姿勢は連続値で持ち、重複判定だけ格子で行う
        （`raspi/nav/hybrid_astar.py`と同じ考え方）。
        """
        s0 = (s_obs - _CHECK_BEFORE_M) % self.total_length
        span = _CHECK_BEFORE_M + _CHECK_AFTER_M
        idx, rel = self._window(s0, span + 0.5)
        cl = self.course.centerline[idx]
        start = self.pose_at(s0)
        if not self.body_clear(*start)[0]:
            return None

        def progress(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            d2 = (cl[None, :, 0] - x[:, None]) ** 2 + (cl[None, :, 1] - y[:, None]) ** 2
            k = np.argmin(d2, axis=1)
            return rel[k], cl[k, 2]

        kappas = np.linspace(-self._kappa_max, self._kappa_max, 2 * _N_STEER + 1)
        sub = np.array([0.5, 1.0]) * _STEP_M

        def key(p: tuple[float, float, float]) -> tuple[int, int, int]:
            return (int(math.floor(p[0] / _XY_RES_M)), int(math.floor(p[1] / _XY_RES_M)),
                    int(math.floor((p[2] % (2 * math.pi)) / _YAW_RES_RAD)))

        parent: dict[tuple[int, int, int], tuple] = {key(start): (None, start)}
        heap: list[tuple[float, float, int, tuple]] = [(span, 0.0, 0, start)]
        seq = 0
        expansions = 0
        while heap and expansions < _MAX_EXPANSIONS:
            _, g, _, pose = heapq.heappop(heap)
            expansions += 1
            x0, y0, th0 = pose
            # 全後継×中間点を1回のnumpy呼び出しで判定する
            th = th0 + kappas[:, None] * sub[None, :]
            with np.errstate(divide="ignore", invalid="ignore"):
                dx = np.where(np.abs(kappas[:, None]) > 1e-9,
                              (np.sin(th) - math.sin(th0)) / kappas[:, None],
                              np.cos(th0) * sub[None, :])
                dy = np.where(np.abs(kappas[:, None]) > 1e-9,
                              (math.cos(th0) - np.cos(th)) / kappas[:, None],
                              np.sin(th0) * sub[None, :])
            xs, ys = x0 + dx, y0 + dy
            ok = self.body_clear(xs.ravel(), ys.ravel(), th.ravel()).reshape(xs.shape).all(axis=1)
            if not ok.any():
                continue
            ex, ey, eth = xs[ok, -1], ys[ok, -1], th[ok, -1]
            prog, cyaw = progress(ex, ey)
            for x, y, t, p, cy in zip(ex, ey, eth, prog, cyaw):
                nxt = (float(x), float(y), float(t))
                k = key(nxt)
                if k in parent:
                    continue
                parent[k] = (key(pose), nxt)
                dyaw = abs((t - cy + math.pi) % (2 * math.pi) - math.pi)
                if p >= span and dyaw <= _GOAL_YAW_TOL_RAD:
                    path = [nxt]
                    pk = parent[k][0]
                    while pk is not None:
                        pk, pp = parent[pk]
                        path.append(pp)
                    return path[::-1]
                seq += 1
                g2 = g + _STEP_M
                heapq.heappush(heap, (g2 + max(0.0, span - float(p)), g2, seq, nxt))
        return None


def passable(course: Course, s_obs: float, spec: VehicleSpec | None = None) -> bool:
    """`course`（障害物は刻み済み）の弧長`s_obs`の前後を前進で抜けられるか。"""
    return PassabilityChecker(course, spec).search(s_obs) is not None


def _width_at(course: Course, i: int) -> float:
    w = course.width
    if w is None:
        raise ValueError("道幅が分からないコース（手描き）には障害物を置けない")
    return float(w) if np.isscalar(w) else float(np.asarray(w)[i])


def add_obstacles(course: Course, rng: np.random.Generator, n: int,
                  spec: VehicleSpec | None = None) -> np.ndarray:
    """`course`へ円柱障害物を最大`n`個刻み、置けたものを`(K,3)`(x,y,半径)で返す。

    **`course`をその場で書き換える**（`grid`・`obstacles`・`_padded_grid`）。
    1個ごとに置き直しを`_MAX_PLACE_TRY`回まで試し、どれも通れなければその1個は諦める
    （コースごと作り直すより安く、置けた個数は戻り値で分かる）。
    """
    checker = PassabilityChecker(course, spec)
    total = checker.total_length
    lo, hi = _START_CLEAR_AHEAD_M, total - _START_CLEAR_BEHIND_M
    placed: list[tuple[float, float, float]] = []
    placed_s: list[float] = []
    if hi <= lo:
        return np.zeros((0, 3))
    for _ in range(n):
        for _try in range(_MAX_PLACE_TRY):
            s = float(rng.uniform(lo, hi))
            if any(min(abs(s - t), total - abs(s - t)) < _MIN_SPACING_M for t in placed_s):
                continue
            i = int(np.argmin(np.abs(checker._arc - s)))
            cx, cy, cyaw = course.centerline[i]
            half = _width_at(course, i) / 2.0
            r = float(rng.uniform(*_RADIUS_RANGE_M))
            # 壁に接する置き方も含める（|offset|+r > half なら壁から生えた突起になる）
            off = float(rng.uniform(-half, half))
            ox, oy = cx - math.sin(cyaw) * off, cy + math.cos(cyaw) * off
            saved = course.grid.copy()
            track.stamp_discs(course.grid, course.origin, course.resolution, [(ox, oy, r)])
            checker.update()
            if checker.search(s) is not None:
                placed.append((ox, oy, r))
                placed_s.append(s)
                break
            course.grid[...] = saved
            checker.update()
    course._padded_grid = None
    obs = np.asarray(placed, dtype=np.float64).reshape(-1, 3)
    course.obstacles = obs if len(obs) else None
    return obs
