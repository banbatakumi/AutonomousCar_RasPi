"""コースの中心線に沿って動く障害物（他車の代わり）。`sim.bench --mover` が使う。

シムの世界（LiDAR と衝突判定が見る占有格子）に円盤（`length` 0）か、進む向きに回した
車の形の矩形（`length` > 0、幅 `2r`）を刻み、動くたびに前の場所を元へ戻して
新しい場所へ刻み直す。中心線を弧長 `s` で辿り、法線方向へ `d` ずらす。速度が負なら逆走
（すれ違う車）。壁には食い込まない（元の値を保存して戻す）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .track import _disc_slices

__all__ = ["Mover", "parse_mover"]


@dataclass
class Mover:
    """中心線を `speed` [m/s] で進む半径 `r` の円盤。

    :param s0: 出発点（コースの `start` に最も近い中心線上の点）から先へ進めた弧長 [m]
    :param d: 中心線からの横ずれ [m]（左が正）
    :param t_start: 動き出すまでの待ち [s]（それまでは `s0` に止まる）
    :param length: 0 なら半径 `r` の円盤。正なら長さ `length`・幅 `2r` の矩形（他車。LiDAR には
        後ろ面しか見えず、長さは見えない）
    """

    s0: float
    d: float
    r: float
    speed: float
    t_start: float = 0.0
    length: float = 0.0
    xy: tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0
    center: np.ndarray | None = None
    start_s: float = 0.0
    _saved: tuple | None = field(default=None, repr=False)

    def bind(self, center: np.ndarray, start_xy: tuple[float, float]) -> None:
        """走らせる閉じた線（世界座標）を渡す。`start_xy` に最も近い点を弧長 0 とする。
        コースが中心線を持たない（PNG コース）ので、走行中に planner の経路から受け取る。"""
        c = np.asarray(center, dtype=float)
        seg = np.hypot(*np.diff(np.vstack([c, c[:1]]), axis=0).T)
        self._s = np.concatenate([[0.0], np.cumsum(seg)])           # 長さ n+1（閉じる）
        self._c = np.vstack([c, c[:1]])
        self._loop = float(self._s[-1])
        k = int(np.argmin((c[:, 0] - start_xy[0]) ** 2 + (c[:, 1] - start_xy[1]) ** 2))
        self.start_s = float(self._s[k])
        self.center = c

    def pose_at(self, t: float) -> tuple[float, float]:
        assert self.center is not None
        s = (self.start_s + self.s0 + self.speed * max(0.0, t - self.t_start)) % self._loop
        i = int(np.searchsorted(self._s, s, side="right") - 1)
        i = min(i, len(self._c) - 2)
        f = (s - self._s[i]) / max(self._s[i + 1] - self._s[i], 1e-9)
        p = self._c[i] + f * (self._c[i + 1] - self._c[i])
        tan = self._c[i + 1] - self._c[i]
        tan = tan / max(float(np.hypot(*tan)), 1e-9)
        self._yaw_at = math.atan2(float(tan[1]), float(tan[0])) + (math.pi if self.speed < 0 else 0.0)
        return (float(p[0] - tan[1] * self.d), float(p[1] + tan[0] * self.d))

    def update(self, grid: np.ndarray, origin: tuple[float, float], resolution: float,
               t: float) -> None:
        """前の場所を元へ戻し、`t` の場所へ刻む。"""
        self.erase(grid)
        if self.center is None:
            return
        self.xy = self.pose_at(t)
        self.yaw = self._yaw_at
        if self.length > 0.0:
            rr = max(1, int(math.ceil(math.hypot(0.5 * self.length, self.r) / resolution)))
            yy, xx = np.mgrid[-rr:rr + 1, -rr:rr + 1] * resolution
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            disc = ((np.abs(xx * c + yy * s) <= 0.5 * self.length)
                    & (np.abs(-xx * s + yy * c) <= self.r))
        else:
            rr = max(1, int(round(self.r / resolution)))
            yy, xx = np.mgrid[-rr:rr + 1, -rr:rr + 1]
            disc = (xx * xx + yy * yy) <= rr * rr
        col = int(round((self.xy[0] - origin[0]) / resolution))
        row = int(round((self.xy[1] - origin[1]) / resolution))
        sl = _disc_slices(*grid.shape, row, col, rr)
        if sl is None:
            return
        gr0, gr1, gc0, gc1, sr0, sr1, sc0, sc1 = sl
        self._saved = (gr0, gr1, gc0, gc1, grid[gr0:gr1, gc0:gc1].copy())
        grid[gr0:gr1, gc0:gc1] |= disc[sr0:sr1, sc0:sc1]

    def erase(self, grid: np.ndarray) -> None:
        if self._saved is not None:
            gr0, gr1, gc0, gc1, old = self._saved
            grid[gr0:gr1, gc0:gc1] = old
            self._saved = None


def parse_mover(spec: str) -> Mover:
    """`S0,SPEED[,D,R,T_START,LENGTH]` から `Mover` を作る（線は走行中に `bind` で受け取る）。

    S0: 出発点から先へ置く弧長 [m]、SPEED: 線に沿った速度 [m/s]（負で逆走）、
    D: 線からの横ずれ [m]（既定 0）、R: 半径 [m]（既定 0.12。矩形なら半幅）、T_START: 動き出す時刻 [s]（既定 0）、
    LENGTH: 0 なら円盤、正なら車の形の矩形の長さ [m]（既定 0）。
    """
    v = [float(x) for x in spec.split(",")]
    if len(v) < 2:
        raise ValueError("--mover は S0,SPEED[,D,R,T_START,LENGTH]")
    return Mover(s0=v[0], speed=v[1], d=v[2] if len(v) > 2 else 0.0,
                 r=v[3] if len(v) > 3 else 0.12, t_start=v[4] if len(v) > 4 else 0.0,
                 length=v[5] if len(v) > 5 else 0.0)
