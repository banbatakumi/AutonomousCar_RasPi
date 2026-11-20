"""`sim.bench` が走行中に入れる自己位置の妨害（車を動かす・壁を欠く）と、その復帰の採点。

- `--kidnap T:DX:DY:DYAW[:HOLD]`: 走行開始から T 秒で、真の車を車体基準で (DX, DY) [m]・DYAW [deg] 動かす。
  HOLD 秒のあいだは持ち上げた状態（車はその姿勢で止まったまま・LiDAR は何も返さない）にする。
  **人が手で車を置き直す**状況（決勝の手動復帰・車がぶつかって弾かれた）を真似る
- `--place T:X:Y:YAW[:HOLD]`: 同じく、世界座標の姿勢 (X, Y, YAW [deg]) へ置き直す（離れた所・逆向きに置かれた）
- `--wall-gap X0,Y0,X1,Y1`: シムの世界の壁を矩形で消す（凍結地図には残る＝地図と実物の食い違い）

採点は真値と planner の自己位置の差で行う（planner の自己申告ではない）。復帰は「位置 0.15m・向き 5° 以内が
1 秒続いた最初の時刻」、誤った姿勢での走行は「位置の差が 0.3m を超えていた間に真の車が進んだ距離」。
**このファイルを実機のコードから import してはいけない。**
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import msgspec
import numpy as np

__all__ = ["Kidnap", "KidnapRunner", "blank_scan", "carve_rects", "parse_kidnap", "parse_place", "parse_rect"]

#: 戻ったとみなす位置・向きの差と、それが続く時間
RECOVER_POS_M = 0.15
RECOVER_YAW_DEG = 5.0
RECOVER_HOLD_S = 1.0
#: 「誤った姿勢で走っている」とみなす位置の差 [m]
WRONG_POS_M = 0.30


def parse_kidnap(text: str) -> tuple[float, float, float, float, float]:
    v = [float(x) for x in text.split(":")]
    if len(v) not in (4, 5):
        raise SystemExit(f"--kidnap は T:DX:DY:DYAW[:HOLD]: {text!r}")
    return (v[0], v[1], v[2], v[3], v[4] if len(v) == 5 else 0.0)


def parse_place(text: str) -> tuple[float, float, float, float, float, bool]:
    v = [float(x) for x in text.split(":")]
    if len(v) not in (4, 5):
        raise SystemExit(f"--place は T:X:Y:YAW[:HOLD]: {text!r}")
    return (v[0], v[1], v[2], v[3], v[4] if len(v) == 5 else 0.0, True)


def parse_rect(text: str) -> tuple[float, float, float, float]:
    v = [float(x) for x in text.split(",")]
    if len(v) != 4:
        raise SystemExit(f"--wall-gap は X0,Y0,X1,Y1: {text!r}")
    return (min(v[0], v[2]), min(v[1], v[3]), max(v[0], v[2]), max(v[1], v[3]))


def carve_rects(grid: np.ndarray, origin: tuple[float, float], resolution: float,
                rects: list[tuple[float, float, float, float]]) -> int:
    """占有格子から世界座標の矩形を消す（in-place）。消した占有セルの数を返す。"""
    x0, y0 = origin
    h, w = grid.shape
    n = 0
    for ax, ay, bx, by in rects:
        c0 = max(0, math.floor((ax - x0) / resolution))
        c1 = min(w, math.ceil((bx - x0) / resolution))
        r0 = max(0, math.floor((ay - y0) / resolution))
        r1 = min(h, math.ceil((by - y0) / resolution))
        if c1 <= c0 or r1 <= r0:
            continue
        n += int(np.count_nonzero(grid[r0:r1, c0:c1]))
        grid[r0:r1, c0:c1] = 0
    return n


def blank_scan(scan):
    """全点を「測れなかった」（dist = 0）にした点群（持ち上げて LiDAR が天井も床も見ない状態）。"""
    d = [0.0] * len(scan.dist)
    sat = None if scan.saturated is None else [False] * len(scan.saturated)
    inten = None if scan.intensity is None else [0] * len(scan.intensity)
    return msgspec.structs.replace(scan, dist=d, saturated=sat, intensity=inten)


@dataclass
class Kidnap:
    """1 回の車の移動と、その後の復帰の記録。"""

    t: float                     #: 走行開始からの時刻 [s]
    dx: float
    dy: float
    dyaw_deg: float
    hold_s: float
    absolute: bool = False       #: True なら (dx, dy, dyaw) は世界座標の姿勢 (x, y, yaw[deg])
    t_world: float = 0.0         #: bench の時刻 [s]
    recovered_t: float | None = None     #: 置いてから戻るまで [s]（持ち上げの時間を除く）
    wrong_dist: float = 0.0      #: 誤った姿勢（差 > 0.3m）のまま真の車が進んだ距離 [m]
    max_err: float = 0.0
    _ok_since: float | None = field(default=None, repr=False)
    _last_xy: tuple[float, float] | None = field(default=None, repr=False)

    def line(self) -> str:
        rec = "戻らず" if self.recovered_t is None else f"{self.recovered_t:.1f}s で復帰"
        what = "置き直し" if self.absolute else "車の移動"
        return (f"{what} {self.t:.0f}s ({self.dx:+.2f},{self.dy:+.2f}m {self.dyaw_deg:+.0f}° 持ち上げ {self.hold_s:.1f}s): "
                f"{rec}・誤った姿勢で {self.wrong_dist:.2f}m 走行・最大の差 {self.max_err:.2f}m")


class KidnapRunner:
    """`Bench` から毎ループ呼ぶ。車を動かし、持ち上げの間は姿勢を押さえ、復帰を採点する。"""

    def __init__(self, items: list[tuple]) -> None:
        self.items = sorted((Kidnap(*it) for it in items), key=lambda k: k.t)
        self._i = 0
        self._hold: tuple[tuple[float, float, float], float] | None = None   # (姿勢, 終わりの bench 時刻)
        self._active: Kidnap | None = None

    @property
    def holding(self) -> bool:
        return self._hold is not None

    def step(self, t: float, race_t0: float | None, vehicle) -> None:
        """bench の時刻 `t` [s]。走行開始（`race_t0`）から数えて時刻が来たら車を動かす。"""
        if race_t0 is None:
            return
        while self._i < len(self.items) and t - race_t0 >= self.items[self._i].t:
            k = self.items[self._i]
            self._i += 1
            if k.absolute:
                pose = (k.dx, k.dy, (math.radians(k.dyaw_deg) + math.pi) % (2 * math.pi) - math.pi)
            else:
                c, s = math.cos(vehicle.yaw), math.sin(vehicle.yaw)
                pose = (vehicle.x + c * k.dx - s * k.dy, vehicle.y + s * k.dx + c * k.dy,
                        (vehicle.yaw + math.radians(k.dyaw_deg) + math.pi) % (2 * math.pi) - math.pi)
            k.t_world = t
            self._hold = (pose, t + max(0.0, k.hold_s))
            self._active = k
            self._place(vehicle, pose)
        if self._hold is not None:
            pose, until = self._hold
            self._place(vehicle, pose)
            if t >= until:
                self._hold = None

    @staticmethod
    def _place(v, pose: tuple[float, float, float]) -> None:
        v.x, v.y, v.yaw = pose
        v.speed = 0.0
        v.yaw_rate = 0.0

    def observe(self, t: float, pos_err: float, yaw_err_deg: float, x: float, y: float) -> None:
        """自己位置の差（真値との）を 1 周期ぶん見る。"""
        k = self._active
        if k is None or self._hold is not None:
            return
        k.max_err = max(k.max_err, pos_err)
        if k._last_xy is not None and pos_err > WRONG_POS_M:
            k.wrong_dist += math.hypot(x - k._last_xy[0], y - k._last_xy[1])
        k._last_xy = (x, y)
        if k.recovered_t is not None:
            return
        if pos_err <= RECOVER_POS_M and yaw_err_deg <= RECOVER_YAW_DEG:
            if k._ok_since is None:
                k._ok_since = t
            elif t - k._ok_since >= RECOVER_HOLD_S:
                k.recovered_t = k._ok_since - (k.t_world + k.hold_s)
        else:
            k._ok_since = None
