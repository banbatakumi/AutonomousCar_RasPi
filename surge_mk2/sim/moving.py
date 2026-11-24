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

__all__ = ["Mover", "parse_mover", "update_all"]


@dataclass
class Mover:
    """中心線を `speed` [m/s] で進む半径 `r` の円盤。

    :param s0: 出発点（コースの `start` に最も近い中心線上の点）から先へ進めた弧長 [m]
    :param d: 中心線からの横ずれ [m]（左が正）
    :param t_start: 動き出すまでの待ち [s]（それまでは `s0` に止まる）
    :param length: 0 なら半径 `r` の円盤。正なら長さ `length`・幅 `2r` の矩形（他車。LiDAR には
        後ろ面しか見えず、長さは見えない）
    :param profile: 0 なら一定の `speed`。正なら**線の速度プロファイル × `profile`** で走る（`speed` が正なら
        それを上限にする）。一定速度の他車はヘアピンでも速さを落とさないので、コーナーで落ちる自車には
        直線でしか抜く機会が無く、実戦より抜きにくい。プロファイルに倍率を掛けた相手は「同じ線を遅く走る車」
    """

    s0: float
    d: float
    r: float
    speed: float
    t_start: float = 0.0
    length: float = 0.0
    profile: float = 0.0
    #: 今の線に沿った速さ [m/s]（負 = 逆走）。`profile` の他車は場所で変わる
    u_now: float = 0.0
    xy: tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0
    center: np.ndarray | None = None
    start_s: float = 0.0
    _saved: tuple | None = field(default=None, repr=False)

    def bind(self, center: np.ndarray, start_xy: tuple[float, float],
             v: np.ndarray | None = None) -> None:
        """走らせる閉じた線（世界座標）と、その点ごとの速度（`profile` を使うとき）を渡す。`start_xy` に
        最も近い点を弧長 0 とする。コースが中心線を持たない（PNG コース）ので、走行中に planner の経路から受け取る。"""
        c = np.asarray(center, dtype=float)
        if self.profile > 0.0:
            if v is None or len(v) != len(c):
                raise ValueError("profile を使う他車には線の速度が要る")
            self._v = np.append(np.asarray(v, dtype=float), float(v[0]))
            self._s_run, self._t_run = 0.0, None
        seg = np.hypot(*np.diff(np.vstack([c, c[:1]]), axis=0).T)
        self._s = np.concatenate([[0.0], np.cumsum(seg)])           # 長さ n+1（閉じる）
        self._c = np.vstack([c, c[:1]])
        self._loop = float(self._s[-1])
        k = int(np.argmin((c[:, 0] - start_xy[0]) ** 2 + (c[:, 1] - start_xy[1]) ** 2))
        self.start_s = float(self._s[k])
        self.center = c

    def rebind(self, center: np.ndarray, t: float, v: np.ndarray | None = None) -> None:
        """走らせる線を取り替える（今の位置のまま、新しい線の上の最も近い点から続ける）。

        ★ planner の経路は走行中に形が替わる（1 周目の出発点からの枝を含む線 26.0m → 定常の周回 21.8m、
        toyota2 の経路 A）。最初に結んだ線のまま走らせると、他車は自分と別の線を走り続け、線が分かれる所で
        横切る・予測と食い違う（追い抜きの評価に作り物の接触と取り違えが混ざった）。"""
        if self.center is None:
            return
        old_xy = self.pose_at(t)                   # `xy` は描いたときだけ進むので、その時刻の位置を出し直す
        t_run = getattr(self, "_t_run", None)
        self.bind(center, old_xy, v)
        # bind は start_xy（今の位置）に最も近い点を弧長 0 にする。そこから続ける
        self.s0 = 0.0
        if self.profile > 0.0:
            self._s_run, self._t_run = 0.0, (t if t_run is not None else None)
        else:
            self.s0 = -self.speed * max(0.0, t - self.t_start)

    def _travelled(self, t: float) -> float:
        """動き出してから `t` までに線に沿って進んだ距離 [m]。"""
        if self.profile <= 0.0:
            self.u_now = self.speed if t > self.t_start else 0.0
            return self.speed * max(0.0, t - self.t_start)
        # プロファイルで走る: 呼ばれるたびに今の場所の速度で進める（時刻は増える一方）
        if t <= self.t_start:
            self.u_now = 0.0
            return 0.0
        if self._t_run is None:
            self._t_run = self.t_start
        dt = t - self._t_run
        if dt > 0.0:
            s = (self.start_s + self.s0 + self._s_run) % self._loop
            i = min(int(np.searchsorted(self._s, s, side="right") - 1), len(self._v) - 2)
            u = self.profile * float(self._v[i])
            if self.speed > 0.0:
                u = min(u, self.speed)
            self.u_now = max(u, 0.3)
            self._s_run += self.u_now * dt
            self._t_run = t
        return self._s_run

    def pose_at(self, t: float) -> tuple[float, float]:
        assert self.center is not None
        s = (self.start_s + self.s0 + self._travelled(t)) % self._loop
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
        """前の場所を元へ戻し、`t` の場所へ刻む。**2 台以上なら `update_all` を使う**。"""
        self.erase(grid)
        self.draw(grid, origin, resolution, t)

    def draw(self, grid: np.ndarray, origin: tuple[float, float], resolution: float,
             t: float) -> None:
        """`t` の場所へ刻む（刻む前の値を保存する）。消すのは `erase`。"""
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


def update_all(movers: list[Mover], grid: np.ndarray, origin: tuple[float, float],
               resolution: float, t: float) -> None:
    """全部を**描いたのと逆の順に消してから**、全部を描く。

    ★ 1 台ずつ「消す→描く」を回すと、2 台の範囲が重なったとき、後の車が保存していた値（＝前の車の
    古い姿を含む）を書き戻し、**消えない幻の障害物**が格子に残った。速い他車が遅い他車に追いつく
    （他車どうしは避けない）たびに起き、自車がそこで止まり・当たっていた（sim.bench toyota2 --mover 2 台、
    2026-11-17）。逆順に消せば、各車が保存した値は「自分より前の車が描く前」の格子に戻る。
    """
    for m in reversed(movers):
        m.erase(grid)
    for m in movers:
        m.draw(grid, origin, resolution, t)


def parse_mover(spec: str) -> Mover:
    """`S0,SPEED[,D,R,T_START,LENGTH[,PROFILE]]` から `Mover` を作る（線は走行中に `bind` で受け取る）。

    S0: 出発点から先へ置く弧長 [m]、SPEED: 線に沿った速度 [m/s]（負で逆走）、
    D: 線からの横ずれ [m]（既定 0）、R: 半径 [m]（既定 0.12。矩形なら半幅）、T_START: 動き出す時刻 [s]（既定 0）、
    LENGTH: 0 なら円盤、正なら車の形の矩形の長さ [m]（既定 0）、PROFILE: 正なら線の速度 × これで走る
    （SPEED は上限。0 なら上限なし。既定 0 = 一定の SPEED）。
    """
    v = [float(x) for x in spec.split(",")]
    if len(v) < 2:
        raise ValueError("--mover は S0,SPEED[,D,R,T_START,LENGTH[,PROFILE]]")
    return Mover(s0=v[0], speed=v[1], d=v[2] if len(v) > 2 else 0.0,
                 r=v[3] if len(v) > 3 else 0.12, t_start=v[4] if len(v) > 4 else 0.0,
                 length=v[5] if len(v) > 5 else 0.0, profile=v[6] if len(v) > 6 else 0.0)
