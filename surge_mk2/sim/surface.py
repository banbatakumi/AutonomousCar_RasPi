"""路面の摩擦を場所と時刻で変える（**シム専用**。横グリップの学習 `raspi/nav/grip.py` の検証用）。

`VehicleSpec.mu` は車 1 台に 1 つの値で、場所でも時刻でも変わらない。ここは真値の車に掛ける
倍率を返す: 横（`mu`）と、前後（加速の上限 `drive_accel_m_s2` と、速度指令での減速の上限
`speed_decel_m_s2`）。前後を横と別に指定すると「前後と横で摩擦が違う路面」になる。

    surface = Surface(spec.mu)
    surface.set_base(0.55)                       # 一様に 0.55
    surface.add_disc(2.0, 1.0, 0.8, 0.30)        # (2.0, 1.0) の半径 0.8m だけ 0.30
    surface.add_change(60.0, 0.42)               # 60 秒後に一様の値を 0.42 へ

制動モード（`brake`）の減速は連動させない（周回中の減速は速度指令で出す。制動モードは停止用）。
"""

from __future__ import annotations

__all__ = ["Surface"]


class Surface:
    def __init__(self, spec_mu: float) -> None:
        self.spec_mu = max(float(spec_mu), 1e-6)
        self._base = (1.0, 1.0)
        self._discs: list[tuple[float, float, float, float, float]] = []
        self._changes: list[tuple[float, float, float]] = []

    def _scale(self, mu: float, mu_long: float | None) -> tuple[float, float]:
        lat = float(mu) / self.spec_mu
        return lat, (lat if mu_long is None else float(mu_long) / self.spec_mu)

    def set_base(self, mu: float, mu_long: float | None = None) -> None:
        self._base = self._scale(mu, mu_long)

    def add_disc(self, x: float, y: float, r: float, mu: float, mu_long: float | None = None) -> None:
        """シムの世界座標 `(x, y)` の半径 `r` [m] の円の中だけ摩擦を変える。"""
        self._discs.append((x, y, r * r, *self._scale(mu, mu_long)))

    def add_change(self, t: float, mu: float, mu_long: float | None = None) -> None:
        """時刻 `t` [s] から、円の外の摩擦を変える。"""
        self._changes.append((t, *self._scale(mu, mu_long)))
        self._changes.sort()

    def mu_at(self, x: float, y: float, t: float) -> float:
        return self.scale(x, y, t)[0] * self.spec_mu

    def scale(self, x: float, y: float, t: float) -> tuple[float, float]:
        """`(横の倍率, 前後の倍率)`。円が重なる所は後から足した方。"""
        out = self._base
        for tc, lat, lon in self._changes:
            if t >= tc:
                out = (lat, lon)
        for cx, cy, r2, lat, lon in self._discs:
            if (x - cx) ** 2 + (y - cy) ** 2 <= r2:
                out = (lat, lon)
        return out
