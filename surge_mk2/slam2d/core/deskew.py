"""点群のモーションスキュー補正 — 走りながら測った1周を「1瞬の点群」に直す。

`raspi/nav/deskew.py`から移植・一般化。回転式LiDARは1周が瞬時ではないため、
車体が動いていると点群が扇形に歪む。歪んだまま地図に焼くと壁が二重になり、
スキャンマッチはその二重の壁に対して「どちらにも合わない」姿勢を返す。

## 車体依存の値を数値としてだけ受け取る

補正に要る「1周のあいだにどれだけ動いたか」は`Twist2D`として**数値で**
受け取る。どのセンサ由来か（ジャイロ+速度、車輪速+IMU、等速度モデルでの
代用等）はここでは関知しない（`core/motion.py`の`MotionModel`が決める）。

## 3種類の点を区別する

| 入力 | 出力 | なぜ |
|---|---|---|
| 実測点 | `hit=True`。終端に壁を打ち、手前を空きとして彫る | |
| 飽和点（測距上限） | `hit=False`。手前を空きとして彫るだけ | 「ちょうど測距上限」ではない。壁として打つと実在しない壁ができるが、そこまでは何も無いという情報は捨てない |
| 無効点（`valid=False`） | 捨てる（1点も出さない） | 知らないことは書かない。無効を「空き」として彫ると、後からそこに壁があると分かってもヒット/ミス比が汚れたまま残る |
"""

from __future__ import annotations

import numpy as np

from .types import RawScan, ScanPoints, Twist2D

__all__ = ["deskew", "truncate"]

NS = 1_000_000_000
#: これ以下のヨーレート・速度は「動いていない」として扱う
_STRAIGHT_EPS = 1e-4
_STILL_SPEED_EPS = 1e-3


def deskew(raw: RawScan, twist: Twist2D, *,
           mount_x: float = 0.0, mount_y: float = 0.0,
           max_range: float = float("inf")) -> ScanPoints:
    """1周ぶんの点群を`t_ref_ns`（点群中でいちばん新しい時刻）へ揃える。

    :param twist: この周のあいだ一定だったと仮定する、車体座標系での速度
    :param mount_x: センサの取付位置オフセット[m]（車体前方+）
    :param mount_y: 同、車体左方+
    :param max_range: これより遠い点は「この距離まで空き」として切り詰める[m]
    """
    valid = raw.valid
    idx = np.flatnonzero(valid)
    if idx.size == 0:
        empty = np.empty(0, dtype=np.float64)
        return ScanPoints(empty, empty, np.empty(0, dtype=bool), 0, False)

    ang = raw.angles[idx]
    r = raw.ranges[idx]
    sat = raw.saturated[idx]
    over = ~np.isfinite(r) | (r > max_range)
    rng = np.where(over, max_range, r)
    hit = ~sat & ~over

    # センサ座標 → 車体座標（取付は yaw=0 前提）
    px = mount_x + rng * np.cos(ang)
    py = mount_y + rng * np.sin(ang)

    t_pt = raw.t_point_ns[idx]
    t_ref = int(t_pt.max()) if t_pt.size and t_pt.max() > 0 else 0

    v, vy, w = twist.vx, twist.vy, twist.yaw_rate
    # 時刻が無い（合成した点群等）か、そもそも動いていないなら補正しない。
    # **動いていないのに補正すると、速度のノイズぶんだけ点群を汚す**
    movable = (t_pt.min() > 0
               and (abs(v) > _STILL_SPEED_EPS or abs(vy) > _STILL_SPEED_EPS
                    or abs(w) > _STRAIGHT_EPS))
    if not movable:
        return ScanPoints(px, py, hit, t_ref, False)

    # 各点を測ってから基準時刻までに車体が動いた量。tau >= 0（過去 → 基準）
    tau = (t_ref - t_pt).astype(np.float64) / NS

    if abs(w) > _STRAIGHT_EPS:
        wt = w * tau
        s, c = np.sin(wt), np.cos(wt)
        dx = (v * s + vy * (c - 1.0)) / w
        dy = (v * (1.0 - c) + vy * s) / w
    else:
        wt = np.zeros_like(tau)
        dx = v * tau
        dy = vy * tau

    # (dx, dy, wt) は「点を測った時刻の車体から見た、基準時刻の車体の姿勢」。
    # 点を基準時刻の座標へ移すのはその**逆変換**（`core/types.py`の`inverse`と同じ式）
    c, s = np.cos(wt), np.sin(wt)
    ox, oy = px - dx, py - dy
    return ScanPoints(ox * c + oy * s, -ox * s + oy * c, hit, t_ref, True)


def truncate(pts: ScanPoints, r: float) -> ScanPoints:
    """`r`[m]より遠い点を「そこまでは空き」に切り詰める。**地図に焼く用。**

    照合には全点を使うが、地図に焼くのは近い点だけにする。測距誤差は距離が
    伸びるほど大きくなるセンサが多く、遠い壁は点線状の太い滲みとして焼かれ、
    次の周の照合を狂わせる悪循環になりうる。近い点だけで焼いても地図は
    埋まる（同じ壁を、そこへ近づいたときに焼けばよい）。
    """
    if r <= 0 or len(pts) == 0:
        return pts
    d = np.hypot(pts.x, pts.y)
    far = d > r
    if not far.any():
        return pts
    scale = np.where(far, r / np.maximum(d, 1e-9), 1.0)
    return ScanPoints(pts.x * scale, pts.y * scale, pts.hit & ~far,
                      pts.t_ref_ns, pts.corrected)
