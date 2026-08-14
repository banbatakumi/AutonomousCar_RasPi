"""点群のモーションスキュー補正 — 走りながら測った1周を「1瞬の点群」に直す。

`docs/architecture.md` §14「Phase 3：点群の歪み補正」が求めている処理。

LD06 は 10Hz 回転なので、1周する 100ms の間に車体が動く。**3 m/s なら1周で 30cm**
進み、点群が扇形に歪む。歪んだまま地図に焼くと壁が二重になり、スキャンマッチは
その二重の壁に対して「どちらにも合わない」姿勢を返す。SLAM 精度に直結する。

## 車輪もジャイロも使わない

補正に要る「1周のあいだにどれだけ動いたか」は、**`Slam` が自分の推定姿勢の差分から
出した速度**を渡す。車輪エンコーダは操舵輪である前輪に付いていて、しかも
`encoder_ticks_per_rev` も車輪半径も未実測（`architecture.md` §15 の #3）。
低速では `wheel_speed[]` が使い物にならないことも実測済み（`PROGRESS.md`）。
**「LiDAR 情報のみ」を看板にする以上、ここに車輪の値を混ぜない。**

## 出力は base_link 座標

`config/vehicle.toml` の `[sensors.lidar] x = 0.120` — LiDAR は **base_link
（後輪車軸中心）より 12cm 前**にある。ここで取付オフセットまで畳んでしまい、
**下流（地図・スキャンマッチ・Pure Pursuit）は全部 base_link で統一する。**

「SLAM は lidar フレームで回して最後に base_link へ直す」でも等価だが、
変換を1箇所に閉じ込めた方が、旋回中だけ 12cm ずれる（＝直線は合うのにコーナーで
外れる）という読みにくい取りこぼしが起きない。

## 3種類の点を区別する

| 入力 | 出力 | なぜ |
|---|---|---|
| 実測点 | `hit=True`。**終端に壁を打ち、手前を空きとして彫る** | |
| `saturated[i]`（5.10m 以上） | `hit=False`。**手前を空きとして彫るだけ** | 「5.10m ちょうど」ではない。壁として打つと実在しない円い壁ができる。だが「5.10m までは何も無い」は確かな情報なので捨てない |
| `dist == 0` / `sector_seen=False` | **捨てる**（1点も出さない） | ★下記 |

**`dist == 0` を「空き」として彫ってはいけない。** `raspi/auto/follow_the_gap.py` は
これを `max_range`（空き）として扱っているが、あれは「開けた場所で一歩も動けなく
なるくらいなら、正面の停止距離と超音波で受ける」という取引の上に成り立っている。
地図は違う。**一度「空き」と書いた場所は、後からそこに壁があると分かっても
ヒット/ミス比が汚れたまま残る。** 知らないことは書かない。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from ..msgs.types import Scan

__all__ = ["Points", "deskew", "truncate"]

NS = 1_000_000_000
#: これ以下のヨーレートは直進として扱う [rad/s]。v/w が発散するのを避ける
_STRAIGHT_EPS = 1e-4


class Points(NamedTuple):
    """脱スキュー済みの点群。**base_link 座標・`t_ref_ns` 時点。**

    `x`/`y` は**レイの終端**。`hit[i]` が False の点は「そこまでは空きだが、
    終端に壁があるとは限らない」（飽和点）。
    """

    x: np.ndarray                          #: (N,) [m]
    y: np.ndarray                          #: (N,) [m]
    hit: np.ndarray                        #: (N,) bool。True なら終端に壁を打つ
    #: この点群が「いつの姿勢」のものか。**1周の終端**（＝いちばん新しい時刻）
    t_ref_ns: int
    #: 脱スキューを実際に掛けたか。時刻が無い／速度が無い／止まっている場合は False
    corrected: bool

    def __len__(self) -> int:
        return int(self.x.size)


def _point_times_ns(scan: Scan) -> np.ndarray:
    """車両角 0〜359 の各点の取得時刻 [ns]。時刻が無いセクタは 0。

    `raspi/msgs/types.py` の `Scan` docstring にある式を**そのまま**使う:

        sensor = (360 - deg) % 360                 # センサ角に戻す
        s, i   = 11 - sensor // 30, sensor % 30    # セクタ番号とパケット内の添字
        t_ns   = sector_t_ns[s] + sector_dur_us[s] * 1000 * i / 29

    **`deg` から直に `30*s+j` と分解してはいけない。** LD06 は裏向きに実装されて
    いるので、車両角では**添字が減る向きに時刻が進む**。素直に分解すると `j=0` の
    点で最大1周ぶん（100ms）外し、脱スキューが歪みを増やす方向に働く。
    """
    deg = np.arange(360)
    sensor = (360 - deg) % 360
    s = 11 - sensor // 30
    i = sensor % 30

    t0 = np.asarray(scan.sector_t_ns, dtype=np.int64)[s]
    dur = np.asarray(scan.sector_dur_us, dtype=np.int64)[s]
    t = t0 + (dur * 1000 * i) // 29
    # 時刻が入っていないセクタ（欠測、または合成した点群）は 0 のまま返す
    return np.where(t0 > 0, t, 0)


def deskew(scan: Scan, speed: float = 0.0, yaw_rate: float = 0.0, *,
           mount_x: float = 0.0, mount_y: float = 0.0,
           max_range: float = 12.0) -> Points:
    """1周ぶんの点群を `t_ref_ns` 時点の base_link 座標へ揃える。

    :param speed: 補正に使う車速 [m/s]。**`VehicleState` は受け取らない**（下記）
    :param yaw_rate: 同 ヨーレート [rad/s]
    :param mount_x: LiDAR の取付位置 [m]（`config/vehicle.toml` の `[sensors.lidar]`）
    :param max_range: これより遠い点は「この距離まで空き」として切り詰める [m]

    速度を数値で受け取るのは、**どこから来た速度かをここで決めない**ため。
    `Slam` は自分の推定姿勢の差分から出した速度を渡す（車輪もジャイロも使わない）。
    """
    dist = np.asarray(scan.dist, dtype=np.float64)
    seen = np.asarray(scan.sector_seen, dtype=bool)
    deg = np.arange(360)

    # 車両角 → `sector_seen` の添字。**境界が1つずれている**ので `deg // 30` と
    # 書いてはいけない（`raspi/auto/base.py` の `sector_of_deg` と同じ式）
    sector = ((deg - 1) % 360) // 30
    valid = seen[sector] & (dist > 0.0)

    saturated = scan.saturated
    if saturated is not None:
        sat = np.asarray(saturated, dtype=bool)
    else:
        sat = np.zeros(360, dtype=bool)

    # 射程を超えた点は「そこまでは空き」に読み替える。壁としては打たない
    over = dist > max_range
    rng = np.where(over, max_range, dist)
    hit = valid & ~sat & ~over

    idx = np.flatnonzero(valid)
    if idx.size == 0:
        empty = np.empty(0, dtype=np.float64)
        return Points(empty, empty, np.empty(0, dtype=bool),
                      int(scan.t_capture), False)

    ang = np.radians(deg[idx].astype(np.float64))
    r = rng[idx]
    # センサ座標 → base_link（取付は yaw=0 前提。yaw が付いたら回転を足す）
    px = mount_x + r * np.cos(ang)
    py = mount_y + r * np.sin(ang)

    t_pt = _point_times_ns(scan)[idx]
    t_ref = int(t_pt.max()) if t_pt.size and t_pt.max() > 0 else int(scan.t_capture)

    v, w = float(speed), float(yaw_rate)
    # 時刻が無い（合成した点群・全セクタ欠測）か、そもそも動いていないなら
    # 補正しない。**動いていないのに補正すると、速度のノイズぶんだけ点群を汚す**
    movable = t_pt.min() > 0 and (abs(v) > 1e-3 or abs(w) > _STRAIGHT_EPS)
    if not movable:
        return Points(px, py, hit[idx], t_ref, False)

    # 各点を測ってから基準時刻までに車体が動いた量。**τ ≥ 0**（過去 → 基準）
    tau = (t_ref - t_pt).astype(np.float64) / NS

    if abs(w) > _STRAIGHT_EPS:
        # 等速円運動。R = v/w は旋回半径（符号付き）
        rad = v / w
        th = w * tau
        dx = rad * np.sin(th)
        dy = rad * (1.0 - np.cos(th))
    else:
        th = np.zeros_like(tau)
        dx = v * tau
        dy = np.zeros_like(tau)

    # (dx, dy, th) は「点を測った時刻の車体から見た、基準時刻の車体の姿勢」。
    # 点を基準時刻の座標へ移すのはその**逆変換**
    c, s = np.cos(th), np.sin(th)
    ox, oy = px - dx, py - dy
    return Points(ox * c + oy * s, -ox * s + oy * c, hit[idx], t_ref, True)


def truncate(pts: Points, r: float) -> Points:
    """`r` [m] より遠い点を「そこまでは空き」に切り詰める。**地図に焼く用。**

    照合には全点を使うが、**地図に焼くのは近い点だけ**にする。LD06 の測距誤差は
    σ = 1cm + 0.5%×距離 で、8m では 5cm ＝ 2.5cm 格子の 2 セル。しかも 1° 刻みの
    点間隔が 14cm ＝ 5.6 セルになるので、遠い壁は**点線状の太い滲み**として焼かれる。
    それが次の周の照合を狂わせ、狂った姿勢でまた焼く、という悪循環になる。

    近い点だけで焼いても地図は埋まる。**同じ壁を、そこへ近づいたときに焼けばよい。**
    """
    if r <= 0 or len(pts) == 0:
        return pts
    d = np.hypot(pts.x, pts.y)
    far = d > r
    if not far.any():
        return pts
    scale = np.where(far, r / np.maximum(d, 1e-9), 1.0)
    return Points(pts.x * scale, pts.y * scale, pts.hit & ~far,
                  pts.t_ref_ns, pts.corrected)


def integrate_pose(x: float, y: float, yaw: float,
                   ds: float, dyaw: float) -> tuple[float, float, float]:
    """自転車モデルの1ステップ積分。**スキャンマッチの探索初期値を作る。**

    `ds` は base_link の進んだ弧長 [m]、`dyaw` はその間の回頭 [rad]。
    直線近似ではなく円弧で積むのは、10Hz（100ms）だと 3 m/s・最大舵角で
    30cm・20° 動くため。直線で積むと初期値が数 cm ずれ、粗探索の格子1つぶんを食う。
    """
    if abs(dyaw) < _STRAIGHT_EPS:
        return x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw + dyaw
    rad = ds / dyaw
    nx = x + rad * (math.sin(yaw + dyaw) - math.sin(yaw))
    ny = y - rad * (math.cos(yaw + dyaw) - math.cos(yaw))
    return nx, ny, yaw + dyaw
