"""SLAM のベンチ（`sim/slam_bench.py`）が点群に入れる誤差・妨害（センサの誤差の型・偽の物体・視野の欠け・柱）。

`slam_bench.py` から切り出した（ベンチの本体と、何を注入するかを分ける）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np

from .course import Course
from .params import SimParams

if TYPE_CHECKING:
    from .slam_bench import TruthDriver

__all__ = ["SensorModel", "CAR_SIZE", "car_box", "inject_boxes", "inject_ghost", "inject_pillars", "mask_fov"]


@dataclass(frozen=True)
class SensorModel:
    """推測航法を狂わせる誤差と、LiDAR の意地悪さ。

    ## 値の根拠（外部調査）

    - **MPU6050**: データシートのゼロ点誤差は±20°/s（較正前）、ノイズ密度
      0.005°/s/√Hz。Allan分散の実測報告でバイアス不安定性 0.0013〜0.0024°/s、
      ノイズ 約0.009°/s/√Hz。**起動時の静止較正が前提**で、残るのは温度ドリフト。
      感度（倍率）誤差はデータシートで±3%
    - **LD06**: 0.3m超で誤差±45mm・標準偏差10mm、角度誤差 最大2°、10Hz回転。
      黒い物体は近赤外を吸収するので欠測が増える（大会コースのトンネルは黒壁）
    - **車速**: 前輪アナログ絶対角エンコーダ。分解能が粗く強いローパス。
      低μ板の上ではスリップして読みが伸びる

    `real` は「静止較正はしたが温度ドリフトが残る」程度、`harsh` は
    「較正が甘く、黒壁で欠測が増え、低μ板で滑る」大会当日の上振れを想定する。
    """

    speed_scale: float = 1.0        #: 車速の倍率誤差（車輪半径の誤差・摩耗）
    speed_noise: float = 0.0        #: 車速の白色ノイズ σ [m/s]
    speed_tau: float = 0.0          #: 車速の計測遅れ（1次遅れ）[s]。強いローパスの再現
    speed_quant: float = 0.0        #: 車速の量子化の刻み [m/s]（アナログ絶対角エンコーダの粗さ。0=無し）
    #: テレメトリの時刻の一定のずれ [ms]（時刻同期の残差。+ は実際より未来の時刻を付ける）。
    #: 脱スキューと`pose_now`は`t_capture`で履歴を引くので、ずれると推測航法が狂う
    time_offset_ms: float = 0.0
    gyro_bias_dps: float = 0.0      #: ジャイロのゼロ点ずれ [°/s]
    gyro_noise_dps: float = 0.0     #: ジャイロの白色ノイズ σ [°/s]
    gyro_scale: float = 1.0         #: ジャイロの倍率誤差
    #: スリップ区間。(x0, y0, x1, y1, 倍率) の矩形の中では車速の読みがこの倍率に
    #: なる（低μ板で後輪が空転・前輪が滑る状況の近似）。世界座標
    slip_zones: tuple[tuple[float, float, float, float, float], ...] = ()
    lidar: SimParams = field(default_factory=SimParams)
    #: 壁越しに別の部屋が見える区間。`(段, 開始, 秒数, deg0, deg1, offset)`:
    #: 段（0=地図作成 1=凍結地図での走行）の進捗の割合`開始`から`秒数`のあいだ、
    #: 車両角`deg0`〜`deg1`の点を「コースの外接矩形を`offset`m広げた部屋の壁」までの
    #: 距離に差し替える（低い壁の上を走査面が越えた状況）。地図作成中に入れると
    #: 壁の外が「既知の空き」になり、走行中の越境点が外壁に対応付く危険な状況を作れる
    ghost: tuple[tuple[float, float, float, float, float, float], ...] = ()

    @classmethod
    def preset(cls, name: str, course: Course | None = None) -> "SensorModel":
        if name == "ideal":
            lid = SimParams(lidar_noise_sigma_m=0.0, lidar_noise_rel=0.0,
                            lidar_drop_rate=0.0, lidar_sector_drop_rate=0.0,
                            lidar_delay_jitter_ms=0.0)
            return cls(lidar=lid)
        if name == "real":
            # 静止較正済みのジャイロ（残るのは温度ドリフト）＋ LD06 の実力値
            return cls(speed_scale=1.02, speed_noise=0.02, speed_tau=0.05,
                       gyro_bias_dps=0.5, gyro_noise_dps=0.3, gyro_scale=1.01)
        if name == "harsh":
            # 測距ノイズはデータシート（σ10mm・±45mm）の倍程度に留め、
            # **欠測・スリップ・ジャイロの粗さ**で意地悪をする
            lid = SimParams(lidar_noise_sigma_m=0.015, lidar_noise_rel=0.005,
                            lidar_drop_rate=0.10, lidar_sector_drop_rate=0.03,
                            lidar_delay_jitter_ms=5.0)
            zones: tuple = ()
            if course is not None and course.centerline is not None:
                # 中心線の 1/3 付近に 1.2m 四方の低μ区間を置く（読みが 15% 伸びる）
                cx, cy = course.centerline[len(course.centerline) // 3, :2]
                zones = ((cx - 0.6, cy - 0.6, cx + 0.6, cy + 0.6, 1.15),)
            return cls(speed_scale=0.95, speed_noise=0.05, speed_tau=0.08,
                       gyro_bias_dps=1.5, gyro_noise_dps=1.0, gyro_scale=1.03,
                       slip_zones=zones, lidar=lid)
        raise ValueError(f"unknown preset: {name}")


def _wall_bbox(course: Course) -> tuple[float, float, float, float]:
    rows, cols = np.nonzero(course.grid)
    r = course.resolution
    return (course.origin[0] + cols.min() * r, course.origin[1] + rows.min() * r,
            course.origin[0] + (cols.max() + 1) * r, course.origin[1] + (rows.max() + 1) * r)


def _ray_box_exit(ox: float, oy: float, ang: np.ndarray, box) -> np.ndarray:
    """矩形の内側の点`(ox, oy)`から`ang`方向へ撃ったレイが矩形を出るまでの距離。"""
    x0, y0, x1, y1 = box
    c, s = np.cos(ang), np.sin(ang)
    with np.errstate(divide="ignore"):
        tx = np.where(c > 0, (x1 - ox) / c, np.where(c < 0, (x0 - ox) / c, np.inf))
        ty = np.where(s > 0, (y1 - oy) / s, np.where(s < 0, (y0 - oy) / s, np.inf))
    return np.minimum(tx, ty)


def inject_ghost(scan, pose_lidar: tuple[float, float, float], deg0: float, deg1: float,
                 box, max_range: float) -> None:
    """`scan.dist`の`deg0`〜`deg1`（車両角）を、仮想の部屋の壁`box`までの距離に書き換える。"""
    deg = np.arange(360)
    span = (deg - deg0) % 360
    sel = span <= (deg1 - deg0) % 360
    ang = pose_lidar[2] + np.radians(deg[sel])
    d = _ray_box_exit(pose_lidar[0], pose_lidar[1], ang, box)
    d = np.where(d <= max_range, d, 0.0)
    dist = np.asarray(scan.dist, dtype=float)
    dist[sel] = d
    scan.dist = dist.tolist()
    if scan.saturated is not None:
        sat = np.asarray(scan.saturated, dtype=bool)
        sat[sel] = False
        scan.saturated = sat.tolist()


#: 他車（小さな障害物）の箱の大きさ [m]（長さ, 幅）。自車と同程度の小型車
CAR_SIZE = (0.40, 0.22)


def inject_boxes(scan, pose_lidar: tuple[float, float, float],
                 boxes: list[tuple[float, float, float, float, float]], max_range: float) -> None:
    """`scan.dist`の全周に、有向矩形`(cx, cy, yaw, 長さ, 幅)`（世界座標）を差し込む。

    他車が走査線を遮った状態を作る。レイが矩形に当たり、かつ壁より手前なら距離を書き換える
    （壁の距離0=無効は「無限遠」として扱う）。
    """
    if not boxes:
        return
    ang = pose_lidar[2] + np.radians(np.arange(360))
    dx, dy = np.cos(ang), np.sin(ang)
    dist = np.asarray(scan.dist, dtype=float)
    cur = np.where(dist > 0.0, dist, np.inf)
    for cx, cy, yaw, ln, wd in boxes:
        c, s = math.cos(yaw), math.sin(yaw)
        ox = c * (pose_lidar[0] - cx) + s * (pose_lidar[1] - cy)
        oy = -s * (pose_lidar[0] - cx) + c * (pose_lidar[1] - cy)
        ux, uy = c * dx + s * dy, -s * dx + c * dy
        t0 = np.full(360, -np.inf)
        t1 = np.full(360, np.inf)
        ok = np.ones(360, dtype=bool)
        for o, u, h in ((ox, ux, ln / 2), (oy, uy, wd / 2)):
            par = np.abs(u) < 1e-9
            ok &= ~par | (np.abs(o) <= h)
            with np.errstate(divide="ignore", invalid="ignore"):
                a, b = (-h - o) / u, (h - o) / u
            t0 = np.maximum(t0, np.where(par, -np.inf, np.minimum(a, b)))
            t1 = np.minimum(t1, np.where(par, np.inf, np.maximum(a, b)))
        hit = ok & (t0 <= t1) & (t0 > 0.05) & (t0 < cur) & (t0 <= max_range)
        cur = np.where(hit, t0, cur)
    out = np.where(np.isfinite(cur), cur, 0.0)
    scan.dist = out.tolist()
    if scan.saturated is not None:
        sat = np.asarray(scan.saturated, dtype=bool)
        sat[out != dist] = False
        scan.saturated = sat.tolist()


def mask_fov(scan, fov_deg: float) -> None:
    """`scan.dist`の車両角 ±`fov_deg`/2 の外を 0（無効）にする。"""
    if fov_deg >= 360.0:
        return
    deg = np.arange(360)
    off = np.minimum(deg, 360 - deg)
    dist = np.asarray(scan.dist, dtype=float)
    dist[off > fov_deg / 2.0] = 0.0
    scan.dist = dist.tolist()


def inject_pillars(scan, pillars, mixed_pixel: float, rng: np.random.Generator) -> None:
    """`scan.dist`に、LiDAR 基準で固定された円柱`(車両角, 距離, 直径)`を差し込む。

    縁のレイ（支柱に当たる範囲の両端とその外側1本）は確率`mixed_pixel`で支柱と背景の間の
    一様な距離になる（ビームの一部が支柱・残りが背景に当たったときの LD06 の誤測距の近似）。
    """
    if not pillars:
        return
    dist = np.asarray(scan.dist, dtype=float)
    ang = np.radians(np.arange(360))
    for deg, r, dia in pillars:
        cx, cy = r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))
        proj = cx * np.cos(ang) + cy * np.sin(ang)
        perp2 = r * r - proj * proj
        rad = dia / 2.0
        hit = (proj > 0) & (perp2 <= rad * rad)
        t = proj - np.sqrt(np.maximum(rad * rad - perp2, 0.0))
        bg = dist.copy()
        dist = np.where(hit & ((bg <= 0) | (t < bg)), t, dist)
        if mixed_pixel > 0.0 and hit.any():
            idx = np.nonzero(hit)[0]
            # 360° を跨ぐ支柱もあるので、当たったレイの両隣で当たっていないものを縁とする
            edge = {int(i) for i in idx if not hit[(i - 1) % 360] or not hit[(i + 1) % 360]}
            edge |= {int(j) % 360 for i in idx for j in (i - 1, i + 1) if not hit[j % 360]}
            for i in edge:
                if rng.random() < mixed_pixel:
                    far = bg[i] if bg[i] > 0 else 2.0
                    near = r - rad
                    dist[i] = float(rng.uniform(near, max(near, far)))
    if scan.saturated is not None:
        sat = np.asarray(scan.saturated, dtype=bool)
        sat[dist != np.asarray(scan.dist, dtype=float)] = False
        scan.saturated = sat.tolist()
    scan.dist = dist.tolist()


def car_box(driver: "TruthDriver", s: float, lateral: float,
            backward: bool = False) -> tuple[float, float, float, float, float]:
    """中心線の弧長`s`の地点から左へ`lateral`ずらした位置の他車の箱。向きは中心線の接線（`backward`で逆走）。"""
    n = len(driver.xy)
    f = (s / driver.step) % n
    i = int(f) % n
    a = f - int(f)
    p0, p1 = driver.xy[i], driver.xy[(i + 1) % n]
    p = p0 * (1.0 - a) + p1 * a
    tx, ty = p1[0] - p0[0], p1[1] - p0[1]
    h = math.atan2(ty, tx)
    x = p[0] - math.sin(h) * lateral
    y = p[1] + math.cos(h) * lateral
    return (float(x), float(y), h + (math.pi if backward else 0.0), CAR_SIZE[0], CAR_SIZE[1])


# ── コースと真値の運転手 ───────────────────────────────────────────────────
