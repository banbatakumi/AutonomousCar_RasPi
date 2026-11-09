"""車体の内側に落ちた LiDAR の点を捨てる。

2026-10-31 の会場の試走（`~/Downloads/toyota` の 9 本）で、全走行で 1 走行 2〜9 回「前方 0.0m に障害物。停止」が
出ていた。障害物の位置は車体座標で (0.13〜0.17, 0.01)・(−0.04, 0.05) ＝ **車体の外形の内側**。生の `/scan` には
LiDAR から 3〜15cm の点があり、車体の外形の内側に点を持つスキャンが 22〜27%（車体の部品・配線の反射）。
1 回で 1.1→0.4m/s 程度まで落ち、ABS の作動はほぼこれ、学習の周も無効にしていた。

車体の内側に壁や障害物があることは無い（あれば既にぶつかっている）ので、**外形＋`margin` より LiDAR に近い点は
「測れなかった」（`dist = 0`）にする。** 「空き」にはしない: その向きの車体の外は見えていない
（`raspi/nav/deskew.py` 冒頭の「3種類の点を区別する」）。

LiDAR は外形の内側にあるので、各向きのレイは外形を 1 回だけ出る。その距離を 1° ごとに前計算しておき、
スキャンごとには比較 1 回で済ませる。
"""

from __future__ import annotations

import math

import msgspec
import numpy as np

from ..msgs.types import Scan

__all__ = ["BODY_MARGIN_M", "OBSTACLE_BODY_EXTRA_M", "body_ranges", "mask_body", "near_body"]

#: 外形からこれだけ外までは車体の点とみなす [m]。LD06 の測距誤差（近距離で σ≈1cm）の 2σ。
#: 外形の数 cm 外に本物の物があっても、そこはもう止まれない距離なので捨てても失うものは無い
BODY_MARGIN_M = 0.02


def body_ranges(footprint, mount_x: float, mount_y: float, margin: float = BODY_MARGIN_M) -> np.ndarray:
    """車両角 0〜359° の各レイが、LiDAR から車体の外形（＋`margin`）を出るまでの距離 [m]。(360,)

    :param footprint: 車体外形の多角形（base_link、`config/vehicle.toml` の `footprint`）。凸でなくてよい
        （最も遠い交点を採る）。空なら全部 0（何も捨てない）
    :param mount_x: LiDAR の取付位置（base_link）[m]。外形の外なら全部 0
    """
    out = np.zeros(360)
    poly = [(float(x), float(y)) for x, y in (footprint or ())]
    if len(poly) < 3 or not _inside(poly, mount_x, mount_y):
        return out
    for deg in range(360):
        a = math.radians(deg)
        dx, dy = math.cos(a), math.sin(a)
        best = 0.0
        for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
            # mount + t·d = p0 + u·(p1 − p0)、t ≥ 0・0 ≤ u ≤ 1
            ex, ey = x1 - x0, y1 - y0
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            qx, qy = x0 - mount_x, y0 - mount_y
            t = (qx * ey - qy * ex) / den
            u = (qx * dy - qy * dx) / den
            if t >= 0.0 and -1e-9 <= u <= 1.0 + 1e-9:
                best = max(best, t)
        # 外形の辺に斜めに当たるレイほど、外形から margin 離れた線までの距離は長い（1/cos）。
        # ここでは向きによらず margin を足すだけにする（角の近くで数 mm 短いが、点の誤差より小さい）
        out[deg] = best + margin
    return out


def mask_body(scan: Scan, ranges: np.ndarray) -> Scan:
    """`ranges`（`body_ranges`）より近い点を `dist = 0`（測れなかった）にした `scan` を返す。

    捨てる点が無ければ `scan` をそのまま返す（写しを作らない）。
    """
    dist = np.asarray(scan.dist, dtype=np.float64)
    if dist.shape != ranges.shape:
        return scan
    drop = (dist > 0.0) & (dist < ranges)
    if not drop.any():
        return scan
    d = dist.copy()
    d[drop] = 0.0
    sat = scan.saturated
    if sat is not None:
        sat = [bool(s) and not k for s, k in zip(sat, drop)]
    inten = scan.intensity
    if inten is not None and len(inten) == d.size:
        inten = [0 if k else int(v) for v, k in zip(inten, drop)]
    return msgspec.structs.replace(scan, dist=d.tolist(), saturated=sat, intensity=inten)


#: 障害物の判定（`near_body`）だけで、車体の外形（＋`BODY_MARGIN_M`）からさらにこれだけ外までの点を使わない [m]。
#: ★2026-11-08 の試走で、ヘアピンなどのコーナーごとに、車の基準点から 0.11〜0.25m（車体の側面の数 cm 外）に
#: LiDAR から 0.12〜0.22m の点が 2〜5 点まとまって出て障害物になり、「障害物を避ける」を入れると毎回そこで止まった
#: （近い物は 2 周期の確認を待たずに扱うため）。前の周にも続けて出るので、点群のノイズの除去では消えない。
#: そこに本物の物があっても、もう避けることも止まることもできない距離なので、障害物として扱っても得るものが無い。
#: SLAM の位置合わせには残す（頂点で壁に寄った所の点を失わない）
OBSTACLE_BODY_EXTRA_M = 0.10


def near_body(x: np.ndarray, y: np.ndarray, ranges: np.ndarray, mount_x: float, mount_y: float,
              extra: float = OBSTACLE_BODY_EXTRA_M) -> np.ndarray:
    """base_link 座標の点 `(x, y)` のうち、LiDAR から見てその向きの車体の外形（`ranges`）＋`extra` より近いもの。"""
    dx = np.asarray(x, dtype=np.float64) - mount_x
    dy = np.asarray(y, dtype=np.float64) - mount_y
    r = np.hypot(dx, dy)
    deg = np.round(np.degrees(np.arctan2(dy, dx))).astype(np.int64) % 360
    return r < ranges[deg] + extra


def _inside(poly: list[tuple[float, float]], x: float, y: float) -> bool:
    """点が多角形の内側か（偶奇則）。"""
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside
