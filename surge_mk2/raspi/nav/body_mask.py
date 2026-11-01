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

__all__ = ["BODY_MARGIN_M", "body_ranges", "mask_body"]

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


def _inside(poly: list[tuple[float, float]], x: float, y: float) -> bool:
    """点が多角形の内側か（偶奇則）。"""
    inside = False
    for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside
