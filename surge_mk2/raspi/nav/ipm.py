"""IPM（逆透視変換）— カメラ画素 ⇄ 地面座標（`docs/architecture.md` §5.2）。

`gui/src/render/CameraView.tsx` の `drawGuide()`/`project()` が「地面座標 →
画素」の順方向を実車で検証済みのまま持っている。ここではそれと**同一の
ピンホールモデル**を使い、逆方向（画素 → 地面座標）を足す。カメラモデルを
ここで独自に再導出しないのは、`scan_window()` が LiDAR の点群の読み方の
契約そのものであるのと同じ理由——片方だけ直すと、もう片方（GUI の進路
ガイド）が古いモデルのまま表示され続ける。

## 高さは base_link の z をそのまま「地面からの高さ」とみなす近似

`config/vehicle.toml` の `cam_front.z` は base_link 基準の取付高さであって
「地面からの高さ」の実測値ではない（`measured = false`）。base_link は
後輪車軸中心なので、真の地面からの高さは本来 `z + wheel_radius` 程度になる
はずだが、その差は数cmで、遠方ほど誤差が拡大する IPM の性質を踏まえても
初期スキャフォールドとしては許容範囲とする。実測較正は保留事項
（`docs/plans` のカメラピッチ較正の項）。

## 座標系

`docs/architecture.md` §5.1 のまま。カメラのローカル座標は
`CameraView.tsx` に合わせて (depth=奥行き, lateral=左, height=下向きが正）。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .grid import OccGrid

__all__ = ["CameraIntrinsics", "CameraExtrinsics", "camera_intrinsics",
           "pixel_to_ground", "ground_to_pixel", "project_mask_to_grid"]

#: `project()`/`pixel_to_ground()` 共通の下限。光軸方向の距離がこれ未満は
#: 「カメラの真下・背後」とみなして解を捨てる（`CameraView.tsx` の `zc < 0.15` と同じ）
_MIN_ZC = 0.15


class CameraIntrinsics(NamedTuple):
    """内部パラメータ。`camera_intrinsics()` が作る。"""

    f: float            #: 焦点距離 [px]
    cx: float           #: 光軸の水平画素位置
    principal_y: float  #: 光軸の垂直画素位置（下端クロップぶん下にずれる）


class CameraExtrinsics(NamedTuple):
    """外部パラメータ。base_link 基準（`Vehicle.cam_front_*` 等から作る）。"""

    x: float        #: [m]
    y: float        #: [m]
    height: float   #: 地面からの高さ [m]（★上記の近似）
    pitch: float    #: [rad] 下向きが正
    yaw: float      #: [rad]


def camera_intrinsics(hfov_rad: float, width: int, height: int,
                       bottom_crop: float = 0.0) -> CameraIntrinsics:
    """`CameraView.tsx` の `drawGuide()` と同一の式。**独自に再導出しない。**"""
    f = (width / 2.0) / math.tan(hfov_rad / 2.0)
    cx = width / 2.0
    principal_y = height / (2.0 * (1.0 - bottom_crop))
    return CameraIntrinsics(f=f, cx=cx, principal_y=principal_y)


def ground_to_pixel(x: float, y: float, intr: CameraIntrinsics,
                     ext: CameraExtrinsics) -> tuple[float, float] | None:
    """base_link 座標の地面点 `(x, y)` → 画素 `(u, v)`。

    `CameraView.tsx` の `project()` と同じ式（順方向）。`pixel_to_ground()` の
    逆変換なので、キャリブレーション確認・テストの往復に使う。
    """
    dx, dy = x - ext.x, y - ext.y
    cy, sy = math.cos(ext.yaw), math.sin(ext.yaw)
    # base_link 座標 → カメラのヨーだけ戻したローカル座標 (depth, lateral)
    depth = dx * cy + dy * sy
    lateral = -dx * sy + dy * cy

    f, cx, principal_y = intr
    cp, sp = math.cos(ext.pitch), math.sin(ext.pitch)
    zc = depth * cp + ext.height * sp          # 光軸方向
    if zc < _MIN_ZC:
        return None
    yc = -depth * sp + ext.height * cp          # 下向きが正
    return cx - (lateral * f) / zc, principal_y + (yc * f) / zc


def pixel_to_ground(u: float, v: float, intr: CameraIntrinsics,
                     ext: CameraExtrinsics) -> tuple[float, float] | None:
    """画素 `(u, v)` → base_link 座標の地面点 `(x, y)`。

    地平線より上（地面との交点が無い）は `None`。`ground_to_pixel()` の式を
    `depth` について解いたもの——導出は `raspi/tests/test_ipm.py` の往復
    テストで検証する。
    """
    f, cx, principal_y = intr
    p = ext.pitch
    cp, sp = math.cos(p), math.sin(p)
    k = (v - principal_y) / f
    denom = sp + k * cp
    if denom <= 1e-6:
        return None                            # 地平線より上（地面と交わらない）
    depth = ext.height * (cp - k * sp) / denom
    if depth <= 0.0:
        return None
    zc = depth * cp + ext.height * sp
    if zc < _MIN_ZC:
        return None
    lateral = (cx - u) * zc / f

    cy, sy = math.cos(ext.yaw), math.sin(ext.yaw)
    x = ext.x + depth * cy - lateral * sy
    y = ext.y + depth * sy + lateral * cy
    return x, y


def _pixel_to_ground_vec(u: np.ndarray, v: np.ndarray, intr: CameraIntrinsics,
                          ext: CameraExtrinsics
                          ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`pixel_to_ground()` のベクトル化版。`(x, y, valid)` を返す。"""
    f, cx, principal_y = intr
    cp, sp = math.cos(ext.pitch), math.sin(ext.pitch)
    u = u.astype(np.float64)
    v = v.astype(np.float64)

    k = (v - principal_y) / f
    denom = sp + k * cp
    valid = denom > 1e-6
    safe_denom = np.where(valid, denom, 1.0)
    depth = ext.height * (cp - k * sp) / safe_denom
    valid &= depth > 0.0

    zc = depth * cp + ext.height * sp
    valid &= zc >= _MIN_ZC
    lateral = (cx - u) * zc / f

    cy, sy = math.cos(ext.yaw), math.sin(ext.yaw)
    x = ext.x + depth * cy - lateral * sy
    y = ext.y + depth * sy + lateral * cy
    return x, y, valid


def project_mask_to_grid(drivable_mask: np.ndarray, intr: CameraIntrinsics,
                          ext: CameraExtrinsics, grid: OccGrid,
                          *, stride: int = 2) -> np.ndarray:
    """走行不可マスク（`drivable_mask == False` の画素）を occupancy grid へ投影する。

    `drivable_mask` は `(height, width)` の bool 配列。**True ＝ 走行可能。**
    返り値は `grid` と同じ形の bool 配列（True ＝ 占有）で、そのまま
    `OccGrid.raycast(..., mask=占有配列)` に渡せる。

    地平線より上の画素（`pixel_to_ground` が解を持たない）は無視する
    （＝空きでも壁でもなく「その先は分からない」——`raycast` 側で
    `max_range` が受け止める。`scan_window()` の「欠測は壁」とは違う扱いに
    なる点に注意: ここは"見えている範囲の外"であって"欠測"ではない）。

    `stride` で画素を間引く。IPM は近距離ほど画素が密になるので全画素を
    使う必要はなく、間引かないと大きな画像で遅くなる。
    """
    occ = np.zeros((grid.height, grid.width), dtype=bool)
    not_drivable = ~drivable_mask[::stride, ::stride]
    vs, us = np.nonzero(not_drivable)
    if vs.size == 0:
        return occ
    vs = (vs * stride).astype(np.float64)
    us = (us * stride).astype(np.float64)

    x, y, valid = _pixel_to_ground_vec(us, vs, intr, ext)
    x, y = x[valid], y[valid]
    if x.size == 0:
        return occ

    col, row = grid.to_cell(x, y)
    inside = grid.inside(col, row)
    occ[row[inside], col[inside]] = True
    return occ
