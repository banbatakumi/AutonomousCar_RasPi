"""IPM（逆透視変換）— カメラ画素 ⇄ 地面座標（`docs/architecture.md` §5.2）。

`gui/src/render/CameraView.tsx` の `drawGuide()`/`project()` が「地面座標 →
画素」の順方向を実車で検証済みのまま持っている。ここではそれと**同一の
カメラモデル**を使い、逆方向（画素 → 地面座標）を足す。カメラモデルを
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

## レンズモデル（2026-10 魚眼化）

IMX219 160° 広角レンズに替えたので、`vehicle.toml` に校正値
（`[sensors.cam_*.fisheye]`）があれば Kannala-Brandt の魚眼モデル
（`raspi/core/camera_model.py`）で、無ければ従来の `hfov` ピンホールで投影する。
どちらも「画素 ⇄ カメラ座標の光線」の部分だけが違い、光線を取付ピッチで回して
地面と交差させる部分は共通。呼び出し側は `vehicle_camera_intrinsics()` だけを使う。

## 座標系

`docs/architecture.md` §5.1 のまま。カメラのローカル座標は
`CameraView.tsx` に合わせて (depth=奥行き, lateral=左, height=下向きが正）。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from raspi.core.camera_model import FisheyeCalib, project_rays, unproject_pixels

from .grid import OccGrid

__all__ = ["CameraIntrinsics", "CameraExtrinsics", "camera_intrinsics",
           "vehicle_camera_intrinsics", "pixel_ray", "pixel_to_ground",
           "ground_to_pixel", "project_mask_to_grid", "project_seen_to_grid"]

#: `project()`/`pixel_to_ground()` 共通の下限。光軸方向の距離がこれ未満は
#: 「カメラの真下・背後」とみなして解を捨てる（`CameraView.tsx` の `zc < 0.15` と同じ）
_MIN_ZC = 0.15


class CameraIntrinsics(NamedTuple):
    """内部パラメータ。`camera_intrinsics()` が作る。

    `k` が None ならピンホール（未校正。`hfov` からの近似）、4要素なら
    Kannala-Brandt の魚眼（`raspi/core/camera_model.py`、校正済み）。
    """

    f: float            #: 水平の焦点距離 [px]（fx）
    cx: float           #: 光軸の水平画素位置
    principal_y: float  #: 光軸の垂直画素位置（下端クロップぶん下にずれる）
    #: 垂直の焦点距離 [px]（fy）。None なら `f` と同じ（正方画素のピンホール）
    fy: float | None = None
    #: 魚眼の歪み係数 (k1..k4)。None ならピンホール
    k: tuple[float, float, float, float] | None = None

    @property
    def f_y(self) -> float:
        return self.f if self.fy is None else self.fy


class CameraExtrinsics(NamedTuple):
    """外部パラメータ。base_link 基準（`Vehicle.cam_front_*` 等から作る）。"""

    x: float        #: [m]
    y: float        #: [m]
    height: float   #: 地面からの高さ [m]（★上記の近似）
    pitch: float    #: [rad] 下向きが正
    yaw: float      #: [rad]


def camera_intrinsics(hfov_rad: float, width: int, height: int,
                       bottom_crop: float = 0.0,
                       fisheye: FisheyeCalib | None = None) -> CameraIntrinsics:
    """内部パラメータを作る。

    `fisheye`（校正値）があればそれを `width`×`height` に縮尺して使う。
    無ければ `CameraView.tsx` の `drawGuide()` と同一のピンホールの式
    （`hfov` から焦点距離を出す）。**独自に再導出しない。**
    """
    if fisheye is not None:
        fx, fy, cx, cy = fisheye.scaled(width, height, bottom_crop)
        return CameraIntrinsics(f=fx, cx=cx, principal_y=cy, fy=fy, k=fisheye.k)
    f = (width / 2.0) / math.tan(hfov_rad / 2.0)
    cx = width / 2.0
    principal_y = height / (2.0 * (1.0 - bottom_crop))
    return CameraIntrinsics(f=f, cx=cx, principal_y=principal_y)


def vehicle_camera_intrinsics(vehicle, cam: str, width: int,
                              height: int) -> CameraIntrinsics:
    """`Vehicle` から前（`cam="front"`）/後カメラの内部パラメータを作る。

    各ノードが `hfov`・`bottom_crop`・`fisheye` を個別に拾うと、どれか1つを
    渡し忘れた（＝校正値を無視した）ノードが黙って古いモデルで動く。口を1つにする。
    """
    return camera_intrinsics(getattr(vehicle, f"cam_{cam}_hfov"), width, height,
                             getattr(vehicle, f"cam_{cam}_bottom_crop"),
                             getattr(vehicle, f"cam_{cam}_fisheye"))


def pixel_ray(u, v, intr: CameraIntrinsics):
    """画素 → カメラ座標の光線 `(x=右, y=下, z=光軸)`。numpy 配列・スカラ両対応。"""
    return unproject_pixels(u, v, intr.f, intr.f_y, intr.cx, intr.principal_y, intr.k)


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

    cp, sp = math.cos(ext.pitch), math.sin(ext.pitch)
    zc = depth * cp + ext.height * sp          # 光軸方向
    if zc < _MIN_ZC:
        return None
    yc = -depth * sp + ext.height * cp          # 下向きが正
    u, v = project_rays(-lateral, yc, zc, intr.f, intr.f_y, intr.cx,
                        intr.principal_y, intr.k)
    return float(u), float(v)


def pixel_to_ground(u: float, v: float, intr: CameraIntrinsics,
                     ext: CameraExtrinsics) -> tuple[float, float] | None:
    """画素 `(u, v)` → base_link 座標の地面点 `(x, y)`。

    地平線より上（地面との交点が無い）は `None`。画素を光線に戻し
    （`pixel_ray`）、取付ピッチで回して地面（カメラの `height` 下の平面）と
    交差させる——導出は `raspi/tests/test_ipm.py` の往復テストで検証する。
    """
    x, y, valid = _pixel_to_ground_vec(np.array([u], dtype=np.float64),
                                       np.array([v], dtype=np.float64), intr, ext)
    if not valid[0]:
        return None
    return float(x[0]), float(y[0])


def _pixel_to_ground_vec(u: np.ndarray, v: np.ndarray, intr: CameraIntrinsics,
                          ext: CameraExtrinsics
                          ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`pixel_to_ground()` のベクトル化版。`(x, y, valid)` を返す。"""
    cp, sp = math.cos(ext.pitch), math.sin(ext.pitch)
    rx, ry, rz = pixel_ray(u.astype(np.float64), v.astype(np.float64), intr)

    # カメラ座標の光線 → ヨーだけ戻したローカル座標（奥行き・下向き・左）
    down = rz * sp + ry * cp
    valid = down > 1e-6                         # 地平線より上は地面と交わらない
    t = ext.height / np.where(valid, down, 1.0)
    depth = t * (rz * cp - ry * sp)
    valid &= depth > 0.0
    zc = t * rz                                 # 光軸方向の距離
    valid &= zc >= _MIN_ZC
    lateral = -t * rx

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


def project_seen_to_grid(mask_shape: tuple[int, int], intr: CameraIntrinsics,
                          ext: CameraExtrinsics, grid: OccGrid,
                          *, stride: int = 2) -> np.ndarray:
    """カメラが実際に見ている範囲を occupancy grid へ投影する。**クラスは問わない。**

    `project_mask_to_grid()` は「走行不可」の画素だけを壁として彫るので、
    水平画角の外や地平線より上（そもそも `pixel_to_ground` が解を持たない範囲）は
    暗黙に「空き」のまま残る。`follow_the_gap_cam.py` は `fov_deg` を絞ってこの穴を
    回避しているが、車両の正面軸から離れた前方距離まで左右にレイを撃つ
    `raspi/nav/drivable_path.py` の中心線抽出では同じ回避策が使えない
    （そもそも角度で視野を切っていない）。

    返り値は `grid` と同じ形の bool 配列（True＝観測済み）。呼び出し側で
    `blocked = (~seen) | occ` を作り、**未観測は壁と同じ扱いにする**
    （`scan_window()` の「欠測は壁」と同じ安全側の判断）。

    `mask_shape` は `(height, width)`。中身の画素値は使わない——全画素を
    対象に「地面座標へ投影できたか」だけを見る。`project_mask_to_grid()` と
    同じ `stride` を渡せば、同じ画素位置の組を2回サンプルすることになり
    座標系が完全に一致する。
    """
    h, w = mask_shape
    seen = np.zeros((grid.height, grid.width), dtype=bool)
    vs, us = np.mgrid[0:h:stride, 0:w:stride]
    us = us.astype(np.float64).reshape(-1)
    vs = vs.astype(np.float64).reshape(-1)

    x, y, valid = _pixel_to_ground_vec(us, vs, intr, ext)
    x, y = x[valid], y[valid]
    if x.size == 0:
        return seen

    col, row = grid.to_cell(x, y)
    inside = grid.inside(col, row)
    seen[row[inside], col[inside]] = True
    return seen
