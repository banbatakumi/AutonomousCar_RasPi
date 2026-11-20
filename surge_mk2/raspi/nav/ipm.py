"""IPM（逆透視変換）— カメラ画素 ⇄ 地面座標（`docs/system_overview.md` §7）。

`gui/src/render/CameraView.tsx` の `drawGuide()`/`project()` が「地面座標 →
画素」の順方向を実車で検証済みのまま持っている。ここではそれと**同一の
カメラモデル**を使い、逆方向（画素 → 地面座標）を足す。カメラモデルを
ここで独自に再導出しないのは、`scan_window()` が LiDAR の点群の読み方の
契約そのものであるのと同じ理由——片方だけ直すと、もう片方（GUI の進路
ガイド）が古いモデルのまま表示され続ける。

## 高さは路面からレンズ中心まで

`config/vehicle.toml` のレンズのプロファイルの `z`（`[sensors.cam_*.lenses.<レンズ>]`）は
**路面からレンズ中心までの高さ**（実測）で、そのまま `CameraExtrinsics.height` に入れる。
x・y は base_link（後輪車軸中心）基準。カメラが低い（約 9cm）ので、地平線付近では
1 画素・1° の差が大きな距離の差になる（2m 先で 1 画素 ≒ 25cm）。

## レンズモデル（2026-10 魚眼化）

IMX219 160° 広角レンズに替えたので、`vehicle.toml` に校正値
（選ばれたレンズの `fisheye`、`[sensors.cam_*.lenses.<レンズ>.fisheye]`）があれば Kannala-Brandt の魚眼モデル
（`raspi/core/camera_model.py`）で、無ければ従来の `hfov` ピンホールで投影する。
どちらも「画素 ⇄ カメラ座標の光線」の部分だけが違い、光線を取付ピッチで回して
地面と交差させる部分は共通。呼び出し側は `vehicle_camera_intrinsics()` だけを使う。

## 座標系

`docs/system_overview.md` §7 のまま。カメラのローカル座標は
`CameraView.tsx` に合わせて (depth=奥行き, lateral=左, height=下向きが正）。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from raspi.core.camera_model import FisheyeCalib, crop_frame, project_rays, unproject_pixels

__all__ = ["CameraIntrinsics", "CameraExtrinsics", "camera_intrinsics",
           "vehicle_camera_intrinsics", "pixel_ray", "pixel_to_ground",
           "ground_to_pixel"]

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
    height: float   #: 路面からレンズ中心までの高さ [m]
    pitch: float    #: [rad] 下向きが正
    yaw: float      #: [rad]


def camera_intrinsics(hfov_rad: float, width: int, height: int,
                       bottom_crop: float = 0.0,
                       fisheye: FisheyeCalib | None = None,
                       top_crop: float = 0.0) -> CameraIntrinsics:
    """内部パラメータを作る。

    `fisheye`（校正値）があればそれを `width`×`height` に縮尺して使う。
    無ければ `CameraView.tsx` の `drawGuide()` と同一のピンホールの式
    （`hfov` から焦点距離を出す）。**独自に再導出しない。**
    """
    if fisheye is not None:
        fx, fy, cx, cy = fisheye.scaled(width, height, bottom_crop, top_crop)
        return CameraIntrinsics(f=fx, cx=cx, principal_y=cy, fy=fy, k=fisheye.k)
    f = (width / 2.0) / math.tan(hfov_rad / 2.0)
    cx = width / 2.0
    full_h, y0 = crop_frame(height, bottom_crop, top_crop)
    principal_y = full_h / 2.0 - y0
    return CameraIntrinsics(f=f, cx=cx, principal_y=principal_y)


def vehicle_camera_intrinsics(vehicle, cam: str, width: int,
                              height: int) -> CameraIntrinsics:
    """`Vehicle` から右・主（`cam="front"`）/左（`cam="left"`）カメラの内部パラメータを作る。

    各ノードが `hfov`・`top_crop`・`bottom_crop`・`fisheye` を個別に拾うと、どれか1つを
    渡し忘れた（＝校正値を無視した）ノードが黙って古いモデルで動く。口を1つにする。
    """
    return camera_intrinsics(getattr(vehicle, f"cam_{cam}_hfov"), width, height,
                             getattr(vehicle, f"cam_{cam}_bottom_crop"),
                             getattr(vehicle, f"cam_{cam}_fisheye"),
                             top_crop=getattr(vehicle, f"cam_{cam}_top_crop"))


def pixel_ray(u, v, intr: CameraIntrinsics):
    """画素 → カメラ座標の光線 `(x=右, y=下, z=光軸)`。numpy 配列・スカラ両対応。"""
    return unproject_pixels(u, v, intr.f, intr.f_y, intr.cx, intr.principal_y, intr.k)


def _ground_to_pixel_vec(x, y, intr: CameraIntrinsics, ext: CameraExtrinsics):
    """`ground_to_pixel()` のベクトル化版。`(u, v, valid)`（valid＝カメラの前方 `_MIN_ZC` 以上）。"""
    dx = np.asarray(x, dtype=np.float64) - ext.x
    dy = np.asarray(y, dtype=np.float64) - ext.y
    cy, sy = math.cos(ext.yaw), math.sin(ext.yaw)
    # base_link 座標 → カメラのヨーだけ戻したローカル座標 (depth, lateral)
    depth = dx * cy + dy * sy
    lateral = -dx * sy + dy * cy

    cp, sp = math.cos(ext.pitch), math.sin(ext.pitch)
    zc = depth * cp + ext.height * sp          # 光軸方向
    valid = zc >= _MIN_ZC
    yc = -depth * sp + ext.height * cp          # 下向きが正
    u, v = project_rays(-lateral, yc, np.where(valid, zc, 1.0), intr.f, intr.f_y,
                        intr.cx, intr.principal_y, intr.k)
    return u, v, valid


def ground_to_pixel(x: float, y: float, intr: CameraIntrinsics,
                     ext: CameraExtrinsics) -> tuple[float, float] | None:
    """base_link 座標の地面点 `(x, y)` → 画素 `(u, v)`。

    `CameraView.tsx` の `project()` と同じ式（順方向）。`pixel_to_ground()` の
    逆変換なので、キャリブレーション確認・テストの往復に使う。
    """
    u, v, valid = _ground_to_pixel_vec(x, y, intr, ext)
    if not bool(valid):
        return None
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
    rx, ry, rz = pixel_ray(u.astype(np.float64), v.astype(np.float64), intr)
    return _rays_to_ground(rx, ry, rz, ext)


#: 走行不可の画素を地面へ落とすときの縦のずれ（画素単位。0.5 ＝ 画素の下端）。

def _rays_to_ground(rx: np.ndarray, ry: np.ndarray, rz: np.ndarray,
                    ext: CameraExtrinsics) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """カメラ座標の光線 → base_link 座標の地面点 `(x, y, valid)`。"""
    cp, sp = math.cos(ext.pitch), math.sin(ext.pitch)

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
