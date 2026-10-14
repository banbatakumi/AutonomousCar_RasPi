"""IPM（逆透視変換）— カメラ画素 ⇄ 地面座標（`docs/architecture.md` §5.2）。

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

`docs/architecture.md` §5.1 のまま。カメラのローカル座標は
`CameraView.tsx` に合わせて (depth=奥行き, lateral=左, height=下向きが正）。
"""

from __future__ import annotations

import math
from functools import lru_cache
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
    height: float   #: 路面からレンズ中心までの高さ [m]
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
#: 分類の量子化で壁が最大1画素遠くに出るぶんを近い側へ見込む
_WALL_V_OFFSET = 1.5


@lru_cache(maxsize=8)
def _image_rays(height: int, width: int, col_stride: int, v_offset: float,
                intr: CameraIntrinsics) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """全行 × `col_stride` 列おきの画素の光線 `(rx, ry, rz)`（形 `(height, ⌈width/col_stride⌉)`）。

    光線は内部パラメータだけで決まり、毎フレーム変わるのは取付ピッチ（IMU 補正）だけ
    なので、魚眼の逆写像（Newton 法）は解像度ごとに1回で済ませる。
    `v_offset` は画素の中心からのずれ（0.5 ＝ 画素の下端）。
    """
    vs, us = np.mgrid[0:height, 0:width:col_stride].astype(np.float64)
    rays = pixel_ray(us, vs + v_offset, intr)
    for r in rays:
        r.setflags(write=False)
    return rays


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

    **縦は間引かず、各画素の下端からさらに1画素下（`v + 1.5`）を投影する。** カメラが低いので
    地平線付近の 1 行は遠方の数十 cm に当たる。行を間引いたり画素の中心を使ったりすると、
    壁の根元が実際より遠くに出る（＝危険側）。マスクは「画素の中心が壁の向こうか」で
    分類されるので、最後の走行不可の行の下端（`v + 0.5`）と次の行の中心の間に本当の境界が
    あり得る。その分（最大1画素）を近い側へ見込んで、低解像度（160x120 など）でも
    壁が真の距離より遠くに出ないようにする。`stride` は列（横方向）だけを間引く。
    """
    occ = np.zeros((grid.height, grid.width), dtype=bool)
    h, w = drivable_mask.shape
    not_drivable = ~drivable_mask[:, ::stride]
    if not not_drivable.any():
        return occ
    rx, ry, rz = _image_rays(h, w, stride, _WALL_V_OFFSET, intr)
    x, y, valid = _rays_to_ground(rx[not_drivable], ry[not_drivable], rz[not_drivable], ext)
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

    **格子のセルの中心を画像へ順投影し、画像の内側に落ちたセルを観測済みにする。**
    画素を地面へ逆投影して点を打つだけだと、遠方（地平線付近）では 1 行が数十 cm に
    広がるので観測済みのセルが飛び飛びの円弧になり、間が全部「未観測＝壁」になる
    （低いカメラでは 1m 先から先がほぼ塞がる）。加えて `project_mask_to_grid()` と同じ
    画素の点（列は `stride` おき）も観測済みにするので、占有セルは必ず観測済みに含まれる。

    `mask_shape` は `(height, width)`。中身の画素値は使わない。
    """
    h, w = mask_shape
    gc, gr = np.meshgrid(np.arange(grid.width), np.arange(grid.height))
    gx, gy = grid.to_world(gc, gr)
    u, v, valid = _ground_to_pixel_vec(gx, gy, intr, ext)
    seen = valid & (u >= 0.0) & (u < w) & (v >= 0.0) & (v < h)

    rx, ry, rz = _image_rays(h, w, stride, 0.5, intr)
    x, y, ok = _rays_to_ground(rx, ry, rz, ext)
    col, row = grid.to_cell(x[ok], y[ok])
    inside = grid.inside(col, row)
    seen[row[inside], col[inside]] = True
    return seen
