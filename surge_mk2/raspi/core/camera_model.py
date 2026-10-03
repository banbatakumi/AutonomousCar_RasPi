"""カメラのレンズモデル — 魚眼（Kannala-Brandt）とピンホールの共通の式。

    from raspi.core.camera_model import FisheyeCalib, project_rays, unproject_pixels

IMX219 160° 広角レンズに替えたことで、従来の「歪みのないピンホール + `hfov`」では
画像の端ほど大きくズレるようになった（160° はピンホールでは原理的に表せない）。
チェッカーボード校正（Mac 側 `tools/cam_calib/`）で求めた値を `config/vehicle.toml`
の `[sensors.cam_*.fisheye]` に置き、IPM（`raspi/nav/ipm.py`）・GUI の進路ガイド
（`gui/src/render/CameraView.tsx`）・補正映像（`telemetry_node`）がここの式を使う。

## モデルは OpenCV の `cv2.fisheye` と同一（Kannala-Brandt, k1..k4）

光軸からの角度 θ に対して、画像上の（正規化）半径を

    θd = θ (1 + k1 θ² + k2 θ⁴ + k3 θ⁶ + k4 θ⁸)
    u = fx · θd · (x / r) + cx,   v = fy · θd · (y / r) + cy     （r = √(x²+y²)）

とする。通常の `cv2.calibrateCamera`（tan θ に多項式を掛ける放射歪み）は 120° を
超えると発散するので使わない。式が単純なので GUI（TypeScript）にも同じものを
書ける——**ここを直したら `CameraView.tsx` の `projectRay()` も直すこと。**

**cv2 には依存しない（numpy だけ）。** 校正（`cv2.fisheye.calibrate`）だけは
Mac 側ツールが cv2 で行うが、使う側は numpy で完結させ、テストも cv2 なしで回す。

## 座標は「フル画角・下端クロップ前」の画素で持つ

`camera_node.bottom_cropped()` は ISP の ScalerCrop で `(0, 0, fw, keep_h)` を
**縦横同じ倍率で**縮小するので、クロップ後の画像は「フル画像の下を切り落とした
だけ」になる。つまり fx, fy, cx, cy は**クロップの有無で変わらない**（`cy` が
画像の下端より下に来ることがあるだけ）。校正値はクロップ前のフル画像の大きさ
（`width`, `height`）と一緒に保存し、使う側の解像度へは倍率を掛けて合わせる
（`FisheyeCalib.scaled()`）。

## カメラ座標系

OpenCV と同じ: x = 画像の右、y = 画像の下、z = 光軸（前）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

__all__ = ["FisheyeCalib", "distort_theta", "undistort_theta",
           "project_rays", "unproject_pixels", "virtual_pinhole_maps",
           "horizontal_fov", "Undistorter"]

#: θd → θ の Newton 法の反復回数。160° 程度の k なら 5〜6 回で 1e-12 まで収束する
_NEWTON_ITERS = 20


@dataclass(frozen=True, slots=True)
class FisheyeCalib:
    """魚眼レンズの校正値（`vehicle.toml` の `[sensors.cam_*.fisheye]`）。

    すべて**フル画角・下端クロップ前**の `width`×`height` 画像の画素単位。
    """

    fx: float
    fy: float
    cx: float
    cy: float
    k: tuple[float, float, float, float]
    width: int
    height: int
    #: 校正の再投影誤差 [px]（参考値。0 は不明）
    rms: float = 0.0

    @classmethod
    def from_dict(cls, d: dict) -> "FisheyeCalib | None":
        """`vehicle.toml` の表から作る。欠けていれば None（＝未校正）。"""
        try:
            k = tuple(float(v) for v in d["k"])
            if len(k) != 4:
                return None
            c = cls(fx=float(d["fx"]), fy=float(d["fy"]),
                    cx=float(d["cx"]), cy=float(d["cy"]), k=k,  # type: ignore[arg-type]
                    width=int(d["width"]), height=int(d["height"]),
                    rms=float(d.get("rms", 0.0)))
        except (KeyError, TypeError, ValueError):
            return None
        if c.fx <= 0 or c.fy <= 0 or c.width <= 0 or c.height <= 0:
            return None
        return c

    def scaled(self, width: int, height: int,
               bottom_crop: float = 0.0) -> tuple[float, float, float, float]:
        """実際に使う画像（`width`×`height`、下端 `bottom_crop` 切り落とし済み）の
        `(fx, fy, cx, cy)`。

        クロップ後の高さからクロップ前の高さ `height / (1 - bottom_crop)` に
        戻してから倍率を出す。縦横の倍率を別々に出すので、`cam_perception_node`
        のようにモデル入力へ非等方にリサイズした画像にもそのまま使える。
        """
        sx = width / self.width
        full_h = height / (1.0 - bottom_crop) if bottom_crop < 1.0 else height
        sy = full_h / self.height
        return self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy


def distort_theta(theta, k):
    """θ → θd（Kannala-Brandt の多項式）。numpy 配列・スカラどちらでもよい。"""
    t2 = theta * theta
    k1, k2, k3, k4 = k
    return theta * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))))


def undistort_theta(theta_d, k):
    """θd → θ（`distort_theta` の逆）。Newton 法。

    多項式が単調でなくなる（校正範囲外の外挿で起こりうる）領域でも発散しないよう、
    各ステップで θ を [0, π) に収める。
    """
    theta_d = np.asarray(theta_d, dtype=np.float64)
    k1, k2, k3, k4 = k
    theta = np.array(theta_d, dtype=np.float64, copy=True)
    for _ in range(_NEWTON_ITERS):
        t2 = theta * theta
        f = theta * (1.0 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4)))) - theta_d
        df = 1.0 + t2 * (3 * k1 + t2 * (5 * k2 + t2 * (7 * k3 + t2 * 9 * k4)))
        df = np.where(np.abs(df) < 1e-9, 1e-9, df)
        theta = np.clip(theta - f / df, 0.0, math.pi - 1e-6)
    return theta


def project_rays(x, y, z, fx: float, fy: float, cx: float, cy: float,
                 k: tuple[float, float, float, float] | None):
    """カメラ座標の光線 `(x, y, z)` → 画素 `(u, v)`。

    `k is None` ならピンホール（`z > 0` が前提。呼び出し側が判定する）。
    魚眼は光軸から 90° を超える光線（`z <= 0`）も式としては写せる。
    """
    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)
    z = np.asarray(z, dtype=np.float64)
    if k is None:
        zs = np.where(np.abs(z) < 1e-12, 1e-12, z)
        return cx + fx * x / zs, cy + fy * y / zs
    r = np.hypot(x, y)
    theta = np.arctan2(r, z)
    theta_d = distort_theta(theta, k)
    # r → 0 の極限は θd/r → 1/z（光軸上）。0 除算を避けて同じ値に寄せる
    safe_r = np.where(r < 1e-12, 1.0, r)
    scale = np.where(r < 1e-12, 1.0 / np.where(np.abs(z) < 1e-12, 1e-12, z),
                     theta_d / safe_r)
    return cx + fx * x * scale, cy + fy * y * scale


def unproject_pixels(u, v, fx: float, fy: float, cx: float, cy: float,
                     k: tuple[float, float, float, float] | None):
    """画素 `(u, v)` → カメラ座標の光線 `(x, y, z)`。

    ピンホールは `(…, …, 1)`（正規化なし。従来の IPM の式と一致させるため）、
    魚眼は単位ベクトルを返す。どちらも光線の向きだけが意味を持つ。
    """
    mx = (np.asarray(u, dtype=np.float64) - cx) / fx
    my = (np.asarray(v, dtype=np.float64) - cy) / fy
    if k is None:
        return mx, my, np.ones_like(mx)
    theta_d = np.hypot(mx, my)
    theta = undistort_theta(theta_d, k)
    safe = np.where(theta_d < 1e-12, 1.0, theta_d)
    s = np.where(theta_d < 1e-12, 0.0, np.sin(theta) / safe)
    return mx * s, my * s, np.cos(theta)


def horizontal_fov(calib: FisheyeCalib) -> float:
    """校正値から見た水平画角 [rad]（主点の高さの行の左端〜右端）。

    `vehicle.toml` の `hfov`（未校正時のピンホールのフォールバック・目安表示）を
    校正結果で更新するのに使う。
    """
    xl, _, zl = unproject_pixels(0.0, calib.cy, calib.fx, calib.fy, calib.cx, calib.cy, calib.k)
    xr, _, zr = unproject_pixels(float(calib.width), calib.cy,
                                 calib.fx, calib.fy, calib.cx, calib.cy, calib.k)
    return float(math.atan2(-float(xl), float(zl)) + math.atan2(float(xr), float(zr)))


def virtual_pinhole_maps(fx: float, fy: float, cx: float, cy: float,
                         k: tuple[float, float, float, float],
                         out_w: int, out_h: int, hfov_out: float,
                         bottom_crop: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """魚眼画像 → 仮想ピンホール画像の `cv2.remap` 用マップ `(map_x, map_y)`（float32）。

    仮想ピンホールの内部パラメータは**固定の式**にする:

        f' = (out_w / 2) / tan(hfov_out / 2),  cx' = out_w / 2,
        cy' = out_h / (2 (1 - bottom_crop))

    これは従来の未校正ピンホール（`raspi/nav/ipm.camera_intrinsics()`）と全く同じ式で、
    `hfov` を `hfov_out` に置き換えただけ。GUI は補正映像を表示している間、
    進路ガイドを「hfov = `undistort_hfov` のピンホール」として描けば済む
    （`estimateNewCameraMatrixForUndistortRectify` を TypeScript に移植しなくてよい）。

    `fx..cy` は入力画像（下端クロップ済み）の解像度に合わせたもの（`FisheyeCalib.scaled()`）。
    """
    f_out = (out_w / 2.0) / math.tan(hfov_out / 2.0)
    cx_out = out_w / 2.0
    cy_out = out_h / (2.0 * (1.0 - bottom_crop))
    vs, us = np.mgrid[0:out_h, 0:out_w].astype(np.float64)
    x = (us - cx_out) / f_out
    y = (vs - cy_out) / f_out
    mu, mv = project_rays(x, y, np.ones_like(x), fx, fy, cx, cy, k)
    return mu.astype(np.float32), mv.astype(np.float32)


class Undistorter:
    """魚眼画像を仮想ピンホール画像に直す（`telemetry_node` の補正映像）。

    マップ（`virtual_pinhole_maps`）は**入力画像の大きさが変わったときだけ**作り直す。
    1枚ごとの仕事は `cv2.remap` だけ（640x360 で Pi 5 なら数 ms）。
    cv2 は遅延 import する——補正映像を使わない構成では読み込まない。
    """

    def __init__(self, calib: FisheyeCalib, hfov_out: float, bottom_crop: float) -> None:
        self.calib = calib
        self.hfov_out = hfov_out
        self.bottom_crop = bottom_crop
        self._key: tuple[int, int] | None = None
        self._maps: tuple[np.ndarray, np.ndarray] | None = None

    def __call__(self, arr: np.ndarray) -> np.ndarray:
        import cv2

        h, w = arr.shape[:2]
        if self._key != (w, h):
            fx, fy, cx, cy = self.calib.scaled(w, h, self.bottom_crop)
            mx, my = virtual_pinhole_maps(fx, fy, cx, cy, self.calib.k, w, h,
                                          self.hfov_out, self.bottom_crop)
            # 固定小数点マップの方が remap が速い
            self._maps = cv2.convertMaps(mx, my, cv2.CV_16SC2)
            self._key = (w, h)
        m1, m2 = self._maps
        return cv2.remap(arr, m1, m2, interpolation=cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT)
