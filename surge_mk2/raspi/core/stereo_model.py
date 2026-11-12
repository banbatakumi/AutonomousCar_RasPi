"""前方ステレオの外部パラメータと平行化（rectification）。

    from raspi.core.stereo_model import StereoCalib, StereoRectifier

計画は `docs/stereo_plan.md`（段階2）。校正は Mac 側 `tools/cam_calib`（`--stereo`）が
`cv2.fisheye.stereoCalibrate` で求め、`config/vehicle.toml` の `[sensors.stereo]` に書く。
**平行化の表はここで起動時に計算する**（`vehicle.toml` には R・T だけを置く）。

## 外部パラメータの約束（OpenCV の `stereoCalibrate` と同じ。cam1 ＝ 左、cam2 ＝ 右）

    X_右 = R · X_左 + T

X はそれぞれのカメラ座標（`camera_model.py` と同じ: x ＝ 画像の右、y ＝ 下、z ＝ 光軸）。
左カメラは右カメラから見て x 負の側にあるので、理想の取付では R ≈ I、T ≈ (−0.075, 0, 0) [m]。
主カメラ（`image/front`）は右なので、**平行化後の座標の原点も右カメラに置く。**

## 平行化

右カメラ座標を回転 `R_rect` で回した座標を「平行化座標」とする（回すだけで原点は右カメラ）:

- x 軸 ＝ 左カメラ → 右カメラの向き（−T/|T|）
- z 軸 ＝ 2台の光軸の平均を x 軸に直交させたもの
- y 軸 ＝ z × x

この座標の中で2台は x 方向にだけ |T| ずれた同じ向きのピンホールになるので、同じ点は
左右の平行化画像の**同じ行**に写る。仮想ピンホールは2台共通で、`virtual_pinhole_maps`
（`camera_model.py`）と同じく f = (幅/2)/tan(hfov/2)、主点は画像の中央。視差 d = u左 − u右 > 0 から

    Z = f · B / d      （B = |T|）

で平行化座標の奥行きが出る。cv2 の `fisheye.stereoRectify` を使わないのは、
`camera_model.py` と同じ式を numpy だけで持ち、テストを cv2 なしで回すため
（`cv2.remap` だけは実行時に遅延 import する。`Undistorter` と同じ流儀）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .camera_model import FisheyeCalib, project_rays, unproject_pixels

__all__ = ["StereoCalib", "StereoRectifier", "rectify_rotation", "camera_to_base_rotation"]


@dataclass(frozen=True, slots=True)
class StereoCalib:
    """`vehicle.toml` の `[sensors.stereo]`。左右の内部パラメータは各レンズの `fisheye` が持つ。"""

    #: 左 → 右の回転（3×3 を行ごとに9個）
    R: tuple[float, ...]
    #: 左 → 右の並進 [m]（右カメラ座標での左カメラの位置）
    T: tuple[float, float, float]
    #: 校正したときのレンズのプロファイル名。**今のレンズと違えば使わない**（`Vehicle` が確かめる）
    lens_right: str = ""
    lens_left: str = ""
    #: 校正の再投影誤差 [px]（参考値）
    rms: float = 0.0

    @classmethod
    def from_dict(cls, d: dict) -> "StereoCalib | None":
        """表から作る。欠けていれば None（＝未校正）。"""
        try:
            R = tuple(float(v) for row in d["R"] for v in row)
            T = tuple(float(v) for v in d["T"])
        except (KeyError, TypeError, ValueError):
            return None
        if len(R) != 9 or len(T) != 3:
            return None
        Rm = np.array(R).reshape(3, 3)
        # 回転行列でなければ（手で書き換えた・桁落ち）使わない
        if not np.allclose(Rm @ Rm.T, np.eye(3), atol=1e-4) or np.linalg.det(Rm) < 0.0:
            return None
        if float(np.linalg.norm(T)) < 1e-3:
            return None
        return cls(R=R, T=T, lens_right=str(d.get("lens_right", "")),  # type: ignore[arg-type]
                   lens_left=str(d.get("lens_left", "")), rms=float(d.get("rms", 0.0)))

    @property
    def R_mat(self) -> np.ndarray:
        return np.array(self.R, dtype=np.float64).reshape(3, 3)

    @property
    def T_vec(self) -> np.ndarray:
        return np.array(self.T, dtype=np.float64)

    @property
    def baseline(self) -> float:
        """基線長 [m]。"""
        return float(np.linalg.norm(self.T))


def rectify_rotation(R: np.ndarray, T: np.ndarray) -> np.ndarray:
    """右カメラ座標 → 平行化座標の回転 `R_rect`（行が平行化座標の x, y, z 軸）。"""
    ex = -np.asarray(T, dtype=np.float64)
    ex = ex / np.linalg.norm(ex)
    # 右の光軸 (0,0,1) と左の光軸（右カメラ座標で R @ (0,0,1)）の平均
    z = np.array([0.0, 0.0, 1.0]) + np.asarray(R, dtype=np.float64)[:, 2]
    z = z - (z @ ex) * ex
    z = z / np.linalg.norm(z)
    ey = np.cross(z, ex)
    return np.stack([ex, ey, z])


def camera_to_base_rotation(pitch: float, yaw: float) -> np.ndarray:
    """カメラ座標（x 右・y 下・z 光軸）→ base_link（x 前・y 左・z 上）の回転。

    取付姿勢の約束は `raspi/nav/ipm.py` の `CameraExtrinsics` と同じ（pitch は下向きが正、
    ロールは持たない）。`P_base = R @ P_cam + (x, y, z)`。
    """
    C = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    cp, sp = math.cos(pitch), math.sin(pitch)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ C


class StereoRectifier:
    """左右の魚眼画像 → 行がそろった仮想ピンホール画像の組。

    :param right, left: 各カメラの内部パラメータ（フル画角・クロップ前。`FisheyeCalib`）
    :param ext: 外部パラメータ
    :param out_w, out_h: 平行化画像の大きさ（行を削る前のフルの大きさ）
    :param hfov: 平行化画像の水平画角 [rad]。魚眼の画角（約 125°）より狭くする
    :param rows: 平行化画像のうち残す行 `(v0, v1)`（半開区間）。None なら全部。
        **横は削らず縦だけ削る**（解像度を落とすと距離の誤差が増えるため。`docs/stereo_plan.md` §3）
    :param crop_right, crop_left: 入力画像の下端クロップ率（`bottom_crop`）
    """

    def __init__(self, right: FisheyeCalib, left: FisheyeCalib, ext: StereoCalib,
                 out_w: int = 640, out_h: int = 480, hfov: float = math.radians(100),
                 rows: tuple[int, int] | None = None,
                 crop_right: float = 0.0, crop_left: float = 0.0) -> None:
        self.right, self.left, self.ext = right, left, ext
        self.out_w, self.out_h = int(out_w), int(out_h)
        self.rows = (0, self.out_h) if rows is None else (int(rows[0]), int(rows[1]))
        if not 0 <= self.rows[0] < self.rows[1] <= self.out_h:
            raise ValueError(f"rows {rows} が 0..{self.out_h} に収まっていない")
        self.crops = {"right": float(crop_right), "left": float(crop_left)}
        #: 仮想ピンホール（2台共通）。`cy` は**行を削る前**の座標
        self.f = (self.out_w / 2.0) / math.tan(hfov / 2.0)
        self.cx = self.out_w / 2.0
        self.cy = self.out_h / 2.0
        #: 右カメラ座標 → 平行化座標
        self.R_rect = rectify_rotation(ext.R_mat, ext.T_vec)
        #: カメラ座標 → 平行化座標の回転（右は R_rect、左は R_rect·R）
        self._rot = {"right": self.R_rect, "left": self.R_rect @ ext.R_mat}
        self._maps: dict[tuple[str, int, int], tuple] = {}

    @property
    def baseline(self) -> float:
        return self.ext.baseline

    def _calib(self, cam: str) -> FisheyeCalib:
        if cam not in self._rot:
            raise ValueError(f"cam は right/left: {cam}")
        return self.right if cam == "right" else self.left

    def depth(self, disparity):
        """視差 [px] → 平行化座標の奥行き Z [m]。視差 ≤ 0 は inf。"""
        d = np.asarray(disparity, dtype=np.float64)
        with np.errstate(divide="ignore"):
            return np.where(d > 0, self.f * self.baseline / np.where(d > 0, d, 1.0), np.inf)

    def points_from_disparity(self, disp: np.ndarray, v0: int | None = None,
                              scale: float = 1.0) -> np.ndarray:
        """左の平行化画像の画素ごとの視差 → 平行化座標の点 (H, W, 3) [m]。視差 ≤ 0 は nan。

        `cv2.StereoBM/SGBM.compute(左, 右)` の結果（÷16 済み）をそのまま渡す。
        右カメラ（原点）での x は `(u左 − d − cx)·Z/f`。

        :param v0: `disp` の1行目が平行化画像の何行目か。None なら `rows` の始まり
        :param scale: `disp` を作った画像の、平行化画像に対する縮尺（半分に縮めて照合したなら 0.5）
        """
        d = np.asarray(disp, dtype=np.float64) / scale
        h, w = d.shape
        v0 = self.rows[0] if v0 is None else v0
        vs, us = np.mgrid[0:h, 0:w].astype(np.float64)
        us = (us + 0.5) / scale - 0.5
        vs = (vs + 0.5) / scale - 0.5 + v0
        with np.errstate(divide="ignore", invalid="ignore"):
            Z = np.where(d > 0, self.f * self.baseline / np.where(d > 0, d, 1.0), np.nan)
        X = (us - d - self.cx) * Z / self.f
        Y = (vs - self.cy) * Z / self.f
        return np.stack([X, Y, Z], -1)

    def points_from_pixels(self, u: np.ndarray, v: np.ndarray, d: np.ndarray,
                           v0: int | None = None, scale: float = 1.0) -> np.ndarray:
        """視差の出た画素だけ → 平行化座標の点 (N, 3)。`u, v` は `disp` の添字、`d` はその視差（> 0）。

        `points_from_disparity` と同じ式を、全画素ではなく渡した画素だけで計算する（Pi で軽くするため）。
        """
        v0 = self.rows[0] if v0 is None else v0
        d = np.asarray(d, dtype=np.float64) / scale
        uu = (np.asarray(u, dtype=np.float64) + 0.5) / scale - 0.5
        vv = (np.asarray(v, dtype=np.float64) + 0.5) / scale - 0.5 + v0
        Z = self.f * self.baseline / d
        return np.stack([(uu - d - self.cx) * Z / self.f, (vv - self.cy) * Z / self.f, Z], -1)

    def rect_to_base(self, pts: np.ndarray, x: float, y: float, z: float,
                     pitch: float = 0.0, yaw: float = 0.0) -> np.ndarray:
        """平行化座標の点 (..., 3) → base_link。`x y z pitch yaw` は右（主）カメラの取付（`Vehicle.cam_front_*`）。

        **行列積（`@`）を使わない。** Pi の numpy は参照 BLAS で、行列積が Mac の 100 倍以上遅い。
        3×3 の回転は成分ごとの掛け算と足し算で書く。
        """
        R = camera_to_base_rotation(pitch, yaw) @ self.R_rect.T
        X, Y, Z = pts[..., 0], pts[..., 1], pts[..., 2]
        return np.stack([R[0, 0] * X + R[0, 1] * Y + R[0, 2] * Z + x,
                         R[1, 0] * X + R[1, 1] * Y + R[1, 2] * Z + y,
                         R[2, 0] * X + R[2, 1] * Y + R[2, 2] * Z + z], -1)

    def maps(self, cam: str, in_w: int, in_h: int) -> tuple[np.ndarray, np.ndarray]:
        """`cv2.remap` 用のマップ（float32、`rows` を削った大きさ）。入力は `in_w`×`in_h`（下端クロップ済み）。"""
        c = self._calib(cam)
        fx, fy, cx, cy = c.scaled(in_w, in_h, self.crops[cam])
        v0, v1 = self.rows
        vs, us = np.mgrid[v0:v1, 0:self.out_w].astype(np.float64)
        ray = np.stack([(us - self.cx) / self.f, (vs - self.cy) / self.f, np.ones_like(us)], -1)
        # 平行化座標の光線 → カメラ座標（回転の転置）
        cam_ray = ray @ self._rot[cam]
        mu, mv = project_rays(cam_ray[..., 0], cam_ray[..., 1], cam_ray[..., 2],
                              fx, fy, cx, cy, c.k)
        return mu.astype(np.float32), mv.astype(np.float32)

    def rectify_points(self, cam: str, u, v, in_w: int, in_h: int):
        """元画像の画素 → 平行化画像の画素 `(u', v')`（行を削る前の座標）。光軸の後ろは nan。"""
        c = self._calib(cam)
        fx, fy, cx, cy = c.scaled(in_w, in_h, self.crops[cam])
        x, y, z = unproject_pixels(u, v, fx, fy, cx, cy, c.k)
        p = np.stack([x, y, z], -1) @ self._rot[cam].T
        with np.errstate(divide="ignore", invalid="ignore"):
            zr = np.where(p[..., 2] > 1e-9, p[..., 2], np.nan)
            return self.cx + self.f * p[..., 0] / zr, self.cy + self.f * p[..., 1] / zr

    def __call__(self, img_right: np.ndarray, img_left: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """平行化した `(右, 左)`。マップは入力の大きさが変わったときだけ作り直す。"""
        import cv2

        out = []
        for cam, img in (("right", img_right), ("left", img_left)):
            h, w = img.shape[:2]
            key = (cam, w, h)
            if key not in self._maps:
                self._maps[key] = cv2.convertMaps(*self.maps(cam, w, h), cv2.CV_16SC2)
            m1, m2 = self._maps[key]
            out.append(cv2.remap(img, m1, m2, interpolation=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_CONSTANT))
        return out[0], out[1]
