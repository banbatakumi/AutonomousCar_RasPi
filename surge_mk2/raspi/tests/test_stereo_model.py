"""`raspi/core/stereo_model.py`（ステレオの平行化）のテスト。cv2 なしで回る。

既知の左右のレンズと R・T のもとで 3 次元の点を両方の魚眼画像に写し、平行化すると
同じ行に乗り、視差から奥行きが戻るかを見る。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from raspi.core.camera_model import FisheyeCalib, project_rays
from raspi.core.stereo_model import (StereoCalib, StereoRectifier, camera_to_base_rotation,
                                     rectify_rotation)


def _rot(rx: float, ry: float, rz: float) -> np.ndarray:
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry),
                              math.cos(rz), math.sin(rz))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


RIGHT = FisheyeCalib(fx=307.9, fy=307.6, cx=307.4, cy=241.5,
                     k=(-0.032, 0.0035, -0.0107, 0.0035), width=640, height=480)
LEFT = FisheyeCalib(fx=305.0, fy=306.2, cx=322.0, cy=236.0,
                    k=(-0.030, 0.0030, -0.0100, 0.0030), width=640, height=480)
R_LR = _rot(math.radians(1.0), math.radians(-0.8), math.radians(0.5))
T_LR = np.array([-0.0752, 0.0012, -0.0009])
EXT = StereoCalib(R=tuple(R_LR.ravel()), T=tuple(T_LR))


def _points(n: int = 300, seed: int = 0) -> np.ndarray:
    """右カメラ座標の点（前方 0.3〜3m、平行化画像に入る範囲）。"""
    rng = np.random.default_rng(seed)
    z = rng.uniform(0.3, 3.0, n)
    x = z * rng.uniform(-0.9, 0.9, n)
    y = z * rng.uniform(-0.6, 0.6, n)
    return np.stack([x, y, z], 1)


def _project(P_right: np.ndarray, cam: str, w: int, h: int, crop: float = 0.0):
    c = RIGHT if cam == "right" else LEFT
    P = P_right if cam == "right" else (P_right - T_LR) @ R_LR       # X_左 = Rᵀ(X_右 − T)
    fx, fy, cx, cy = c.scaled(w, h, crop)
    return project_rays(P[:, 0], P[:, 1], P[:, 2], fx, fy, cx, cy, c.k)


def test_rectify_rotation_is_rotation():
    Rr = rectify_rotation(R_LR, T_LR)
    assert np.allclose(Rr @ Rr.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(Rr) == pytest.approx(1.0)
    # 左カメラは平行化座標で真左（x 負）にだけずれている
    left_c = Rr @ T_LR
    assert left_c[0] == pytest.approx(-np.linalg.norm(T_LR))
    assert np.allclose(left_c[1:], 0.0, atol=1e-12)


@pytest.mark.parametrize("w,h,crop", [(640, 480, 0.0), (320, 180, 0.25)])
def test_same_row_and_depth(w, h, crop):
    rect = StereoRectifier(RIGHT, LEFT, EXT, 640, 480, math.radians(100),
                           crop_right=crop, crop_left=crop)
    P = _points()
    ur, vr = rect.rectify_points("right", *_project(P, "right", w, h, crop), w, h)
    ul, vl = rect.rectify_points("left", *_project(P, "left", w, h, crop), w, h)
    assert np.max(np.abs(vl - vr)) < 1e-6
    # 平行化座標の奥行き（右カメラを原点に回しただけ）と視差から出した奥行き
    Z = (P @ rect.R_rect.T)[:, 2]
    assert np.allclose(rect.depth(ul - ur), Z, rtol=1e-6)
    assert np.all(ul > ur)                                  # 視差は正（左の像は右へずれる）


def test_maps_agree_with_rectify_points():
    """マップが指す魚眼の画素を平行化し直すと、元の平行化画素に戻る。"""
    rect = StereoRectifier(RIGHT, LEFT, EXT, 640, 480, math.radians(100), rows=(200, 360))
    for cam in ("right", "left"):
        mu, mv = rect.maps(cam, 640, 480)
        assert mu.shape == (160, 640)
        vs, us = np.mgrid[200:360:7, 0:640:13]
        u2, v2 = rect.rectify_points(cam, mu[vs - 200, us], mv[vs - 200, us], 640, 480)
        assert np.max(np.abs(u2 - us)) < 1e-2                # float32 のマップの丸め
        assert np.max(np.abs(v2 - vs)) < 1e-2


def test_depth_of_nonpositive_disparity_is_inf():
    rect = StereoRectifier(RIGHT, LEFT, EXT)
    d = rect.depth(np.array([0.0, -1.0, 10.0]))
    assert np.isinf(d[0]) and np.isinf(d[1])
    assert d[2] == pytest.approx(rect.f * EXT.baseline / 10.0)


def test_bad_rows_rejected():
    with pytest.raises(ValueError):
        StereoRectifier(RIGHT, LEFT, EXT, 640, 480, rows=(300, 200))


@pytest.mark.parametrize("scale", [1.0, 0.5])
def test_points_from_disparity_roundtrip(scale):
    """平行化画像上の視差から点を戻すと元の点になる（縮めて照合した場合も）。"""
    rect = StereoRectifier(RIGHT, LEFT, EXT, 640, 480, math.radians(100), rows=(160, 400))
    P = _points(50, seed=4) @ rect.R_rect.T                   # 平行化座標の点
    ur = rect.cx + rect.f * P[:, 0] / P[:, 2]
    v = rect.cy + rect.f * P[:, 1] / P[:, 2]
    ul = ur + rect.f * rect.baseline / P[:, 2]
    ok = (v >= 160) & (v < 400) & (ul >= 0) & (ul < 640)
    h, w = int(240 * scale), int(640 * scale)
    disp = np.full((h, w), -1.0)
    # 縮めた画像の画素中心に乗る点だけを置く（丸めの誤差を持ち込まない）
    uj = np.round((ul[ok] + 0.5) * scale - 0.5).astype(int)
    vi = np.round((v[ok] - 160 + 0.5) * scale - 0.5).astype(int)
    disp[vi, uj] = (ul[ok] - ur[ok]) * scale
    pts = rect.points_from_disparity(disp, scale=scale)
    got = pts[vi, uj]
    ul_c = (uj + 0.5) / scale - 0.5                          # 実際に置いた画素の位置
    v_c = (vi + 0.5) / scale - 0.5 + 160
    Z = P[ok, 2]
    assert np.allclose(got[:, 2], Z, rtol=1e-9)
    assert np.allclose(got[:, 0], (ul_c - (ul[ok] - ur[ok]) - rect.cx) * Z / rect.f, atol=1e-9)
    assert np.allclose(got[:, 1], (v_c - rect.cy) * Z / rect.f, atol=1e-9)
    assert np.isnan(pts[0, 0, 2]) or disp[0, 0] > 0


def test_camera_to_base_rotation_matches_ipm_convention():
    R = camera_to_base_rotation(0.0, 0.0)
    assert np.allclose(R @ [0, 0, 1], [1, 0, 0])             # 光軸 → 前
    assert np.allclose(R @ [1, 0, 0], [0, -1, 0])            # 画像の右 → 車の右（y 負）
    assert np.allclose(R @ [0, 1, 0], [0, 0, -1])            # 画像の下 → 下
    p = math.radians(10)                                      # 下向き 10°
    assert np.allclose(camera_to_base_rotation(p, 0.0) @ [0, 0, 1], [math.cos(p), 0, -math.sin(p)])


def test_rect_to_base_puts_points_ahead():
    rect = StereoRectifier(RIGHT, LEFT, EXT)
    pts = np.array([[0.0, 0.0, 2.0]]) @ np.eye(3)            # 平行化座標で正面 2m
    b = rect.rect_to_base(pts, 0.09, -0.0375, 0.08)
    assert b[0, 0] == pytest.approx(0.09 + 2.0, abs=0.05)
    assert b[0, 2] == pytest.approx(0.08, abs=0.05)


def test_points_from_pixels_matches_dense():
    rect = StereoRectifier(RIGHT, LEFT, EXT, 640, 480, math.radians(100), rows=(160, 400))
    rng = np.random.default_rng(1)
    disp = np.where(rng.uniform(size=(120, 320)) < 0.3, rng.uniform(2, 40, (120, 320)), -1.0)
    dense = rect.points_from_disparity(disp, scale=0.5)
    v, u = np.nonzero(disp > 0)
    sparse = rect.points_from_pixels(u, v, disp[v, u], scale=0.5)
    assert np.allclose(sparse, dense[v, u], rtol=1e-12)
    R = camera_to_base_rotation(0.05, 0.02) @ rect.R_rect.T
    assert np.allclose(rect.rect_to_base(sparse, 0.1, -0.03, 0.08, 0.05, 0.02),
                       sparse @ R.T + [0.1, -0.03, 0.08], atol=1e-12)
