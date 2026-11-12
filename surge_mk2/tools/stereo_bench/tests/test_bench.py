"""`tools/stereo_bench` のテスト — 物差しそのものを確かめる。

既知の左右のレンズ・R・T・取付で、**模様を貼った平らな壁**を両方の魚眼画像に描き、
同じ壁を見た LiDAR の距離と一緒に `logger_node` と同じ形の `.mcap` に書く。
ベンチがその記録から、壁の距離（LiDAR もステレオも）を数 % 以内で戻すかを見る
（`docs/stereo_plan.md` 段階3「改善の前に、この物差し自体が合うかを先に確かめる」）。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")
pytest.importorskip("mcap")

from raspi.core.camera_model import FisheyeCalib, unproject_pixels  # noqa: E402
from raspi.core.stereo_model import (StereoCalib, StereoRectifier,  # noqa: E402
                                     camera_to_base_rotation)
from tools.stereo_bench.bench import PRESETS, Geometry, run, wall_check  # noqa: E402
from tools.stereo_bench.data import load_frames, pair_times  # noqa: E402

RIGHT = FisheyeCalib(fx=309.5, fy=309.8, cx=305.1, cy=241.5,
                     k=(-0.024, -0.036, 0.041, -0.0196), width=640, height=480)
LEFT = FisheyeCalib(fx=312.2, fy=312.6, cx=305.8, cy=243.8,
                    k=(-0.042, 0.0178, -0.0257, 0.0094), width=640, height=480)


def _rotvec(rx, ry, rz):
    return cv2.Rodrigues(np.array([rx, ry, rz], float))[0]


R_LR = _rotvec(math.radians(-0.12), math.radians(-0.75), math.radians(1.5))
T_LR = np.array([-0.0760, 0.0009, -0.0056])
EXT = StereoCalib(R=tuple(R_LR.ravel()), T=tuple(T_LR), lens_right="r", lens_left="l")
CAM = (0.09, -0.0375, 0.08, 0.0, 0.0)          # 右カメラ x y z pitch yaw
LIDAR = (0.07, 0.0, 0.095, 0.0)


def _texture(seed=0, n=512):
    """1px = 4mm の模様（ぼかしたランダム。照合が効く程度の細かさ）。"""
    rng = np.random.default_rng(seed)
    t = rng.uniform(0, 255, (n, n)).astype(np.float32)
    t = cv2.GaussianBlur(t, (0, 0), 1.2)
    return cv2.normalize(t, None, 20, 235, cv2.NORM_MINMAX)


TEX = _texture()
TEX_M = 0.004


def _render_wall(calib: FisheyeCalib, R_cb: np.ndarray, o_b: np.ndarray, wall_x: float) -> np.ndarray:
    """base_link で x = wall_x の壁を、姿勢 (R_cb: カメラ→base, o_b: 位置) のカメラで写した BGR 画像。"""
    vs, us = np.mgrid[0:480, 0:640].astype(np.float64)
    x, y, z = unproject_pixels(us, vs, calib.fx, calib.fy, calib.cx, calib.cy, calib.k)
    ray = np.stack([x, y, z], -1) @ R_cb.T
    s = (wall_x - o_b[0]) / np.where(ray[..., 0] > 1e-6, ray[..., 0], np.nan)
    py = o_b[1] + s * ray[..., 1]
    pz = o_b[2] + s * ray[..., 2]
    mx = (py / TEX_M + TEX.shape[1] / 2).astype(np.float32)
    my = (-(pz) / TEX_M + TEX.shape[0] / 2).astype(np.float32)
    img = cv2.remap(TEX, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    img[~np.isfinite(s)] = 60
    return cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2BGR)


def _scene(wall_x: float):
    R_rb = camera_to_base_rotation(CAM[3], CAM[4])
    o_r = np.array(CAM[:3])
    # 左カメラ: X_右 = R·X_左 + T → 左の光線を右カメラ座標へ R、位置は T
    R_lb = R_rb @ R_LR
    o_l = o_r + R_rb @ T_LR
    right = _render_wall(RIGHT, R_rb, o_r, wall_x)
    left = _render_wall(LEFT, R_lb, o_l, wall_x)
    scan = np.zeros(360)
    for deg in range(-60, 61):
        scan[deg % 360] = (wall_x - LIDAR[0]) / math.cos(math.radians(deg))
    return right, left, scan


@pytest.fixture(scope="module")
def recording(tmp_path_factory):
    from raspi.msgs.types import Scan, VehicleState
    from raspi.rec.mcap_log import McapLog

    path = tmp_path_factory.mktemp("rec") / "stereo.mcap"
    log = McapLog(path)
    t0 = 1_000_000_000
    for i, wall in enumerate((0.8, 1.6)):
        right, left, scan = _scene(wall)
        t = t0 + i * 200_000_000
        log.write("/vehicle_state", VehicleState(t_capture=t, speed=0.0))
        log.write("/scan", Scan(t_capture=t - 50_000_000, dist=scan.tolist()))
        for cam, img, dt in (("front", right, 0), ("left", left, 30_000)):
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            log.write_viz_image(buf.tobytes(), cam, t + dt)
    log.close()
    return path


def _geom():
    rect = StereoRectifier(RIGHT, LEFT, EXT, 640, 480, math.radians(100))
    return Geometry(rect, CAM, LIDAR)


def test_pair_times():
    assert pair_times([0, 100, 200], [5, 104, 400], 10) == [(0, 0), (1, 1)]
    assert pair_times([0, 3], [1], 10) == [(0, 0)]          # 1枚は1組にしか入らない


def test_load_frames(recording):
    fr = load_frames(recording)
    assert len(fr) == 2
    assert all(f.scan is not None and f.speed == 0.0 for f in fr)
    assert abs(fr[0].dt_ns - 30_000) < 2_000
    assert load_frames(recording, pair_tol_ns=10_000) == []


def test_wall_ruler(recording):
    """物差しの確認: LiDAR もステレオも壁の距離を戻す（壁は LiDAR から x 方向に wall − 0.07m）。"""
    frames = load_frames(recording)
    w = wall_check(frames[:1], _geom(), [PRESETS["sgbm"], PRESETS["bm"]], 0.8 - LIDAR[0])
    assert w["lidar"][0] == pytest.approx(0.73, rel=0.01)
    assert w["sgbm"][0] == pytest.approx(0.73, rel=0.03)
    assert w["bm"][0] == pytest.approx(0.73, rel=0.03)


def test_run_against_lidar(recording):
    frames = load_frames(recording)
    res = run(frames, _geom(), [PRESETS["sgbm"], PRESETS["sgbm-half"]])
    for r in res:
        s = r.summary()
        assert s["bins"] > 40, s
        assert abs(s["bias"]) < 0.03, s
        assert s["abs_med"] < 0.03, s
        assert s["valid"] > 0.3, s


def test_moving_frames_are_not_compared(recording):
    frames = load_frames(recording)
    for f in frames:
        f.speed = 1.0
    res = run(frames, _geom(), [PRESETS["bm"]])
    assert res[0].summary()["bins"] == 0
