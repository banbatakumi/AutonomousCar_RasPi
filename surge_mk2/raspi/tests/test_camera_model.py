"""魚眼レンズモデル（`raspi/core/camera_model.py`）と、それを使う IPM のテスト。

cv2 があれば `cv2.fisheye` と式が一致することも確かめる（校正ツールは cv2 で
値を出すので、ここがずれると校正値を正しく使えない）。
"""

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.camera_model import (FisheyeCalib, distort_theta,  # noqa: E402
                                     horizontal_fov, project_rays,
                                     undistort_theta, unproject_pixels,
                                     virtual_pinhole_maps)
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.nav.ipm import (CameraExtrinsics, camera_intrinsics,  # noqa: E402
                           ground_to_pixel, pixel_to_ground,
                           vehicle_camera_intrinsics)

#: IMX219 160° 相当の典型値（640x480 フル画角）。実機の校正値ではない
CALIB = FisheyeCalib(fx=190.0, fy=190.0, cx=322.0, cy=238.0,
                     k=(0.02, -0.01, 0.003, -0.0005), width=640, height=480)

try:
    import cv2  # noqa: F401
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


class TestTheta(unittest.TestCase):
    def test_undistort_inverts_distort(self):
        theta = np.linspace(0.0, math.radians(85), 50)
        back = undistort_theta(distort_theta(theta, CALIB.k), CALIB.k)
        np.testing.assert_allclose(back, theta, atol=1e-9)

    def test_zero_k_is_equidistant(self):
        theta = np.linspace(0.0, 1.4, 10)
        np.testing.assert_allclose(distort_theta(theta, (0, 0, 0, 0)), theta)


class TestProjectUnproject(unittest.TestCase):
    def test_round_trip(self):
        rng = np.random.default_rng(0)
        # 光軸から 80° 以内のランダムな光線
        th = rng.uniform(0, math.radians(80), 200)
        ph = rng.uniform(-math.pi, math.pi, 200)
        x, y, z = np.sin(th) * np.cos(ph), np.sin(th) * np.sin(ph), np.cos(th)
        u, v = project_rays(x, y, z, CALIB.fx, CALIB.fy, CALIB.cx, CALIB.cy, CALIB.k)
        rx, ry, rz = unproject_pixels(u, v, CALIB.fx, CALIB.fy, CALIB.cx, CALIB.cy, CALIB.k)
        np.testing.assert_allclose(np.stack([rx, ry, rz]), np.stack([x, y, z]), atol=1e-9)

    def test_optical_axis_maps_to_principal_point(self):
        u, v = project_rays(0.0, 0.0, 1.0, CALIB.fx, CALIB.fy, CALIB.cx, CALIB.cy, CALIB.k)
        self.assertAlmostEqual(float(u), CALIB.cx)
        self.assertAlmostEqual(float(v), CALIB.cy)

    def test_pinhole_when_k_is_none(self):
        u, v = project_rays(0.2, -0.1, 2.0, 300.0, 300.0, 320.0, 240.0, None)
        self.assertAlmostEqual(float(u), 320.0 + 300.0 * 0.1)
        self.assertAlmostEqual(float(v), 240.0 - 300.0 * 0.05)

    @unittest.skipUnless(HAVE_CV2, "cv2 が無い")
    def test_matches_cv2_fisheye(self):
        import cv2
        pts = np.array([[[0.3, -0.2, 1.0]], [[1.5, 0.7, 1.0]], [[-0.1, 0.05, 1.0]],
                        [[2.5, -1.8, 1.0]]], dtype=np.float64)
        K = np.array([[CALIB.fx, 0, CALIB.cx], [0, CALIB.fy, CALIB.cy], [0, 0, 1]])
        D = np.array(CALIB.k, dtype=np.float64).reshape(4, 1)
        ref, _ = cv2.fisheye.projectPoints(pts, np.zeros(3), np.zeros(3), K, D)
        u, v = project_rays(pts[:, 0, 0], pts[:, 0, 1], pts[:, 0, 2],
                            CALIB.fx, CALIB.fy, CALIB.cx, CALIB.cy, CALIB.k)
        np.testing.assert_allclose(np.stack([u, v], axis=1), ref[:, 0, :], atol=1e-6)

    def test_horizontal_fov_is_wide(self):
        self.assertGreater(math.degrees(horizontal_fov(CALIB)), 120.0)


class TestScaling(unittest.TestCase):
    def test_bottom_crop_keeps_intrinsics(self):
        """下端クロップは「下を切っただけ」なので fx..cy は変わらない。"""
        full = CALIB.scaled(640, 480, 0.0)
        cropped = CALIB.scaled(640, 360, 0.25)
        np.testing.assert_allclose(full, cropped)

    def test_resolution_scale(self):
        fx, fy, cx, cy = CALIB.scaled(320, 240)
        self.assertAlmostEqual(fx, CALIB.fx / 2)
        self.assertAlmostEqual(cy, CALIB.cy / 2)

    def test_anisotropic_resize(self):
        fx, fy, cx, cy = CALIB.scaled(224, 224)
        self.assertAlmostEqual(fx, CALIB.fx * 224 / 640)
        self.assertAlmostEqual(fy, CALIB.fy * 224 / 480)

    def test_from_dict_rejects_incomplete(self):
        self.assertIsNone(FisheyeCalib.from_dict({}))
        self.assertIsNone(FisheyeCalib.from_dict({"fx": 1, "fy": 1, "cx": 0, "cy": 0,
                                                  "k": [0, 0], "width": 1, "height": 1}))


class TestVirtualPinholeMaps(unittest.TestCase):
    def test_center_maps_to_principal_point(self):
        mx, my = virtual_pinhole_maps(*CALIB.scaled(640, 480), CALIB.k, 640, 480,
                                      math.radians(110))
        self.assertEqual(mx.shape, (480, 640))
        # 仮想ピンホールの主点 (320, 240) は実カメラの光軸 = (cx, cy) を見ている
        self.assertAlmostEqual(float(mx[240, 320]), CALIB.cx, places=3)
        self.assertAlmostEqual(float(my[240, 320]), CALIB.cy, places=3)

    def test_matches_pinhole_intrinsics_formula(self):
        """仮想ピンホールは `camera_intrinsics(hfov_out, …)` と同じ幾何（GUI がそれで描く）。"""
        hfov = math.radians(110)
        w, h, crop = 640, 360, 0.25
        intr_src = camera_intrinsics(0.0, w, h, crop, CALIB)
        mx, my = virtual_pinhole_maps(intr_src.f, intr_src.f_y, intr_src.cx,
                                      intr_src.principal_y, CALIB.k, w, h, hfov, crop)
        virt = camera_intrinsics(hfov, w, h, crop)
        ext = CameraExtrinsics(x=0.1, y=0.0, height=0.09, pitch=0.1, yaw=0.0)
        for gx, gy in ((0.5, 0.1), (1.0, -0.3), (0.4, 0.25)):
            pv = ground_to_pixel(gx, gy, virt, ext)
            ps = ground_to_pixel(gx, gy, intr_src, ext)
            self.assertIsNotNone(pv)
            self.assertIsNotNone(ps)
            iu, iv = int(round(pv[0])), int(round(pv[1]))
            self.assertAlmostEqual(float(mx[iv, iu]), ps[0], delta=1.5)
            self.assertAlmostEqual(float(my[iv, iu]), ps[1], delta=1.5)


class TestUndistorter(unittest.TestCase):
    @unittest.skipUnless(HAVE_CV2, "cv2 が無い")
    def test_keeps_shape_and_caches_maps(self):
        from raspi.core.camera_model import Undistorter
        u = Undistorter(CALIB, math.radians(110), 0.25)
        img = np.zeros((360, 640, 3), np.uint8)
        img[180, :, :] = 255
        out = u(img)
        self.assertEqual(out.shape, img.shape)
        maps = u._maps
        u(img)
        self.assertIs(u._maps, maps)          # 同じ大きさならマップを作り直さない


class TestFisheyeIpm(unittest.TestCase):
    def test_round_trip_wide_angle(self):
        intr = camera_intrinsics(0.0, 640, 360, 0.25, CALIB)
        ext = CameraExtrinsics(x=0.097, y=0.0, height=0.09, pitch=0.2, yaw=0.0)
        # ピンホール（66°）では写らない真横寄りの点も魚眼なら写る
        for x, y in ((0.3, 0.0), (0.4, 0.6), (1.0, -1.2), (2.5, 0.3)):
            px = ground_to_pixel(x, y, intr, ext)
            self.assertIsNotNone(px, f"({x},{y})")
            back = pixel_to_ground(px[0], px[1], intr, ext)
            self.assertIsNotNone(back)
            self.assertAlmostEqual(back[0], x, delta=1e-6)
            self.assertAlmostEqual(back[1], y, delta=1e-6)

    def test_vehicle_toml_section_is_used(self):
        toml = """
[sensors.cam_front]
hfov = 1.152
bottom_crop = 0.25
[sensors.cam_front.fisheye]
width = 640
height = 480
fx = 190.0
fy = 191.0
cx = 322.0
cy = 238.0
k = [0.02, -0.01, 0.003, -0.0005]
rms = 0.3
"""
        with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as f:
            f.write(toml)
        v = Vehicle.load(f.name)
        Path(f.name).unlink()
        self.assertIsNotNone(v.cam_front_fisheye)
        self.assertIsNone(v.cam_rear_fisheye)
        intr = vehicle_camera_intrinsics(v, "front", 640, 360)
        self.assertEqual(intr.k, (0.02, -0.01, 0.003, -0.0005))
        self.assertAlmostEqual(intr.f_y, 191.0)
        # 後カメラは未校正なのでピンホールのまま
        self.assertIsNone(vehicle_camera_intrinsics(v, "rear", 640, 480).k)


class TestLensProfiles(unittest.TestCase):
    """レンズのプロファイル（`lens = "…"` で `lenses.<名前>` を選ぶ、2026-10-03）。"""

    TOML = """
[sensors.cam_front]
x = 0.1
pitch = 0.05
lens = "{lens}"
[sensors.cam_front.lenses.stock]
hfov = 1.152
bottom_crop = 0.25
[sensors.cam_front.lenses.wide160]
hfov = 2.23
bottom_crop = 0.2
undistort_hfov = 1.9
[sensors.cam_front.lenses.wide160.fisheye]
width = 640
height = 480
fx = 280.0
fy = 280.0
cx = 320.0
cy = 240.0
k = [0.01, 0.0, 0.0, 0.0]
"""

    def _load(self, lens: str) -> Vehicle:
        with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as f:
            f.write(self.TOML.format(lens=lens))
        try:
            return Vehicle.load(f.name)
        finally:
            Path(f.name).unlink()

    def test_selected_profile_is_used(self):
        v = self._load("wide160")
        self.assertEqual(v.cam_front_lens, "wide160")
        self.assertAlmostEqual(v.cam_front_hfov, 2.23)
        self.assertAlmostEqual(v.cam_front_bottom_crop, 0.2)
        self.assertAlmostEqual(v.cam_front_undistort_hfov, 1.9)
        self.assertIsNotNone(v.cam_front_fisheye)
        # 取付位置・姿勢はレンズに依らずカメラの表のまま
        self.assertAlmostEqual(v.cam_front_x, 0.1)
        self.assertAlmostEqual(v.cam_front_pitch, 0.05)

    def test_switching_back_to_stock(self):
        v = self._load("stock")
        self.assertEqual(v.cam_front_lens, "stock")
        self.assertAlmostEqual(v.cam_front_hfov, 1.152)
        self.assertAlmostEqual(v.cam_front_bottom_crop, 0.25)
        self.assertIsNone(v.cam_front_fisheye)          # 純正は未校正 → ピンホール
        self.assertIsNone(vehicle_camera_intrinsics(v, "front", 640, 360).k)

    def test_unknown_lens(self):
        from raspi.core.vehicle import resolve_lens
        cam = {"x": 0.1, "lens": "nope", "lenses": {"stock": {"hfov": 1.0}}}
        with self.assertRaises(ValueError):
            resolve_lens(cam, strict=True)
        # ノードは止めずに、レンズ固有の値だけ既定値で動く
        loose = resolve_lens(cam)
        self.assertEqual(loose["x"], 0.1)
        self.assertNotIn("hfov", loose)

    def test_flat_layout_without_profiles_still_works(self):
        from raspi.core.vehicle import resolve_lens
        r = resolve_lens({"hfov": 1.2, "bottom_crop": 0.1})
        self.assertEqual(r, {"hfov": 1.2, "bottom_crop": 0.1, "lens": ""})

    def test_repo_vehicle_toml_resolves(self):
        """リポジトリの `vehicle.toml` が選んでいるレンズが実在する（打ち間違い防止）。"""
        import tomllib

        from raspi.core.vehicle import DEFAULT_PATH, resolve_lens
        with open(DEFAULT_PATH, "rb") as f:
            d = tomllib.load(f)
        for cam in ("cam_front", "cam_rear"):
            resolve_lens(d["sensors"][cam], strict=True)


if __name__ == "__main__":
    unittest.main()
