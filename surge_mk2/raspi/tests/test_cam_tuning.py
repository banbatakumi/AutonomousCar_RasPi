"""レンズごとの ISP チューニング（色むら補正の差分）。カメラ不要・Mac でも動く。

    python3 -m unittest raspi.tests.test_cam_tuning

守るもの:

- **純正レンズ（`isp_tuning` 無し）は標準のまま**——レンズの切り替えを壊さない
- 差分が読めない・センサーが違うときも映像は止めない（標準へ落ちる）
- 表を作る当てはめが、車体で画面の半分が隠れていても中心からの色むらを復元する
"""

from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.cam_tuning import (TUNING_DIR, apply_override, check_sensor,  # noqa: E402
                                   lens_tuning, pick_name)
from raspi.core.vehicle import DEFAULT_PATH, Vehicle, resolve_lens  # noqa: E402
from raspi.tools.alsc_calib import GRID, fit_tables, split_bayer  # noqa: E402

try:
    import tomllib
except ImportError:                                            # Python 3.10
    import tomli as tomllib


def _base() -> dict:
    """標準のチューニングの骨組み（`imx219.json` と同じ形）。"""
    return {"version": 2.0, "algorithms": [
        {"rpi.awb": {"enabled": True}},
        {"rpi.alsc": {"omega": 1.3, "luminance_lut": [1.0] * 4,
                      "calibrations_Cr": [{"ct": 3000, "table": [1.5] * 4}],
                      "calibrations_Cb": [{"ct": 3000, "table": [2.5] * 4}]}}]}


class TestApplyOverride(unittest.TestCase):
    def test_replaces_only_given_keys(self):
        base = _base()
        out = apply_override(base, {"sensor": "imx219", "rpi.alsc": {
            "calibrations_Cr": [{"ct": 4000, "table": [1.0] * 4}]}})
        alsc = out["algorithms"][1]["rpi.alsc"]
        self.assertEqual(alsc["calibrations_Cr"][0]["ct"], 4000)
        self.assertEqual(alsc["calibrations_Cb"][0]["table"], [2.5] * 4)   # 触っていない
        self.assertEqual(alsc["omega"], 1.3)
        self.assertEqual(base["algorithms"][1]["rpi.alsc"]["calibrations_Cr"][0]["ct"], 3000)

    def test_unknown_algorithm_is_an_error(self):
        with self.assertRaises(ValueError):
            apply_override(_base(), {"rpi.alcs": {}})


class TestLensTuning(unittest.TestCase):
    def _dir(self, doc) -> Path:
        d = Path(tempfile.mkdtemp())
        (d / "x.json").write_text(doc if isinstance(doc, str) else json.dumps(doc))
        return d

    def test_no_name_means_stock(self):
        """純正レンズ: 標準のチューニングを読みにも行かない。"""
        self.assertEqual(lens_tuning("", self.fail), (None, ""))

    def test_merges(self):
        d = self._dir({"sensor": "imx219", "rpi.alsc": {"omega": 9}})
        asked = []
        out, sensor = lens_tuning("x.json", lambda n: asked.append(n) or _base(), d)
        self.assertEqual(sensor, "imx219")
        self.assertEqual(asked, ["imx219.json"])               # 差分の sensor から標準を選ぶ
        self.assertEqual(out["algorithms"][1]["rpi.alsc"]["omega"], 9)

    def test_one_tuning_per_process(self):
        """libcamera はプロセスに1つしか持てない: 先頭のカメラに合わせ、食い違いは知らせる。"""
        self.assertEqual(pick_name(["a.json", "a.json"]), "a.json")
        self.assertEqual(pick_name(["", ""]), "")
        self.assertEqual(pick_name([]), "")
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertEqual(pick_name(["a.json", ""]), "a.json")
            self.assertEqual(pick_name(["", "a.json"]), "")    # 前が純正なら両方とも標準
        self.assertEqual(err.getvalue().count("プロセスに1つ"), 2)

    def test_check_sensor(self):
        self.assertTrue(check_sensor("imx219", "imx219", 0))
        self.assertTrue(check_sensor("", "ov5647", 0))         # 標準で開いたなら何でもよい
        with contextlib.redirect_stderr(io.StringIO()) as err:
            self.assertFalse(check_sensor("imx219", "ov5647", 1))
        self.assertIn("cam1", err.getvalue())

    def test_falls_back_to_stock(self):
        cases = {"sensor 無し": (self._dir({"rpi.alsc": {}}), "x.json"),
                 "壊れた JSON": (self._dir("{"), "x.json"),
                 "ファイル無し": (self._dir({}), "nai.json")}
        for label, (d, name) in cases.items():
            with self.subTest(label), contextlib.redirect_stderr(io.StringIO()) as err:
                self.assertEqual(lens_tuning(name, lambda n: _base(), d), (None, ""))
                self.assertIn("標準のまま", err.getvalue())


class TestVehicleProfiles(unittest.TestCase):
    """`config/vehicle.toml` の実物: レンズを切り替えるとチューニングも切り替わる。"""

    def setUp(self):
        with open(DEFAULT_PATH, "rb") as f:
            self.sensors = tomllib.load(f)["sensors"]

    def test_stock_lens_keeps_stock_tuning(self):
        for cam in ("cam_front", "cam_rear"):
            prof = resolve_lens({**self.sensors[cam], "lens": "stock"}, strict=True)
            self.assertNotIn("isp_tuning", prof, cam)

    def test_named_files_exist_and_fit_the_grid(self):
        for cam in ("cam_front", "cam_rear"):
            for lens, prof in self.sensors[cam]["lenses"].items():
                name = prof.get("isp_tuning")
                if not name:
                    continue
                doc = json.loads((TUNING_DIR / name).read_text(encoding="utf-8"))
                for key in ("calibrations_Cr", "calibrations_Cb"):
                    for cal in doc["rpi.alsc"][key]:
                        t = cal["table"]
                        self.assertEqual(len(t), GRID * GRID, f"{cam}.{lens} {key}")
                        self.assertGreaterEqual(min(t), 1.0)
                        self.assertLess(max(t), 2.0)

    def test_vehicle_exposes_selected_tuning(self):
        v = Vehicle.load()
        want = resolve_lens(self.sensors["cam_front"]).get("isp_tuning", "")
        self.assertEqual(v.cam_front_isp_tuning, want)


class TestFit(unittest.TestCase):
    def _wall(self, h=308, w=410):
        """中心で青が弱く（G/B が大きく）周辺で戻る壁。明るさは左右で3倍違う。"""
        yy, xx = np.mgrid[0:h, 0:w]
        r2 = (((xx + .5) / w - .5) ** 2 + (((yy + .5) / h - .5) * h / w) ** 2) \
            / (0.25 + 0.25 * (h / w) ** 2)
        g = 6000.0 * (1 + 2 * xx / w)
        return g / (1.30 + 0.05 * r2), g, g / (1.75 - 0.30 * r2 + 0.10 * r2 ** 2), r2

    def test_recovers_radial_shading_from_upper_half(self):
        r, g, b, _ = self._wall()
        r[170:], g[170:], b[170:] = 3000.0, 800.0, 100.0       # 下は車体（暗くて色も違う）
        cr, cb, info = fit_tables(r, g, b, np.zeros(g.shape, bool), rows=(0.0, 0.54))
        # 真値: 格子の中心の r²（像は 4:3）。データの無い外側は最も外の値で止まる
        yy, xx = np.mgrid[0:GRID, 0:GRID]
        asp = g.shape[0] / g.shape[1]
        c2 = (((xx + .5) / GRID - .5) ** 2 + (((yy + .5) / GRID - .5) * asp) ** 2) \
            / (0.25 + 0.25 * asp ** 2)
        c2 = np.minimum(c2, info["r2_max"])
        for got, true in ((cr, 1.30 + 0.05 * c2), (cb, 1.75 - 0.30 * c2 + 0.10 * c2 ** 2)):
            got = np.array(got).reshape(GRID, GRID)
            np.testing.assert_allclose(got, true / true.min(), atol=0.003)
            np.testing.assert_allclose(got, got[::-1], atol=1e-9)   # 隠れた下側も対称に埋まる
        self.assertLess(info["Cb"]["residual_sigma"], 0.005)

    def test_rejects_wall_that_misses_the_centre(self):
        r, g, b, _ = self._wall()
        with self.assertRaises(ValueError):
            fit_tables(r, g, b, np.zeros(g.shape, bool), rows=(0.0, 0.2))

    def test_split_bayer_order(self):
        raw = np.zeros((4, 4))
        raw[0::2, 0::2], raw[0::2, 1::2], raw[1::2, 0::2], raw[1::2, 1::2] = 10, 20, 22, 30
        r, g, b, sat = split_bayer(raw + 100, "BGGR", 100)
        self.assertEqual((r[0, 0], g[0, 0], b[0, 0]), (30, 21, 10))
        self.assertFalse(sat.any())


if __name__ == "__main__":
    unittest.main()
