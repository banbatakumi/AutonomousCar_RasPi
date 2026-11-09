"""`raspi/nav/scan_filter.py`: 走行モードに依らない点群のノイズの除去。"""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.msgs import Scan, VehicleState  # noqa: E402
from raspi.nav.scan_filter import ScanFilter, ScanFilterConfig, config_from_items  # noqa: E402

DT_NS = 100_000_000


def _room(x: float = 0.0) -> np.ndarray:
    """車の周り 2m の正方形の壁（車が +x へ `x` 進んだ所から見た距離）。"""
    ang = np.radians(np.arange(360))
    c, s = np.cos(ang), np.sin(ang)
    with np.errstate(divide="ignore"):
        tx = np.where(c > 1e-9, (2.0 - x) / c, np.where(c < -1e-9, (-2.0 - x) / c, np.inf))
        ty = np.where(np.abs(s) > 1e-9, 2.0 / np.abs(s), np.inf)
    return np.minimum(tx, ty)


def _scan(d, t, intensity=None) -> Scan:
    return Scan(t_capture=t, dist=[float(v) for v in d], sector_seen=[True] * 12,
                intensity=None if intensity is None else [int(v) for v in intensity])


class TestScanFilter(unittest.TestCase):
    def _run(self, frames, speed=0.0, cfg=ScanFilterConfig()):
        f = ScanFilter(cfg)
        vs = VehicleState(speed=speed)
        out = None
        for i, (d, inten) in enumerate(frames):
            out = f.apply(_scan(d, (i + 1) * DT_NS, inten), vs)
        return f, np.asarray(out.dist)

    def test_clean_walls_are_kept_while_moving(self):
        frames = [(_room(0.2 * i), None) for i in range(4)]
        f, d = self._run(frames, speed=2.0)
        self.assertEqual(f.dropped, 0)
        self.assertTrue((d > 0).all())

    def test_random_speckles_are_dropped(self):
        rng = np.random.default_rng(0)
        frames = []
        noisy = []
        for i in range(5):
            d = _room()
            k = rng.choice(360, 6, replace=False)
            d[k] = rng.uniform(0.15, 1.2, 6)       # 日光のような、距離がばらばらの点
            frames.append((d, None))
            noisy.append(k)
        f, d = self._run(frames)
        self.assertTrue((d[noisy[-1]] == 0).all())
        self.assertLessEqual(f.dropped, 7)

    def test_a_small_object_that_stays_is_kept(self):
        # 正面 1m の幅 3° の物（3 点連なる）は孤立しないので、初めて見えた周でも消さない
        frames = []
        for _ in range(2):
            d = _room()
            d[[359, 0, 1]] = 1.0
            frames.append((d, None))
        _, d = self._run(frames)
        self.assertTrue((d[[359, 0, 1]] == 1.0).all())

    def test_a_lone_point_that_repeats_is_kept(self):
        # 細い柱（1 点だけ）でも、前の周の同じ所に見えていれば本物として残す
        frames = []
        for _ in range(3):
            d = _room()
            d[90] = 0.6
            frames.append((d, None))
        _, d = self._run(frames)
        self.assertEqual(d[90], 0.6)

    def test_low_intensity_points_go_even_in_small_clusters(self):
        frames = []
        for i in range(30):
            d = _room()
            inten = np.full(360, 200)
            if i == 29:
                d[[100, 101]] = [0.4, 0.42]       # 新しく出た 2 点の塊・強度が低い
                inten[[100, 101]] = 10
            frames.append((d, inten))
        f, d = self._run(frames)
        self.assertGreater(f.threshold, 50.0)
        self.assertTrue((d[[100, 101]] == 0).all())
        self.assertEqual(f.dropped_intensity, 2)

    def test_dark_but_stable_walls_are_kept(self):
        # 黒い幕の壁（強度が低いが、連なって毎周同じ所に出る）は消さない
        frames = []
        for _ in range(30):
            inten = np.full(360, 200)
            inten[180:270] = 8
            frames.append((_room(), inten))
        f, d = self._run(frames)
        self.assertTrue((d[180:270] > 0).all())

    def test_disabled_and_config(self):
        cfg = config_from_items((("enabled", False), ("speckle_m", 0.3), ("unknown", 1)))
        self.assertFalse(cfg.enabled)
        self.assertEqual(cfg.speckle_m, 0.3)
        d = _room()
        d[10] = 0.3
        f, out = self._run([(d, None), (d, None)], cfg=cfg)
        self.assertEqual(f.dropped, 0)


if __name__ == "__main__":
    unittest.main()
