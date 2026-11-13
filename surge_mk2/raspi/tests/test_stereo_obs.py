import math
import unittest

from raspi.core.stereo_obs import MAX_POINTS, scan_to_obs
from raspi.msgs.types import Scan
from raspi.proto import packets

NOW = 10_000_000_000


def scan(dist_at: dict[int, float], *, t=NOW - 40_000_000, seen=True) -> Scan:
    d = [0.0] * 360
    for k, v in dist_at.items():
        d[k] = v
    return Scan(dist=d, t_capture=t, sector_seen=[seen] * 12)


class StereoObsTest(unittest.TestCase):
    def test_failed_scan_not_sent(self):
        self.assertIsNone(scan_to_obs(Scan(), NOW))
        self.assertIsNone(scan_to_obs(scan({}, seen=False), NOW))

    def test_alive_with_no_points(self):
        o = scan_to_obs(scan({}), NOW)
        self.assertEqual((o.count, o.age_ms), (0, 40))

    def test_stale_not_sent(self):
        self.assertIsNone(scan_to_obs(scan({0: 1.0}, t=NOW - 300_000_000), NOW))

    def test_coordinates(self):
        o = scan_to_obs(scan({0: 1.0, 90: 0.5}), NOW)
        self.assertEqual(o.count, 2)
        self.assertEqual(o.xy[:4], [0, 500, 1000, 0])  # 近い順（0.5m@90° が先）

    def test_bin_keeps_nearest(self):
        o = scan_to_obs(scan({10: 1.0, 11: 0.8}), NOW)
        self.assertEqual(o.count, 1)
        self.assertAlmostEqual(math.hypot(o.xy[0], o.xy[1]) / 1000, 0.8, places=2)

    def test_cap_keeps_nearest(self):
        o = scan_to_obs(scan({i * 2: 2.0 + i * 0.01 for i in range(60)}), NOW)
        self.assertEqual(o.count, MAX_POINTS)

    def test_encodes_to_fixed_length(self):
        o = scan_to_obs(scan({0: 1.0}), NOW)
        self.assertEqual(len(o.encode()), packets.StereoObs.LEN)


if __name__ == "__main__":
    unittest.main()
