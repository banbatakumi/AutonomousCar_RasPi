"""`tools/slam_replay.py`（記録を slam2d に通し直す）のテスト。

    python3 -m unittest raspi.tests.test_slam_replay
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.proto import packets  # noqa: E402
from raspi.rec.framelog import FrameLogWriter  # noqa: E402
from raspi.rec.mcap_log import HAS_MCAP  # noqa: E402
from raspi.tests.test_mcap import read_back  # noqa: E402

MS = 1_000_000


def _has_slam2d() -> bool:
    try:
        import raspi.auto._slam2d_nav  # noqa: F401
    except ImportError:
        return False
    return True


@unittest.skipUnless(HAS_MCAP and _has_slam2d(), "mcap / slam2d が入っていない")
class TestSlamReplay(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        d = Path(self._td.name)
        self.sfl = d / "t.sfl"
        self.out = d / "t_slam.mcap"

    def tearDown(self):
        self._td.cleanup()

    def _write_sfl(self, rotations: int = 6) -> None:
        """止まったまま、四角い部屋の中で LiDAR を回した記録。"""
        import math

        w = FrameLogWriter(self.sfl, meta={"node": "test"}, flush_interval_s=0.0)
        t0, seq = w.t0_mono_ns, 0
        n_telem = rotations * 10 + 10
        for i in range(n_telem):
            seq += 1
            t = packets.Telemetry(t_us=i * 10_000, speed=0, flags=1)
            w.write_rx(t0 + i * 10 * MS, t.TYPE, seq & 0xFF, t.encode())
        # 2m 四方の部屋（中心に車）。センサ角→距離 [mm]
        def wall(deg: float) -> int:
            a = math.radians(deg)
            return int(1000 / max(abs(math.cos(a)), abs(math.sin(a))))
        for rot in range(rotations):
            for idx in range(12):
                seq += 1
                dist = [wall(idx * 30 + k) for k in range(30)]
                s = packets.LidarSector(sector_idx=idx, rot_speed_dps=3600,
                                        duration_us=8000, dist=dist,
                                        t_start_us=(rot * 100 + idx * 8) * 1000)
                w.write_rx(t0 + (rot * 100 + idx * 8 + 5) * MS, s.TYPE, seq & 0xFF,
                           s.encode())
        w.close()

    def test_sfl_input_adds_slam_topics(self):
        from raspi.tools.slam_replay import replay

        self._write_sfl()
        _, slam = replay(self.sfl, self.out)
        got = read_back(self.out)
        msgs, sch = got["msgs"], got["schemas"]
        self.assertGreater(slam.scans, 2)
        self.assertEqual(len(msgs["/slam2d"]), slam.scans)
        self.assertIn("/pose_compare", msgs)
        self.assertEqual(sch["/viz/slam2d/pose"], "foxglove.PoseInFrame")
        self.assertEqual(sch["/viz/slam2d/scan"], "foxglove.PointCloud")
        self.assertEqual(sch["/viz/slam2d/map"], "foxglove.Grid")
        # 元の変換結果（sfl2mcap と同じもの）も入っている
        self.assertIn("/vehicle_state", msgs)
        self.assertIn("/viz/odom/path", msgs)
        # /tf に odom→base_link（推測航法）と odom→slam2d（始点合わせ）の両方がある
        pairs = {(b["parent_frame_id"], b["child_frame_id"]) for _, b in msgs["/tf"]}
        self.assertEqual(pairs, {("odom", "base_link"), ("odom", "slam2d")})
        # 止まっているので推測航法と SLAM はほぼ一致する
        for _, c in msgs["/pose_compare"]:
            self.assertLess(c["err_pos"], 0.05)

    def test_every_scan_is_compared(self):
        """比較は推測航法が追いつくまで保留する（点群の基準時刻は車両状態より先に来る）。"""
        from raspi.tools.slam_replay import replay

        self._write_sfl()
        _, slam = replay(self.sfl, self.out)
        n_cmp = len(read_back(self.out)["msgs"]["/pose_compare"])
        self.assertGreaterEqual(n_cmp, slam.scans - 1)

    def test_mcap_input_is_copied_without_duplicates(self):
        """`.mcap` 入力: 元の中身は写し、作り直すトピック（/odom 等）は二重にしない。"""
        from raspi.tools.sfl2mcap import export
        from raspi.tools.slam_replay import replay

        self._write_sfl()
        src = Path(self._td.name) / "src.mcap"
        export(self.sfl, src, quiet=True)
        before = read_back(src)["msgs"]
        replay(src, self.out)
        after = read_back(self.out)
        msgs = after["msgs"]
        for topic in ("/vehicle_state", "/scan", "/viz/scan", "/diag/link"):
            with self.subTest(topic=topic):
                self.assertEqual(len(msgs[topic]), len(before[topic]))
                self.assertEqual(msgs[topic][0], before[topic][0])   # 時刻も中身もそのまま
        self.assertEqual(len(msgs["/odom"]), len(before["/odom"]))
        self.assertIn("/slam2d", msgs)
        self.assertEqual(after["metadata"]["surge"]["converter"], "slam_replay")

    def test_mcap_without_surge_metadata_is_refused(self):
        from mcap.writer import Writer

        from raspi.tools.slam_replay import replay

        src = Path(self._td.name) / "foreign.mcap"
        with open(src, "wb") as f:
            w = Writer(f)
            w.start()
            w.finish()
        with self.assertRaises(ValueError):
            replay(src, self.out)


if __name__ == "__main__":
    unittest.main()
