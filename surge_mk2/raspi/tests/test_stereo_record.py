"""ステレオ開発のために `.mcap` へ残すもの（`stereo/info`・メタデータ `provenance`・`cam/config`）。"""

from __future__ import annotations

import json
import sys
import tempfile
import tomllib
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.stereo_match import MatchParams, ObstacleParams  # noqa: E402
from raspi.msgs import Scan, StereoInfo  # noqa: E402
from raspi.msgs.types import TOPIC_CAM_CONFIG, TOPIC_STEREO_INFO  # noqa: E402
from raspi.nodes.logger_node import DEFAULT_TOPICS  # noqa: E402
from raspi.nodes.stereo_node import StereoNode  # noqa: E402
from raspi.rec import provenance  # noqa: E402

try:
    import mcap  # noqa: F401
    HAS_MCAP = True
except ImportError:
    HAS_MCAP = False


def _fake_node(proc):
    return SimpleNamespace(proc=proc, dt_lr_ns=-120_000, cam_ms=19.0, wait_ms=1.0, read_ms=1.5, proc_ms=26.0,
                           latency_ms=47.0)


def _fake_proc():
    return SimpleNamespace(match=MatchParams("sgbm", 1.0, 96, 5), obst=ObstacleParams(),
                           rect=SimpleNamespace(rows=(150, 260)), cam_pose=(0.097, -0.02, 0.05, 0.1, 0.0),
                           last_n_points=321, last_valid_frac=0.82)


class TestStereoInfo(unittest.TestCase):
    def test_info_carries_the_inputs_needed_to_replay(self):
        scan = Scan(t_capture=123, seq=7)
        info = StereoNode.info(_fake_node(_fake_proc()), scan)
        self.assertEqual((info.t_capture, info.seq), (123, 7))
        self.assertEqual((info.dt_lr_ns, info.latency_ms), (-120_000, 47.0))
        self.assertEqual((info.n_points, info.valid_frac), (321, 0.82))
        self.assertEqual((info.algo, info.num_disp, info.block, info.rows), ("sgbm", 96, 5, [150, 260]))
        self.assertEqual((info.z_min, info.z_max, info.min_points), (0.03, 0.20, 4))
        self.assertEqual(info.cam_pose, [0.097, -0.02, 0.05, 0.1, 0.0])

    def test_info_is_none_without_a_processor(self):
        self.assertIsNone(StereoNode.info(_fake_node(None), Scan()))

    def test_topics_are_logged(self):
        self.assertIn(TOPIC_STEREO_INFO, DEFAULT_TOPICS)
        self.assertIn(TOPIC_CAM_CONFIG, DEFAULT_TOPICS)

    def test_owner_is_registered(self):
        from raspi.bus.zbus import TOPIC_OWNER

        self.assertEqual(TOPIC_OWNER[TOPIC_STEREO_INFO], "stereo")


class TestProvenance(unittest.TestCase):
    def test_collect(self):
        p = provenance.collect()
        self.assertEqual(set(p), {"git", "protocol_version", "src_sha", "vehicle_toml"})
        self.assertTrue(all(isinstance(v, str) for v in p.values()))
        cfg = tomllib.loads(p["vehicle_toml"])                       # 全文がそのまま読める
        self.assertIn("sensors", cfg)
        for rel in provenance.SRC_FILES:
            self.assertIn(f"{rel}=", p["src_sha"])
        self.assertNotIn("missing", p["src_sha"])

    def test_missing_files_do_not_break_recording(self):
        with tempfile.TemporaryDirectory() as d:
            p = provenance.collect(Path(d))
        self.assertEqual(p["vehicle_toml"], "")
        self.assertIn("missing", p["src_sha"])


@unittest.skipUnless(HAS_MCAP, "mcap が無い")
class TestRoundTrip(unittest.TestCase):
    def test_metadata_and_stereo_info_are_readable(self):
        from raspi.rec.mcap_log import McapLog
        from tools.stereo_bench.data import load_run_info, load_stereo_info

        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "t.mcap"
            log = McapLog(path, metadata={"node": "test"})
            log.add_metadata("provenance", provenance.collect())
            log.write("/stereo/info", StereoInfo(t_capture=2_000_000_000, seq=1, n_points=5, rows=[150, 260]),
                      t_mono_ns=2_000_000_000)
            log.write("/stereo/info", StereoInfo(t_capture=1_000_000_000, seq=0, n_points=3, rows=[150, 260]),
                      t_mono_ns=1_000_000_000)
            log.close()
            meta = load_run_info(path)
            info = load_stereo_info(path)
        self.assertEqual(meta["surge"]["node"], "test")
        self.assertIn("vehicle_toml", meta["provenance"])
        self.assertEqual([i["seq"] for i in info], [0, 1])             # 時刻順
        self.assertEqual(info[1]["n_points"], 5)
        self.assertEqual(info[0]["rows"], [150, 260])
        self.assertLess(info[0]["log_time"], info[1]["log_time"])


if __name__ == "__main__":
    unittest.main()
