"""診断タブ用に足した Pi 側の値の単体テスト（2026-10-05）。

- `raspi/io/pihealth.py`（CPU・メモリ・スロットリングの読み取り）
- `BusBridge.build_diag` の io_node 内部値（`LinkDiag.loop_max_ms` ほか）
- `TelemetryServer._nodes_status`（`hb/<node>` → ノード表）
- `camera_node.make_hb_detail`（`hb/camera` の `detail`）
"""

from __future__ import annotations

import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import raspi.nodes.camera_node as cn  # noqa: E402
import raspi.nodes.telemetry_node as tn  # noqa: E402
from raspi.io import pihealth  # noqa: E402
from raspi.nodes.io_node import IoNode  # noqa: E402
from raspi.tests.test_io_node_log import FakeLink  # noqa: E402

STAT_A = """cpu  100 0 100 800 0 0 0 0 0 0
cpu0 50 0 50 400 0 0 0 0 0 0
cpu1 50 0 50 400 0 0 0 0 0 0
intr 12345
"""
# 区間: cpu0 は busy +90 / total +100、cpu1 は busy +10 / total +100
STAT_B = """cpu  150 0 150 900 0 0 0 0 0 0
cpu0 95 0 95 410 0 0 0 0 0 0
cpu1 55 0 55 490 0 0 0 0 0 0
intr 23456
"""
MEMINFO = """MemTotal:        8000000 kB
MemFree:         1000000 kB
MemAvailable:    6000000 kB
"""


class TestPiHealthParsers(unittest.TestCase):
    def test_cpu_times_counts_idle_and_iowait_as_free(self):
        t = pihealth.parse_cpu_times("cpu  10 0 10 70 10 0 0 0 0 0\n")
        self.assertEqual(t["cpu"], (20, 100))

    def test_cpu_times_skips_non_cpu_lines(self):
        self.assertEqual(set(pihealth.parse_cpu_times(STAT_A)), {"cpu", "cpu0", "cpu1"})

    def test_meminfo(self):
        used, total_mb = pihealth.parse_meminfo(MEMINFO)
        self.assertAlmostEqual(used, 25.0)
        self.assertAlmostEqual(total_mb, 8000000 / 1024)

    def test_meminfo_missing_line_is_none(self):
        self.assertIsNone(pihealth.parse_meminfo("MemTotal: 100 kB\n"))

    def test_throttled_both_formats(self):
        self.assertEqual(pihealth.parse_throttled("throttled=0x50005\n"), 0x50005)
        self.assertEqual(pihealth.parse_throttled("50000\n"), 0x50000)
        self.assertIsNone(pihealth.parse_throttled("error\n"))


class TestPiHealthReader(unittest.TestCase):
    def _reader(self, d: Path, **kw) -> pihealth.PiHealthReader:
        return pihealth.PiHealthReader(
            proc_stat=d / "stat", proc_meminfo=d / "meminfo", proc_loadavg=d / "loadavg",
            cpufreq_dir=d, throttled_sysfs=(d / "get_throttled",), use_vcgencmd=False, **kw)

    def test_unavailable_off_pi(self):
        with tempfile.TemporaryDirectory() as td:
            r = self._reader(Path(td))
            self.assertFalse(r.available)
            self.assertFalse(r.read().available)

    def test_reads_interval_usage(self):
        with tempfile.TemporaryDirectory() as td:
            d = Path(td)
            (d / "stat").write_text(STAT_A)
            (d / "meminfo").write_text(MEMINFO)
            (d / "loadavg").write_text("0.52 0.40 0.30 1/200 999\n")
            (d / "scaling_cur_freq").write_text("1500000\n")
            (d / "scaling_max_freq").write_text("2000000\n")
            (d / "get_throttled").write_text("50000\n")
            r = self._reader(d)
            first = r.read()
            self.assertTrue(first.available)
            self.assertIsNone(first.cpu_pct, "初回は区間が無いので使用率を出さない")
            (d / "stat").write_text(STAT_B)
            h = r.read()
            self.assertAlmostEqual(h.cpu_pct, 50.0)
            self.assertAlmostEqual(h.cpu_max_pct, 90.0)
            self.assertAlmostEqual(h.load1, 0.52)
            self.assertAlmostEqual(h.mem_used_pct, 25.0)
            self.assertEqual((h.cpu_khz, h.cpu_max_khz), (1500000, 2000000))
            self.assertEqual(h.throttled, 0x50000)

    def test_missing_optional_files_become_none(self):
        with tempfile.TemporaryDirectory() as td:
            d = Path(td)
            (d / "stat").write_text(STAT_A)
            h = self._reader(d).read()
            self.assertTrue(h.available)
            self.assertIsNone(h.mem_used_pct)
            self.assertIsNone(h.cpu_khz)
            self.assertIsNone(h.throttled)


class TestBuildDiagInternals(unittest.TestCase):
    def test_passes_io_node_counters(self):
        node = IoNode(FakeLink())
        node.sync.resets = 2
        node.bridge.state_builder.odom_jumps = 3
        d = node.bridge.build_diag(node.state, node.sync, loop_max_ms=4.5, cmd_timeouts=7,
                                   disk_free_pct=61.0, log_errors=1)
        self.assertEqual((d.loop_max_ms, d.cmd_timeouts, d.stm_resets, d.odom_jumps,
                          d.disk_free_pct, d.log_errors), (4.5, 7, 2, 3, 61.0, 1))
        self.assertIsNone(d.fw_build_epoch, "VERSION を受けるまでは未受信")

    def test_defaults_when_caller_does_not_pass(self):
        """記録の再生（`replay_node`・`sfl2mcap`）は新しい引数を渡さない。"""
        node = IoNode(FakeLink())
        d = node.bridge.build_diag(node.state, node.sync)
        self.assertIsNone(d.loop_max_ms)
        self.assertEqual(d.cmd_timeouts, 0)


class TestNodesStatus(unittest.TestCase):
    def test_sorted_with_age(self):
        now = time.monotonic_ns()
        fake = SimpleNamespace(_node_hb={
            "planning": (now - 150_000_000, 22, "ftg engaged"),
            "io": (now - 20_000_000, 11, "OK"),
        })
        rows = tn.TelemetryServer._nodes_status(fake)
        self.assertEqual([r["node"] for r in rows], ["io", "planning"])
        self.assertEqual((rows[0]["pid"], rows[0]["detail"]), (11, "OK"))
        self.assertTrue(140 <= rows[1]["age_ms"] <= 400, rows[1])

    def test_empty(self):
        self.assertEqual(tn.TelemetryServer._nodes_status(SimpleNamespace(_node_hb={})), [])


def _worker(idx: int, frames: int, dropped: int = 0, enabled: bool = True):
    return SimpleNamespace(idx=idx, _enabled=enabled,
                           stats=SimpleNamespace(frames=frames, dropped=dropped))


class TestCameraHbDetail(unittest.TestCase):
    def test_fps_from_frame_increments(self):
        front, rear = _worker(0, 0), _worker(1, 0, enabled=False)
        node = SimpleNamespace(workers=[front, rear])
        detail = cn.make_hb_detail()
        with mock.patch.object(cn.time, "monotonic", return_value=100.0):
            self.assertEqual(detail(node), "front=- rear=off")
        front.stats.frames, front.stats.dropped = 60, 1
        with mock.patch.object(cn.time, "monotonic",
                               return_value=100.0 + cn.HB_FPS_WINDOW_S):
            self.assertEqual(detail(node), "front=30.0fps drop=1 rear=off")

    def test_counter_reset_does_not_go_negative(self):
        """`CameraNode.run` は開始時に `CamStats` を作り直す（枚数が 0 に戻る）。"""
        front = _worker(0, 500)
        node = SimpleNamespace(workers=[front])
        detail = cn.make_hb_detail()
        with mock.patch.object(cn.time, "monotonic", return_value=10.0):
            detail(node)
        front.stats.frames = 3
        with mock.patch.object(cn.time, "monotonic", return_value=20.0):
            self.assertEqual(detail(node), "front=-")


if __name__ == "__main__":
    unittest.main()
