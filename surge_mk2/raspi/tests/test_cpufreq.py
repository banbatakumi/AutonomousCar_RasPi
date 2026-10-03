"""`CpuFreqCap` — 上限周波数の書き込み（クランプ・同じ値は書かない・書けなければ何もしない）。"""

from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

from raspi.io.cpufreq import CpuFreqCap


class CpuFreqCapTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        (self.root / "cpuinfo_max_freq").write_text("2400000\n")
        (self.root / "cpuinfo_min_freq").write_text("1500000\n")
        self.max_path = self.root / "scaling_max_freq"
        self.max_path.write_text("2400000\n")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_writes_and_clamps(self) -> None:
        cap = CpuFreqCap(self.root)
        self.assertTrue(cap.available)
        self.assertTrue(cap.set_max_khz(2_000_000))
        self.assertEqual(self.max_path.read_text(), "2000000")
        cap.set_max_khz(100)
        self.assertEqual(self.max_path.read_text(), "1500000")
        cap.restore()
        self.assertEqual(self.max_path.read_text(), "2400000")

    def test_same_value_is_not_rewritten(self) -> None:
        cap = CpuFreqCap(self.root)
        self.max_path.write_text("1500000\n")
        before = self.max_path.stat().st_mtime_ns
        cap.set_max_khz(1_500_000)
        self.assertEqual(self.max_path.stat().st_mtime_ns, before)

    def test_external_change_is_corrected(self) -> None:
        cap = CpuFreqCap(self.root)
        cap.set_max_khz(1_500_000)
        self.max_path.write_text("2400000\n")    # 外から戻されても
        cap.set_max_khz(1_500_000)                # 次の呼び出しで直す
        self.assertEqual(self.max_path.read_text(), "1500000")

    def test_missing_policy_is_unavailable(self) -> None:
        cap = CpuFreqCap(self.root / "nope")
        self.assertFalse(cap.available)
        self.assertFalse(cap.set_max_khz(1_500_000))
        self.assertFalse(cap.restore())

    @unittest.skipIf(os.geteuid() == 0, "root は読み取り専用でも書ける")
    def test_read_only_is_unavailable(self) -> None:
        self.max_path.chmod(0o444)
        self.assertFalse(CpuFreqCap(self.root).available)


if __name__ == "__main__":
    unittest.main()
