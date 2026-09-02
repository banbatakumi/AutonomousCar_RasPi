"""`raspi/core/cleanup.py` の後始末カウンタ（C7: マルチスレッド競合）。"""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core import cleanup  # noqa: E402


class TestQuietClose(unittest.TestCase):
    def test_swallows_the_exception(self):
        with cleanup.quiet_close("test resource"):
            raise RuntimeError("boom")  # ここで例外は外に出ない

    def test_records_the_failure(self):
        before = cleanup.failure_count()
        with cleanup.quiet_close("widget"):
            raise ValueError("bad")
        self.assertEqual(cleanup.failure_count(), before + 1)
        what, reason = cleanup.recent_failures()[-1][1:]
        self.assertEqual(what, "widget")
        self.assertIn("ValueError", reason)


class TestConcurrentFailures(unittest.TestCase):
    """★ C7: 複数スレッドから同時に失敗を記録しても `_COUNT` がずれないこと。"""

    def test_count_matches_the_number_of_calls_under_contention(self):
        before = cleanup.failure_count()
        n_threads = 20
        n_per_thread = 200

        def worker():
            for _ in range(n_per_thread):
                cleanup.note_cleanup_failure("stress", RuntimeError("x"))

        threads = [threading.Thread(target=worker) for _ in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.assertEqual(cleanup.failure_count(), before + n_threads * n_per_thread)


if __name__ == "__main__":
    unittest.main()
