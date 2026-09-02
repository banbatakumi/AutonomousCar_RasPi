"""`camera_node.CameraWorker` の連続キャプチャ失敗リトライ判定（B2）。

一過性の `_grab()` 例外1回でスレッドが恒久停止しないよう、連続失敗回数が
閾値に達したときだけ諦める設計にした。`_should_give_up()` はモジュール
レベルの純粋関数（picamera2非依存）なのでMacでもそのままテストできる。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.nodes.camera_node import (  # noqa: E402
    MAX_CONSECUTIVE_GRAB_FAILURES,
    CamStats,
    _MAX_GAPS_SAMPLES,
    _should_give_up,
)


class TestShouldGiveUp(unittest.TestCase):
    def test_below_threshold_keeps_retrying(self):
        for n in range(MAX_CONSECUTIVE_GRAB_FAILURES):
            self.assertFalse(_should_give_up(n, MAX_CONSECUTIVE_GRAB_FAILURES),
                             f"{n}回目はまだ諦めるべきではない")

    def test_reaching_threshold_gives_up(self):
        self.assertTrue(_should_give_up(MAX_CONSECUTIVE_GRAB_FAILURES,
                                        MAX_CONSECUTIVE_GRAB_FAILURES))

    def test_a_single_transient_failure_does_not_give_up(self):
        """一過性の1回失敗では継続すること（B2の主目的）。"""
        self.assertFalse(_should_give_up(1, MAX_CONSECUTIVE_GRAB_FAILURES))


class TestCamStatsGapsCap(unittest.TestCase):
    """★ D4: `gaps_ms` が無制限に伸びず、上限で頭打ちになること。"""

    def test_caps_at_max_gaps_samples(self):
        s = CamStats()
        for i in range(_MAX_GAPS_SAMPLES + 500):
            s.gaps_ms.append(float(i))
        self.assertEqual(len(s.gaps_ms), _MAX_GAPS_SAMPLES)

    def test_summary_still_works_with_deque(self):
        s = CamStats()
        s.frames = 10
        for i in range(5):
            s.gaps_ms.append(float(i))
        text = s.summary(1.0)
        self.assertIn("fps", text)


if __name__ == "__main__":
    unittest.main()
