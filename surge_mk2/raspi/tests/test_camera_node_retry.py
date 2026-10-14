"""`camera_node.CameraWorker` の連続キャプチャ失敗リトライ判定（B2）。

一過性の `_grab()` 例外1回でスレッドが恒久停止しないよう、連続失敗回数が
閾値に達したときだけ諦める設計にした。`_should_give_up()` はモジュール
レベルの純粋関数（picamera2非依存）なのでMacでもそのままテストできる。
"""

from __future__ import annotations

import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.nodes.camera_node import (  # noqa: E402
    MAX_CONSECUTIVE_GRAB_FAILURES,
    CameraNode,
    CamStats,
    _MAX_GAPS_SAMPLES,
    _should_give_up,
    _Stamped,
    count_dropped,
    devices_by_i2c,
    run_bounded,
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


class _FakeCameraWorker(threading.Thread):
    """`CameraWorker`の身代わり。picamera2 に触らず、`CameraNode.run()`の
    生死監視・ハング監視だけを踏む（実カメラ無しでMacでも動く）。
    """

    def __init__(self, idx: int, *, dies_immediately: bool = False,
                progresses: bool = True) -> None:
        super().__init__(daemon=True, name=f"fakecam{idx}")
        self.idx = idx
        self.error: Exception | None = None
        self.stats = CamStats()
        self._dies_immediately = dies_immediately
        self._progresses = progresses
        self._running = False

    def start_camera(self) -> None:
        pass

    def run(self) -> None:
        if self._dies_immediately:
            # 実ワーカーの「連続失敗で諦める」（B2）と同じく、死ぬときは
            # 必ず`error`を残してから終わる
            self.error = RuntimeError("模擬カメラ異常")
            return
        self._running = True
        while self._running:
            if self._progresses:
                self.stats.last_grab_ns = time.monotonic_ns()
            time.sleep(0.01)

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        pass


class TestCameraNodeDeadWorkerStopsTheNode(unittest.TestCase):
    """issue #4: 前後どちらか1台だけ死んでも、ノード全体を終了させること。

    以前は `any(is_alive())`（全滅するまで回り続ける）で、1台生きていれば
    もう1台が死んでも誰も気づかなかった。
    """

    def test_one_dead_worker_ends_run_promptly(self):
        node = CameraNode.__new__(CameraNode)
        node.workers = [_FakeCameraWorker(0, progresses=True),
                        _FakeCameraWorker(1, dies_immediately=True)]
        node._cfg_sub = None
        node._cfg_thread = None
        node._t_start = 0.0

        t0 = time.monotonic()
        node.run(duration_s=5.0)               # 5秒待たずに戻ってくること
        elapsed = time.monotonic() - t0

        self.assertLess(elapsed, 1.0, "1台死んでもノードが終了していない")
        self.assertTrue(any(w.error for w in node.workers))


class TestCameraNodeHangDetection(unittest.TestCase):
    """issue #4: `capture_request()`がブロックしたままのハングを検出する。

    スレッド自体は`is_alive()`のままなので、生死監視だけでは検出できない。
    `CamStats.last_grab_ns`が進まなくなったら`hang_s`後にノードを終了させる。
    """

    def test_stalled_worker_is_detected_as_hung(self):
        node = CameraNode.__new__(CameraNode)
        node.workers = [_FakeCameraWorker(0, progresses=False)]   # 進捗が止まったまま
        node._cfg_sub = None
        node._cfg_thread = None
        node._t_start = 0.0

        t0 = time.monotonic()
        node.run(duration_s=5.0, hang_s=0.1)   # 0.1s でハング判定させる
        elapsed = time.monotonic() - t0

        self.assertLess(elapsed, 1.0, "ハングしたワーカーが検出されていない")
        self.assertIsNotNone(node.workers[0].error)
        self.assertIn("ハング", str(node.workers[0].error))

    def test_progressing_worker_is_not_treated_as_hung(self):
        """回帰防止: 正常に進捗しているワーカーはハング扱いされず、
        `--duration`まで普通に動き続けること。"""
        node = CameraNode.__new__(CameraNode)
        node.workers = [_FakeCameraWorker(0, progresses=True)]
        node._cfg_sub = None
        node._cfg_thread = None
        node._t_start = 0.0

        node.run(duration_s=0.2, hang_s=0.1)

        self.assertIsNone(node.workers[0].error)


class TestDropCount(unittest.TestCase):
    def test_normal_jitter_is_not_a_drop(self):
        self.assertEqual(count_dropped(33.3e6 * 1.4, 33.3e6), 0)

    def test_one_missing_frame(self):
        self.assertEqual(count_dropped(66.6e6, 33.3e6), 1)

    def test_three_missing_frames(self):
        self.assertEqual(count_dropped(133.3e6, 33.3e6), 3)

    def test_unknown_period_counts_nothing(self):
        self.assertEqual(count_dropped(1e9, 0), 0)


class TestRunBounded(unittest.TestCase):
    def test_finishes_in_time(self):
        self.assertTrue(run_bounded(lambda: None, 1.0))

    def test_exception_counts_as_finished(self):
        def boom():
            raise RuntimeError("x")
        self.assertTrue(run_bounded(boom, 1.0))

    def test_hang_is_reported(self):
        ev = threading.Event()
        try:
            self.assertFalse(run_bounded(lambda: ev.wait(5.0), 0.1))
        finally:
            ev.set()


class TestDevicesByI2c(unittest.TestCase):
    INFOS = [{"Id": "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10"},
             {"Id": "/base/axi/pcie@120000/rp1/i2c@88000/imx219@10"}]

    def test_swapped_enumeration(self):
        self.assertEqual(devices_by_i2c(self.INFOS, ["front", "rear"]),
                         {"rear": 0, "front": 1})

    def test_only_one_visible(self):
        self.assertEqual(devices_by_i2c(self.INFOS[:1], ["front", "rear"]), {"rear": 0})


class TestHangLimitFollowsFps(unittest.TestCase):
    def test_slow_fps_is_not_hung(self):
        """0.5fps（周期2秒）の設定で、1枚の間隔が hang_s を超えてもハング扱いしない。"""
        w = _FakeCameraWorker(0, progresses=False)
        w.fps = 0.5
        node = CameraNode.__new__(CameraNode)
        node.workers = [w]
        node._cfg_sub = None
        node._cfg_thread = None
        node._t_start = 0.0
        node.run(duration_s=0.5, hang_s=0.1)
        self.assertIsNone(w.error)


class TestStopBeforeStart(unittest.TestCase):
    def test_stop_with_unstarted_worker_does_not_raise(self):
        node = CameraNode.__new__(CameraNode)
        node.workers = [_FakeCameraWorker(0)]       # start していない
        node._cfg_thread = None
        node._cfg_running = False
        node.stop()                                  # RuntimeError にならないこと

    def test_stop_requested_before_run_returns_immediately(self):
        node = CameraNode.__new__(CameraNode)
        node.workers = [_FakeCameraWorker(0)]
        node._cfg_sub = None
        node._cfg_thread = None
        node._t_start = 0.0
        node._stop_requested = True
        t0 = time.monotonic()
        node.run(duration_s=5.0)
        self.assertLess(time.monotonic() - t0, 2.0)


class TestStamped(unittest.TestCase):
    def test_line_start_gets_timestamp(self):
        import io
        out = io.StringIO()
        st = _Stamped(out)
        st.write("abc")
        st.write("def\nghi\n")
        lines = out.getvalue().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertRegex(lines[0], r"^\d{4}-\d\d-\d\d \d\d:\d\d:\d\d abcdef$")
        self.assertRegex(lines[1], r"^\d{4}-\d\d-\d\d \d\d:\d\d:\d\d ghi$")

    def test_carriage_return_status_is_left_alone(self):
        import io
        out = io.StringIO()
        _Stamped(out).write("\rcam0: 3枚")
        self.assertEqual(out.getvalue(), "\rcam0: 3枚")


class TestTransientErrorIsCleared(unittest.TestCase):
    def test_error_cleared_after_recovery(self):
        from raspi.nodes.camera_node import CameraWorker

        w = CameraWorker.__new__(CameraWorker)
        threading.Thread.__init__(w, daemon=True)
        w.idx = 0
        w.error = None
        w.stats = CamStats()
        w._cfg_lock = threading.Lock()
        w._pending_fps = None
        w._pending_enabled = None
        w._enabled = True
        calls = {"n": 0}

        def grab():
            calls["n"] += 1
            if calls["n"] == 1:
                raise RuntimeError("一過性")
            if calls["n"] >= 3:
                w._running = False
        w._grab = grab
        w.run()
        self.assertGreaterEqual(calls["n"], 3)
        self.assertIsNone(w.error)


if __name__ == "__main__":
    unittest.main()


class TestDisabledWorkerIsNotHung(unittest.TestCase):
    """後カメラを OFF にしている間（disarm 中の既定）は `_grab()` が呼ばれない。
    それを「ハング」と誤検出してノードごと落ちる不具合（2026-10-01）の回帰テスト。
    """

    def test_disabled_worker_keeps_last_grab_ns_fresh(self):
        from raspi.nodes.camera_node import CameraWorker

        w = CameraWorker.__new__(CameraWorker)
        threading.Thread.__init__(w, daemon=True)
        w.idx = 1
        w.error = None
        w.stats = CamStats()
        w.stats.last_grab_ns = time.monotonic_ns() - int(10e9)   # 10秒止まっている扱い
        w.fps = 10.0
        w._cfg_lock = threading.Lock()
        w._pending_fps = None
        w._pending_enabled = None
        w._enabled = False
        w._running = False
        w.start()
        try:
            time.sleep(0.5)
            age_s = (time.monotonic_ns() - w.stats.last_grab_ns) / 1e9
        finally:
            w._running = False
            w.join(timeout=2.0)
        self.assertLess(age_s, 1.0, "OFF 中でも last_grab_ns が更新されていない")
