"""E-Stop ハートビートと状態表示の単体テスト（GPIO 不要）。

安全機構なので、**正常に動くこと**より**止まるべきときに止まること**を厚く見る。
時刻は仮想クロックで進めるので、スレッドも sleep も要らず決定的に検証できる。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.io.gpio import (  # noqa: E402
    PIN_HEARTBEAT,
    FakePin,
    Heartbeat,
    StatusIndicator,
    open_output,
)


class VirtualClock:
    """テスト用の時計。`sleep` は実際には待たず時刻だけ進める。"""

    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def now(self) -> float:
        return self.t

    def sleep(self, d: float) -> None:
        self.t += max(0.0, d)

    def advance(self, d: float) -> None:
        self.t += d


def make(hz=100, kick_timeout_s=0.1):
    """スレッドを起こさずに `_step()` を直接叩ける状態の Heartbeat を作る。

    `_running` を立てておかないと `alive` が常に False になり、
    停止判定のテストが素通りしてしまう。
    """
    clk = VirtualClock()
    pin = FakePin()
    hb = Heartbeat(pin, hz=hz, kick_timeout_s=kick_timeout_s,
                   clock=clk.now, sleep=clk.sleep)
    hb._running = True
    hb._last_kick = clk.now()
    hb._next_t = clk.now() + hb._half
    return clk, pin, hb


def run_steps(clk, hb, n, kick_every=True):
    """`n` 回トグルを進める。`kick_every=False` ならメインループ停止を模す。"""
    for _ in range(n):
        clk.t = hb._next_t
        if kick_every:
            hb.kick()
        hb._step()


class TestWaveform(unittest.TestCase):
    def test_toggles_at_twice_the_frequency(self):
        """100Hz の矩形波 = 5ms ごとのトグル = 1秒で 200 エッジ。"""
        clk, pin, hb = make(hz=100)
        run_steps(clk, hb, 200)
        self.assertEqual(hb.stats.edges, 200)
        self.assertEqual(pin.writes, 200)

    def test_level_alternates(self):
        clk, pin, hb = make()
        levels = []
        for _ in range(6):
            clk.t = hb._next_t
            hb.kick()
            hb._step()
            levels.append(pin.level)
        self.assertEqual(levels, [True, False, True, False, True, False])

    def test_schedule_does_not_drift(self):
        clk, _pin, hb = make(hz=100)
        t0 = hb._next_t
        run_steps(clk, hb, 1000)
        # 5ms × 1000 回ぶん、基準時刻がきっちり進んでいること
        self.assertAlmostEqual(hb._next_t - t0, 1000 * 0.005, places=6)

    def test_large_delay_resets_the_schedule(self):
        """大きく遅れたら取り戻そうとしないこと（溜めたエッジを吐いても無意味）。"""
        clk, _pin, hb = make(hz=100)
        hb.kick()
        clk.advance(0.5)                   # 500ms 遅れる
        hb._step()
        # 次の予定は「今から 5ms 後」であって、500ms 前を基準にしていないこと
        self.assertAlmostEqual(hb._next_t - clk.now(), 0.005, places=6)


class TestStallDetection(unittest.TestCase):
    """メインループが固まったら波形を止めること。ここが安全機構の肝。"""

    def test_stops_when_kick_stops(self):
        clk, pin, hb = make(kick_timeout_s=0.1)
        run_steps(clk, hb, 10)
        before = hb.stats.edges

        run_steps(clk, hb, 100, kick_every=False)   # kick を止める
        self.assertGreater(hb.stats.skipped, 0)
        self.assertEqual(hb.stats.stalls, 1)
        self.assertFalse(hb.alive)
        # タイムアウト後はエッジが出ていないこと
        self.assertLess(hb.stats.edges - before, 21)   # 100ms/5ms = 20 回まで

    def test_keeps_running_within_timeout(self):
        """タイムアウト未満の遅れでは止めないこと（誤発報を避ける）。"""
        clk, _pin, hb = make(kick_timeout_s=0.1)
        hb.kick()
        clk.advance(0.09)
        hb._step()
        self.assertEqual(hb.stats.skipped, 0)
        self.assertTrue(hb.stats.edges, 1)

    def test_resumes_when_kick_returns(self):
        clk, _pin, hb = make(kick_timeout_s=0.1)
        run_steps(clk, hb, 50, kick_every=False)
        self.assertFalse(hb.alive)
        run_steps(clk, hb, 5)
        self.assertTrue(hb.alive)

    def test_repeated_stalls_counted_separately(self):
        clk, _pin, hb = make(kick_timeout_s=0.1)
        for _ in range(3):
            run_steps(clk, hb, 40, kick_every=False)
            run_steps(clk, hb, 5)
        self.assertEqual(hb.stats.stalls, 3)

    def test_none_timeout_never_stalls(self):
        """kick を要求しないモード（波形確認用）。"""
        clk, _pin, hb = make(kick_timeout_s=None)
        run_steps(clk, hb, 100, kick_every=False)
        self.assertEqual(hb.stats.edges, 100)
        self.assertEqual(hb.stats.skipped, 0)


class TestJitterAccounting(unittest.TestCase):
    def test_late_is_measured(self):
        clk, _pin, hb = make()
        hb.kick()
        clk.advance(0.008)                  # 予定より 3ms 遅れ
        hb._step()
        self.assertAlmostEqual(hb.stats.max_late_ns / 1e6, 3.0, places=3)

    def test_histogram_buckets(self):
        clk, _pin, hb = make()
        for extra in (0.0005, 0.0015, 0.003, 0.007, 0.02):
            hb.kick()
            clk.t = hb._next_t + extra
            hb._step()
        self.assertEqual(set(hb.stats.late_hist),
                         {"<1ms", "<2ms", "<5ms", "<10ms", "<50ms"})

    def test_on_time_is_not_counted_as_late(self):
        clk, _pin, hb = make()
        run_steps(clk, hb, 20)
        self.assertEqual(hb.stats.max_late_ns, 0)


class TestLifecycle(unittest.TestCase):
    def test_real_thread_produces_edges(self):
        """実スレッドでも動くこと（仮想クロックだけで満足しない）。"""
        pin = FakePin()
        hb = Heartbeat(pin, hz=100)
        hb.start()
        for _ in range(12):
            hb.kick()
            time.sleep(0.005)
        hb.stop()
        self.assertGreater(hb.stats.edges, 4)

    def test_stop_closes_the_pin(self):
        pin = FakePin()
        hb = Heartbeat(pin, hz=100)
        hb.start()
        hb.stop()
        self.assertTrue(pin.closed)
        self.assertFalse(pin.level)

    def test_stop_is_idempotent(self):
        hb = Heartbeat(FakePin())
        hb.start()
        hb.stop()
        hb.stop()

    def test_context_manager(self):
        pin = FakePin()
        with Heartbeat(pin, hz=100) as hb:
            hb.kick()
        self.assertTrue(pin.closed)

    def test_alive_is_false_before_start_and_after_stop(self):
        hb = Heartbeat(FakePin())
        self.assertFalse(hb.alive)
        hb.start()
        self.assertTrue(hb.alive)
        hb.stop()
        self.assertFalse(hb.alive)


class TestOpenOutput(unittest.TestCase):
    """`open_output` は環境で結果が変わる。**実機とそれ以外の両方を書く。**

    E-Stop のピンをテストで掴みっぱなしにしないよう、開けたものは必ず閉じる。
    """

    def test_unopenable_pin_returns_none(self):
        """開けないピンでは例外ではなく None（io_node が起動できるように）。"""
        self.assertIsNone(open_output(9999))

    def test_real_pin_works_when_gpio_is_available(self):
        pin = open_output(PIN_HEARTBEAT)
        if pin is None:
            self.skipTest("GPIO の無い環境")
        try:
            pin.write(True)
            pin.write(False)
        finally:
            pin.close()


class TestStatusIndicator(unittest.TestCase):
    def setUp(self):
        self.clk = VirtualClock()
        self.g, self.r, self.b = FakePin("g"), FakePin("r"), FakePin("b")
        self.ind = StatusIndicator(self.g, self.r, self.b, clock=self.clk.now)

    def test_ok_is_green(self):
        self.ind.update("OK")
        self.assertTrue(self.g.level)
        self.assertFalse(self.r.level)

    def test_fault_is_red_and_beeps(self):
        self.ind.update("OK")
        self.ind.update("FAULT")
        self.assertFalse(self.g.level)
        self.assertTrue(self.r.level)
        self.assertTrue(self.b.level)

    def test_beep_stops_after_the_window(self):
        self.ind.update("FAULT")
        self.clk.advance(StatusIndicator.BEEP_S + 0.01)
        self.ind.update("FAULT")
        self.assertFalse(self.b.level)

    def test_no_repeat_beep_while_still_faulted(self):
        self.ind.update("FAULT")
        self.clk.advance(1.0)
        self.ind.update("FAULT")
        self.assertFalse(self.b.level)

    def test_degraded_blinks_green(self):
        seen = set()
        for _ in range(8):
            self.ind.update("DEGRADED")
            seen.add(self.g.level)
            self.clk.advance(0.13)
        self.assertEqual(seen, {True, False})

    def test_missing_pins_are_tolerated(self):
        """ブザーだけ配線されていない、のような状態でも落ちないこと。"""
        ind = StatusIndicator(FakePin(), None, None, clock=self.clk.now)
        ind.update("FAULT")
        ind.close()

    def test_estop_overrides_health(self):
        """リンクが健全でも E-Stop 中は E-Stop の表示が勝つこと。"""
        self.ind.update("OK", estop=True)
        self.assertFalse(self.g.level)
        self.assertTrue(self.b.level)          # 発動時は長めのブザー

    def test_estop_blinks_red_faster_than_init(self):
        seen = set()
        for _ in range(8):
            self.ind.update("OK", estop=True)
            seen.add(self.r.level)
            self.clk.advance(0.065)            # 4Hz を捉える間隔
        self.assertEqual(seen, {True, False})

    def test_estop_beep_is_longer_than_fault_beep(self):
        self.ind.update("OK", estop=True)
        self.clk.advance(StatusIndicator.BEEP_S + 0.01)
        self.ind.update("OK", estop=True)
        self.assertTrue(self.b.level)          # FAULT 用の 0.2秒では終わらない
        self.clk.advance(StatusIndicator.ESTOP_BEEP_S)
        self.ind.update("OK", estop=True)
        self.assertFalse(self.b.level)

    def test_no_repeat_beep_while_estop_persists(self):
        self.ind.update("OK", estop=True)
        self.clk.advance(5.0)
        self.ind.update("OK", estop=True)
        self.assertFalse(self.b.level)

    def test_recovery_from_estop(self):
        self.ind.update("OK", estop=True)
        self.ind.update("OK", estop=False)
        self.assertTrue(self.g.level)
        self.assertFalse(self.r.level)

    def test_close_turns_everything_off(self):
        self.ind.update("FAULT")
        self.ind.close()
        self.assertFalse(self.r.level)
        self.assertTrue(self.g.closed and self.r.closed and self.b.closed)


if __name__ == "__main__":
    unittest.main()
