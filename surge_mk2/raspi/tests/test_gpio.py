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
    Indication,
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
    """LED 2個で何を伝えるかの仕様を固定する。

    - 緑 = 生存と可動性（**変化し続けること自体が生存の証拠**）
    - 赤 = 異常の重さ（＝人間が何をすべきか）
    """

    def setUp(self):
        self.clk = VirtualClock()
        self.g, self.r, self.b = FakePin("g"), FakePin("r"), FakePin("b")
        self.ind = StatusIndicator(self.g, self.r, self.b, clock=self.clk.now)

    def show(self, seconds: float, **kw):
        """指定秒ぶん進めながら更新し、緑・赤それぞれが取った値の集合を返す。"""
        gs, rs = set(), set()
        for _ in range(max(2, int(seconds / 0.02))):
            self.ind.update(Indication(**kw))
            gs.add(self.g.level)
            rs.add(self.r.level)
            self.clk.advance(0.02)
        return gs, rs

    def duty(self, seconds: float, pin, **kw) -> float:
        """指定秒のあいだ、そのピンが点いていた割合。"""
        n = max(2, int(seconds / 0.01))
        on = 0
        for _ in range(n):
            self.ind.update(Indication(**kw))
            on += pin.level
            self.clk.advance(0.01)
        return on / n

    # ── 緑 ──

    def test_armed_is_solid_green(self):
        """車輪が動きうる状態が一番目立つこと。"""
        gs, _ = self.show(1.5, health="OK", armed=True)
        self.assertEqual(gs, {True})

    def test_disarmed_green_flashes(self):
        """点きっぱなしにしない。**明滅そのものが生存の証拠**。"""
        gs, _ = self.show(2.5, health="OK", armed=False)
        self.assertEqual(gs, {True, False})

    def test_disarmed_flash_is_short(self):
        """ARMED の点灯と見間違えないよう、消えている時間のほうが長いこと。"""
        self.assertLess(self.duty(2.0, self.g, health="OK"), 0.3)

    def test_init_blinks_green_more_than_alive_flash(self):
        init = self.duty(1.0, self.g, health="INIT")
        self.clk.advance(1.0)
        alive = self.duty(2.0, self.g, health="OK")
        self.assertGreater(init, alive)

    def test_armed_wins_over_init(self):
        gs, _ = self.show(1.0, health="INIT", armed=True)
        self.assertEqual(gs, {True})

    # ── 赤 ──

    def test_no_trouble_is_dark(self):
        _, rs = self.show(2.0, health="OK")
        self.assertEqual(rs, {False})

    def test_latched_is_solid_red(self):
        """人間が車両を触るまで戻らない状態を最も強く出す。"""
        _, rs = self.show(2.0, health="OK", estop=True)
        self.assertEqual(rs, {True})

    def test_power_lock_is_solid_red_too(self):
        _, rs = self.show(2.0, health="OK", power_locked=True)
        self.assertEqual(rs, {True})

    def test_fault_blinks_red(self):
        _, rs = self.show(1.0, health="FAULT")
        self.assertEqual(rs, {True, False})

    def test_warning_blinks_red(self):
        _, rs = self.show(2.0, health="OK", warning=True)
        self.assertEqual(rs, {True, False})

    def test_degraded_is_warning_tier(self):
        _, rs = self.show(2.0, health="DEGRADED")
        self.assertEqual(rs, {True, False})

    def test_fault_blinks_faster_than_warning(self):
        fault = self._transitions(1.0, health="FAULT")
        self.clk.advance(1.0)
        warn = self._transitions(1.0, health="OK", warning=True)
        self.assertGreater(fault, warn)

    def _transitions(self, seconds: float, **kw) -> int:
        prev = None
        n = 0
        for _ in range(int(seconds / 0.01)):
            self.ind.update(Indication(**kw))
            if prev is not None and self.r.level != prev:
                n += 1
            prev = self.r.level
            self.clk.advance(0.01)
        return n

    def test_latched_wins_over_fault(self):
        _, rs = self.show(2.0, health="FAULT", estop=True)
        self.assertEqual(rs, {True})

    # ── 緑と赤は独立 ──

    def test_axes_are_independent(self):
        """ARMED かつ警告あり、が同時に出せること。"""
        gs, rs = self.show(2.0, health="OK", armed=True, warning=True)
        self.assertEqual(gs, {True})
        self.assertEqual(rs, {True, False})

    # ── ブザー ──

    def test_beeps_on_entering_estop(self):
        self.ind.update(Indication(health="OK"))
        self.ind.update(Indication(health="OK", estop=True))
        self.assertTrue(self.b.level)

    def test_no_beep_while_estop_persists(self):
        self.ind.update(Indication(health="OK"))
        self.ind.update(Indication(health="OK", estop=True))
        self.clk.advance(5.0)
        self.ind.update(Indication(health="OK", estop=True))
        self.assertFalse(self.b.level)

    def test_beeps_on_entering_fault(self):
        self.ind.update(Indication(health="OK"))
        self.ind.update(Indication(health="FAULT"))
        self.assertTrue(self.b.level)

    def test_no_beep_on_first_update(self):
        """起動直後にいきなり鳴らさないこと。"""
        self.ind.update(Indication(health="FAULT", estop=True))
        self.assertFalse(self.b.level)

    def test_estop_beep_is_longer_than_fault_beep(self):
        self.ind.update(Indication(health="OK"))
        self.ind.update(Indication(health="OK", estop=True))
        self.clk.advance(StatusIndicator.BEEP_S + 0.01)
        self.ind.update(Indication(health="OK", estop=True))
        self.assertTrue(self.b.level)
        self.clk.advance(StatusIndicator.ESTOP_BEEP_S)
        self.ind.update(Indication(health="OK", estop=True))
        self.assertFalse(self.b.level)

    # ── 配線の欠けに強いこと ──

    def test_missing_pins_are_tolerated(self):
        ind = StatusIndicator(FakePin(), None, None, clock=self.clk.now)
        ind.update(Indication(health="FAULT", estop=True))
        ind.close()

    def test_close_turns_everything_off(self):
        self.ind.update(Indication(health="OK", estop=True))
        self.ind.close()
        self.assertFalse(self.r.level)
        self.assertTrue(self.g.closed and self.r.closed and self.b.closed)


if __name__ == "__main__":
    unittest.main()
