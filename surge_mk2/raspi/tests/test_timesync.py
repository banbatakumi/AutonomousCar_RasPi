"""時刻同期の単体テスト（ハードウェア不要・合成クロックで検証）。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.timesync import U32, TimeSync  # noqa: E402


class Clock:
    """合成 STM32 クロック。t_stm = offset_us + t_pi_us * ratio、u32 でラップ。"""

    def __init__(self, offset_us: int, ratio: float = 1.0):
        self.offset_us = offset_us
        self.ratio = ratio

    def stm_us(self, pi_ns: int) -> int:
        t = self.offset_us + int((pi_ns / 1000) * self.ratio)
        return t & 0xFFFFFFFF


def run_session(clock, *, n=60, period_ns=200_000_000, proc_us=300,
                jitter_ns=None, start_pi_ns=10_000_000_000):
    """PING/PONG を n 回まわし、TimeSync に食わせる。

    jitter_ns: 各往復に足す片道の遅延ジッタ列（None なら 0）。
    """
    ts = TimeSync()
    pi = start_pi_ns
    for i in range(n):
        t1 = pi
        up = jitter_ns[i][0] if jitter_ns else 0     # Pi→STM32 の片道
        down = jitter_ns[i][1] if jitter_ns else 0   # STM32→Pi の片道
        base_owd = 500_000                            # 基本片道 0.5ms
        t2_pi = t1 + base_owd + up
        t2 = clock.stm_us(t2_pi)
        t3_pi = t2_pi + proc_us * 1000                # STM32 処理時間
        t3 = clock.stm_us(t3_pi)
        t4 = t3_pi + base_owd + down
        ts.add_pong(t1, t2, t3, t4)
        # 次サイクルまでに TELEMETRY が来ている想定で基準を更新
        pi = t4 + period_ns
        ts.observe_stm_us(clock.stm_us(pi))
    return ts, pi


class TestUnwrap(unittest.TestCase):
    def test_basic(self):
        ts = TimeSync()
        ts.observe_stm_us(1000)
        self.assertEqual(ts._unwrap(1000), 1000)

    def test_wrap_forward(self):
        ts = TimeSync()
        # 基準を u32 上限近くに置く
        near_top = U32 - 1000
        ts.observe_stm_us(near_top)
        # ラップ後の小さい raw は次の周回として解釈されるべき
        unwrapped = ts._unwrap(500)
        self.assertEqual(unwrapped, U32 + 500)

    def test_no_spurious_jump(self):
        ts = TimeSync()
        ts.observe_stm_us(1_000_000)
        self.assertEqual(ts._unwrap(1_000_100), 1_000_100)


class TestOffsetRecovery(unittest.TestCase):
    def test_pure_offset_no_jitter(self):
        clock = Clock(offset_us=12_345_678, ratio=1.0)
        ts, _ = run_session(clock, n=40)
        # ジッタ無しなら offset は理論値にほぼ一致
        # offset(ns) = t_pi - t_stm。STM32 = pi_us + offset_us なので t_pi - t_stm = -offset_us
        off_ns = ts.offset_ns
        self.assertIsNotNone(off_ns)
        self.assertAlmostEqual(off_ns / 1000, -12_345_678, delta=1.0)

    def test_conversion_roundtrip(self):
        clock = Clock(offset_us=5_000_000, ratio=1.0)
        ts, pi = run_session(clock, n=40)
        # ある STM32 時刻を Pi 時刻に戻すと、生成に使った関係と一致するはず
        stm_now = clock.stm_us(pi)
        pi_est = ts.to_pi_ns(stm_now)
        self.assertIsNotNone(pi_est)
        self.assertAlmostEqual(pi_est, pi, delta=50_000)   # 50μs 以内


class TestDriftRecovery(unittest.TestCase):
    def test_positive_drift(self):
        # STM32 が Pi より 50ppm 速い。ドリフト推定には十分な時間スパンが要る
        clock = Clock(offset_us=1_000_000, ratio=1 + 50e-6)
        ts, _ = run_session(clock, n=200)      # 200 × 200ms = 40秒
        drift = ts.drift_ppm
        self.assertIsNotNone(drift)
        # to_pi_ns は STM32→Pi 変換なので、回帰の傾き b ≈ 1/ratio → drift ≈ -50ppm
        self.assertAlmostEqual(drift, -50.0, delta=5.0)

    def test_conversion_accurate_under_drift(self):
        clock = Clock(offset_us=2_000_000, ratio=1 + 30e-6)
        ts, pi = run_session(clock, n=200)
        stm_now = clock.stm_us(pi)
        pi_est = ts.to_pi_ns(stm_now)
        self.assertAlmostEqual(pi_est, pi, delta=100_000)

    def test_drift_gated_until_enough_span(self):
        """短い時間スパンではドリフトを出さない（ジッタで暴れるのを防ぐ）。"""
        clock = Clock(offset_us=1_000_000, ratio=1 + 30e-6)
        ts, _ = run_session(clock, n=20)       # 20 × 200ms = 4秒 < 15秒ガード
        self.assertIsNone(ts.drift_ppm)         # まだ回帰しない
        self.assertIsNotNone(ts.offset_ns)      # オフセットは出る


class TestJitterRejection(unittest.TestCase):
    def test_min_filter_ignores_delayed_samples(self):
        clock = Clock(offset_us=7_777_777, ratio=1.0)
        # 偶数サイクルはクリーン、奇数サイクルは大きく遅延させる
        jit = []
        for i in range(60):
            if i % 2:
                jit.append((3_000_000, 4_000_000))   # 数ms の非対称遅延
            else:
                jit.append((0, 0))
        ts, _ = run_session(clock, n=60, jitter_ns=jit)
        # min filter がクリーンなサンプルを選ぶので理論オフセットに近いはず
        self.assertAlmostEqual(ts.offset_ns / 1000, -7_777_777, delta=50.0)

    def test_bad_sample_rejected(self):
        ts = TimeSync()
        # T4 が T1 より前（物理的にありえない）→ 破棄
        self.assertIsNone(ts.add_pong(t1_ns=1000, t2_us=10, t3_us=20, t4_ns=500))
        self.assertEqual(ts.n_samples, 0)


class TestWrapDuringSession(unittest.TestCase):
    def test_offset_stable_across_wrap(self):
        # 基準を u32 上限直前に置き、セッション中にラップさせる
        clock = Clock(offset_us=U32 - 2_000_000, ratio=1.0)
        ts, pi = run_session(clock, n=60, period_ns=100_000_000)
        # ラップを跨いでも変換が破綻しないこと
        stm_now = clock.stm_us(pi)
        pi_est = ts.to_pi_ns(stm_now)
        self.assertIsNotNone(pi_est)
        self.assertAlmostEqual(pi_est, pi, delta=100_000)


class TestReadiness(unittest.TestCase):
    def test_not_ready_initially(self):
        ts = TimeSync()
        self.assertFalse(ts.ready())
        self.assertIsNone(ts.offset_ns)
        self.assertIsNone(ts.to_pi_ns(123))

    def test_ready_after_samples(self):
        clock = Clock(offset_us=1, ratio=1.0)
        ts, _ = run_session(clock, n=10)
        self.assertTrue(ts.ready())


if __name__ == "__main__":
    unittest.main(verbosity=2)
