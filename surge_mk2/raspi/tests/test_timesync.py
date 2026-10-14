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


class TestOffsetBeforeFit(unittest.TestCase):
    def test_large_drift_does_not_skew_time_before_regression_starts(self):
        """回帰が始まる前（最初の 15 秒）も、ドリフトのぶんずれた時刻を返さない（2026-10-13）。

        実機の STM32 の時計は Pi より約 0.35% 速い。窓全体で遅延が最小の1点からオフセットを
        取っていた頃は、その点が古いほどずれた（10 秒前の点なら 35ms）。"""
        clock = Clock(offset_us=1_000_000, ratio=1 + 3500e-6)
        ts, pi = run_session(clock, n=50)       # 50 × 200ms = 10秒 < 15秒ガード
        self.assertIsNone(ts.drift_ppm)
        pi_est = ts.to_pi_ns(clock.stm_us(pi))
        # 最後の PONG から 200ms ぶんのドリフト（0.7ms）だけが残る
        self.assertAlmostEqual(pi_est, pi, delta=1_000_000)


def _run_delays(clock, owd_ns_list, period_ns=200_000_000, proc_us=300):
    """片道の遅延 [ns] を1往復ごとに指定して回す。各往復の直後の換算誤差 [ns] の列も返す。"""
    ts = TimeSync()
    pi = 10_000_000_000
    errs = []
    for owd in owd_ns_list:
        t1 = pi
        t2_pi = t1 + owd
        t3_pi = t2_pi + proc_us * 1000
        t4 = t3_pi + owd
        ts.observe_stm_us(clock.stm_us(t2_pi))
        ts.add_pong(t1, clock.stm_us(t2_pi), clock.stm_us(t3_pi), t4)
        q = t4 + 100_000_000                     # 次の PONG までの間に来る TELEMETRY
        errs.append(ts.to_pi_ns(clock.stm_us(q)) - q)
        pi = t1 + period_ns
    return ts, errs


class TestFitIsKept(unittest.TestCase):
    """回帰に使える点が足りなくなっても、古い1点のオフセットへ落とさない。

    STM32 の時計は Pi より約 0.35% 速いので、51 秒ぶんの窓の中の古い点を基準にすると
    最大約 0.18 秒ずれる。"""

    CLOCK = dict(offset_us=5_000_000, ratio=1 + 3500e-6)

    def test_a_lasting_rise_of_the_round_trip_delay_does_not_skew_time(self):
        # 60 秒は片道 0.2ms、その後は 0.75ms（往復 1.5ms）が続く: 新しい点は全部「遅延が小さい」から
        # 外れ、古い点だけが残って回帰のスパンが 15 秒を切る
        ts, errs = _run_delays(Clock(**self.CLOCK), [200_000] * 300 + [750_000] * 400)
        self.assertLess(max(abs(e) for e in errs[80:]), 500_000)
        self.assertIsNotNone(ts.drift_ppm)

    def test_one_sample_with_a_tiny_delay_does_not_skew_time(self):
        owd = [200_000] * 700
        owd[350] = 10_000                        # 1つだけ遅延が小さい → 他が全部しきい値の外
        ts, errs = _run_delays(Clock(**self.CLOCK), owd)
        self.assertLess(max(abs(e) for e in errs[80:]), 500_000)

    def test_before_the_first_fit_a_stale_good_sample_is_not_used(self):
        # 回帰が始まる前（15 秒未満）に遅延が増えた: 「遅延が小さい」のは増える前の点だけ
        ts, errs = _run_delays(Clock(**self.CLOCK), [200_000] * 10 + [750_000] * 50)
        self.assertIsNone(ts.drift_ppm)
        self.assertLess(abs(errs[-1]), 6_000_000)     # 直近 8 個（1.6 秒）の中の点から。古い点なら 35ms

    def test_reset_forgets_the_fit(self):
        ts, _ = _run_delays(Clock(**self.CLOCK), [200_000] * 300)
        self.assertIsNotNone(ts.drift_ppm)
        ts.reset()
        self.assertIsNone(ts.drift_ppm)
        self.assertIsNone(ts.to_pi_ns(0))


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


class TestStmReset(unittest.TestCase):
    """issue #9: STM32 が再起動すると `t_us` が0付近へ戻る。検出してリセットすること。"""

    def test_backward_jump_not_near_wrap_triggers_reset(self):
        ts = TimeSync()
        clock = Clock(offset_us=600_000_000, ratio=1.0)  # 6e8系
        # 6e8 系列を流して基準を進めておく
        for i in range(20):
            ts.observe_stm_us(clock.stm_us(10_000_000_000 + i * 100_000_000))
        self.assertGreater(ts._ref_stm_us, 0)
        resets_before = ts.resets
        # STM32 が再起動した想定: 生値が1e3系列（=ほぼ0）へ戻る。
        # 直前の生値は U32 の終端付近ではないので、正規のラップでは説明できない
        ts.observe_stm_us(1_000)
        self.assertEqual(ts.resets, resets_before + 1)
        self.assertEqual(ts._ref_stm_us, 1_000)     # リセット後は新しい生値が基準になる
        self.assertEqual(ts.n_samples, 0)          # 古いサンプルは破棄される

    def test_legitimate_u32_wrap_does_not_reset(self):
        ts = TimeSync()
        ts.observe_stm_us(U32 - 500)
        resets_before = ts.resets
        # 正規のラップ: 直前の生値が終端付近から小さい値へ
        ts.observe_stm_us(500)
        self.assertEqual(ts.resets, resets_before)

    def test_small_jitter_backward_does_not_reset(self):
        ts = TimeSync()
        ts.observe_stm_us(5_000_000)
        resets_before = ts.resets
        ts.observe_stm_us(4_999_900)               # 100us の後退（ジッタ相当）
        self.assertEqual(ts.resets, resets_before)


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
