"""`tools/sysid/fit.py` のテスト。

`fit.py`は実車ログでしか動作確認されておらず、正解値との突き合わせが
一度も行われていなかった。ここでは`sim/vehicle.py`の操舵モデル
（むだ時間→一次遅れ→レート制限、ZOH離散化）と同じ式で合成した
ステップ応答を使い、既知の`(dead_time_s, tau_steer_s, steer_rate_limit_rad_s)`
を`fit_steer`が復元できることを検証する。
"""

import math
import sys
import unittest
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))  # surge_mk2/

import numpy as np  # noqa: E402

from tools.sysid.fit import (  # noqa: E402
    Sample,
    _fit_first_order_with_delay,
    _skip_rate_limited_plateau,
    fit_accel,
    fit_steer,
)

DT = 0.02   # 50Hz。実機TELEMETRYと同じレート


def _simulate_steer(*, dead_time_s: float, tau_steer_s: float, rate_limit_rad_s: float,
                    amplitude_rad: float, hold_s: float, cycles: int,
                    dt: float = DT, echo_ramp_s: float = 0.0) -> list[Sample]:
    """`sim/vehicle.py`の`VehicleModel.step()`と同じ式（むだ時間→ZOH一次遅れ→
    レート制限）で`steer_cmd_echo`→`steer_actual`のステップ応答を合成する。

    :param echo_ramp_s: `steer_cmd_echo`自体を`target`へ一次遅れで追従させる
        時定数 [s]。既定0は従来通り`steer_cmd_echo=target`（瞬時切り替え）。
        実機の`steer_cmd_echo`は瞬時に切り替わらず、新しい値に落ち着くまで
        実測で約80ms（時定数換算で20〜30ms程度）かけてなだらかにランプする
        ことが判明した（`tools/sysid/fit.py`の`_step_responses`docstring参照）
        ——`test_recovers_known_tau_when_echo_ramps_gradually`がこの現実的な
        条件を再現する回帰テストに使う
    """
    n_steps = cycles * 2 + 1   # 中立→+A→-A→+A→-A→...
    total_t = n_steps * hold_s

    delay_q: deque[list] = deque()
    delayed = 0.0
    actual = 0.0
    echo = 0.0
    samples: list[Sample] = []

    t = 0.0
    while t < total_t:
        idx = min(int(t / hold_s), n_steps - 1)
        target = 0.0 if idx == 0 else (amplitude_rad if idx % 2 == 1 else -amplitude_rad)

        if echo_ramp_s > 1e-6:
            echo += (target - echo) * min(1.0, dt / echo_ramp_s)
        else:
            echo = target

        delay_q.append([dead_time_s, target])
        while delay_q and delay_q[0][0] <= 0.0:
            delayed = delay_q.popleft()[1]
        for item in delay_q:
            item[0] -= dt

        want = delayed
        if tau_steer_s > 1e-6:
            want = actual + (want - actual) * (1.0 - math.exp(-dt / tau_steer_s))
        if rate_limit_rad_s > 1e-6:
            d = max(-rate_limit_rad_s * dt, min(rate_limit_rad_s * dt, want - actual))
            actual += d
        else:
            actual = want

        samples.append(Sample(t=t, target_speed=0.0, target_steer=target, brake=False,
                              speed=0.0, steer_actual=actual, steer_cmd_echo=echo,
                              yaw_rate=0.0, accel_x=0.0, tc_active=False))
        t += dt

    return samples


class TestFitSteer(unittest.TestCase):
    def test_recovers_known_tau_and_dead_time(self):
        """レート制限が実質効かない（十分速い）条件では、既知の
        `dead_time_s`/`tau_steer_s`を精度良く復元できる。"""
        true_dead_time, true_tau, true_rate_limit = 0.03, 0.15, 20.0
        samples = _simulate_steer(dead_time_s=true_dead_time, tau_steer_s=true_tau,
                                  rate_limit_rad_s=true_rate_limit,
                                  amplitude_rad=math.radians(30), hold_s=1.2, cycles=4)
        result = fit_steer(samples)
        self.assertAlmostEqual(result["dead_time_s"], true_dead_time, delta=0.01)
        self.assertAlmostEqual(result["tau_steer_s"], true_tau, delta=true_tau * 0.15)

    def test_recovers_known_rate_limit_when_genuinely_saturated(self):
        """振幅に対してレート制限が実際に効く条件（実機の
        `steer_rate_limit_rad_s≈11rad/s`に対し`tau_steer_s`由来の自然な
        変化速度がそれより遅い、という現実的な比率）では、
        `steer_rate_limit_rad_s`を正しく復元し、かつプラトー区間を
        正しく除外して`tau_steer_s`も大きく崩れない。"""
        true_dead_time, true_tau, true_rate_limit = 0.03, 0.15, 3.0
        samples = _simulate_steer(dead_time_s=true_dead_time, tau_steer_s=true_tau,
                                  rate_limit_rad_s=true_rate_limit,
                                  amplitude_rad=0.5, hold_s=1.5, cycles=4)
        result = fit_steer(samples)
        self.assertAlmostEqual(result["steer_rate_limit_rad_s"], true_rate_limit,
                               delta=true_rate_limit * 0.1)
        self.assertAlmostEqual(result["tau_steer_s"], true_tau, delta=true_tau * 0.2)

    def test_recovers_known_tau_when_echo_ramps_gradually(self):
        """`steer_cmd_echo`が瞬時に切り替わらず実機のようになだらかにランプ
        しても（2026-09-01、実機ログで発覚。`_step_responses`docstring参照）、
        `tau_steer_s`・`dead_time_s`を大きく崩さず復元できる回帰テスト。

        修正前は、区間の先頭1サンプル(`target[start]`・`actual[start-1]`)を
        そのまま基準点に使っていたため、ランプの途中の値を拾って`delta`
        （ステップ振幅）を過小評価し、`tau_steer_s`が真値の10倍以上に
        水増しされていた（実測: 振幅30°の実機ログで0.5s台→6s台）。"""
        true_dead_time, true_tau, true_rate_limit = 0.03, 0.15, 20.0
        samples = _simulate_steer(dead_time_s=true_dead_time, tau_steer_s=true_tau,
                                  rate_limit_rad_s=true_rate_limit,
                                  amplitude_rad=math.radians(30), hold_s=1.2, cycles=4,
                                  echo_ramp_s=0.025)
        result = fit_steer(samples)
        self.assertAlmostEqual(result["dead_time_s"], true_dead_time, delta=0.03)
        self.assertAlmostEqual(result["tau_steer_s"], true_tau, delta=true_tau * 0.3)

    def test_slow_tau_without_saturation_is_not_biased_by_plateau_heuristic(self):
        """レート制限が実質効かないほど大きい（応答が遅い）`tau_steer_s`でも、
        自然な指数減衰の立ち上がりを「レート制限プラトー」と誤検出して
        過大評価しない（過去に発見された回帰: 50Hzサンプリングだと
        `tau≈0.5s`前後の応答の最初の数サンプルが旧ヒューリスティックの
        条件を満たしてしまい、tauがさらに底上げされていた）。"""
        true_dead_time, true_tau, true_rate_limit = 0.0, 0.54, 50.0
        samples = _simulate_steer(dead_time_s=true_dead_time, tau_steer_s=true_tau,
                                  rate_limit_rad_s=true_rate_limit,
                                  amplitude_rad=math.radians(30), hold_s=3.0, cycles=4)
        result = fit_steer(samples)
        self.assertAlmostEqual(result["tau_steer_s"], true_tau, delta=true_tau * 0.1)


class TestSkipRateLimitedPlateau(unittest.TestCase):
    def test_pure_exponential_decay_is_not_treated_as_plateau(self):
        """純粋な一次遅れ応答（レート制限なし）は、tauが大きくても
        プラトーとして誤除外されない（`t[0]`＝除外なしを返す）。"""
        tau = 0.54
        t = np.arange(0, 2.0, DT)
        y = 1.0 - np.exp(-t / tau)
        start = _skip_rate_limited_plateau(t, y)
        self.assertEqual(start, float(t[0]))

    def test_genuine_flat_rate_plateau_is_still_detected(self):
        """実際に変化率が一定（レート制限中）の区間はこれまで通り検出される。"""
        rate = 3.0
        t = np.arange(0, 1.0, DT)
        y = np.minimum(rate * t, 1.0)   # 一定速度で立ち上がり、その後頭打ち
        start = _skip_rate_limited_plateau(t, y)
        self.assertGreater(start, float(t[0]))


def _simulate_accel_phases(*, true_accel: float, true_decel: float,
                           tc_active: bool, rng: np.random.Generator) -> list[Sample]:
    """加速→制動の合成ログ。`tc_active`で両フェーズの`VehicleState.tc_active`を
    一律に固定する（介入あり/なしの2パターンをテストで作り分けるため）。"""
    samples: list[Sample] = []
    t = 0.0
    speed = 0.0
    # 加速フェーズ: 静止から加速し、低速区間ぶんは95パーセンタイルの
    # ノイズ耐性で自然に無視される想定（明示的な除外条件でも二重に効く）
    for _ in range(150):
        speed = min(speed + true_accel * DT, 2.0)
        samples.append(Sample(t=t, target_speed=2.0, target_steer=0.0, brake=False,
                              speed=speed, steer_actual=0.0, steer_cmd_echo=0.0,
                              yaw_rate=0.0, accel_x=true_accel + rng.normal(0, 0.02),
                              tc_active=tc_active))
        t += DT
    # 制動フェーズ: 停止まで減速
    for _ in range(int(speed / (true_decel * DT)) + 5):
        speed = max(speed - true_decel * DT, 0.0)
        samples.append(Sample(t=t, target_speed=0.0, target_steer=0.0, brake=True,
                              speed=speed, steer_actual=0.0, steer_cmd_echo=0.0,
                              yaw_rate=0.0, accel_x=-(true_decel + rng.normal(0, 0.02)),
                              tc_active=tc_active))
        t += DT
    return samples


class TestFitAccel(unittest.TestCase):
    def test_recovers_known_accel_and_decel_from_synthetic_phases(self):
        """加速フェーズ・制動フェーズそれぞれで観測される`|accel_x|`の95
        パーセンタイルから、既知の`drive_accel_m_s2`/`brake_decel_m_s2`を
        復元できる（速度がほぼ0の区間はノイズとして除外されることも確認）。
        両フェーズでTCが介入している（`tc_active=True`）ことが前提。"""
        rng = np.random.default_rng(0)
        true_accel, true_decel = 1.8, 4.2
        samples = _simulate_accel_phases(true_accel=true_accel, true_decel=true_decel,
                                         tc_active=True, rng=rng)

        result = fit_accel(samples)
        self.assertAlmostEqual(result["drive_accel_m_s2"], true_accel, delta=0.15)
        self.assertAlmostEqual(result["brake_decel_m_s2"], true_decel, delta=0.15)

    def test_raises_when_tc_never_engages(self):
        """`VehicleState.tc_active`が全区間Falseなら、タイヤが滑り出す本当の
        上限を測れていない可能性が高いとしてエラーにする
        （`fit_corner`の`_check_corner_saturated`と同じ考え方）。"""
        rng = np.random.default_rng(0)
        samples = _simulate_accel_phases(true_accel=1.8, true_decel=4.2,
                                         tc_active=False, rng=rng)
        with self.assertRaises(ValueError):
            fit_accel(samples)


class TestFitFirstOrderWithDelay(unittest.TestCase):
    def test_recovers_dead_time_and_tau_from_synthetic_step(self):
        """`dead_time_s`は「10%到達点への線形補間」で決めているため、
        50Hzサンプリング下では原理的に最大1サンプル(0.02s)程度の
        システマティックな誤差が乗る——ここでは`tau`の精密な復元
        （こちらはサンプリング間隔に依らない、`_skip_rate_limited_plateau`が
        効かない前提での回帰精度）と、`dead_time`がその誤差の範囲に
        収まることを確認する。"""
        true_dead_time, true_tau = 0.04, 0.2
        t = np.arange(0, 2.0, DT)
        y = np.where(t < true_dead_time, 0.0, 1.0 - np.exp(-(t - true_dead_time) / true_tau))
        dead_time, tau = _fit_first_order_with_delay(t, y)
        self.assertAlmostEqual(dead_time, true_dead_time, delta=1.5 * DT)
        self.assertAlmostEqual(tau, true_tau, delta=true_tau * 0.05)


if __name__ == "__main__":
    unittest.main()
