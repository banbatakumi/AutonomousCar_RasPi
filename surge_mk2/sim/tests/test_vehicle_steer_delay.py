"""`sim/vehicle.py`の操舵むだ時間（A2: サブステップ化での量子化バグ修正）。

A2（2026-08）: 1step=100msを1回で積分すると、`dead_time_s`の値によらず実効遅延が
100msに量子化されていた。`step()`をサブステップに分割して10ms粒度にした。

2026-09-24: むだ時間を`SteerServo`の「入力が始まった時刻」の履歴引き（ステップ中点
で遡る）に置き換えた。以前の「1step=1エントリ」のキューは常に遅れ側へ
`ceil(d/dt)+1`stepに量子化していたが、今は四捨五入相当。ここでは`VehicleModel`だけを
使ってサブステップ幅ごとの反応時間を確かめる。
"""

from __future__ import annotations

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from sim.vehicle import DriveInput, VehicleModel, VehicleSpec  # noqa: E402


def _time_to_react(spec: "VehicleSpec", *, dt_sub: float, total_s: float = 0.1) -> float | None:
    """`target_steer`を与えてから`steer_actual`が動き始めるまでの時間 [s]。

    `tau_steer_s=0`にして一次遅れフィルタを無効化し、むだ時間キューだけの
    挙動を切り出して見る（`step()`は`tau<=1e-3`のとき`steer_actual = want`を
    即座に代入するため、`want`＝むだ時間後の値が変わった瞬間がそのまま見える）。
    """
    v = VehicleModel(replace(spec, tau_steer_s=0.0), (0.0, 0.0, 0.0))
    v.apply(DriveInput(armed=True, target_speed=0.0, target_steer=0.5))
    n_sub = round(total_s / dt_sub)
    for i in range(n_sub):
        v.step(dt_sub)
        if v.steer_actual != 0.0:
            return (i + 1) * dt_sub
    return None


def _expected_reaction_time(dead_time_s: float, dt_sub: float) -> float:
    """`SteerServo`（2026-09-24〜）の量子化特性の参照実装。

    むだ時間は「ステップ中点 `t + dt/2 - dead`」時点で有効だった入力を引く。
    指令を与えた直後のステップを k=0 として、`k*dt + dt/2 - dead >= 0` を満たす
    最初の k のステップで動き始め、その終わり `(k+1)*dt` に `steer_actual` が変わる。
    以前の「1step=1エントリ」方式は `(ceil(dead/dt)+1)*dt` と常に遅れ側に
    量子化していた（期待値で `dt` 近く余計に遅れる）が、今は四捨五入に相当する。"""
    k = max(0, math.ceil((dead_time_s - 0.5 * dt_sub) / dt_sub - 1e-9))
    return (k + 1) * dt_sub


class TestSteerDelaySubstepResolution(unittest.TestCase):
    def test_zero_dead_time_reacts_immediately(self):
        spec = VehicleSpec(dead_time_s=0.0)
        t = _time_to_react(spec, dt_sub=0.01)
        self.assertAlmostEqual(t, _expected_reaction_time(0.0, 0.01), delta=1e-9)

    def test_dead_time_within_a_single_100ms_step_is_resolved_at_10ms_granularity(self):
        """★ A2の本体: 修正前は1step=100msを1回で積分していたため、
        `dead_time_s`（0.015〜0.095sの範囲）の値によらず実効遅延は常に
        100ms（`dead_time_s=0`以外は必ず2step目まで持ち越されるため実質
        100〜200ms相当）に量子化されていた。10ms刻みのサブステップ化により、
        `dead_time_s`の値ごとに異なる反応時間になる——量子化が10ms粒度まで
        改善されたことを確認する。"""
        spec = VehicleSpec(dead_time_s=0.03)
        t = _time_to_react(spec, dt_sub=0.01)
        self.assertAlmostEqual(t, _expected_reaction_time(0.03, 0.01), delta=1e-9)
        # 修正前の挙動（常に100ms以上＝1step全体）ではなく、100ms未満で反応する
        self.assertLess(t, 0.1)

    def test_different_dead_times_now_react_at_different_times(self):
        """★ 量子化されていた証拠: 修正前は`dead_time_s`が0.02でも0.08でも
        同じ反応時間になっていたはず。修正後は値に応じて反応時間が変わる
        （単調増加する）ことを確認する。"""
        spec_short = VehicleSpec(dead_time_s=0.02)
        spec_long = VehicleSpec(dead_time_s=0.08)
        t_short = _time_to_react(spec_short, dt_sub=0.01)
        t_long = _time_to_react(spec_long, dt_sub=0.01)
        self.assertLess(t_short, t_long)

    def test_dead_time_near_the_upper_end_of_the_randomization_range(self):
        """`dead_time_s_range`の上限付近（0.08s+パイプライン遅延≈0.095s）。"""
        spec = VehicleSpec(dead_time_s=0.09)
        t = _time_to_react(spec, dt_sub=0.01)
        self.assertAlmostEqual(t, _expected_reaction_time(0.09, 0.01), delta=1e-9)

    def test_coarser_substep_changes_the_granularity_accordingly(self):
        """サブステップ幅を変えると、量子化誤差もそのサブステップ幅に応じて
        変わること（`_DYNAMICS_SUBSTEP_S`の値そのものへの依存を確認する
        回帰テスト）。"""
        spec = VehicleSpec(dead_time_s=0.045)
        t = _time_to_react(spec, dt_sub=0.02)
        self.assertAlmostEqual(t, _expected_reaction_time(0.045, 0.02), delta=1e-9)


if __name__ == "__main__":
    unittest.main()
