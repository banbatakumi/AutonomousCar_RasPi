"""`sim/vehicle.py`の操舵むだ時間キュー（A2: サブステップ化での量子化バグ修正）。

`VehicleModel._pop_delayed`は「1step=1エントリ」方式で、`dead_time_s`ぶん
寝かせてから指令を取り出す。1step=100ms（`sim/gym_env.py`の`_STEP_NS`）を
1回で積分すると、`dead_time_s`（ドメインランダム化で0.015〜0.095s、常に
100ms未満）の値によらず実効遅延は常にちょうど100msに量子化されてしまう
（`dead_time_s=0`のときだけ例外的に即時反映）。`sim/gym_env.py`の`step()`を
`_DYNAMICS_SUBSTEP_S`(0.01s)刻みのサブステップに分割することで、
10ms単位の精度に改善した——ここではそのサブステップ分割を`VehicleModel`
だけを使って再現し、実効遅延が指定した`dead_time_s`に近づくことを確認する。
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
    """`_pop_delayed`の構造（whileチェックがfor減算より前に行われるため、
    エントリが追加されたstep自身はpop対象にならない）により、反応時間には
    厳密に`dead_time_s`ちょうどではなく `+1 dt_sub` の系統的なオフセットが
    乗る（`dead_time_s=0`の特殊ケースのみ、appendした直後のwhileでpopされ
    即時反映される）。この関数は「今の実装が実際に持つ量子化特性」を表す
    参照実装であり、テストはこれとの一致を見る。"""
    if dead_time_s <= 0.0:
        return dt_sub
    return (math.ceil(dead_time_s / dt_sub) + 1) * dt_sub


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
