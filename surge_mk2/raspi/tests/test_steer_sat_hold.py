"""「舵が上限の直後」（`slam2d_raceline.steer_sat_hold`）: 舵が上限の間にできた横偏差が戻るまで、学習を止めたままにする。

実機 `surge_20261031_053025` の低μのヘアピンで、舵が上限の間に外へ 17cm ずれ、上限を外れた直後の横偏差を
安全装置 2（線から外れた）が前兆と取り、2 周で横の上限が 2.73 → 2.46 に下がった。
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto.slam2d_raceline import steer_sat_hold  # noqa: E402

NS = 10 ** 8   # 計画の周期 0.1s [ns]


def _run(seq: list[tuple[bool, float]]) -> list[bool]:
    """(舵が上限か, 横偏差) の 10Hz の列 → 各周期で「舵が上限の直後」で止めているか。"""
    ns, cx, out = 0, 0.0, []
    for k, (sat, c) in enumerate(seq):
        ns, cx = steer_sat_hold(ns, cx, sat, c, (k + 1) * NS)
        out.append(bool(ns) and not sat)
    return out


class SteerSatHoldTest(unittest.TestCase):
    def test_real_hairpin_exit(self) -> None:
        """053025 の 24.2s〜: 上限の間に 0.17m ずれ、外れた後ゆっくり戻る。0.12m を割るまでは必ず止め、0.06m を割ったら解く。"""
        sat = [(True, c) for c in (0.03, 0.05, 0.10, 0.14, 0.17)]
        after = [0.17, 0.17, 0.15, 0.15, 0.12, 0.12, 0.10, 0.09, 0.08, 0.07, 0.06, 0.05, 0.04, 0.03]
        held = _run(sat + [(False, c) for c in after])[len(sat):]
        for c, h in zip(after, held):
            if c >= 0.12:
                self.assertTrue(h, c)
        self.assertFalse(held[after.index(0.05)])
        self.assertFalse(any(held[after.index(0.05):]))

    def test_growing_deviation_releases(self) -> None:
        """出口でさらに膨らむ（本当に滑っている）なら、張り付いていた間の最大 + 4cm を超えた時点で解く。"""
        seq = [(True, 0.08)] * 3 + [(False, 0.09), (False, 0.11), (False, 0.13), (False, 0.15)]
        held = _run(seq)[3:]
        self.assertEqual(held, [True, True, False, False])

    def test_timeout(self) -> None:
        """戻らなくても 1.5s で解く。"""
        held = _run([(True, 0.10)] + [(False, 0.10)] * 20)[1:]
        self.assertTrue(all(held[:15]))
        self.assertFalse(any(held[16:]))

    def test_no_saturation_no_hold(self) -> None:
        """舵が上限にならなければ何もしない（大きな横偏差でも止めない）。"""
        self.assertFalse(any(_run([(False, c) for c in (0.05, 0.15, 0.20, 0.10)])))

    def test_new_saturation_resets_max(self) -> None:
        """解けた後の新しい張り付きは、前の最大を持ち越さない。"""
        seq = [(True, 0.20), (False, 0.03), (True, 0.05), (False, 0.08), (False, 0.10)]
        held = _run(seq)
        self.assertEqual(held[3:], [True, False])   # 0.10 > 0.05 + 0.04 で解く


if __name__ == "__main__":
    unittest.main()
