"""`ml_common/train_utils.py` のテスト（best の選び方・かたまり単位の分割・乱数の固定）。"""

import math
import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.train_utils import BestTracker, group_split, is_val_group, seed_all, time_block  # noqa: E402


class TestBestTracker(unittest.TestCase):
    def test_min_and_max(self):
        lo = BestTracker("min")
        self.assertTrue(lo.update(0.5, "a"))
        self.assertFalse(lo.update(0.5, "b"))           # 同じ値では更新しない
        self.assertTrue(lo.update(0.4, "c"))
        self.assertEqual((lo.best, lo.info), (0.4, "c"))
        hi = BestTracker("max")
        self.assertTrue(hi.update(0.0))                 # IoU 0 でも最初の値は最良（以前の -1.0 始まりと同じ）
        self.assertFalse(hi.update(-0.1))

    def test_nan_never_becomes_best(self):
        t = BestTracker("min")
        self.assertFalse(t.update(math.nan))
        self.assertFalse(t.found)
        self.assertTrue(math.isnan(t.best_or_nan()))
        t.update(1.0)
        self.assertFalse(t.update(math.nan))
        self.assertEqual(t.best_or_nan(), 1.0)

    def test_bad_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            BestTracker("lowest")


class TestGroupSplit(unittest.TestCase):
    @staticmethod
    def _items(source="a", n=3000, hz=10):
        return [(source, int(i / hz * 1e9)) for i in range(n)]

    @staticmethod
    def _key(item):
        return f"{item[0]}:{time_block(item[1], 5.0)}"

    def test_groups_never_straddle_and_order_is_kept(self):
        items = self._items()
        train, val = group_split(items, self._key, val_ratio=0.2, seed=0)
        self.assertFalse({self._key(i) for i in train} & {self._key(i) for i in val})
        self.assertEqual(sorted(train + val), items)
        self.assertEqual(train, sorted(train))
        self.assertTrue(0.05 < len(val) / len(items) < 0.4)

    def test_matches_the_per_group_decision(self):
        """1つずつの判定（学習後に train/val を言い直すのに使う）と一致する。"""
        items = self._items()
        _train, val = group_split(items, self._key, val_ratio=0.3, seed=7)
        self.assertEqual({self._key(i) for i in val},
                         {self._key(i) for i in items if is_val_group(self._key(i), val_ratio=0.3, seed=7)})

    def test_assignment_is_stable_when_data_is_added(self):
        a, b = self._items("a"), self._items("b")
        before = set(group_split(a, self._key, val_ratio=0.2)[1])
        after = {i for i in group_split(a + b, self._key, val_ratio=0.2)[1] if i[0] == "a"}
        self.assertEqual(before, after)

    def test_zero_val_ratio_puts_everything_in_train(self):
        items = [("a", 0), ("a", 9 * 10**9)]
        train, val = group_split(items, self._key, val_ratio=0.0)
        self.assertEqual((len(train), len(val)), (2, 0))


class TestSeedAll(unittest.TestCase):
    def test_all_random_sources_repeat(self):
        import numpy as np
        import torch

        def draw():
            return random.random(), float(np.random.rand()), float(torch.rand(1))

        seed_all(3)
        a = draw()
        seed_all(3)
        self.assertEqual(a, draw())


if __name__ == "__main__":
    unittest.main()
