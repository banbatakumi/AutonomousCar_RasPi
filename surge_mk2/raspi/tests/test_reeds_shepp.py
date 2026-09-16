"""`raspi/nav/reeds_shepp.py`の単体テスト。

**公式の正しさは暗記ではなく検証で担保する**——生成した経路を`integrate_path()`
で実際に積分し、始点から終点に到達するかを確認する。これがこのテストファイル
の主目的（幾何学的な到達性チェック）で、それ以外の性質は副次的に見る。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.nav.reeds_shepp import (  # noqa: E402
    _GENERATORS, candidate_paths, integrate_path, sample_path, sample_path_array,
    shortest_path,
)

R = 0.4  # この車体の最小旋回半径に近い値


def _reaches(start, goal, turning_radius=R, tol=1e-3):
    path = shortest_path(start, goal, turning_radius)
    assert path.segments, f"経路が生成されなかった: start={start} goal={goal}"
    end = integrate_path(start, path)
    dx = end[0] - goal[0]
    dy = end[1] - goal[1]
    dyaw = (end[2] - goal[2] + math.pi) % (2 * math.pi) - math.pi
    ok = math.hypot(dx, dy) < tol and abs(dyaw) < tol * 10
    return ok, path, end


class TestReachability(unittest.TestCase):
    """様々な配置で、生成された経路が実際に目標へ到達することを確認する。"""

    def _check(self, goal, start=(0.0, 0.0, 0.0)):
        ok, path, end = _reaches(start, goal)
        self.assertTrue(ok, f"start={start} goal={goal} 到達せず: end={end} "
                            f"path={[(s.gear, round(s.curvature, 2), round(s.length, 3)) for s in path.segments]}")

    def test_straight_ahead(self):
        self._check((1.0, 0.0, 0.0))

    def test_straight_behind(self):
        self._check((-1.0, 0.0, 0.0))

    def test_quarter_turn_forward(self):
        self._check((1.0, 1.0, math.radians(90)))

    def test_quarter_turn_forward_other_side(self):
        self._check((1.0, -1.0, math.radians(-90)))

    def test_half_turn(self):
        self._check((0.0, 1.0, math.radians(180)))

    def test_reverse_into_spot(self):
        self._check((-1.0, 0.5, math.radians(180)))

    def test_lateral_offset_same_heading(self):
        """向きは同じで真横にオフセット——Aicardi単一フィードバックでは
        位置が収束しなかった配置（PROGRESS.md参照）。CSC系(S字)で解けるはず。"""
        self._check((0.0, 0.5, 0.0))

    def test_small_lateral_offset(self):
        self._check((0.0, 0.15, 0.0))

    def test_target_behind_and_rotated(self):
        self._check((-0.8, 0.3, math.radians(60)))

    def test_arbitrary_non_origin_start(self):
        self._check((2.0, 1.0, math.radians(45)), start=(1.0, 0.3, math.radians(-30)))

    def test_same_pose_gives_empty_or_trivial_path(self):
        path = shortest_path((0, 0, 0), (0, 0, 0), R)
        self.assertLessEqual(path.length, 1e-6)


class TestManyRandomPoses(unittest.TestCase):
    """ランダムな配置を多数試し、到達失敗の割合を見る。"""

    def test_random_batch_all_reachable(self):
        import random
        rng = random.Random(42)
        failures = []
        n = 200
        for _ in range(n):
            gx = rng.uniform(-2, 2)
            gy = rng.uniform(-2, 2)
            gphi = rng.uniform(-math.pi, math.pi)
            ok, path, end = _reaches((0, 0, 0), (gx, gy, gphi), tol=5e-3)
            if not ok:
                failures.append((gx, gy, gphi))
        rate = 1 - len(failures) / n
        print(f"到達率: {rate:.1%} ({n - len(failures)}/{n})")
        self.assertEqual(failures, [], f"失敗例: {failures[:5]}")


class TestEveryCandidateReachesGoal(unittest.TestCase):
    """**全候補が目標へ到達すること。**

    障害物回避は「最短が塞がれていたら次の候補」という選び方をするので、
    最短だけが正しくても意味がない。9つの語族×4つの対称変換のどこかに
    符号の誤りがあれば、ここで落ちる（`reeds_shepp.py`の検証方針の中核）。
    """

    def test_all_candidates(self):
        import random
        rng = random.Random(7)
        bad = []
        total = 0
        for _ in range(150):
            goal = (rng.uniform(-2, 2), rng.uniform(-2, 2),
                    rng.uniform(-math.pi, math.pi))
            for c in candidate_paths((0, 0, 0), goal, R):
                total += 1
                end = integrate_path((0, 0, 0), c)
                dyaw = (end[2] - goal[2] + math.pi) % (2 * math.pi) - math.pi
                if math.hypot(end[0] - goal[0], end[1] - goal[1]) > 5e-3 or abs(dyaw) > 2e-2:
                    bad.append((goal, [(s.gear, round(s.curvature, 2), round(s.length, 3))
                                       for s in c.segments]))
        print(f"検証した候補数: {total}")
        self.assertGreater(total, 1000, "候補が少なすぎる（語族が生成されていない）")
        self.assertEqual(bad[:3], [], f"到達しない候補 {len(bad)}/{total} 件")


class TestWordFamilyCoverage(unittest.TestCase):
    """**どの語族も「1件も生成されない」状態になっていないこと。**

    ★到達性の検証では 0 件を検出できない（生成されなければ検証対象にならない）。
    実際、CCCC/CCSC系を実装した当初、条件`v<=0`の量に`[0,2π)`の畳み込みを
    使ったせいで3つの語族が全滅していたが、到達率は100%のままだった。
    """

    def test_every_generator_produces_candidates(self):
        import random
        rng = random.Random(11)
        counts = {g.__name__: 0 for g in _GENERATORS}
        for _ in range(400):
            x, y = rng.uniform(-4, 4), rng.uniform(-4, 4)
            phi = rng.uniform(-math.pi, math.pi)
            for gen in _GENERATORS:
                for args in ((x, y, phi), (-x, y, -phi), (x, -y, -phi), (-x, -y, phi)):
                    if gen(*args) is not None:
                        counts[gen.__name__] += 1
        print("語族ごとの生成数:", counts)
        empty = [k for k, v in counts.items() if v == 0]
        self.assertEqual(empty, [], f"1件も生成されない語族: {empty}")

    def test_candidate_count_is_richer_than_the_old_six_words(self):
        """旧実装（CSC+CCCの6語）より候補が増えていること。"""
        import random
        rng = random.Random(3)
        uniq = []
        for _ in range(100):
            goal = (rng.uniform(-2, 2), rng.uniform(-2, 2),
                    rng.uniform(-math.pi, math.pi))
            cs = candidate_paths((0, 0, 0), goal, R)
            uniq.append(len({round(c.length, 4) for c in cs}))
        mean = sum(uniq) / len(uniq)
        print(f"ユニーク候補数の平均: {mean:.1f}")
        self.assertGreater(mean, 9.0, "旧実装（平均6.5本）から増えていない")


class TestCandidatePaths(unittest.TestCase):
    def test_sorted_by_length_and_shortest_matches_shortest_path(self):
        start, goal = (0.0, 0.0, 0.0), (1.0, 0.5, math.radians(60))
        candidates = candidate_paths(start, goal, R)
        self.assertTrue(candidates)
        lengths = [c.length for c in candidates]
        self.assertEqual(lengths, sorted(lengths))
        best = shortest_path(start, goal, R)
        self.assertAlmostEqual(candidates[0].length, best.length, places=6)

    def test_multiple_candidates_usually_exist(self):
        candidates = candidate_paths((0, 0, 0), (0.0, 0.5, 0.0), R)
        self.assertGreater(len(candidates), 1)


class TestSamplePath(unittest.TestCase):
    """`sample_path_array()`（numpy版）と`sample_path()`が一致すること。

    衝突判定はこのモジュールではなく`local_map.LocalMap.path_clearance()`が
    担うので、ここで縛るのはサンプリングの正しさだけ。
    """

    def test_array_and_list_agree(self):
        import numpy as np
        path = shortest_path((0, 0, 0), (1.0, 0.5, math.radians(60)), R)
        a = sample_path_array((0, 0, 0), path, step=0.05)
        b = np.asarray(sample_path((0, 0, 0), path, step=0.05))
        self.assertEqual(a.shape, b.shape)
        self.assertLess(float(np.abs(a - b).max()), 1e-12)

    def test_first_sample_is_the_start(self):
        path = shortest_path((0, 0, 0), (1.0, 0.0, 0.0), R)
        first = sample_path((0.3, -0.2, 0.5), path)[0]
        self.assertEqual(first, (0.3, -0.2, 0.5))


if __name__ == "__main__":
    unittest.main()
