"""`pipeline.SlamSystem`（frontend+backend統合）の基本動作テスト。

oval周回でのループ閉じ効果の検証（Phase 5の主目的）は`test_pipeline_oval.py`。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from slam2d.backend.loop_detection import LoopDetectorConfig  # noqa: E402
from slam2d.core.frontend import FrontendConfig  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.motion import ExternalTwistModel  # noqa: E402
from slam2d.core.types import Twist2D  # noqa: E402
from slam2d.pipeline import PipelineConfig, SlamSystem  # noqa: E402
from slam2d.tests.helpers import ROOM, make_out_and_back_path, make_raw_scan  # noqa: E402


class TestSlamSystemBasic(unittest.TestCase):
    def test_runs_without_loop_closure(self):
        """ループが起きない短い走行でも、grid/keyframesが素直に育つ。"""
        grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        sys_ = SlamSystem(grid, motion, PipelineConfig(
            frontend=FrontendConfig(kf_dist=0.02)))
        for i in range(15):
            raw = make_raw_scan(3.0 + 0.01 * i, 2.0, 0.0, segs=ROOM, max_range=8.0)
            sys_.update(raw, 0.1)
        self.assertEqual(sys_.loop_closures, 0)
        self.assertGreater(len(sys_.frontend.keyframes), 1)

    def test_first_node_is_fixed_and_graph_grows(self):
        grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        sys_ = SlamSystem(grid, motion, PipelineConfig(
            frontend=FrontendConfig(kf_dist=0.02)))
        for i in range(10):
            raw = make_raw_scan(3.0 + 0.01 * i, 2.0, 0.0, segs=ROOM, max_range=8.0)
            sys_.update(raw, 0.1)
        self.assertEqual(sys_._graph.size, len(sys_.frontend.keyframes))


class TestSlamSystemLoopClosureTriggersRebuild(unittest.TestCase):
    def test_revisit_triggers_loop_closure_and_rebuild(self):
        """既知の場所へ戻ると、ループが検出されグラフ最適化・地図の焼き直しが起きる。

        前進→Uターン→前進、で出発点付近に戻る経路（`make_out_and_back_path`）
        を使う。真の軌道と`ExternalTwistModel`に渡すtwistを一致させることで、
        `Frontend`が実際に意図通りの経路を推定できるようにしてある。
        """
        grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        holder = {"twist": Twist2D(0.0, 0.0, 0.0)}
        motion = ExternalTwistModel(lambda: holder["twist"])
        cfg = PipelineConfig(
            frontend=FrontendConfig(kf_dist=0.02),
            loop_detector=LoopDetectorConfig(min_index_gap=10, radius=0.3),
            # 単発の往復経路では再訪が1回しか起きないため、バッチが溜まるのを
            # 待たず即座に反映する（既定の3だとこの経路では一生バッチが
            # 埋まらない）。バッチ処理自体の効果は`test_pipeline_oval.py`で
            # 別途検証している
            loop_batch_size=1,
        )
        sys_ = SlamSystem(grid, motion, cfg)

        seq_before = sys_.frontend.grid.seq
        for (x, y, yaw), (vx, vy, wz) in make_out_and_back_path(v=0.15, out_steps=20, turn_steps=16):
            holder["twist"] = Twist2D(vx, vy, wz)
            raw = make_raw_scan(x, y, yaw, segs=ROOM, max_range=8.0)
            sys_.update(raw, 0.1)

        self.assertGreater(sys_.loop_closures, 0)
        # 地図が焼き直された（新しいOccGridインスタンスに置き換わり、seqが進んだ）
        self.assertGreater(sys_.frontend.grid.seq, seq_before)
        self.assertGreater(len(sys_.frontend.keyframes), 0)


class TestSlamSystemBatching(unittest.TestCase):
    def test_pending_candidate_is_not_applied_until_batch_size_reached(self):
        """`loop_batch_size`未満の候補は保留され、グラフ・地図に反映されない。"""
        grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        holder = {"twist": Twist2D(0.0, 0.0, 0.0)}
        motion = ExternalTwistModel(lambda: holder["twist"])
        cfg = PipelineConfig(
            frontend=FrontendConfig(kf_dist=0.02),
            loop_detector=LoopDetectorConfig(min_index_gap=10, radius=0.3),
            loop_batch_size=3,     # 1回の再訪だけでは満たされない
        )
        sys_ = SlamSystem(grid, motion, cfg)
        grid_before = sys_.frontend.grid    # 焼き直しでインスタンス自体が置き換わる
        for (x, y, yaw), (vx, vy, wz) in make_out_and_back_path(v=0.15, out_steps=20, turn_steps=16):
            holder["twist"] = Twist2D(vx, vy, wz)
            raw = make_raw_scan(x, y, yaw, segs=ROOM, max_range=8.0)
            sys_.update(raw, 0.1)

        self.assertEqual(sys_.loop_closures, 0)          # まだ反映されていない
        self.assertGreater(len(sys_._pending), 0)          # 候補は保留中
        self.assertIs(sys_.frontend.grid, grid_before)      # 地図も未変更(差し替わっていない)

        sys_.flush()
        self.assertGreater(sys_.loop_closures, 0)
        self.assertEqual(len(sys_._pending), 0)
        self.assertIsNot(sys_.frontend.grid, grid_before)

    def test_flush_with_nothing_pending_is_a_noop(self):
        grid = OccGrid(resolution=0.05, size_m=12.0, origin=(-1.0, -1.0))
        motion = ExternalTwistModel(lambda: Twist2D(0.1, 0.0, 0.0))
        sys_ = SlamSystem(grid, motion, PipelineConfig(frontend=FrontendConfig(kf_dist=0.02)))
        for i in range(5):
            raw = make_raw_scan(3.0 + 0.01 * i, 2.0, 0.0, segs=ROOM, max_range=8.0)
            sys_.update(raw, 0.1)
        grid_before = sys_.frontend.grid
        sys_.flush()
        self.assertIs(sys_.frontend.grid, grid_before)


if __name__ == "__main__":
    unittest.main()
