"""`pipeline.SlamSystem`（フロントエンド＋ループ閉じ）のテスト。

ここで縛るのは**ループ閉じが実際にドリフトを減らすこと**。以前の実装では
「拘束は入るが姿勢が動かない」（ロバストカーネルの閾値が情報行列と釣り合って
おらず、正しい拘束まで無効化されていた）という壊れ方をしていて、テストが
「例外が出ないこと」しか見ていなかったので気づけなかった。

周回コース（oval）を、ジャイロにゼロ点ずれを乗せた推測航法で2周する。
ループ閉じ前後で**キーフレーム姿勢の真値からのずれ**を比べる。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.backend.loop_detection import LoopDetectorConfig  # noqa: E402
from slam2d.core.frontend import FrontendConfig  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.motion import ExternalTwistModel, OdometryNoise  # noqa: E402
from slam2d.core.types import Twist2D, wrap_angle  # noqa: E402
from slam2d.pipeline import PipelineConfig, SlamSystem  # noqa: E402
from slam2d.tests.helpers import (  # noqa: E402
    make_oval_track, make_raw_scan, oval_centerline_length, oval_centerline_pose,
    oval_centerline_twist,
)

DT = 0.1
SPEED = 0.6
NS = 1_000_000_000


def _drive(system, *, laps=2.0, gyro_bias_dps=0.0, noise_sigma=0.0, seed=0, twist_holder=None):
    """ovalの中心線を`laps`周ぶん走らせ、(真の姿勢, 推定姿勢) の列を返す。"""
    track = make_oval_track()
    total = oval_centerline_length()
    rng = np.random.default_rng(seed)
    bias = math.radians(gyro_bias_dps)
    out = []
    s = 0.0
    t_ns = NS
    n = int(total * laps / (SPEED * DT))
    for _ in range(n):
        s += SPEED * DT
        t_ns += int(DT * NS)
        x, y, yaw = oval_centerline_pose(s)
        v, w = oval_centerline_twist(s, SPEED)
        if twist_holder is not None:
            twist_holder["twist"] = Twist2D(v, 0.0, w + bias)
        raw = make_raw_scan(x, y, yaw, segs=track, max_range=8.0, noise_sigma=noise_sigma,
                            rng=rng, t_point_ns=t_ns)
        system.update(raw, DT)
        out.append(((x, y, yaw), system.frontend.pose))
    return out


def _make_system(**pipe_kwargs):
    grid = OccGrid(resolution=0.025, size_m=16.0, origin=(-8.0, -8.0))
    holder = {"twist": Twist2D(0.0, 0.0, 0.0)}
    motion = ExternalTwistModel(lambda: holder["twist"],
                                noise=OdometryNoise())
    cfg = PipelineConfig(frontend=FrontendConfig(max_range=8.0, kf_dist=0.1),
                         loop_detector=LoopDetectorConfig(min_path_gap=6.0),
                         **pipe_kwargs)
    return SlamSystem(grid, motion, cfg), holder


def _kf_errors(system, truth_at):
    """キーフレームごとの真値からのずれ[m]。最初のキーフレームを基準に合わせる。"""
    errs = []
    for kf, truth in zip(system.frontend.keyframes, truth_at):
        errs.append(math.hypot(kf.pose.x - truth[0], kf.pose.y - truth[1]))
    return np.asarray(errs)


class TestLoopClosureReducesDrift(unittest.TestCase):
    def test_gyro_bias_drift_is_corrected(self):
        system, holder = _make_system()
        log = _drive(system, laps=2.0, gyro_bias_dps=1.0, noise_sigma=0.01,
                     twist_holder=holder)

        # SLAM の原点は最初の姿勢なので、真値をそこ基準へ移す
        ox, oy, oyaw = log[0][0]
        c, s = math.cos(-oyaw), math.sin(-oyaw)

        def rel(p):
            dx, dy = p[0] - ox, p[1] - oy
            return (dx * c - dy * s, dx * s + dy * c, wrap_angle(p[2] - oyaw))

        # キーフレームの真値は「そのキーフレームが作られた時刻の真の姿勢」
        # ——推定と真値の対応が要るので、走行ログから最も近い時刻のものを拾う
        est = [p for _t, p in log]
        truth = [rel(t) for t, _p in log]
        idx = []
        j = 0
        for kf in system.frontend.keyframes:
            while j + 1 < len(est) and not (abs(est[j].x - kf.pose.x) < 1e-9
                                            and abs(est[j].y - kf.pose.y) < 1e-9):
                j += 1
            idx.append(min(j, len(truth) - 1))
        truth_at = [truth[i] for i in idx]

        before = _kf_errors(system, truth_at)
        self.assertGreater(len(system.loops), 0, "ループ拘束が1本も入っていない")
        system.flush()
        after = _kf_errors(system, truth_at)

        self.assertGreater(float(before.mean()), 0.05, "そもそもドリフトしていない")
        self.assertLess(float(after.mean()), 0.5 * float(before.mean()),
                        f"ループ閉じでドリフトが減っていない: {before.mean():.3f} -> {after.mean():.3f}")
        self.assertGreater(system.optimizations, 0)

    def test_map_is_rebuilt_on_optimize(self):
        system, holder = _make_system()
        _drive(system, laps=1.6, gyro_bias_dps=1.0, twist_holder=holder)
        grid_before = system.frontend.grid
        self.assertGreater(len(system.loops), 0)
        system.flush()
        self.assertIsNot(system.frontend.grid, grid_before)
        self.assertGreater(system.frontend.grid.seq, grid_before.seq)
        self.assertGreater(int(system.frontend.grid.wall_mask().sum()), 500)

    def test_no_loops_no_optimization(self):
        """ループが見つからない短い走行では、最適化も焼き直しも起きない。"""
        system, holder = _make_system()
        _drive(system, laps=0.15, twist_holder=holder)
        grid_before = system.frontend.grid
        system.flush()
        self.assertEqual(system.loop_closures, 0)
        self.assertEqual(system.optimizations, 0)
        self.assertIs(system.frontend.grid, grid_before)


class TestGraphBookkeeping(unittest.TestCase):
    def test_nodes_match_keyframes(self):
        system, holder = _make_system()
        _drive(system, laps=0.3, twist_holder=holder)
        self.assertEqual(system._graph.size, len(system.frontend.keyframes))
        self.assertGreater(len(system.frontend.keyframes), 5)


if __name__ == "__main__":
    unittest.main()
