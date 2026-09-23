"""`backend/loop_detection.py`（ループ検出）のテスト。

ループ検出は**入れてよい拘束より、入れてはいけない拘束を弾けるか**が本体。
実測で地図を壊したのは「当たり率0.99・得点0.92のまま1.4mずれた拘束」1本
だったので、合格条件（向き・補正量・曖昧さ）を個別に縛る。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.backend.loop_detection import LoopDetectorConfig, find_loop_closure  # noqa: E402
from slam2d.core.frontend import Keyframe  # noqa: E402
from slam2d.core.types import Pose2D, ScanPoints  # noqa: E402
from slam2d.tests.helpers import ROOM, ray_segments  # noqa: E402

INFO = np.eye(3) * 100.0


def _scan_body(x, y, yaw, segs=ROOM, n=360, max_range=8.0):
    ang = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    r = np.array([ray_segments(x, y, yaw + a, segs, max_range) for a in ang])
    ok = r > 0.0
    return r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])


def _keyframes(pairs, segs=ROOM):
    """`(真の姿勢, 推定姿勢)`の列から`Keyframe`の列を作る（走行距離も積む）。

    **点群は真の姿勢から作り、`Keyframe.pose`には推定姿勢を入れる**——これが
    「ドリフトした推定で焼いた地図」の作り方。両者を同じにすると、そもそも
    直すべきずれが存在しない状態になってしまう。
    """
    out = []
    path = 0.0
    prev = None
    for (true_pose, est_pose) in pairs:
        if prev is not None:
            path += math.hypot(est_pose[0] - prev[0], est_pose[1] - prev[1])
        prev = (est_pose[0], est_pose[1])
        hx, hy = _scan_body(*true_pose, segs=segs)
        pts = ScanPoints(hx, hy, np.ones(hx.size, dtype=bool), 0, True)
        out.append(Keyframe(Pose2D(*est_pose), pts, hx.astype(np.float32),
                            hy.astype(np.float32), INFO, path, 0))
    return out


def _straight_and_back(drift=(0.0, 0.0, 0.0)):
    """まっすぐ進んで戻り、出発点付近を再訪する経路。

    戻りの区間は**推定だけ**が`drift`ぶんずれている（点群は真の姿勢から作る）。
    """
    out = [((1.0 + 0.1 * i, 2.0, 0.0), (1.0 + 0.1 * i, 2.0, 0.0)) for i in range(40)]
    for i in range(40):
        true_pose = (4.9 - 0.1 * i, 2.0, 0.0)
        est = (true_pose[0] + drift[0], true_pose[1] + drift[1], true_pose[2] + drift[2])
        out.append((true_pose, est))
    return out


class TestFindLoopClosure(unittest.TestCase):
    def test_revisit_is_detected_and_corrects_the_drift(self):
        """20cmずれて戻ってきたら、そのずれを打ち消す拘束が出る。"""
        kfs = _keyframes(_straight_and_back(drift=(0.0, 0.20, 0.0)))
        cfg = LoopDetectorConfig(min_path_gap=4.0)
        cand = find_loop_closure(kfs, len(kfs) - 1, config=cfg)
        self.assertIsNotNone(cand)
        self.assertLess(cand.src, 10)
        # src から見た dst の相対姿勢は、真の位置関係（ほぼ同じ場所）に近いはず
        self.assertAlmostEqual(cand.delta.y, 0.0, delta=0.05)
        self.assertGreater(cand.inlier, 0.8)
        self.assertGreater(cand.correction, 0.1)          # 実際に20cm動かしている

    def test_recent_keyframes_are_not_candidates(self):
        kfs = _keyframes([((1.0 + 0.02 * i, 2.0, 0.0),) * 2 for i in range(30)])
        cand = find_loop_closure(kfs, len(kfs) - 1,
                                 config=LoopDetectorConfig(min_path_gap=4.0))
        self.assertIsNone(cand)

    def test_opposite_heading_is_rejected(self):
        """逆向きで通過しただけの近接は再訪として採らない。"""
        poses = [(1.0 + 0.1 * i, 2.0, 0.0) for i in range(40)]
        poses += [(4.9 - 0.1 * i, 2.05, math.pi) for i in range(40)]
        kfs = _keyframes([(p, p) for p in poses])
        cfg = LoopDetectorConfig(min_path_gap=4.0)
        self.assertIsNone(find_loop_closure(kfs, len(kfs) - 1, config=cfg))

    def test_implausible_correction_is_rejected(self):
        """ドリフトの見積もりに対して動かしすぎる拘束は捨てる。"""
        kfs = _keyframes(_straight_and_back(drift=(0.0, 0.20, 0.0)))
        cfg = LoopDetectorConfig(min_path_gap=4.0, max_correction=0.01,
                                 max_correction_per_m=0.0)
        self.assertIsNone(find_loop_closure(kfs, len(kfs) - 1, config=cfg))

    def test_large_drift_is_found_with_the_wide_search(self):
        """50cm・6°のドリフトでも、粗→細の多段探索で拾える。

        （8m走って50cmのずれ。これ以上大きいと`max_correction`の「ドリフトは
        走行距離に比例する」という見積もりから外れるので、拘束としては採らない
        ——それは`test_implausible_correction_is_rejected`が縛っている）
        """
        kfs = _keyframes(_straight_and_back(drift=(0.0, 0.5, math.radians(6.0))))
        cfg = LoopDetectorConfig(min_path_gap=4.0, radius=1.5)
        cand = find_loop_closure(kfs, len(kfs) - 1, config=cfg)
        self.assertIsNotNone(cand)
        self.assertAlmostEqual(cand.delta.y, 0.0, delta=0.08)
        self.assertAlmostEqual(math.degrees(cand.delta.yaw), 0.0, delta=2.0)

    def test_too_few_points_is_rejected(self):
        kfs = _keyframes(_straight_and_back(drift=(0.0, 0.2, 0.0)))
        cfg = LoopDetectorConfig(min_path_gap=4.0, min_used=10_000)
        self.assertIsNone(find_loop_closure(kfs, len(kfs) - 1, config=cfg))


if __name__ == "__main__":
    unittest.main()
