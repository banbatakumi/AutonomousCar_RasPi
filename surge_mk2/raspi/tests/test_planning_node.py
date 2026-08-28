"""`raspi/nodes/planning_node.py` の入力ルーティングのテスト。

**LiDAR 系 planner は無改造で動くこと**、そして**カメラ系 planner に
切り替えた瞬間に `scan/cam` だけを見るようになること**の2つを確認する。
バス・実プロセスは要らない——`Subscriber`/`Publisher` の代わりに
`latest` 辞書と `send()` の記録だけを持つ身代わりを使う
（`raspi/tests/test_auto.py` の `FakeSub` と同じ流儀）。
"""

import sys
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto import E2ELidar  # noqa: E402
from raspi.msgs import AutoCtrl, Scan  # noqa: E402
from raspi.msgs.types import TOPIC_E2E_MODEL, TOPIC_SCAN, TOPIC_SCAN_CAM, E2EModelCtrl  # noqa: E402
from raspi.nodes.planning_node import PlanningNode, input_topics  # noqa: E402


class FakeSub:
    """`Subscriber` の身代わり。`latest` だけを持つ（`test_auto.py` と同じ）。"""

    def __init__(self, latest=None):
        self.latest = latest or {}


class FakePub:
    """`Publisher` の身代わり。送った内容を記録するだけ。"""

    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


def _scan() -> Scan:
    """全周 3m の開けた点群。ルーティングのテストなのでギャップ選択の中身は問わない。"""
    return Scan(dist=[3.0] * 360, sector_seen=[True] * 12, seq=1)


class TestInputTopics(unittest.TestCase):
    def test_includes_lidar_and_camera_topics(self):
        """`input_topics()` は登録済み全 planner の `input_topic` の和集合。"""
        topics = input_topics()
        self.assertIn(TOPIC_SCAN, topics)
        self.assertIn(TOPIC_SCAN_CAM, topics)


class TestReplanRouting(unittest.TestCase):
    """`_replan()` が `planner.input_topic` からしか読まないこと。"""

    def test_lidar_planner_reads_scan_topic(self):
        sub = FakeSub({TOPIC_SCAN: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg")
        node._replan(1)
        self.assertTrue(node.state.ready, node.state.reason)

    def test_lidar_planner_ignores_camera_topic(self):
        """`scan/cam` にしかデータが無ければ、LiDAR 版 planner は動かない
        （＝別センサのデータを誤って拾わないこと）。"""
        sub = FakeSub({TOPIC_SCAN_CAM: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg")
        node._replan(1)
        self.assertEqual(node._last_scan_seq, -1)

    def test_camera_planner_reads_scan_cam_topic(self):
        sub = FakeSub({TOPIC_SCAN_CAM: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg_cam")
        node._replan(1)
        self.assertTrue(node.state.ready, node.state.reason)

    def test_camera_planner_ignores_scan_topic(self):
        """実 LiDAR の `scan` にしかデータが無ければ、カメラ版 planner は動かない。"""
        sub = FakeSub({TOPIC_SCAN: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg_cam")
        node._replan(1)
        self.assertEqual(node._last_scan_seq, -1)


class TestCurrentStaleness(unittest.TestCase):
    """`_current()` の鮮度判定が `planner.stale_ms` を見ること。"""

    def test_camera_planner_uses_its_own_stale_ms(self):
        """`ftg_cam` の `stale_ms`(500) は LiDAR 版の 300ms とは別に効く。"""
        sub = FakeSub({TOPIC_SCAN_CAM: _scan()})   # t_pub は既定の 0
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg_cam")

        fresh = node._current(400 * 1_000_000)     # LiDAR 基準なら古いが 500ms 未満
        self.assertNotIn("古い", fresh.reason)

        stale = node._current(600 * 1_000_000)     # 500ms を超えた
        self.assertIn("古い", stale.reason)

    def test_lidar_planner_still_uses_300ms(self):
        """既存 planner の挙動が変わっていないことの回帰確認。"""
        sub = FakeSub({TOPIC_SCAN: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg")

        fresh = node._current(250 * 1_000_000)
        self.assertNotIn("古い", fresh.reason)

        stale = node._current(350 * 1_000_000)
        self.assertIn("古い", stale.reason)


class _FakeReloadablePlanner:
    """`reload_if_changed`を持つplannerの身代わり。呼ばれた名前を記録するだけ。"""

    def __init__(self) -> None:
        self.calls: list[str] = []

    def reload_if_changed(self, name: str) -> None:
        self.calls.append(name)


class TestE2EModelRouting(unittest.TestCase):
    """`e2e/model`（GUIが選んだモデル名）が対応する planner にだけ届くこと。"""

    def test_dispatches_to_planner_with_reload_if_changed(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeReloadablePlanner()
        node.planner = fake
        node._apply_e2e_model(E2EModelCtrl(name="alpha"))
        self.assertEqual(fake.calls, ["alpha"])

    def test_noop_for_planner_without_reload_if_changed(self):
        """`reload_if_changed`を持たない普通のplanner（例: `ftg`）は無視される
        （例外にならないことだけを確認する）。"""
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        node._apply_e2e_model(E2EModelCtrl(name="alpha"))

    def test_switching_to_e2e_lidar_applies_cached_model_immediately(self):
        """モード切替直後に、既に届いている`e2e/model`を次のポンプを待たず反映する。"""
        sub = FakeSub({TOPIC_E2E_MODEL: E2EModelCtrl(name="alpha")})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg")
        with mock.patch.object(E2ELidar, "reload_if_changed") as m:
            node._apply_ctrl(AutoCtrl(mode="e2e_lidar", engaged=False))
        m.assert_called_once_with("alpha")


if __name__ == "__main__":
    unittest.main()
