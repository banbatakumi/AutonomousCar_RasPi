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

from raspi.auto import E2ELidar, make_planner  # noqa: E402
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


class _FakeCrashingPlanner:
    """`plan()` が常に例外を投げる身代わり（B3）。"""

    name = "crashing"
    input_topic = TOPIC_SCAN
    stale_ms = 300

    def plan(self, scan, vs, params, dt):
        raise RuntimeError("planner内部のバグ")


class TestReplanSurvivesPlannerException(unittest.TestCase):
    """`planner.plan()` が例外を投げてもノードが継続すること（B3）。"""

    def test_exception_does_not_propagate_and_state_stays_not_ready(self):
        sub = FakeSub({TOPIC_SCAN: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg")
        node.planner = _FakeCrashingPlanner()

        node._replan(1)  # 例外を外に伝播させないこと

        self.assertFalse(node.state.ready)
        self.assertIn("例外", node.state.reason)

    def test_node_keeps_working_after_a_crashing_period(self):
        """1周期壊れても、次の周期で正常なplannerに戻せば普通に動くこと。"""
        sub = FakeSub({TOPIC_SCAN: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg")
        node.planner = _FakeCrashingPlanner()
        node._replan(1)
        self.assertFalse(node.state.ready)

        sub.latest[TOPIC_SCAN] = Scan(dist=[3.0] * 360, sector_seen=[True] * 12, seq=2)
        node.planner = make_planner("ftg")
        node._replan(2)
        self.assertTrue(node.state.ready, node.state.reason)


class _FakeFreezeClearPlanner:
    """`request_freeze`/`request_clear` の呼び出し記録（B4）。"""

    name = "freeze_clear"
    input_topic = TOPIC_SCAN
    stale_ms = 300

    def __init__(self) -> None:
        self.freeze_calls = 0
        self.clear_calls = 0

    def request_freeze(self) -> None:
        self.freeze_calls += 1

    def request_clear(self) -> None:
        self.clear_calls += 1

    def reset(self) -> None:
        pass


class TestFreezeClearWithModeChange(unittest.TestCase):
    """`freeze_seq`/`clear_seq` の増加が、モード変更と同時に来ても握り潰されないこと（B4）。"""

    def test_freeze_seq_increment_alone_still_fires(self):
        """回帰確認: モード変更を伴わない単体のfreezeは元から動いていた。"""
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg", freeze_seq=1))
        self.assertEqual(fake.freeze_calls, 1)

    def test_freeze_seq_increment_with_mode_change_still_fires(self):
        """バグ修正の本体: モード変更と同一メッセージでもfreezeが効くこと。"""
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg_cam", freeze_seq=1))
        self.assertEqual(fake.freeze_calls, 1, "モード変更と同時だとfreezeが握り潰されている")

    def test_clear_seq_increment_with_mode_change_still_fires(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg_cam", clear_seq=1))
        self.assertEqual(fake.clear_calls, 1, "モード変更と同時だとclearが握り潰されている")


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
