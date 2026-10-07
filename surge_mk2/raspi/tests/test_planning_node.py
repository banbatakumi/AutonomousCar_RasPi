"""`raspi/nodes/planning_node.py` の入力ルーティングのテスト。

**LiDAR 系 planner は無改造で動くこと**、そして**カメラ系 planner に
切り替えた瞬間に `scan/cam` だけを見るようになること**の2つを確認する。
バス・実プロセスは要らない——`Subscriber`/`Publisher` の代わりに
`latest` 辞書と `send()` の記録だけを持つ身代わりを使う
（`raspi/tests/test_auto.py` の `FakeSub` と同じ流儀）。
"""

import sys
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.auto.e2e_lidar import E2ELidar  # noqa: E402
from raspi.auto.registry import make_planner  # noqa: E402
from raspi.msgs import AutoCtrl, Scan  # noqa: E402
from raspi.msgs.types import (TOPIC_AUTO_CMD, TOPIC_E2E_MODEL, TOPIC_SCAN,  # noqa: E402
                              TOPIC_SCAN_CAM, E2EModelCtrl)
from raspi.nodes.planning_node import (  # noqa: E402
    POLL_MAX_MS, PlanningNode, _poll_timeout_ms, input_topics)


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


class _TimedSub:
    """本物の `Subscriber.poll` のように待ち、`t_new_ns` を過ぎたら新しい周の点群を出す。"""

    def __init__(self, t_new_ns: int):
        self.latest = {TOPIC_SCAN: Scan(dist=[3.0] * 360, sector_seen=[True] * 12, seq=1,
                                        t_pub=time.monotonic_ns())}
        self.t_new_ns = t_new_ns
        self.t_seen_ns = None

    def poll(self, timeout_ms):
        time.sleep(timeout_ms / 1000)
        now = time.monotonic_ns()
        if self.t_seen_ns is None and now >= self.t_new_ns:
            self.latest[TOPIC_SCAN] = Scan(dist=[3.0] * 360, sector_seen=[True] * 12, seq=2,
                                           t_pub=now)
            self.t_seen_ns = now
        return []


class _TimedPub:
    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((time.monotonic_ns(), topic, msg))


class TestImmediateAutoCmd(unittest.TestCase):
    """新しい判断は 50Hz の定期送信を待たずに `auto/cmd` へ出る（2026-09-26）。"""

    def test_new_plan_is_sent_right_away_and_periodic_continues(self):
        t0 = time.monotonic_ns()
        sub = _TimedSub(t0 + 70_000_000)
        pub = _TimedPub()
        node = PlanningNode(pub=pub, sub=sub, mode="ftg")
        node.run(duration_s=0.15)
        cmds = [t for t, topic, _ in pub.sent if topic == TOPIC_AUTO_CMD]
        after = [t for t in cmds if t >= sub.t_seen_ns]
        self.assertTrue(after)
        # 新しい周を見てから送るまで（poll の 2ms とループ1周ぶん）。定期送信だけなら最大20ms
        self.assertLess((after[0] - sub.t_seen_ns) / 1e6, 5.0)
        # 定期送信（途絶判定の鮮度を保つ繰り返し）は残っている: 0.15s で 50Hz なら約7回
        self.assertGreaterEqual(len(cmds), 6)


class TestAccelLimitPassThrough(unittest.TestCase):
    """planner の `AutoState.accel_limit` は `auto/cmd` に載る（2026-09-27、前後運動試験が段ごとの
    加速度を指定する）。制動・未 ready のときは載せない（制動が優先）。"""

    def test_accel_limit_goes_into_auto_cmd(self):
        from raspi.msgs.types import AutoState
        node = PlanningNode(pub=_TimedPub(), sub=_TimedSub(0), mode="ftg")
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        cmd = node._cmd_from(AutoState(ready=True, target_speed=1.0, accel_limit=1.6))
        self.assertEqual(cmd.accel_limit, 1.6)
        cmd = node._cmd_from(AutoState(ready=True, brake=True, accel_limit=1.6))
        self.assertTrue(cmd.brake)
        self.assertEqual(cmd.accel_limit, 0.0)

    def test_brake_torque_goes_into_auto_cmd_only_for_a_decided_brake(self):
        """planner が決めた制動（ready）の強さは載せる。「分からない」（ready=False）の制動は 0
        （中継側の GUI の値）。"""
        from raspi.msgs.types import AutoState
        node = PlanningNode(pub=_TimedPub(), sub=_TimedSub(0), mode="ftg")
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        cmd = node._cmd_from(AutoState(ready=True, brake=True, brake_torque=0.04))
        self.assertEqual((cmd.brake, cmd.brake_torque), (True, 0.04))
        cmd = node._cmd_from(AutoState(ready=False, brake=True, brake_torque=0.04))
        self.assertEqual((cmd.brake, cmd.brake_torque), (True, 0.0))


class TestNanPlannerOutputBrakes(unittest.TestCase):
    """issue #1: planner が NaN/Inf を出したら「分からない」（制動）と同じに扱うこと。
    `command_from_cmd`/`_merge_auto` にも同じ検査がある（多層防御）が、ここで
    落とせば `AutoState.reason` に理由が残る。"""

    def test_nan_target_speed_brakes(self):
        from raspi.msgs.types import AutoState
        node = PlanningNode(pub=_TimedPub(), sub=_TimedSub(0), mode="ftg")
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        cmd = node._cmd_from(AutoState(ready=True, target_speed=float("nan"), target_steer=0.1))
        self.assertTrue(cmd.brake)
        self.assertEqual(cmd.target_speed, 0.0)

    def test_inf_target_steer_brakes(self):
        from raspi.msgs.types import AutoState
        node = PlanningNode(pub=_TimedPub(), sub=_TimedSub(0), mode="ftg")
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        cmd = node._cmd_from(AutoState(ready=True, target_speed=0.5, target_steer=float("inf")))
        self.assertTrue(cmd.brake)


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
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg", freeze_seq=1))
        self.assertEqual(fake.freeze_calls, 1)

    def test_freeze_seq_increment_with_mode_change_still_fires(self):
        """バグ修正の本体: モード変更と同一メッセージでもfreezeが効くこと。"""
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg_cam", freeze_seq=1))
        self.assertEqual(fake.freeze_calls, 1, "モード変更と同時だとfreezeが握り潰されている")

    def test_clear_seq_increment_with_mode_change_still_fires(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg_cam", clear_seq=1))
        self.assertEqual(fake.clear_calls, 1, "モード変更と同時だとclearが握り潰されている")


class _FakeDisengagePlanner(_FakeFreezeClearPlanner):
    def __init__(self) -> None:
        super().__init__()
        self.resets = 0
        self.disengages = 0

    def reset(self) -> None:
        self.resets += 1

    def on_disengage(self) -> None:
        self.disengages += 1


class TestDisengage(unittest.TestCase):
    """自動運転の解除は `on_disengage` があればそちら（slam2d 系が地図作成中の地図を残す）。"""

    def test_release_calls_on_disengage_instead_of_reset(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeDisengagePlanner()
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=False))
        self.assertEqual((fake.disengages, fake.resets), (1, 0))

    def test_planner_without_hook_is_reset(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        calls = []
        fake.reset = lambda: calls.append(1)
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=False))
        self.assertEqual(calls, [1])


class TestRestartWhileEngaged(unittest.TestCase):
    """走行中にこのノードが起動し直したとき、engaged のまま走り出さないこと。"""

    def _node(self):
        sub = FakeSub({TOPIC_SCAN: _scan()})
        return PlanningNode(pub=FakePub(), sub=sub, mode="ftg")

    def test_first_ctrl_already_engaged_is_not_accepted(self):
        node = self._node()
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        self.assertFalse(node.ctrl.engaged)
        node._replan(1)
        st = node._current(1)
        self.assertFalse(st.engaged)
        self.assertIn("起動し直した", st.reason)
        cmd = node._cmd_from(st)
        self.assertEqual((cmd.mode, cmd.arm), (0, False))
        # 同じ意思が繰り返し届いても受けない
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        self.assertFalse(node.ctrl.engaged)

    def test_engage_is_accepted_after_a_release(self):
        node = self._node()
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=False))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        self.assertTrue(node.ctrl.engaged)
        node._replan(1)
        self.assertNotIn("起動し直した", node._current(1).reason)

    def test_counters_in_the_first_ctrl_are_history(self):
        """最初の1通に載っている回数（読み込み・駐車の目標など）は起動前の出来事。実行しない。"""
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        loads = []
        fake.request_load = loads.append
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True, freeze_seq=3, clear_seq=2,
                                  race_seq=5, race_map="m"))
        self.assertEqual((fake.freeze_calls, fake.clear_calls, loads), (0, 0, []))
        # その後に増えたぶんは効く
        node._apply_ctrl(AutoCtrl(mode="ftg", freeze_seq=4, clear_seq=2, race_seq=6, race_map="m"))
        self.assertEqual((fake.freeze_calls, fake.clear_calls, loads), (1, 0, ["m"]))


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


class TestPollTimeout(unittest.TestCase):
    """poll は次の定期送信の期限まで待つ（固定2msで500Hz起床していたのを直した、2026-10-04）。"""

    def test_past_deadline_does_not_block(self):
        self.assertEqual(_poll_timeout_ms(0), 0)
        self.assertEqual(_poll_timeout_ms(-5_000_000), 0)

    def test_rounds_up_so_it_never_wakes_before_the_deadline(self):
        self.assertEqual(_poll_timeout_ms(1), 1)
        self.assertEqual(_poll_timeout_ms(1_000_001), 2)

    def test_capped(self):
        self.assertEqual(_poll_timeout_ms(10**9), POLL_MAX_MS)


if __name__ == "__main__":
    unittest.main()
