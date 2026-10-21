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
from raspi.msgs import AutoCtrl, AutoState, Scan, VehicleState  # noqa: E402
from raspi.msgs.types import (TOPIC_AUTO_CMD, TOPIC_E2E_MODEL, TOPIC_SCAN,  # noqa: E402
                              TOPIC_SCAN_CAM, TOPIC_VEHICLE_STATE, E2EModelCtrl)
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

    def test_launch_goes_into_auto_cmd_only_while_driving(self):
        """ローンチコントロールの要求（v0.20）は走る指令にだけ載る（制動・未 ready では載せない）。"""
        from raspi.msgs.types import AutoState
        node = PlanningNode(pub=_TimedPub(), sub=_TimedSub(0), mode="ftg")
        node._apply_ctrl(AutoCtrl(mode="ftg"))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        self.assertTrue(node._cmd_from(AutoState(ready=True, target_speed=2.0, launch=True)).launch)
        self.assertFalse(node._cmd_from(AutoState(ready=True, target_speed=2.0)).launch)
        self.assertFalse(node._cmd_from(AutoState(ready=True, brake=True, launch=True)).launch)
        self.assertFalse(node._cmd_from(AutoState(ready=False, target_speed=2.0, launch=True)).launch)

    def test_lights_go_into_auto_cmd_even_while_braking(self):
        """灯火（障害物でのパッシング・ウィンカー、駐車完了のハザード）は制動中の指令にも載る。"""
        from raspi.msgs.types import AutoState
        node = PlanningNode(pub=_TimedPub(), sub=_TimedSub(0), mode="ftg")
        node._apply_ctrl(AutoCtrl(mode="ftg"))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        run = node._cmd_from(AutoState(ready=True, target_speed=2.0, passing=True, winker_left=True))
        self.assertTrue(run.passing and run.winker_left and not run.winker_right)
        park = node._cmd_from(AutoState(ready=True, brake=True, winker_left=True, winker_right=True))
        self.assertTrue(park.brake and park.winker_left and park.winker_right)

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


class TestPerceptionReason(unittest.TestCase):
    """知覚ノードが載せた `reason` が、鮮度判定の「古い」より先に表示へ出ること。"""

    def test_reason_replaces_stale_message_when_t_capture_is_zero(self):
        sub = FakeSub({TOPIC_SCAN_CAM: Scan(dist=[0.0] * 360, sector_seen=[False] * 12,
                                            reason="モデル未選択")})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg_cam")
        st = node._current(10_000 * 1_000_000)
        self.assertEqual(st.reason, "モデル未選択")
        self.assertNotIn("古い", st.reason)
        self.assertFalse(st.ready)            # 止まる挙動は変わらない

    def test_reason_and_age_when_capture_exists(self):
        scan = Scan(dist=[3.0] * 360, sector_seen=[True] * 12, t_capture=1_000_000,
                    reason="フレームが来ない")
        node = PlanningNode(pub=FakePub(), sub=FakeSub({TOPIC_SCAN_CAM: scan}), mode="ftg_cam")
        st = node._current(2_000 * 1_000_000)
        self.assertIn("フレームが来ない", st.reason)
        self.assertIn("古い", st.reason)

    def test_without_reason_message_is_unchanged(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub({TOPIC_SCAN_CAM: _scan()}),
                            mode="ftg_cam")
        self.assertTrue(node._current(600 * 1_000_000).reason.startswith("点群が"))


class TestVehicleStateStaleness(unittest.TestCase):
    """`vehicle_state` が古ければ制動に読み替える（TELEMETRY だけ止まったとき、固まった値で走らない）。"""

    MS = 1_000_000

    def _node(self, vs):
        # 点群は 200ms 時点のもの（下の判定時刻ではまだ新しい）
        scan = Scan(dist=[3.0] * 360, sector_seen=[True] * 12, seq=1,
                    t_pub=200 * self.MS, t_capture=200 * self.MS)
        sub = FakeSub({TOPIC_SCAN: scan, TOPIC_VEHICLE_STATE: vs})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg")
        node._replan(200 * self.MS)
        return node

    def test_stale_vehicle_state_is_not_ready(self):
        node = self._node(VehicleState(t_pub=10 * self.MS))
        self.assertTrue(node._current(300 * self.MS).ready)       # 290ms 古い: まだ走る
        st = node._current(320 * self.MS)                         # 310ms 古い
        self.assertFalse(st.ready)
        self.assertIn("車両状態", st.reason)

    def test_missing_or_untimed_vehicle_state_is_left_to_the_planner(self):
        """届いたことが無い・時刻の無いものでは止めない（planner が `vs is None` を扱う）。"""
        self.assertTrue(self._node(VehicleState())._current(400 * self.MS).ready)
        node = self._node(VehicleState())
        del node.sub.latest[TOPIC_VEHICLE_STATE]
        self.assertTrue(node._current(400 * self.MS).ready)


class TestNotReadyKeepsLastSteer(unittest.TestCase):
    """「分からない」（ready=False）の制動は、最後に出した舵を保つ（直進へ戻さない）。"""

    def _engaged(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        node._apply_ctrl(AutoCtrl(mode="ftg"))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        return node

    def test_brake_holds_the_steer_of_the_last_command(self):
        node = self._engaged()
        node._cmd_from(AutoState(ready=True, target_speed=1.0, target_steer=0.3))
        st = AutoState(reason="点群が 400ms 古い")              # 新しい AutoState（舵 0）
        cmd = node._cmd_from(st)
        self.assertTrue(cmd.brake)
        self.assertEqual(cmd.target_steer, 0.3)
        self.assertEqual(st.target_steer, 0.3)                    # auto/state にも出す舵が見える

    def test_nan_output_brakes_with_the_last_finite_steer(self):
        node = self._engaged()
        node._cmd_from(AutoState(ready=True, target_speed=1.0, target_steer=-0.2))
        cmd = node._cmd_from(AutoState(ready=True, target_speed=1.0, target_steer=float("nan")))
        self.assertTrue(cmd.brake)
        self.assertEqual(cmd.target_steer, -0.2)

    def test_a_decided_brake_uses_the_planners_steer(self):
        node = self._engaged()
        node._cmd_from(AutoState(ready=True, target_speed=1.0, target_steer=0.3))
        cmd = node._cmd_from(AutoState(ready=True, brake=True, target_steer=0.1))
        self.assertEqual(cmd.target_steer, 0.1)

    def test_steer_is_not_carried_into_the_next_engage(self):
        node = self._engaged()
        node._cmd_from(AutoState(ready=True, target_speed=1.0, target_steer=0.3))
        node._apply_ctrl(AutoCtrl(mode="ftg"))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        self.assertEqual(node._cmd_from(AutoState()).target_steer, 0.0)


class TestSeqAfterTelemetryRestart(unittest.TestCase):
    """telemetry_node が起動し直すと `AutoCtrl` の回数（`*_seq`）は 0 に戻る。逆行では発火せず、
    その後の新しい操作（0 → 1）は前の値 N を超えるのを待たずに効くこと。"""

    def test_counter_going_back_syncs_and_next_increment_fires(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg", quiet=True)
        node.planner = _FakeFreezeClearPlanner()
        node._apply_ctrl(AutoCtrl(mode="ftg", freeze_seq=5, clear_seq=3))    # 最初の1通は同期だけ
        self.assertEqual((node.planner.freeze_calls, node.planner.clear_calls), (0, 0))
        node._apply_ctrl(AutoCtrl(mode="ftg", freeze_seq=6, clear_seq=3))
        self.assertEqual(node.planner.freeze_calls, 1)
        node._apply_ctrl(AutoCtrl(mode="ftg"))                                # 再起動で 0 へ逆行
        self.assertEqual((node.planner.freeze_calls, node.planner.clear_calls), (1, 0))
        node._apply_ctrl(AutoCtrl(mode="ftg", freeze_seq=1, clear_seq=1))    # 新しい操作
        self.assertEqual((node.planner.freeze_calls, node.planner.clear_calls), (2, 1))


class _FakeRaisingRequests:
    """`auto/ctrl` の要求で例外を投げる planner の身代わり。"""

    name = "raising"
    input_topic = TOPIC_SCAN
    stale_ms = 300

    def request_load(self, name):
        raise ValueError("地図ファイルが壊れている")

    def request_freeze(self):
        pass

    def reset(self):
        raise RuntimeError("reset の中のバグ")

    def plan(self, scan, vs, p, dt):
        return AutoState(ready=True, target_speed=0.2, reason="走行中")


class TestCtrlRequestSurvivesPlannerException(unittest.TestCase):
    """`auto/ctrl` の要求で planner が例外を投げても、ノードは落ちず理由を `auto/state` に出す。"""

    def test_request_exception_is_reported_and_cleared_by_the_next_request(self):
        sub = FakeSub({TOPIC_SCAN: _scan()})
        node = PlanningNode(pub=FakePub(), sub=sub, mode="ftg", quiet=True)
        node.planner = _FakeRaisingRequests()
        node._apply_ctrl(AutoCtrl(mode="ftg"))
        node._apply_ctrl(AutoCtrl(mode="ftg", race_seq=1, race_map="broken"))   # 落ちないこと
        node._replan(1)
        self.assertIn("地図の読み込みに失敗", node._current(1).reason)
        node._apply_ctrl(AutoCtrl(mode="ftg", race_seq=1, freeze_seq=1))        # 通る要求で消える
        sub.latest[TOPIC_SCAN] = Scan(dist=[3.0] * 360, sector_seen=[True] * 12, seq=2)
        node._replan(2)
        self.assertNotIn("失敗", node._current(2).reason)

    def test_reset_exception_on_release_does_not_propagate(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg", quiet=True)
        node.planner = _FakeRaisingRequests()
        node._apply_ctrl(AutoCtrl(mode="ftg"))
        node._apply_ctrl(AutoCtrl(mode="ftg", engaged=True))
        node._apply_ctrl(AutoCtrl(mode="ftg"))                                  # reset() が投げる
        self.assertFalse(node.ctrl.engaged)

    def test_planner_that_cannot_be_built_leaves_no_mode(self):
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="", quiet=True)
        with mock.patch("raspi.nodes.planning_node.make_planner",
                        side_effect=RuntimeError("依存が無い")):
            node._apply_ctrl(AutoCtrl(mode="ftg"))
        self.assertIsNone(node.planner)
        self.assertIn("依存が無い", node._current(1).reason)


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
        self.grip_clears: list[str] = []

    def request_freeze(self) -> None:
        self.freeze_calls += 1

    def request_clear(self) -> None:
        self.clear_calls += 1

    def request_clear_grip(self, name: str) -> None:
        self.grip_clears.append(name)

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

    def test_grip_clear_seq_increment_passes_the_map_name_once(self):
        """「学習値を消す」は回数で一度だけ、対象の地図名を planner へ渡す。"""
        node = PlanningNode(pub=FakePub(), sub=FakeSub(), mode="ftg")
        fake = _FakeFreezeClearPlanner()
        node.planner = fake
        node._apply_ctrl(AutoCtrl(mode="ftg"))      # 起動後の最初の1通（解除状態）
        req = AutoCtrl(mode="ftg", grip_clear_seq=1, grip_clear_map="course")
        node._apply_ctrl(req)
        node._apply_ctrl(req)                       # 同じ回数の再送では繰り返さない
        self.assertEqual(fake.grip_clears, ["course"])


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


class TestStandstillSteer(unittest.TestCase):
    """停車が続いたら舵を中立へ戻す（据え切りでステア MD が過熱する）。以前は `park_to_point` だけだった。"""

    def _node(self, mode="ftg", speed=0.0, stopped=True):
        vs = VehicleState(speed=speed, stopped=stopped)
        node = PlanningNode(pub=FakePub(), sub=FakeSub({TOPIC_VEHICLE_STATE: vs}), mode=mode)
        node.ctrl = AutoCtrl(mode=mode, engaged=True)
        return node

    def _run(self, node, cmd, seconds):
        out, t = cmd, 0
        for _ in range(int(seconds * 50)):
            t += 20_000_000
            out = node._relax_standstill_steer(cmd, t)
        return out

    def test_full_lock_is_released_after_standing_still(self):
        from raspi.msgs import DriveCmd
        node = self._node()
        cmd = DriveCmd(mode=2, arm=True, brake=True, target_steer=0.4)
        self.assertEqual(self._run(node, cmd, 1.0).target_steer, 0.4)     # まだ保つ
        self.assertEqual(self._run(node, cmd, 3.0).target_steer, 0.0)     # 2 秒を超えたら中立

    def test_moving_or_starting_is_untouched(self):
        from raspi.msgs import DriveCmd
        node = self._node(speed=1.0, stopped=False)
        cmd = DriveCmd(mode=2, arm=True, brake=True, target_steer=0.4)
        self.assertEqual(self._run(node, cmd, 5.0).target_steer, 0.4)
        node = self._node()
        go = DriveCmd(mode=2, arm=True, target_speed=1.0, target_steer=0.4)   # 発進の指令
        self.assertEqual(self._run(node, go, 5.0).target_steer, 0.4)

    def test_sysid_planner_is_exempt(self):
        from raspi.auto.registry import PLANNERS
        from raspi.msgs import DriveCmd
        sysid = next(k for k, v in PLANNERS.items() if v.category == "sysid")
        node = self._node(mode=sysid)
        cmd = DriveCmd(mode=2, arm=True, brake=True, target_steer=0.4)
        self.assertEqual(self._run(node, cmd, 5.0).target_steer, 0.4)


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
