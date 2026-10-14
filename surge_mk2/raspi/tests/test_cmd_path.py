"""GUI → WS → バス → io_node → UART の指令経路のテスト。

**この経路が「意図せず開いている」ことが一番危ない。** 通信のテストではなく
安全条件のテストとして書く。ハードウェアもシリアルも要らない。
"""

import asyncio
import sys
import time
import unittest
from pathlib import Path

import msgspec

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.io.serial_link import RxFrame  # noqa: E402
from raspi.msgs import DriveCmd  # noqa: E402
from raspi.nodes.io_node import (  # noqa: E402
    CMD_TIMEOUT_NS,
    FAILSAFE_BRAKE_MAX_NS,
    FAILSAFE_STOPPED_NS,
    IoNode,
)
from raspi.proto import packets  # noqa: E402
from raspi.proto.framing import RxStats  # noqa: E402


class FakeLink:
    """`SerialLink` の最小の身代わり。送ったパケットを覚えるだけ。"""

    def __init__(self):
        self.sent = []
        self.stats = RxStats()
        self.on_tx = None
        self._seq = 0

    def poll(self) -> list[RxFrame]:
        return []

    def send(self, packet) -> int:
        self.sent.append(packet)
        self._seq = (self._seq + 1) & 0xFF
        if self.on_tx:
            self.on_tx(time.monotonic_ns(), packet.TYPE, self._seq, packet.encode())
        return time.monotonic_ns()

    def send_raw(self, *a, **kw):
        return time.monotonic_ns()

    def reset_input(self):
        pass

    def close(self):
        pass


class FakeSub:
    """`cmd` を任意のタイミングで流し込めるバスの身代わり。"""

    def __init__(self):
        self.queue = []

    def push(self, cmd: DriveCmd):
        self.queue.append(("cmd", cmd))

    def poll(self, timeout_ms=0):
        out, self.queue = self.queue, []
        return out

    def close(self):
        pass


def commands(link) -> list:
    return [p for p in link.sent if isinstance(p, packets.Command)]


def mark_handshaken(node: IoNode) -> IoNode:
    """`handshake()` で STM32 の `VERSION` が一致したことにする（一致するまで ARM は通らない）。"""
    node.state.version = packets.Version(protocol_version=packets.PROTOCOL_VERSION)
    return node


class TestIoNodeArmGate(unittest.TestCase):
    """モータが回るのに必要な条件。**どれか1つでも欠けたら DISARM。**"""

    def setUp(self):
        self.link = FakeLink()
        self.sub = FakeSub()

    def node(self, *, handshaken: bool = True, **kw) -> IoNode:
        n = IoNode(self.link, sub=self.sub, **kw)
        return mark_handshaken(n) if handshaken else n

    def assert_all_disarm(self):
        cs = commands(self.link)
        self.assertTrue(cs)
        self.assertTrue(all(c.mode == packets.Mode.DISARM and c.flags == 0 for c in cs))

    def test_without_a_version_from_the_stm32_arm_is_blocked(self):
        """STM32 の版を確認できていない間は、`--allow-arm` があっても通さない。"""
        n = self.node(allow_arm=True, handshaken=False)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.1)
        self.assert_all_disarm()
        self.assertTrue(n._arm_block_reason())
        # 取り直しを試みている（`handshake()` で取り損ねても走れなくならないように）
        self.assertTrue(any(isinstance(p, packets.VersionReq) for p in self.link.sent))

    def test_a_protocol_version_mismatch_blocks_arm(self):
        """片側だけ入れ替えると舵角・フラグの意味が食い違う。表示するだけでなく走らせない。"""
        n = self.node(allow_arm=True, handshaken=False)
        n.state.version = packets.Version(protocol_version=packets.PROTOCOL_VERSION - 1)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.1)
        self.assert_all_disarm()
        self.assertIn("protocol_version", n._arm_block_reason())

    def test_arm_is_blocked_while_the_stm32_has_never_seen_the_heartbeat(self):
        """E-Stop の層（GPIO6）が STM32 に見えていなければ止める手段が欠ける。`--no-heartbeat` なら承知の上。"""
        class _Hb:
            def kick(self):
                pass

        n = self.node(allow_arm=True, heartbeat=_Hb())
        n.state.no_heartbeat = True
        self.assertIn("ハートビート", n._arm_block_reason())
        n.state.no_heartbeat = False
        self.assertNotIn("ハートビート", n._arm_block_reason())
        # ハートビートを出さない（--no-heartbeat）起動では、このビットを理由に封印しない
        n2 = self.node(allow_arm=True)
        n2.state.no_heartbeat = True
        self.assertNotIn("ハートビート", n2._arm_block_reason())

    def test_arm_waits_until_the_control_params_are_in_the_stm32(self):
        """`[control]`（ステアのリンクの換算を含む）が STM32 に入るまでは走らせない。"""
        n = self.node(allow_arm=True, control_params={"tc_slip_target": 0.1})
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.1)
        self.assert_all_disarm()
        sets = [p for p in self.link.sent if isinstance(p, packets.ConfigSet)]
        self.assertTrue(sets)

        self.link.sent.clear()
        n.control_sync.on_ack(packets.ConfigAck(
            param_id=sets[0].param_id, applied=sets[0].value, result=packets.ConfigResult.OK))
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.1)
        self.assertEqual(n._arm_block_reason(), "")
        self.assertTrue(any(c.flags & packets.CMD_FLG_ARM for c in commands(self.link)))

    def test_a_drifted_control_param_blocks_arm_until_it_is_back(self):
        """読み戻しが食い違った＝STM32 の値が勝手に変わった（気づけなかった再起動など）。"""
        n = self.node(allow_arm=True, control_params={"tc_slip_target": 0.1})
        n.run(duration_s=0.05)
        sent = [p for p in self.link.sent if isinstance(p, packets.ConfigSet)][0]
        ok = packets.ConfigAck(param_id=sent.param_id, applied=sent.value,
                               result=packets.ConfigResult.OK)
        n.control_sync.on_ack(ok)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.05)
        self.assertEqual(n._arm_block_reason(), "")

        n.control_sync.on_ack(packets.ConfigAck(param_id=sent.param_id, applied=0.0,
                                                result=packets.ConfigResult.OK))
        self.link.sent.clear()
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.05)
        self.assert_all_disarm()
        self.assertTrue(n._arm_block_reason())

        n.control_sync.on_ack(ok)                 # 送り直しが入った
        self.link.sent.clear()
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.05)
        self.assertTrue(any(c.flags & packets.CMD_FLG_ARM for c in commands(self.link)))

    def test_an_unreadable_control_section_blocks_arm(self):
        n = self.node(allow_arm=True, control_params_error="vehicle.toml の [control] を読めません")
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.1)
        self.assert_all_disarm()

    # ── 異常系: 走っていたなら、DISARM の前に制動する ──

    def _armed_then(self, n: IoNode, **cmd) -> None:
        """ARM した通常の指令を1つ通してから、`cmd` の状況にする。"""
        self.sub.push(DriveCmd(mode=2, arm=True, target_speed=1.0, target_steer=0.2,
                               steer_rate_limit=7.0, source="gui"))
        n.run(duration_s=0.05)
        self.assertIsNotNone(n._last_armed_pkt)
        self.link.sent.clear()

    def test_a_stale_cmd_brakes_before_disarming(self):
        """`cmd` が途絶えたら、ARM を立てたまま最大制動を送る（いきなり DISARM だと STM32 は
        駆動電源を切り、モータドライバごと落ちて惰行する）。モードと舵角は最後の指令のまま。"""
        n = self.node(allow_arm=True, max_speed=2.0, max_steer=0.5)
        self._armed_then(n)
        n._cmd_ns -= CMD_TIMEOUT_NS + 1
        n.run(duration_s=0.1)
        cs = commands(self.link)
        self.assertTrue(cs)
        for c in cs:
            self.assertEqual(c.mode, packets.Mode.AUTO)
            self.assertTrue(c.flags & packets.CMD_FLG_ARM)
            self.assertTrue(c.flags & packets.CMD_FLG_BRAKE)
            self.assertEqual(c.target_speed, 0)
            self.assertEqual(c.brake_torque, 0)          # 0 = STM32 の最大
            self.assertEqual(c.target_steer, 2000)       # 直進へ戻さない
        self.assertEqual(n.failsafe_brakes, 1)

    def test_the_ws_deadman_brakes_before_disarming(self):
        """telemetry_node のデッドマンは `cmd` 自体は届き続ける（mode=0）。発行元で見分ける。"""
        n = self.node(allow_arm=True, max_speed=2.0, max_steer=0.5)
        self._armed_then(n)
        self.sub.push(DriveCmd(mode=0, source="deadman"))
        n.run(duration_s=0.1)
        cs = commands(self.link)
        self.assertTrue(cs)
        self.assertTrue(all(c.flags & packets.CMD_FLG_BRAKE and c.flags & packets.CMD_FLG_ARM
                            for c in cs))

    def test_an_explicit_disarm_from_the_operator_is_passed_through(self):
        """人が DISARM を押したのは異常系ではない（止まるまでの制動は STM32 が受け持つ）。"""
        n = self.node(allow_arm=True, max_speed=2.0, max_steer=0.5)
        self._armed_then(n)
        self.sub.push(DriveCmd(mode=0, arm=False, source="gui"))
        n.run(duration_s=0.1)
        self.assert_all_disarm()

    def test_failsafe_braking_ends_once_the_vehicle_has_stopped(self):
        n = self.node(allow_arm=True, max_speed=2.0, max_steer=0.5)
        self._armed_then(n)
        n.cmd_stale = True
        n.state.health = "OK"
        t0 = time.monotonic_ns()
        n.state.telemetry = packets.Telemetry(speed=800)            # 0.8 m/s
        self.assertTrue(n._failsafe_packet(t0).flags & packets.CMD_FLG_BRAKE)
        n.state.telemetry = packets.Telemetry(speed=10)             # 0.01 m/s = 停止
        self.assertTrue(n._failsafe_packet(t0 + 1).flags & packets.CMD_FLG_BRAKE)
        self.assertTrue(n._failsafe_packet(t0 + FAILSAFE_STOPPED_NS // 2).flags
                        & packets.CMD_FLG_BRAKE)
        done = n._failsafe_packet(t0 + 2 + FAILSAFE_STOPPED_NS)
        self.assertEqual((done.mode, done.flags), (packets.Mode.DISARM, 0))
        # 以後は指令が戻るまで DISARM のまま（制動へ戻らない）
        again = n._failsafe_packet(t0 + 3 + FAILSAFE_STOPPED_NS)
        self.assertEqual((again.mode, again.flags), (packets.Mode.DISARM, 0))

    def test_failsafe_braking_times_out_when_telemetry_is_lost(self):
        """`TELEMETRY` が見えなければ停止を確認できない。上限まで制動してから DISARM。"""
        n = self.node(allow_arm=True, max_speed=2.0, max_steer=0.5)
        self._armed_then(n)
        n.state.health = "FAULT"
        n.state.telemetry = packets.Telemetry(speed=0)              # 古い値は信じない
        t0 = time.monotonic_ns()
        self.assertTrue(n._failsafe_packet(t0).flags & packets.CMD_FLG_BRAKE)
        self.assertTrue(n._failsafe_packet(t0 + FAILSAFE_BRAKE_MAX_NS - 1).flags
                        & packets.CMD_FLG_BRAKE)
        done = n._failsafe_packet(t0 + FAILSAFE_BRAKE_MAX_NS)
        self.assertEqual((done.mode, done.flags), (packets.Mode.DISARM, 0))

    def test_without_allow_arm_a_live_cmd_is_still_disarmed(self):
        n = self.node(allow_arm=False)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, source="gui"))
        n.run(duration_s=0.1)
        cs = commands(self.link)
        self.assertTrue(cs)
        self.assertTrue(all(c.mode == packets.Mode.DISARM for c in cs))
        self.assertTrue(all(c.flags == 0 for c in cs))
        self.assertTrue(all(c.target_speed == 0 for c in cs))

    def test_with_allow_arm_the_cmd_reaches_the_wire(self):
        n = self.node(allow_arm=True, max_speed=2.0)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0,
                               target_steer=0.1, source="gui"))
        n.run(duration_s=0.1)
        live = [c for c in commands(self.link) if c.mode == packets.Mode.MANUAL]
        self.assertTrue(live)
        self.assertTrue(live[0].flags & packets.CMD_FLG_ARM)
        self.assertEqual(live[0].target_speed, 1000)

    def test_pi_side_clamp_applies_even_when_armed(self):
        """STM32 の上限だけに頼らない。上位が壊れたときに下位だけが守る形にしない。"""
        n = self.node(allow_arm=True, max_speed=0.3, max_steer=0.05)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=9.0, target_steer=1.5))
        n.run(duration_s=0.1)
        live = [c for c in commands(self.link) if c.mode == packets.Mode.MANUAL]
        self.assertTrue(live)
        self.assertEqual(live[0].target_speed, 300)
        self.assertEqual(live[0].target_steer, 500)

    def test_limits_overrides_max_speed_even_when_more_permissive(self):
        """★v0.11・2026-08-22。`LIMITS` を受信済みなら `--max-speed`/`--max-steer` より
        緩くても無条件に採用する（`min()` の多層防御ではなく「STM32 実測を優先」に変更した）。"""
        n = self.node(allow_arm=True, max_speed=0.3, max_steer=0.05)
        n.state.limits = packets.Limits(max_speed_m_s=5.0, max_accel_m_s2=3.0,
                                        max_torque_nm=0.15, max_steer_rad=0.524)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=9.0, target_steer=1.5))
        n.run(duration_s=0.1)
        live = [c for c in commands(self.link) if c.mode == packets.Mode.MANUAL]
        self.assertTrue(live)
        # 9.0/1.5 は LIMITS(5.0/0.524) でクランプされる。--max-speed=0.3 の出る幕は無い
        self.assertEqual(live[0].target_speed, 5000)
        self.assertEqual(live[0].target_steer, 5240)

    def test_limits_overrides_max_speed_even_when_more_restrictive(self):
        """逆方向。`LIMITS` が `--max-speed` より厳しくても、こちらが優先される。"""
        n = self.node(allow_arm=True, max_speed=2.0, max_steer=0.5)
        n.state.limits = packets.Limits(max_speed_m_s=0.2, max_accel_m_s2=3.0,
                                        max_torque_nm=0.15, max_steer_rad=0.05)
        self.sub.push(DriveCmd(mode=1, arm=True, target_speed=1.0, target_steer=0.3))
        n.run(duration_s=0.1)
        live = [c for c in commands(self.link) if c.mode == packets.Mode.MANUAL]
        self.assertTrue(live)
        self.assertEqual(live[0].target_speed, 200)
        self.assertEqual(live[0].target_steer, 500)

    def test_stale_cmd_falls_back_to_disarm(self):
        """publish 側が死んだら止める。**沈黙は「そのまま走り続けろ」ではない。**"""
        n = self.node(allow_arm=True)
        n.cmd = DriveCmd(mode=1, arm=True, target_speed=1.0)
        n._cmd_ns = time.monotonic_ns() - CMD_TIMEOUT_NS - 1
        n.run(duration_s=0.1)
        self.assertTrue(n.cmd_stale)
        self.assertTrue(all(c.mode == packets.Mode.DISARM
                            for c in commands(self.link)))

    def test_cmd_going_stale_is_counted(self):
        n = self.node(allow_arm=True)
        self.sub.push(DriveCmd(mode=1, arm=True))
        n.run(duration_s=0.05)
        self.assertFalse(n.cmd_stale)
        n._cmd_ns -= CMD_TIMEOUT_NS + 1
        n.run(duration_s=0.05)
        self.assertTrue(n.cmd_stale)
        self.assertEqual(n.cmd_timeouts, 1)

    def test_no_cmd_at_all_means_disarm(self):
        n = self.node(allow_arm=True)
        n.run(duration_s=0.1)
        self.assertTrue(n.cmd_stale)
        self.assertTrue(commands(self.link))
        self.assertTrue(all(c.mode == packets.Mode.DISARM
                            for c in commands(self.link)))


class TestEndToEndLatency(unittest.TestCase):
    """`cmd_seq_echo` を使った往復測定（`uart_protocol.md` §6.1）。時刻同期に依存しない。"""

    def test_rtt_is_measured_from_the_echo(self):
        from raspi.core.bus_bridge import BusBridge

        b = BusBridge(None)
        t0 = time.monotonic_ns()
        b.note_command_tx(57, t0 - 12_000_000)          # 12ms 前に送った
        b.on_telemetry(packets.Telemetry(cmd_seq_echo=57), t0)
        self.assertIsNotNone(b.cmd_rtt_ms)
        self.assertAlmostEqual(b.cmd_rtt_ms, 12.0, delta=3.0)

    def test_a_wrapped_seq_does_not_report_a_fake_2_second_latency(self):
        """SEQ は u8 なので 100Hz では 2.56秒で一周する。古い記録を拾わせない。"""
        from raspi.core.bus_bridge import BusBridge

        b = BusBridge(None)
        b.note_command_tx(57, time.monotonic_ns() - 3_000_000_000)
        b.on_telemetry(packets.Telemetry(cmd_seq_echo=57), time.monotonic_ns())
        self.assertIsNone(b.cmd_rtt_ms)


class TestLidarSectorTime(unittest.TestCase):
    """LiDAR セクタの時刻は受信時刻ではなく、STM32 が付けた `t_start_us` を換算した値。"""

    def _bridge(self, offset_ns):
        from raspi.core.bus_bridge import BusBridge

        b = BusBridge(None)
        b.to_pi_ns = (lambda t_us: None) if offset_ns is None else (lambda t_us: t_us * 1000 + offset_ns)
        seen = []
        b.scans.feed = lambda msg, t: seen.append(t)
        return b, seen

    def test_the_sector_is_stamped_with_the_converted_stm32_time(self):
        b, seen = self._bridge(offset_ns=5_000_000_000)
        rx = 5_000_000_000 + 1_000_000 * 1000 + 12_000_000     # 測り始めの 12ms 後に受信
        b.on_frame(rx, packets.LidarSector.TYPE, 0, packets.LidarSector(t_start_us=1_000_000))
        self.assertEqual(seen, [rx - 12_000_000])

    def test_it_falls_back_to_the_rx_time_when_the_conversion_is_not_usable(self):
        rx = 9_000_000_000
        for offset in (None,                       # 時刻同期が未収束
                       rx,                         # 換算後が受信より後
                       rx - 1_000_000_000 - 1_000_000_000):   # 1秒も古い（同期の外れ）
            b, seen = self._bridge(offset_ns=offset)
            b.on_frame(rx, packets.LidarSector.TYPE, 0, packets.LidarSector(t_start_us=1_000_000))
            self.assertEqual(seen, [rx])

    def test_without_a_time_sync_the_rx_time_is_used(self):
        from raspi.core.bus_bridge import BusBridge

        b = BusBridge(None)
        seen = []
        b.scans.feed = lambda msg, t: seen.append(t)
        b.on_frame(123, packets.LidarSector.TYPE, 0, packets.LidarSector(t_start_us=1))
        self.assertEqual(seen, [123])


class TestWsDeadman(unittest.IsolatedAsyncioTestCase):
    """WS 層のデッドマン。**誰も操縦していなければ明示的な DISARM を流し続ける。**"""

    def _server(self):
        from raspi.nodes.telemetry_node import TelemetryServer

        srv = TelemetryServer.__new__(TelemetryServer)   # バスを開かずに中身だけ
        srv._last_cmd = None
        srv._last_cmd_ns = 0
        srv._cmd_was_live = False
        srv._last_auto_key = None
        srv.deadman_trips = 0
        srv.controller = None
        srv.controller_name = ""
        srv._running = True
        srv.cmds_published = 0
        srv.published = []
        srv.control_clients = set()
        srv.telemetry_clients = set()
        srv.camera_clients = {"front": set(), "rear": set()}
        srv._jpeg_impl = None
        srv._jpeg = None                   # カメラ配信なし（JPEG の実測も出ない）
        srv.sub = type("S", (), {"latest": {}})()
        srv._sfl_active = False
        srv._mcap_proc = None
        srv._mcap_started_ns = 0
        srv._mcap_error = None

        class _P:
            """本物の `Publisher.send` と同じく、渡された msg に `seq`/`t_pub` を押す。
            押さない偽物だと「送った指令と次の指令を比べる」処理の不一致を見逃す"""
            seq = 0

            def send(self, topic, msg):
                self.seq += 1
                msg.seq = self.seq
                msg.t_pub = time.monotonic_ns()
                srv.published.append((topic, msg))
        srv.pub = _P()
        return srv

    async def test_publishes_disarm_when_nobody_is_driving(self):
        srv = self._server()
        task = asyncio.create_task(srv._cmd_pump())
        await asyncio.sleep(0.1)
        srv._running = False
        task.cancel()
        self.assertTrue(srv.published)
        self.assertTrue(all(m.mode == 0 and m.source == "deadman"
                            for _, m in srv.published))

    async def test_a_live_cmd_passes_through_then_expires(self):
        from raspi.nodes.telemetry_node import CMD_DEADMAN_NS

        srv = self._server()
        srv._last_cmd = DriveCmd(mode=1, arm=True, target_speed=0.5, source="gui:x")
        srv._last_cmd_ns = time.monotonic_ns()
        task = asyncio.create_task(srv._cmd_pump())
        await asyncio.sleep(0.06)
        self.assertTrue(any(m.mode == 1 for _, m in srv.published))

        srv._last_cmd_ns -= CMD_DEADMAN_NS + 1        # 指令が古くなった
        srv.published.clear()
        await asyncio.sleep(0.06)
        srv._running = False
        task.cancel()
        await asyncio.sleep(0)
        self.assertTrue(srv.published)
        self.assertTrue(all(m.mode == 0 for _, m in srv.published))
        self.assertEqual(srv.deadman_trips, 1)


class TestImmediateCmdRelay(unittest.IsolatedAsyncioTestCase):
    """新しい `auto/cmd` はその場で `/cmd` へ中継する（2026-09-26）。**安全の判断は定期送信と
    同じ関数を通す**ので、即時の経路からデッドマンをすり抜けた指令が出ることはない。"""

    def _server(self, live: bool):
        srv = TestWsDeadman._server(self)
        srv._auto_engaged = True
        srv._auto_mode = "ftg"
        srv._auto_was_fresh = True
        srv._auto_last_steer = 0.0
        srv.auto_stalls = 0
        if live:
            srv._last_cmd = DriveCmd(mode=2, arm=True, accel_limit=6.0, steer_rate_limit=7.0,
                                     source="gui:x")
            srv._last_cmd_ns = time.monotonic_ns()
        return srv

    def _auto(self, srv, speed: float):
        from raspi.msgs.types import TOPIC_AUTO_CMD
        srv.sub.latest[TOPIC_AUTO_CMD] = DriveCmd(mode=2, arm=True, target_speed=speed,
                                                  t_pub=time.monotonic_ns(), source="planning:ftg")

    async def test_changed_auto_cmd_is_relayed_at_once_with_gui_settings(self):
        srv = self._server(live=True)
        self._auto(srv, 1.0)
        srv._on_auto_cmd(time.monotonic_ns())
        self.assertEqual(len(srv.published), 1)
        m = srv.published[0][1]
        self.assertEqual((m.mode, m.target_speed, m.steer_rate_limit), (2, 1.0, 7.0))
        # 同じ中身（planning_node の 50Hz の繰り返し）は即時には送らない
        self._auto(srv, 1.0)
        srv._on_auto_cmd(time.monotonic_ns())
        self.assertEqual(len(srv.published), 1)

    async def test_deadman_still_wins_on_the_immediate_path(self):
        """操縦者（GUI）が途絶えていれば、新しい判断が来ても DISARM しか出ない。"""
        srv = self._server(live=False)
        self._auto(srv, 1.5)
        srv._on_auto_cmd(time.monotonic_ns())
        self.assertTrue(srv.published)
        self.assertTrue(all(m.mode == 0 and m.source == "deadman" for _, m in srv.published))


class TestBusPumpWakesOnAutoCmd(unittest.IsolatedAsyncioTestCase):
    """`_bus_pump` は本物のバスでは `auto/cmd` の到着で起きる（2026-10-06）。1ms ごとに
    覗いていた頃は、それだけで実機の CPU を約8%使っていた。"""

    async def test_relays_at_once_without_polling_at_1khz(self):
        import os
        import tempfile

        from raspi.bus import LATEST, Publisher, Subscriber
        from raspi.msgs.types import TOPIC_AUTO_CMD

        tmp = tempfile.TemporaryDirectory()
        old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = tmp.name
        srv = TestImmediateCmdRelay._server(self, live=True)
        srv._on_armed_change = lambda: None
        planning = Publisher("planning")
        sub = Subscriber({TOPIC_AUTO_CMD: LATEST})
        polls = []
        real_poll = sub.poll
        sub.poll = lambda timeout_ms=0: (polls.append(1), real_poll(timeout_ms))[1]
        srv.sub = sub
        pump = asyncio.create_task(srv._bus_pump())
        try:
            await asyncio.sleep(0.2)                      # 接続が張られるのを待つ
            srv._last_cmd_ns = time.monotonic_ns()
            n0 = len(polls)
            t0 = time.monotonic_ns()
            planning.send(TOPIC_AUTO_CMD, DriveCmd(mode=2, arm=True, target_speed=1.0,
                                                   source="planning:ftg"))
            while not srv.published and time.monotonic_ns() - t0 < 500_000_000:
                await asyncio.sleep(0.0005)
            waited_ms = (time.monotonic_ns() - t0) / 1e6
            self.assertTrue(srv.published, "auto/cmd が中継されていない")
            self.assertEqual(srv.published[0][1].target_speed, 1.0)
            # 時間切れ（10ms）を待たずに出ている。CI の揺れを見込んで緩めに取る
            self.assertLess(waited_ms, 8.0)
            await asyncio.sleep(0.3)
            # 何も届かない 0.3秒で覗く回数は 10ms ごと（約30回）。1kHz なら約300回
            self.assertLess(len(polls) - n0, 80)
        finally:
            srv._running = False
            await asyncio.wait_for(pump, 1.0)
            sub.close()
            planning.close()
            if old is None:
                os.environ.pop("SURGE_BUS_DIR", None)
            else:
                os.environ["SURGE_BUS_DIR"] = old
            tmp.cleanup()

    async def test_image_arrival_wakes_the_camera_pump(self):
        """画像が届いたら `BUS_IDLE_POLL_S` を待たずに `_cam_wake` が立つ（2026-10-12）。"""
        import os
        import tempfile

        from raspi.bus import LATEST, Publisher, Subscriber
        from raspi.msgs.types import TOPIC_AUTO_CMD, TOPIC_IMAGE_FRONT, ImageRef

        tmp = tempfile.TemporaryDirectory()
        old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = tmp.name
        srv = TestImmediateCmdRelay._server(self, live=True)
        srv._on_armed_change = lambda: None
        srv._cam_wake = asyncio.Event()
        camera = Publisher("camera")
        planning = Publisher("planning")
        sub = Subscriber({TOPIC_AUTO_CMD: LATEST, TOPIC_IMAGE_FRONT: LATEST})
        srv.sub = sub
        pump = asyncio.create_task(srv._bus_pump())
        try:
            await asyncio.sleep(0.2)
            waits = []
            for seq in range(1, 6):
                srv._cam_wake.clear()
                await asyncio.sleep(0.013)                # 位相をずらす
                t0 = time.monotonic_ns()
                camera.send(TOPIC_IMAGE_FRONT, ImageRef(cam="front", ring_seq=seq))
                await asyncio.wait_for(srv._cam_wake.wait(), 0.5)
                waits.append((time.monotonic_ns() - t0) / 1e6)
            self.assertLess(sorted(waits)[len(waits) // 2], 5.0, waits)
        finally:
            srv._running = False
            await asyncio.wait_for(pump, 1.0)
            sub.close()
            camera.close()
            planning.close()
            if old is None:
                os.environ.pop("SURGE_BUS_DIR", None)
            else:
                os.environ["SURGE_BUS_DIR"] = old
            tmp.cleanup()

    async def test_telemetry_goes_out_when_vehicle_state_arrives(self):
        """`/ws/telemetry` は `vehicle_state` が届いた周に出る（2026-10-13）。刻みで送っていた頃は
        届いてから送るまでに平均 7.5〜9.4ms・最大 20ms 待っていた（実機）。100Hz の2回に1回を送る。"""
        import os
        import tempfile

        import msgspec

        from raspi.bus import LATEST, Publisher, Subscriber
        from raspi.msgs.types import TOPIC_AUTO_CMD, TOPIC_VEHICLE_STATE, VehicleState

        tmp = tempfile.TemporaryDirectory()
        old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = tmp.name
        srv = TestImmediateCmdRelay._server(self, live=True)
        srv._on_armed_change = lambda: None
        srv._tel_wake = asyncio.Event()
        srv._snapshot_sent = {}
        srv.frames_sent = 0
        sent = []

        class _Ws:
            async def send(self, payload):
                sent.append((time.monotonic_ns(), msgspec.msgpack.decode(payload)))

        srv.telemetry_clients = {_Ws()}
        io = Publisher("io")
        planning = Publisher("planning")
        sub = Subscriber({TOPIC_AUTO_CMD: LATEST, TOPIC_VEHICLE_STATE: LATEST})
        srv.sub = sub
        pumps = [asyncio.create_task(srv._bus_pump()),
                 asyncio.create_task(srv._telemetry_pump())]
        try:
            await asyncio.sleep(0.2)
            sent.clear()
            for i in range(40):                           # 100Hz で 0.4 秒
                io.send(TOPIC_VEHICLE_STATE, VehicleState(speed=float(i)))
                await asyncio.sleep(0.01)
            await asyncio.sleep(0.02)
            # 2回に1回（約50Hz）。CI の揺れで sleep が延びるぶんを見込む
            self.assertGreaterEqual(len(sent), 12)
            self.assertLessEqual(len(sent), 24)
            # 載っている `vehicle_state` は届いたばかりのもの（実機で約 0.5ms。Mac の ipc は数ms 掛かる）
            # （最初の `vehicle_state` が届く前に周期で出たフレームには載っていない）
            ages = sorted((t - m["vs"]["t_pub"]) / 1e6 for t, m in sent if m["vs"])
            self.assertLess(ages[len(ages) // 2], 6.0, ages)
        finally:
            srv._running = False
            await asyncio.wait_for(asyncio.gather(*pumps), 1.0)
            sub.close()
            io.close()
            planning.close()
            if old is None:
                os.environ.pop("SURGE_BUS_DIR", None)
            else:
                os.environ["SURGE_BUS_DIR"] = old
            tmp.cleanup()

    async def test_telemetry_still_ticks_without_vehicle_state(self):
        """`vehicle_state` が来なくても（io_node が居ない）、診断を出すため周期ごとには送る。"""
        srv = TestImmediateCmdRelay._server(self, live=True)
        srv._tel_wake = asyncio.Event()
        srv._snapshot_sent = {}
        srv.frames_sent = 0
        sent = []

        class _Ws:
            async def send(self, payload):
                sent.append(payload)

        srv.telemetry_clients = {_Ws()}
        pump = asyncio.create_task(srv._telemetry_pump())
        await asyncio.sleep(0.25)
        srv._running = False
        await asyncio.wait_for(pump, 1.0)
        self.assertGreaterEqual(len(sent), 6)             # 50Hz × 0.25s ≒ 12
        self.assertLessEqual(len(sent), 14)

    async def test_fake_sub_without_sockets_still_polls(self):
        srv = TestImmediateCmdRelay._server(self, live=True)
        srv._on_armed_change = lambda: None
        srv.sub.poll = lambda timeout_ms=0: []
        pump = asyncio.create_task(srv._bus_pump())
        TestImmediateCmdRelay._auto(self, srv, 1.0)
        await asyncio.sleep(0.05)
        srv._running = False
        await asyncio.wait_for(pump, 1.0)
        self.assertEqual(len(srv.published), 1)


class TestAutoAccelLimit(unittest.IsolatedAsyncioTestCase):
    """自律中の加速度の上限は GUI の値と planner の値の**小さい方**（planner は締めるだけ、
    2026-09-27）。"""

    async def test_planner_can_only_tighten(self):
        srv = TestImmediateCmdRelay._server(self, live=True)       # GUI の accel_limit は 6.0
        from raspi.msgs.types import TOPIC_AUTO_CMD
        for auto_lim, want in ((1.6, 1.6), (0.0, 6.0), (8.0, 6.0)):
            srv.sub.latest[TOPIC_AUTO_CMD] = DriveCmd(mode=2, arm=True, target_speed=1.0, accel_limit=auto_lim,
                                                      t_pub=time.monotonic_ns(), source="planning:x")
            got = srv._merge_auto(srv._last_cmd, time.monotonic_ns())
            self.assertEqual(got.accel_limit, want, auto_lim)
        # GUI が 0（指定なし＝ファームの既定）なら planner の値
        srv._last_cmd = msgspec.structs.replace(srv._last_cmd, accel_limit=0.0)
        srv.sub.latest[TOPIC_AUTO_CMD] = DriveCmd(mode=2, arm=True, target_speed=1.0, accel_limit=1.6,
                                                  t_pub=time.monotonic_ns(), source="planning:x")
        self.assertEqual(srv._merge_auto(srv._last_cmd, time.monotonic_ns()).accel_limit, 1.6)

    async def test_lights_are_ored_with_the_gui(self):
        """パッシング・ウィンカーは planner の要求と GUI の値の OR（どちらも相手を消さない）。"""
        srv = TestImmediateCmdRelay._server(self, live=True)
        from raspi.msgs.types import TOPIC_AUTO_CMD

        def merged(gui: dict, auto: dict) -> DriveCmd:
            srv._last_cmd = msgspec.structs.replace(
                srv._last_cmd, **{"passing": False, "winker_left": False, "winker_right": False, **gui})
            srv.sub.latest[TOPIC_AUTO_CMD] = DriveCmd(mode=2, arm=True, target_speed=1.0, **auto,
                                                      t_pub=time.monotonic_ns(), source="planning:x")
            return srv._merge_auto(srv._last_cmd, time.monotonic_ns())

        got = merged({}, {"passing": True, "winker_right": True})
        self.assertTrue(got.passing and got.winker_right and not got.winker_left)
        got = merged({"winker_left": True}, {"winker_right": True})
        self.assertTrue(got.winker_left and got.winker_right)           # ハザード
        got = merged({"winker_left": True}, {})
        self.assertTrue(got.winker_left and not got.passing)            # 人の操作は消さない

    async def test_launch_comes_only_from_the_planner(self):
        """ローンチコントロール（v0.20）は planner の指定だけが通る。`auto/cmd` が途絶えたら下ろす。"""
        srv = TestImmediateCmdRelay._server(self, live=True)
        from raspi.msgs.types import TOPIC_AUTO_CMD
        srv._last_cmd = msgspec.structs.replace(srv._last_cmd, launch=True)      # GUI 側の値は使わない
        srv.sub.latest[TOPIC_AUTO_CMD] = DriveCmd(mode=2, arm=True, target_speed=2.0,
                                                  t_pub=time.monotonic_ns(), source="planning:x")
        self.assertFalse(srv._merge_auto(srv._last_cmd, time.monotonic_ns()).launch)
        srv.sub.latest[TOPIC_AUTO_CMD] = DriveCmd(mode=2, arm=True, target_speed=2.0, launch=True,
                                                  t_pub=time.monotonic_ns(), source="planning:x")
        self.assertTrue(srv._merge_auto(srv._last_cmd, time.monotonic_ns()).launch)
        stale = srv._merge_auto(srv._last_cmd, time.monotonic_ns() + 10_000_000_000)
        self.assertTrue(stale.brake)
        self.assertFalse(stale.launch)


class TestAutoBrakeTorque(unittest.IsolatedAsyncioTestCase):
    """自律中の制動トルク: planner の指定があればそれ、無ければ GUI の値。人（GUI）もブレーキを
    掛けていれば強い方（0 は STM32 の最大＝一番強い）（2026-09-27）。"""

    def _merge(self, gui_brake, gui_nm, auto_brake, auto_nm):
        from raspi.msgs.types import TOPIC_AUTO_CMD
        srv = TestImmediateCmdRelay._server(self, live=True)
        srv._last_cmd = msgspec.structs.replace(srv._last_cmd, brake=gui_brake, brake_torque=gui_nm)
        srv.sub.latest[TOPIC_AUTO_CMD] = DriveCmd(mode=2, arm=True, brake=auto_brake, brake_torque=auto_nm,
                                                  t_pub=time.monotonic_ns(), source="planning:x")
        m = srv._merge_auto(srv._last_cmd, time.monotonic_ns())
        return m.brake, m.brake_torque

    async def test_rules(self):
        self.assertEqual(self._merge(False, 0.15, True, 0.04), (True, 0.04))    # planner の強さ
        self.assertEqual(self._merge(False, 0.15, True, 0.0), (True, 0.15))     # 指定なし→GUI
        self.assertEqual(self._merge(True, 0.10, True, 0.04), (True, 0.10))     # 人の方が強い
        self.assertEqual(self._merge(True, 0.0, True, 0.04), (True, 0.0))       # 人は最大
        self.assertEqual(self._merge(True, 0.05, False, 0.0), (True, 0.05))     # 人だけ


class TestImmediateHumanRelay(unittest.IsolatedAsyncioTestCase):
    """人の操作（E-Stop・操縦権の解放・GUI の指令の変化）も、50Hz の定期送信を待たずに `/cmd`
    へ出す（2026-09-27）。判断は定期送信と同じ `_publish_cmd` を通す。"""

    def _server(self):
        srv = TestWsDeadman._server(self)
        srv._auto_engaged = False
        srv._auto_mode = ""
        srv._why_released = ""
        srv.bad_cmds = 0

        async def _noop(*a, **k):
            return None
        srv._broadcast_control_status = _noop
        srv._send_json = _noop
        srv.ws = object()
        srv.controller = srv.ws
        srv.controller_name = "x"
        return srv

    async def _send(self, srv, ws, **m):
        import json
        await srv._on_control(ws, json.dumps(m))

    async def test_gui_cmd_is_relayed_only_when_changed(self):
        srv = self._server()
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=0.5)
        self.assertEqual(len(srv.published), 1)
        self.assertEqual((srv.published[0][1].mode, srv.published[0][1].target_speed), (1, 0.5))
        # 同じ中身の繰り返しは即時には送らない（デッドマンの鮮度だけ更新）
        t0 = srv._last_cmd_ns
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=0.5)
        self.assertEqual(len(srv.published), 1)
        self.assertGreaterEqual(srv._last_cmd_ns, t0)
        # 人のブレーキはその場で出る
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=0.5, brake=True)
        self.assertEqual(len(srv.published), 2)
        self.assertTrue(srv.published[-1][1].brake)

    async def test_same_cmd_after_deadman_is_relayed_again(self):
        """デッドマンで DISARM を出した後に来た指令は、中身が同じでもその場で出す。"""
        from raspi.nodes.telemetry_node import CMD_DEADMAN_NS
        srv = self._server()
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=0.5)
        srv._last_cmd_ns -= CMD_DEADMAN_NS + 1
        srv._publish_cmd(time.monotonic_ns())                 # 定期送信がデッドマンを出す
        self.assertEqual(srv.published[-1][1].source, "deadman")
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=0.5)
        self.assertEqual(srv.published[-1][1].mode, 1)

    async def test_late_drive_cmds_are_dropped_but_late_stops_pass(self):
        """Wi-Fi の詰まりで溜まっていた「走れ」は、復帰してまとめて届いてもデッドマンを
        生き返らせない（2026-10-06）。止める指令は遅れていても通す。"""
        srv = self._server()
        now_ms = time.monotonic_ns() / 1e6
        off = 12345.0                                      # GUI の時計は Pi と無関係に進む
        await self._send(srv, srv.ws, type="cmd", t=now_ms - off, mode=1, arm=True, speed=0.5)
        self.assertEqual(srv.published[-1][1].mode, 1)
        srv._last_cmd_ns = 0                               # 途絶でデッドマンが働いた後
        srv._publish_cmd(time.monotonic_ns())
        n = len(srv.published)
        # 30秒前に送られた指令が今届く
        await self._send(srv, srv.ws, type="cmd", t=now_ms - off - 30_000, mode=1, arm=True,
                         speed=0.5)
        self.assertEqual(len(srv.published), n)
        self.assertEqual(srv.stale_cmds, 1)
        self.assertEqual(srv._last_cmd_ns, 0)
        # 同じだけ遅れた DISARM は通す
        await self._send(srv, srv.ws, type="cmd", t=now_ms - off - 30_000, mode=0, arm=False)
        self.assertGreater(srv._last_cmd_ns, 0)
        # 今送られた指令は通る。`t` を載せない古い GUI も通る
        await self._send(srv, srv.ws, type="cmd", t=time.monotonic_ns() / 1e6 - off, mode=1,
                         arm=True, speed=0.5)
        self.assertEqual(srv.published[-1][1].mode, 1)
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=0.7)
        self.assertEqual(srv.published[-1][1].target_speed, 0.7)
        self.assertEqual(srv.stale_cmds, 1)

    async def test_auto_rides_through_a_short_link_gap(self):
        """自律走行中は GUI の指令が `auto_link_grace_ms` 途絶えるまで DISARM しない。
        手動操縦は `cmd_deadman_ms` のまま（2026-10-06）。"""
        from raspi.nodes.telemetry_node import AUTO_LINK_GRACE_NS, CMD_DEADMAN_NS
        self.assertGreater(AUTO_LINK_GRACE_NS, CMD_DEADMAN_NS)
        srv = self._server()
        srv._merge_auto = lambda gui, now: gui             # planner 側の合成はここでは見ない
        await self._send(srv, srv.ws, type="cmd", mode=2, arm=True)
        gap = CMD_DEADMAN_NS * 4
        # 自律に入っていなければ、モードが AUTO でも従来どおり落ちる
        srv._publish_cmd(srv._last_cmd_ns + gap)
        self.assertEqual(srv.published[-1][1].source, "deadman")
        srv._auto_engaged, srv._auto_mode = True, "slam2d_route"
        ctrl = []
        srv._publish_auto_ctrl = lambda: ctrl.append(srv._auto_engaged)
        srv._publish_cmd(srv._last_cmd_ns + gap)
        self.assertEqual((srv.published[-1][1].mode, srv.published[-1][1].arm), (2, True))
        self.assertTrue(srv._auto_engaged)
        srv._publish_cmd(srv._last_cmd_ns + AUTO_LINK_GRACE_NS + 1)
        self.assertEqual(srv.published[-1][1].source, "deadman")
        # **猶予を使い切ったら engage も落とす。** 残すと、回線が戻って GUI の ARM が届いた
        # 瞬間に人の操作なしで再発進する
        self.assertFalse(srv._auto_engaged)
        self.assertEqual(ctrl, [False])
        await self._send(srv, srv.ws, type="cmd", mode=2, arm=True)
        self.assertNotIn("auto:", srv.published[-1][1].source)
        srv._auto_engaged = True
        # **人が DISARM したら猶予は使わない。** GUI は ARM を切った瞬間に DISARM を1発送って
        # 送信を止める（`ControlChannel.disarm`）。その1発でその場で止まる
        await self._send(srv, srv.ws, type="cmd", mode=2, arm=True)
        await self._send(srv, srv.ws, type="cmd", mode=0, arm=False, light_mode=1)
        self.assertEqual((srv.published[-1][1].mode, srv.published[-1][1].arm), (0, False))
        srv._publish_cmd(srv._last_cmd_ns + gap)
        self.assertEqual(srv.published[-1][1].source, "deadman")
        # 手動操縦（mode=1）は自律の設定が残っていても短いまま
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=0.5)
        srv._publish_cmd(srv._last_cmd_ns + gap)
        self.assertEqual(srv.published[-1][1].source, "deadman")

    async def test_estop_from_anyone_goes_out_at_once(self):
        srv = self._server()
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=1.0)
        await self._send(srv, object(), type="estop")        # 操縦権の無い接続からでも
        m = srv.published[-1][1]
        self.assertEqual((m.mode, m.source), (0, "deadman"))
        self.assertIsNone(srv.controller)

    async def test_release_goes_out_at_once(self):
        srv = self._server()
        await self._send(srv, srv.ws, type="cmd", mode=1, arm=True, speed=1.0)
        await self._send(srv, srv.ws, type="release_control")
        self.assertEqual(srv.published[-1][1].mode, 0)


class TestImmediateCommand(unittest.TestCase):
    """io_node は `cmd` の中身が変わったらその場で COMMAND を送る（2026-09-26）。"""

    def test_changed_cmd_goes_out_right_away_and_repeats_do_not_add_sends(self):
        class TimedLink(FakeLink):
            def send(self, packet):
                self.sent.append((time.monotonic_ns(), packet))
                return time.monotonic_ns()

        class TimedSub(FakeSub):
            """毎ループ同じ指令を流し、`t_change` 以降だけ速度を変える。"""

            def __init__(self, t_change):
                super().__init__()
                self.t_change = t_change
                self.t_seen = None

            def poll(self, timeout_ms=0):
                now = time.monotonic_ns()
                speed = 0.5
                if now >= self.t_change:
                    speed = 1.0
                    if self.t_seen is None:
                        self.t_seen = now
                return [("cmd", DriveCmd(mode=1, arm=True, target_speed=speed, source="gui"))]

        link = TimedLink()
        t0 = time.monotonic_ns()
        sub = TimedSub(t0 + 104_000_000)
        n = mark_handshaken(IoNode(link, sub=sub, allow_arm=True, max_speed=2.0))
        n.run(duration_s=0.2)
        cs = [(t, p) for t, p in link.sent if isinstance(p, packets.Command)]
        first_new = next(t for t, p in cs if p.target_speed == 1000)
        self.assertLess((first_new - sub.t_seen) / 1e6, 2.0)      # 定期送信だけなら最大10ms
        # 同じ中身の繰り返しでは送信は増えない（100Hz の定期送信＋変化の1回ぶん）
        self.assertLessEqual(len(cs), int(0.2 * 100) + 3)


class TestControlOwnership(unittest.IsolatedAsyncioTestCase):
    """操縦権は同時に1人だけ。2画面から舵を切ると再現困難な症状になる。"""

    def _server(self):
        from raspi.nodes.telemetry_node import TelemetryServer

        srv = TelemetryServer.__new__(TelemetryServer)
        srv._auto_presets = {}
        srv._save_auto_presets = lambda: None      # ディスクに触らせない
        srv.controller = None
        srv.controller_name = ""
        srv._last_cmd = None
        srv._last_cmd_ns = 0
        srv.control_clients = set()
        srv.telemetry_clients = set()
        srv.camera_clients = {"front": set(), "rear": set()}
        srv.deadman_trips = 0
        # 公開面の守り（2026-08-21 レビュー 🔴1・🟡4）。既定は**トークン無し**
        # ＝従来どおりの挙動。トークンを掛けた場合は個別のテストで差し替える
        srv.token = ""
        srv.auth_rejects = 0
        srv.origin_rejects = 0
        srv.bad_cmds = 0
        srv._jpeg_impl = None
        srv._jpeg = None                   # カメラ配信なし（JPEG の実測も出ない）
        srv.sub = type("S", (), {"latest": {}})()
        srv._sfl_active = False
        srv._mcap_proc = None
        srv._mcap_started_ns = 0
        srv._mcap_error = None
        # 自動運転（§8）。**engage していない既定の状態**を組む
        srv._auto_mode = ""
        srv._auto_engaged = False
        srv._auto_params = {}
        srv._auto_params_by_mode = {}
        srv._auto_was_fresh = True
        srv._auto_last_steer = 0.0
        srv.auto_stalls = 0
        srv._publish_auto_ctrl = lambda: None      # バスに触らせない
        srv._save_auto_conf = lambda: None         # ディスクに触らせない
        # 人の操作は `/cmd` をその場で出す（2026-09-27、`_relay_now`）。出したものは控えるだけ
        srv._cmd_was_live = False
        srv.cmds_published = 0
        srv.published = []
        srv.pub = type("P", (), {"send": lambda _s, topic, msg: srv.published.append((topic, msg))})()
        # ファン（Pi5純正クーリング）。**engage していない既定の状態**を組む
        from raspi.io.fan import FakeFan
        srv._fan = FakeFan()
        srv._fan_mode = "auto"
        srv._fan_duty = 0.5
        # Wi-Fi（SSID・電波強度）。**未接続の既定状態**を組む
        from raspi.io.wifi import FakeWifi, WifiState
        srv._wifi = FakeWifi()
        srv._wifi_state = WifiState(ssid=None, rssi_dbm=None, available=False)
        from raspi.io.pihealth import PiHealth
        srv._pi_state = PiHealth(available=False)
        srv._node_hb = {}
        # カメラ capture 設定（後方ON/OFF・前後FPS上限・GUI配信fps）。**既定値のまま**を組む。
        # ARM/DISARMそれぞれの値を持つ（2026-09-03）
        srv._cam_rear_enabled_armed = True
        srv._cam_rear_enabled_disarm = False
        srv._cam_front_fps_armed = 15.0
        srv._cam_front_fps_disarm = 15.0
        srv._cam_rear_fps_armed = 15.0
        srv.camera_hz = 15.0
        # 配信映像の魚眼補正（2026-10-03）。未校正・OFF の既定状態
        srv._cam_undistort = False
        srv._undistorters = {}
        srv._publish_cam_config = lambda: None     # バスに触らせない
        srv._save_camera_conf = lambda: None       # ディスクに触らせない
        # GUIの運転設定（速度ダイヤル・制御モード等）。**未保存の既定状態**を組む
        srv._drive_settings = {}
        srv._save_drive_settings = lambda: None    # ディスクに触らせない
        # 矢印信号認識の設定（ON/OFF・HSVしきい値）。**既定値のまま**を組む
        srv._signal_enabled = True
        srv._signal_roi_top = 0.05
        srv._signal_roi_bottom = 0.55
        srv._signal_sat_min = 40
        srv._signal_sat_max = 180
        srv._signal_val_min = 150
        srv._signal_min_lit_frac = 0.02
        srv._publish_signal_config = lambda: None  # バスに触らせない
        srv._save_signal_conf = lambda: None       # ディスクに触らせない
        # カメラセグメンテーションモデルの選択（`ftg_cam` 用）。**未選択の既定状態**を組む
        srv._cam_model = ""
        srv._publish_cam_model = lambda: None      # バスに触らせない
        srv._save_cam_model_conf = lambda: None    # ディスクに触らせない
        # E2E LiDARモデルの選択（`e2e_lidar` 用）。`_cam_model` と同じ形
        srv._e2e_model = ""
        srv._publish_e2e_model = lambda: None      # バスに触らせない
        srv._save_e2e_model_conf = lambda: None    # ディスクに触らせない
        # カメラE2Eモデルの選択（`cam_e2e` 用）。同じ形
        srv._cam_e2e_model = ""
        srv._publish_cam_e2e_model = lambda: None  # バスに触らせない
        srv._save_cam_e2e_model_conf = lambda: None  # ディスクに触らせない
        srv.sent = []

        async def _send_json(ws, obj):
            srv.sent.append((ws, obj))
        srv._send_json = _send_json
        return srv

    async def test_second_client_is_denied(self):
        srv = self._server()
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        self.assertIs(srv.controller, a)
        await srv._on_control(b, b'{"type":"take_control","name":"B"}')
        self.assertIs(srv.controller, a)
        self.assertTrue(any(o.get("type") == "control_denied"
                            for _, o in srv.sent))

    async def test_cmd_from_a_non_controller_is_dropped(self):
        srv = self._server()
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(b, b'{"type":"cmd","mode":1,"speed":5.0}')
        self.assertIsNone(srv._last_cmd)
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.5}')
        self.assertEqual(srv._last_cmd.target_speed, 0.5)

    async def test_anyone_can_estop(self):
        """止めるほうは権限を問わない。**止める操作に条件をつけない。**"""
        srv = self._server()
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.5}')
        await srv._on_control(b, b'{"type":"estop"}')
        self.assertIsNone(srv.controller)

    async def test_torque_mode_and_target_torque_reach_drive_cmd(self):
        """v0.6: WS の cmd JSON から `torque_mode`/`target_torque` が
        DriveCmd まで届く。**フィールド追加を _on_control 側に反映し忘れると
        GUI が送っても静かに握りつぶされる**（実際にこの取りこぼしがあった）。"""
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(
            a, b'{"type":"cmd","mode":1,"speed":0.0,"torque_mode":true,"target_torque":0.05}')
        self.assertTrue(srv._last_cmd.torque_mode)
        self.assertEqual(srv._last_cmd.target_torque, 0.05)

    async def test_torque_mode_defaults_false_for_old_gui(self):
        """古い GUI がこのキーを送ってこなくても速度指令のまま動く（安全側の既定）。"""
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.5}')
        self.assertFalse(srv._last_cmd.torque_mode)
        self.assertEqual(srv._last_cmd.target_torque, 0.0)

    async def test_auto_stop_reaches_drive_cmd(self):
        """v0.7: WS の cmd JSON の `auto_stop` が DriveCmd まで届く。"""
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.5,"auto_stop":true}')
        self.assertTrue(srv._last_cmd.auto_stop)

    async def test_auto_stop_defaults_false_for_old_gui(self):
        """キーを送ってこない GUI に**勝手な自動介入を足さない**（既定 False）。"""
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.5}')
        self.assertFalse(srv._last_cmd.auto_stop)

    async def test_side_brake_and_winkers_reach_drive_cmd(self):
        """v0.13/v0.14: WS の cmd JSON の `side_brake`/`winker_left`/`winker_right`
        が DriveCmd まで届く。**`_decode_cmd` へのフィールド追加を忘れると、
        GUI 側は「要求中」と表示されるのに実際には一切 STM32 へ届かない**
        （`torque_mode` と同種の取りこぼしが `side_brake` で実際に発生し、
        2026-08-30 に実車確認で発覚した）。"""
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(
            a, b'{"type":"cmd","mode":1,"speed":0.0,'
               b'"side_brake":true,"winker_left":true,"winker_right":true}')
        self.assertTrue(srv._last_cmd.side_brake)
        self.assertTrue(srv._last_cmd.winker_left)
        self.assertTrue(srv._last_cmd.winker_right)

    async def test_launch_reaches_drive_cmd_and_defaults_false(self):
        """v0.20: 手動操作のローンチコントロール。WS の cmd JSON の `launch` が `DriveCmd` に届き、
        キーを送らない古い GUI では False のまま。"""
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":2.0,"launch":true}')
        self.assertTrue(srv._last_cmd.launch)
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":2.0}')
        self.assertFalse(srv._last_cmd.launch)

    async def test_side_brake_and_winkers_default_false_for_old_gui(self):
        """キーを送ってこない GUI で誤って有効化しない（既定 False）。"""
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.5}')
        self.assertFalse(srv._last_cmd.side_brake)
        self.assertFalse(srv._last_cmd.winker_left)
        self.assertFalse(srv._last_cmd.winker_right)


    # ── 自律走行の開始には操縦権が要る（レビュー 🔴1-(c)） ──

    async def test_auto_engage_requires_control(self):
        """**未認証の第三者が自律走行を開始できてはいけない。**

        `auto` は「解除は誰でもできるべき」という意図で無条件に通していたが、
        `engage` も同じ経路だったため開始まで誰でもできてしまっていた。
        """
        srv = self._server()
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"auto","mode":"ftg"}')
        # 操縦権を持たない b は engage できない
        await srv._on_control(b, b'{"type":"auto","engaged":true}')
        self.assertFalse(srv._auto_engaged)
        self.assertTrue(any(o.get("reason") == "auto_engage" for _, o in srv.sent))
        # 操縦権を持つ a は engage できる
        await srv._on_control(a, b'{"type":"auto","engaged":true}')
        self.assertTrue(srv._auto_engaged)

    async def test_anyone_can_disengage_auto(self):
        """**止める側には条件をつけない。** 解除は操縦権が無くても通す。"""
        srv = self._server()
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"auto","mode":"ftg"}')
        await srv._on_control(a, b'{"type":"auto","engaged":true}')
        self.assertTrue(srv._auto_engaged)
        await srv._on_control(b, b'{"type":"auto","engaged":false}')
        self.assertFalse(srv._auto_engaged)

    # ── 共有トークン（レビュー 🔴1-(a)） ──

    async def test_take_control_needs_the_token_when_set(self):
        """トークンを設定したら、合っていない `take_control` は通らない。"""
        srv = self._server()
        srv.token = "himitsu"
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        self.assertIsNone(srv.controller)
        await srv._on_control(a, b'{"type":"take_control","name":"A","token":"chigau"}')
        self.assertIsNone(srv.controller)
        self.assertEqual(srv.auth_rejects, 2)
        self.assertTrue(any(o.get("reason") == "bad_token" for _, o in srv.sent))
        await srv._on_control(b, b'{"type":"take_control","name":"B","token":"himitsu"}')
        self.assertIs(srv.controller, b)

    async def test_estop_works_without_the_token(self):
        """**止める操作にトークンを要求しない。**

        止める指令に条件を足すと、その条件が満たせない状況
        （＝一番止めたい状況）で止まらなくなる。
        """
        srv = self._server()
        srv.token = "himitsu"
        a, b = object(), object()
        await srv._on_control(a, b'{"type":"take_control","name":"A","token":"himitsu"}')
        self.assertIs(srv.controller, a)
        await srv._on_control(b, b'{"type":"estop"}')
        self.assertIsNone(srv.controller)

    # ── 壊れた cmd で接続を切らない（レビュー 🟡4） ──

    async def test_a_malformed_cmd_is_dropped_without_killing_the_channel(self):
        """`{"speed":"abc"}` は msgspec を素通りし `float()` で例外になる。

        伝播すると制御チャネルのタスクごと死に、**GUI のバグ1つで走行中に
        操縦が切れる**。捨てるだけにして接続は維持する。
        """
        srv = self._server()
        a = object()
        await srv._on_control(a, b'{"type":"take_control","name":"A"}')
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.5}')
        good = srv._last_cmd
        # 例外が外に漏れないこと（漏れれば await がここで落ちる）
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":"abc"}')
        await srv._on_control(a, b'{"type":"cmd","mode":"x","speed":0.5}')
        self.assertEqual(srv.bad_cmds, 2)
        # 直前の正常な指令は残っており、操縦権も生きている
        self.assertIs(srv._last_cmd, good)
        self.assertIs(srv.controller, a)
        # 次の正常な指令はそのまま通る
        await srv._on_control(a, b'{"type":"cmd","mode":1,"speed":0.25}')
        self.assertEqual(srv._last_cmd.target_speed, 0.25)


class TestOriginCheck(unittest.TestCase):
    """クロスサイト WebSocket ハイジャック対策（レビュー 🔴1-(b)）。

    WebSocket は同一オリジンポリシーの対象外なので、**サーバが `Origin` を
    見なければ**別サイトのページから `/ws/control` を張れてしまう。
    """

    @staticmethod
    def _req(origin, host="surge.local:8000"):
        headers = {"Host": host}
        if origin is not None:
            headers["Origin"] = origin
        return type("R", (), {"headers": headers, "path": "/ws/control"})()

    def _ok(self, origin, host="surge.local:8000"):
        from raspi.nodes.telemetry_node import TelemetryServer
        return TelemetryServer._origin_ok(self._req(origin, host))

    def test_same_origin_passes(self):
        self.assertTrue(self._ok("http://surge.local:8000"))
        self.assertTrue(self._ok("https://surge.local:8000"))

    def test_ip_address_access_passes(self):
        """**IP 直打ちでも通ること。** オリジンを定数で列挙すると必ず取りこぼす。"""
        self.assertTrue(self._ok("http://192.168.1.20:8000", host="192.168.1.20:8000"))

    def test_vite_dev_server_passes(self):
        """開発中は 5173 の Vite から Pi へ中継する（`gui/vite.config.ts`）。"""
        self.assertTrue(self._ok("http://localhost:5173"))

    def test_a_foreign_site_is_rejected(self):
        self.assertFalse(self._ok("http://evil.example"))
        self.assertFalse(self._ok("http://surge.local.evil.example:8000"))
        # ポートだけ違うのも別オリジン
        self.assertFalse(self._ok("http://surge.local:9999"))

    def test_no_origin_passes(self):
        """`Origin` 無し = ブラウザ以外（curl・`tools/`）。

        クロスサイトの話ではないのでこの検査の対象外。同一ネットワークの
        第三者を止めるのは Origin ではなくトークン。
        """
        self.assertTrue(self._ok(None))


class TestIoNodeRearmAfterStop(unittest.TestCase):
    """E-Stop・駆動電源ラッチ・STM32 の再起動の後は、人が ARM を入れ直すまで通さない。

    STM32 は E-Stop を解除した次の周期から指令をそのまま実行する。ARM 付きの指令が流れ続けて
    いると、車のそばでボタン2を押した瞬間に走り出す。
    """

    ARMED = dict(mode=1, arm=True, target_speed=1.0, source="gui")

    def setUp(self):
        self.link = FakeLink()
        self.sub = FakeSub()
        self.node = mark_handshaken(IoNode(self.link, sub=self.sub, allow_arm=True))
        self.node.state.health = "OK"
        self.node._control_synced = True          # `run()` が [control] の同期を確かめた後の状態

    def packet(self, **cmd) -> packets.Command:
        self.sub.push(DriveCmd(**cmd))
        self.node._recv_cmd(time.monotonic_ns())
        return self.node._command_packet()

    def assert_armed(self, pkt):
        self.assertTrue(pkt.flags & packets.CMD_FLG_ARM)
        self.assertEqual(pkt.mode, packets.Mode.MANUAL)

    def assert_disarm(self, pkt):
        self.assertEqual((pkt.mode, pkt.flags, pkt.target_speed), (packets.Mode.DISARM, 0, 0))

    def test_arm_is_blocked_while_the_estop_is_latched(self):
        self.assert_armed(self.packet(**self.ARMED))
        self.node.state.estop_active = True
        self.assert_disarm(self.packet(**self.ARMED))
        self.assertIn("E-Stop", self.node._arm_block_reason())

    def test_releasing_the_estop_does_not_resume_a_held_arm(self):
        self.node.state.estop_active = True
        self.assert_disarm(self.packet(**self.ARMED))
        self.node.state.estop_active = False
        for _ in range(3):
            self.assert_disarm(self.packet(**self.ARMED))
        self.assertIn("入れ直", self.node._arm_block_reason())

    def test_dropping_arm_once_after_the_release_allows_arming_again(self):
        self.node.state.estop_active = True
        self.packet(**self.ARMED)
        self.node.state.estop_active = False
        self.assert_disarm(self.packet(mode=0, arm=False, source="gui"))
        self.assertEqual(self.node._arm_block_reason(), "")
        self.assert_armed(self.packet(**self.ARMED))

    def test_an_arm_dropped_during_the_estop_does_not_count(self):
        """E-Stop 中に外して入れ直した ARM は、解除の瞬間に走り出すのと同じ。"""
        self.node.state.estop_active = True
        self.packet(mode=0, arm=False, source="gui")
        self.packet(**self.ARMED)
        self.node.state.estop_active = False
        self.assert_disarm(self.packet(**self.ARMED))

    def test_a_stale_cmd_does_not_count_as_dropping_arm(self):
        self.node.state.estop_active = True
        self.packet(**self.ARMED)
        self.node.state.estop_active = False
        self.node._recv_cmd(time.monotonic_ns() + CMD_TIMEOUT_NS + 1)     # 途絶
        self.assertTrue(self.node.cmd_stale)
        self.assertTrue(self.node._arm_block_reason())
        self.assert_disarm(self.packet(**self.ARMED))

    def test_drive_power_lock_behaves_the_same(self):
        self.node.state.drive_power_locked = True
        self.assert_disarm(self.packet(**self.ARMED))
        self.assertIn("駆動電源", self.node._arm_block_reason())
        self.node.state.drive_power_locked = False
        self.assert_disarm(self.packet(**self.ARMED))
        self.packet(mode=0, arm=False, source="gui")
        self.assert_armed(self.packet(**self.ARMED))

    def test_an_stm32_restart_needs_arm_again(self):
        self.assert_armed(self.packet(**self.ARMED))
        self.node._on_time_reset(time.monotonic_ns())
        self.node._control_synced = True          # [control] は入り直したとして、ラッチだけを見る
        self.assert_disarm(self.packet(**self.ARMED))
        self.assertIn("再起動", self.node._arm_block_reason())
        self.packet(mode=0, arm=False, source="gui")
        self.assert_armed(self.packet(**self.ARMED))

    def test_an_stm32_restart_asks_for_the_settings_again(self):
        """STM32 は ON/OFF・自動停止のマージンを既定値へ戻す。前の値を出し続けない。"""
        self.node.state.auto_stop_margin_cm = 30.0
        self.node.state.brake_hold_enabled = False
        self.link.sent.clear()
        self.node._on_time_reset(time.monotonic_ns())
        self.assertIsNone(self.node.state.auto_stop_margin_cm)
        self.assertIsNone(self.node.state.brake_hold_enabled)
        asked = {p.param_id for p in self.link.sent if isinstance(p, packets.ConfigGet)}
        self.assertLessEqual({packets.Param.AUTO_STOP_MARGIN_CM, packets.Param.BRAKE_HOLD_ENABLE}, asked)

    def test_the_stm32_refusing_arm_makes_us_ask_for_it_again(self):
        """STM32 が ARM を受け付けなかった（`LOG` "ARM ignored"）なら、こちらも入れ直させる。"""
        self.assert_armed(self.packet(**self.ARMED))
        self.node._on_stm_log(packets.Log(
            severity=packets.LogSeverity.WARN,
            message=b"ARM ignored: disarm once to re-arm (after e-stop / COMMAND loss)"))
        self.assert_disarm(self.packet(**self.ARMED))
        self.assertIn("通信途絶から復帰", self.node._arm_block_reason())
        self.assertIn("ARM ignored", self.node.stm_log[0])      # diag/link にも出る
        self.assert_disarm(self.packet(mode=0, arm=False, source="gui"))
        self.assert_armed(self.packet(**self.ARMED))

    def test_new_firmware_warnings_reach_the_diag(self):
        """ファームの WARN/ERROR は `stm_log`（`diag/link`）に載る。ARM の判断は変えない。"""
        for sev, text in ((packets.LogSeverity.WARN,
                           b"armed without heartbeat (RAS_SIG never seen): e-stop is inactive"),
                          (packets.LogSeverity.ERROR,
                           b"restarted by watchdog reset (main loop had stopped)")):
            self.node._on_stm_log(packets.Log(severity=sev, message=text))
            self.assertIn(text.decode(), self.node.stm_log[0])
        self.assertEqual(self.node._arm_block_reason(), "")

    def test_after_an_stm32_restart_arm_bit_zero_is_on_the_wire(self):
        """再起動を検出した後、STM32 に実際に届く COMMAND の ARM ビットが 0 であること。"""
        self.packet(**self.ARMED)
        self.node._on_time_reset(time.monotonic_ns())          # `_control_synced` も落ちる
        self.link.sent.clear()
        self.sub.push(DriveCmd(**self.ARMED))
        self.node._recv_cmd(time.monotonic_ns())
        self.node._send_command()
        cs = commands(self.link)
        self.assertTrue(cs)
        self.assertTrue(all(not (c.flags & packets.CMD_FLG_ARM) for c in cs))

    def test_the_cmd_deadman_does_not_latch(self):
        """`cmd` の途絶は従来どおり: 指令が戻ればそのまま通る。"""
        self.assert_armed(self.packet(**self.ARMED))
        self.node._recv_cmd(time.monotonic_ns() + CMD_TIMEOUT_NS + 1)
        self.assertEqual(self.node._arm_block_reason(), "")
        self.assert_armed(self.packet(**self.ARMED))


if __name__ == "__main__":
    unittest.main()


class TestIoWaitsOnLinkAndBus(unittest.TestCase):
    """io_node は UART の受信とバスを同じ poller で待つ（2026-09-27）。以前は UART の1バイトを
    `read_timeout`（5ms）待ってからでないと `/cmd` を読まず、即時の中継がそのぶん止まった。"""

    def setUp(self):
        import os
        import socket
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self._old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = self._tmp.name
        self.a, self.b = socket.socketpair()

    def tearDown(self):
        import os
        self.a.close()
        self.b.close()
        if self._old is None:
            os.environ.pop("SURGE_BUS_DIR", None)
        else:
            os.environ["SURGE_BUS_DIR"] = self._old
        self._tmp.cleanup()

    def _node(self):
        from raspi.bus import Publisher, Subscriber
        from raspi.msgs.types import TOPIC_CMD
        from raspi.nodes import io_node

        sock = self.a

        class FdLink(FakeLink):
            def fileno(self):
                return sock.fileno()

            def poll(self, block=True):
                sock.setblocking(False)
                try:
                    return [b"x"] if sock.recv(4096) else []
                except BlockingIOError:
                    return []

        self.pub = Publisher("control")
        self.sub = Subscriber([TOPIC_CMD])
        node = io_node.IoNode(FdLink(), sub=self.sub)
        node._poller = node._make_poller()
        self.assertIsNotNone(node._poller)
        # slow joiner: 購読が繋がるまで流す
        end = time.monotonic() + 2.0
        while not self.sub.poll(20) and time.monotonic() < end:
            self.pub.send(TOPIC_CMD, DriveCmd(mode=0, source="warmup"))
        return node

    def _wait(self, node, trigger):
        import threading
        from raspi.nodes import io_node
        th = threading.Timer(0.02, trigger)
        th.start()
        t0 = time.monotonic()
        old = io_node.LINK_WAIT_S
        io_node.LINK_WAIT_S = 1.0          # 上限では起きない長さにして、何で起きたかを見る
        try:
            got = node._poll_link(time.monotonic_ns(), time.monotonic_ns() + 10**9)
        finally:
            io_node.LINK_WAIT_S = old
            th.join()
        return time.monotonic() - t0, got

    def test_bus_message_wakes_the_loop(self):
        from raspi.msgs.types import TOPIC_CMD
        node = self._node()
        try:
            dt, _ = self._wait(node, lambda: self.pub.send(TOPIC_CMD, DriveCmd(mode=1, source="gui")))
            self.assertLess(dt, 0.3)
        finally:
            self.sub.close()
            self.pub.close()

    def test_uart_bytes_wake_the_loop_and_are_read(self):
        node = self._node()
        try:
            dt, got = self._wait(node, lambda: self.b.send(b"\xaa\x55"))
            self.assertLess(dt, 0.3)
            self.assertTrue(got)
        finally:
            self.sub.close()
            self.pub.close()

    def _partial_node(self):
        """`"|"` が届くまではフレームが揃わない偽の UART（本物は十数バイトずつ届く）。"""
        node = self._node()
        sock = self.a
        buf = bytearray()

        def poll(block=True):
            sock.setblocking(False)
            try:
                buf.extend(sock.recv(4096))
            except BlockingIOError:
                pass
            if b"|" not in buf:
                return []
            buf.clear()
            return [b"frame"]

        node.link.poll = poll
        return node

    def test_partial_uart_bytes_do_not_wake_the_loop(self):
        """フレームの途中までしか届いていない間は、ループへ返さず残りを待つ（2026-10-13）。
        起こされるたびに1周回していた頃は、実機で毎秒約620周のうち6割が空振りだった。"""
        import threading
        node = self._partial_node()
        try:
            threading.Timer(0.06, lambda: self.b.send(b"cd|")).start()
            dt, got = self._wait(node, lambda: self.b.send(b"ab"))
            self.assertEqual(got, [b"frame"])
            self.assertGreater(dt, 0.05)       # 途中（20ms 後の "ab"）では返っていない
            self.assertLess(dt, 0.3)
        finally:
            self.sub.close()
            self.pub.close()

    def test_bus_message_still_wakes_while_a_frame_is_partial(self):
        """フレームの残りを待っている間も、`cmd` が届けばすぐ返す（指令の即時中継を遅らせない）。"""
        import threading
        from raspi.msgs.types import TOPIC_CMD
        node = self._partial_node()
        try:
            threading.Timer(0.06, lambda: self.pub.send(
                TOPIC_CMD, DriveCmd(mode=1, source="gui"))).start()
            dt, got = self._wait(node, lambda: self.b.send(b"ab"))
            self.assertEqual(got, [])
            self.assertGreater(dt, 0.05)
            self.assertLess(dt, 0.3)
        finally:
            self.sub.close()
            self.pub.close()

    def test_partial_frame_wait_ends_at_the_deadline(self):
        """残りが来なくても、次の定期送信の時刻（とハートビートの申告）までには必ず返す。"""
        node = self._partial_node()
        try:
            self.b.send(b"ab")
            t0 = time.monotonic()
            got = node._poll_link(time.monotonic_ns(), time.monotonic_ns() + 30_000_000)
            dt = time.monotonic() - t0
            self.assertEqual(got, [])
            self.assertLess(dt, 0.02)          # `LINK_WAIT_S`（5ms）で返る
        finally:
            self.sub.close()
            self.pub.close()
