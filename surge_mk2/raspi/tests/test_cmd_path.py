"""GUI → WS → バス → io_node → UART の指令経路のテスト。

**この経路が「意図せず開いている」ことが一番危ない。** 通信のテストではなく
安全条件のテストとして書く。ハードウェアもシリアルも要らない。
"""

import asyncio
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.io.serial_link import RxFrame  # noqa: E402
from raspi.msgs import DriveCmd  # noqa: E402
from raspi.nodes.io_node import CMD_TIMEOUT_NS, IoNode  # noqa: E402
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


class TestIoNodeArmGate(unittest.TestCase):
    """モータが回るのに必要な3条件。**どれか1つでも欠けたら DISARM。**"""

    def setUp(self):
        self.link = FakeLink()
        self.sub = FakeSub()

    def node(self, **kw) -> IoNode:
        return IoNode(self.link, sub=self.sub, **kw)

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


class TestWsDeadman(unittest.IsolatedAsyncioTestCase):
    """WS 層のデッドマン。**誰も操縦していなければ明示的な DISARM を流し続ける。**"""

    def _server(self):
        from raspi.nodes.telemetry_node import TelemetryServer

        srv = TelemetryServer.__new__(TelemetryServer)   # バスを開かずに中身だけ
        srv._last_cmd = None
        srv._last_cmd_ns = 0
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
        srv.sub = type("S", (), {"latest": {}})()
        srv._sfl_active = False
        srv._mcap_proc = None
        srv._mcap_started_ns = 0
        srv._mcap_error = None

        class _P:
            def send(self, topic, msg):
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


class TestControlOwnership(unittest.IsolatedAsyncioTestCase):
    """操縦権は同時に1人だけ。2画面から舵を切ると再現困難な症状になる。"""

    def _server(self):
        from raspi.nodes.telemetry_node import TelemetryServer

        srv = TelemetryServer.__new__(TelemetryServer)
        srv.controller = None
        srv.controller_name = ""
        srv._last_cmd = None
        srv._last_cmd_ns = 0
        srv.control_clients = set()
        srv.telemetry_clients = set()
        srv.camera_clients = {"front": set(), "rear": set()}
        srv.deadman_trips = 0
        srv._jpeg_impl = None
        srv.sub = type("S", (), {"latest": {}})()
        srv._sfl_active = False
        srv._mcap_proc = None
        srv._mcap_started_ns = 0
        srv._mcap_error = None
        # 自動運転（§8）。**engage していない既定の状態**を組む
        srv._auto_mode = ""
        srv._auto_engaged = False
        srv._auto_params = {}
        srv._auto_was_fresh = True
        srv.auto_stalls = 0
        srv._publish_auto_ctrl = lambda: None      # バスに触らせない
        srv._save_auto_conf = lambda: None         # ディスクに触らせない
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


if __name__ == "__main__":
    unittest.main()
