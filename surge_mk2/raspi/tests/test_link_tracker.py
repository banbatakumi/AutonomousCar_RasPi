"""LinkTracker の単体テスト（io_node と replay_node が共有する中核）。

ここがズレると実機と再生の結果が食い違う。境界値まで押さえておく。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.link_tracker import (  # noqa: E402
    TELEM_DEGRADED_NS,
    TELEM_FAULT_NS,
    LinkTracker,
    format_status,
)
from raspi.core.link_tracker import _MAX_PENDING_PINGS  # noqa: E402
from raspi.proto import packets  # noqa: E402

MS = 1_000_000


def feed(tr: LinkTracker, pkt, t_ns: int, seq: int = 0):
    return tr.feed(t_ns, pkt.TYPE, seq, pkt.encode())


class TestDispatch(unittest.TestCase):
    def test_telemetry_updates_state(self):
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=1000, speed=1234), 500)
        self.assertEqual(tr.state.telemetry.speed, 1234)
        self.assertEqual(tr.state.last_telem_ns, 500)
        self.assertEqual(tr.state.counts[packets.Telemetry.TYPE], 1)

    def test_counts_accumulate_per_type(self):
        tr = LinkTracker()
        for i in range(3):
            feed(tr, packets.Telemetry(t_us=i), i)
        feed(tr, packets.Stats(t_us=9), 10)
        self.assertEqual(tr.state.counts[packets.Telemetry.TYPE], 3)
        self.assertEqual(tr.state.counts[packets.Stats.TYPE], 1)

    def test_lidar_sectors_collected(self):
        tr = LinkTracker()
        for idx in (0, 5, 5, 11):
            feed(tr, packets.LidarSector(sector_idx=idx), 1)
        self.assertEqual(tr.state.lidar_sectors, {0, 5, 11})

    def test_version_and_stats_stored(self):
        tr = LinkTracker()
        feed(tr, packets.Version(protocol_version=4, fw_id=0xABCD), 1)
        feed(tr, packets.Stats(t_us=7, rx_frame_ok=42), 2)
        self.assertEqual(tr.state.version.fw_id, 0xABCD)
        self.assertEqual(tr.state.stats.rx_frame_ok, 42)

    def test_limits_stored(self):
        """★v0.11。受け取るまでは None、受け取ったらそのままキャッシュされる。"""
        tr = LinkTracker()
        self.assertIsNone(tr.state.limits)
        feed(tr, packets.Limits(max_speed_m_s=5.0, max_accel_m_s2=3.0,
                                 max_torque_nm=0.15, max_steer_rad=0.524), 1)
        self.assertEqual(tr.state.limits.max_speed_m_s, 5.0)
        self.assertEqual(tr.state.limits.max_accel_m_s2, 3.0)
        # f32 の丸め誤差。0.15 は f32 で正確に表現できない（0x11/0x13 の f32 と同じ事情）
        self.assertAlmostEqual(tr.state.limits.max_torque_nm, 0.15, places=6)
        self.assertAlmostEqual(tr.state.limits.max_steer_rad, 0.524, places=6)

    def test_unknown_type_counted_but_returns_none(self):
        tr = LinkTracker()
        self.assertIsNone(tr.feed(1, 0x7F, 0, b"\x00"))
        self.assertEqual(tr.state.counts[0x7F], 1)

    def test_callbacks_fire(self):
        seen_t, seen_f = [], []
        tr = LinkTracker(on_telemetry=lambda m, t: seen_t.append((m.speed, t)),
                         on_frame=lambda t, ty, s, m: seen_f.append((ty, s)))
        feed(tr, packets.Telemetry(t_us=1000, speed=77), 500, seq=3)
        feed(tr, packets.Stats(t_us=1000), 600, seq=4)
        self.assertEqual(len(seen_t), 1)
        self.assertEqual(seen_t[0][0], 77)
        self.assertEqual(seen_f, [(packets.Telemetry.TYPE, 3), (packets.Stats.TYPE, 4)])

    def test_on_frame_fires_for_unknown_type(self):
        """未知の TYPE でもフレームが来たこと自体は下流に伝わること。"""
        seen = []
        tr = LinkTracker(on_frame=lambda t, ty, s, m: seen.append((ty, m)))
        tr.feed(1, 0x7F, 0, b"")
        self.assertEqual(seen, [])   # デコードできないものは on_frame を呼ばない


class TestTimeSync(unittest.TestCase):
    def test_pong_matched_with_recorded_ping(self):
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=1_000_000), 0)   # unwrap の基準を作る
        tr.note_ping_sent(7, 1_000_000_000)
        feed(tr, packets.Pong(ping_id=7, t_ping_rx_us=1_000_500,
                              t_pong_tx_us=1_000_800), 1_001_000_000)
        self.assertEqual(tr.sync.n_samples, 1)

    def test_unmatched_pong_is_ignored(self):
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=1_000_000), 0)
        feed(tr, packets.Pong(ping_id=99, t_ping_rx_us=1_000_500,
                              t_pong_tx_us=1_000_800), 1_001_000_000)
        self.assertEqual(tr.sync.n_samples, 0)

    def test_pending_pings_are_capped(self):
        """応答の返らない PING で T1 が無限に溜まらないこと。"""
        tr = LinkTracker()
        for i in range(200):
            tr.note_ping_sent(i, i * 1000)
        self.assertLessEqual(len(tr._pending_pings), 33)

    def test_newest_pings_survive_the_cap(self):
        tr = LinkTracker()
        for i in range(200):
            tr.note_ping_sent(i, i * 1000)
        self.assertIn(199, tr._pending_pings)

    def test_eviction_is_by_insertion_order_not_by_id_value(self):
        """★ C6: `ping_id` がラップアラウンドした直後でも、破棄されるのは
        （数値最小ではなく）実際に一番古く送った PING であること。"""
        tr = LinkTracker()
        for i in range(_MAX_PENDING_PINGS):
            tr.note_ping_sent(4_294_967_295 - _MAX_PENDING_PINGS + 1 + i, i * 1000)
        # ここで最古は ping_id が最大に近い側（先に送った）。次に ID がラップして
        # 小さい値の ping_id を送ると、数値最小基準だとこの新しい ping を
        # 誤って即座に捨ててしまう
        tr.note_ping_sent(0, 999_000)
        self.assertIn(0, tr._pending_pings, "直近送った PING(id=0) が誤って破棄されている")


class TestHealth(unittest.TestCase):
    def test_init_until_first_telemetry(self):
        tr = LinkTracker()
        self.assertIsNone(tr.update_health(10**12))
        self.assertEqual(tr.state.health, "INIT")

    def test_transitions_and_thresholds(self):
        tr = LinkTracker()
        t0 = 10**12
        feed(tr, packets.Telemetry(t_us=0), t0)

        self.assertEqual(tr.update_health(t0), "OK")
        self.assertIsNone(tr.update_health(t0 + TELEM_DEGRADED_NS))       # 境界は OK のまま
        self.assertEqual(tr.update_health(t0 + TELEM_DEGRADED_NS + 1), "DEGRADED")
        self.assertIsNone(tr.update_health(t0 + TELEM_FAULT_NS))          # 境界は DEGRADED
        self.assertEqual(tr.update_health(t0 + TELEM_FAULT_NS + 1), "FAULT")

    def test_recovers_when_telemetry_returns(self):
        tr = LinkTracker()
        t0 = 10**12
        feed(tr, packets.Telemetry(t_us=0), t0)
        tr.update_health(t0 + 500 * MS)
        self.assertEqual(tr.state.health, "FAULT")
        feed(tr, packets.Telemetry(t_us=1000), t0 + 500 * MS)
        self.assertEqual(tr.update_health(t0 + 500 * MS), "OK")

    def test_returns_none_when_unchanged(self):
        tr = LinkTracker()
        t0 = 10**12
        feed(tr, packets.Telemetry(t_us=0), t0)
        self.assertEqual(tr.update_health(t0), "OK")
        self.assertIsNone(tr.update_health(t0 + 1))
        self.assertIsNone(tr.update_health(t0 + 2))


class TestLatchingFlags(unittest.TestCase):
    """E-Stop と駆動電源ラッチ。**人間が物理操作しないと戻らない**ので、
    立った瞬間を必ず捉えること。"""

    def test_estop_detected_from_flags(self):
        tr = LinkTracker()
        self.assertFalse(tr.state.estop_active)
        feed(tr, packets.Telemetry(t_us=0, flags=packets.FLG_ESTOP_ACTIVE), 1)
        self.assertTrue(tr.state.estop_active)

    def test_drive_power_lock_detected(self):
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=0, flags=packets.FLG_DRIVE_POWER_LOCKED), 1)
        self.assertTrue(tr.state.drive_power_locked)

    def test_callback_fires_on_edges_only(self):
        seen = []
        tr = LinkTracker(on_latch=lambda n, v, t: seen.append((n, v)))
        for _ in range(3):
            feed(tr, packets.Telemetry(t_us=0, flags=packets.FLG_ESTOP_ACTIVE), 1)
        feed(tr, packets.Telemetry(t_us=0, flags=0), 2)
        feed(tr, packets.Telemetry(t_us=0, flags=0), 3)
        self.assertEqual(seen, [("estop_active", True), ("estop_active", False)])

    def test_both_flags_independently(self):
        seen = []
        tr = LinkTracker(on_latch=lambda n, v, t: seen.append((n, v)))
        feed(tr, packets.Telemetry(
            t_us=0, flags=packets.FLG_ESTOP_ACTIVE | packets.FLG_DRIVE_POWER_LOCKED), 1)
        self.assertEqual(sorted(seen),
                         [("drive_power_locked", True), ("estop_active", True)])

    def test_unrelated_flags_do_not_trigger(self):
        seen = []
        tr = LinkTracker(on_latch=lambda n, v, t: seen.append((n, v)))
        feed(tr, packets.Telemetry(
            t_us=0, flags=packets.FLG_ARMED | packets.FLG_IMU_OK
            | packets.FLG_FAULT_DRIVE_UNDERVOLTAGE), 1)
        self.assertEqual(seen, [])


class TestFormatStatus(unittest.TestCase):
    def test_handles_empty_state(self):
        tr = LinkTracker()
        line = format_status(tr.state, tr.sync)
        self.assertIn("INIT", line)
        self.assertIn("TELEMETRY 未受信", line)

    def test_estop_is_shown_prominently(self):
        """E-Stop は 16進フラグに埋もれさせない。解除操作まで書く。"""
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=0, flags=packets.FLG_ESTOP_ACTIVE), 10**12)
        line = format_status(tr.state, tr.sync)
        self.assertIn("E-STOP", line)
        self.assertIn("ボタン2", line)

    def test_drive_lock_is_shown(self):
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=0, flags=packets.FLG_DRIVE_POWER_LOCKED), 10**12)
        self.assertIn("駆動電源ラッチ", format_status(tr.state, tr.sync))

    def test_no_latch_text_when_clear(self):
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=0, flags=packets.FLG_IMU_OK), 10**12)
        line = format_status(tr.state, tr.sync)
        self.assertNotIn("E-STOP", line)
        self.assertNotIn("ラッチ", line)

    def test_shows_vehicle_values(self):
        tr = LinkTracker()
        feed(tr, packets.Telemetry(t_us=0, speed=-1500, steer_actual=2000,
                                   accel_z=9810, flags=0x2580), 10**12)
        tr.update_health(10**12)
        line = format_status(tr.state, tr.sync)
        self.assertIn("v=-1.50", line)
        self.assertIn("steer=+0.200", line)
        self.assertIn("flags=0x2580", line)
        self.assertIn("OK", line)


if __name__ == "__main__":
    unittest.main()
