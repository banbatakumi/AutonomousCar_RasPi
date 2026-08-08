"""MCAP 書き出し（`rec/mcap_log.py` / `tools/sfl2mcap.py` / `nodes/logger_node.py`）のテスト。

実機もカメラも要らない。**書いた MCAP を `mcap` のリーダで読み直して**
中身を確かめる（書けたことではなく、読めることを検査する）。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import base64
import io
import json
import os
import struct
import sys
import tempfile
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.msgs import DriveCmd, Scan, VehicleState  # noqa: E402
from raspi.proto import packets  # noqa: E402
from raspi.rec import FrameLogWriter  # noqa: E402
from raspi.rec.mcap_log import (  # noqa: E402
    HAS_MCAP,
    McapLog,
    json_schema,
    pointcloud_from_scan,
)

MS = 1_000_000

requires_mcap = unittest.skipUnless(HAS_MCAP, "mcap が入っていない（pip install mcap）")


def _has_jpeg() -> bool:
    """simplejpeg / Pillow のどちらかがあるか。**Mac には既定で無い。**"""
    try:
        import numpy  # noqa: F401

        from raspi.core.jpeg import make_encoder
    except ImportError:
        return False
    return make_encoder(70)[0] is not None


def read_back(path: Path) -> dict:
    """MCAP を読み直して `{topic: [(log_time, dict), ...]}` にする。

    スキーマ名も `schemas[topic]` に入れて返す。
    """
    from mcap.reader import make_reader

    out: dict = {"msgs": {}, "schemas": {}, "metadata": {}}
    with open(path, "rb") as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            out["msgs"].setdefault(channel.topic, []).append(
                (message.log_time, json.loads(message.data)))
            out["schemas"][channel.topic] = schema.name
            assert channel.message_encoding == "json"
            assert schema.encoding == "jsonschema"
        for rec in reader.iter_metadata():
            out["metadata"][rec.name] = rec.metadata
    return out


# ── スキーマ ────────────────────────────────────────────────────────────

class TestJsonSchema(unittest.TestCase):
    def test_ref_is_flattened(self):
        """トップレベルが `$ref` のままだと Foxglove がフィールドを見つけられない。"""
        s = json_schema(VehicleState)
        self.assertNotIn("$ref", s)
        self.assertEqual(s["type"], "object")
        self.assertIn("speed", s["properties"])
        self.assertIn("odom_center", s["properties"])

    def test_every_bus_type_has_a_schema(self):
        from raspi.msgs.types import TOPIC_TYPES

        for cls in set(TOPIC_TYPES.values()):
            with self.subTest(cls=cls.__name__):
                self.assertEqual(json_schema(cls)["type"], "object")


# ── 点群 ────────────────────────────────────────────────────────────────

class TestPointCloud(unittest.TestCase):
    def _points(self, cloud) -> list[tuple[float, float, float]]:
        raw = base64.b64decode(cloud["data"])
        self.assertEqual(len(raw) % 12, 0)
        return [struct.unpack_from("<3f", raw, i) for i in range(0, len(raw), 12)]

    def test_invalid_points_are_dropped(self):
        """`dist=0` は「測れなかった」であって「原点に壁がある」ではない。"""
        scan = Scan(dist=[0.0] * 360)
        scan.dist[0] = 2.0
        scan.dist[90] = 3.0
        pts = self._points(pointcloud_from_scan(scan, 0))
        self.assertEqual(len(pts), 2)

    def test_index_is_the_vehicle_angle_ccw(self):
        """添字がそのまま度。x=前・y=左（反時計回りが正）。"""
        scan = Scan(dist=[0.0] * 360)
        scan.dist[0] = 2.0        # 真正面
        scan.dist[90] = 3.0       # 左
        (x0, y0, z0), (x90, y90, _) = self._points(pointcloud_from_scan(scan, 0))
        self.assertAlmostEqual(x0, 2.0, places=5)
        self.assertAlmostEqual(y0, 0.0, places=5)
        self.assertAlmostEqual(z0, 0.0, places=5)
        self.assertAlmostEqual(x90, 0.0, places=5)
        self.assertAlmostEqual(y90, 3.0, places=5)

    def test_stride_matches_the_fields(self):
        cloud = pointcloud_from_scan(Scan(dist=[1.0] * 360), 0)
        self.assertEqual(cloud["point_stride"], 12)
        self.assertEqual([f["name"] for f in cloud["fields"]], ["x", "y", "z"])
        self.assertEqual(len(base64.b64decode(cloud["data"])), 360 * 12)


# ── McapLog ─────────────────────────────────────────────────────────────

@requires_mcap
class TestMcapLog(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.path = Path(self._td.name) / "t.mcap"

    def tearDown(self):
        self._td.cleanup()

    def test_round_trip(self):
        with McapLog(self.path) as log:
            log.write("/vehicle_state", VehicleState(speed=1.5, t_capture=log.t0_mono_ns))
            log.write("/cmd", DriveCmd(target_speed=0.3, source="gui",
                                       t_capture=log.t0_mono_ns))
        got = read_back(self.path)
        self.assertAlmostEqual(got["msgs"]["/vehicle_state"][0][1]["speed"], 1.5)
        self.assertEqual(got["msgs"]["/cmd"][0][1]["source"], "gui")
        self.assertEqual(got["schemas"]["/vehicle_state"], "VehicleState")

    def test_log_time_is_unix_epoch(self):
        """単調時刻のまま書くと Foxglove が 1970年と表示する。"""
        t0_mono, t0_unix = 5_000_000_000, 1_800_000_000_000_000_000
        with McapLog(self.path, t0_mono_ns=t0_mono, t0_unix_ns=t0_unix) as log:
            log.write("/vehicle_state", VehicleState(), t_mono_ns=t0_mono + 2 * 10**9)
        (log_time, _), = read_back(self.path)["msgs"]["/vehicle_state"]
        self.assertEqual(log_time, t0_unix + 2 * 10**9)

    def test_message_keeps_the_monotonic_time(self):
        """**中身の `t_capture` は触らない。** `.sfl` と突き合わせられなくなる。"""
        t0_mono, t0_unix = 5_000_000_000, 1_800_000_000_000_000_000
        with McapLog(self.path, t0_mono_ns=t0_mono, t0_unix_ns=t0_unix) as log:
            log.write("/vehicle_state", VehicleState(t_capture=t0_mono + 7))
        (_, body), = read_back(self.path)["msgs"]["/vehicle_state"]
        self.assertEqual(body["t_capture"], t0_mono + 7)

    def test_conversion_base_is_recorded(self):
        """後から単調時刻に戻せること。"""
        with McapLog(self.path, t0_mono_ns=11, t0_unix_ns=22) as log:
            log.write("/cmd", DriveCmd())
        meta = read_back(self.path)["metadata"]["surge"]
        self.assertEqual(meta["t0_mono_ns"], "11")
        self.assertEqual(meta["t0_unix_ns"], "22")

    def test_t_capture_is_preferred_over_t_pub(self):
        """センサが測った時刻で並べる（publish 時刻だと UART の往復ぶんずれる）。"""
        with McapLog(self.path, t0_mono_ns=0, t0_unix_ns=0) as log:
            log.write("/vehicle_state", VehicleState(t_capture=100, t_pub=900))
        (log_time, _), = read_back(self.path)["msgs"]["/vehicle_state"]
        self.assertEqual(log_time, 100)

    def test_viz_topics_use_foxglove_schema_names(self):
        """名前が違うと Foxglove が 3D パネルで描いてくれない。"""
        with McapLog(self.path, t0_mono_ns=0, t0_unix_ns=0) as log:
            log.write_viz_scan(Scan(dist=[1.0] * 360, t_capture=1))
            log.write_viz_image(b"\xff\xd8\xff\xd9", "front", 2)
        s = read_back(self.path)["schemas"]
        self.assertEqual(s["/viz/scan"], "foxglove.PointCloud")
        self.assertEqual(s["/viz/image/front"], "foxglove.CompressedImage")

    def test_image_payload_survives(self):
        blob = bytes(range(256)) * 4
        with McapLog(self.path, t0_mono_ns=0, t0_unix_ns=0) as log:
            log.write_viz_image(blob, "rear", 0)
        (_, body), = read_back(self.path)["msgs"]["/viz/image/rear"]
        self.assertEqual(base64.b64decode(body["data"]), blob)
        self.assertEqual(body["format"], "jpeg")

    def test_counts_are_tracked(self):
        with McapLog(self.path) as log:
            for _ in range(5):
                log.write("/cmd", DriveCmd())
        self.assertEqual(log.counts["/cmd"], 5)
        self.assertEqual(log.written, 5)

    def test_write_after_close_is_refused(self):
        log = McapLog(self.path)
        log.close()
        with self.assertRaises(ValueError):
            log.write("/cmd", DriveCmd())

    def test_close_is_idempotent(self):
        log = McapLog(self.path)
        log.close()
        log.close()

    def test_unknown_compression_is_refused(self):
        with self.assertRaises(ValueError):
            McapLog(self.path, compression="gzip")

    def test_stdout_mode_produces_a_valid_file(self):
        """**SD カードを削らないための経路。** ssh 越しに PC へ流す。"""
        import sys as _sys

        class FakeStdout:
            def __init__(self):
                self.buffer = io.BytesIO()

        real, _sys.stdout = _sys.stdout, FakeStdout()
        try:
            with McapLog("-", t0_mono_ns=0, t0_unix_ns=0) as log:
                self.assertTrue(log.to_stdout)
                self.assertIsNone(log.path)
                for i in range(10):
                    log.write("/vehicle_state", VehicleState(speed=i * 0.1),
                              t_mono_ns=i)
            blob = _sys.stdout.buffer.getvalue()
        finally:
            _sys.stdout = real

        self.assertGreater(len(blob), 0)
        self.assertEqual(log.size_bytes, len(blob))   # パイプでも大きさが分かる
        self.path.write_bytes(blob)
        msgs = read_back(self.path)["msgs"]
        self.assertEqual(len(msgs["/vehicle_state"]), 10)


# ── .sfl → .mcap ────────────────────────────────────────────────────────

@requires_mcap
class TestSflExport(unittest.TestCase):
    """`replay_node` + `BusBridge` を通した変換。**実機と同じ解釈になること。**"""

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.sfl = Path(self._td.name) / "t.sfl"
        self.mcap = Path(self._td.name) / "t.mcap"

    def tearDown(self):
        self._td.cleanup()

    def _write_log(self, n_telem=10, with_lidar=True) -> None:
        w = FrameLogWriter(self.sfl, meta={"node": "test"}, flush_interval_s=0.0)
        t0, seq = w.t0_mono_ns, 0
        for i in range(n_telem):
            seq += 1
            t = packets.Telemetry(t_us=i * 20_000, speed=i * 10, flags=1)
            w.write_rx(t0 + i * 20 * MS, t.TYPE, seq & 0xFF, t.encode())
        if with_lidar:
            # 12セクタ × 2周ぶん。**セクタ番号が戻った時点で1周が完成する**ので、
            # 2周ぶん入れて1周ぶん出てくるのが期待どおり
            for rot in range(2):
                for idx in range(12):
                    seq += 1
                    s = packets.LidarSector(sector_idx=idx, rot_speed_dps=3600,
                                            duration_us=8000,
                                            dist=[1000 + idx] * 30)
                    w.write_rx(t0 + (rot * 100 + idx * 8) * MS, s.TYPE,
                               seq & 0xFF, s.encode())
        cmd = packets.Command(mode=1, flags=0, target_speed=500, target_steer=0,
                              accel_limit=0, steer_rate_limit=0)
        w.write_tx(t0 + 5 * MS, cmd.TYPE, 1, cmd.encode())
        w.write_event(t0 + 6 * MS, "health", {"from": "INIT", "to": "OK"})
        w.close()

    def _export(self, **kw):
        from raspi.tools.sfl2mcap import export

        return export(self.sfl, self.mcap, quiet=True, **kw)

    def test_topics_are_produced(self):
        self._write_log()
        self._export()
        msgs = read_back(self.mcap)["msgs"]
        self.assertEqual(len(msgs["/vehicle_state"]), 10)
        self.assertEqual(len(msgs["/scan"]), 1)        # 2周入れて完成は1周
        self.assertIn("/uart/tx/command", msgs)
        self.assertIn("/events", msgs)

    def test_values_are_si_converted(self):
        """生値ではなく SI で入っていること（`msgs.convert` を通っている証拠）。"""
        self._write_log()
        self._export()
        msgs = read_back(self.mcap)["msgs"]
        speeds = [b["speed"] for _, b in msgs["/vehicle_state"]]
        self.assertAlmostEqual(speeds[-1], 0.09)       # 90 * 1e-3 m/s
        self.assertTrue(all(b["mode"] == 1 for _, b in msgs["/vehicle_state"]))

    def test_timestamps_are_the_recording_time_not_now(self):
        self._write_log()
        before = time.time_ns()
        self._export()
        (log_time, _) = read_back(self.mcap)["msgs"]["/vehicle_state"][0]
        # 記録は「今」より前に始まっている。変換時刻で押していたら未来になる
        self.assertLessEqual(log_time, before)

    def test_telemetry_is_not_duplicated_into_uart_rx(self):
        """`/vehicle_state` と同じ中身を二重に書かない（ファイルが倍になる）。"""
        self._write_log()
        self._export()
        msgs = read_back(self.mcap)["msgs"]
        self.assertNotIn("/uart/rx/telemetry", msgs)
        self.assertNotIn("/uart/rx/lidar_sector", msgs)

    def test_viz_can_be_turned_off(self):
        self._write_log()
        self._export(viz=False)
        msgs = read_back(self.mcap)["msgs"]
        self.assertIn("/scan", msgs)
        self.assertNotIn("/viz/scan", msgs)

    def test_viz_scan_is_written_by_default(self):
        self._write_log()
        self._export()
        self.assertIn("/viz/scan", read_back(self.mcap)["msgs"])

    def test_seq_is_stamped(self):
        """`Publisher` と同じく通し番号を押すこと（実機の記録と形を合わせる）。"""
        self._write_log()
        self._export()
        seqs = [b["seq"] for _, b in read_back(self.mcap)["msgs"]["/vehicle_state"]]
        self.assertEqual(seqs, list(range(1, 11)))

    def test_source_is_recorded_in_metadata(self):
        self._write_log()
        self._export()
        meta = read_back(self.mcap)["metadata"]["surge"]
        self.assertEqual(meta["source"], "t.sfl")
        self.assertEqual(meta["converter"], "sfl2mcap")

    def test_section_can_be_cut_out(self):
        self._write_log(n_telem=50)
        self._export(start_s=0.2, end_s=0.4)
        n = len(read_back(self.mcap)["msgs"]["/vehicle_state"])
        self.assertLess(n, 50)
        self.assertGreater(n, 0)


# ── 尻切れの復旧 ────────────────────────────────────────────────────────

@requires_mcap
class TestRepair(unittest.TestCase):
    """**電源断で `finish()` を通らなかった `.mcap` は普通のリーダで開けない。**

    `.sfl` と違って索引が要る形式なので、復旧経路が無いと記録を丸ごと失う。
    Pi には原因不明の再起動の実績があるので、ここは落とせない。
    """

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.path = Path(self._td.name) / "t.mcap"
        self.out = Path(self._td.name) / "fixed.mcap"

    def tearDown(self):
        self._td.cleanup()

    def _write(self, n=200) -> None:
        with McapLog(self.path, t0_mono_ns=0, t0_unix_ns=0) as log:
            for i in range(n):
                log.write("/vehicle_state", VehicleState(speed=i * 0.01),
                          t_mono_ns=i * 20_000_000)
                if i % 10 == 0:
                    log.write("/cmd", DriveCmd(target_speed=0.1),
                              t_mono_ns=i * 20_000_000)

    def test_truncated_file_is_unreadable_without_repair(self):
        """前提の確認。**これが読めるなら復旧ツールは要らない。**"""
        self._write()
        raw = self.path.read_bytes()
        self.path.write_bytes(raw[: len(raw) * 2 // 3])
        with self.assertRaises(Exception):
            read_back(self.path)

    def test_repair_recovers_the_messages(self):
        from raspi.tools.mcap_repair import repair

        self._write()
        raw = self.path.read_bytes()
        self.path.write_bytes(raw[: len(raw) * 2 // 3])

        r = repair(self.path, self.out)
        self.assertTrue(r["truncated"])
        self.assertGreater(r["messages"], 0)
        msgs = read_back(self.out)["msgs"]
        self.assertIn("/vehicle_state", msgs)
        # 救えた範囲が壊れていないこと（先頭から順に並んでいる）
        speeds = [b["speed"] for _, b in msgs["/vehicle_state"]]
        self.assertAlmostEqual(speeds[0], 0.0)
        self.assertEqual(speeds, sorted(speeds))

    def test_repair_of_a_healthy_file_keeps_everything(self):
        from raspi.tools.mcap_repair import repair

        self._write(n=50)
        r = repair(self.path, self.out)
        self.assertFalse(r["truncated"])
        self.assertEqual(r["topics"]["/vehicle_state"], 50)
        self.assertEqual(read_back(self.out)["schemas"]["/vehicle_state"],
                         "VehicleState")


# ── logger_node ─────────────────────────────────────────────────────────

@requires_mcap
class TestLoggerNode(unittest.TestCase):
    """バス → MCAP。`ipc://` をテンポラリに閉じ込めるので実機と干渉しない。"""

    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self._bus = tempfile.TemporaryDirectory()
        self._old = os.environ.get("SURGE_BUS_DIR")
        os.environ["SURGE_BUS_DIR"] = self._bus.name
        self.path = Path(self._td.name) / "t.mcap"

    def tearDown(self):
        if self._old is None:
            os.environ.pop("SURGE_BUS_DIR", None)
        else:
            os.environ["SURGE_BUS_DIR"] = self._old
        self._td.cleanup()
        self._bus.cleanup()

    def test_bus_messages_land_in_the_file(self):
        from raspi.bus import Publisher
        from raspi.nodes.logger_node import LoggerNode

        node = LoggerNode(self.path, topics=["vehicle_state", "scan"], image_hz=0)
        pub = Publisher("io")
        time.sleep(0.3)                                   # slow joiner
        for i in range(20):
            pub.send("vehicle_state", VehicleState(speed=i * 0.1))
            time.sleep(0.005)
        pub.send("scan", Scan(dist=[1.0] * 360))
        node.run(duration_s=0.5)
        node.close()
        pub.close()

        msgs = read_back(self.path)["msgs"]
        # RELIABLE 購読なので**取りこぼさない**のが要点
        self.assertEqual(len(msgs["/vehicle_state"]), 20)
        self.assertEqual(len(msgs["/scan"]), 1)
        self.assertIn("/viz/scan", msgs)

    def test_stop_ends_the_run(self):
        import threading

        from raspi.nodes.logger_node import LoggerNode

        node = LoggerNode(self.path, topics=["vehicle_state"], image_hz=0)
        threading.Timer(0.2, node.stop).start()
        node.run()                                        # duration 無しでも止まる
        node.close()
        self.assertTrue(self.path.exists())

    @unittest.skipUnless(_has_jpeg(), "JPEG エンコーダが無い（pip install pillow）")
    def test_images_are_recorded_and_thinned(self):
        """**`.mcap` を作った理由そのもの。** カメラ画像は他のどこにも残らない。

        `ImageRef` は全部残し、**画素を焼くのは `--image-hz` に間引く**という
        作り分けが効いていることまで見る。
        """
        import threading

        import numpy as np

        from raspi.bus import FrameRing, Publisher
        from raspi.msgs import ImageRef
        from raspi.nodes.logger_node import LoggerNode

        ring = FrameRing.create("surge_camtest", 64, 48, "RGB888", n_slots=4)
        node = LoggerNode(self.path, topics=["image/"], image_hz=5)
        pub = Publisher("camera")
        time.sleep(0.3)

        def feed():
            for i in range(30):                           # 30Hz を1秒ぶん
                img = np.zeros((48, 64, 3), dtype=np.uint8)
                img[:, :, 0] = 255                        # 赤（RGB のチャネル0）
                d = ring.write(img.tobytes(), t_capture_ns=time.monotonic_ns(),
                               frame_id=i)
                pub.send("image/front", ImageRef(
                    t_capture=d.t_capture_ns, cam="front", shm_name=d.name,
                    slot=d.slot, ring_seq=d.seq, frame_id=d.frame_id,
                    width=d.width, height=d.height, fmt=d.fmt,
                    stride=d.stride, nbytes=d.nbytes))
                time.sleep(1 / 30)

        threading.Thread(target=feed, daemon=True).start()
        try:
            node.run(duration_s=1.2)
        finally:
            node.close()
            pub.close()
            ring.close()
            ring.unlink()

        msgs = read_back(self.path)["msgs"]
        # 参照は全部、画素は 5Hz ぶんだけ
        self.assertGreaterEqual(len(msgs["/image/front"]), 25)
        self.assertGreaterEqual(len(msgs["/viz/image/front"]), 3)
        self.assertLessEqual(len(msgs["/viz/image/front"]), 8)

        body = msgs["/viz/image/front"][0][1]
        self.assertEqual(body["format"], "jpeg")
        jpg = base64.b64decode(body["data"])
        self.assertEqual(jpg[:2], b"\xff\xd8")            # JPEG の SOI

    @unittest.skipUnless(_has_jpeg(), "JPEG エンコーダが無い（pip install pillow）")
    def test_recreated_shm_is_detected(self):
        """**camera_node を再起動すると共有メモリは作り直される。**

        古い方を掴んだままだと、Linux では unlink 後もマッピングが生きるので
        **エラーも出さずに同じ画像を記録し続ける。** 掴み直せること、
        掴み直した後は新しい中身が読めることまで見る。
        """
        import numpy as np
        from PIL import Image

        from raspi.bus import FrameRing
        from raspi.core.jpeg import STALE_GAP, RingJpeg

        def solid(ch: int):
            a = np.zeros((16, 32, 3), dtype=np.uint8)
            a[:, :, ch] = 255
            return a.tobytes()

        jp = RingJpeg(90)
        old = FrameRing.create("surge_stale", 32, 16, "RGB888", n_slots=4)
        try:
            for i in range(STALE_GAP + 10):            # 書き手を十分進めておく
                old.write(solid(0), t_capture_ns=i + 1, frame_id=i)   # 赤
            got = jp.encode_latest("surge_stale", expect_seq=old.write_seq)
            self.assertIsNotNone(got)
            self.assertEqual(jp.reattached, 0)

            old.unlink()                               # camera_node が落ちた
            new = FrameRing.create("surge_stale", 32, 16, "RGB888", n_slots=4)
            new.write(solid(2), t_capture_ns=999, frame_id=0)         # 青
            try:
                got = jp.encode_latest("surge_stale", expect_seq=new.write_seq)
                self.assertIsNotNone(got)
                self.assertEqual(jp.reattached, 1)     # 掴み直した
                r, g, b = Image.open(io.BytesIO(got[0])).convert("RGB").getpixel((0, 0))
                self.assertGreater(b, 200)             # 新しい方（青）が読めている
                self.assertLess(r, 60)
            finally:
                new.close()
                new.unlink()
        finally:
            jp.close()
            old.close()

    @unittest.skipUnless(_has_jpeg(), "JPEG エンコーダが無い（pip install pillow）")
    def test_failure_reason_is_kept(self):
        """**数だけ数えても原因は分からない。** `errors=448` で時間を溶かした反省。"""
        from raspi.core.jpeg import RingJpeg

        jp = RingJpeg(70)
        self.assertIsNone(jp.encode_latest("surge_does_not_exist"))
        self.assertEqual(jp.errors, 1)
        self.assertIn("FileNotFoundError", jp.last_error)

    @unittest.skipUnless(_has_jpeg(), "JPEG エンコーダが無い（pip install pillow）")
    def test_colors_are_not_swapped(self):
        """**BGR/RGB を取り違えると赤と青が入れ替わる。** 記録では気づきにくい。"""
        import numpy as np
        from PIL import Image

        from raspi.bus import FrameRing
        from raspi.core.jpeg import RingJpeg

        ring = FrameRing.create("surge_camtest2", 32, 16, "RGB888", n_slots=2)
        try:
            img = np.zeros((16, 32, 3), dtype=np.uint8)
            img[:, :, 0] = 255                            # メモリ順 R,G,B の R
            ring.write(img.tobytes(), t_capture_ns=1, frame_id=1)
            got = RingJpeg(90).encode_latest("surge_camtest2")
            self.assertIsNotNone(got)
            r, g, b = Image.open(io.BytesIO(got[0])).convert("RGB").getpixel((0, 0))
            self.assertGreater(r, 200)
            self.assertLess(b, 60)
        finally:
            ring.close()
            ring.unlink()


if __name__ == "__main__":
    unittest.main()
