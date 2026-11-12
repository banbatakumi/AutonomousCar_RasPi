"""前方ステレオのカメラの対応と、左右の同期まわりの純粋な部分。

picamera2 が無い開発機（Mac）でも通るように、カメラには触らない。
"""

from __future__ import annotations

import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.vehicle import CAMERA_DEVICES, Vehicle, resolve_lens  # noqa: E402
from raspi.msgs import CamConfig, ImageRef, TOPIC_IMAGE_LEFT  # noqa: E402
import msgspec  # noqa: E402

from raspi.msgs.types import type_for_topic  # noqa: E402
from raspi.nodes import camera_node  # noqa: E402
from raspi.nodes.camera_node import (  # noqa: E402
    STEREO_SYNC_TOL_NS, CameraNode, CameraWorker, exposure_changed, frame_phase_error_ns)

_TOML = """
[sensors.cam_front]
lens = "b"
[sensors.cam_front.lenses.b]
y = -0.0375
[sensors.cam_left]
lens = "c"
[sensors.cam_left.lenses.c]
y = 0.0375
"""


class TestCameras(unittest.TestCase):
    def test_main_is_the_right_camera(self):
        """共有メモリの 0 番＝`image/front` は主カメラ＝右＝libcamera 0 番（2026-11-12 にケーブルを左右で入れ替えた）。"""
        self.assertEqual(CAMERA_DEVICES, (("front", 0), ("left", 1)))

    def test_vehicle_reads_both_cameras(self):
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "v.toml"
            p.write_text(_TOML, encoding="utf-8")
            v = Vehicle.load(p)
        self.assertEqual(v.camera_roles, ("front", "left"))
        self.assertEqual((v.cam_front_lens, v.cam_front_y, v.cam_left_y), ("b", -0.0375, 0.0375))
        self.assertEqual(v.camera_devices, (("front", 0), ("left", 1)))

    def test_repo_vehicle_toml_is_symmetric_stereo(self):
        """リポジトリの `vehicle.toml` は左右対称・基線 75mm のステレオ。"""
        import tomllib
        from raspi.core.vehicle import DEFAULT_PATH
        d = tomllib.loads(Path(DEFAULT_PATH).read_text(encoding="utf-8"))
        sensors = d["sensors"]
        self.assertNotIn("camera", sensors)          # 配置の切り替えは無くした
        self.assertNotIn("cam_rear", sensors)
        right = resolve_lens(sensors["cam_front"], strict=True)
        left = resolve_lens(sensors["cam_left"], strict=True)
        self.assertLess(right["y"], 0.0)
        self.assertGreater(left["y"], 0.0)
        self.assertAlmostEqual(left["y"] - right["y"], 0.075, places=3)


class TestMessages(unittest.TestCase):
    def test_roundtrip(self):
        raw = msgspec.msgpack.encode(ImageRef(cam="left", synced=True))
        ref = msgspec.msgpack.decode(raw, type=type_for_topic(TOPIC_IMAGE_LEFT))
        self.assertIsInstance(ref, ImageRef)
        self.assertTrue(ref.synced)
        self.assertFalse(ImageRef().synced)
        self.assertFalse(CamConfig().stereo)

    def test_left_topic_has_an_owner(self):
        from raspi.bus.zbus import TOPIC_OWNER
        self.assertEqual(TOPIC_OWNER[TOPIC_IMAGE_LEFT], "camera")


class TestSyncedIsMeasured(unittest.TestCase):
    """`synced` は libcamera の `SyncReady` だけでなく、もう1台との差を測って決める
    （fps を変えた後、`SyncReady` が立ったまま最大 19ms ずれていた）。"""

    T30 = 1e9 / 30

    def test_phase_error_wraps_by_the_period(self):
        t = 5_000_000_000
        self.assertEqual(frame_phase_error_ns(t, t, self.T30), 0)
        self.assertAlmostEqual(frame_phase_error_ns(t + 20_000, t, self.T30), 20_000, delta=1)
        # 相手の直近のコマが1コマ前でも、同じ位相なら差は小さい
        self.assertAlmostEqual(frame_phase_error_ns(t + round(self.T30) + 20_000, t, self.T30),
                               20_000, delta=2)
        self.assertAlmostEqual(frame_phase_error_ns(t, t + 20_000, self.T30), 20_000, delta=1)
        # 半周期ずれが最大
        self.assertAlmostEqual(frame_phase_error_ns(t + self.T30 / 2, t, self.T30), self.T30 / 2, delta=1)

    def test_missing_timestamp_is_not_aligned(self):
        self.assertEqual(frame_phase_error_ns(0, 5, self.T30), float("inf"))
        self.assertEqual(frame_phase_error_ns(5, 0, self.T30), float("inf"))
        self.assertEqual(frame_phase_error_ns(5, 5, 0), float("inf"))

    def _worker(self, peer_t):
        from types import SimpleNamespace
        w = CameraWorker.__new__(CameraWorker)
        w.fps = 30.0
        w.peer = None if peer_t is None else SimpleNamespace(stats=SimpleNamespace(last_t_capture=peer_t))
        return w

    def test_aligned_only_within_tolerance(self):
        t = 7_000_000_000
        self.assertTrue(self._worker(t + 100_000)._aligned_with_peer(t))        # 実測の最大 0.11ms
        self.assertFalse(self._worker(t + STEREO_SYNC_TOL_NS + 50_000)._aligned_with_peer(t))
        self.assertFalse(self._worker(t + 1_700_000)._aligned_with_peer(t))     # fps 変更後に出た 1.7ms
        self.assertFalse(self._worker(0)._aligned_with_peer(t))                 # 相手がまだ撮っていない
        self.assertFalse(self._worker(None)._aligned_with_peer(t))              # 相手がいない


class TestExposureFollow(unittest.TestCase):
    def test_changed(self):
        self.assertFalse(exposure_changed(None, None, 0.03))
        self.assertTrue(exposure_changed(None, (10000, 2.0), 0.03))
        self.assertFalse(exposure_changed((10000, 2.0), (10100, 2.0), 0.03))
        self.assertTrue(exposure_changed((10000, 2.0), (10000, 2.2), 0.03))


class _FakeWorker:
    def __init__(self, role: str) -> None:
        self.idx = 0 if role == "front" else 1
        self.calls: list[tuple] = []

    def request_fps(self, v): self.calls.append(("fps", v))
    def request_enabled(self, v): self.calls.append(("enabled", v))
    def request_stereo(self, v): self.calls.append(("stereo", v))


class _OneShotSub:
    def __init__(self, node, msg) -> None:
        self.node, self.msg = node, msg

    def poll(self, _ms):
        self.node._cfg_running = False
        from raspi.msgs.types import TOPIC_CAM_CONFIG
        return [(TOPIC_CAM_CONFIG, self.msg)]


class TestConfigLoop(unittest.TestCase):
    """`cam/config` → 各ワーカーへの希望。左カメラは主カメラと同じ fps で同期するか、止めるか。"""

    def _run(self, msg, topic):
        node = CameraNode.__new__(CameraNode)
        node.workers = [_FakeWorker("front"), _FakeWorker("second")]
        node._cfg_sub = _OneShotSub(node, msg)
        node._cfg_running = True
        old = camera_node.CAM_TOPIC
        camera_node.CAM_TOPIC = topic
        try:
            node._config_loop()
        finally:
            camera_node.CAM_TOPIC = old
        return [dict(w.calls) for w in node.workers]

    STEREO = {0: ("image/front", "front"), 1: ("image/left", "left")}

    def test_stereo_on(self):
        front, left = self._run(CamConfig(front_fps=30.0, stereo=True), self.STEREO)
        self.assertEqual(left, {"fps": 30.0, "stereo": True, "enabled": True})
        self.assertEqual(front["stereo"], True)

    def test_stereo_off_stops_left(self):
        front, left = self._run(CamConfig(front_fps=10.0, stereo=False), self.STEREO)
        self.assertEqual((left["enabled"], left["stereo"]), (False, False))
        self.assertEqual((front["enabled"], front["stereo"]), (True, False))

    def test_left_stops_with_main(self):
        _, left = self._run(CamConfig(front_enabled=False, stereo=True), self.STEREO)
        self.assertFalse(left["enabled"])


class TestSyncToggle(unittest.TestCase):
    """同期の入り切りは、カメラを止めてコントロールを置き直してから掛け直す。"""

    def test_toggle_restarts_camera(self):
        class Cam:
            def __init__(self): self.log = []
            def stop(self): self.log.append("stop")
            def start(self): self.log.append("start")
            def set_controls(self, c): self.log.append(("ctrl", dict(c)))

        w = CameraWorker.__new__(CameraWorker)
        w.cam = Cam()
        w.fps = 30.0
        w.stats = camera_node.CamStats()
        w._cfg_lock = threading.Lock()
        w._pending_fps = None
        w._pending_enabled = None
        w._pending_sync = None
        w._enabled = True
        w._sync_on = True
        w.sync_role = "client"
        w.synced = True
        w._followed = (1000, 1.0)
        w.follow = object()
        w._sync_controls = lambda: {"SyncMode": "off" if not w._sync_on else "client"}
        w.request_stereo(False)
        w._apply_pending()
        self.assertEqual(w.cam.log, ["stop", ("ctrl", {"SyncMode": "off"}),
                                     ("ctrl", {"AeEnable": True}), "start"])
        self.assertFalse(w.synced)
        self.assertIsNone(w._followed)
        w.cam.log.clear()
        w.request_stereo(False)          # 同じ要求では何もしない
        w._apply_pending()
        self.assertEqual(w.cam.log, [])


class TestTelemetryStereoDemand(unittest.TestCase):
    """telemetry_node が `CamConfig.stereo` を立てる条件。"""

    def _node(self, armed: bool, in_use: bool):
        from raspi.nodes import telemetry_node as tel

        class Stub:
            _desired_stereo = tel.TelemetryServer._desired_stereo
            _desired_depth = tel.TelemetryServer._desired_depth
            camera_clients: dict = {"depth": set()}
            _auto_mode = ""
            def _vehicle_armed(self): return armed
            def _cam_in_use_while_disarmed(self, cam):
                assert cam == "left"
                return in_use
        return tel, Stub()

    def test_demand(self):
        # 段階4 から ARM だけでは左を回さない（深度か左の映像を使う者がいる間だけ）
        for armed, in_use, want in [(True, False, False), (False, True, True), (False, False, False),
                                    (True, True, True)]:
            _tel, node = self._node(armed, in_use)
            self.assertEqual(node._desired_stereo(), want, (armed, in_use))



class TestStereoHbAndI2c(unittest.TestCase):
    """`hb/camera` の表示と、I2C アドレスからの役割。"""

    def _worker(self, idx, sync_on, synced):
        from types import SimpleNamespace
        return SimpleNamespace(idx=idx, stats=camera_node.CamStats(), _enabled=sync_on or idx == 0,
                               _sync_on=sync_on, synced=synced)

    def _detail(self, workers):
        from types import SimpleNamespace
        from unittest.mock import patch
        with patch.object(camera_node, "CAM_TOPIC", TestConfigLoop.STEREO):
            return camera_node.make_hb_detail()(SimpleNamespace(workers=workers))

    def test_sync_state(self):
        self.assertEqual(self._detail([self._worker(0, True, True), self._worker(1, True, True)]),
                         "front=- left=- sync=ready")
        self.assertEqual(self._detail([self._worker(0, True, True), self._worker(1, True, False)]),
                         "front=- left=- sync=wait")
        self.assertEqual(self._detail([self._worker(0, False, False), self._worker(1, False, False)]),
                         "front=- left=off sync=off")
        # 片方しか開けなかったら、揃ったとは言わない
        self.assertEqual(self._detail([self._worker(0, True, True)]), "front=- sync=wait")

    def test_i2c_roles(self):
        from unittest.mock import patch
        infos = [{"Id": "/base/axi/pcie@120000/rp1/i2c@88000/imx219@10"},
                 {"Id": "/base/axi/pcie@120000/rp1/i2c@80000/imx219@10"}]
        with patch.object(camera_node, "I2C_ADDR_ROLE", {"i2c@88000": "left", "i2c@80000": "front"}):
            self.assertEqual(camera_node.devices_by_i2c(infos, ["front", "left"]), {"left": 0, "front": 1})


if __name__ == "__main__":
    unittest.main()
