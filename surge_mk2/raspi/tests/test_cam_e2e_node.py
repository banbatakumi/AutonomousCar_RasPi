"""`raspi/nodes/cam_e2e_node.py` の配線テスト。

**実モデル・実カメラは要らない。** ダミーの ONNX モデル（画像の平均輝度から
1個のスカラーを作るだけの回帰モデル）で、フレーム読み取り→前処理→推論→
`CamE2ECmd` 化という配管全体が壊れずに流れることだけを確認する
（`test_cam_perception_node.py` と同じ方針）。推論の精度は問わない
——それは実データが要る領域（`ml_cam_e2e/`）の仕事。
"""

import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402
import onnx  # noqa: E402
from onnx import TensorProto, helper  # noqa: E402

from raspi.bus import FrameRing  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import AutoCtrl, CamE2EModelCtrl, ImageRef, Scan, VehicleState  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_CAM_E2E_CMD,
    TOPIC_CAM_E2E_MODEL,
    TOPIC_IMAGE_FRONT,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
)
from raspi.nodes.cam_e2e_node import CamE2ENode, RegressionModel  # noqa: E402


def _make_dummy_regression_model(path: Path, h: int, w: int) -> None:
    """画素の平均から `[1,1,1,1]` のスカラーを作るだけの ONNX モデル。

    前処理で `(pixel - mean) / std` に正規化した入力（0..1）を渡すので、
    出力は「明るいほど大きい」値になる——推論そのものの正しさは問わず、
    配管が通ることと「入力が変われば出力も変わる」ことだけを確認したいので十分。
    """
    inp = helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, 3, h, w])
    out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, 1, 1, 1])
    node = helper.make_node("ReduceMean", ["input"], ["output"],
                            axes=[1, 2, 3], keepdims=1)
    graph = helper.make_graph([node], "dummy", [inp], [out])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    onnx.checker.check_model(model)
    onnx.save(model, str(path))


class TestRegressionModel(unittest.TestCase):
    def test_brighter_frame_gives_a_larger_output(self):
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "dummy.onnx"
            _make_dummy_regression_model(model_path, 32, 32)
            model = RegressionModel(str(model_path), input_size=(32, 32),
                                    mean=0.0, std=255.0, max_steer=0.524)

            dark = np.zeros((240, 320, 3), dtype=np.uint8)
            bright = np.full((240, 320, 3), 255, dtype=np.uint8)
            self.assertLess(model.infer(dark), model.infer(bright))

    def test_infer_returns_a_python_float(self):
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "dummy.onnx"
            _make_dummy_regression_model(model_path, 16, 16)
            model = RegressionModel(str(model_path), input_size=(16, 16))
            out = model.infer(np.zeros((64, 64, 3), dtype=np.uint8))
            self.assertIsInstance(out, float)


class TestProcessFrame(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.model_path = Path(self._tmp.name) / "dummy.onnx"
        _make_dummy_regression_model(self.model_path, 32, 32)

    def tearDown(self):
        self._tmp.cleanup()

    def test_process_frame_returns_the_model_output(self):
        model = RegressionModel(str(self.model_path), input_size=(32, 32),
                                mean=0.0, std=255.0, max_steer=0.524)
        node = CamE2ENode(model=model, vehicle=Vehicle.load())
        frame = np.full((240, 320, 3), 128, dtype=np.uint8)
        steer_norm = node.process_frame(frame)
        self.assertIsInstance(steer_norm, float)


class TestFrontLidarDist(unittest.TestCase):
    def setUp(self):
        self.node = CamE2ENode(vehicle=Vehicle.load(), lidar_fov_deg=40.0, lidar_max_range=3.0)

    def test_none_scan_is_unseen(self):
        dist, seen = self.node.front_lidar_dist(None)
        self.assertEqual(dist, 0.0)
        self.assertFalse(seen)

    def test_reports_the_minimum_distance_in_front_fov(self):
        dist = [3.0] * 360
        dist[0] = 1.2       # 正面
        dist[10] = 0.8       # 視野内だがもっと近い
        scan = Scan(dist=dist, sector_seen=[True] * 12)
        got, seen = self.node.front_lidar_dist(scan)
        self.assertTrue(seen)
        self.assertAlmostEqual(got, 0.8)

    def test_unseen_sector_in_front_fov_is_treated_as_wall(self):
        """`scan_window()` は欠測を距離0（壁）として返す（安全側）。"""
        scan = Scan(dist=[3.0] * 360, sector_seen=[False] * 12)
        got, seen = self.node.front_lidar_dist(scan)
        self.assertFalse(seen)


class TestModelSelection(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.models_dir = Path(self._tmp.name)
        _make_dummy_regression_model(self.models_dir / "model_a.onnx", 32, 32)
        (self.models_dir / "model_a.json").write_text(
            '{"input_size": [32, 32], "mean": 0.0, "std": 255.0, "max_steer": 0.524}')
        _make_dummy_regression_model(self.models_dir / "model_b.onnx", 16, 16)
        (self.models_dir / "model_b.json").write_text(
            '{"input_size": [16, 16], "mean": 0.0, "std": 255.0, "max_steer": 0.3}')

    def tearDown(self):
        self._tmp.cleanup()

    def test_starts_with_no_model_when_none_given(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        self.assertIsNone(node.model)

    def test_reload_switches_to_the_named_model(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        changed = node.reload_if_changed("model_a")
        self.assertTrue(changed)
        self.assertEqual(node.model.input_size, (32, 32))
        self.assertAlmostEqual(node.model.max_steer, 0.524)

    def test_reload_is_a_noop_when_name_unchanged(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        node.reload_if_changed("model_a")
        first = node.model
        self.assertFalse(node.reload_if_changed("model_a"))
        self.assertIs(node.model, first)

    def test_missing_model_keeps_the_previous_one(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        node.reload_if_changed("model_a")
        first = node.model
        self.assertFalse(node.reload_if_changed("no-such-model"))
        self.assertIs(node.model, first)


class _FakeSub:
    def __init__(self, latest=None):
        self.latest = latest or {}

    def poll(self, timeout_ms):
        return []


class _FakePub:
    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


def _ref_with_frame(ring_name: str):
    ring = FrameRing.create(ring_name, 32, 24, "RGB888", n_slots=2)
    data = np.full((24, 32, 3), 200, dtype=np.uint8)
    desc = ring.write(data, t_capture_ns=1, frame_id=1)
    ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                   frame_id=desc.frame_id, width=desc.width, height=desc.height,
                   fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
    return ring, ref


class TestModeGating(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.models_dir = Path(self._tmp.name)
        _make_dummy_regression_model(self.models_dir / "model_a.onnx", 32, 32)
        (self.models_dir / "model_a.json").write_text(
            '{"input_size": [32, 32], "mean": 0.0, "std": 255.0, "max_steer": 0.524}')

    def tearDown(self):
        self._tmp.cleanup()

    def test_stays_idle_when_a_different_mode_is_selected(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_idle")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="line_trace")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            self.assertIsNotNone(node.model, "モデル自体はモード非依存でロードされるはず")
            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertTrue(cmds)
            self.assertFalse(cmds[-1].ready, "cam_e2e以外が選ばれているのに推論結果が出ている")
        finally:
            node.close()
            ring.unlink()

    def test_selecting_cam_e2e_mode_starts_inference(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_active")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertTrue(cmds)
            self.assertTrue(any(c.ready for c in cmds),
                            "cam_e2eが選ばれたのに推論結果が出ていない")
        finally:
            node.close()
            ring.unlink()

    def test_disarmed_stays_idle_even_when_cam_e2e_mode_selected(self):
        """cam_e2eが選ばれていてもDISARM中は推論を回さない（省電力バグ修正）。"""
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_disarm")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e"),
                            TOPIC_VEHICLE_STATE: VehicleState(armed=False)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertTrue(cmds)
            self.assertFalse(cmds[-1].ready, "DISARM中なのに推論結果が出ている")
        finally:
            node.close()
            ring.unlink()

    def test_armed_with_cam_e2e_mode_selected_still_activates_inference(self):
        """ARM中はモード選択だけで推論が回る（回帰防止）。"""
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_armed")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e"),
                            TOPIC_VEHICLE_STATE: VehicleState(armed=True)})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertTrue(any(c.ready for c in cmds),
                            "ARM中にcam_e2eが選ばれたのに推論結果が出ていない")
        finally:
            node.close()
            ring.unlink()

    def test_lidar_front_dist_is_included_even_without_a_scan(self):
        """`scan` がまだ届いていなくても `ready` な推論結果自体は出ること
        （`lidar_seen=False` になるだけ）。"""
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_noscan")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertTrue(any(c.ready for c in cmds))
            self.assertTrue(any(not c.lidar_seen for c in cmds))
        finally:
            node.close()
            ring.unlink()

    def test_scan_updates_lidar_front_dist(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_wscan")
        try:
            scan = Scan(dist=[1.5] * 360, sector_seen=[True] * 12)
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_SCAN: scan,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            ready_cmds = [c for c in cmds if c.ready]
            self.assertTrue(ready_cmds)
            self.assertTrue(all(c.lidar_seen and c.lidar_front_dist > 0 for c in ready_cmds))
        finally:
            node.close()
            ring.unlink()


class TestStop(unittest.TestCase):
    def test_stop_breaks_the_run_loop(self):
        node = CamE2ENode(models_dir=Path("/nonexistent"), vehicle=Vehicle.load())
        sub = _FakeSub({})
        pub = _FakePub()

        thread = threading.Thread(target=node.run, kwargs={"sub": sub, "pub": pub})
        thread.start()
        time.sleep(0.05)
        self.assertTrue(node._running)
        node.stop()
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive(), "stop() を呼んでも run() が終わらない")


if __name__ == "__main__":
    unittest.main()
