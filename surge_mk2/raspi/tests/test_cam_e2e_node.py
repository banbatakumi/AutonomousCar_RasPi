"""`raspi/nodes/cam_e2e_node.py` の配線テスト。

**実モデル・実カメラは要らない。** ダミーの ONNX モデル（R・G チャンネルの
平均を2個の出力にするだけ）で、フレーム読み取り→色順の修正→前処理→推論→
`CamE2ECmd` 化という配管全体が壊れずに流れることだけを確認する
（`test_cam_perception_node.py` と同じ方針）。推論の精度は問わない
——それは実データが要る領域（`ml_cam_e2e/`）の仕事。
"""

import json
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
from raspi.core.cam_e2e_preproc import PREPROC_VERSION  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import AutoCtrl, CamE2EModelCtrl, ImageRef, VehicleState  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_CAM_E2E_CMD,
    TOPIC_CAM_E2E_MODEL,
    TOPIC_IMAGE_FRONT,
    TOPIC_VEHICLE_STATE,
)
from raspi.nodes.cam_e2e_node import (  # noqa: E402
    MODEL_OUTPUTS,
    CamE2ENode,
    RegressionModel,
    load_model,
)


def _make_dummy_regression_model(path: Path, h: int, w: int, n_out: int = 2) -> None:
    """R・G（先頭 `n_out` チャンネル）の平均を `[1, n_out]` で返すだけの ONNX モデル。

    前処理で 0..1 に正規化した RGB を渡すので、出力は `(赤の明るさ, 緑の明るさ)`。
    推論そのものの正しさは問わず、配管が通ることと**色順が RGB で届くこと**を
    確かめるのに使う。
    """
    inp = helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, 3, h, w])
    out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, n_out])
    idx = helper.make_tensor("idx", TensorProto.INT64, [n_out], list(range(n_out)))
    nodes = [
        helper.make_node("ReduceMean", ["input"], ["chan"], axes=[2, 3], keepdims=0),
        helper.make_node("Gather", ["chan", "idx"], ["output"], axis=1),
    ]
    graph = helper.make_graph(nodes, "dummy", [inp], [out], initializer=[idx])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    onnx.checker.check_model(model)
    onnx.save(model, str(path))


def _write_contract(path: Path, w: int, h: int, **over) -> None:
    cfg = {"input_size": [w, h], "mean": 0.0, "std": 255.0, "max_steer": 0.524,
           "speed_ref": 1.5, "outputs": list(MODEL_OUTPUTS),
           "preproc_version": PREPROC_VERSION}
    cfg.update(over)
    path.write_text(json.dumps(cfg))


def _make_model_files(models_dir: Path, name: str, w: int, h: int, **over) -> None:
    _make_dummy_regression_model(models_dir / f"{name}.onnx", h, w)
    _write_contract(models_dir / f"{name}.json", w, h, **over)


class TestRegressionModel(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.model_path = Path(self._tmp.name) / "dummy.onnx"
        _make_dummy_regression_model(self.model_path, 32, 32)
        self.model = RegressionModel(str(self.model_path), input_size=(32, 32),
                                     mean=0.0, std=255.0, max_steer=0.524, speed_ref=1.5)

    def tearDown(self):
        self._tmp.cleanup()

    def test_returns_two_python_floats(self):
        out = self.model.infer(np.zeros((64, 64, 3), dtype=np.uint8))
        self.assertEqual(len(out), 2)
        self.assertIsInstance(out[0], float)
        self.assertIsInstance(out[1], float)

    def test_input_is_taken_as_rgb(self):
        red = np.zeros((240, 320, 3), dtype=np.uint8)
        red[..., 0] = 255
        steer, speed = self.model.infer(red)
        self.assertAlmostEqual(steer, 1.0, places=5)
        self.assertAlmostEqual(speed, 0.0, places=5)

    def test_single_output_model_is_rejected(self):
        """操舵だけの旧モデルを置いたまま走らせない。"""
        one = Path(self._tmp.name) / "one.onnx"
        _make_dummy_regression_model(one, 32, 32, n_out=1)
        model = RegressionModel(str(one), input_size=(32, 32))
        with self.assertRaises(ValueError):
            model.infer(np.zeros((64, 64, 3), dtype=np.uint8))


class TestLoadModel(unittest.TestCase):
    """契約（同梱JSON）が合わないモデルは読み込みで弾く。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_reads_the_contract(self):
        _make_model_files(self.dir, "ok", 32, 16)
        model = load_model(self.dir / "ok.onnx")
        self.assertEqual(model.input_size, (32, 16))
        self.assertAlmostEqual(model.max_steer, 0.524)
        self.assertAlmostEqual(model.speed_ref, 1.5)

    def test_rejects_bad_contracts(self):
        cases = {
            "old_outputs": {"outputs": ["steer_norm"]},
            "old_preproc": {"preproc_version": PREPROC_VERSION + 1},
            "no_speed_ref": {"speed_ref": 0.0},
            "no_max_steer": {"max_steer": 0.0},
        }
        for name, over in cases.items():
            _make_model_files(self.dir, name, 32, 32, **over)
            with self.assertRaises(ValueError, msg=name):
                load_model(self.dir / f"{name}.onnx")

    def test_missing_contract_file_is_rejected(self):
        """セグメンテーション用など、同梱JSONの無い .onnx を読まない。"""
        _make_dummy_regression_model(self.dir / "bare.onnx", 32, 32)
        with self.assertRaises(FileNotFoundError):
            load_model(self.dir / "bare.onnx")


class TestProcessFrame(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        _make_model_files(Path(self._tmp.name), "m", 32, 32)
        self.node = CamE2ENode(model=load_model(Path(self._tmp.name) / "m.onnx"),
                               vehicle=Vehicle.load())

    def tearDown(self):
        self._tmp.cleanup()

    def test_bgr_ring_frame_reaches_the_model_as_rgb(self):
        """実機のリングは BGR888。赤い物（ch2）がモデルの R 入力に届くこと
        ——以前は入れ替えずに渡していて、学習（RGB）と色が逆だった。"""
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        frame[..., 2] = 255
        steer, speed = self.node.process_frame(frame, "BGR888")
        self.assertAlmostEqual(steer, 1.0, places=5)
        self.assertAlmostEqual(speed, 0.0, places=5)

    def test_rgb_ring_frame_is_not_swapped(self):
        frame = np.zeros((240, 320, 3), dtype=np.uint8)
        frame[..., 2] = 255
        steer, _speed = self.node.process_frame(frame, "RGB888")
        self.assertAlmostEqual(steer, 0.0, places=5)


class TestModelSelection(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.models_dir = Path(self._tmp.name)
        _make_model_files(self.models_dir, "model_a", 32, 32)
        _make_model_files(self.models_dir, "model_b", 16, 16, max_steer=0.3)

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
    desc = ring.write(data, t_capture_ns=time.monotonic_ns(), frame_id=1)
    ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                   frame_id=desc.frame_id, width=desc.width, height=desc.height,
                   fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
    return ring, ref


class TestModeGating(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.models_dir = Path(self._tmp.name)
        _make_model_files(self.models_dir, "model_a", 32, 32)

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

    def test_ready_cmd_carries_both_outputs_and_the_contract(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_out")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            ready = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD and c.ready]
            self.assertTrue(ready)
            cmd = ready[-1]
            # ダミーモデルは (R平均, G平均)。リングは一様な 200 なので両方 200/255
            self.assertAlmostEqual(cmd.steer_norm, 200 / 255, places=3)
            self.assertAlmostEqual(cmd.speed_norm, 200 / 255, places=3)
            self.assertAlmostEqual(cmd.model_max_steer, 0.524)
            self.assertAlmostEqual(cmd.model_speed_ref, 1.5)
            self.assertGreater(cmd.t_capture, 0)
        finally:
            node.close()
            ring.unlink()

    def test_publishes_only_once_per_inference_period(self):
        """`Publisher` は送るたびに seq を振るので、再送すると planning_node の
        `plan()` が無駄に回る。推論を試みた周期だけ publish すること。"""
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load(), infer_hz=10.0)
        ring, ref = _ref_with_frame("surge_test_ce2e_rate")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref,
                            TOPIC_CAM_E2E_MODEL: CamE2EModelCtrl(name="model_a"),
                            TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.05)      # 10Hz の1周期（100ms）未満

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertEqual(len(cmds), 1)
        finally:
            node.close()
            ring.unlink()

    def test_no_model_reports_not_ready_with_a_fresh_timestamp(self):
        """`t_capture=0` だと planning_node が「入力が古い」で先に止めてしまい、
        planner の「モデル未選択」が GUI に出ない。"""
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring, ref = _ref_with_frame("surge_test_ce2e_nomodel")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertTrue(cmds)
            self.assertFalse(cmds[-1].ready)
            self.assertLess(time.monotonic_ns() - cmds[-1].t_capture, 1_000_000_000)
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


class TestRunSurvivesProcessFrameException(unittest.TestCase):
    """`process_frame()` が例外を投げてもノードが継続すること。

    `planning_node._replan()` の `planner.plan()` 例外保護（B3）と同じパターンを
    `run()` 側の推論呼び出しにも横展開したもの——推論側のバグでノード全体を
    巻き込んで落とさず、契約の「ready=False」に自然に落とす。
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.models_dir = Path(self._tmp.name)
        _make_model_files(self.models_dir, "model_a", 32, 32)

    def tearDown(self):
        self._tmp.cleanup()

    def test_exception_does_not_propagate_and_falls_back_to_not_ready(self):
        node = CamE2ENode(models_dir=self.models_dir, vehicle=Vehicle.load(),
                          infer_hz=1000.0)
        node.reload_if_changed("model_a")
        self.assertIsNotNone(node.model)

        def _raise(*a, **kw):
            raise RuntimeError("推論側のバグ")
        node.process_frame = _raise

        ring, ref = _ref_with_frame("surge_test_ce2e_exc")
        try:
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_AUTO_CTRL: AutoCtrl(mode="cam_e2e")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)  # 例外を外に伝播させないこと

            cmds = [c for topic, c in pub.sent if topic == TOPIC_CAM_E2E_CMD]
            self.assertTrue(cmds, "cam_e2e/cmd が一度も publish されていない")
            self.assertFalse(cmds[-1].ready,
                             "process_frame()が例外を投げたのにready=Trueになっている")
        finally:
            node.close()
            ring.unlink()


if __name__ == "__main__":
    unittest.main()
