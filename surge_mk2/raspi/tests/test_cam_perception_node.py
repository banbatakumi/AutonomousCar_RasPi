"""`raspi/nodes/cam_perception_node.py` の配線テスト。

**実モデル・実カメラは要らない。** ダミーの ONNX モデル（1オペレータだけの
恒等/閾値モデル）で、フレーム読み取り→前処理→推論→IPM→raycast→`Scan`化
という配管全体が壊れずに流れることだけを確認する（計画の構築順序 6番）。
推論の精度は問わない——それは実データが要る領域（`ml_cam/`）の仕事。
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
from raspi.msgs import CamModelCtrl, ImageRef  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_CAM_MODEL,
    TOPIC_IMAGE_FRONT,
    TOPIC_SCAN_CAM,
)
from raspi.nodes.cam_perception_node import (  # noqa: E402
    CamPerceptionNode,
    SegmentationModel,
)


def _make_dummy_model(path: Path, h: int, w: int) -> None:
    """RGB 3チャンネルの平均を取るだけの ONNX モデル。**1オペレータ。**

    前処理で `(pixel - mean) / std` に正規化した入力を渡すので、出力は
    そのまま「明るいほど 1 に近い」確率マップになる——推論そのものの
    正しさは問わず、配管が通ることだけを確認したいのでこれで十分。
    """
    inp = helper.make_tensor_value_info("input", TensorProto.FLOAT, [1, 3, h, w])
    out = helper.make_tensor_value_info("output", TensorProto.FLOAT, [1, 1, h, w])
    node = helper.make_node("ReduceMean", ["input"], ["output"], axes=[1], keepdims=1)
    graph = helper.make_graph([node], "dummy", [inp], [out])
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 13)])
    onnx.checker.check_model(model)
    onnx.save(model, str(path))


class TestSegmentationModel(unittest.TestCase):
    def test_bright_region_is_drivable_dark_region_is_not(self):
        with tempfile.TemporaryDirectory() as d:
            model_path = Path(d) / "dummy.onnx"
            _make_dummy_model(model_path, 32, 32)
            model = SegmentationModel(str(model_path), input_size=(32, 32),
                                      mean=0.0, std=255.0, threshold=0.5)

            frame = np.full((240, 320, 3), 255, dtype=np.uint8)   # 全面「走行可能」
            frame[100:180, 100:220] = 0                          # 暗い矩形＝障害物

            mask = model.infer(frame)
            self.assertEqual(mask.shape, (32, 32))
            # リサイズ後もおおむね中央付近が暗い（障害物）はず
            self.assertFalse(bool(mask[16, 16]))
            self.assertTrue(bool(mask[2, 2]))                    # 端は明るいまま


class TestCamPerceptionNodeProcessFrame(unittest.TestCase):
    def setUp(self):
        # `tearDown` まで実体を残すため `with` を使わず自前で閉じる
        self._tmp = tempfile.TemporaryDirectory()
        self.model_path = Path(self._tmp.name) / "dummy.onnx"
        _make_dummy_model(self.model_path, 64, 64)

    def tearDown(self):
        self._tmp.cleanup()

    def test_returns_scan_shaped_message_with_camera_fov_only(self):
        model = SegmentationModel(str(self.model_path), input_size=(64, 64),
                                  mean=0.0, std=255.0, threshold=0.5)
        node = CamPerceptionNode(model=model, vehicle=Vehicle.load(), fov_deg=60.0,
                                 max_range=3.0, grid_resolution=0.05, grid_size_m=6.0)

        frame = np.full((240, 320, 3), 255, dtype=np.uint8)      # 全面「走行可能」
        st = node.process_frame(frame, seq=7)

        self.assertEqual(len(st.dist), 360)
        self.assertEqual(st.seq, 7)
        # カメラ視野の外は常に sector_seen=False（契約2）
        visible = {i for i, s in enumerate(st.sector_seen) if s}
        self.assertLess(len(visible), 12, "視野外まで見えている扱いになっている")
        self.assertGreater(len(visible), 0, "視野内が1セクタも見えていない")
        # 全面「走行可能」なら、視野内の距離は 0.0 に張り付かない（契約1）
        for deg in range(-25, 26):
            self.assertGreater(st.dist[deg % 360], 0.0)

    def test_a_dark_region_shortens_distance_in_that_direction(self):
        """暗い矩形（障害物）を置いた方向だけ距離が縮むこと。"""
        model = SegmentationModel(str(self.model_path), input_size=(64, 64),
                                  mean=0.0, std=255.0, threshold=0.5)
        node = CamPerceptionNode(model=model, vehicle=Vehicle.load(), fov_deg=60.0,
                                 max_range=3.0, grid_resolution=0.05, grid_size_m=6.0)

        open_frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        blocked_frame = open_frame.copy()
        blocked_frame[140:200, 130:190] = 0                      # 画面下寄り中央＝正面近く

        st_open = node.process_frame(open_frame, seq=1)
        st_blocked = node.process_frame(blocked_frame, seq=2)

        self.assertLess(st_blocked.dist[0], st_open.dist[0],
                        "障害物を置いても正面方向の距離が縮まなかった")

    def test_failed_frame_marks_everything_unseen(self):
        model = SegmentationModel(str(self.model_path), input_size=(64, 64))
        node = CamPerceptionNode(model=model, vehicle=Vehicle.load())
        st = node.failed_frame(seq=3)
        self.assertEqual(st.sector_seen, [False] * 12)
        self.assertEqual(st.seq, 3)


class TestModelSelection(unittest.TestCase):
    """`cam/model` トピック経由でのモデル切替。

    想定シーンは「走行開始（engage）ボタンを押す前に、GUIでモデルを選び直す」
    ——プロセスの再起動もSSHも要らない、という今回追加した機能そのもの。
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.models_dir = Path(self._tmp.name)
        _make_dummy_model(self.models_dir / "model_a.onnx", 32, 32)
        (self.models_dir / "model_a.json").write_text(
            '{"input_size": [32, 32], "mean": 0.0, "std": 255.0, "threshold": 0.5}')
        _make_dummy_model(self.models_dir / "model_b.onnx", 16, 16)
        (self.models_dir / "model_b.json").write_text(
            '{"input_size": [16, 16], "mean": 0.0, "std": 255.0, "threshold": 0.5}')

    def tearDown(self):
        self._tmp.cleanup()

    def test_starts_with_no_model_when_none_given(self):
        """`--model` 省略時の初期状態。GUI選択が来るまでは推論しようがない。"""
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        self.assertIsNone(node.model)

    def test_reload_switches_to_the_named_model(self):
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        changed = node.reload_if_changed("model_a")
        self.assertTrue(changed)
        self.assertIsNotNone(node.model)
        self.assertEqual(node.model.input_size, (32, 32))     # model_a.json 由来

    def test_reload_is_a_noop_when_name_unchanged(self):
        """同じ名前が来続けても毎周期作り直さない（ONNXセッション生成は軽くない）。"""
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        node.reload_if_changed("model_a")
        first = node.model
        changed = node.reload_if_changed("model_a")
        self.assertFalse(changed)
        self.assertIs(node.model, first)

    def test_reload_switches_between_two_different_models(self):
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        node.reload_if_changed("model_a")
        self.assertEqual(node.model.input_size, (32, 32))
        node.reload_if_changed("model_b")
        self.assertEqual(node.model.input_size, (16, 16))

    def test_missing_model_keeps_the_previous_one(self):
        """存在しないモデル名を選んでも、前のモデルのまま走り続けられること。"""
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        node.reload_if_changed("model_a")
        first = node.model
        changed = node.reload_if_changed("no-such-model")
        self.assertFalse(changed)
        self.assertIs(node.model, first, "存在しないモデル名で前のモデルが失われた")

    def test_missing_json_falls_back_to_default_preprocessing(self):
        _make_dummy_model(self.models_dir / "no_config.onnx", 24, 24)
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        changed = node.reload_if_changed("no_config")
        self.assertTrue(changed)
        self.assertEqual(node.model.input_size, (224, 224))    # SegmentationModel の既定値


class _FakeSub:
    """`Subscriber` の身代わり。`latest` を読むだけ・`poll` は何も待たない。"""

    def __init__(self, latest=None):
        self.latest = latest or {}

    def poll(self, timeout_ms):
        return []


class _FakePub:
    def __init__(self):
        self.sent = []

    def send(self, topic, msg):
        self.sent.append((topic, msg))


class TestRunModelSwitchIntegration(unittest.TestCase):
    """`run()` 全体を通した配線確認（バスは身代わり、モデル・共有メモリは本物）。"""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.models_dir = Path(self._tmp.name)
        _make_dummy_model(self.models_dir / "model_a.onnx", 32, 32)
        # ダミーONNXは固定形状 [1,3,32,32] で作ってあるので、既定値(224x224)に
        # 落ちないよう合わせた .json を必ず置くこと（実際にこれを忘れて
        # 「既定サイズで推論しようとして ONNXRuntime が形状不一致で落ちる」
        # というテスト自身のバグを一度踏んだ）
        (self.models_dir / "model_a.json").write_text(
            '{"input_size": [32, 32], "mean": 0.0, "std": 255.0, "threshold": 0.5}')

    def tearDown(self):
        self._tmp.cleanup()

    def test_publishes_failed_frame_until_a_model_is_selected(self):
        """モデル未選択のまま `run()` を回しても、壁扱いの `Scan` を出し続けるだけ
        で例外にならないこと（契約2）。"""
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        sub = _FakeSub({})
        pub = _FakePub()
        node.run(sub=sub, pub=pub, duration_s=0.02)
        self.assertTrue(pub.sent)
        topic, st = pub.sent[0]
        self.assertEqual(topic, TOPIC_SCAN_CAM)
        self.assertEqual(st.sector_seen, [False] * 12)

    def test_selecting_a_model_via_cam_model_topic_starts_inference(self):
        """`cam/model` に流れてきた名前を `run()` が実際に拾って切り替えること。"""
        node = CamPerceptionNode(models_dir=self.models_dir, vehicle=Vehicle.load())
        ring = FrameRing.create("surge_test_cam_perception_run", 32, 24, "RGB888", n_slots=2)
        try:
            data = np.full((24, 32, 3), 255, dtype=np.uint8)
            desc = ring.write(data, t_capture_ns=1, frame_id=1)
            ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                           frame_id=desc.frame_id, width=desc.width, height=desc.height,
                           fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
            sub = _FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_CAM_MODEL: CamModelCtrl(name="model_a")})
            pub = _FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.02)

            self.assertIsNotNone(node.model)
            self.assertEqual(node._loaded_model_name, "model_a")
            topic, st = pub.sent[-1]
            self.assertEqual(topic, TOPIC_SCAN_CAM)
            self.assertTrue(any(st.sector_seen), "モデルが選ばれたのに推論結果が出ていない")
        finally:
            node.close()
            ring.unlink()


class TestStop(unittest.TestCase):
    """`stop()` で `run()` が（`duration_s`無しでも）確実に抜けること（B1）。"""

    def test_stop_breaks_the_run_loop(self):
        node = CamPerceptionNode(models_dir=Path("/nonexistent"), vehicle=Vehicle.load())
        sub = _FakeSub({})
        pub = _FakePub()

        thread = threading.Thread(target=node.run, kwargs={"sub": sub, "pub": pub})
        thread.start()
        # `run()` がループへ入るまで少し待ってから止める
        time.sleep(0.05)
        self.assertTrue(node._running)
        node.stop()
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive(), "stop() を呼んでも run() が終わらない")


class TestReadFrame(unittest.TestCase):
    """`FrameRing` からの読み取り（`raspi/core/jpeg.py` と同じ掴み直しの作法）。"""

    def test_reads_back_a_written_frame(self):
        model_dir = tempfile.TemporaryDirectory()
        try:
            model_path = Path(model_dir.name) / "dummy.onnx"
            _make_dummy_model(model_path, 8, 8)
            model = SegmentationModel(str(model_path), input_size=(8, 8))
            node = CamPerceptionNode(model=model, vehicle=Vehicle.load())

            ring = FrameRing.create("surge_test_cam_perception", 16, 12, "RGB888",
                                    n_slots=4)
            try:
                data = np.zeros((12, 16, 3), dtype=np.uint8)
                data[..., 0] = 42
                desc = ring.write(data, t_capture_ns=123456789, frame_id=1)
                ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                               frame_id=desc.frame_id, width=desc.width,
                               height=desc.height, fmt=desc.fmt, stride=desc.stride,
                               nbytes=desc.nbytes, cam="front")

                got = node.read_frame(ref)
                self.assertIsNotNone(got)
                arr, t_capture = got
                self.assertTrue(np.array_equal(arr, data))
                self.assertEqual(t_capture, 123456789)
            finally:
                node.close()
                ring.unlink()
        finally:
            model_dir.cleanup()

    def test_missing_shm_returns_none_instead_of_raising(self):
        model_dir = tempfile.TemporaryDirectory()
        try:
            model_path = Path(model_dir.name) / "dummy.onnx"
            _make_dummy_model(model_path, 8, 8)
            model = SegmentationModel(str(model_path), input_size=(8, 8))
            node = CamPerceptionNode(model=model, vehicle=Vehicle.load())
            ref = ImageRef(shm_name="surge_does_not_exist", ring_seq=1)
            self.assertIsNone(node.read_frame(ref))
        finally:
            model_dir.cleanup()


if __name__ == "__main__":
    unittest.main()
