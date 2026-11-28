"""`raspi/core/arrow_yolo.py` と、`arrow_signal_node` の方式切替（classic/yolo）のテスト。

**学習済みの YOLO は要らない。** 出力を決め打ちで返す偽のセッション（と、`onnx` があれば
定数を返すだけの本物の ONNX）で、前処理→後処理→クラス名→方向→confirm の配管を見る。
"""

import tempfile
import time
import unittest
from pathlib import Path

import numpy as np

from raspi.bus import FrameRing
from raspi.core.arrow_yolo import YoloArrowDetector, class_to_direction, letterbox, postprocess
from raspi.msgs import ImageRef, SignalConfig, VehicleState
from raspi.msgs.types import TOPIC_IMAGE_FRONT, TOPIC_ROUTE_SELECT, TOPIC_SIGNAL_CONFIG, \
    TOPIC_SIGNAL_STATUS, TOPIC_VEHICLE_STATE
from raspi.nodes.arrow_signal_node import ArrowSignalNode
from raspi.tests.fakes import FakePub, FakeSub

IMGSZ = 320


def _raw_output(dets, n_classes=3, n_anchors=50):
    """`[(cx, cy, w, h, cls, score), ...]`（入力画像の画素）→ ultralytics の既定出力 (1, 4+nc, N)。"""
    out = np.zeros((1, 4 + n_classes, n_anchors), dtype=np.float32)
    for i, (cx, cy, w, h, c, sc) in enumerate(dets):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + c, i] = sc
    return out


class _Inp:
    name = "images"
    shape = [1, 3, IMGSZ, IMGSZ]


class _Meta:
    def __init__(self, names):
        self.custom_metadata_map = {"names": repr(dict(enumerate(names)))}


class FakeSession:
    def __init__(self, out, names=("left", "right", "straight")):
        self.out = out
        self.names = names
        self.calls = 0

    def get_inputs(self):
        return [_Inp()]

    def get_modelmeta(self):
        return _Meta(self.names)

    def run(self, _outs, feeds):
        x = feeds["images"]
        assert x.shape == (1, 3, IMGSZ, IMGSZ) and x.dtype == np.float32
        self.calls += 1
        return [self.out]


def _detector(out, names=("left", "right", "straight"), **kw):
    det = YoloArrowDetector(min_period_s=0.0, **kw)
    assert det.load(Path("m.onnx"), session=FakeSession(out, names)), det.error
    return det


class TestHelpers(unittest.TestCase):
    def test_class_to_direction(self):
        self.assertEqual(class_to_direction("Left"), "left")
        self.assertEqual(class_to_direction("center"), "straight")
        self.assertIsNone(class_to_direction("left_dark"))

    def test_letterbox_keeps_aspect(self):
        img, scale, (px, py) = letterbox(np.zeros((240, 320, 3), np.uint8), (IMGSZ, IMGSZ))
        self.assertEqual(img.shape, (IMGSZ, IMGSZ, 3))
        self.assertAlmostEqual(scale, 1.0)
        self.assertEqual((px, py), (0, 40))

    def test_postprocess_nms_and_order(self):
        out = _raw_output([(100, 100, 40, 40, 0, 0.6), (102, 101, 40, 40, 0, 0.9),
                           (250, 100, 40, 40, 1, 0.7), (50, 50, 10, 10, 2, 0.1)])
        dets = postprocess(out, conf_min=0.5, n_classes=3)
        self.assertEqual([d[5] for d in dets], [0, 1])          # 重なりは1つに、低スコアは捨てる
        self.assertAlmostEqual(dets[0][4], 0.9, places=5)

    def test_postprocess_nms_exported_format(self):
        out = np.array([[[10, 10, 50, 50, 0.8, 1], [0, 0, 5, 5, 0.2, 0]]], np.float32)
        dets = postprocess(out, conf_min=0.5, n_classes=3)
        self.assertEqual(len(dets), 1)
        self.assertEqual(dets[0][5], 1)


class TestYoloArrowDetector(unittest.TestCase):
    def test_best_detection_decides_direction_and_box_is_normalized(self):
        # 240x320 の画像は 320x320 へ上下 40px の余白付きで入る
        det = _detector(_raw_output([(80, 40 + 60, 40, 40, 1, 0.9), (200, 100, 40, 40, 0, 0.6)]))
        r = det.process_frame(np.zeros((240, 320, 3), np.uint8))
        self.assertEqual(r.value, "right")
        self.assertEqual(r.label, "right")
        self.assertAlmostEqual(r.score, 0.9, places=5)
        x0, y0, x1, y1 = r.box
        self.assertAlmostEqual(x0, 60 / 320, places=4)
        self.assertAlmostEqual(y0, 40 / 240, places=4)
        self.assertAlmostEqual(y1, 80 / 240, places=4)

    def test_non_direction_class_on_top_is_not_applicable(self):
        det = _detector(_raw_output([(160, 100, 40, 40, 3, 0.95), (160, 100, 40, 40, 0, 0.6)],
                                    n_classes=4),
                        names=("left", "right", "straight", "left_dark"))
        r = det.process_frame(np.zeros((240, 320, 3), np.uint8))
        self.assertIsNone(r.value)
        self.assertEqual(r.label, "left_dark")

    def test_detection_outside_roi_is_dropped(self):
        det = _detector(_raw_output([(160, 40 + 220, 40, 30, 0, 0.9)]), roi_band=(0.0, 0.5))
        self.assertIsNone(det.process_frame(np.zeros((240, 320, 3), np.uint8)).value)

    def test_below_conf_is_nothing(self):
        det = _detector(_raw_output([(160, 100, 40, 40, 0, 0.4)]), conf_min=0.5)
        self.assertIsNone(det.process_frame(np.zeros((240, 320, 3), np.uint8)).value)

    def test_model_without_direction_classes_fails_to_load(self):
        det = YoloArrowDetector()
        self.assertFalse(det.load(Path("m.onnx"), session=FakeSession(_raw_output([]),
                                                                      names=("cat", "dog", "x"))))
        self.assertFalse(det.ready)
        self.assertIn("left/right/straight", det.error)

    def test_due_throttles(self):
        t = [0.0]
        det = YoloArrowDetector(min_period_s=0.1, clock=lambda: t[0])
        det.load(Path("m.onnx"), session=FakeSession(_raw_output([])))
        det.process_frame(np.zeros((240, 320, 3), np.uint8))
        self.assertFalse(det.due())
        t[0] = 0.11
        self.assertTrue(det.due())


class TestRealOnnx(unittest.TestCase):
    """`onnx` があれば、定数を返す本物の ONNX を書いて `make_session`・メタデータの読み口まで通す。"""

    def test_constant_model_end_to_end(self):
        try:
            import onnx
            from onnx import TensorProto, helper, numpy_helper
        except ModuleNotFoundError:
            self.skipTest("onnx が無い（ダミーモデルを書くだけの道具。Pi には入れない）")
        out = _raw_output([(60, 100, 40, 40, 0, 0.9)])
        x = helper.make_tensor_value_info("images", TensorProto.FLOAT, [1, 3, IMGSZ, IMGSZ])
        y = helper.make_tensor_value_info("output0", TensorProto.FLOAT, list(out.shape))
        # 入力を使わないと消されるので、入力の総和×0 を定数に足す
        nodes = [helper.make_node("ReduceSum", ["images"], ["s"], keepdims=0),
                 helper.make_node("Mul", ["s", "zero"], ["z"]),
                 helper.make_node("Add", ["c", "z"], ["output0"])]
        g = helper.make_graph(nodes, "g", [x], [y], initializer=[
            numpy_helper.from_array(out, "c"),
            numpy_helper.from_array(np.float32(0.0), "zero")])
        m = helper.make_model(g, opset_imports=[helper.make_opsetid("", 13)])
        m.ir_version = 8
        helper.set_model_props(m, {"names": "{0: 'left', 1: 'right', 2: 'straight'}"})
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "arrow.onnx"
            onnx.save(m, p)
            det = YoloArrowDetector(min_period_s=0.0)
            self.assertTrue(det.load(p), det.error)
            self.assertEqual(det.names, ["left", "right", "straight"])
            r = det.process_frame(np.zeros((240, 320, 3), np.uint8))
            self.assertEqual(r.value, "left")


class TestNodeMethodSwitch(unittest.TestCase):
    """`SignalConfig.method` で classic/yolo を切り替える配管。"""

    def _ref(self, name):
        frame = np.full((240, 320, 3), 20, np.uint8)       # 古典CVでは何も点灯しない画像
        ring = FrameRing.create(name, 320, 240, "BGR888", n_slots=2)
        desc = ring.write(frame, t_capture_ns=time.monotonic_ns(), frame_id=1)
        ref = ImageRef(shm_name=ring.name, slot=desc.slot, ring_seq=desc.seq,
                       frame_id=desc.frame_id, width=desc.width, height=desc.height,
                       fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes, cam="front")
        return ring, ref

    def _node(self, d):
        (Path(d) / "arrow.onnx").write_bytes(b"x")          # 実在チェック用（中身は偽セッション）
        yolo = YoloArrowDetector(min_period_s=0.0)
        sess = FakeSession(_raw_output([(60, 100, 40, 40, 0, 0.9)]))
        orig = yolo.load
        yolo.load = lambda path, session=None: orig(path, session=sess)
        return ArrowSignalNode(confirm_frames=1, yolo=yolo, models_dir=d), sess

    def _run(self, node, cfg, ring_name):
        ring, ref = self._ref(ring_name)
        try:
            sub = FakeSub({TOPIC_IMAGE_FRONT: ref, TOPIC_VEHICLE_STATE: VehicleState(armed=True),
                           TOPIC_SIGNAL_CONFIG: cfg})
            pub = FakePub()
            node.run(sub=sub, pub=pub, duration_s=0.05)
            return pub
        finally:
            node.close()
            ring.unlink()

    def test_yolo_method_publishes_route_select(self):
        with tempfile.TemporaryDirectory() as d:
            node, sess = self._node(d)
            pub = self._run(node, SignalConfig(method="yolo", yolo_model="arrow"),
                            "surge_test_arrow_yolo_on")
            self.assertGreater(sess.calls, 0)
            routes = [m for t, m in pub.sent if t == TOPIC_ROUTE_SELECT]
            self.assertTrue(routes and routes[-1].value == "left")
            st = [m for t, m in pub.sent if t == TOPIC_SIGNAL_STATUS][-1]
            self.assertEqual(st.method, "yolo")
            self.assertEqual(st.label, "left")
            self.assertGreater(st.box[2], st.box[0])

    def test_classic_method_does_not_load_or_run_yolo(self):
        with tempfile.TemporaryDirectory() as d:
            node, sess = self._node(d)
            pub = self._run(node, SignalConfig(method="classic", yolo_model="arrow"),
                            "surge_test_arrow_yolo_off")
            self.assertEqual(sess.calls, 0)
            self.assertFalse(node.yolo.ready)
            self.assertFalse([m for t, m in pub.sent if t == TOPIC_ROUTE_SELECT])

    def test_missing_model_falls_back_to_classic_and_reports_error(self):
        with tempfile.TemporaryDirectory() as d:
            node, _ = self._node(d)
            pub = self._run(node, SignalConfig(method="yolo", yolo_model="nope"),
                            "surge_test_arrow_yolo_missing")
            st = [m for t, m in pub.sent if t == TOPIC_SIGNAL_STATUS][-1]
            self.assertEqual(st.method, "classic")
            self.assertIn("モデルが無い", st.error)


if __name__ == "__main__":
    unittest.main()
