"""`ml_cam/preview.py` のテスト。

**実物のONNXモデル・cv2のウィンドウは要らない。** `compute_iou`・
`compare_masks`・`load_model_config` という、numpy配列とファイルI/Oだけの
純粋関数を検証する（`main()` の cv2 ループは `ml_cam/annotate.py` と同じ理由で
テストしない）。
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))          # repo root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))          # ml_cam/

import numpy as np  # noqa: E402

from preview import compare_masks, compute_iou, load_model_config  # noqa: E402


class TestComputeIou(unittest.TestCase):
    def test_perfect_match_is_one(self):
        mask = np.zeros((10, 10), dtype=bool)
        mask[:, :5] = True
        self.assertEqual(compute_iou(mask, mask), 1.0)

    def test_both_empty_counts_as_match(self):
        empty = np.zeros((10, 10), dtype=bool)
        self.assertEqual(compute_iou(empty, empty), 1.0)

    def test_partial_overlap(self):
        pred = np.zeros((10, 10), dtype=bool)
        pred[:, :6] = True                       # 60列
        gt = np.zeros((10, 10), dtype=bool)
        gt[:, 4:10] = True                       # 60列、4〜5が重なる
        # 交差=2列*10行=20, 和=10列*10行=100（6+6-2重複） → IoU=0.2
        self.assertAlmostEqual(compute_iou(pred, gt), 0.2)


class TestCompareMasks(unittest.TestCase):
    def test_splits_into_tp_fp_fn(self):
        pred = np.array([[True, True, False]])
        gt = np.array([[True, False, True]])
        tp, fp, fn = compare_masks(pred, gt)
        self.assertTrue((tp == np.array([[True, False, False]])).all())
        self.assertTrue((fp == np.array([[False, True, False]])).all())
        self.assertTrue((fn == np.array([[False, False, True]])).all())


class TestLoadModelConfig(unittest.TestCase):
    def test_reads_sibling_json(self):
        with tempfile.TemporaryDirectory() as d:
            onnx_path = Path(d) / "model.onnx"
            (Path(d) / "model.json").write_text(json.dumps({"input_size": [64, 48]}))
            cfg = load_model_config(onnx_path)
            self.assertEqual(cfg["input_size"], [64, 48])

    def test_missing_json_returns_empty_dict(self):
        with tempfile.TemporaryDirectory() as d:
            cfg = load_model_config(Path(d) / "model.onnx")
            self.assertEqual(cfg, {})


if __name__ == "__main__":
    unittest.main()
