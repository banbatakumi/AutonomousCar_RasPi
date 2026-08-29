"""`ml_cam/annotate.py` のテスト。

**実物の SAM チェックポイント・cv2 のウィンドウは要らない。** `AnnotationSession`
は SAM をダックタイピングで受け取るだけなので、`set_image`/`predict` を
持つ偽物（`FakePredictor`）を渡して、点の追加・取り消し・マスク確定という
状態機械の部分だけを検証する。cv2 のマウス/キー入力ループ（`main()`）は
対話的なので、ここではテストしない。
"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam/

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from annotate import AnnotationSession, save_mask  # noqa: E402


class FakePredictor:
    """`SamPredictor` の身代わり。**点が右半分にあれば右半分を返す**という
    単純なルールで、`AnnotationSession` が SAM の戻り値をどう使うかだけを試す。
    """

    def __init__(self) -> None:
        self.image = None
        self.set_image_calls = 0

    def set_image(self, image_rgb) -> None:
        self.image = image_rgb
        self.set_image_calls += 1

    def predict(self, *, point_coords, point_labels, multimask_output=True):
        h, w = self.image.shape[:2]
        # 前景点（label=1）の平均 x 座標で左右どちらを選ぶか決める
        fg = point_coords[point_labels == 1]
        mask = np.zeros((h, w), dtype=bool)
        if len(fg) == 0:
            masks = np.zeros((3, h, w), dtype=bool)
            scores = np.array([0.1, 0.1, 0.1])
            return masks, scores, None
        mean_x = fg[:, 0].mean()
        if mean_x < w / 2:
            mask[:, : w // 2] = True
        else:
            mask[:, w // 2:] = True
        # 背景点（label=0）は選んだ領域から除外する
        for (x, y), lb in zip(point_coords, point_labels):
            if lb == 0:
                mask[y, x] = False
        masks = np.stack([mask, mask, mask])
        scores = np.array([0.9, 0.5, 0.2])          # 1番目（=index0）が最良
        return masks, scores, None


def _image(w=20, h=10) -> np.ndarray:
    return np.zeros((h, w, 3), dtype=np.uint8)


class TestAnnotationSession(unittest.TestCase):
    def test_no_points_means_no_mask_yet(self):
        session = AnnotationSession(FakePredictor())
        session.load_image(_image())
        self.assertIsNone(session.predict_mask())

    def test_load_image_calls_set_image_and_clears_points(self):
        predictor = FakePredictor()
        session = AnnotationSession(predictor)
        session.load_image(_image())
        session.add_point(2, 2)
        session.load_image(_image())                # 次のフレームへ
        self.assertEqual(predictor.set_image_calls, 2)
        self.assertEqual(session.points, [])

    def test_foreground_point_picks_the_corresponding_side(self):
        session = AnnotationSession(FakePredictor())
        session.load_image(_image(w=20, h=10))
        session.add_point(3, 5, foreground=True)      # 左側
        mask = session.predict_mask()
        self.assertTrue(mask[:, :10].all())
        self.assertFalse(mask[:, 10:].any())

    def test_background_point_carves_out_of_the_mask(self):
        session = AnnotationSession(FakePredictor())
        session.load_image(_image(w=20, h=10))
        session.add_point(3, 5, foreground=True)
        session.add_point(3, 5, foreground=False)     # 同じ点を除外指定
        mask = session.predict_mask()
        self.assertFalse(mask[5, 3])

    def test_undo_removes_the_last_point(self):
        session = AnnotationSession(FakePredictor())
        session.load_image(_image(w=20, h=10))
        session.add_point(3, 5, foreground=True)       # 左を選ぶ点
        session.add_point(15, 5, foreground=True)      # 右を選ぶ点（平均が右へ寄る）
        session.undo()
        mask = session.predict_mask()
        # 右の点を取り消したので、残るのは左の点だけ → 左側が選ばれる
        self.assertTrue(mask[:, :10].all())

    def test_seed_replays_points_after_load_image(self):
        session = AnnotationSession(FakePredictor())
        session.load_image(_image(w=20, h=10))
        session.add_point(3, 5, foreground=True)
        session.add_point(15, 2, foreground=False)
        carried_points = list(session.points)
        carried_labels = list(session.labels)

        session.load_image(_image(w=20, h=10))            # 次のフレームへ（クリアされる）
        self.assertEqual(session.points, [])

        session.seed(carried_points, carried_labels)
        self.assertEqual(session.points, carried_points)
        self.assertEqual(session.labels, carried_labels)

    def test_clear_resets_points(self):
        session = AnnotationSession(FakePredictor())
        session.load_image(_image())
        session.add_point(1, 1)
        session.clear()
        self.assertEqual(session.points, [])
        self.assertEqual(session.labels, [])


class TestSaveMask(unittest.TestCase):
    def test_round_trips_as_0_255_png(self):
        mask = np.zeros((10, 20), dtype=bool)
        mask[:, :10] = True
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "a_mask.png"
            save_mask(mask, out)
            loaded = np.asarray(Image.open(out).convert("L"))
            self.assertEqual(set(np.unique(loaded).tolist()), {0, 255})
            self.assertTrue((loaded[:, :10] == 255).all())
            self.assertTrue((loaded[:, 10:] == 0).all())


if __name__ == "__main__":
    unittest.main()
