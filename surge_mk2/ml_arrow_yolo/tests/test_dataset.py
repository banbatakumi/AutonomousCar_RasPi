"""`ml_arrow_yolo/dataset.py` のテスト（ラベルの読み書き・確定の約束・記録ごとの分割）。"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dataset import (  # noqa: E402
    DEFAULT_CLASSES,
    Box,
    RunPaths,
    format_label_text,
    label_stats,
    list_images,
    parse_label_text,
    read_classes,
    read_labels,
    source_of,
    split_by_source,
    write_classes,
    write_dataset_yaml,
    write_labels,
)


def _touch_images(paths: RunPaths, names):
    paths.images.mkdir(parents=True, exist_ok=True)
    out = []
    for n in names:
        p = paths.images / f"{n}.jpg"
        p.write_bytes(b"\xff\xd8\xff\xd9")
        out.append(p)
    return out


class TestLabelText(unittest.TestCase):
    def test_round_trip(self):
        boxes = [Box(1, 0.1, 0.2, 0.3, 0.6), Box(0, 0.5, 0.5, 0.9, 0.7)]
        back = parse_label_text(format_label_text(boxes))
        self.assertEqual([b.cls for b in back], [1, 0])
        for a, b in zip(boxes, back):
            for k in ("x0", "y0", "x1", "y1"):
                self.assertAlmostEqual(getattr(a, k), getattr(b, k), places=5)

    def test_yolo_format_is_center_and_size(self):
        self.assertEqual(format_label_text([Box(2, 0.2, 0.4, 0.6, 0.8)]).split(),
                         ["2", "0.400000", "0.600000", "0.400000", "0.400000"])

    def test_reversed_corners_and_out_of_range_are_fixed(self):
        b = Box(0, 0.8, 1.2, 0.2, -0.1).normalized()
        self.assertEqual((b.x0, b.y0, b.x1, b.y1), (0.2, 0.0, 0.8, 1.0))

    def test_broken_lines_are_skipped(self):
        self.assertEqual(len(parse_label_text("0 0.5 0.5 0.1\nx 1 2 3 4\n1 0.5 0.5 0.2 0.2\n")), 1)


class TestConfirmedMeansFileExists(unittest.TestCase):
    def test_missing_is_none_empty_is_background(self):
        with tempfile.TemporaryDirectory() as d:
            paths = RunPaths(Path(d))
            img = _touch_images(paths, ["a__1"])[0]
            self.assertIsNone(read_labels(paths.label_path(img)))
            write_labels(paths.label_path(img), [])
            self.assertEqual(read_labels(paths.label_path(img)), [])

    def test_stats(self):
        with tempfile.TemporaryDirectory() as d:
            paths = RunPaths(Path(d))
            a, b, c = _touch_images(paths, ["a__1", "a__2", "a__3"])
            write_labels(paths.label_path(a), [Box(0, 0, 0, 0.1, 0.1), Box(1, 0, 0, 0.1, 0.1)])
            write_labels(paths.label_path(b), [])
            write_labels(paths.draft_path(c), [Box(2, 0, 0, 0.1, 0.1)])
            st = label_stats(paths, 3)
            self.assertEqual((st["images"], st["labeled"], st["background"], st["drafts"]), (3, 2, 1, 1))
            self.assertEqual(st["per_class"], [1, 1, 0])          # 下書きは数えない


class TestClasses(unittest.TestCase):
    def test_default_and_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            paths = RunPaths(Path(d))
            self.assertEqual(read_classes(paths), DEFAULT_CLASSES)
            write_classes(paths, ["left", "right", "straight", "left_dark"])
            self.assertEqual(read_classes(paths)[-1], "left_dark")


class TestSplit(unittest.TestCase):
    def test_split_keeps_each_recording_on_one_side(self):
        imgs = [Path(f"/x/images/{s}__{t:03d}.jpg") for s in ("rec_a", "rec_b", "rec_c")
                for t in range(10)]
        train, val = split_by_source(imgs, 0.3)
        self.assertEqual(len(train) + len(val), 30)
        self.assertFalse({source_of(p) for p in train} & {source_of(p) for p in val})
        self.assertTrue(train)

    def test_single_recording_splits_by_time(self):
        imgs = [Path(f"/x/images/rec__{t:03d}.jpg") for t in range(10)]
        train, val = split_by_source(imgs, 0.2)
        self.assertEqual(len(val), 2)
        self.assertLess(max(train), min(val))

    def test_zero_val_uses_train_as_val(self):
        imgs = [Path("/x/images/rec__1.jpg")]
        self.assertEqual(split_by_source(imgs, 0.0), (imgs, imgs))


class TestDatasetYaml(unittest.TestCase):
    def test_yaml_lists_and_names(self):
        with tempfile.TemporaryDirectory() as d:
            paths = RunPaths(Path(d))
            imgs = _touch_images(paths, ["a__1", "b__1"])
            y = write_dataset_yaml(paths, imgs[:1], imgs[1:], ["left", "right"])
            text = y.read_text()
            self.assertIn("  1: right", text)
            self.assertIn("/images/", (y.parent / "train.txt").read_text())
            self.assertEqual(list_images(paths), imgs)


if __name__ == "__main__":
    unittest.main()
