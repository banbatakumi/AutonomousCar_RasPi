import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.epoch_log import make_epoch_line_parser  # noqa: E402


class TestMakeEpochLineParser(unittest.TestCase):
    def test_parses_val_iou_line(self):
        parse = make_epoch_line_parser("val_iou")
        parsed = parse("epoch   3/30  loss=0.1234  val_iou=0.567  (12s)")
        self.assertEqual(parsed, (3, 0.1234, 0.567))

    def test_parses_val_mae_line(self):
        parse = make_epoch_line_parser("val_mae")
        parsed = parse("epoch   3/30  loss=0.1234  val_mae=0.045  (2.7deg, 12s)")
        self.assertEqual(parsed, (3, 0.1234, 0.045))

    def test_parses_nan_metric(self):
        parse = make_epoch_line_parser("val_mae")
        parsed = parse("epoch  30/30  loss=0.0100  val_mae=nan  (nandeg, 300s)")
        self.assertEqual(parsed, (30, 0.0100, None))

    def test_returns_none_for_unrelated_lines(self):
        parse = make_epoch_line_parser("val_iou")
        self.assertIsNone(parse("# device: cpu  学習 100件 / 検証 15件"))

    def test_metric_names_do_not_cross_match(self):
        parse_iou = make_epoch_line_parser("val_iou")
        self.assertIsNone(parse_iou("epoch   3/30  loss=0.1234  val_mae=0.045"))


if __name__ == "__main__":
    unittest.main()
