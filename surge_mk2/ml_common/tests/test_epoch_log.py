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

    def test_parses_diverged_loss_line(self):
        """lossがnan/infに発散した行を捨てない（捨てるとグラフが止まる）。"""
        import math
        parse = make_epoch_line_parser("val_iou")
        e, loss, metric = parse("epoch   4/30  loss=nan  val_iou=0.100  (3s)")
        self.assertEqual(e, 4)
        self.assertTrue(math.isnan(loss))
        self.assertEqual(metric, 0.1)
        e, loss, metric = parse("epoch   5/30  loss=inf  val_iou=nan  (3s)")
        self.assertTrue(math.isinf(loss))
        self.assertIsNone(metric)

    def test_parses_scientific_notation(self):
        parse = make_epoch_line_parser("val_mae")
        self.assertEqual(parse("epoch   1/3  loss=1.5e-05  val_mae=2e-03  (1s)"),
                         (1, 1.5e-05, 2e-03))

    def test_returns_none_for_unrelated_lines(self):
        parse = make_epoch_line_parser("val_iou")
        self.assertIsNone(parse("# device: cpu  学習 100件 / 検証 15件"))

    def test_metric_names_do_not_cross_match(self):
        parse_iou = make_epoch_line_parser("val_iou")
        self.assertIsNone(parse_iou("epoch   3/30  loss=0.1234  val_mae=0.045"))


if __name__ == "__main__":
    unittest.main()
