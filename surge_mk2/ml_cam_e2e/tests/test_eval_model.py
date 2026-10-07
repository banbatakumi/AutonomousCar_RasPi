"""`ml_cam_e2e/eval_model.py` の純粋関数のテスト（推論そのものは `test_pipeline.py`）。"""

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import samples as S  # noqa: E402
from eval_model import (  # noqa: E402
    EVAL_COLUMNS,
    SPLIT_TRAIN,
    SPLIT_UNUSED,
    SPLIT_VAL,
    read_eval,
    split_of,
    load_train_config,
    speed_trained,
    summarize,
    worst_segments,
)

NS = 1_000_000_000


def _row(t_s, err_deg, *, split=SPLIT_VAL, source="a.mcap", speed_err=0.0):
    return {"file": f"{t_s}.jpg", "source_mcap": source, "cam": "front",
            "t_capture_ns": int(t_s * NS), "split": split,
            "steer_true": 0.0, "steer_pred": math.radians(err_deg),
            "speed_true": 1.0, "speed_pred": 1.0 + speed_err}


class TestSpeedTrained(unittest.TestCase):
    def test_zero_weight_means_the_speed_output_is_untrained(self):
        self.assertFalse(speed_trained({"speed_weight": 0.0}))
        self.assertTrue(speed_trained({"speed_weight": 0.5}))

    def test_old_config_without_the_key_counts_as_trained(self):
        """`speed_weight` を書いていなかった頃の既定は 0.5。"""
        self.assertTrue(speed_trained({}))

    def test_missing_or_broken_config_is_an_empty_dict(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(load_train_config(Path(d)), {})
            (Path(d) / "train_config.json").write_text("{broken")
            self.assertEqual(load_train_config(Path(d)), {})
            (Path(d) / "train_config.json").write_text('{"speed_weight": 0.0}')
            self.assertFalse(speed_trained(load_train_config(Path(d))))


class TestSummarize(unittest.TestCase):
    def test_val_and_train_are_reported_separately_and_unused_is_ignored(self):
        rows = [_row(0, 2.0), _row(1, 4.0, speed_err=0.2),
                _row(2, 10.0, split=SPLIT_TRAIN), _row(3, 90.0, split=SPLIT_UNUSED)]
        got = summarize(rows)
        self.assertEqual(got[SPLIT_VAL]["n"], 2)
        self.assertAlmostEqual(got[SPLIT_VAL]["steer_mae_deg"], 3.0)
        self.assertAlmostEqual(got[SPLIT_VAL]["speed_mae"], 0.1)
        self.assertEqual(got[SPLIT_TRAIN]["n"], 1)
        self.assertAlmostEqual(got[SPLIT_TRAIN]["steer_mae_deg"], 10.0)

    def test_empty_split_is_nan_not_a_crash(self):
        got = summarize([_row(0, 1.0, split=SPLIT_TRAIN)])
        self.assertEqual(got[SPLIT_VAL]["n"], 0)
        self.assertTrue(math.isnan(got[SPLIT_VAL]["steer_mae_deg"]))


class TestWorstSegments(unittest.TestCase):
    def test_neighbouring_frames_collapse_into_one_scene(self):
        """同じコーナーの数枚が上位を占めないよう、1秒以内は1場面にまとめる。"""
        rows = [_row(10.0, 20.0), _row(10.2, 25.0), _row(10.4, 22.0),   # 同じ場面
                _row(30.0, 15.0), _row(50.0, 1.0)]
        got = worst_segments(rows, k=2, merge_s=1.0)
        self.assertEqual(len(got), 2)
        self.assertEqual(got[0]["index"], 1)                 # 場面の中で最大の1枚が代表
        self.assertAlmostEqual(got[0]["err_deg"], 25.0)
        self.assertEqual(got[0]["n_frames"], 3)
        self.assertEqual(got[1]["index"], 3)

    def test_same_time_in_another_record_is_a_different_scene(self):
        rows = [_row(10.0, 20.0), _row(10.0, 18.0, source="b.mcap")]
        self.assertEqual(len(worst_segments(rows, k=5)), 2)

    def test_unused_frames_are_not_listed(self):
        rows = [_row(0, 80.0, split=SPLIT_UNUSED), _row(5, 3.0)]
        got = worst_segments(rows, k=5)
        self.assertEqual([g["index"] for g in got], [1])


class TestSplitOf(unittest.TestCase):
    def _s(self, t_s, **kw):
        kw.setdefault("target_speed", 0.5)
        return S.Sample(path=Path("x.jpg"), source_mcap="a.mcap", cam="front",
                        t_ns=int(t_s * NS), target_steer=0.0, speed_actual=0.5,
                        brake=False, **kw)

    def test_matches_the_training_split(self):
        cfg = {"block_s": 2.0, "val_ratio": 0.3, "seed": 7}
        samples = [self._s(t) for t in range(200)]
        _train, val = S.split(samples, block_s=2.0, val_ratio=0.3, seed=7)
        val_t = {s.t_ns for s in val}
        for s in samples:
            want = SPLIT_VAL if s.t_ns in val_t else SPLIT_TRAIN
            self.assertEqual(split_of(s, [], cfg), want)

    def test_excluded_and_dropped_frames_are_unused(self):
        ex = [S.Exclusion("a.mcap", "front", 0, NS)]
        self.assertEqual(split_of(self._s(0.5), ex, {}), SPLIT_UNUSED)
        self.assertEqual(split_of(self._s(5.0, target_speed=-1.0), [], {}), SPLIT_UNUSED)


class TestReadEval(unittest.TestCase):
    def test_numbers_come_back_as_numbers(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "eval.csv"
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(EVAL_COLUMNS)
                w.writerow(["a.jpg", "a.mcap", "front", 123, SPLIT_VAL, 0.1, 0.2, 1.0, 0.9])
            (row,) = read_eval(path)
            self.assertEqual(row["t_capture_ns"], 123)
            self.assertAlmostEqual(row["steer_pred"], 0.2)
            self.assertEqual(row["split"], SPLIT_VAL)


if __name__ == "__main__":
    unittest.main()
