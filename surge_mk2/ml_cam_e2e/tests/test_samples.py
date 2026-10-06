"""`ml_cam_e2e/samples.py` のテスト（torch 不要の純粋関数）。"""

import csv
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_cam_e2e/

import samples as S  # noqa: E402

NS = 1_000_000_000


def _s(t_s=0.0, *, steer=0.0, speed=0.5, actual=0.5, brake=False, source="a.mcap",
       cam="front") -> S.Sample:
    return S.Sample(path=Path("x.jpg"), source_mcap=source, cam=cam, t_ns=int(t_s * NS),
                    target_steer=steer, target_speed=speed, speed_actual=actual, brake=brake)


class TestLoadManifest(unittest.TestCase):
    def test_reads_sorts_dedupes_and_skips_missing_files(self):
        with tempfile.TemporaryDirectory() as d:
            frames = Path(d)
            for name in ("a.jpg", "b.jpg"):
                (frames / name).write_bytes(b"x")
            with open(frames / "manifest.csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(S.MANIFEST_COLUMNS)
                w.writerow(["b.jpg", "run.mcap", "front", "200", "0.1", "0.3", "0.25", "0"])
                w.writerow(["a.jpg", "run.mcap", "front", "100", "0.0", "0.0", "", "1"])
                w.writerow(["gone.jpg", "run.mcap", "front", "50", "0.0", "0.0", "", "0"])
                w.writerow(["b.jpg", "run.mcap", "front", "200", "0.2", "0.3", "0.25", "0"])

            got = S.load_manifest(frames)
            self.assertEqual([s.path.name for s in got], ["a.jpg", "b.jpg"])
            self.assertTrue(got[0].brake)
            self.assertTrue(math.isnan(got[0].speed_actual))
            self.assertAlmostEqual(got[1].target_steer, 0.2, msg="同じファイルは後勝ち")
            self.assertAlmostEqual(got[1].speed_actual, 0.25)

    def test_missing_manifest_returns_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(S.load_manifest(Path(d)), [])


class TestExclusions(unittest.TestCase):
    def test_round_trip_with_japanese_note(self):
        with tempfile.TemporaryDirectory() as d:
            ex = [S.Exclusion("b.mcap", "front", 5, 9, "コースアウト"),
                  S.Exclusion("a.mcap", "front", 1, 2)]
            S.save_exclusions(Path(d), ex)
            got = S.load_exclusions(Path(d))
            self.assertEqual(got, sorted(ex, key=lambda e: e.source_mcap))

    def test_no_file_means_no_exclusions(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(S.load_exclusions(Path(d)), [])

    def test_range_is_inclusive_and_scoped_to_record_and_camera(self):
        ex = [S.Exclusion("a.mcap", "front", 1 * NS, 2 * NS)]
        self.assertTrue(S.is_excluded(_s(1.0), ex))
        self.assertTrue(S.is_excluded(_s(2.0), ex))
        self.assertFalse(S.is_excluded(_s(2.1), ex))
        self.assertFalse(S.is_excluded(_s(1.5, source="b.mcap"), ex))
        self.assertFalse(S.is_excluded(_s(1.5, cam="rear"), ex))


class TestDropReason(unittest.TestCase):
    def test_normal_driving_is_kept(self):
        self.assertIsNone(S.drop_reason(_s(speed=0.5, actual=0.5)))

    def test_reverse_is_dropped(self):
        self.assertEqual(S.drop_reason(_s(speed=-0.3, actual=-0.3)), "後退")

    def test_waiting_at_standstill_is_dropped(self):
        self.assertEqual(S.drop_reason(_s(speed=0.0, actual=0.0)), "静止")
        self.assertEqual(S.drop_reason(_s(speed=0.5, actual=0.0, brake=True)), "静止")

    def test_lifting_off_while_moving_is_kept(self):
        """走行中にスロットルを抜いた瞬間は「ここで減速」の手本。"""
        self.assertIsNone(S.drop_reason(_s(speed=0.0, actual=0.8)))
        self.assertIsNone(S.drop_reason(_s(speed=0.5, actual=0.8, brake=True)))

    def test_launch_from_standstill_is_kept(self):
        self.assertIsNone(S.drop_reason(_s(speed=0.5, actual=0.0)))

    def test_unknown_actual_speed_falls_back_to_the_command(self):
        self.assertEqual(S.drop_reason(_s(speed=0.0, actual=math.nan)), "静止")
        self.assertIsNone(S.drop_reason(_s(speed=0.5, actual=math.nan)))

    def test_usable_combines_both(self):
        ex = [S.Exclusion("a.mcap", "front", 0, NS)]
        self.assertFalse(S.usable(_s(0.5), ex))
        self.assertTrue(S.usable(_s(1.5), ex))
        self.assertFalse(S.usable(_s(1.5, speed=-1.0), ex))


class TestSplit(unittest.TestCase):
    def test_frames_in_the_same_block_never_straddle_train_and_val(self):
        samples = [_s(t / 10) for t in range(3000)]            # 300秒・10Hz
        train, val = S.split(samples, block_s=5.0, val_ratio=0.2, seed=0)
        blocks_train = {s.t_ns // (5 * NS) for s in train}
        blocks_val = {s.t_ns // (5 * NS) for s in val}
        self.assertFalse(blocks_train & blocks_val)
        self.assertEqual(len(train) + len(val), len(samples))
        self.assertTrue(0.05 < len(val) / len(samples) < 0.4)

    def test_assignment_is_stable_when_data_is_added(self):
        """データを足しても既存フレームの振り分けが動かない（評価が再現できる）。"""
        a = [_s(t / 10) for t in range(1000)]
        b = [_s(t / 10, source="b.mcap") for t in range(1000)]
        before = {s.t_ns for s in S.split(a)[1]}
        after = {s.t_ns for s in S.split(a + b)[1] if s.source_mcap == "a.mcap"}
        self.assertEqual(before, after)

    def test_seed_and_record_name_change_the_assignment(self):
        flags = [S.is_val("a.mcap", b * 5 * NS, seed=0) for b in range(200)]
        self.assertNotEqual(flags, [S.is_val("a.mcap", b * 5 * NS, seed=1) for b in range(200)])
        self.assertNotEqual(flags, [S.is_val("b.mcap", b * 5 * NS, seed=0) for b in range(200)])


class TestLabels(unittest.TestCase):
    def test_steer_norm_is_clamped(self):
        self.assertAlmostEqual(S.steer_norm(_s(steer=0.262), 0.524), 0.5)
        self.assertEqual(S.steer_norm(_s(steer=-9.0), 0.524), -1.0)

    def test_speed_norm_is_zero_when_braking_and_clamped(self):
        self.assertAlmostEqual(S.speed_norm(_s(speed=0.5), 2.0), 0.25)
        self.assertEqual(S.speed_norm(_s(speed=0.5, brake=True), 2.0), 0.0)
        self.assertEqual(S.speed_norm(_s(speed=5.0), 2.0), 1.0)

    def test_auto_speed_ref_is_the_fastest_label_with_a_floor(self):
        self.assertAlmostEqual(S.auto_speed_ref([_s(speed=0.4), _s(speed=1.3)]), 1.3)
        self.assertAlmostEqual(S.auto_speed_ref([_s(speed=0.01)]), 0.1)
        self.assertAlmostEqual(S.auto_speed_ref([_s(speed=3.0, brake=True)]), 0.1)


class TestBalance(unittest.TestCase):
    def test_histogram_counts_and_clamps_the_edges(self):
        self.assertEqual(S.histogram([-1.0, -0.9, 0.0, 1.0, 2.0], -1.0, 1.0, 4), [2, 0, 1, 2])

    def test_alpha_zero_is_uniform_weights(self):
        w = S.balance_weights([0.0] * 8 + [0.9] * 2, alpha=0.0)
        self.assertEqual(len(set(w)), 1)

    def test_alpha_one_gives_every_bin_the_same_total_mass(self):
        norms = [0.0] * 90 + [0.9] * 10
        w = S.balance_weights(norms, alpha=1.0)
        self.assertAlmostEqual(sum(w[:90]), sum(w[90:]))

    def test_rare_steering_is_weighted_up(self):
        w = S.balance_weights([0.0] * 90 + [0.9] * 10, alpha=0.5)
        self.assertGreater(w[-1], w[0])

    def test_balanced_histogram_keeps_the_total_and_flattens(self):
        counts = [90, 10, 0]
        out = S.balanced_histogram(counts, 0.5)
        self.assertAlmostEqual(sum(out), 100.0)
        self.assertLess(out[0], 90)
        self.assertGreater(out[1], 10)
        self.assertEqual(out[2], 0.0)
        self.assertEqual([round(v) for v in S.balanced_histogram(counts, 0.0)], counts)


class TestPerRecordStats(unittest.TestCase):
    def test_counts_per_record(self):
        samples = ([_s(t, steer=0.0) for t in range(6)]                       # 直進6
                   + [_s(6, steer=0.3), _s(7, steer=-0.3), _s(8, speed=-1.0)]  # 左1 右1 後退1
                   + [_s(0, source="b.mcap")])
        ex = [S.Exclusion("a.mcap", "front", 0, 1 * NS)]                       # 先頭2枚
        rows = S.per_record_stats(samples, ex, 0.524)
        self.assertEqual([r["source_mcap"] for r in rows], ["a.mcap", "b.mcap"])
        a = rows[0]
        self.assertEqual((a["total"], a["excluded"], a["dropped"], a["used"]), (9, 2, 1, 6))
        self.assertEqual((a["left"], a["right"]), (1, 1))
        self.assertAlmostEqual(a["straight_ratio"], 4 / 6)
        self.assertAlmostEqual(a["duration_s"], 8.0)


if __name__ == "__main__":
    unittest.main()
