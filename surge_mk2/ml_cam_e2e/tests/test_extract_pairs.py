"""`ml_cam_e2e/extract_pairs.py` のテスト。

`ml_cam/tests/test_extract_frames.py` と同じく、`raspi/rec/mcap_log.py`
（Pi側の書き手）で実際に `.mcap` を作り、読み戻して検証する往復テスト。
画像に加えて `/cmd`（`DriveCmd`）も同じ書き手で書く。
"""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))          # repo root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))          # ml_cam_e2e/

from extract_pairs import (  # noqa: E402
    CMD_MCAP_TOPIC,
    STATE_MCAP_TOPIC,
    check_manifest_header,
    extract_one,
    load_cmd_series,
    nearest_cmd,
    reject_reason,
)
from raspi.msgs.types import TOPIC_CMD, DriveCmd, VehicleState  # noqa: E402
from samples import MANIFEST_COLUMNS, load_manifest  # noqa: E402
from raspi.rec.mcap_log import McapLog  # noqa: E402


def _tiny_jpeg() -> bytes:
    return b"\xff\xd8\xff\xe0mock-jpeg-bytes\xff\xd9"


class TestTopicConstant(unittest.TestCase):
    def test_cmd_mcap_topic_matches_the_real_writer(self):
        """`_mcap_topic()`（`raspi/rec/mcap_log.py`）は先頭に `/` を付けるだけ。
        `extract_pairs.py` 側で独自に再定義した値がズレていないこと。"""
        self.assertEqual(CMD_MCAP_TOPIC, "/" + TOPIC_CMD)


class TestLoadCmdSeries(unittest.TestCase):
    def test_reads_and_sorts_by_time(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=0.2), t_mono_ns=20_000_000)
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=0.1), t_mono_ns=10_000_000)

            times, cmds = load_cmd_series(mcap_path)
            self.assertEqual(times, sorted(times))
            self.assertEqual([c["target_steer"] for c in cmds], [0.1, 0.2])


class TestNearestCmd(unittest.TestCase):
    def test_picks_the_closer_of_the_two_neighbors(self):
        times = [0, 100, 300]
        cmds = [{"v": 0}, {"v": 100}, {"v": 300}]
        self.assertEqual(nearest_cmd(times, cmds, 80, max_gap_ns=1000)["v"], 100)
        self.assertEqual(nearest_cmd(times, cmds, 40, max_gap_ns=1000)["v"], 0)

    def test_returns_none_beyond_max_gap(self):
        times = [0]
        cmds = [{"v": 0}]
        self.assertIsNone(nearest_cmd(times, cmds, 1000, max_gap_ns=500))

    def test_returns_none_for_empty_series(self):
        self.assertIsNone(nearest_cmd([], [], 0, max_gap_ns=500))


class TestExtractOne(unittest.TestCase):
    def test_pairs_frames_with_the_nearest_manual_armed_command(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            jpg = _tiny_jpeg()
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=0.15,
                                              target_speed=0.4), t_mono_ns=0)
                log.write_viz_image(jpg, "front", t_mono_ns=5_000_000)   # 5ms後

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                w.writerow(MANIFEST_COLUMNS)
                n, _ = extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=100_000_000)

            self.assertEqual(n, 1)
            with open(out_dir / "manifest.csv") as mf:
                rows = list(csv.DictReader(mf))
            self.assertEqual(float(rows[0]["target_steer"]), 0.15)
            self.assertEqual(float(rows[0]["target_speed"]), 0.4)

    def test_excludes_frames_where_arm_is_false(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=False, target_steer=0.15), t_mono_ns=0)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=0)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                n, _ = extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=100_000_000)
            self.assertEqual(n, 0, "ARMしていない指令のフレームは除外されるはず")

    def test_include_auto_extracts_frames_during_auto_mode(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=2, arm=True, target_steer=0.15,
                                                   target_speed=1.2,
                                                   source="auto:slam2d_route"), t_mono_ns=0)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=0)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                w.writerow(MANIFEST_COLUMNS)
                n, _ = extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=100_000_000,
                                   include_auto=True)
            self.assertEqual(n, 1)
            (sample,) = load_manifest(out_dir)
            self.assertAlmostEqual(sample.target_speed, 1.2)

    def test_excludes_frames_during_auto_mode(self):
        """`mode=AUTO`（既存 planner の出力）は人間の操作ではないので教師データにしない。"""
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=2, arm=True, target_steer=0.15), t_mono_ns=0)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=0)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                n, _ = extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=100_000_000)
            self.assertEqual(n, 0, "AUTO中の指令のフレームは除外されるはず")

    def test_excludes_frames_beyond_max_gap(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=0.15), t_mono_ns=0)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=500_000_000)   # 500ms後

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                n, _ = extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=100_000_000)
            self.assertEqual(n, 0, "時刻差が許容を超えるフレームは除外されるはず")

    def test_min_interval_thins_valid_pairs_only(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            jpg = _tiny_jpeg()
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=0.1), t_mono_ns=0)
                log.write_viz_image(jpg, "front", t_mono_ns=0)
                log.write_viz_image(jpg, "front", t_mono_ns=100_000_000)   # +100ms → 間引かれる
                log.write_viz_image(jpg, "front", t_mono_ns=250_000_000)   # +250ms → 採用

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                n, _ = extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=1_000_000_000,
                                   min_interval_ns=200_000_000)
            self.assertEqual(n, 2)


    def test_excludes_torque_mode_and_counts_the_reason(self):
        """トルクモード中は `target_speed` が 0 のまま。速度の手本にならない。"""
        from collections import Counter

        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, torque_mode=True,
                                                   target_torque=0.05), t_mono_ns=0)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=0)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            rejects = Counter()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                n, _ = extract_one(mcap_path, out_dir, {"front"}, csv.writer(mf),
                                   max_gap_ns=100_000_000, rejects=rejects)
            self.assertEqual(n, 0)
            self.assertEqual(rejects["トルクモード"], 1)

    def test_writes_actual_speed_and_brake_readable_by_samples(self):
        """書いた manifest を `samples.load_manifest()` がそのまま読めること。"""
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=-0.1,
                                                   target_speed=0.6, brake=True), t_mono_ns=0)
                log.write(STATE_MCAP_TOPIC, VehicleState(speed=0.55), t_mono_ns=2_000_000)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=5_000_000)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                w.writerow(MANIFEST_COLUMNS)
                extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=100_000_000)

            (sample,) = load_manifest(out_dir)
            self.assertAlmostEqual(sample.target_steer, -0.1)
            self.assertAlmostEqual(sample.target_speed, 0.6)
            self.assertAlmostEqual(sample.speed_actual, 0.55)
            self.assertTrue(sample.brake)

    def test_missing_vehicle_state_leaves_actual_speed_unknown(self):
        import math

        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_speed=0.6),
                          t_mono_ns=0)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=0)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                w.writerow(MANIFEST_COLUMNS)
                extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=100_000_000)
            (sample,) = load_manifest(out_dir)
            self.assertTrue(math.isnan(sample.speed_actual))

    def test_label_shift_pairs_the_frame_with_a_later_command(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=0.0),
                          t_mono_ns=0)
                log.write(CMD_MCAP_TOPIC, DriveCmd(mode=1, arm=True, target_steer=0.3),
                          t_mono_ns=150_000_000)
                log.write_viz_image(_tiny_jpeg(), "front", t_mono_ns=0)

            for shift_ms, want in ((0, 0.0), (150, 0.3)):
                out_dir = Path(d) / f"frames{shift_ms}"
                out_dir.mkdir()
                with open(out_dir / "manifest.csv", "a", newline="") as mf:
                    w = csv.writer(mf)
                    w.writerow(MANIFEST_COLUMNS)
                    extract_one(mcap_path, out_dir, {"front"}, w, max_gap_ns=50_000_000,
                                label_shift_ns=shift_ms * 1_000_000)
                (sample,) = load_manifest(out_dir)
                self.assertAlmostEqual(sample.target_steer, want)


class TestRejectReason(unittest.TestCase):
    def test_reasons(self):
        self.assertIsNone(reject_reason({"arm": True, "mode": 1}))
        self.assertEqual(reject_reason(None), "近い指令が無い")
        self.assertEqual(reject_reason({"arm": False, "mode": 1}), "DISARM中")
        self.assertEqual(reject_reason({"arm": True, "mode": 2}), "自動運転中")

    def test_auto_driving_can_be_opted_in_as_the_demonstration(self):
        """slam2d_route などの自動運転を手本にする（カメラで真似させる）使い方。"""
        auto = {"arm": True, "mode": 2, "source": "auto:slam2d_route"}
        self.assertEqual(reject_reason(auto), "自動運転中")
        self.assertIsNone(reject_reason(auto, include_auto=True))
        self.assertIsNone(reject_reason({"arm": True, "mode": 1}, include_auto=True))

    def test_its_own_driving_is_never_a_demonstration(self):
        """このモデル自身の走行を手本にすると、自分の誤差を覚え直すだけになる。"""
        own = {"arm": True, "mode": 2, "source": "auto:cam_e2e"}
        self.assertEqual(reject_reason(own, include_auto=True), "カメラE2E自身の走行")
        self.assertEqual(reject_reason(own), "カメラE2E自身の走行")

    def test_opting_in_does_not_relax_the_other_filters(self):
        self.assertEqual(reject_reason({"arm": False, "mode": 2}, include_auto=True), "DISARM中")
        self.assertEqual(reject_reason({"arm": True, "mode": 2, "torque_mode": True},
                                       include_auto=True), "トルクモード")
        self.assertEqual(reject_reason({"arm": True, "mode": 1, "torque_mode": True}),
                         "トルクモード")


class TestManifestHeader(unittest.TestCase):
    def test_old_six_column_manifest_is_refused(self):
        """列の違う manifest に追記すると、以降の行が全部ずれて読まれる。"""
        with tempfile.TemporaryDirectory() as d:
            old = Path(d) / "old.csv"
            old.write_text("file,source_mcap,cam,t_capture_ns,target_steer,target_speed\n")
            new = Path(d) / "new.csv"
            new.write_text(",".join(MANIFEST_COLUMNS) + "\n")
            self.assertFalse(check_manifest_header(old))
            self.assertTrue(check_manifest_header(new))


if __name__ == "__main__":
    unittest.main()
