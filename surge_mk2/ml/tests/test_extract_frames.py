"""`ml/extract_frames.py` のテスト。

**実データも実走行も要らない。** `raspi/rec/mcap_log.py`（Pi側の書き手、
GUIの録画がこれを裏で呼ぶ）で実際に `.mcap` を1つ作り、それを
`extract_frames.py` で読み戻して JPEG が正しく復元されることを確認する。
書き手と読み手を両方このリポジトリの本物のコードで通す往復テスト。
"""

import csv
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))          # repo root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))          # ml/

from extract_frames import VIZ_IMAGE_PREFIX, extract_one  # noqa: E402
from raspi.rec.mcap_log import McapLog  # noqa: E402


def _tiny_jpeg() -> bytes:
    """本物の JPEG である必要は無い（`extract_frames.py` はバイト列を右から左に
    書き出すだけで、デコードはしない）。"""
    return b"\xff\xd8\xff\xe0mock-jpeg-bytes\xff\xd9"


class TestExtractFrames(unittest.TestCase):
    def test_round_trips_through_the_real_writer(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            jpg = _tiny_jpeg()
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=1_700_000_000_000_000_000) as log:
                log.write_viz_image(jpg, "front", t_mono_ns=1_000_000)
                log.write_viz_image(jpg, "front", t_mono_ns=2_000_000)
                log.write_viz_image(jpg, "rear", t_mono_ns=1_500_000)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            manifest_path = out_dir / "manifest.csv"
            with open(manifest_path, "a", newline="") as mf:
                w = csv.writer(mf)
                w.writerow(["file", "source_mcap", "cam", "t_capture_ns"])
                n = extract_one(mcap_path, out_dir, {"front"}, w)

            self.assertEqual(n, 2, "front カメラの2枚だけを取り出すはず")
            jpg_files = sorted(out_dir.glob("*.jpg"))
            self.assertEqual(len(jpg_files), 2)
            for f in jpg_files:
                self.assertEqual(f.read_bytes(), jpg)

            with open(manifest_path) as mf:
                rows = list(csv.reader(mf))
            self.assertEqual(rows[0], ["file", "source_mcap", "cam", "t_capture_ns"])
            self.assertEqual(len(rows) - 1, 2)
            for row in rows[1:]:
                self.assertEqual(row[1], "run.mcap")
                self.assertEqual(row[2], "front")

    def test_rear_camera_is_skipped_when_not_requested(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write_viz_image(_tiny_jpeg(), "rear", t_mono_ns=0)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                n = extract_one(mcap_path, out_dir, {"front"}, w)
            self.assertEqual(n, 0)

    def test_min_interval_ns_thins_out_close_frames(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            jpg = _tiny_jpeg()
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write_viz_image(jpg, "front", t_mono_ns=0)
                log.write_viz_image(jpg, "front", t_mono_ns=100_000_000)   # +100ms → 間引かれる
                log.write_viz_image(jpg, "front", t_mono_ns=250_000_000)   # +250ms → 採用される

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                n = extract_one(mcap_path, out_dir, {"front"}, w, min_interval_ns=200_000_000)

            self.assertEqual(n, 2, "0msと250msの2枚だけ採用され、100msは間引かれるはず")

    def test_min_interval_ns_is_per_camera(self):
        with tempfile.TemporaryDirectory() as d:
            mcap_path = Path(d) / "run.mcap"
            jpg = _tiny_jpeg()
            with McapLog(mcap_path, t0_mono_ns=0, t0_unix_ns=0) as log:
                log.write_viz_image(jpg, "front", t_mono_ns=0)
                log.write_viz_image(jpg, "rear", t_mono_ns=10_000_000)

            out_dir = Path(d) / "frames"
            out_dir.mkdir()
            with open(out_dir / "manifest.csv", "a", newline="") as mf:
                w = csv.writer(mf)
                n = extract_one(mcap_path, out_dir, {"front", "rear"}, w,
                                min_interval_ns=200_000_000)

            self.assertEqual(n, 2, "別カメラなので間引きの基準時刻を共有しないはず")

    def test_topic_prefix_matches_mcap_log(self):
        """`VIZ_IMAGE_PREFIX` が `raspi/rec/mcap_log.py` の値とズレていないこと。"""
        from raspi.rec.mcap_log import VIZ_IMAGE_PREFIX as PI_SIDE_PREFIX

        self.assertEqual(VIZ_IMAGE_PREFIX, PI_SIDE_PREFIX)


if __name__ == "__main__":
    unittest.main()
