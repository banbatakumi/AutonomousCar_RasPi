import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.notes import read_note, write_note  # noqa: E402


class TestNote(unittest.TestCase):
    def test_missing_note_returns_empty(self):
        self.assertEqual(read_note(Path("/nonexistent")), "")

    def test_round_trips(self):
        with tempfile.TemporaryDirectory() as d:
            dir_ = Path(d) / "v1"
            write_note(dir_, "テストの備考")
            self.assertEqual(read_note(dir_), "テストの備考")

    def test_write_overwrites_previous_content(self):
        with tempfile.TemporaryDirectory() as d:
            dir_ = Path(d)
            write_note(dir_, "1回目")
            write_note(dir_, "2回目に書き直した")
            self.assertEqual(read_note(dir_), "2回目に書き直した")

    def test_write_creates_missing_parent_dirs(self):
        with tempfile.TemporaryDirectory() as d:
            dir_ = Path(d) / "nested" / "v1"
            write_note(dir_, "x")
            self.assertTrue(dir_.is_dir())


if __name__ == "__main__":
    unittest.main()
