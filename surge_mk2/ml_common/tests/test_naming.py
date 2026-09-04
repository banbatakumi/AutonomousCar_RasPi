import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.naming import list_versioned_names, next_versioned_name  # noqa: E402


class TestNextVersionedName(unittest.TestCase):
    def test_no_existing_suggests_v1(self):
        self.assertEqual(next_versioned_name([]), "v1")

    def test_ignores_non_v_named(self):
        self.assertEqual(next_versioned_name(["ppo_e2e", "first_model"]), "v1")

    def test_suggests_next_number_after_highest(self):
        self.assertEqual(next_versioned_name(["v1", "v2"]), "v3")

    def test_ignores_gaps_and_uses_max(self):
        self.assertEqual(next_versioned_name(["v1", "v5", "v3"]), "v6")


class TestListVersionedNames(unittest.TestCase):
    def test_missing_dir_returns_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(list_versioned_names(Path(d) / "nope"), [])

    def test_lists_only_directories_sorted(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "v2").mkdir()
            (root / "v1").mkdir()
            (root / "not_a_dir.txt").write_text("x")
            self.assertEqual(list_versioned_names(root), ["v1", "v2"])


if __name__ == "__main__":
    unittest.main()
