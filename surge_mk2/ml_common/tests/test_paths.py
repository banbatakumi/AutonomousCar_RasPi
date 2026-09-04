import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_common.paths import make_rel  # noqa: E402


class TestMakeRel(unittest.TestCase):
    def test_shortens_paths_under_repo_root(self):
        repo_root = Path("/repo")
        rel = make_rel(repo_root)
        self.assertEqual(rel(repo_root / "ml_lidar" / "runs" / "v1"), "ml_lidar/runs/v1")

    def test_falls_back_to_absolute_outside_repo_root(self):
        rel = make_rel(Path("/repo"))
        p = Path("/some/other/place")
        self.assertEqual(rel(p), str(p))


if __name__ == "__main__":
    unittest.main()
