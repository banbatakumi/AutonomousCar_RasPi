"""`raspi/auto/mapstore.py`（地図データの保存・一覧・削除・アップロード検証）の単体テスト。

`slam2d`にもGUIにも触れず、ファイルI/Oだけで完結する。
"""

import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto import mapstore  # noqa: E402


def make_trinary(h=8, w=8) -> np.ndarray:
    t = np.zeros((h, w), dtype=np.uint8)
    t[:2, :] = 2       # 占有
    t[2:5, :] = 1       # 空き
    return t            # 残りは未知(0)


class TestMapstore(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self._orig_dir = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(self._td.name) / "saved_maps"

    def tearDown(self):
        mapstore.MAPS_DIR = self._orig_dir
        self._td.cleanup()

    def test_save_then_load_roundtrip(self):
        t = make_trinary()
        mapstore.save_map(
            "course_a", resolution=0.025, origin_x=-1.0, origin_y=-1.0,
            trinary=t, centerline_xy=np.zeros((4, 2)),
            raceline_xy=np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]),
            raceline_v=np.array([1.0, 1.0, 1.0, 1.0]))

        loaded = mapstore.load_map("course_a")
        self.assertIsNotNone(loaded)
        self.assertTrue(np.array_equal(loaded.trinary, t))
        self.assertAlmostEqual(loaded.resolution, 0.025)
        self.assertAlmostEqual(loaded.origin_x, -1.0)
        self.assertEqual(loaded.raceline_xy.shape, (4, 2))

    def test_save_writes_npz_and_json_sidecar(self):
        mapstore.save_map(
            "course_b", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=make_trinary(), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        self.assertTrue((mapstore.MAPS_DIR / "course_b.npz").is_file())
        self.assertTrue((mapstore.MAPS_DIR / "course_b.json").is_file())

    def test_list_maps_does_not_require_npz_body(self):
        """一覧はnpzを展開せず`.json`だけ読む設計——壊れたnpzでも一覧には出る。"""
        mapstore.save_map(
            "course_c", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=make_trinary(4, 4), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.array([[0.0, 0.0], [3.0, 4.0]]), raceline_v=np.array([1.0, 1.0]))
        (mapstore.MAPS_DIR / "course_c.npz").write_bytes(b"corrupted")

        files = mapstore.list_maps()
        self.assertEqual(len(files), 1)
        self.assertEqual(files[0]["name"], "course_c")
        self.assertEqual(files[0]["raceline_points"], 2)
        self.assertAlmostEqual(files[0]["length_m"], 10.0)  # (0,0)→(3,4)→(0,0) の往復

    def test_load_missing_returns_none(self):
        self.assertIsNone(mapstore.load_map("no_such_map"))

    def test_load_corrupted_npz_returns_none(self):
        mapstore.MAPS_DIR.mkdir(parents=True, exist_ok=True)
        (mapstore.MAPS_DIR / "broken.npz").write_bytes(b"not a real npz file")
        self.assertIsNone(mapstore.load_map("broken"))

    def test_delete_removes_both_files(self):
        mapstore.save_map(
            "course_d", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=make_trinary(), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        mapstore.delete_map("course_d")
        self.assertFalse((mapstore.MAPS_DIR / "course_d.npz").exists())
        self.assertFalse((mapstore.MAPS_DIR / "course_d.json").exists())
        self.assertEqual(mapstore.list_maps(), [])

    def test_delete_missing_does_not_raise(self):
        mapstore.delete_map("does_not_exist")   # 例外を投げないことだけ確認

    def test_resolve_map_path_rejects_traversal(self):
        self.assertIsNone(mapstore.resolve_map_path(""))
        self.assertIsNone(mapstore.resolve_map_path("../etc/passwd"))
        self.assertIsNone(mapstore.resolve_map_path("a/b"))
        self.assertIsNone(mapstore.resolve_map_path("a\\b"))
        self.assertIsNone(mapstore.resolve_map_path("."))
        self.assertIsNone(mapstore.resolve_map_path(".."))
        self.assertIsNotNone(mapstore.resolve_map_path("valid_name"))

    def test_validate_upload_accepts_well_formed_bytes(self):
        mapstore.save_map(
            "course_e", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=make_trinary(), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        data = (mapstore.MAPS_DIR / "course_e.npz").read_bytes()
        loaded = mapstore.validate_upload(data)
        self.assertIsNotNone(loaded)
        self.assertEqual(loaded.trinary.shape, (8, 8))

    def test_validate_upload_rejects_garbage(self):
        self.assertIsNone(mapstore.validate_upload(b"definitely not an npz file"))

    def test_validate_upload_rejects_missing_keys(self):
        """必須キーが欠けた(有効な)npzも拒否する——アップロード経路の境界検証。"""
        import io
        buf = io.BytesIO()
        np.savez_compressed(buf, trinary=make_trinary())  # resolution等が無い
        self.assertIsNone(mapstore.validate_upload(buf.getvalue()))

    def test_save_upload_writes_files_on_valid_data(self):
        mapstore.save_map(
            "course_f", resolution=0.05, origin_x=0.0, origin_y=0.0,
            trinary=make_trinary(), centerline_xy=np.zeros((0, 2)),
            raceline_xy=np.zeros((0, 2)), raceline_v=np.zeros(0))
        data = (mapstore.MAPS_DIR / "course_f.npz").read_bytes()
        mapstore.delete_map("course_f")

        result = mapstore.save_upload("course_f_uploaded", data)
        self.assertIsNotNone(result)
        self.assertTrue((mapstore.MAPS_DIR / "course_f_uploaded.npz").is_file())
        self.assertTrue((mapstore.MAPS_DIR / "course_f_uploaded.json").is_file())

    def test_save_upload_rejects_garbage_without_writing(self):
        result = mapstore.save_upload("bad", b"garbage")
        self.assertIsNone(result)
        self.assertFalse((mapstore.MAPS_DIR / "bad.npz").exists())

    def test_list_maps_empty_when_dir_missing(self):
        self.assertEqual(mapstore.list_maps(), [])


if __name__ == "__main__":
    unittest.main()
