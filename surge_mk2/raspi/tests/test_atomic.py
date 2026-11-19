"""`raspi/core/atomic.py`: 設定・地図の書き込みが原子的であること（途中で落ちても元のファイルが残る）。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from raspi.core.atomic import atomic_write_bytes, atomic_write_text, atomic_writer


class TestAtomicWrite(unittest.TestCase):
    def setUp(self) -> None:
        self._td = tempfile.TemporaryDirectory()
        self.addCleanup(self._td.cleanup)
        self.dir = Path(self._td.name)

    def test_writes_the_content_and_creates_parents(self):
        path = self.dir / "sub" / "auto.json"
        atomic_write_bytes(path, b'{"mode":"ftg"}')
        self.assertEqual(path.read_bytes(), b'{"mode":"ftg"}')
        atomic_write_text(path, "あ", fsync=True)
        self.assertEqual(path.read_text(encoding="utf-8"), "あ")

    def test_failure_keeps_old_file_and_removes_tmp(self):
        """`os.replace()` に届く前に失敗したら、元のファイルは古いまま・一時ファイルは残らない。"""
        path = self.dir / "auto.json"
        path.write_bytes(b'{"mode":"old"}')
        with mock.patch("os.replace", side_effect=RuntimeError("boom")):
            with self.assertRaises(RuntimeError):
                atomic_write_bytes(path, b'{"mode":"new"}')
        self.assertEqual(path.read_bytes(), b'{"mode":"old"}')
        self.assertEqual([p.name for p in self.dir.iterdir()], ["auto.json"])

    def test_exception_inside_block_keeps_old_file(self):
        path = self.dir / "map.npz"
        path.write_bytes(b"old")
        with self.assertRaises(ValueError):
            with atomic_writer(path) as f:
                f.write(b"half")
                raise ValueError("書きかけで落ちた")
        self.assertEqual(path.read_bytes(), b"old")
        self.assertEqual([p.name for p in self.dir.iterdir()], ["map.npz"])

    def test_no_leftover_tmp_file_on_success(self):
        path = self.dir / "auto.json"
        atomic_write_bytes(path, b"{}")
        self.assertEqual([p.name for p in self.dir.iterdir()], ["auto.json"])


if __name__ == "__main__":
    unittest.main()
