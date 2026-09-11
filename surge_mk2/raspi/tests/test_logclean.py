"""`raspi/rec/logclean.py`（ディスク空き容量監視・世代管理の自動削除）の単体テスト。

`shutil.disk_usage` を差し替えて容量を偽装する（実ディスクの空きに依存しない）。

    python3 -m unittest discover -s raspi/tests -t .
"""

from __future__ import annotations

import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.rec import logclean  # noqa: E402


def _usage(total_gb: float, free_gb: float):
    """`shutil.disk_usage` の戻り値と同じ形（total, used, free）を偽装する。"""
    total = int(total_gb * 1e9)
    free = int(free_gb * 1e9)
    return mock.Mock(total=total, used=total - free, free=free)


class TempDirCase(unittest.TestCase):
    def setUp(self):
        self._td = tempfile.TemporaryDirectory()
        self.tmp = Path(self._td.name)

    def tearDown(self):
        self._td.cleanup()

    def _touch(self, name: str, age_s: float = 0.0, size: int = 1000) -> Path:
        """`age_s` 秒前の mtime を持つファイルを作る（大きいほど古い＝先に消える）。"""
        p = self.tmp / name
        p.write_bytes(b"x" * size)
        t = time.time() - age_s
        os.utime(p, (t, t))
        return p


class TestDiskFreePct(TempDirCase):
    def test_matches_shutil_disk_usage(self):
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        return_value=_usage(100, 40)):
            self.assertAlmostEqual(logclean.disk_free_pct(self.tmp), 40.0)


class TestCheckDiskNoAction(TempDirCase):
    def test_plenty_of_space_does_nothing(self):
        self._touch("a.sfl")
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        return_value=_usage(100, 50)):
            status = logclean.check_disk(self.tmp)
        self.assertFalse(status.warned)
        self.assertEqual(status.deleted, [])
        self.assertAlmostEqual(status.free_pct, 50.0)
        self.assertTrue((self.tmp / "a.sfl").exists())

    def test_disk_usage_failure_reports_error_and_free_pct_100(self):
        """空き容量すら取れない環境（パス不在等）では、削除しない側に倒す。"""
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        side_effect=OSError("no such path")):
            status = logclean.check_disk(self.tmp / "does_not_exist")
        self.assertIsNotNone(status.error)
        self.assertEqual(status.free_pct, 100.0)
        self.assertEqual(status.deleted, [])


class TestCheckDiskWarn(TempDirCase):
    def test_below_warn_but_above_critical_only_warns(self):
        self._touch("a.sfl")
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        return_value=_usage(100, 10)):   # 10% free
            status = logclean.check_disk(self.tmp, warn_free_pct=15.0,
                                         critical_free_pct=5.0)
        self.assertTrue(status.warned)
        self.assertEqual(status.deleted, [])
        self.assertTrue((self.tmp / "a.sfl").exists())


class TestCheckDiskGenerationalDelete(TempDirCase):
    def test_deletes_oldest_first_until_back_above_critical(self):
        """世代管理: mtime の古い順に消し、critical を上回った時点で止まること。"""
        old = self._touch("old.sfl", age_s=300)
        mid = self._touch("mid.sfl", age_s=200)
        new = self._touch("new.sfl", age_s=100)

        # 1回目の disk_usage 呼び出しは 3% free（critical=5% を下回る）。
        # 削除のたびに再計算し、2回消した時点で 6% に回復したことにする
        usages = iter([
            _usage(100, 3),     # 開始判定
            _usage(100, 3),     # old.sfl 削除後もまだ足りない
            _usage(100, 6),     # mid.sfl 削除後に critical を上回る
        ])
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        side_effect=lambda _p: next(usages)):
            status = logclean.check_disk(self.tmp, warn_free_pct=15.0,
                                         critical_free_pct=5.0)

        self.assertTrue(status.warned)
        self.assertEqual(status.deleted, ["old.sfl", "mid.sfl"])
        self.assertFalse(old.exists())
        self.assertFalse(mid.exists())
        self.assertTrue(new.exists())
        self.assertAlmostEqual(status.free_pct, 6.0)
        self.assertGreater(status.freed_bytes, 0)

    def test_protected_path_is_never_deleted(self):
        """記録中の `.sfl` は世代管理の対象から外れること。"""
        active = self._touch("active.sfl", age_s=500)   # 一番古いが記録中
        self._touch("mid.sfl", age_s=200)

        usages = iter([_usage(100, 2), _usage(100, 6)])
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        side_effect=lambda _p: next(usages)):
            status = logclean.check_disk(self.tmp, critical_free_pct=5.0,
                                         protect={active})

        self.assertEqual(status.deleted, ["mid.sfl"])
        self.assertTrue(active.exists())

    def test_stops_at_max_delete_even_if_still_critical(self):
        """暴走防止: 1回の呼び出しでの削除数に上限があること。"""
        for i in range(5):
            self._touch(f"f{i}.sfl", age_s=100 - i)

        # 空き容量は最後まで回復しない設定（常に critical のまま）
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        return_value=_usage(100, 1)):
            status = logclean.check_disk(self.tmp, critical_free_pct=5.0,
                                         max_delete=2)

        self.assertEqual(len(status.deleted), 2)

    def test_only_sfl_and_mcap_are_candidates(self):
        """記録以外のファイル（設定等）には触らないこと。"""
        self._touch("config.json", age_s=1000)
        sfl = self._touch("a.sfl", age_s=500)

        usages = iter([_usage(100, 2), _usage(100, 10)])
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        side_effect=lambda _p: next(usages)):
            status = logclean.check_disk(self.tmp, critical_free_pct=5.0)

        self.assertEqual(status.deleted, ["a.sfl"])
        self.assertFalse(sfl.exists())
        self.assertTrue((self.tmp / "config.json").exists())

    def test_mcap_files_are_also_candidates(self):
        self._touch("old.mcap", age_s=500)
        usages = iter([_usage(100, 2), _usage(100, 10)])
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        side_effect=lambda _p: next(usages)):
            status = logclean.check_disk(self.tmp, critical_free_pct=5.0)
        self.assertEqual(status.deleted, ["old.mcap"])

    def test_no_candidates_left_still_reports_warned(self):
        """消せるものが無くても crash せず、warned だけ立てて終わること。"""
        with mock.patch("raspi.rec.logclean.shutil.disk_usage",
                        return_value=_usage(100, 1)):
            status = logclean.check_disk(self.tmp, critical_free_pct=5.0)
        self.assertTrue(status.warned)
        self.assertEqual(status.deleted, [])


if __name__ == "__main__":
    unittest.main()
