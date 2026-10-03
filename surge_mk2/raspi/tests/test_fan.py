"""`SysfsFan` — auto を保つ定期呼び出しが governor を毎秒蹴らないこと（2026-10-04）。

以前は1Hzの `set_auto()` が `cur_state` を毎秒書き換え、低温時にファンが毎秒
回っては止まっていた。sysfs を一時ファイルで置き換えて振る舞いを固定する。
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from raspi.io.fan import SysfsFan


def _fan(root: Path, enable: str, cur_state: str) -> SysfsFan:
    for name, val in (("pwm1", "0"), ("pwm1_enable", enable), ("cur_state", cur_state)):
        (root / name).write_text(val)
    fan = object.__new__(SysfsFan)
    fan.available = True
    fan._pwm1 = root / "pwm1"
    fan._pwm1_enable = root / "pwm1_enable"
    fan._fan1_input = None
    fan._cur_state = root / "cur_state"
    return fan


class EnsureAutoTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_already_auto_does_not_kick(self) -> None:
        fan = _fan(self.root, "2", "0")
        for _ in range(5):
            self.assertTrue(fan.ensure_auto())
        self.assertEqual((self.root / "cur_state").read_text(), "0")

    def test_fallen_out_of_auto_is_restored_with_kick(self) -> None:
        fan = _fan(self.root, "1", "0")
        self.assertTrue(fan.ensure_auto())
        self.assertEqual((self.root / "pwm1_enable").read_text(), "2")
        self.assertEqual((self.root / "cur_state").read_text(), "1")   # 再評価のきっかけ

    def test_explicit_set_auto_still_kicks(self) -> None:
        fan = _fan(self.root, "1", "2")
        self.assertTrue(fan.set_auto())
        self.assertEqual((self.root / "cur_state").read_text(), "1")


if __name__ == "__main__":
    unittest.main()
