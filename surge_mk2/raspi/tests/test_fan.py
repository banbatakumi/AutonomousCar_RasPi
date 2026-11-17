"""`SysfsFan` — auto を保つ定期呼び出しが governor を毎秒蹴らないこと。

以前は1Hzの `set_auto()` が `cur_state` を毎秒書き換え、低温時にファンが毎秒
回っては止まっていた。sysfs を一時ファイルで置き換えて振る舞いを固定する。
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from raspi.io.fan import SysfsFan


def _fan(root: Path, enable: str, cur_state: str, temp_mc: int | None = None,
         pwm1: str = "0") -> SysfsFan:
    for name, val in (("pwm1", pwm1), ("pwm1_enable", enable), ("cur_state", cur_state)):
        (root / name).write_text(val)
    fan = object.__new__(SysfsFan)
    fan.available = True
    fan._pwm1 = root / "pwm1"
    fan._pwm1_enable = root / "pwm1_enable"
    fan._fan1_input = None
    fan._cur_state = root / "cur_state"
    fan._zone_temp = None
    fan._trip_mc = None
    fan._release_mc = None
    if temp_mc is not None:
        (root / "temp").write_text(str(temp_mc))
        fan._zone_temp, fan._trip_mc, fan._release_mc = root / "temp", 50000, 45000
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


class LowTempKickTest(unittest.TestCase):
    """最低の trip より冷えているときは 1段上げない（解除ラインより下で回っていれば 0 を書く）。

    thermal zone はポーリングしないので、低温で 0→1 にすると誰も戻さず回り続けた（実機 43°C）。"""

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def test_cold_and_off_stays_off(self) -> None:
        fan = _fan(self.root, "1", "0", temp_mc=43000)
        self.assertTrue(fan.set_auto())
        self.assertEqual((self.root / "cur_state").read_text(), "0")

    def test_cold_and_spinning_is_stopped(self) -> None:
        fan = _fan(self.root, "1", "2", temp_mc=43000)
        self.assertTrue(fan.set_auto())
        self.assertEqual((self.root / "cur_state").read_text(), "0")

    def test_hysteresis_band_and_off_stays_off(self) -> None:
        # 45〜50°C で 0 は正しい。上げると 45°C まで冷えるまで回り続ける
        fan = _fan(self.root, "1", "0", temp_mc=48500)
        self.assertTrue(fan.set_auto())
        self.assertEqual((self.root / "cur_state").read_text(), "0")

    def test_off_state_with_leftover_manual_pwm_is_reapplied(self) -> None:
        # 手動で回していた pwm が状態 0 のまま残っている → 1→0 で出し直す（最後に 0）
        fan = _fan(self.root, "1", "0", temp_mc=43000, pwm1="200")
        writes: list[str] = []
        orig = Path.write_text

        def spy(path, data, *a, **k):
            if path.name == "cur_state":
                writes.append(data)
            return orig(path, data, *a, **k)

        with mock.patch.object(Path, "write_text", spy):
            self.assertTrue(fan.set_auto())
        self.assertEqual(writes, ["1", "0"])

    def test_hysteresis_band_and_spinning_keeps_nudge(self) -> None:
        fan = _fan(self.root, "1", "2", temp_mc=47000)
        self.assertTrue(fan.set_auto())
        self.assertEqual((self.root / "cur_state").read_text(), "1")

    def test_above_trip_keeps_nudge(self) -> None:
        fan = _fan(self.root, "1", "0", temp_mc=52000)
        self.assertTrue(fan.set_auto())
        self.assertEqual((self.root / "cur_state").read_text(), "1")

    def test_probe_zone_finds_lowest_release(self) -> None:
        cdev = self.root / "cooling_device0"
        other = self.root / "cooling_device1"
        zone = self.root / "thermal_zone0"
        for d in (cdev, other, zone):
            d.mkdir()
        (zone / "temp").write_text("43000")
        # trip0 は critical（110°C・ヒステリシス0）、trip1 が最低の active（50°C・5°C）
        trips = [(110000, 0), (50000, 5000), (60000, 5000)]
        for i, (t, h) in enumerate(trips):
            (zone / f"trip_point_{i}_temp").write_text(str(t))
            (zone / f"trip_point_{i}_hyst").write_text(str(h))
            (zone / f"cdev{i}").symlink_to(cdev)
            (zone / f"cdev{i}_trip_point").write_text(str(i))
        (zone / "cdev3").symlink_to(other)             # 別の cooling device の trip は見ない
        (zone / "cdev3_trip_point").write_text("9")
        fan = _fan(self.root, "2", "0")
        with mock.patch("raspi.io.fan.THERMAL_ZONE_GLOB", str(self.root / "thermal_zone*")):
            fan._probe_zone(cdev)
        self.assertEqual(fan._zone_temp, zone / "temp")
        self.assertEqual(fan._trip_mc, 50000)
        self.assertEqual(fan._release_mc, 45000)


if __name__ == "__main__":
    unittest.main()
