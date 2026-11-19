"""`raspi/io/gpio.Buzzer` — ブザーが鳴らせなくても io_node を落とさない（2026-10 の実機の `tone is out of the device's range`）。"""

from __future__ import annotations

import io
import unittest
from contextlib import redirect_stderr


from raspi.io.gpio import MELODY_BOOT, MELODY_GUI_CONNECT, Buzzer


class _Dev:
    """`GpiozeroTone` の代わり。`bad` の音名は gpiozero と同じ ValueError を出す。"""

    def __init__(self, bad=()):
        self.bad = set(bad)
        self.played: list[str] = []
        self.stops = 0

    def play(self, tone):
        if tone in self.bad:
            raise ValueError("tone is out of the device's range")
        self.played.append(tone)

    def stop(self):
        self.stops += 1


class TestBuzzer(unittest.TestCase):
    def test_beep_failure_does_not_raise_into_the_main_loop(self):
        """単発ビープ（メインループから同期で呼ばれる）が失敗しても例外を出さず、ログは一度だけ。"""
        dev = _Dev(bad={Buzzer.BEEP_TONE})
        b = Buzzer(dev)
        b.SETTLE_S = 0.0
        err = io.StringIO()
        with redirect_stderr(err):
            b.write(True)
            b.write(False)
            b.write(True)
        self.assertEqual(err.getvalue().count("ブザーを鳴らせない"), 1)

    def test_melody_skips_a_bad_note_and_stops(self):
        dev = _Dev(bad={"C6"})
        b = Buzzer(dev)
        b.SETTLE_S = 0.0
        with redirect_stderr(io.StringIO()):
            b._run(MELODY_GUI_CONNECT, b._gen)
        self.assertEqual(dev.played, ["A5"])
        self.assertGreaterEqual(dev.stops, 1)           # 鳴らしっぱなしにしない

    def test_melodies_are_within_the_tonal_buzzer_range(self):
        """`GpiozeroTone`（`TonalBuzzer(octaves=2)` = A2〜A6、110〜1760Hz）で鳴らせる音だけを使う。"""
        names = "C C# D D# E F F# G G# A A# B".split()

        def hz(n: str) -> float:
            midi = 12 * (int(n[-1]) + 1) + names.index(n[:-1])
            return 440.0 * 2 ** ((midi - 69) / 12)

        for name, _, _ in (*MELODY_BOOT, *MELODY_GUI_CONNECT, (Buzzer.BEEP_TONE, 0, 0)):
            with self.subTest(name=name):
                self.assertTrue(110.0 <= hz(name) <= 1760.0, name)


if __name__ == "__main__":
    unittest.main()
