"""`BusBridge` が `VehicleState.t_capture` に入れる時刻。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.bus_bridge import BusBridge  # noqa: E402
from raspi.proto import packets  # noqa: E402


class TestStateCaptureTime(unittest.TestCase):
    """換算した STM32 時刻を使うが、時刻同期が外れているときは受信時刻で代用する
    （LiDAR セクタの時刻と同じ判定。片方だけ外れた時刻を使うと状態と点群の時間軸が食い違う）。"""

    NOW = 9_000_000_000

    def _capture(self, t_pi_ns):
        b = BusBridge(None, clock=lambda: self.NOW)
        seen = []
        b.state_builder.build = lambda t, capture: seen.append(capture)
        b.on_telemetry(packets.Telemetry(), t_pi_ns)
        return seen[0]

    def test_the_converted_time_is_used_when_plausible(self):
        self.assertEqual(self._capture(self.NOW - 3_000_000), self.NOW - 3_000_000)

    def test_falls_back_to_now_without_a_time_sync(self):
        self.assertEqual(self._capture(None), self.NOW)

    def test_falls_back_to_now_when_the_converted_time_is_in_the_future(self):
        self.assertEqual(self._capture(self.NOW + 150_000_000), self.NOW)

    def test_falls_back_to_now_when_the_converted_time_is_too_old(self):
        self.assertEqual(self._capture(self.NOW - 1_000_000_000), self.NOW)


if __name__ == "__main__":
    unittest.main()
