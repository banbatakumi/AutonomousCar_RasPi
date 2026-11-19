"""安全側の仮の上限が、Pi（`raspi/msgs/convert.py`）・io_node の起動引数・GUI の生成物で一致すること。"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

from raspi.msgs import convert

ROOT = Path(__file__).resolve().parents[2]


class TestFallbackLimits(unittest.TestCase):
    def test_install_services_max_speed_matches(self):
        text = (ROOT / "raspi" / "setup" / "install_services.sh").read_text(encoding="utf-8")
        m = re.search(r'^ARM="[^"]*--max-speed\s+([0-9.]+)', text, re.M)
        self.assertIsNotNone(m, "install_services.sh に ARM=... --max-speed が無い")
        self.assertAlmostEqual(float(m.group(1)), convert.FALLBACK_MAX_SPEED_M_S)

    def test_generated_ts_carries_the_python_values(self):
        ts = (ROOT / "gui" / "src" / "generated" / "vehicle.ts").read_text(encoding="utf-8")
        for key, value in (("maxBrakeTorqueNm", convert.MAX_BRAKE_TORQUE_NM),
                           ("maxTargetTorqueNm", convert.MAX_TARGET_TORQUE_NM),
                           ("maxSpeedMS", convert.FALLBACK_MAX_SPEED_M_S)):
            m = re.search(rf"^\s*{key}: ([0-9.]+),", ts, re.M)
            self.assertIsNotNone(m, key)
            self.assertAlmostEqual(float(m.group(1)), value, msg=key)


if __name__ == "__main__":
    unittest.main()
