"""`raspi/core/log.py`: 書式・レベル・重ねて出さないこと。"""

from __future__ import annotations

import io
import logging
import unittest
from unittest import mock

from raspi.core import log as surge_log


class TestSetupLogging(unittest.TestCase):
    def tearDown(self) -> None:
        root = logging.getLogger("surge")
        for h in list(root.handlers):
            if getattr(h, "_surge", False):
                root.removeHandler(h)
        root.propagate = True

    def test_format_level_and_single_handler(self):
        buf = io.StringIO()
        with mock.patch("sys.stderr", buf):
            surge_log.setup_logging("camera", level="INFO")
            surge_log.setup_logging("camera", level="INFO")       # 2回呼んでも1本
            lg = surge_log.get_logger("camera")
            lg.info("起動")
            lg.debug("出ない")
        out = buf.getvalue().splitlines()
        self.assertEqual(len(out), 1)
        self.assertRegex(out[0], r"^\d{4}-\d\d-\d\d \d\d:\d\d:\d\d I camera: 起動$")


if __name__ == "__main__":
    unittest.main()
