"""`raspi/core/auto_gate.py` の単体テスト。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.auto_gate import cam_infer_active, vehicle_armed  # noqa: E402
from raspi.msgs import AutoCtrl, VehicleState  # noqa: E402

_MODES = ("ftg_cam", "cam_centerline")


class TestVehicleArmed(unittest.TestCase):
    def test_none_is_armed(self):
        """`VehicleState` 未受信は安全側＝ARM相当として扱う。"""
        self.assertTrue(vehicle_armed(None))

    def test_armed_true(self):
        self.assertTrue(vehicle_armed(VehicleState(armed=True)))

    def test_armed_false(self):
        self.assertFalse(vehicle_armed(VehicleState(armed=False)))


class TestCamInferActive(unittest.TestCase):
    def test_mode_not_selected_is_idle_regardless_of_armed(self):
        self.assertFalse(cam_infer_active(AutoCtrl(mode="line_trace"),
                                          VehicleState(armed=True), _MODES))
        self.assertFalse(cam_infer_active(None, VehicleState(armed=True), _MODES))

    def test_mode_selected_and_armed_is_active(self):
        self.assertTrue(cam_infer_active(AutoCtrl(mode="ftg_cam"),
                                         VehicleState(armed=True), _MODES))

    def test_mode_selected_and_disarmed_is_idle(self):
        self.assertFalse(cam_infer_active(AutoCtrl(mode="ftg_cam"),
                                          VehicleState(armed=False), _MODES))

    def test_mode_selected_and_no_vehicle_state_is_active(self):
        """起動直後（`VehicleState`未受信）はARM相当として推論を始める。"""
        self.assertTrue(cam_infer_active(AutoCtrl(mode="cam_centerline"), None, _MODES))

    def test_single_mode_string_container_does_not_substring_match(self):
        """`modes`にタプルを渡す前提——文字列を直接渡すと部分一致になる罠を確認する。"""
        self.assertTrue(cam_infer_active(AutoCtrl(mode="line_trace"),
                                         VehicleState(armed=True), ("line_trace",)))
        self.assertFalse(cam_infer_active(AutoCtrl(mode="race"),
                                          VehicleState(armed=True), ("line_trace",)))


if __name__ == "__main__":
    unittest.main()
