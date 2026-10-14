"""Pi 用の小さな道具（`raspi/tools/`）の、実機なしで確かめられる所。"""

from __future__ import annotations

import time

from raspi.tools import arm_test


class _Link:
    def poll(self):
        return []

    def send(self, msg):
        pass


class _Hb:
    def kick(self):
        pass


class _Log:
    def write_rx(self, *a):
        pass


def test_arm_test_stops_on_ctrl_c_inside_a_stage():
    """Ctrl-C の旗が立ったら、段階の残り時間を待たずに降りる（arm=1 を送り続けない）。"""
    stop = {"v": True}
    rig = arm_test.Rig(_Link(), _Log(), _Hb(), stop)
    t0 = time.monotonic()
    try:
        rig.pump(3.0, arm=True, mode=0, steer=0, label="段階2")
    except arm_test.Aborted as e:
        assert "Ctrl-C" in str(e)
    else:
        raise AssertionError("Aborted にならなかった")
    assert time.monotonic() - t0 < 0.5
