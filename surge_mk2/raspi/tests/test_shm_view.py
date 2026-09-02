"""`raspi/tools/shm_view.py` の保存タイミング（C3）。

`write_png()` が `still_valid()`（seqlock検証）より前に実行されていたため、
torn（壊れた）画像が検証前にディスクへ保存されてしまうバグがあった。
`still_valid()` が False を返すフレームでは保存自体が起きないことを確認する。
"""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.bus.shm_ring import FrameRing  # noqa: E402
from raspi.tools import shm_view  # noqa: E402

W, H = 4, 4
FMT = "RGB888"


class _TornRef:
    """`still_valid()` が常に False を返す身代わり参照。"""

    def __init__(self, desc, arr: np.ndarray) -> None:
        self.desc = desc
        self._arr = arr

    def as_array(self) -> np.ndarray:
        return self._arr

    def still_valid(self) -> bool:
        return False


class TestTornFrameIsNotSaved(unittest.TestCase):
    def setUp(self):
        self.name = f"surge_test_shmview_{id(self):x}"
        self.ring = FrameRing.create(self.name, W, H, FMT, n_slots=2)
        self.ring.write(bytes([1]) * (W * H * 3), t_capture_ns=1, frame_id=1)
        self._td = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.ring.unlink()
        self._td.cleanup()

    def test_write_png_is_not_called_when_still_valid_is_false(self):
        real_ref = self.ring.latest()
        # `.copy()` で共有メモリへの参照を切る。**そうしないと `main()` 内の
        # `ring.close()` が「使用中の参照が残っている」で失敗する**（seqlock
        # 契約と同じ要請。この身代わりは検証対象ではないので実配列は要らない）
        torn_ref = _TornRef(real_ref.desc, real_ref.as_array().copy())
        real_ref = None
        save_path = Path(self._td.name) / "out.png"

        argv = ["shm_view", self.name, "--duration", "0.05", "--hz", "30",
               "--save", str(save_path)]
        with mock.patch.object(FrameRing, "latest", return_value=torn_ref), \
             mock.patch.object(FrameRing, "attach", return_value=self.ring), \
             mock.patch("raspi.tools.shm_view.write_png") as mock_write_png, \
             mock.patch("sys.argv", argv):
            shm_view.main()

        mock_write_png.assert_not_called()
        self.assertFalse(save_path.exists())


if __name__ == "__main__":
    unittest.main()
