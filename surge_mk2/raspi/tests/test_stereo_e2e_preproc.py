"""`raspi/core/stereo_e2e_preproc.py` のテスト（numpy だけ）。"""

from __future__ import annotations

import numpy as np
import pytest

from raspi.core.stereo_e2e_preproc import (DEFAULT_DISP_HW, disp_band_input, disp_from_bytes,
                                           disp_input, disp_to_bytes)


def test_band_blocks_mean_and_valid_fraction():
    nd = 16
    d = np.full((24, nd + 40), -1.0, np.float32)
    d[:12, nd:nd + 20] = 8.0                   # 左上のブロックは視差 8（= 0.5·num_disp）
    d[12:, nd + 20:] = 4.0
    d[12, nd + 20] = -1.0                      # 右下のブロックに1画素だけ穴
    out = disp_band_input(d, nd, (2, 2))
    assert out.shape == (2, 2, 2) and out.dtype == np.float32
    np.testing.assert_allclose(out[0], [[0.5, 0.0], [0.0, 0.25]])
    np.testing.assert_allclose(out[1], [[1.0, 0.0], [0.0, 1 - 1 / 240]])


def test_left_columns_without_match_are_dropped():
    nd = 96
    d = np.full((110, 640), 10.0, np.float32)
    d[:, :nd] = 90.0                           # 照合できない列に偽の値があっても使わない
    out = disp_band_input(d, nd)
    assert out.shape == (2, *DEFAULT_DISP_HW)
    np.testing.assert_allclose(out[0], 10.0 / nd, rtol=1e-6)


def test_bytes_roundtrip_and_input_shape():
    rng = np.random.default_rng(0)
    a = rng.uniform(0, 1, (2, 12, 68)).astype(np.float32)
    b = disp_from_bytes(disp_to_bytes(a), 12, 68)
    np.testing.assert_allclose(a, b, atol=1e-3)
    assert disp_input(b).shape == (1, 2, 12, 68)
    with pytest.raises(ValueError):
        disp_from_bytes(disp_to_bytes(a), 12, 67)


def test_too_small_band_is_an_error():
    with pytest.raises(ValueError):
        disp_band_input(np.zeros((4, 100), np.float32), 96)
