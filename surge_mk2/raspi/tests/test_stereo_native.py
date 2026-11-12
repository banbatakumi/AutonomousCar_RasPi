"""C 版の後処理（`raspi/core/stereo_native.c`）が numpy 版と同じ結果を返すか。

合成の視差（床・壁・箱・ばらばらの誤対応・縮めた照合）を両方に通して突き合わせる。
コンパイラが無い環境では飛ばす（そのとき `stereo_node` は numpy 版で動く）。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from raspi.core.camera_model import FisheyeCalib
from raspi.core.stereo_match import (ObstacleParams, fov_degrees, obstacles_from_disparity,
                                     obstacles_from_disparity_numpy)
from raspi.core.stereo_model import StereoCalib, StereoRectifier
from raspi.core.stereo_native import load

pytestmark = pytest.mark.skipif(load() is None, reason="C 版をビルドできない（cc が無い）")

CAL = FisheyeCalib(fx=309.5, fy=309.8, cx=305.1, cy=241.5, k=(-0.024, -0.036, 0.041, -0.0196),
                   width=640, height=480)
R_LR = np.eye(3)
EXT = StereoCalib(R=tuple(R_LR.ravel()), T=(-0.0760, 0.0009, -0.0056))
CAM = (0.09, -0.0375, 0.08, 0.02, 0.01)


def _disp(h, w, seed, scale):
    rng = np.random.default_rng(seed)
    d = np.where(rng.uniform(size=(h, w)) < 0.5, rng.uniform(1.0, 96 * scale, (h, w)), -1.0)
    # 縦に広がった「物」をいくつか（同じ視差の縦帯）
    for c in rng.integers(100, w - 20, 6):
        d[h // 4: 3 * h // 4, c:c + 8] = rng.uniform(10, 40) * scale
    return d.astype(np.float32)


@pytest.mark.parametrize("scale,rows,seed", [(1.0, (130, 330), 0), (0.5, (130, 330), 1),
                                              (1.0, (0, 480), 2)])
def test_native_matches_numpy(scale, rows, seed):
    rect = StereoRectifier(CAL, CAL, EXT, 640, 480, math.radians(100), rows=rows)
    fov = fov_degrees(rect, CAM)
    d = _disp(int((rows[1] - rows[0]) * scale), int(640 * scale), seed, scale)
    for q in (ObstacleParams(), ObstacleParams(z_min=0.0, z_max=0.5, r_max=4.0, min_points=2,
                                               near_pct=50.0, min_height_span=0.0)):
        a = obstacles_from_disparity(rect, d, CAM, fov, q, scale=scale, skip_cols=int(96 * scale))
        b = obstacles_from_disparity_numpy(rect, d, CAM, fov, q, scale=scale, skip_cols=int(96 * scale))
        assert a.n_points == b.n_points
        assert a.valid_frac == pytest.approx(b.valid_frac, abs=1e-12)
        assert a.sector_seen == b.sector_seen
        da, db = np.array(a.dist), np.array(b.dist)
        assert (da > 0).sum() > 5
        assert np.allclose(da, db, atol=1e-9), np.nonzero(~np.isclose(da, db))


def test_native_empty():
    rect = StereoRectifier(CAL, CAL, EXT, 640, 480, math.radians(100), rows=(130, 330))
    d = np.full((200, 640), -1.0, np.float32)
    a = obstacles_from_disparity(rect, d, CAM, [0], ObstacleParams(), skip_cols=96)
    assert a.n_points == 0 and not any(a.dist) and a.valid_frac == 0.0
