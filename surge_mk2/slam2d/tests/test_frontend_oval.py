"""oval（角の無いループコース）周回での回転ドリフト比較。

Phase 4 完了条件: oval合成コースで固定比率ブレンド（`anisotropic=False`）vs
固有値ベース動的異方性ブレンド（`anisotropic=True`）の回転ドリフト量を
比較・記録する。改善しなければ等方フォールバックで進める、という計画の
判断材料をここで作る。

corridor problem（直線区間で進行方向の位置が点群から決まらない）は
`raspi/nav/slam.py`のdocstringで「等方的に寄せるとovalのような角の無い
ループで位置推定が回転する」と説明されている現象。ovalは直線区間の
どちらの端でも同じ形に見えるため、測距ノイズによるスキャンマッチの
ふらつきが進行方向に無頓着に反映されると、周回するたびに向きがわずかに
ずれ、それが蓄積して大きな回転ドリフトになる。
"""

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from slam2d.core.frontend import Frontend, FrontendConfig  # noqa: E402
from slam2d.core.grid import OccGrid  # noqa: E402
from slam2d.core.motion import ExternalTwistModel  # noqa: E402
from slam2d.core.types import Pose2D, Twist2D, between, wrap_angle  # noqa: E402
from slam2d.tests.helpers import (  # noqa: E402
    make_oval_track, make_raw_scan, oval_centerline_length,
    oval_centerline_pose, oval_centerline_twist,
)

STRAIGHT = 4.0
RADIUS = 2.0
WIDTH = 1.0
SPEED = 0.5
DT = 0.1
NOISE_SIGMA = 0.01     # LD06相当（1cm）
N_LAPS = 3
SEED = 42


def _run_oval_simulation(*, anisotropic: bool) -> list[float]:
    """oval周回をシミュレートし、各周回終了時点でのyaw誤差[rad]のリストを返す。"""
    track = make_oval_track(straight=STRAIGHT, radius=RADIUS, width=WIDTH)
    grid = OccGrid(resolution=0.05, size_m=14.0, origin=(-7.0, -7.0))
    rng = np.random.default_rng(SEED)

    holder = {"twist": Twist2D(SPEED, 0.0, 0.0)}
    motion = ExternalTwistModel(lambda: holder["twist"])
    fe = Frontend(grid, motion,
                 FrontendConfig(anisotropic=anisotropic, kf_dist=0.02,
                               kf_yaw=math.radians(1.0), max_range=8.0))

    lap_length = oval_centerline_length(straight=STRAIGHT, radius=RADIUS)
    start_true = Pose2D(*oval_centerline_pose(0.0, straight=STRAIGHT, radius=RADIUS))
    # 端数切り捨てで最後の周回境界に届かないことがないよう、余裕を持たせる
    n_steps = math.ceil(lap_length * N_LAPS / SPEED / DT) + 5

    yaw_errors: list[float] = []
    next_lap_boundary = lap_length
    s = 0.0
    for _ in range(n_steps):
        s += SPEED * DT
        tx, ty, tyaw = oval_centerline_pose(s, straight=STRAIGHT, radius=RADIUS)
        tv, twr = oval_centerline_twist(s, SPEED, straight=STRAIGHT, radius=RADIUS)
        holder["twist"] = Twist2D(tv, 0.0, twr)
        raw = make_raw_scan(tx, ty, tyaw, segs=track, max_range=8.0,
                           noise_sigma=NOISE_SIGMA, rng=rng)
        fe.update(raw, DT)

        if s >= next_lap_boundary and len(yaw_errors) < N_LAPS:
            true_now = Pose2D(tx, ty, tyaw)
            true_rel = between(start_true, true_now)
            err = between(true_rel, fe.pose)
            yaw_errors.append(abs(wrap_angle(err.yaw)))
            next_lap_boundary += lap_length

    return yaw_errors


class TestOvalRotationDrift(unittest.TestCase):
    def test_anisotropic_vs_isotropic_rotation_drift(self):
        """3周ぶんのyawドリフトを両モードで比較し、結果を記録する。

        改善しなくても即座に失敗にはしない（計画の「改善しなければ等方
        フォールバックで進める」という判断に使う実測データを残すのが目的）。
        代わりに、いずれのモードも実用上破綻しない範囲（見失わない・
        1周あたりの回転ドリフトが数十度のオーダーに収まる）であることは確認する。
        """
        t0 = time.time()
        aniso_errors = _run_oval_simulation(anisotropic=True)
        t1 = time.time()
        iso_errors = _run_oval_simulation(anisotropic=False)
        t2 = time.time()

        aniso_deg = [math.degrees(e) for e in aniso_errors]
        iso_deg = [math.degrees(e) for e in iso_errors]
        print(f"\n[oval drift] anisotropic=True  yaw誤差(周回毎, deg): {aniso_deg} "
              f"({t1 - t0:.1f}s)")
        print(f"[oval drift] anisotropic=False yaw誤差(周回毎, deg): {iso_deg} "
              f"({t2 - t1:.1f}s)")

        self.assertEqual(len(aniso_errors), N_LAPS)
        self.assertEqual(len(iso_errors), N_LAPS)
        # 実用上破綻していないことの確認（見失わず、数十度オーダーに収まる）
        self.assertLess(aniso_deg[-1], 45.0)
        self.assertLess(iso_deg[-1], 45.0)


if __name__ == "__main__":
    unittest.main()
