"""`sim/lidar.py` — 走査中に動く LiDAR は点ごとに別の姿勢で撃つ（セクタ内の時刻差）。

実機の点 i は `t_start + dur * i / 29` に測られ、`raspi/nav/deskew.py` はそれを前提に補正する。
シムが 1 セクタを同じ姿勢で撃っていると、回りながら撃った点群の脱スキューが
セクタ内の時刻差（約 8ms）ぶん過補正になる。
"""

import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from raspi.msgs.convert import ScanAssembler  # noqa: E402
from raspi.nav.deskew import deskew  # noqa: E402
from sim.course import Course  # noqa: E402
from sim.lidar import VirtualLidar  # noqa: E402
from sim.params import SimParams  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402

NS = 1_000_000_000
HALF = 5.0                      # 部屋は一辺 10m の正方形


def _room() -> Course:
    res = 0.02
    n = int(2 * (HALF + 0.5) / res)
    grid = np.zeros((n, n), dtype=bool)
    o = -(HALF + 0.5)
    wall = 0
    c = int(round((HALF - o) / res))
    for k in range(-wall, wall + 1):
        grid[c + k, :] = True
        grid[:, c + k] = True
        grid[int(round((-HALF - o) / res)) + k, :] = True
        grid[:, int(round((-HALF - o) / res)) + k] = True
    return Course(name="room", path=Path("room"), resolution=res, origin=(o, o),
                  start=(0.0, 0.0, 0.0), grid=grid)


class _Veh:
    """等速円運動（v, w 一定）。時刻 t [s] の姿勢を返す。"""

    def __init__(self, v, w):
        self.v, self.w = v, w
        self.x = self.y = self.yaw = 0.0

    def pose(self, t):
        yaw = self.w * t
        if abs(self.w) < 1e-9:
            return self.v * t, 0.0, 0.0
        r = self.v / self.w
        return r * math.sin(yaw), r * (1.0 - math.cos(yaw)), yaw

    def set(self, t):
        self.x, self.y, self.yaw = self.pose(t)


def _scan(v, w):
    spec = VehicleSpec.load()
    params = replace(SimParams(), lidar_noise_sigma_m=0.0, lidar_noise_rel=0.0,
                     lidar_drop_rate=0.0, lidar_sector_drop_rate=0.0,
                     lidar_delay_ms=0.0, lidar_delay_jitter_ms=0.0)
    lidar = VirtualLidar(_room(), spec, params, seed=1)
    veh = _Veh(v, w)
    asm = ScanAssembler()
    scan = None
    for ms in range(1, 400):
        t_ns = ms * 1_000_000
        veh.set(t_ns / NS)
        for _gen, pkt in lidar.poll(t_ns, veh, lambda t: t // 1000):
            done = asm.feed(pkt, pkt.t_start_us * 1000)
            if done is not None and done.t_capture > 0 and all(done.sector_seen):
                scan = done
    assert scan is not None
    return scan, veh, spec


def _wall_error(scan, veh, spec, *, correct):
    mx = spec.sensor_pose("lidar")[0]
    pts = deskew(scan, veh.v if correct else 0.0, veh.w if correct else 0.0,
                 mount_x=mx)
    hit = pts.hit
    x, y, yaw = veh.pose(pts.t_ref_ns / NS)
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + pts.x * c - pts.y * s
    wy = y + pts.x * s + pts.y * c
    err = np.minimum(np.abs(HALF - np.abs(wx)), np.abs(HALF - np.abs(wy)))
    return err[hit]


class TestIntraSectorMotion(unittest.TestCase):
    def test_deskew_puts_points_on_walls(self):
        scan, veh, spec = _scan(1.0, 2.0)
        fixed = _wall_error(scan, veh, spec, correct=True)
        raw = _wall_error(scan, veh, spec, correct=False)
        self.assertGreater(fixed.size, 300)
        self.assertLess(np.percentile(fixed, 99), 0.04)
        self.assertGreater(raw.max(), 0.15)          # 補正しないと大きくずれる

    def test_stationary_is_unchanged(self):
        scan, veh, spec = _scan(0.0, 0.0)
        err = _wall_error(scan, veh, spec, correct=False)
        self.assertLess(np.percentile(err, 99), 0.04)


if __name__ == "__main__":
    unittest.main()
