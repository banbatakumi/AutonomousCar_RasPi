"""MPC 経路追従（`nav/mpc.py`）のテスト（2026-10-10）。

経路座標の線形化自転車モデルで舵を決める。ここでは車を自転車モデルで動かす閉ループで、
収束・制約・開経路の終端・計算時間を確かめる。
"""

import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.registry import PLANNERS  # noqa: E402
from raspi.nav.mpc import MPCConfig, MPCFollower  # noqa: E402
from raspi.nav.purepursuit import SteerMap  # noqa: E402
from raspi.tests.test_laptime import _line, stadium  # noqa: E402

L = 0.23


def run(path, pose, cfg, v=2.0, t=6.0, dt=0.05):
    """自転車モデルの閉ループ。舵は指令どおり（遅れなし）。戻り値は (横偏差列, 舵列)。"""
    mpc = MPCFollower()
    x, y, yaw = pose
    steer = 0.0
    cross, steers = [], []
    for _ in range(int(t / dt)):
        pp = mpc.follow(path, (x, y, yaw), v, steer, cfg, hint=-1)
        steer = pp.steer
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += v / L * math.tan(steer) * dt
        cross.append(pp.cross_track)
        steers.append(steer)
    return np.array(cross), np.array(steers)


def straight(length=20.0, v=2.0):
    n = int(length / 0.05)
    return _line(np.c_[np.arange(n) * 0.05, np.zeros(n)], closed=False, v=v)


class TestMPC(unittest.TestCase):
    def setUp(self):
        self.cfg = MPCConfig(wheelbase=L, delay_s=0.0, period=0.05)

    def test_直線で横ずれが収束する(self):
        cross, _ = run(straight(), (0.0, 0.3, 0.0), self.cfg)
        self.assertLess(abs(cross[-1]), 0.01)
        self.assertGreater(cross[0], 0.25)         # 左にずれていれば正

    def test_周回を内外へ偏らず回る(self):
        path = _line(stadium(r=1.0, straight=4.0), closed=True, v=1.5)
        t = path.xy[1] - path.xy[0]
        cross, _ = run(path, (path.xy[0, 0], path.xy[0, 1], math.atan2(t[1], t[0])),
                       self.cfg, v=1.5, t=12.0)
        self.assertLess(np.max(np.abs(cross)), 0.05)        # 直線→半円の入りの一瞬を含む

    def test_舵レートと舵角の制約を守る(self):
        cfg = self.cfg._replace(steer_rate=3.0)
        _, st = run(straight(), (0.0, 1.0, 0.0), cfg, dt=cfg.dt)
        self.assertLessEqual(np.max(np.abs(st)), cfg.max_steer + 1e-9)
        self.assertLessEqual(np.max(np.abs(np.diff(st))), cfg.steer_rate * cfg.dt + 1e-6)

    def test_開経路の終端で舵が暴れない(self):
        _, st = run(straight(3.0, 1.0), (2.0, 0.0, 0.0), self.cfg, v=1.0, t=3.0)
        self.assertLess(np.max(np.abs(st)), 0.05)

    def test_舵の効きを通した指令を返す(self):
        sm = SteerMap(servo_gain=0.5)
        a = MPCFollower().follow(straight(), (0.0, 0.03, 0.0), 2.0, 0.0,
                                 self.cfg._replace(steer_map=sm, servo_gain=0.5, max_steer=1.0))
        b = MPCFollower().follow(straight(), (0.0, 0.03, 0.0), 2.0, 0.0,
                                 self.cfg._replace(max_steer=1.0))
        # 効きが半分なら、同じ実効舵角のために指令は2倍（左にずれているので右＝負）
        self.assertLess(b.steer, 0.0)
        self.assertAlmostEqual(a.steer, 2.0 * b.steer, delta=abs(b.steer) * 0.05)

    def test_計算時間(self):
        path = _line(stadium(), closed=True, v=2.0)
        mpc = MPCFollower()
        mpc.follow(path, (0.0, -1.0, 0.0), 2.0, 0.0, self.cfg)
        t0 = time.perf_counter()
        for _ in range(50):
            mpc.follow(path, (0.0, -0.9, 0.1), 2.0, 0.0, self.cfg)
        self.assertLess((time.perf_counter() - t0) / 50, 0.005)

    def test_同じ時刻に2回呼んでも履歴は1つしか進まない(self):
        """回避経路へ乗り換えた周期は `follow` を2回呼ぶ。以前は「呼ぶたびに1周期進む」並びで、
        2回目で履歴が2つ進み、遅延の予測がずれた。"""
        cfg = MPCConfig(wheelbase=L, delay_s=0.15)
        path = straight()
        mpc = MPCFollower()
        for k in range(5):
            mpc.follow(path, (k * 0.2, 0.2, 0.0), 2.0, 0.0, cfg, now=1.0 + 0.1 * k)
        n = len(mpc._sent)
        a = mpc.follow(path, (1.0, 0.2, 0.0), 2.0, 0.0, cfg, now=1.5)
        b = mpc.follow(path, (1.0, 0.2, 0.0), 2.0, 0.0, cfg, now=1.5)
        self.assertEqual(len(mpc._sent), min(n + 1, len(mpc._sent)))
        self.assertEqual(sum(1 for t, _ in mpc._sent if abs(t - 1.5) < 1e-9), 1)
        self.assertAlmostEqual(a.steer, b.steer, places=6)

    def test_履歴は時刻で持ち遅延より古いものは捨てる(self):
        cfg = MPCConfig(wheelbase=L, delay_s=0.06)
        path = straight()
        mpc = MPCFollower()
        for k in range(20):                        # 50Hz で呼ぶ
            mpc.follow(path, (k * 0.04, 0.1, 0.0), 2.0, 0.0, cfg, now=k * 0.02)
        ages = [19 * 0.02 - t for t, _ in mpc._sent]
        self.assertTrue(all(a < 0.06 + 1e-6 for a in ages))
        self.assertGreaterEqual(len(mpc._sent), 3)

    def test_50Hzで呼んでも収束する(self):
        cfg = MPCConfig(wheelbase=L, delay_s=0.0)
        mpc = MPCFollower()
        x, y, yaw, steer = 0.0, 0.3, 0.0, 0.0
        path = straight()
        for k in range(300):
            pp = mpc.follow(path, (x, y, yaw), 2.0, steer, cfg, now=k * 0.02)
            steer = pp.steer
            x += 2.0 * math.cos(yaw) * 0.02
            y += 2.0 * math.sin(yaw) * 0.02
            yaw += 2.0 / L * math.tan(steer) * 0.02
        self.assertLess(abs(y), 0.02)

    def test_経路の点の間を進んでも舵が振れない(self):
        """★ 経路の上をぴったり走る車には、点と点の間のどこに居ても同じ舵を返す。

        偏差を「最寄りの点」の接線で測ると、点の間（10cm）を進む間に向きの偏差が κ·刻み だけ
        増えて次の点で跳ね戻り、50Hz では舵指令がのこぎり波になる（実機の記録: ヘアピンで
        約 11Hz・±0.05rad。半径 0.55m の円では指令の幅 0.16rad）。経路上へ射影した位置で測る。
        """
        from raspi.nav import raceline as rl
        r = 0.55
        n = int(round(2.0 * math.pi * r / 0.1))
        th = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        xy = np.c_[r * np.cos(th), r * np.sin(th)]
        path = rl.RaceLine(xy=xy, v=np.full(n, 1.2), kappa=rl.curvature(xy), length=2.0 * math.pi * r)
        cfg = MPCConfig()
        steer = math.atan(cfg.wheelbase / r)
        m = MPCFollower()
        out = []
        for k in range(60):
            a = 0.3 + k * (1.2 * 0.02) / r             # 円の上を 1.2m/s・50Hz で進む（偏差 0）
            pose = (r * math.cos(a), r * math.sin(a), a + math.pi / 2.0)
            out.append(m.follow(path, pose, 1.2, steer, cfg, now=k * 0.02).steer)
        out = np.asarray(out[5:])
        self.assertLess(out.max() - out.min(), 0.005)
        self.assertAlmostEqual(float(out.mean()), steer, delta=0.02)

    def test_plannerの切替パラメータ(self):
        keys = {s.key for s in PLANNERS["slam2d_raceline"].params}
        self.assertTrue({"ctrl_mode", "mpc_horizon", "mpc_dt", "mpc_q_y", "mpc_q_psi",
                         "mpc_r_d"} <= keys)


if __name__ == "__main__":
    unittest.main()
