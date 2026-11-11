"""`raspi/auto/e2e_fusion.py`（経路点を追う planner）と `raspi/core/e2e_fusion_preproc.py` のテスト。

モデルは要らない。`E2EFusionPlan` を手で作り、planner が「撮影後に動いたぶん経路をずらして追う」
「自信の無い経路では走らない」「速度を物理の上限で抑える」ことを確かめる。
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.e2e_fusion import E2EFusion, PoseTrack  # noqa: E402
from raspi.auto.registry import PLANNERS, merged_params  # noqa: E402
from raspi.core import e2e_fusion_preproc as P  # noqa: E402
from raspi.msgs import E2EFusionPlan, VehicleState  # noqa: E402
from raspi.msgs.types import TOPIC_E2E_FUSION_PLAN, TOPIC_TYPES  # noqa: E402

NS = 1_000_000_000
T0 = 5 * NS
K, DS = 32, 0.25


def _vs(t: int, speed: float = 1.0, yaw_rate: float = 0.0, steer: float = 0.0) -> VehicleState:
    return VehicleState(t_capture=t, speed=speed, yaw_rate=yaw_rate, steer_actual=steer,
                        stopped=speed < 0.02)


def _arc(kappa: float, k: int = K, ds: float = DS) -> tuple[list[float], list[float]]:
    """原点から前へ出る曲率 `kappa` の円弧の上の、弧長 `ds` おきの点（車体座標）。"""
    s = ds * np.arange(1, k + 1)
    if abs(kappa) < 1e-9:
        return s.tolist(), [0.0] * k
    return (np.sin(kappa * s) / kappa).tolist(), ((1.0 - np.cos(kappa * s)) / kappa).tolist()


def _plan(kappa: float = 0.0, *, t: int = T0, v: float = 2.0, b: float = 0.1, steer: float | None = None,
          **kw) -> E2EFusionPlan:
    x, y = _arc(kappa)
    if steer is None:
        steer = math.atan(0.23 * kappa)
    return E2EFusionPlan(ready=True, t_capture=t, steer=steer, speed=v, path_x=x, path_y=y,
                         path_v=[v] * K, path_b=[b] * K, ds=DS, has_image=True, has_lidar=True, **kw)


def _planner(speed: float = 1.0, yaw_rate: float = 0.0, *, until: int = T0) -> tuple[E2EFusion, VehicleState]:
    """`until` まで 100Hz の車両状態を流した planner と、最後の車両状態。"""
    pl = E2EFusion()
    vs = None
    for t in range(until - NS // 2, until + 1, 10_000_000):
        vs = _vs(t, speed, yaw_rate)
        pl.on_vehicle_state(vs)
    return pl, vs


class TestPreproc(unittest.TestCase):
    def test_lidar_input_stacks_newest_first_and_fills_missing_history(self):
        new = np.stack([np.full(5, 0.2, np.float32), np.ones(5, np.float32)])
        old = np.stack([np.full(5, 0.6, np.float32), np.zeros(5, np.float32)])
        x = P.lidar_input([new, old, None], 3)
        self.assertEqual(x.shape, (1, 4, 5))
        np.testing.assert_allclose(x[0, 0], 0.2)       # 最新
        np.testing.assert_allclose(x[0, 1], 0.6)       # 1周前
        np.testing.assert_allclose(x[0, 2], 0.6)       # 2周前は無い → それより新しい周で埋める
        np.testing.assert_allclose(x[0, 3], 1.0)       # 最新の周の「実測点か」
        with self.assertRaises(ValueError):
            P.lidar_input([None, old], 3)

    def test_lidar_rows_normalizes_and_clips(self):
        r = P.lidar_rows([0.0, 4.0, 20.0], [False, True, True], 8.0)
        np.testing.assert_allclose(r, [[0.0, 0.5, 1.0], [0.0, 1.0, 1.0]])

    def test_decode_output_layout(self):
        k = 3
        vec = np.concatenate([[0.5, 0.4], [0.1, 0.2, 0.3], [-0.1, 0.0, 0.1], [0.2, 0.4, 0.6], [0.05, 0.1, 0.2]])
        o = P.decode_output(vec, k=k, max_steer=0.4, speed_ref=5.0, xy_scale=8.0)
        self.assertAlmostEqual(o.steer, 0.2)
        self.assertAlmostEqual(o.speed, 2.0)
        np.testing.assert_allclose(o.path_xy, [[0.8, -0.8], [1.6, 0.0], [2.4, 0.8]])
        np.testing.assert_allclose(o.path_v, [1.0, 2.0, 3.0])
        np.testing.assert_allclose(o.path_b, [0.05, 0.1, 0.2])

    def test_decode_output_rejects_wrong_size_and_nan(self):
        with self.assertRaises(ValueError):
            P.decode_output(np.zeros(5), k=3, max_steer=0.4, speed_ref=5.0, xy_scale=8.0)
        bad = np.zeros(P.output_size(3))
        bad[4] = np.nan
        with self.assertRaises(ValueError):
            P.decode_output(bad, k=3, max_steer=0.4, speed_ref=5.0, xy_scale=8.0)


class TestPoseTrack(unittest.TestCase):
    def test_integrates_a_circle_and_interpolates_history(self):
        tr = PoseTrack()
        for i in range(101):                             # 1s・1m/s・0.5rad/s
            tr.update(_vs(T0 + i * 10_000_000, 1.0, 0.5))
        self.assertAlmostEqual(tr.yaw, 0.5, places=3)
        self.assertAlmostEqual(tr.x, math.sin(0.5) / 0.5, places=3)
        self.assertAlmostEqual(tr.y, (1 - math.cos(0.5)) / 0.5, places=3)
        mid = tr.at(T0 + NS // 2 + 5_000_000)            # 0.505s
        self.assertAlmostEqual(mid[2], 0.2525, places=3)
        self.assertIsNone(tr.at(T0 - NS))                # 履歴より前
        self.assertEqual(tr.at(T0 + 10 * NS), tr.pose)   # 未来は最新

    def test_stopped_car_does_not_move(self):
        tr = PoseTrack()
        for i in range(50):
            tr.update(VehicleState(t_capture=T0 + i * 10_000_000, speed=0.3, stopped=True))
        self.assertEqual((tr.x, tr.y), (0.0, 0.0))


class TestRegistration(unittest.TestCase):
    def test_planner_and_topic_are_registered(self):
        self.assertIs(PLANNERS["e2e_fusion"], E2EFusion)
        self.assertIs(TOPIC_TYPES[TOPIC_E2E_FUSION_PLAN], E2EFusionPlan)
        from raspi.bus.zbus import TOPIC_OWNER
        self.assertEqual(TOPIC_OWNER[TOPIC_E2E_FUSION_PLAN], "e2e_fusion")

    def test_only_human_intent_params_are_visible(self):
        shown = [s.key for s in E2EFusion.params if not s.hidden]
        self.assertEqual(shown, ["max_speed", "use_model_speed", "drive_direct"])


class TestPlan(unittest.TestCase):
    def setUp(self):
        self.p = merged_params("e2e_fusion", {"max_speed": 3.0})

    def test_straight_path_goes_straight(self):
        pl, vs = _planner()
        st = pl.plan(_plan(0.0), vs, self.p, 0.1)
        self.assertTrue(st.ready, st.reason)
        # 0 ちょうどではない（同定した舵のオフセットを打ち消すぶんが乗る）
        self.assertAlmostEqual(st.target_steer, 0.0, delta=0.03)
        self.assertGreater(st.target_speed, 0.5)

    def test_left_path_steers_left_and_right_path_right(self):
        pl, vs = _planner()
        zero = pl.plan(_plan(0.0), vs, self.p, 0.1).target_steer
        pl, vs = _planner()
        left = pl.plan(_plan(0.8), vs, self.p, 0.1).target_steer
        pl, vs = _planner()
        right = pl.plan(_plan(-0.8), vs, self.p, 0.1).target_steer
        self.assertGreater(left - zero, 0.08)
        self.assertAlmostEqual(right - zero, -(left - zero), delta=0.02)

    def test_path_is_shifted_by_the_motion_since_capture(self):
        """経路は 0.2s 前の車体座標。その間に左へ回っていたら、同じ直線の経路は今の車から見て右にある。"""
        pl, vs = _planner(speed=1.0, yaw_rate=1.0)           # 左へ回りながら走っている
        st = pl.plan(_plan(0.0, t=T0 - NS // 5, steer=-0.2), vs, self.p, 0.1)
        self.assertTrue(st.ready, st.reason)
        self.assertLess(st.target_steer, -0.05)              # 右へ戻す
        # 同じ経路を「今」撮ったものとして渡すと、まっすぐ
        pl2, vs2 = _planner(speed=1.0, yaw_rate=1.0)
        st2 = pl2.plan(_plan(0.0, t=T0), vs2, self.p, 0.1)
        self.assertGreater(st2.target_steer, st.target_steer + 0.05)

    def test_speed_is_capped_by_max_speed_model_speed_and_curvature(self):
        pl, vs = _planner(speed=2.0)
        fast = pl.plan(_plan(0.0, v=4.5), vs, self.p, 0.1)
        self.assertLessEqual(fast.target_speed, 3.0 + 1e-6)            # 最高速度
        pl, vs = _planner(speed=2.0)
        slow = pl.plan(_plan(0.0, v=1.2), vs, self.p, 0.1)
        self.assertLess(slow.target_speed, 2.0)                        # モデルの速度へ向けて落とす
        pl, vs = _planner(speed=2.0)
        tight = pl.plan(_plan(1.6, v=4.5), vs, self.p, 0.1)            # 半径 0.63m
        v_lat = math.sqrt(pl.vehicle.a_lat_max / 1.6)
        self.assertLess(tight.target_speed, max(v_lat, 2.0) + 0.3)
        self.assertLess(tight.target_speed, fast.target_speed)

    def test_without_model_speed_the_path_end_is_taken_at_a_safe_speed(self):
        p = merged_params("e2e_fusion", {"max_speed": 4.0, "use_model_speed": 0.0})
        pl, vs = _planner(speed=1.0)
        path, why = pl.build_path(_plan(0.0, v=4.0), p, vs.speed)
        self.assertIsNotNone(path, why)
        v_safe = math.sqrt(pl.vehicle.a_lat_max / pl.vehicle.kappa_max)
        self.assertAlmostEqual(float(path.v[-1]), min(4.0, v_safe), places=2)
        self.assertGreater(float(path.v.max()), float(path.v[-1]))     # 途中は加速してよい

    def test_uncertain_path_is_refused(self):
        pl, vs = _planner()
        st = pl.plan(_plan(0.0, b=2.0), vs, self.p, 0.1)
        self.assertFalse(st.ready)
        self.assertIn("自信が無い", st.reason)

    def test_path_uncertain_only_far_away_is_truncated_not_refused(self):
        pl, vs = _planner()
        plan = _plan(0.0)
        plan.path_b = [0.1] * 12 + [2.0] * (K - 12)                    # 3m 先から先は分からない
        st = pl.plan(plan, vs, self.p, 0.1)
        self.assertTrue(st.ready, st.reason)
        self.assertIn("3.0m", st.reason)

    def test_not_ready_nan_stale_and_backward_plans_stop(self):
        pl, vs = _planner()
        self.assertEqual(pl.plan(E2EFusionPlan(ready=False, reason="モデルが無い", t_capture=T0), vs, self.p,
                                 0.1).reason, "モデルが無い")
        self.assertFalse(pl.plan(_plan(0.0, steer=float("nan")), vs, self.p, 0.1).ready)
        bad = _plan(0.0)
        bad.path_y[3] = float("inf")
        self.assertFalse(pl.plan(bad, vs, self.p, 0.1).ready)
        self.assertIn("古い", pl.plan(_plan(0.0, t=T0 - NS), vs, self.p, 0.1).reason)
        back = _plan(0.0)
        back.path_x = [-x for x in back.path_x]
        self.assertIn("後ろ", pl.plan(back, vs, self.p, 0.1).reason)
        self.assertFalse(pl.plan(_plan(0.0), None, self.p, 0.1).ready)

    def test_disagreement_between_path_and_direct_steer_slows_down(self):
        pl, vs = _planner(speed=2.0)
        agree = pl.plan(_plan(0.0, v=2.5), vs, self.p, 0.1)
        pl, vs = _planner(speed=2.0)
        clash = pl.plan(_plan(0.0, v=2.5, steer=0.35), vs, self.p, 0.1)   # 経路は直進、直接出力は左 20°
        self.assertIn("食い違い", clash.reason)
        self.assertAlmostEqual(clash.target_speed, agree.target_speed * self.p["disagree_slow"], places=3)
        self.assertAlmostEqual(clash.target_steer, agree.target_steer, places=6)   # 舵は経路のまま

    def test_drive_direct_uses_the_direct_outputs(self):
        p = merged_params("e2e_fusion", {"max_speed": 3.0, "drive_direct": 1.0})
        self.assertEqual(p["drive_direct"], 1.0)
        pl, vs = _planner()
        st = pl.plan(_plan(0.8, v=1.5, steer=0.05), vs, p, 0.1)
        self.assertAlmostEqual(st.target_steer, 0.05)
        self.assertAlmostEqual(st.target_speed, 1.5)
        self.assertIsNone(pl.tick(vs))                                 # 直接出力では合間に追い直さない


class TestTick(unittest.TestCase):
    def test_tick_keeps_following_the_same_path_as_the_car_moves(self):
        p = merged_params("e2e_fusion", {"max_speed": 3.0})
        pl, vs = _planner(speed=1.0)
        st = pl.plan(_plan(0.8), vs, p, 0.1)
        # 直進のまま 0.08s 進む（経路は左へ曲がっていくので、車は経路の右に外れていく）
        steer = st.target_steer
        for i in range(1, 9):
            vs = _vs(T0 + i * 10_000_000, 1.0, 0.0)
            pl.on_vehicle_state(vs)
        out = pl.tick(vs)
        self.assertIsNotNone(out)
        # ★ この経路は 8m で1周して車の近くへ戻ってくる。最寄りの点を経路全体から探すと終点に飛び、
        #   舵がほぼ 0 になる（実際に起きた）。前回居た点の近くだけで探すこと
        self.assertGreater(out[0], steer)                              # 左へ切り足す
        self.assertIsNotNone(out[1])

    def test_tick_does_nothing_before_a_plan_and_after_reset(self):
        pl, vs = _planner()
        self.assertIsNone(pl.tick(vs))
        pl.plan(_plan(0.0), vs, merged_params("e2e_fusion", {}), 0.1)
        self.assertIsNotNone(pl.tick(vs))
        pl.reset()
        self.assertIsNone(pl.tick(vs))


if __name__ == "__main__":
    unittest.main()
