"""`ml_lidar/env.py` のテスト。

センターライン弧長進捗（ラップアラウンド含む）は正方形ループの合成データで
決定的に検証し、reset/step の配線と衝突終端は`course_gen`の実コースで
スモークテストする。
"""

import dataclasses
import math
import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

import numpy as np  # noqa: E402

from ml_lidar.course_gen import random_walk_loop_course  # noqa: E402
from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from sim.lidar import SCAN_HZ  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402


def _square_centerline(side: float = 2.0, step: float = 0.1) -> np.ndarray:
    """一辺`side`の正方形ループのセンターライン `(N,3)`（x,y,yaw）。閉じる最終点は含めない。"""
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side)]
    yaws = [0.0, math.pi / 2, math.pi, -math.pi / 2]
    pts = []
    for (cx, cy), yaw, (nx, ny) in zip(corners, yaws, corners[1:] + corners[:1]):
        length = math.hypot(nx - cx, ny - cy)
        n = max(1, int(round(length / step)))
        for i in range(n):
            t = i / n
            pts.append((cx + (nx - cx) * t, cy + (ny - cy) * t, yaw))
    return np.asarray(pts, dtype=np.float64)


class TestCenterlineProgress(unittest.TestCase):
    """`_progress_delta()`のラップアラウンド処理を、一辺2mの正方形ループ（周長8m）で検証する。"""

    def setUp(self) -> None:
        self.env = LidarE2EEnv(EnvConfig())
        course = types.SimpleNamespace(centerline=_square_centerline())
        self.env._prepare_centerline(course)

    def test_total_length_matches_perimeter(self) -> None:
        self.assertAlmostEqual(self.env._total_length, 8.0, delta=0.2)

    def test_forward_progress_is_positive(self) -> None:
        s0 = self.env._project_arc_length(0.0, 0.0)
        self.env._s_prev = s0
        s1 = self.env._project_arc_length(1.0, 0.0)
        self.assertGreater(self.env._progress_delta(s1), 0.0)

    def test_reverse_progress_is_negative_before_clipping(self) -> None:
        """`_progress_delta()`自体はクリップしない（クリップは`step()`の報酬計算側の責務）。"""
        s0 = self.env._project_arc_length(1.0, 0.0)
        self.env._s_prev = s0
        s1 = self.env._project_arc_length(0.0, 0.0)
        self.assertLess(self.env._progress_delta(s1), 0.0)

    def test_wraparound_finish_to_start_stays_positive(self) -> None:
        """ゴール手前(0, 0.05)からスタート直後(0.05, 0)へまたぐ動き。

        補正が無いと `delta ≈ -total_length` (≈-7.9m) になり、「後退した」と
        誤判定されて周回完走もprogressも壊れる。正しくは小さな正の値になるべき。
        """
        # サンプリング点(0.1m間隔)の中間を突くと最近傍が始点(0,0)と誤タイになるため、
        # 明確にどちらか一方へ寄せた座標を使う
        near_finish = self.env._project_arc_length(0.0, 0.08)  # 最終エッジの終端付近
        self.env._s_prev = near_finish
        near_start = self.env._project_arc_length(0.02, 0.0)   # スタート直後
        delta = self.env._progress_delta(near_start)
        self.assertGreater(delta, 0.0)
        self.assertLess(delta, 0.5)


class TestMinimalReward(unittest.TestCase):
    """v21: 報酬が`progress/dt`＋終端項だけであることを固定する回帰テスト。

    v16〜v20で足した罰則3種（`steer_effort_weight`・`raceline_weight`・
    `steer_angle_weight`）は2026-09-17に全て撤去した（env.py EnvConfig
    docstring「v21」参照）。**罰則をうっかり足し直すと静かに壊れる**類の
    不変条件なので、値そのものを突き合わせて固定しておく。
    """

    def test_nonterminal_reward_is_exactly_progress_rate(self) -> None:
        """衝突・完走していないステップの報酬は`info["progress_m"]/dt`に厳密一致する。"""
        cfg = EnvConfig(randomize_lidar=False, randomize_dynamics=False)
        course = random_walk_loop_course(np.random.default_rng(3))
        env = LidarE2EEnv(cfg, course=course)
        env.reset(seed=0)

        checked = 0
        for a0 in (1.0, -1.0, 0.4, 0.0):          # 据え切り・切り戻しも含めて振る
            action = np.array([a0, 0.5], dtype=np.float32)
            for _ in range(10):
                _, reward, terminated, _, info = env.step(action)
                if terminated:
                    break
                self.assertAlmostEqual(reward, info["progress_m"] / cfg.dt, places=9)
                checked += 1
            if terminated:
                break
        self.assertGreater(checked, 0, "非終端ステップが1つも無かった")

    def test_removed_reward_weights_are_gone(self) -> None:
        """撤去した罰則の重みが`EnvConfig`に復活していないこと。"""
        fields = {f.name for f in dataclasses.fields(EnvConfig)}
        for name in ("steer_effort_weight", "raceline_weight", "raceline_tolerance_m",
                     "steer_angle_weight", "steer_rate_weight"):
            self.assertNotIn(name, fields,
                             f"撤去済みの罰則{name}が復活している（env.py docstring「v21」参照）")


class TestControlPeriod(unittest.TestCase):
    """v21b: 制御周期がLiDARの回転周期＝実車の計画周期と一致していることを固定する。"""

    def test_dt_matches_lidar_scan_period(self) -> None:
        """`dt`が`sim.lidar.SCAN_HZ`の周期と一致すること。

        ずれると①観測のスキャン部が更新されないステップが生まれ（`dt=0.05`では
        実測54%）、②`raspi/nodes/planning_node.py:_replan()`が`scan.seq`単位でしか
        `plan()`を呼ばない実車と決定レートがずれる（env.py docstring「v21b」参照）。
        `SCAN_HZ`側が変わったらこのテストが落ちて気づけるように、値を直書きせず
        `sim.lidar`から引く。
        """
        self.assertAlmostEqual(EnvConfig().dt, 1.0 / SCAN_HZ, places=9)

    def test_scan_is_fresh_on_almost_every_step(self) -> None:
        """実際に回して、観測のスキャン部が前ステップと同一になる割合が十分低いこと。

        `dt`と`SCAN_HZ`が一致していても、LiDARの遅延ジッタぶんは据え置きが残る
        （実測10〜12%）。`dt=0.05`時代の54%との差は大きいので、緩い上限で十分に
        退行を検出できる。
        """
        cfg = EnvConfig(randomize_lidar=True, randomize_dynamics=False)
        action = np.array([0.05, 0.4], dtype=np.float32)   # ゆるい旋回で壁まで走る
        same = total = 0
        for seed in range(11, 19):                          # 1本は早期に衝突しうるので束ねる
            env = LidarE2EEnv(cfg, seed=seed)
            obs, _ = env.reset(seed=seed)
            prev = np.asarray(obs[:-2]).copy()
            for _ in range(60):
                obs, _, terminated, truncated, _ = env.step(action)
                cur = np.asarray(obs[:-2])
                same += int(np.array_equal(cur, prev))
                total += 1
                prev = cur.copy()
                if terminated or truncated:
                    break
        self.assertGreater(total, 60)
        self.assertLess(same / total, 0.30, "スキャンが更新されないステップが多すぎる")


class TestEnvSmoke(unittest.TestCase):
    """reset/stepの配線・観測の形状・衝突終端をコース生成込みで確認する。"""

    def _make_env(self, seed: int, **cfg_kwargs) -> LidarE2EEnv:
        course = random_walk_loop_course(np.random.default_rng(seed))
        cfg = EnvConfig(randomize_lidar=False, randomize_dynamics=False, **cfg_kwargs)
        return LidarE2EEnv(cfg, course=course)

    def test_reset_returns_obs_within_bounds(self) -> None:
        env = self._make_env(seed=1)
        obs, info = env.reset(seed=0)
        self.assertEqual(obs.shape, env.observation_space.shape)
        self.assertTrue(np.all(obs >= env.observation_space.low - 1e-4))
        self.assertTrue(np.all(obs <= env.observation_space.high + 1e-4))
        self.assertEqual(info, {})

    def test_step_returns_expected_shapes(self) -> None:
        env = self._make_env(seed=1)
        env.reset(seed=0)
        obs, reward, terminated, truncated, info = env.step(np.array([0.0, 0.3], dtype=np.float32))
        self.assertEqual(obs.shape, env.observation_space.shape)
        self.assertIsInstance(float(reward), float)
        self.assertFalse(truncated)
        self.assertIn("progress_m", info)
        self.assertIn("collided", info)

    def test_command_carries_gui_rate_limits(self) -> None:
        """実機のCOMMANDと同じく、GUIが毎回載せるレート制限（`[safety]`）が入る。"""
        env = self._make_env(seed=1)
        env.reset(seed=0)
        env.step(np.array([0.0, 0.3], dtype=np.float32))
        sp = env.vehicle.spec
        self.assertGreater(sp.cmd_steer_rate_limit_rad_s, 0.0)
        self.assertEqual(env.vehicle.cmd.steer_rate_limit, sp.cmd_steer_rate_limit_rad_s)
        self.assertEqual(env.vehicle.cmd.accel_limit, sp.cmd_accel_limit_m_s2)

    def test_control_latency_delays_the_new_command(self) -> None:
        """`control_latency_s`のあいだは前の指令が効く（発進が遅れる）。"""
        speeds = []
        for latency in (0.0, 0.05):
            env = self._make_env(seed=1)
            env.reset(seed=0)
            env.vehicle.spec.control_latency_s = latency
            env.step(np.array([0.0, 1.0], dtype=np.float32))
            speeds.append(env.vehicle.speed)
        self.assertGreater(speeds[0], 0.0)
        self.assertLess(speeds[1], speeds[0] * 0.8)

    def test_collision_terminates_with_penalty(self) -> None:
        env = self._make_env(seed=2)
        env.reset(seed=0)
        # 全開加速＋目一杯の据え切りを続ければ、有限ステップ内に必ず壁へ当たる
        # （最大舵角での旋回半径はコース半径よりはるかに小さい）
        action = np.array([1.0, 1.0], dtype=np.float32)
        terminated = False
        reward = 0.0
        info: dict = {}
        for _ in range(400):
            obs, reward, terminated, truncated, info = env.step(action)
            if terminated:
                break
        self.assertTrue(terminated, "400ステップ以内に衝突しなかった")
        self.assertTrue(info["collided"])
        self.assertLess(reward, -8.0)


class TestWindowedProgress(unittest.TestCase):
    """窓付き射影は前回の添字の近傍だけを探し、大域最近傍が飛ぶ場所でも1ステップの進捗が
    `max_speed*dt*1.5`を超えない。既定OFFは従来の大域最近傍のまま。"""

    @staticmethod
    def _env(windowed: bool) -> LidarE2EEnv:
        env = LidarE2EEnv(EnvConfig(windowed_progress=windowed))
        env._prepare_centerline(types.SimpleNamespace(centerline=_square_centerline(side=2.0)))
        return env

    def test_default_is_global_nearest(self) -> None:
        self.assertFalse(EnvConfig().windowed_progress)

    def test_window_ignores_far_section(self) -> None:
        # 往路と復路が0.3mしか離れていない細長い周回。往路上の添字を覚えていれば、
        # 復路側に少しだけ近い点でも往路へ射影する（大域最近傍は復路へ飛ぶ）
        out = [(x, 0.0, 0.0) for x in np.arange(0.0, 6.0, 0.1)]
        back = [(x, 0.3, math.pi) for x in np.arange(6.0, 0.0, -0.1)]
        env = LidarE2EEnv(EnvConfig(windowed_progress=True))
        env._prepare_centerline(types.SimpleNamespace(centerline=np.asarray(out + back)))
        env._s_idx = 30
        env._last_proj_idx = 30
        s_win = env._project_arc_length(3.0, 0.17)
        s_glob = env._project_arc_length(3.0, 0.17, windowed=False)
        self.assertAlmostEqual(s_win, 3.0, delta=0.11)
        self.assertGreater(s_glob, 8.0)

    def test_window_wraps_around_loop_end(self) -> None:
        env = self._env(True)
        n = len(env._centerline_xy)
        env._s_idx = n - 1
        env._last_proj_idx = n - 1
        # 始点（添字0）は循環で窓に入る
        s = env._project_arc_length(0.0, 0.0)
        self.assertAlmostEqual(s, 0.0, delta=0.06)

    def test_step_progress_is_clamped(self) -> None:
        cfg = EnvConfig(windowed_progress=True)
        env = LidarE2EEnv(cfg, seed=0)
        env.reset(seed=0)
        limit = cfg.max_speed * cfg.dt * 1.5
        for _ in range(60):
            _, _, term, _, info = env.step(np.array([0.0, 1.0], dtype=np.float32))
            self.assertLessEqual(info["progress_m"], limit + 1e-9)
            if term:
                break


class TestAutoStop(unittest.TestCase):
    def _env_with_wall(self, auto_stop: bool, gap_m: float) -> LidarE2EEnv:
        """車体先端から`gap_m`前に壁（LiDARの正面5点）があるスキャンを持つ環境。"""
        env = LidarE2EEnv(EnvConfig(auto_stop=auto_stop, randomize_lidar=False,
                                    randomize_dynamics=False), seed=0)
        env.reset(seed=0)
        scan = env._scan
        dist_m = gap_m + env._front_x - env._lidar_xy[0]
        d = [0.0] * 360
        for deg in (-3, -1, 0, 1, 3):
            d[deg % 360] = dist_m
        scan.dist = d
        return env

    def test_default_off(self) -> None:
        self.assertFalse(EnvConfig().auto_stop)

    def test_latches_when_obstacle_within_stop_distance(self) -> None:
        env = self._env_with_wall(True, 0.1)          # 静止時のd_stop=0.15m以内
        env._update_auto_stop()
        self.assertTrue(env._auto_stop_latched)

    def test_far_obstacle_does_not_latch(self) -> None:
        env = self._env_with_wall(True, 3.0)
        env._update_auto_stop()
        self.assertFalse(env._auto_stop_latched)

    def test_stop_distance_grows_with_speed(self) -> None:
        env = self._env_with_wall(True, 0.9)
        env._update_auto_stop()
        self.assertFalse(env._auto_stop_latched)          # 静止（d_stop=0.15m）では作動しない
        env.vehicle.speed_measured = 2.0                   # 2m/s: 0.2+0.667+0.15
        env._update_auto_stop()
        self.assertTrue(env._auto_stop_latched)

    def test_wide_off_axis_points_are_ignored(self) -> None:
        env = self._env_with_wall(True, 0.1)
        env._scan.dist = [0.0] * 360
        for deg in (60, 61, 62, 63):                       # 横向きの壁（矩形の外）
            env._scan.dist[deg] = 0.3
        env._update_auto_stop()
        self.assertFalse(env._auto_stop_latched)

    def test_step_brakes_to_standstill_and_reports(self) -> None:
        env = LidarE2EEnv(EnvConfig(auto_stop=True, randomize_lidar=False,
                                    randomize_dynamics=False), seed=3)
        env.reset(seed=3)
        stopped = False
        for _ in range(200):
            _, _, term, _, info = env.step(np.array([0.0, 1.0], dtype=np.float32))
            if info["auto_stop"] and info["speed"] < 0.05:
                stopped = True
            if term:
                break
        self.assertGreater(info["auto_stop_steps"], 0)
        self.assertTrue(stopped or info["collided"])


class TestExtraRandomization(unittest.TestCase):
    def test_all_off_consumes_no_extra_random_numbers(self) -> None:
        a = LidarE2EEnv(EnvConfig(), seed=5)
        b = LidarE2EEnv(EnvConfig(lidar_mount_yaw_err_deg=0.0, steer_offset_jitter_rad=0.0), seed=5)
        a.reset(seed=5)
        b.reset(seed=5)
        self.assertEqual(a._rng.random(), b._rng.random())

    def test_lidar_delay_holds_scan_back(self) -> None:
        env = LidarE2EEnv(EnvConfig(lidar_delay=True, lidar_dr=True), seed=1)
        env.reset(seed=1)
        self.assertGreaterEqual(env.lidar.params.lidar_delay_ms, 0.0)
        env.step(np.array([0.0, 0.0], dtype=np.float32))
        self.assertLessEqual(len(env._lidar_queue), 3 * 12)

    def test_mount_error_and_steer_offset_are_applied(self) -> None:
        env = LidarE2EEnv(EnvConfig(lidar_mount_yaw_err_deg=3.0, steer_offset_jitter_rad=0.02), seed=2)
        base = VehicleSpec.load()
        env.reset(seed=2)
        self.assertNotEqual(env.lidar.mount[2], base.sensor_pose("lidar")[2])
        self.assertNotEqual(env.vehicle.spec.steer_offset_rad, base.steer_offset_rad)
        self.assertLessEqual(abs(env.lidar.mount[2] - base.sensor_pose("lidar")[2]),
                             math.radians(3.0) + 1e-9)


if __name__ == "__main__":
    unittest.main()
