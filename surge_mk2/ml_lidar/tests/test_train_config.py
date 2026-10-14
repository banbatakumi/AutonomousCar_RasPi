"""`train_rl.py`の新規run／既存run（resume・旧run）の設定の扱いのテスト。

学習の挙動を変える項目（窓付き射影・自動停止・LiDARのランダム化・選定方法）は、
新規runだけが明示してON、保存値が無い旧runは従来値で動くことを確かめる。
"""

import dataclasses
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_lidar.env import EnvConfig  # noqa: E402
from ml_lidar.train_rl import (  # noqa: E402
    GeneralizationEvalCallback,
    build_env_config,
    build_selection,
    load_saved_env_config,
    load_saved_selection,
    parse_args,
    resume_seed_offset,
)


class TestNewRunDefaults(unittest.TestCase):
    def test_new_run_turns_the_new_behaviours_on(self) -> None:
        cfg = build_env_config(parse_args(["--run-name", "x"]))
        self.assertTrue(cfg.windowed_progress)
        self.assertTrue(cfg.auto_stop)
        self.assertTrue(cfg.lidar_dr)
        self.assertTrue(cfg.lidar_delay)
        self.assertGreater(cfg.lidar_mount_yaw_err_deg, 0.0)
        self.assertGreater(cfg.steer_offset_jitter_rad, 0.0)

    def test_flags_can_turn_them_off(self) -> None:
        cfg = build_env_config(parse_args([
            "--run-name", "x", "--no-windowed-progress", "--no-auto-stop", "--no-lidar-dr",
            "--no-lidar-delay", "--lidar-mount-yaw-err-deg", "0", "--steer-offset-jitter-rad", "0"]))
        self.assertFalse(cfg.windowed_progress or cfg.auto_stop or cfg.lidar_dr or cfg.lidar_delay)
        self.assertEqual((cfg.lidar_mount_yaw_err_deg, cfg.steer_offset_jitter_rad), (0.0, 0.0))

    def test_envconfig_defaults_stay_legacy(self) -> None:
        cfg = EnvConfig()
        self.assertFalse(cfg.windowed_progress or cfg.auto_stop or cfg.lidar_dr or cfg.lidar_delay)
        self.assertEqual((cfg.lidar_mount_yaw_err_deg, cfg.steer_offset_jitter_rad), (0.0, 0.0))


class TestSavedConfig(unittest.TestCase):
    def test_old_run_without_new_keys_loads_legacy_values(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / "env_config.json").write_text(json.dumps(
                {"dt": 0.1, "fov_deg": 270.0, "max_speed": 2.0}), encoding="utf-8")
            cfg = load_saved_env_config(run)
            self.assertFalse(cfg.windowed_progress or cfg.auto_stop or cfg.lidar_dr)
            self.assertEqual(load_saved_selection(run), {"fixed_seed": None, "metric": "lap_time"})

    def test_new_run_roundtrip(self) -> None:
        args = parse_args(["--run-name", "x", "--obstacle-prob", "0.5"])
        cfg = build_env_config(args)
        sel = build_selection(args, cfg)
        self.assertEqual(sel["metric"], "fail_rate")
        self.assertEqual(sel["fixed_seed"], args.gen_eval_seed)
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run / "env_config.json").write_text(json.dumps(
                {**dataclasses.asdict(cfg), "selection": sel}), encoding="utf-8")
            self.assertEqual(load_saved_env_config(run), cfg)
            self.assertEqual(load_saved_selection(run), sel)

    def test_no_obstacles_keeps_lap_time_metric(self) -> None:
        args = parse_args(["--run-name", "x", "--obstacle-prob", "0"])
        self.assertEqual(build_selection(args, build_env_config(args))["metric"], "lap_time")


class TestResumeSeed(unittest.TestCase):
    def test_depends_on_resume_point_and_keeps_worker_seeds_apart(self) -> None:
        a = resume_seed_offset(Path("ppo_100000_steps.zip"))
        b = resume_seed_offset(Path("ppo_200000_steps.zip"))
        self.assertNotEqual(a, b)
        self.assertEqual(a, resume_seed_offset(Path("ppo_100000_steps.zip")))
        self.assertEqual(a % 1000, 0)
        self.assertGreater(a, 0)

    def test_final_model_without_a_step_count_still_gives_an_offset(self) -> None:
        self.assertGreater(resume_seed_offset(Path("/nonexistent/final_model.zip")), 0)


class TestSelectionScoreAndSeed(unittest.TestCase):
    def test_fixed_seed_is_used_for_every_evaluation(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            cb = GeneralizationEvalCallback(EnvConfig(), Path(tmp), 1, 2, 5.0,
                                            fixed_seed=7000, metric="fail_rate")
            self.assertEqual(cb.fixed_seed, 7000)
            self.assertEqual(cb.metric, "fail_rate")

    def test_unknown_metric_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError):
                GeneralizationEvalCallback(EnvConfig(), Path(tmp), 1, 2, 5.0, metric="bogus")


if __name__ == "__main__":
    unittest.main()
