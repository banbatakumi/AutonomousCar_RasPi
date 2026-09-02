"""`ml_lidar/app.py`（Tkinter操作パネル）のテスト。

**GUIの見た目・クリック操作はテストしない**（`ml_cam/tests/test_app.py`と同じ方針）。
run名・run一覧・コマンド組み立てというTkinterを知らない純粋関数だけを厳密に
テストし、ウィジェット構築だけ「例外を出さずに組み立てられるか」という
スモークテストに留める（ディスプレイが無い環境ではスキップする）。
"""

import json
import sys
import tempfile
import tkinter as tk
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # ml_lidar/

from app import (  # noqa: E402
    App,
    MODELS_DIR,
    ML_LIDAR_DIR,
    REPO_ROOT,
    RUNS_DIR,
    build_export_cmd,
    build_tensorboard_cmd,
    build_train_cmd,
    build_watch_cmd,
    discover_runs,
    format_run_row,
    list_run_names,
    next_run_name,
    read_note,
    rel,
    venv_bin,
    write_note,
)


def _has_display() -> bool:
    try:
        root = tk.Tk()
        root.withdraw()
        root.destroy()
        return True
    except tk.TclError:
        return False


_HAS_DISPLAY = _has_display()


class TestPaths(unittest.TestCase):
    def test_rel_shortens_paths_under_repo_root(self):
        p = REPO_ROOT / "ml_lidar" / "runs" / "v1"
        self.assertEqual(rel(p), "ml_lidar/runs/v1")

    def test_rel_falls_back_to_absolute_outside_repo(self):
        p = Path("/some/other/place")
        self.assertEqual(rel(p), str(p))

    def test_venv_bin_is_a_sibling_of_python(self):
        self.assertEqual(venv_bin("/x/y/.venv/bin/python", "tensorboard"),
                         "/x/y/.venv/bin/tensorboard")


class TestNextRunName(unittest.TestCase):
    def test_no_existing_runs_suggests_v1(self):
        self.assertEqual(next_run_name([]), "v1")

    def test_ignores_non_v_named_runs(self):
        self.assertEqual(next_run_name(["ppo_e2e", "first_model"]), "v1")

    def test_suggests_next_number_after_the_highest(self):
        self.assertEqual(next_run_name(["v1", "v2"]), "v3")

    def test_ignores_gaps_and_uses_the_max(self):
        self.assertEqual(next_run_name(["v1", "v5", "v3"]), "v6")


class TestListRunNames(unittest.TestCase):
    def test_missing_dir_returns_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(list_run_names(Path(d) / "nope"), [])

    def test_lists_only_directories_sorted(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            (root / "v2").mkdir()
            (root / "v1").mkdir()
            (root / "not_a_dir.txt").write_text("x")
            self.assertEqual(list_run_names(root), ["v1", "v2"])


class TestDiscoverRuns(unittest.TestCase):
    def test_missing_dir_returns_empty(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(discover_runs(Path(d) / "nope"), [])

    def test_reads_config_and_best_model_presence(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            complete = root / "v1"
            complete.mkdir()
            (complete / "run_config.json").write_text(
                json.dumps({"max_speed": 1.5, "max_steer": 0.45}))
            (complete / "best_model.zip").write_bytes(b"")

            partial = root / "v2"
            partial.mkdir()

            runs = {r["name"]: r for r in discover_runs(root)}
            self.assertTrue(runs["v1"]["has_best_model"])
            self.assertEqual(runs["v1"]["config"]["max_speed"], 1.5)
            self.assertFalse(runs["v2"]["has_best_model"])
            self.assertEqual(runs["v2"]["config"], {})

    def test_corrupt_run_config_json_is_treated_as_empty(self):
        with tempfile.TemporaryDirectory() as d:
            root = Path(d)
            broken = root / "v1"
            broken.mkdir()
            (broken / "run_config.json").write_text("{ not json")
            runs = discover_runs(root)
            self.assertEqual(runs[0]["config"], {})


class TestFormatRunRow(unittest.TestCase):
    def test_with_config_shows_speed(self):
        row = format_run_row({"name": "v1", "has_best_model": True,
                              "config": {"max_speed": 1.5, "max_steer": 0.45}})
        self.assertIn("v1", row)
        self.assertIn("speed=1.5", row)
        self.assertIn("✓", row)

    def test_does_not_show_max_steer(self):
        """最大舵角はもう学習ごとに変わらない（常にvehicle.toml由来）ので表示しない
        （2026-08-28、バンビの指摘で削除）。"""
        row = format_run_row({"name": "v1", "has_best_model": True,
                              "config": {"max_speed": 1.5, "max_steer": 0.45}})
        self.assertNotIn("steer", row)

    def test_without_config_or_model_shows_placeholders(self):
        row = format_run_row({"name": "v2", "has_best_model": False, "config": {}})
        self.assertIn("v2", row)
        self.assertIn("run_config無し", row)
        self.assertIn("…", row)


class TestNote(unittest.TestCase):
    """runごとの備考（`<run_dir>/note.txt`）。学習前後いつでも書ける自由記述で、
    エクスポート時に`models/e2e_lidar/<名前>.json`へ運ばれ、実車のGUIにも出る
    （2026-08-29追加）。"""

    def test_read_note_returns_empty_string_when_missing(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(read_note(Path(d)), "")

    def test_write_then_read_note_round_trips(self):
        with tempfile.TemporaryDirectory() as d:
            run_dir = Path(d)
            write_note(run_dir, "グリップ限界+報酬改善版、circuitで学習")
            self.assertEqual(read_note(run_dir), "グリップ限界+報酬改善版、circuitで学習")

    def test_write_note_overwrites_previous_content(self):
        with tempfile.TemporaryDirectory() as d:
            run_dir = Path(d)
            write_note(run_dir, "1回目")
            write_note(run_dir, "2回目に書き直した")
            self.assertEqual(read_note(run_dir), "2回目に書き直した")


class TestBuildCmds(unittest.TestCase):
    def test_train_cmd(self):
        cmd = build_train_cmd("python3", "v3", timesteps=1000, n_envs=4,
                              max_speed=2.0, early_stop_patience=5)
        self.assertEqual(cmd[0], "python3")
        self.assertEqual(cmd[1], str(ML_LIDAR_DIR / "train_rl.py"))
        self.assertEqual(cmd[cmd.index("--out") + 1], "ml_lidar/runs/v3")
        self.assertEqual(cmd[cmd.index("--timesteps") + 1], "1000")
        self.assertEqual(cmd[cmd.index("--n-envs") + 1], "4")
        self.assertEqual(cmd[cmd.index("--max-speed") + 1], "2.0")
        self.assertEqual(cmd[cmd.index("--early-stop-patience") + 1], "5")
        self.assertNotIn("--max-steer", cmd)
        self.assertNotIn("--resume-from", cmd)

    def test_train_cmd_with_resume_from(self):
        cmd = build_train_cmd("python3", "v3", timesteps=1000, n_envs=4, max_speed=2.0,
                              early_stop_patience=5, resume_from="ml_lidar/runs/v2/best_model.zip")
        self.assertEqual(cmd[cmd.index("--resume-from") + 1],
                         "ml_lidar/runs/v2/best_model.zip")

    def test_train_cmd_reward_norm_and_curriculum_frac(self):
        """2026-09-02追加。v10「舵が発散し壁に衝突」の切り分け用にGUIから
        `--no-reward-norm`/`--curriculum-frac`を渡せるようにした
        （`App._build_train_tab()`のチェックボックス・入力欄と対）。"""
        cmd = build_train_cmd("python3", "v3", timesteps=1000, n_envs=4, max_speed=2.0,
                              early_stop_patience=5, reward_norm=False, curriculum_frac=0.0)
        self.assertIn("--no-reward-norm", cmd)
        self.assertEqual(cmd[cmd.index("--curriculum-frac") + 1], "0.0")

    def test_train_cmd_reward_norm_true_omits_no_reward_norm_flag(self):
        cmd = build_train_cmd("python3", "v3", timesteps=1000, n_envs=4, max_speed=2.0,
                              early_stop_patience=5, reward_norm=True)
        self.assertNotIn("--no-reward-norm", cmd)

    def test_export_cmd_does_not_pass_max_speed_or_max_steer(self):
        """run_config.json から自動で読ませるため、明示的には渡さない。"""
        cmd = build_export_cmd("python3", "v3", "my_model")
        self.assertEqual(cmd[1], str(ML_LIDAR_DIR / "export_onnx_rl.py"))
        self.assertEqual(cmd[cmd.index("--model") + 1], "ml_lidar/runs/v3/best_model.zip")
        self.assertEqual(cmd[cmd.index("--out") + 1], "models/e2e_lidar/my_model.onnx")
        self.assertNotIn("--max-speed", cmd)
        self.assertNotIn("--max-steer", cmd)

    def test_watch_cmd(self):
        cmd = build_watch_cmd("python3", "v3", panels=9)
        self.assertEqual(cmd[1], str(ML_LIDAR_DIR / "watch.py"))
        self.assertEqual(cmd[cmd.index("--model") + 1], "ml_lidar/runs/v3/best_model.zip")
        self.assertEqual(cmd[cmd.index("--panels") + 1], "9")

    def test_tensorboard_cmd(self):
        cmd = build_tensorboard_cmd("/x/.venv/bin/tensorboard", "v3")
        self.assertEqual(cmd[0], "/x/.venv/bin/tensorboard")
        self.assertEqual(cmd[cmd.index("--logdir") + 1], "ml_lidar/runs/v3/tb")


class TestConstants(unittest.TestCase):
    def test_runs_and_models_dir_are_under_expected_places(self):
        self.assertEqual(RUNS_DIR, ML_LIDAR_DIR / "runs")
        self.assertEqual(MODELS_DIR, REPO_ROOT / "models" / "e2e_lidar")


@unittest.skipUnless(_HAS_DISPLAY, "ディスプレイが無い環境（ヘッドレスCI等）ではスキップ")
class TestAppWidgetsSmoke(unittest.TestCase):
    """ウィジェットが例外無く組み立てられることだけを見る。クリック操作は試さない。"""

    def test_app_constructs_without_raising(self):
        root = tk.Tk()
        try:
            root.withdraw()
            app = App(root)
            self.assertIsNotNone(app.log_widget)
            self.assertEqual(app.status_label.cget("text"), "待機中")
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
