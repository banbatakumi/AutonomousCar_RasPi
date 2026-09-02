"""`ml_lidar/train_rl.py`の`--resume-from`のCLI統合テスト（サブプロセス経由）。

実際にPPOを短時間学習させて配線を確認する重いテストなので、既存の軽量な単体
テスト群とは別ファイルに分けてある。設定は可能な限り小さくしてある
（`--n-envs 1`・`--max-steps`短め・`--n-eval-episodes 1`等）が、それでも
1回あたり数秒〜十数秒かかる（PPOの既定`n_steps=2048`ぶんのロールアウトが
最低1回は走るため、`--timesteps`をどれだけ小さくしても短縮しきれない）。
"""

import json
import subprocess
import sys
import tempfile
import unittest
import unittest.mock
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # surge_mk2/

from ml_lidar.train_rl import CurriculumCallback  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
TRAIN_RL = REPO_ROOT / "ml_lidar" / "train_rl.py"


def _run_train(*extra_args: str, out: Path) -> subprocess.CompletedProcess:
    cmd = [sys.executable, str(TRAIN_RL), "--n-envs", "1", "--max-steps", "50",
          "--eval-freq", "200", "--n-eval-episodes", "1", "--early-stop-patience", "0",
          "--out", str(out), *extra_args]
    return subprocess.run(cmd, cwd=str(REPO_ROOT), capture_output=True, text=True, timeout=120)


def _tb_run_dirs(out: Path) -> list[Path]:
    """SB3が`tensorboard_log`配下に作る`PPO_1`・`PPO_2`…のサブフォルダ一覧。"""
    tb = out / "tb"
    if not tb.exists():
        return []
    return sorted(p for p in tb.iterdir() if p.is_dir())


class TestTrainRlResume(unittest.TestCase):
    def test_fresh_run_then_resume_continues_training(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "v1"

            fresh = _run_train("--timesteps", "200", out=out)
            self.assertEqual(fresh.returncode, 0, fresh.stderr)
            best = out / "best_model.zip"
            self.assertTrue(best.exists())
            tb_before = _tb_run_dirs(out)
            self.assertEqual(len(tb_before), 1)

            resumed = _run_train("--timesteps", "2200", "--resume-from", str(best), out=out)
            self.assertEqual(resumed.returncode, 0, resumed.stderr)
            self.assertIn("から再開", resumed.stdout)
            self.assertTrue((out / "last_model.zip").exists())
            # ★再開(reset_num_timesteps=False)は同じTensorBoardサブフォルダを使い続ける
            # べきで、新しく PPO_2 を増やしてはいけない
            tb_after = _tb_run_dirs(out)
            self.assertEqual(len(tb_after), 1)
            self.assertEqual(tb_before[0].name, tb_after[0].name)

    def test_resume_from_missing_checkpoint_fails_clearly(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "v1"
            missing = Path(d) / "nope.zip"
            result = _run_train("--timesteps", "200", "--resume-from", str(missing), out=out)
            self.assertNotEqual(result.returncode, 0)


class TestTrainRlRewardNorm(unittest.TestCase):
    """`--no-reward-norm`（2026-09-02追加、v10の性能低下診断でVecNormalizeが
    疑わしい候補に挙がったための切り分け用フラグ）。無効化しても学習が完走し、
    `vecnormalize.pkl`を書かないことを確認する。"""

    def test_no_reward_norm_completes_without_vecnormalize_pkl(self):
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "v1"
            result = _run_train("--timesteps", "200", "--no-reward-norm", out=out)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse((out / "vecnormalize.pkl").exists())
            self.assertTrue((out / "last_model.zip").exists())
            cfg = json.loads((out / "run_config.json").read_text())
            self.assertFalse(cfg["reward_norm"])


class TestTrainRlOverwriteClearsOutDir(unittest.TestCase):
    def test_rerunning_fresh_with_same_out_does_not_accumulate_tb_runs(self):
        """`--resume-from`無しの再学習は`--out`を一度空にする（2026-08-28追加）。
        バンビが実際に踏んだ不具合: 同じrun名で新規学習を繰り返すと、SB3が
        TensorBoardログを`PPO_1`・`PPO_2`…と勝手に増やし続けてしまっていた。
        """
        with tempfile.TemporaryDirectory() as d:
            out = Path(d) / "v1"

            r1 = _run_train("--timesteps", "200", out=out)
            self.assertEqual(r1.returncode, 0, r1.stderr)
            self.assertEqual(len(_tb_run_dirs(out)), 1)

            r2 = _run_train("--timesteps", "200", out=out)
            self.assertEqual(r2.returncode, 0, r2.stderr)
            self.assertEqual(len(_tb_run_dirs(out)), 1,
                             "新規学習を繰り返すとTensorBoardのサブフォルダが積み上がってはいけない")


class TestCurriculumCallback(unittest.TestCase):
    """`CurriculumCallback`（2026-09-02追加）の進捗計算。実際のPPO学習は回さず、
    `num_timesteps`を直接差し込んで`_progress()`の式だけを検証する
    （軽量な単体テスト。CLI統合テストは重いので上の`TestTrainRlResume`等と分離）。
    """

    def test_progress_ramps_linearly_then_clamps_to_one(self):
        cb = CurriculumCallback(curriculum_frac=0.5, total_timesteps=1000)
        cb.num_timesteps = 0
        self.assertEqual(cb._progress(), 0.0)
        cb.num_timesteps = 250
        self.assertAlmostEqual(cb._progress(), 0.5)
        cb.num_timesteps = 500
        self.assertAlmostEqual(cb._progress(), 1.0)
        cb.num_timesteps = 900   # 500(=frac*total)を超えても1.0で頭打ち
        self.assertAlmostEqual(cb._progress(), 1.0)

    def test_curriculum_frac_zero_or_negative_disables_ramp(self):
        for frac in (0.0, -1.0):
            with self.subTest(curriculum_frac=frac):
                cb = CurriculumCallback(curriculum_frac=frac, total_timesteps=1000)
                cb.num_timesteps = 0
                self.assertEqual(cb._progress(), 1.0)

    def test_push_calls_env_method_with_current_progress(self):
        """`training_env`はBaseCallbackの読み取り専用property（`self.model.get_env()`
        経由）なので、`self.model`をモックして間接的に差し込む。"""
        cb = CurriculumCallback(curriculum_frac=0.5, total_timesteps=1000)
        cb.num_timesteps = 250
        mock_env = unittest.mock.Mock()
        cb.model = unittest.mock.Mock(get_env=unittest.mock.Mock(return_value=mock_env))
        cb._push()
        mock_env.env_method.assert_called_once_with("set_curriculum_progress", 0.5)
        cb.model.logger.record.assert_called_once_with("curriculum/progress", 0.5)


if __name__ == "__main__":
    unittest.main()
