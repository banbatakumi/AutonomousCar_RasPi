"""`ml_lidar/train_rl.py`の`--resume-from`のCLI統合テスト（サブプロセス経由）。

実際にPPOを短時間学習させて配線を確認する重いテストなので、既存の軽量な単体
テスト群とは別ファイルに分けてある。設定は可能な限り小さくしてある
（`--n-envs 1`・`--max-steps`短め・`--n-eval-episodes 1`等）が、それでも
1回あたり数秒〜十数秒かかる（PPOの既定`n_steps=2048`ぶんのロールアウトが
最低1回は走るため、`--timesteps`をどれだけ小さくしても短縮しきれない）。
"""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()
