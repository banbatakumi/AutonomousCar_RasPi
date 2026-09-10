"""ml_lidar/run_info.py — `runs/<name>/`の中身だけを見て判定する軽量ヘルパー。

`pathlib`以外の依存を持たない。`ml_lidar/app.py`（Tkinter GUI、torch/stable-baselines3を
読み込まない薄い操作パネルという設計方針）と`ml_lidar/train_rl.py`（PPO学習本体）の
両方から使うため、重い依存を持つ`train_rl.py`側には置かない——`app.py`がそれを
importすると、サブプロセス起動だけで済むはずのGUIプロセスにまで学習依存一式が
読み込まれてしまう。
"""

from __future__ import annotations

from pathlib import Path

__all__ = ["has_existing_run"]


def has_existing_run(run_dir: Path) -> bool:
    """このrunに既に学習結果があるか。

    `train_rl.py`の`--resume`/`--overwrite`要否判定、`app.py`の続き学習/上書き
    確認ダイアログの要否判定の両方に使う。
    """
    if (run_dir / "final_model.zip").exists():
        return True
    if (run_dir / "eval" / "best_model.zip").exists():
        return True
    ckpt_dir = run_dir / "checkpoints"
    return ckpt_dir.is_dir() and any(ckpt_dir.glob("*.zip"))
