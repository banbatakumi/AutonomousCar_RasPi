"""ml_lidar/train_rl.py — PPO(Stable-Baselines3) で LiDAR E2E 方策を学習する。

    .venv/bin/python ml_lidar/train_rl.py --timesteps 2000000 --n-envs 8
    .venv/bin/tensorboard --logdir ml_lidar/runs/ppo_e2e/tb   # 学習曲線を見る

**毎エピソード`sim/random_course.py`で新しいコースを作る**（固定プールを使い回さない。
2026-08-28、有限プールへの過学習を避けるため変更——生成が軽い(数ms)ので、無限に
コースを作らせても学習速度への影響はほぼ無い）。`circuit`/`fuji`（手作りの既存
コース）は学習には一切使わず評価専用にする。定期的にこの2コースで評価して
`best_model.zip` を保存することで、「学習に使っていないコースにどれだけ汎化
できているか」を学習の外側から見張る。評価環境はLiDARノイズも固定値（既定の
`SimParams()`）にして、条件を揃えて比較できるようにしてある。

評価スコアが `--early-stop-patience` 回連続で更新されなければ、`--timesteps` に
達していなくても学習を打ち切る（既定で有効。`--early-stop-patience 0` で無効化）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # surge_mk2/

from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import (  # noqa: E402
    EvalCallback,
    StopTrainingOnNoModelImprovement,
)
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv  # noqa: E402

from ml_lidar.env import GymSurgeEnv  # noqa: E402
from sim.course import Course, DEFAULT_COURSE_DIR  # noqa: E402
from sim.random_course import generate_random_course  # noqa: E402

__all__ = ["make_train_env_fn", "make_eval_env"]


def make_train_env_fn(seed: int, *, max_steps: int, max_speed: float, max_steer: float):
    """ワーカープロセスの中で、**エピソードのたびに新しいランダムコースを作る**
    `GymSurgeEnv` を返す関数を作る。

    固定プールを使い回すと、有限個のコース形状に過学習するリスクがある
    （プールを増やせば緩和はできるが、根本的には解決しない）。生成は軽い(数ms)ので、
    毎回新しく作らせて実質無限のコース多様性にする。

    `course_fn=generate_random_course`をそのまま渡す。`SimE2EEnv`は`self.rng`
    （`reset(seed=...)`で正しく差し替わる、環境自身の乱数状態）を渡して呼ぶ設計に
    してあるので、外部に別のRNGを持たなくてよい（`generate_random_course(rng, ...)`
    の第一引数がそのままこの形）。

    LiDARノイズも既定で毎エピソードランダム化される（`SimE2EEnv`の`randomize_lidar`
    既定`True`）——ドメインランダム化で、シムの既定ノイズ量以外の条件にも頑健にする。
    """

    def _make():
        env = GymSurgeEnv(course_fn=generate_random_course, max_steps=max_steps,
                          max_speed=max_speed, max_steer=max_steer, seed=seed)
        return Monitor(env)   # エピソード報酬/長さの集計を正しく取るため

    return _make


def make_eval_env(*, max_steps: int, max_speed: float, max_steer: float,
                  seed: int = 0) -> GymSurgeEnv:
    """`circuit`/`fuji` ——学習に使っていない既知コースで評価する。

    `randomize_lidar=False`でLiDARノイズも既定値に固定する。学習側はノイズを
    ランダム化しているので、評価だけは条件を揃えないと「今回は運良く/悪くノイズが
    軽かった」で数字がぶれてしまう。
    """
    courses = [Course.load(DEFAULT_COURSE_DIR / "circuit.json"),
              Course.load(DEFAULT_COURSE_DIR / "fuji.json")]
    return GymSurgeEnv(courses, max_steps=max_steps, max_speed=max_speed,
                       max_steer=max_steer, seed=seed, randomize_lidar=False)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timesteps", type=int, default=2_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--max-steps", type=int, default=1500, help="1エピソードの最大ステップ数")
    ap.add_argument("--max-speed", type=float, default=1.5)
    ap.add_argument("--max-steer", type=float, default=0.45)
    ap.add_argument("--eval-freq", type=int, default=20_000,
                    help="このステップ数ごとに circuit/fuji で評価する（全ワーカー合計換算）")
    ap.add_argument("--n-eval-episodes", type=int, default=10)
    ap.add_argument("--early-stop-patience", type=int, default=10,
                    help="評価スコアがこの回数連続で更新されなかったら学習を打ち切る。"
                         "0で無効（--timesteps まで律儀に回り続ける）")
    ap.add_argument("--early-stop-min-evals", type=int, default=10,
                    help="打ち切りを許可するまでの最低評価回数。学習ごく初期の"
                         "ノイズだけで早まって打ち切らないための下駄")
    ap.add_argument("--out", type=Path, default=Path("ml_lidar/runs/ppo_e2e"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cpu",
                    help="MlpPolicyはネットワークが小さく、GPU転送のオーバーヘッドが"
                         "計算量を上回りやすいため既定はcpu（SB3のドキュメントの推奨と同じ）")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)

    env_fns = [make_train_env_fn(args.seed + i, max_steps=args.max_steps,
                                 max_speed=args.max_speed, max_steer=args.max_steer)
              for i in range(args.n_envs)]
    vec_cls = SubprocVecEnv if args.n_envs > 1 else DummyVecEnv
    vec_env = vec_cls(env_fns)

    model = PPO("MlpPolicy", vec_env, verbose=1, device=args.device, seed=args.seed,
               tensorboard_log=str(args.out / "tb"))

    eval_env = DummyVecEnv([lambda: Monitor(make_eval_env(
        max_steps=args.max_steps, max_speed=args.max_speed,
        max_steer=args.max_steer, seed=args.seed + 999))])
    # ★早期終了。評価スコアが`early_stop_patience`回連続で更新されなければ、
    # `--timesteps`に達していなくても学習を打ち切る（無駄な計算を続けない）。
    # `min_evals`で学習ごく初期のノイズだけで早まって打ち切らないようにしてある
    stop_callback = None
    if args.early_stop_patience > 0:
        stop_callback = StopTrainingOnNoModelImprovement(
            max_no_improvement_evals=args.early_stop_patience,
            min_evals=args.early_stop_min_evals, verbose=1)

    eval_callback = EvalCallback(
        eval_env, best_model_save_path=str(args.out), log_path=str(args.out),
        eval_freq=max(1, args.eval_freq // args.n_envs),
        n_eval_episodes=args.n_eval_episodes, deterministic=True,
        callback_after_eval=stop_callback)

    # 早期終了時は StopTrainingOnNoModelImprovement 自身が理由をログに出す（verbose=1）
    model.learn(total_timesteps=args.timesteps, callback=eval_callback)
    model.save(str(args.out / "last_model"))
    print(f"# 完了。最良モデル → {args.out}/best_model.zip 、最終モデル → "
          f"{args.out}/last_model.zip")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
