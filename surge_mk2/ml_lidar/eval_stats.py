"""ml_lidar/eval_stats.py — 複数シード・複数コースアーキタイプでの統計評価。

    .venv/bin/python -m ml_lidar.eval_stats --model ml_lidar/runs/v1/eval/best_model.zip

`train_rl.py`のEvalCallback（4固定コース、resume用のbest_model選定用）とは別物。Copilot CLIの
独立レビュー「単一シード・単一コースの成功率は信頼できない」を受け、学習ループの
外側で、コース形状パラメータ自体を都度ランダムに振った独立コースで統計を取る
（学習中に出た/出なかったコースの偏りを引きずらない）。

モデルの観測・行動契約（`fov_deg`/`max_range`/`max_speed`/`steer_tau`。最大舵角は
`config/vehicle.toml`固定なのでここには含まれない）は`train_rl.py`が
`<run_dir>/env_config.json`に書き出したものを`--model`から遡って自動で読む。
見つからなければCLI引数の既定値（`train_rl.py`と同じ）にフォールバックする。

## `--episodes`は最低でも既定値(300)を使うこと。60では判定を誤る

v1/v2の比較検証（2026-09-07）で、`--episodes 60`程度（当初の既定値）だと結論そのものが
シード依存でひっくり返ることを実測した——`--seed 1000`でn=100を評価するとv2の衝突率が
0%（v1は3%）で「v2の圧勝」に見えたが、`--seed 5000`でn=150を追加すると衝突率がv1=0%・
v2=5.3%と逆転し、さらに`--seed 9000`でn=300を足して3系統合計n=550まで積み増して初めて
v1=2.2%・v2=5.5%で安定した。二項比率の標準誤差`√(p(1-p)/n)`がn=100程度では数%の桁で
効いてくるので、比較のたびに`--episodes`を小さく削らないこと。**1回の実行で結論を出さず、
疑わしい場合は`--seed`を変えて複数回走らせ、`episodes`をこのファイルのdocstringのように
合算してから判断する**（単一seed系統は、真の性能ではなくその系統がたまたま引いたコース
分布に依存した数値になりうる）。
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
from pathlib import Path

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402
from gymnasium.wrappers import TimeLimit  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402

__all__ = ["find_env_config", "run_episode", "evaluate", "parse_args", "main"]

_ENV_CONFIG_SEARCH_LEVELS = 3


def find_env_config(model_path: Path) -> EnvConfig | None:
    """`model_path`から`env_config.json`を遡って探す（`train_rl.py`が書き出したもの）。

    `--model`には`<run_dir>/eval/best_model.zip`・`<run_dir>/checkpoints/ppo_N_steps.zip`・
    `<run_dir>/final_model.zip`のいずれも渡しうるので、階層をいくつか遡って探す。
    """
    field_names = {f.name for f in dataclasses.fields(EnvConfig)}
    d = model_path.resolve().parent
    for _ in range(_ENV_CONFIG_SEARCH_LEVELS):
        cand = d / "env_config.json"
        if cand.exists():
            data = json.loads(cand.read_text(encoding="utf-8"))
            return EnvConfig(**{k: v for k, v in data.items() if k in field_names})
        d = d.parent
    return None


def run_episode(model: PPO, cfg: EnvConfig, *, seed: int, max_episode_steps: int) -> dict:
    """ランダムコース1本で1エピソード走らせ、結果をまとめる。

    ステア滑らかさ（`mean_abs_steer_diff`・`steer_sign_flip_rate`）は生アクション
    `action[0]`（-1..1、`steer_tau`フィルタ前）の隣接ステップ差分で見る。フィルタ後の
    `vehicle.steer_actual`だけ見ると、フィルタが振動を隠しているだけの状態を
    「滑らか」と誤判定しうる（v7検証、2026-09-08で実際に踏んだ落とし穴。
    `env.py`モジュールdocstring参照）。
    """
    env = LidarE2EEnv(cfg, seed=seed)
    env = TimeLimit(env, max_episode_steps=max_episode_steps)
    obs, _ = env.reset(seed=seed)

    total_reward = 0.0
    steps = 0
    collided = False
    speeds: list[float] = []
    raw_steers: list[float] = []
    terminated = truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        raw_steers.append(float(np.clip(action[0], -1.0, 1.0)))
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += float(reward)
        steps += 1
        speeds.append(float(info["speed"]))
        collided = collided or bool(info["collided"])
    env.close()

    diffs = np.diff(raw_steers) if len(raw_steers) > 1 else np.array([])
    signs = np.sign(raw_steers)
    sign_flips = np.diff(signs) != 0 if len(signs) > 1 else np.array([])

    return {
        "reward": total_reward,
        "steps": steps,
        "collided": collided,
        "lap_done": bool(terminated and not collided),
        "mean_speed": float(np.mean(speeds)) if speeds else 0.0,
        "mean_abs_steer_diff": float(np.mean(np.abs(diffs))) if diffs.size else 0.0,
        "steer_sign_flip_rate": float(np.mean(sign_flips)) if sign_flips.size else 0.0,
    }


def evaluate(model: PPO, cfg: EnvConfig, *, n_episodes: int, episode_s: float,
            base_seed: int) -> dict:
    max_episode_steps = max(1, int(round(episode_s / cfg.dt)))
    episodes = [run_episode(model, cfg, seed=base_seed + i, max_episode_steps=max_episode_steps)
               for i in range(n_episodes)]
    n = len(episodes)
    return {
        "n_episodes": n,
        "collision_rate": sum(e["collided"] for e in episodes) / n,
        "lap_rate": sum(e["lap_done"] for e in episodes) / n,
        "mean_reward": float(np.mean([e["reward"] for e in episodes])),
        "mean_steps": float(np.mean([e["steps"] for e in episodes])),
        "mean_speed": float(np.mean([e["mean_speed"] for e in episodes])),
        "mean_abs_steer_diff": float(np.mean([e["mean_abs_steer_diff"] for e in episodes])),
        "steer_sign_flip_rate": float(np.mean([e["steer_sign_flip_rate"] for e in episodes])),
        "episodes": episodes,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ml_lidar 複数シード・複数コースの統計評価")
    p.add_argument("--model", required=True, type=Path)
    p.add_argument("--episodes", type=int, default=300,
                   help="ランダムコース群への総エピソード数。モジュールdocstring参照——"
                        "60程度では衝突率のようなまれな事象の推定が安定しない")
    p.add_argument("--episode-s", type=float, default=30.0)
    p.add_argument("--seed", type=int, default=1000,
                   help="学習・train_rl.pyのeval用コースとは別系統の基準シード")
    p.add_argument("--out", type=Path, default=None, help="結果JSONの保存先（省略可）")
    # env_config.jsonが見つからないときのフォールバック既定値（train_rl.pyと同じ）
    p.add_argument("--fov-deg", type=float, default=270.0)
    p.add_argument("--max-range", type=float, default=10.0)
    p.add_argument("--max-speed", type=float, default=2.0)
    p.add_argument("--steer-tau", type=float, default=0.10)
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    cfg = find_env_config(args.model)
    if cfg is None:
        print(f"!! {args.model} の近くにenv_config.jsonが見つからない。CLI既定値にフォールバックする",
             file=sys.stderr)
        cfg = EnvConfig(fov_deg=args.fov_deg, max_range=args.max_range,
                        max_speed=args.max_speed, steer_tau=args.steer_tau)
    # 統計評価は学習の汎化そのものを見るためのものなので、ランダム化は常にONにする
    # （固定・ノイズ無効のtrain_rl.py evalとは目的が違う）
    cfg = dataclasses.replace(cfg, randomize_lidar=True, randomize_dynamics=True)

    model = PPO.load(str(args.model), device="cpu")
    result = evaluate(model, cfg, n_episodes=args.episodes, episode_s=args.episode_s,
                      base_seed=args.seed)

    print(f"episodes={result['n_episodes']} "
         f"collision_rate={result['collision_rate']:.1%} "
         f"lap_rate={result['lap_rate']:.1%} "
         f"mean_reward={result['mean_reward']:.2f} "
         f"mean_speed={result['mean_speed']:.2f}m/s "
         f"mean_abs_steer_diff={result['mean_abs_steer_diff']:.3f} "
         f"steer_sign_flip_rate={result['steer_sign_flip_rate']:.1%}")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"→ {args.out}")


if __name__ == "__main__":
    main()
