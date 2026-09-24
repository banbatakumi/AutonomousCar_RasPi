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
from sim.course import Course  # noqa: E402
from sim.raceline import compute_raceline_offsets, compute_speed_profile  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402

__all__ = ["find_env_config", "reference_lap", "run_episode", "evaluate", "parse_args", "main"]

#: 参照ラインを壁から離す余裕 [m]（`sim.raceline`の既定と同じ）
_RACELINE_SAFETY_MARGIN_M = 0.03

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


def reference_lap(course: Course, spec: VehicleSpec,
                  max_speed: float) -> tuple[float, float, float]:
    """このコース・この車両諸元で到達可能な
    `(理想ラップタイム[s], 理想ライン長[m], 理想ラインが要する舵の総量[rad/m])`。

    理想ライン＝最小曲率線（MCL、`sim.raceline.compute_raceline_offsets`）、速度は
    `compute_speed_profile`のグリップ＋加減速の3段アルゴリズム。**学習には一切
    関与しない評価専用の基準**で、`raceline_weight`（v18/v19で撤去済み）のように
    報酬へ戻すものではない（`ml_lidar/env.py`のdocstring「v21」参照）。

    :param spec: **そのエピソードで実際に使われた**`VehicleSpec`を渡すこと
        （`env.vehicle.spec`）。`randomize_dynamics=True`だと`mu`がエピソード
        ごとに振れるので、固定の`VehicleSpec.load()`を使うと理想タイムだけが
        実際のグリップとずれて比が歪む。
    """
    offsets = compute_raceline_offsets(
        course.centerline, course.width,
        vehicle_half_width_m=max(abs(p[1]) for p in spec.footprint),
        safety_margin_m=_RACELINE_SAFETY_MARGIN_M,
        # ★`course`は必須。`Course.width`が`None`のコース（`sim/editor.py`製の
        # 手描きコースはこれ）では、`_lateral_bounds()`が壁までのレイキャストに
        # これを使う。渡し忘れるとtoyota/course1のような観戦用コースで
        # ValueErrorになる（2026-09-22に実際に踏んだ）
        course=course,
        # 障害物のあるコースでは箱制約を障害物の無い側へ狭める（`sim.raceline`の
        # `_narrow_bounds_for_obstacles`）。★法線オフセットで表すので、障害物をかわす
        # 経路の表現力は2026-09-24に分かった「内側の弦を表せない」問題と同じ限界がある。
        # 障害物ありの`lap_ratio`は目安にとどめ、衝突・完走・停止の率を主に見ること
        obstacles=course.obstacles)
    v = compute_speed_profile(course.centerline, offsets, mu=spec.mu, max_speed=max_speed,
                              drive_accel_m_s2=spec.drive_accel_m_s2,
                              brake_decel_m_s2=spec.brake_decel_m_s2)
    yaw = course.centerline[:, 2]
    xy = course.centerline[:, :2] + offsets[:, None] * np.column_stack((-np.sin(yaw), np.cos(yaw)))
    closed = np.vstack([xy, xy[:1]])
    seg = np.hypot(*np.diff(closed, axis=0).T)
    v_mid = 0.5 * (v + np.roll(v, -1))          # 区間の代表速度（両端点の平均）
    length = float(np.sum(seg))

    # 理想ラインの曲率が要求する路面舵角 `atan(L*κ)` の総変化量を、走行距離で割る。
    # 「1m進むのに何rad舵を動かす必要があるか」——方策の実測値と同じ単位なので、
    # `steer_travel_ratio`として「どれだけ無駄に舵を動かしているか」が読める
    d1 = np.gradient(xy, axis=0)
    d2 = np.gradient(d1, axis=0)
    kappa = (d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum((d1 ** 2).sum(1) ** 1.5, 1e-12)
    steer = np.arctan(spec.wheelbase * kappa)
    travel = float(np.sum(np.abs(np.diff(np.r_[steer, steer[:1]]))))
    return (float(np.sum(seg / np.maximum(v_mid, 1e-3))), length,
            travel / max(length, 1e-6))


def run_episode(model: PPO, cfg: EnvConfig, *, seed: int, max_episode_steps: int) -> dict:
    """ランダムコース1本で1エピソード走らせ、結果をまとめる。

    ステア滑らかさ（`mean_abs_steer_diff`・`steer_sign_flip_rate`）は生アクション
    `action[0]`（-1..1、`steer_tau`フィルタ前）の隣接ステップ差分で見る。フィルタ後の
    `vehicle.steer_actual`だけ見ると、フィルタが振動を隠しているだけの状態を
    「滑らか」と誤判定しうる（v7検証、2026-09-08で実際に踏んだ落とし穴。
    `env.py`モジュールdocstring参照）。

    ## ラップタイムは`mean_speed`では代用できない（v21で追加、2026-09-17）

    `lap_time_s`とその理想比`lap_ratio`が**第一指標**。`mean_speed`は瞬間速度の
    単純平均なので、遠回りしても上がってしまいライン取りの評価に使えない。
    `lap_ratio`は`dist_ratio`（走行距離/理想ライン長）と`speed_ratio`（平均速度/
    理想平均速度）へ分解して返す——`lap_ratio ≈ dist_ratio / speed_ratio`なので、
    タイムをラインで失っているのか速度で失っているのかがそのまま読める
    （v17の実測は dist_ratio 0.98〜1.04・speed_ratio 0.80〜0.83 で、欠損はほぼ
    全部が速度だった。この分解が無かったためにv18〜v20はラインを3回続けて狙って
    しまった）。周回できなかったエピソードは`lap_time_s=None`で、平均からは外す。
    """
    env = LidarE2EEnv(cfg, seed=seed)
    env = TimeLimit(env, max_episode_steps=max_episode_steps)
    obs, _ = env.reset(seed=seed)

    total_reward = 0.0
    steps = 0
    collided = False
    steers: list[float] = [float(env.unwrapped._steer_target)]
    speeds: list[float] = []
    raw_steers: list[float] = []
    path: list[tuple[float, float]] = [(env.unwrapped.vehicle.x, env.unwrapped.vehicle.y)]
    terminated = truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        raw_steers.append(float(np.clip(action[0], -1.0, 1.0)))
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += float(reward)
        steps += 1
        speeds.append(float(info["speed"]))
        steers.append(float(env.unwrapped._steer_target))
        collided = collided or bool(info["collided"])
        path.append((env.unwrapped.vehicle.x, env.unwrapped.vehicle.y))
    lap_done = bool(terminated and not collided)
    course = env.unwrapped.course
    n_obstacles = 0 if course.obstacles is None else int(len(course.obstacles))
    width_m = float(course.width) if np.isscalar(course.width) else float(np.median(course.width))
    ideal_lap_s, ideal_len_m, ideal_travel_per_m = reference_lap(
        course, env.unwrapped.vehicle.spec, cfg.max_speed)
    env.close()

    xy = np.asarray(path, dtype=np.float64)
    path_len_m = float(np.hypot(*np.diff(xy, axis=0).T).sum()) if len(xy) > 1 else 0.0
    lap_time_s = steps * cfg.dt if lap_done else None

    # dtに依存しない舵の指標（`_steer_target`＝方策が積分で直接動かす目標舵角[rad]）。
    # `mean_abs_steer_diff`/`steer_sign_flip_rate`は「1ステップあたり」なので
    # dt=0.05とdt=0.10のrunを比較できない——実時間・実距離で測り直したのがこの3つ
    th = np.asarray(steers, dtype=np.float64)
    dth = np.diff(th) / cfg.dt if len(th) > 1 else np.array([])
    steer_travel_per_m = (float(np.sum(np.abs(np.diff(th))) / path_len_m)
                          if path_len_m > 1e-3 and len(th) > 1 else 0.0)
    steer_rate_mean = float(np.mean(np.abs(dth))) if dth.size else 0.0
    steer_reversals_per_s = (float(np.mean(np.diff(np.sign(dth)) != 0) / cfg.dt)
                             if dth.size > 1 else 0.0)

    diffs = np.diff(raw_steers) if len(raw_steers) > 1 else np.array([])
    signs = np.sign(raw_steers)
    sign_flips = np.diff(signs) != 0 if len(signs) > 1 else np.array([])

    return {
        "reward": total_reward,
        "steps": steps,
        "collided": collided,
        # 衝突も完走もせず打ち切り＝止まった（または極端に遅い）。障害物の前で
        # 「止まれば安全」に縮退していないかを見る（`obstacles.py`のdocstring参照）
        "stalled": (not collided) and (not lap_done),
        "n_obstacles": n_obstacles,
        "width_m": width_m,
        "lap_done": lap_done,
        "lap_time_s": lap_time_s,
        "ideal_lap_s": ideal_lap_s,
        "lap_ratio": (lap_time_s / ideal_lap_s) if lap_done and ideal_lap_s > 0 else None,
        "dist_ratio": (path_len_m / ideal_len_m) if lap_done and ideal_len_m > 0 else None,
        "speed_ratio": ((path_len_m / lap_time_s) / (ideal_len_m / ideal_lap_s))
                       if lap_done and lap_time_s and ideal_len_m > 0 else None,
        "mean_speed": float(np.mean(speeds)) if speeds else 0.0,
        "mean_abs_steer_diff": float(np.mean(np.abs(diffs))) if diffs.size else 0.0,
        "steer_sign_flip_rate": float(np.mean(sign_flips)) if sign_flips.size else 0.0,
        "steer_travel_per_m": steer_travel_per_m,
        "ideal_steer_travel_per_m": ideal_travel_per_m,
        "steer_travel_ratio": (steer_travel_per_m / ideal_travel_per_m
                               if ideal_travel_per_m > 1e-9 else None),
        "steer_rate_mean": steer_rate_mean,
        "steer_reversals_per_s": steer_reversals_per_s,
    }


def evaluate(model: PPO, cfg: EnvConfig, *, n_episodes: int, episode_s: float,
            base_seed: int) -> dict:
    max_episode_steps = max(1, int(round(episode_s / cfg.dt)))
    episodes = [run_episode(model, cfg, seed=base_seed + i, max_episode_steps=max_episode_steps)
               for i in range(n_episodes)]
    n = len(episodes)

    def _mean_of(key: str) -> float | None:
        """`None`（周回できなかったエピソード）を除いた平均。1本も無ければ`None`。"""
        vals = [e[key] for e in episodes if e[key] is not None]
        return float(np.mean(vals)) if vals else None

    return {
        "n_episodes": n,
        "collision_rate": sum(e["collided"] for e in episodes) / n,
        "lap_rate": sum(e["lap_done"] for e in episodes) / n,
        "stall_rate": sum(e["stalled"] for e in episodes) / n,
        "mean_lap_time_s": _mean_of("lap_time_s"),
        "mean_lap_ratio": _mean_of("lap_ratio"),
        "mean_dist_ratio": _mean_of("dist_ratio"),
        "mean_speed_ratio": _mean_of("speed_ratio"),
        "mean_reward": float(np.mean([e["reward"] for e in episodes])),
        "mean_steps": float(np.mean([e["steps"] for e in episodes])),
        "mean_speed": float(np.mean([e["mean_speed"] for e in episodes])),
        "mean_abs_steer_diff": float(np.mean([e["mean_abs_steer_diff"] for e in episodes])),
        "steer_sign_flip_rate": float(np.mean([e["steer_sign_flip_rate"] for e in episodes])),
        "steer_travel_per_m": float(np.mean([e["steer_travel_per_m"] for e in episodes])),
        "steer_travel_ratio": _mean_of("steer_travel_ratio"),
        "steer_rate_mean": float(np.mean([e["steer_rate_mean"] for e in episodes])),
        "steer_reversals_per_s": float(np.mean([e["steer_reversals_per_s"] for e in episodes])),
        "by_width": by_width(episodes),
        "by_obstacles": by_obstacles(episodes),
        "episodes": episodes,
    }


#: `by_width()`の道幅バケット境界 [m]。v21以前の学習レンジ上限1.4mを境目に置いてある
#: ——**v22で道幅レンジを0.8〜1.4→0.8〜2.5mへ広げた**ので、「旧レンジ内（狭い）で
#: 退行していないか」と「新しく入れた広いコースで改善しているか」を分けて読む必要がある
#: （`ml_lidar/course_gen.py`の`_WIDTH_RANGE_M`参照）
_WIDTH_BUCKETS_M: tuple[float, ...] = (1.4, 2.0)


def by_width(episodes: list[dict]) -> list[dict]:
    """道幅バケットごとの衝突率・ラップ比。集計全体では狭いコースの退行が埋もれるため。"""
    edges = (0.0,) + _WIDTH_BUCKETS_M + (float("inf"),)
    out = []
    for lo, hi in zip(edges, edges[1:]):
        bucket = [e for e in episodes if lo <= e["width_m"] < hi]
        if not bucket:
            continue
        ratios = [e["lap_ratio"] for e in bucket if e["lap_ratio"] is not None]
        out.append({
            "width_lo": lo,
            "width_hi": hi,
            "n": len(bucket),
            "collision_rate": sum(e["collided"] for e in bucket) / len(bucket),
            "lap_rate": sum(e["lap_done"] for e in bucket) / len(bucket),
            "mean_lap_ratio": float(np.mean(ratios)) if ratios else None,
            "mean_speed": float(np.mean([e["mean_speed"] for e in bucket])),
        })
    return out


def by_obstacles(episodes: list[dict]) -> list[dict]:
    """障害物の個数ごとの衝突・完走・停止の率。0個のバケットで「障害物に備えて
    常に遅くなっていないか」を、1個以上で「避けられているか」を読む。"""
    out = []
    for k in sorted({e["n_obstacles"] for e in episodes}):
        bucket = [e for e in episodes if e["n_obstacles"] == k]
        ratios = [e["lap_ratio"] for e in bucket if e["lap_ratio"] is not None]
        out.append({
            "n_obstacles": k,
            "n": len(bucket),
            "collision_rate": sum(e["collided"] for e in bucket) / len(bucket),
            "lap_rate": sum(e["lap_done"] for e in bucket) / len(bucket),
            "stall_rate": sum(e["stalled"] for e in bucket) / len(bucket),
            "mean_lap_ratio": float(np.mean(ratios)) if ratios else None,
            "mean_speed": float(np.mean([e["mean_speed"] for e in bucket])),
        })
    return out


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
    # 障害物はモデルの観測・行動契約ではなく評価条件なので、env_config.jsonより優先して
    # 上書きできる（障害物を見たことのない旧モデルを障害物コースで測るため）
    p.add_argument("--obstacle-prob", type=float, default=None,
                   help="障害物を置くコースの割合（省略時はenv_config.jsonの値、旧runは0）")
    p.add_argument("--max-obstacles", type=int, default=None,
                   help="1コースあたりの障害物の上限（省略時はenv_config.jsonの値）")
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
    if args.obstacle_prob is not None:
        cfg = dataclasses.replace(cfg, obstacle_prob=args.obstacle_prob)
    if args.max_obstacles is not None:
        cfg = dataclasses.replace(cfg, max_obstacles=args.max_obstacles)

    model = PPO.load(str(args.model), device="cpu")
    result = evaluate(model, cfg, n_episodes=args.episodes, episode_s=args.episode_s,
                      base_seed=args.seed)

    def _fmt(key: str, spec: str) -> str:
        v = result[key]
        return "n/a" if v is None else format(v, spec)

    print(f"episodes={result['n_episodes']} "
         f"collision_rate={result['collision_rate']:.1%} "
         f"lap_rate={result['lap_rate']:.1%} "
         f"stall_rate={result['stall_rate']:.1%}")
    # ★第一指標。lap_ratio ≈ dist_ratio / speed_ratio に分解して、タイムを
    # 「ライン（遠回り）」で失っているのか「速度」で失っているのかを読む
    print(f"  lap_time={_fmt('mean_lap_time_s', '.2f')}s "
         f"lap_ratio={_fmt('mean_lap_ratio', '.3f')}x(対MCL理想) "
         f"= dist_ratio {_fmt('mean_dist_ratio', '.3f')} / "
         f"speed_ratio {_fmt('mean_speed_ratio', '.3f')}")
    print(f"  mean_reward={result['mean_reward']:.2f} "
         f"mean_speed={result['mean_speed']:.2f}m/s")
    # ★舵の滑らかさはここを見る。dtに依存しない実距離・実時間ベースの指標
    print(f"  舵: 総量={result['steer_travel_per_m']:.3f} rad/m "
         f"(理想比 {_fmt('steer_travel_ratio', '.2f')}x) "
         f"|dθ/dt|={result['steer_rate_mean']:.3f} rad/s "
         f"切返し={result['steer_reversals_per_s']:.2f} 回/s")
    # 道幅別（v22で学習レンジを広げたので、狭いコースの退行が全体平均に埋もれないよう分けて出す）
    for b in result["by_width"]:
        hi = "∞" if b["width_hi"] == float("inf") else f"{b['width_hi']:.1f}"
        lap = "n/a" if b["mean_lap_ratio"] is None else f"{b['mean_lap_ratio']:.3f}"
        print(f"  道幅{b['width_lo']:.1f}〜{hi}m (n={b['n']:3d}): "
             f"collision={b['collision_rate']:5.1%} lap_ratio={lap} speed={b['mean_speed']:.2f}")

    if len(result["by_obstacles"]) > 1 or result["by_obstacles"][0]["n_obstacles"] > 0:
        for b in result["by_obstacles"]:
            lap = "n/a" if b["mean_lap_ratio"] is None else f"{b['mean_lap_ratio']:.3f}"
            print(f"  障害物{b['n_obstacles']}個 (n={b['n']:3d}): "
                 f"collision={b['collision_rate']:5.1%} stall={b['stall_rate']:5.1%} "
                 f"lap_ratio={lap} speed={b['mean_speed']:.2f}")

    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"→ {args.out}")


if __name__ == "__main__":
    main()
