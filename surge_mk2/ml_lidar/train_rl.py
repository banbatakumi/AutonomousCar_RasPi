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

学習コースの道幅も毎エピソード0.7〜1.3mでランダム化する（`generate_random_course_dr`、
2026-08-28。道幅1.0m固定に過学習し大会コースの狭い/広い区間に汎化できないおそれが
あったため）。**評価用の`circuit`/`fuji`は幅1.0m固定のまま変更していない**——比較の
条件を揃えるためで、評価スコアの改善だけでは道幅への汎化は測れない点に注意。

評価スコアが `--early-stop-patience` 回連続で更新されなければ、`--timesteps` に
達していなくても学習を打ち切る（既定で有効。`--early-stop-patience 0` で無効化）。

**`--resume-from <チェックポイント.zip>`で途中から再開できる**（2026-08-28追加）。
`PPO.load()`で読み込み、`reset_num_timesteps=False`で続きから数える。`--timesteps`は
**累計の目標値**（再開後に追加でNステップ学習したいなら「読み込んだ時点の
`num_timesteps` + N」を指定すること）。指定しなければ従来通りゼロから新規に作る。

**★`--resume-from`を指定しない（＝新規学習）ときは、`--out`が既存でも中身を
一度空にしてから作り直す**（2026-08-28追加）。SB3はTensorBoardログを
`tensorboard_log`で指定すると、`reset_num_timesteps=True`のたびに
`{アルゴリズム名}_{N+1}`という新しいサブフォルダを増やし続ける仕様のため、
同じ`--out`（＝同じrun名）で新規学習を繰り返すと、TensorBoardに`PPO_1`・`PPO_2`・
`PPO_3`…と過去の（もう存在しない）モデルの学習曲線がゴミとして積み上がってしまう
（バンビが実際に踏んだ）。`--out`を「新規学習ならその名前の中身を空にする」という
約束にすることで解決する。
"""

from __future__ import annotations

import os

# ★BLAS(Accelerate/OpenBLAS)がプロセス内で勝手にマルチスレッド化すると、
# `--n-envs`個のワーカープロセス同士でCPUコアを取り合ってしまい、かえって
# 遅くなる（2026-09-01、M3 MacBook AirでCPU使用率が30%程度にしか上がらない
# 問題の調査で判明。numpyをimportする前に設定する必要があるので最上部に置く）
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("VECLIB_MAXIMUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("NUMEXPR_NUM_THREADS", "1")

import argparse
import json
import shutil
import sys
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # surge_mk2/

import torch  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import (  # noqa: E402
    BaseCallback,
    EvalCallback,
    StopTrainingOnNoModelImprovement,
)
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv  # noqa: E402

from ml_lidar.env import GymSurgeEnv  # noqa: E402
from sim.course import Course, DEFAULT_COURSE_DIR  # noqa: E402
from sim.random_course import generate_diverse_course  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402

__all__ = ["make_train_env_fn", "make_eval_env", "linear_schedule", "RacelineMetricsCallback"]


class RacelineMetricsCallback(BaseCallback):
    """理想ラインからの平均横偏差（`sim/gym_env.py`の`info["raceline_cross"]`）を
    TensorBoardに記録する（2026-09-01追加）。実車確認の前に、シムだけで
    「アペックスを突けているか」の定量的な進捗を学習曲線として追えるようにする狙い
    （`raceline_weight`/`speed_match_weight`導入の効果測定用）。`record_mean`は
    SB3のLoggerが次のdumpまでの値を自動平均する機能——毎ステップ全ワーカーぶん
    呼んでも、TensorBoard上は1ロールアウトあたり1点にまとまる。"""

    def _on_step(self) -> bool:
        for info in self.locals["infos"]:
            if "raceline_cross" in info:
                self.logger.record_mean("raceline/mean_cross_dev", info["raceline_cross"])
        return True


def linear_schedule(initial_value: float) -> Callable[[float], float]:
    """SB3の`learning_rate`が受け付ける形（`progress_remaining`は1.0→0.0で減る）。

    学習後半ほど1回の更新を小さくして、方策が大きくズレて崩れるのを防ぐ
    （2026-08-29、v2で2M step付近をピークに3M stepまで評価スコアが下がり続ける
    「方策崩壊」が起きたことを受けて追加。定率の学習率のままだと、方策がある程度
    良くなった後も同じ大きさの更新を続けてしまい、良い状態から押し出されやすい）。
    """

    def _schedule(progress_remaining: float) -> float:
        return progress_remaining * initial_value

    return _schedule


def make_train_env_fn(seed: int, *, max_steps: int, max_speed: float,
                      steer_tau: float, steer_rate_weight: float, speed_weight: float,
                      slip_weight: float, raceline_weight: float, raceline_tolerance_m: float,
                      speed_match_weight: float):
    """ワーカープロセスの中で、**エピソードのたびに新しいランダムコースを作る**
    `GymSurgeEnv` を返す関数を作る。

    固定プールを使い回すと、有限個のコース形状に過学習するリスクがある
    （プールを増やせば緩和はできるが、根本的には解決しない）。生成は軽い(数ms)ので、
    毎回新しく作らせて実質無限のコース多様性にする。

    `course_fn=generate_diverse_course`をそのまま渡す。`SimE2EEnv`は`self.rng`
    （`reset(seed=...)`で正しく差し替わる、環境自身の乱数状態）を渡して呼ぶ設計に
    してあるので、外部に別のRNGを持たなくてよい（`generate_diverse_course(rng, ...)`
    の第一引数がそのままこの形）。`generate_diverse_course`は毎エピソード
    organic/circuit/corridor/narrow/obstacleのアーキタイプを重み付きランダムで
    選ぶ（2026-08-31、単一の生成器だけだと似た形のコースばかりになるという
    バンビの指摘への対応。詳細は`sim/random_course.py`モジュールdocstring
    「アーキタイプの多様化」参照）。旧`generate_random_course_dr`（organic1種類）
    はそのまま`sim/random_course.py`に残っており、`generate_diverse_course`内部の
    organicアーキタイプが呼んでいる。

    LiDARノイズも既定で毎エピソードランダム化される（`SimE2EEnv`の`randomize_lidar`
    既定`True`）——ドメインランダム化で、シムの既定ノイズ量以外の条件にも頑健にする。
    道幅も`generate_diverse_course`が毎エピソード0.7〜1.3mでランダム化する
    （2026-08-28、幅1.0m固定への過学習を避けるため。詳細は`sim/random_course.py`）。
    """

    def _make():
        env = GymSurgeEnv(course_fn=generate_diverse_course, max_steps=max_steps,
                          max_speed=max_speed, seed=seed, steer_tau=steer_tau,
                          steer_rate_weight=steer_rate_weight, speed_weight=speed_weight,
                          slip_weight=slip_weight, raceline_weight=raceline_weight,
                          raceline_tolerance_m=raceline_tolerance_m,
                          speed_match_weight=speed_match_weight)
        return Monitor(env)   # エピソード報酬/長さの集計を正しく取るため

    return _make


def make_eval_env(*, max_steps: int, max_speed: float, steer_tau: float,
                  steer_rate_weight: float, speed_weight: float, slip_weight: float,
                  raceline_weight: float, raceline_tolerance_m: float,
                  speed_match_weight: float, seed: int = 0) -> GymSurgeEnv:
    """`circuit`/`fuji` ——学習に使っていない既知コースで評価する。

    `randomize_lidar=False`でLiDARノイズも既定値に固定する。学習側はノイズを
    ランダム化しているので、評価だけは条件を揃えないと「今回は運良く/悪くノイズが
    軽かった」で数字がぶれてしまう。`randomize_dynamics=False`も同じ理由
    （2026-08-31追加、`[dynamics]`未実測パラメータのドメインランダム化とセット）。
    `steer_tau`/`steer_rate_weight`/`speed_weight`/`slip_weight`/
    `raceline_weight`/`raceline_tolerance_m`/`speed_match_weight`は
    学習側と揃える（実運用の滑らかさ・速度の攻め方をそのまま評価スコア・
    `best_model`選定に反映させるため）。
    """
    courses = [Course.load(DEFAULT_COURSE_DIR / "circuit.json"),
              Course.load(DEFAULT_COURSE_DIR / "fuji.json")]
    return GymSurgeEnv(courses, max_steps=max_steps, max_speed=max_speed,
                       seed=seed, randomize_lidar=False, randomize_dynamics=False,
                       steer_tau=steer_tau, steer_rate_weight=steer_rate_weight,
                       speed_weight=speed_weight, slip_weight=slip_weight,
                       raceline_weight=raceline_weight, raceline_tolerance_m=raceline_tolerance_m,
                       speed_match_weight=speed_match_weight)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timesteps", type=int, default=2_000_000)
    ap.add_argument("--n-envs", type=int, default=8)
    ap.add_argument("--max-steps", type=int, default=1500, help="1エピソードの最大ステップ数")
    ap.add_argument("--max-speed", type=float, default=1.5)
    ap.add_argument("--eval-freq", type=int, default=20_000,
                    help="このステップ数ごとに circuit/fuji で評価する（全ワーカー合計換算）")
    ap.add_argument("--n-eval-episodes", type=int, default=30,
                    help="評価1回あたりのエピソード数。少ないとエピソード間のばらつき"
                         "（ランダムな開始位置・コース内の局所地形）に埋もれて"
                         "`best_model`の選定がノイズの山を拾うだけになる"
                         "（2026-08-29、v2の診断で発覚。旧既定は10）")
    ap.add_argument("--early-stop-patience", type=int, default=10,
                    help="評価スコアがこの回数連続で更新されなかったら学習を打ち切る。"
                         "0で無効（--timesteps まで律儀に回り続ける）")
    ap.add_argument("--early-stop-min-evals", type=int, default=10,
                    help="打ち切りを許可するまでの最低評価回数。学習ごく初期の"
                         "ノイズだけで早まって打ち切らないための下駄")
    ap.add_argument("--target-kl", type=float, default=0.03,
                    help="1回の方策更新で許容するKLダイバージェンスの上限。超えたら"
                         "そのエポックの残りを打ち切る（SB3のPPOの機能）。0以下で無効。"
                         "★2026-08-29追加: v2の学習曲線が2M step付近でピークに達した後"
                         "3M stepまで下がり続ける「方策崩壊」を起こしていたのを受けて、"
                         "更新1回の破壊力に上限を設ける")
    ap.add_argument("--n-epochs", type=int, default=4,
                    help="1回のロールアウトを何エポック再利用して勾配更新するか"
                         "（SB3既定は10）。同じデータを回しすぎると方策が一気にズレて"
                         "不安定化しやすいため既定を下げてある（2026-08-29、方策崩壊対策）")
    ap.add_argument("--learning-rate", type=float, default=3e-4,
                    help="学習率の初期値。学習の進み具合（`--timesteps`に対する残り割合）"
                         "に比例して線形に0まで下げる（`linear_schedule()`）。学習後半の"
                         "更新を小さくして収束を安定させる（2026-08-29、方策崩壊対策）")
    ap.add_argument("--steer-tau", type=float, default=0.10,
                    help="舵指令に掛ける一次遅れの時定数[s]。`raspi/auto/e2e_lidar.py`の"
                         "`steer_tau`ParamSpec既定と同じ値を学習側にも適用し、推論時にだけ"
                         "付いていた平滑化フィルタとのズレを無くす（2026-08-29追加）")
    ap.add_argument("--steer-rate-weight", type=float, default=0.2,
                    help="平滑化後の舵角の1ステップ変化量に掛ける罰則の重み。"
                         "方策自身に「滑らかな操舵の方が得」という圧力を与える"
                         "（2026-08-29追加。`sim/gym_env.py`の`SimE2EEnv`docstring参照）")
    ap.add_argument("--speed-weight", type=float, default=0.1,
                    help="毎ステップ`speed/max_speed`に掛けて加算する速度ボーナス。"
                         "`progress`（弧長方向の移動量）だけでは`collision_penalty=-5.0`"
                         "に対して速度の効きが弱く、方策が速度・ライン取りに消極的に"
                         "なりやすかった（v5評価でバンビが指摘、2026-08-30追加）。"
                         "2026-09-01: 曲率を考慮しない一律ボーナスがコーナー前の減速を"
                         "妨げていた疑いがあり、既定値を0.3→0.1に下げ主役を"
                         "--speed-match-weightに譲った——`sim/gym_env.py`の"
                         "`SimE2EEnv`docstring参照")
    ap.add_argument("--slip-weight", type=float, default=0.2,
                    help="要求向心加速度がグリップ限界(mu*g)を超えた比率"
                         "（`VehicleModel.slip_frac`）に掛ける罰則の重み。滑走・"
                         "グリップ限界超過そのものを直接罰する（2026-08-31追加、"
                         "バンビの「高速旋回で滑る設計か」という指摘への対応）。"
                         "初期値0.2は未検証——`sim/gym_env.py`の`SimE2EEnv`docstring参照")
    ap.add_argument("--raceline-weight", type=float, default=0.3,
                    help="`sim/raceline.py`が道幅内で計算した理想ライン（曲率最小化・"
                         "Trajectory-Aided Learning方式）からの横偏差のうち"
                         "--raceline-tolerance-mを超えた分に掛ける罰則の重み"
                         "（2026-09-01追加、v8実車評価「衝突しないが綺麗なライン取りが"
                         "できない」への対応）。初期値0.3は未検証——`sim/gym_env.py`の"
                         "`SimE2EEnv`docstring参照")
    ap.add_argument("--raceline-tolerance-m", type=float, default=0.08,
                    help="理想ラインへの追従で許容する誤差[m]。理想ラインぴったりを"
                         "要求しない許容帯（2026-09-01追加）")
    ap.add_argument("--speed-match-weight", type=float, default=0.3,
                    help="理想ライン上の目標速度（曲率に応じてグリップ限界まで"
                         "減速・加速するプロファイル）とのズレの小ささに応じて"
                         "加算するボーナス。--speed-weightと違い曲率を考慮した"
                         "速度整形を担う（2026-09-01追加）。初期値0.3は未検証——"
                         "`sim/gym_env.py`の`SimE2EEnv`docstring参照")
    ap.add_argument("--hidden-sizes", type=int, nargs="+", default=[256, 256],
                    help="方策/価値ネットワークの隠れ層サイズ（SB3既定は[64,64]）。"
                         "★2026-08-29追加: 観測は点群361点+速度1個=362次元あるのに、"
                         "SB3の既定[64,64]だと最初の層だけで362→64へ一気に圧縮されて"
                         "しまい、点群の空間的なパターン（ギャップ・壁の位置）を"
                         "表現する容量が乏しいのではという疑いがあった。CPU実測では"
                         "[256,256]でも[64,64]の約1.5倍の学習時間で収まる")
    ap.add_argument("--out", type=Path, default=Path("ml_lidar/runs/ppo_e2e"))
    ap.add_argument("--resume-from", type=Path, default=None,
                    help="このチェックポイント(.zip)から続きを学習する（PPO.load()。"
                         "ゼロから作り直さない）。--timestepsは累計の目標値として扱う")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", default="cpu",
                    help="MlpPolicyはネットワークが比較的小さく、GPU転送のオーバーヘッドが"
                         "計算量を上回りやすいため既定はcpu（SB3のドキュメントの推奨と同じ）。"
                         "`--hidden-sizes`を大きく増やす場合はGPUの方が有利になりうる")
    args = ap.parse_args()

    # ★CPU学習時、torchのデフォルトのマルチスレッド化は`--hidden-sizes`程度の
    # 小さいMlpPolicyでは並列化オーバーヘッドの方が計算量を上回りやすく、かつ
    # `--n-envs`個のワーカープロセスとCPUコアを取り合ってしまう（SB3ドキュメントの
    # 推奨と同じ理由。2026-09-01追加、CPU使用率が上がらない問題の調査から）。
    # ★ロールアウト収集中と勾配更新中でスレッド数を動的に切り替える案も試したが
    # （勾配更新中はワーカーが全員アイドルで競合しないはず、という仮説）、実測では
    # 逆にfpsが下がった（2026-09-01、M3実機でON平均502fps・OFF平均556fps）。
    # `[256,256]`程度の小さいネットワークでは勾配更新フェーズ単体でもマルチ
    # スレッド化のオーバーヘッドの方が計算量削減分を上回るため、常時1スレッド
    # 固定のままにしてある
    if args.device == "cpu":
        torch.set_num_threads(1)

    # ★新規学習（再開ではない）なら、古いTensorBoardログ（PPO_1・PPO_2…）が
    # 積み上がらないよう出力先を空にしてから作り直す（上のdocstring参照）
    if args.resume_from is None and args.out.exists():
        shutil.rmtree(args.out)
    args.out.mkdir(parents=True, exist_ok=True)
    # ★`ml_lidar/export_onnx_rl.py`の`--max-speed`はこの学習に渡した値と一致していないと
    # 壊れる（モデルの正規化出力を違う物理レンジで解釈してしまう）。覚えておく/
    # シェル履歴を掘る運用は事故のもとなので、ここに記録しておき`export_onnx_rl.py`側が
    # 省略時にここから拾えるようにする（2026-08-28）。`max_steer`はもうCLI引数ではない
    # （`config/vehicle.toml`の車両物理限界を直接使う）が、この学習で実際に使われた値の
    # 記録として残しておく（後から見返す用。`run一覧`GUIの表示にも使う）
    (args.out / "run_config.json").write_text(json.dumps({
        "max_speed": args.max_speed, "max_steer": VehicleSpec.load().max_steer,
        "max_steps": args.max_steps, "timesteps": args.timesteps,
        "n_envs": args.n_envs, "seed": args.seed,
        "n_eval_episodes": args.n_eval_episodes, "target_kl": args.target_kl,
        "n_epochs": args.n_epochs, "learning_rate": args.learning_rate,
        "steer_tau": args.steer_tau, "steer_rate_weight": args.steer_rate_weight,
        "speed_weight": args.speed_weight, "slip_weight": args.slip_weight,
        "raceline_weight": args.raceline_weight,
        "raceline_tolerance_m": args.raceline_tolerance_m,
        "speed_match_weight": args.speed_match_weight,
        "hidden_sizes": args.hidden_sizes,
    }, indent=2), encoding="utf-8")

    env_fns = [make_train_env_fn(args.seed + i, max_steps=args.max_steps,
                                 max_speed=args.max_speed, steer_tau=args.steer_tau,
                                 steer_rate_weight=args.steer_rate_weight,
                                 speed_weight=args.speed_weight,
                                 slip_weight=args.slip_weight,
                                 raceline_weight=args.raceline_weight,
                                 raceline_tolerance_m=args.raceline_tolerance_m,
                                 speed_match_weight=args.speed_match_weight)
              for i in range(args.n_envs)]
    vec_cls = SubprocVecEnv if args.n_envs > 1 else DummyVecEnv
    vec_env = vec_cls(env_fns)

    if args.resume_from is not None:
        # ★ハイパラ（target_kl/n_epochs/learning_rate等）はチェックポイントに
        # 保存済みの値がそのまま復元される——ここで指定したCLI引数は再開時には
        # 効かない（SB3の`PPO.load()`の仕様）。変えたい場合はゼロから学習し直すこと
        model = PPO.load(str(args.resume_from), env=vec_env, device=args.device,
                         tensorboard_log=str(args.out / "tb"))
        print(f"# {args.resume_from} から再開（既に {model.num_timesteps} ステップ学習済み）")
    else:
        model = PPO("MlpPolicy", vec_env, verbose=1, device=args.device, seed=args.seed,
                   tensorboard_log=str(args.out / "tb"), n_epochs=args.n_epochs,
                   target_kl=(args.target_kl if args.target_kl > 0 else None),
                   learning_rate=linear_schedule(args.learning_rate),
                   policy_kwargs=dict(net_arch=dict(pi=args.hidden_sizes, vf=args.hidden_sizes)))

    eval_env = DummyVecEnv([lambda: Monitor(make_eval_env(
        max_steps=args.max_steps, max_speed=args.max_speed, steer_tau=args.steer_tau,
        steer_rate_weight=args.steer_rate_weight, speed_weight=args.speed_weight,
        slip_weight=args.slip_weight, raceline_weight=args.raceline_weight,
        raceline_tolerance_m=args.raceline_tolerance_m,
        speed_match_weight=args.speed_match_weight, seed=args.seed + 999))])
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
    raceline_metrics_callback = RacelineMetricsCallback()

    # 早期終了時は StopTrainingOnNoModelImprovement 自身が理由をログに出す（verbose=1）
    # ★再開時は reset_num_timesteps=False で num_timesteps を引き継ぐ
    # （True のままだとカウンタが0に戻り、--timesteps 未達のまま即終了する）
    model.learn(total_timesteps=args.timesteps,
               callback=[eval_callback, raceline_metrics_callback],
               reset_num_timesteps=args.resume_from is None)
    model.save(str(args.out / "last_model"))
    print(f"# 完了。最良モデル → {args.out}/best_model.zip 、最終モデル → "
          f"{args.out}/last_model.zip")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
