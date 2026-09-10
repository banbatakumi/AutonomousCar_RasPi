"""ml_lidar/train_rl.py — PPO学習エントリ（Stable-Baselines3）。

    .venv/bin/python -m ml_lidar.train_rl --run-name v1 --total-timesteps 1000000

設計方針・確定した仕様は `ML_LIDAR_V2_PROMPT.md` を参照。報酬は`progress + collision_penalty`
の最小構成（`ml_lidar/env.py`）。TAL項・報酬正規化・カリキュラム学習は入れない。

## 同じrun名で再実行するとき

既存runがある状態で`--resume`/`--overwrite`のどちらも指定せずに実行すると、
上書き事故を避けるためエラーで止まる。`--resume`は最新チェックポイント
（無ければ`final_model`）から`--total-timesteps`ぶん追加学習する
（観測・行動契約は保存済み`env_config.json`を使い、`--fov-deg`等のCLI引数は無視する）。
`--overwrite`はrunの中身を消してから0から学習する。

## 「安全な位置で停止する縮退方策」を最初に疑うこと

`progress`は速度に比例する設計だが、学習初期の探索不足で「動かない方が安全」という
局所最適に落ちることがある（Copilot CLIによる独立レビューでの指摘）。学習曲線で
`ep_len_mean`が打ち切り上限（`--episode-s`）に張り付いたまま`rollout/ep_rew_mean`が
ほぼ0から伸びないなら、これを疑うこと——対策は小さな生存ペナルティ、または
最低速度未満をprogress=0扱いにすることだが、**実際にこの失敗が観測されるまでは
`env.py`に入れない**（最小構成から始める方針）。

## eval用コースは5種の固定コース（best_model選定の1種類への過適合を避ける）

`EvalCallback`のeval環境は「直線主体」「コーナー主体」「中間」「ヘアピン主体」「連続シケイン
主体」の5本を1つの`VecEnv`にまとめたもの（`make_eval_vec_env()`）。ヘアピン主体は、手続き生成の
出力空間を正n角形から閉じた極座標スプラインへ置き換えた際に追加した——toyotaコース
（急角のシケインが連続する手描きコース）で曲がりきれず衝突する、という実機投入前の
観戦チェックで見つかった失敗パターンを、`best_model`選定の指標にも反映するため
（`ml_lidar/course_gen.py`のモジュールdocstring参照）。連続シケイン主体は、ヘアピン主体が
孤立1発の折り返ししか再現しておらず、toyotaの「短い直線を挟んで急角ターンが連続する」配置とは
質的に異なると分かったため、v5着手時に追加した（`course_gen.py`の`chicane_len`参照）。
VecEnv内の各サブ環境は常に同じ固定コースへ`reset()`するので、`--n-eval-episodes`を5の倍数に
すれば評価が自然に5種へ均等配分される。ノイズ・ドメインランダム化は無効化する（訓練環境の
ランダム化込みの衝突率でモデル選定してはいけない、という過去の落とし穴）。

## `EvalCallback`だけではbest_model選定に使えない（実体は`GeneralizationEvalCallback`）

`EvalCallback`は`deterministic=True`・ノイズ/DRオフの5固定コースを評価する——決定論的
方策×決定論的環境なので、`--n-eval-episodes`を何回に増やしても同じ5本の軌道を
繰り返すだけで、実際に得られる情報量は「5本のコースそれぞれの成否」の5ビットぶんしか
ない（v1/v2検証、2026-09-07。`deterministic=False`で書いていた最初の検証スクリプトが
たまたま探索ノイズ込みの数値を返し、一見意味のある分散に見えていたのが判明の経緯）。
`EvalCallback`自体はresumeに必要な`eval/best_model.zip`を作り続けるためそのまま残すが、
**実運用のモデル選定は`GeneralizationEvalCallback`が書き出す`eval/best_model_generalized.zip`
の方を使うこと**——`eval_stats.py`と同じ「ランダムコース・DRあり」の統計評価を学習ループ内で
定期的に走らせ、`mean_reward`が改善したときだけ更新する。

こちらも呼び出しのたびに`base_seed`を`self.num_timesteps`から作るため、v1/v2検証で
「単一のseed系統(n=100)ではv2が0%衝突・v1が3%衝突に見えたが、別のseed系統を足して
合計n=550まで積み増すとv1=2.2%・v2=5.5%で結論が逆転した」という再現済みの落とし穴を、
特定のseed系統に固定して評価し続けることでは踏まないようにしてある。1回あたり
`n_gen_eval_episodes`本のランダムコースを走らせる分`EvalCallback`より重いので、
既定の間隔(`--gen-eval-freq`)は`--eval-freq`より疎にしてある。

## `sim/courses/`の自作コードは使わない

学習・eval双方とも手続き生成コース（`ml_lidar/course_gen.py`）のみを使う。
自作コース（`course1.json`等）は学習プロセスに一切関与しない観戦専用
（`ml_lidar/watch.py`）に限定する方針（`ML_LIDAR_V2_PROMPT.md`）。
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import shutil
import sys
from pathlib import Path
from typing import Callable, NamedTuple

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402
from gymnasium.wrappers import TimeLimit  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, EvalCallback  # noqa: E402
from stable_baselines3.common.monitor import Monitor  # noqa: E402
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecEnv  # noqa: E402

from sim.course import Course  # noqa: E402

from ml_lidar.course_gen import walk_loop_course  # noqa: E402
from ml_lidar.env import EnvConfig, LidarE2EEnv  # noqa: E402
from ml_lidar.eval_stats import evaluate as eval_stats_evaluate  # noqa: E402
from ml_lidar.features import Conv1dExtractor  # noqa: E402
from ml_lidar.run_info import has_existing_run  # noqa: E402

__all__ = ["RUNS_DIR", "EvalCourseSpec", "EVAL_COURSE_PARAMS", "GeneralizationEvalCallback",
          "build_env_config", "has_existing_run", "make_train_env", "make_eval_vec_env",
          "build_model", "find_latest_checkpoint", "find_watch_source", "load_saved_env_config",
          "parse_args", "main"]

RUNS_DIR = ML_LIDAR_DIR / "runs"

class EvalCourseSpec(NamedTuple):
    name: str
    n_checkpoints: int
    base_radius_m: float
    angle_jitter_frac: float
    radius_jitter_frac: float
    width_m: float
    seed: int
    clockwise: bool = False
    chicane_len: int = 0
    chicane_offset_frac: float = 0.0

    def build_course(self, role: str = "eval") -> Course:
        """この評価アーキタイプのコースを1本作る（`train_rl.py`のeval・
        `live_watch.py`のパネル表示が同じシードから同じ形状を再現するのに使う）。
        """
        return walk_loop_course(self.n_checkpoints, self.base_radius_m, self.angle_jitter_frac,
                                self.radius_jitter_frac, self.width_m, seed=self.seed,
                                role=role, name=self.name, clockwise=self.clockwise,
                                chicane_len=self.chicane_len,
                                chicane_offset_frac=self.chicane_offset_frac)


#: 5種の固定評価コース。手続き生成をチェックポイント＋旋回率制限ウォーク方式へ置き換えた
#: 際に再構成した（`course_gen.py`のモジュールdocstring参照）——旧・極座標r(θ)方式は
#: 「中心1点を囲む星型」にしか写像できない構造的な限界があり、直線/コーナー/中間/
#: ヘアピン/連続シケインという5本の名前が付いていても見た目がほぼ同じ「歪んだドーナツ」に
#: しかならなかった（2026-09、ユーザーが可視化画面で指摘）。新方式は中心1点に依存しない
#: ため、以下のパラメータで実際に視覚的に別形状になる（scratchpadで`bbox`・最小旋回半径・
#: 方向転換回数を実測して較正済み）。シードを固定することで、`train_rl.py`を何度実行しても
#: 同じ5本の形状で評価する（`EvalCallback`の経時比較が意味を持つための前提）。
#:
#: 各アーキタイプの狙い:
#:   straight: チェックポイント少(6)・ジッター極小・半径大——チェックポイント間隔が
#:             広く方向転換もほぼ無いので、長い直線区間をつなぐ多角形に近くなる
#:   corner:   チェックポイント多め(9)・角度/半径ジッターとも強めにして、常に旋回率
#:             上限付近を歩かせる——ループ全体を通して曲率が高い状態が続く
#:   medium:   その中間（学習側の分布の典型値に最も近い）
#:   hairpin:  チェックポイント少なく(6)半径ジッターだけ大きくする——1〜2箇所の
#:             チェックポイントが極端に内側/外側へ振れ、旋回率制限ウォークがそこで
#:             最小旋回半径ぎりぎりの弧を長く描く「本物のヘアピン」になる
#:   chicane:  背景(angle/radius_jitter_frac)は最小域で穏やかにし、`chicane_len=3`・
#:             `chicane_offset_frac`だけでクラスタを際立たせる。連続する3点の半径を
#:             外→内→外と交互に振ることで、S字に蛇行する連続した方向転換
#:             （`sim/courses/toyota.json`の「短い直線を挟んで急角ターンが連続する」
#:             配置に近い）が出る。`hairpin`（孤立1発の折り返し）とは質的に区別される
#:             ——**背景を穏やかにするのは意図的**。`corner`と同じ強い背景に
#:             `chicane_len`を足すと、`corner`が既に旋回率上限ギリギリで常時旋回しており
#:             chicane側の「クラスタだけ際立つ」効果が背景に埋もれて区別できなくなる
#:             （旧方式での既知の教訓、`course_gen.py`のモジュールdocstring参照）
#:
#: `clockwise`は両方向が混ざるよう割り当ててある（5本なので完全な2本ずつにはならない）
#: ——学習側（`random_walk_loop_course()`）は旋回方向を毎エピソード50/50でランダム化
#: しているので、eval・観戦パネルが特定方向に偏らないよう明示的に指定する
EVAL_COURSE_PARAMS: tuple[EvalCourseSpec, ...] = (
    EvalCourseSpec("straight", 6, 3.5, 0.15, 0.10, 1.1, seed=1001, clockwise=False),
    EvalCourseSpec("corner", 9, 2.0, 0.45, 0.45, 1.0, seed=1002, clockwise=True),
    EvalCourseSpec("medium", 8, 2.5, 0.30, 0.30, 1.0, seed=1003, clockwise=False),
    EvalCourseSpec("hairpin", 6, 1.8, 0.20, 0.65, 1.0, seed=1004, clockwise=True),
    # 背景(angle/radius_jitter_frac)は最小域で穏やかにし、chicane_offset_fracだけで
    # クラスタを際立たせる。scratchpadでseed 1000〜1029を走査し、方向転換が明確に
    # 複数回(6回)出る、かつ生成が一発(リトライ無し)で成功するseedを選んである
    EvalCourseSpec("chicane", 9, 2.5, 0.15, 0.12, 1.0, seed=1011, clockwise=True,
                   chicane_len=3, chicane_offset_frac=0.40),
)


def build_env_config(args: argparse.Namespace) -> EnvConfig:
    return EnvConfig(
        fov_deg=args.fov_deg,
        max_range=args.max_range,
        max_speed=args.max_speed,
        steer_tau=args.steer_tau,
        steer_rate_weight=args.steer_rate_weight,
        dynamics_jitter_frac=args.dynamics_jitter_frac,
    )


def _max_episode_steps(cfg: EnvConfig, episode_s: float) -> int:
    return max(1, int(round(episode_s / cfg.dt)))


def find_latest_checkpoint(run_dir: Path) -> Path | None:
    """`--resume`の再開対象。最新チェックポイント（ステップ数最大）→無ければfinal_model。"""
    ckpt_dir = run_dir / "checkpoints"
    candidates = list(ckpt_dir.glob("ppo_*_steps.zip")) if ckpt_dir.is_dir() else []
    if candidates:
        def _steps(p: Path) -> int:
            try:
                return int(p.stem.split("_")[-2])
            except (IndexError, ValueError):
                return -1
        return max(candidates, key=_steps)
    final = run_dir / "final_model.zip"
    return final if final.exists() else None


def find_watch_source(run_dir: Path) -> tuple[Path, float] | None:
    """観戦（`ml_lidar/live_watch.py`）に使う最新モデルの`(パス, mtime)`。

    観戦専用の追加保存は行わず、既存の成果物だけを見る。`eval/best_model.zip`
    （`EvalCallback`が既定`--eval-freq`2万step間隔で更新——`checkpoints/`の
    resume用チェックポイント（既定10万step間隔）より高頻度）と
    `find_latest_checkpoint()`（checkpoints/→final_model）のうち、
    実際に**mtimeが新しい方**を返す（best_modelは「改善したときだけ」更新される
    ので、改善が止まっている間はchekpoints/の方が新しくなることもあるため）。
    """
    best = run_dir / "eval" / "best_model.zip"
    ckpt = find_latest_checkpoint(run_dir)
    candidates = [p for p in (best, ckpt) if p is not None and p.exists()]
    if not candidates:
        return None
    newest = max(candidates, key=lambda p: p.stat().st_mtime)
    return newest, newest.stat().st_mtime


def load_saved_env_config(run_dir: Path) -> EnvConfig | None:
    """`<run_dir>/env_config.json`を読む。`ml_lidar/live_watch.py`も同じものを使う。"""
    path = run_dir / "env_config.json"
    if not path.exists():
        return None
    data = json.loads(path.read_text(encoding="utf-8"))
    field_names = {f.name for f in dataclasses.fields(EnvConfig)}
    return EnvConfig(**{k: v for k, v in data.items() if k in field_names})


def make_train_env(cfg: EnvConfig, max_episode_steps: int, seed: int) -> Callable[[], Monitor]:
    """`SubprocVecEnv`の1ワーカーぶんのファクトリ。コースは`course_gen`が毎`reset()`で作り直す。"""
    def _thunk() -> Monitor:
        env = LidarE2EEnv(cfg, seed=seed)
        env = TimeLimit(env, max_episode_steps=max_episode_steps)
        return Monitor(env)
    return _thunk


def make_eval_vec_env(cfg: EnvConfig, max_episode_steps: int) -> VecEnv:
    """直線主体・コーナー主体・中間・ヘアピン主体・連続シケイン主体の5固定コースを束ねた
    eval用VecEnv。ノイズ・DRは無効化する。"""
    eval_cfg = EnvConfig(**{**cfg.__dict__, "randomize_lidar": False, "randomize_dynamics": False})

    def _make_thunk(spec: EvalCourseSpec) -> Callable[[], Monitor]:
        course = spec.build_course()

        def _thunk() -> Monitor:
            env = LidarE2EEnv(eval_cfg, course=course)
            env = TimeLimit(env, max_episode_steps=max_episode_steps)
            return Monitor(env)
        return _thunk

    return DummyVecEnv([_make_thunk(spec) for spec in EVAL_COURSE_PARAMS])


class GeneralizationEvalCallback(BaseCallback):
    """`eval_stats.py`と同じ「ランダムコース・DRあり」の統計評価を学習ループ内で定期実行し、
    `mean_reward`が改善したときだけ`<save_dir>/best_model_generalized.zip`を更新する。

    `EvalCallback`（5固定コース・決定論的）はresume用の`eval/best_model.zip`を作り続けるため
    残すが、実運用のモデル選定はこちら（モジュールdocstring参照）。呼び出しごとに
    `base_seed`を`self.num_timesteps`から作ることで、特定のseed系統に固定して評価し
    続けることによる誤判定（v1/v2検証で実際に踏んだ、単一seed系統では結論が逆転した
    落とし穴）を避ける。
    """

    def __init__(self, cfg: EnvConfig, save_dir: Path, eval_freq: int, n_episodes: int,
                episode_s: float, verbose: int = 0) -> None:
        super().__init__(verbose)
        self.cfg = dataclasses.replace(cfg, randomize_lidar=True, randomize_dynamics=True)
        self.save_dir = save_dir
        self.eval_freq = max(1, eval_freq)
        self.n_episodes = n_episodes
        self.episode_s = episode_s
        self._log_path = save_dir / "generalization_evaluations.npz"
        (self._timesteps, self._mean_rewards, self._collision_rates, self._mean_speeds,
         self._mean_abs_steer_diffs, self._steer_sign_flip_rates) = self._load_history()
        self.best_mean_reward = max(self._mean_rewards) if self._mean_rewards else -np.inf

    def _load_history(self) -> tuple[list[int], list[float], list[float], list[float],
                                     list[float], list[float]]:
        """既存の`generalization_evaluations.npz`があれば読み込む（`--resume`でも
        `best_mean_reward`と履歴グラフが引き継がれるようにする——0からだと、resume直後に
        悪化したモデルでも`best_model_generalized`を上書きしてしまいかねない）。

        `mean_abs_steer_diff`/`steer_sign_flip_rate`はv8で追加した列なので、
        それより前のrunを`--resume`した場合は無い——欠損時は空のまま扱う。
        """
        if not self._log_path.exists():
            return [], [], [], [], [], []
        data = np.load(self._log_path)
        steer_diff = list(data["mean_abs_steer_diff"]) if "mean_abs_steer_diff" in data else []
        sign_flip = list(data["steer_sign_flip_rate"]) if "steer_sign_flip_rate" in data else []
        return (list(data["timesteps"]), list(data["mean_reward"]),
               list(data["collision_rate"]), list(data["mean_speed"]), steer_diff, sign_flip)

    def _on_step(self) -> bool:
        if self.n_calls % self.eval_freq != 0:
            return True
        base_seed = int(self.num_timesteps)
        result = eval_stats_evaluate(self.model, self.cfg, n_episodes=self.n_episodes,
                                     episode_s=self.episode_s, base_seed=base_seed)
        self._timesteps.append(self.num_timesteps)
        self._mean_rewards.append(result["mean_reward"])
        self._collision_rates.append(result["collision_rate"])
        self._mean_speeds.append(result["mean_speed"])
        self._mean_abs_steer_diffs.append(result["mean_abs_steer_diff"])
        self._steer_sign_flip_rates.append(result["steer_sign_flip_rate"])
        self.save_dir.mkdir(parents=True, exist_ok=True)
        np.savez(self._log_path,
                timesteps=np.array(self._timesteps),
                mean_reward=np.array(self._mean_rewards),
                collision_rate=np.array(self._collision_rates),
                mean_speed=np.array(self._mean_speeds),
                mean_abs_steer_diff=np.array(self._mean_abs_steer_diffs),
                steer_sign_flip_rate=np.array(self._steer_sign_flip_rates))
        if self.verbose:
            print(f"[GeneralizationEval] step={self.num_timesteps} "
                 f"mean_reward={result['mean_reward']:.2f} "
                 f"collision_rate={result['collision_rate']:.1%} "
                 f"mean_speed={result['mean_speed']:.2f}m/s "
                 f"mean_abs_steer_diff={result['mean_abs_steer_diff']:.3f} "
                 f"steer_sign_flip_rate={result['steer_sign_flip_rate']:.1%}")
        if result["mean_reward"] > self.best_mean_reward:
            self.best_mean_reward = result["mean_reward"]
            self.model.save(str(self.save_dir / "best_model_generalized"))
        return True


def build_model(vec_env: VecEnv, cfg: EnvConfig, args: argparse.Namespace, tb_log: Path) -> PPO:
    n_window = int(round(cfg.fov_deg)) + 1
    policy_kwargs = dict(
        features_extractor_class=Conv1dExtractor,
        features_extractor_kwargs=dict(n_scan=n_window, features_dim=args.features_dim),
        log_std_init=args.log_std_init,
    )
    return PPO(
        "MlpPolicy", vec_env,
        learning_rate=args.learning_rate,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range,
        ent_coef=args.ent_coef,
        use_sde=args.use_sde,
        sde_sample_freq=args.sde_sample_freq,
        policy_kwargs=policy_kwargs,
        tensorboard_log=str(tb_log),
        seed=args.seed,
        device=args.device,
        verbose=1,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="ml_lidar PPO学習（Stable-Baselines3）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    # ── 実際に調整見込みのある基本パラメータ（GUIの常時表示タブに出す想定） ──
    p.add_argument("--run-name", required=True, help="`ml_lidar/runs/<run-name>/`に出力する")
    p.add_argument("--resume", action="store_true",
                   help="既存runの続きから学習する（最新チェックポイント→無ければfinal_modelを再開。"
                        "観測・行動契約は保存済みenv_config.jsonを使い、モデル契約系の引数は無視する）")
    p.add_argument("--overwrite", action="store_true",
                   help="既存runの中身を消してから0から学習する")
    p.add_argument("--total-timesteps", type=int, default=1_000_000,
                   help="--resume指定時は「これまでの続きに何ステップ追加するか」の意味になる")
    p.add_argument("--n-envs", type=int, default=8)
    p.add_argument("--episode-s", type=float, default=30.0, help="TimeLimitで打ち切るまでの秒数")
    p.add_argument("--max-speed", type=float, default=2.0, help="[m/s]")
    p.add_argument("--seed", type=int, default=0)

    # ── モデル契約（`raspi/auto/e2e_lidar.py`・`export_onnx_rl.py`と揃える） ──
    # 最大舵角は`config/vehicle.toml`（`VehicleSpec.max_steer`）をそのまま使うため、
    # ここには無い（`env.py`のEnvConfig docstring参照）
    p.add_argument("--fov-deg", type=float, default=270.0)
    p.add_argument("--max-range", type=float, default=10.0, help="[m] LiDAR観測レンジ")
    p.add_argument("--steer-tau", type=float, default=0.10, help="[s] プランナ側の舵平滑化")
    p.add_argument("--steer-rate-weight", type=float, default=0.0,
                   help="生アクションa[0](-1..1、フィルタ前)の隣接ステップ差分への罰則重み。"
                        "既定0(無効)。v8でv1〜v7の生アクション振動(env.py docstring参照)に"
                        "対処するため導入。強すぎると過度に保守的な方策に倒れるので、"
                        "eval_stats.pyのステア滑らかさ指標とcollision_rate/mean_speedを"
                        "同時に見ながら調整すること")

    # ── 評価 ──
    p.add_argument("--eval-freq", type=int, default=20_000,
                   help="学習ステップ何回ごとに評価するか（全env合算の総ステップ数基準）")
    p.add_argument("--n-eval-episodes", type=int, default=10,
                   help="1回のeval合計エピソード数。5の倍数だと5コースへ均等配分される")
    p.add_argument("--checkpoint-freq", type=int, default=100_000,
                   help="resume用の番号付きチェックポイント間隔[step]")
    p.add_argument("--gen-eval-freq", type=int, default=100_000,
                   help="汎化性能評価(ランダムコース・DRあり、実運用のbest_model選定の実体。"
                        "モジュールdocstring参照)の間隔[step]。--eval-freqの5固定コース評価より"
                        "1回あたり重いため既定を疎にしてある")
    p.add_argument("--gen-eval-episodes", type=int, default=200,
                   help="1回の汎化性能評価で走らせるランダムコースのエピソード数。"
                        "v1/v2検証でn=100程度では衝突率の推定が全く安定しないと分かったため"
                        "それより多くしてあるが、学習を止めて実行するコストとのバランスで"
                        "eval_stats.py単体実行時の既定(300)より少なめに抑えてある。"
                        "学習後の最終確認は`eval_stats.py`を複数`--seed`で実行して行うこと")

    # ── ドメインランダム化 ──
    p.add_argument("--dynamics-jitter-frac", type=float, default=0.2,
                   help="車両動特性(tau_steer_s等)を既定値の±何割ランダム化するか")
    p.add_argument("--no-randomize-lidar", action="store_true")
    p.add_argument("--no-randomize-dynamics", action="store_true")

    # ── PPO詳細ハイパラ（原因切り分け用途中心。GUIでは折りたたみ/別タブ想定） ──
    p.add_argument("--learning-rate", type=float, default=3e-4)
    p.add_argument("--n-steps", type=int, default=2048, help="1env・1更新あたりのロールアウト長")
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--n-epochs", type=int, default=10)
    p.add_argument("--gamma", type=float, default=0.99)
    p.add_argument("--gae-lambda", type=float, default=0.95)
    p.add_argument("--clip-range", type=float, default=0.2)
    p.add_argument("--ent-coef", type=float, default=0.0)
    p.add_argument("--use-sde", action="store_true",
                   help="gSDE(generalized State-Dependent Exploration、Raffin+2021 "
                        "arXiv:2005.05719)を有効化。毎ステップi.i.d.なガウス探索ノイズの"
                        "代わりに、方策特徴量に依存し`--sde-sample-freq`ステップごとにしか"
                        "再サンプリングしないノイズを使う——生アクションの振動(v7〜v10で"
                        "reward側のsteer_rate_weightを0/0.03/0.1/0.2と振っても単調悪化する"
                        "だけだったため探索ノイズの生成過程自体を変える方向、PROGRESS.md"
                        "2026-09-10節参照)。報酬は変えないので`--steer-rate-weight`とは"
                        "独立——両方同時に有効化すると切り分けができなくなるので、"
                        "gSDE単体で試す間は`--steer-rate-weight 0`のままにすること")
    p.add_argument("--sde-sample-freq", type=int, default=4,
                   help="gSDEのノイズ再サンプリング間隔[step]。小さいほど毎ステップ独立の"
                        "通常ノイズに近づき、大きいほど探索が単調になる。論文でPPOに対して"
                        "良好だった4〜8のオーダーを既定にしてある")
    p.add_argument("--log-std-init", type=float, default=0.0,
                   help="方策の探索ノイズ標準偏差の初期値(exp(log_std_init))。SB3既定は0.0だが、"
                        "gSDE有効時はノイズが方策側MLPの潜在層(net_archのpi、既定64次元。"
                        "features_dimそのものではない)ぶんの内積で作られるため、実効的な標準偏差が"
                        "理論上sqrt(latent_dim_pi)倍(既定なら8倍)に膨らむ——v11(log_std_init=0.0)で"
                        "train/approx_klが学習序盤に1.62(通常は0.01前後が目安)まで跳ね上がり"
                        "崩壊したのはこれが原因と特定済み(PROGRESS.md 2026-09-10節)。"
                        "--use-sde時はexp(log_std_init)*sqrt(latent_dim_pi)が1程度になるよう"
                        "-3前後まで下げること")
    p.add_argument("--features-dim", type=int, default=128)
    p.add_argument("--device", default="auto")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    if args.resume and args.overwrite:
        print("!! --resumeと--overwriteは同時に指定できない", file=sys.stderr)
        raise SystemExit(1)

    run_dir = RUNS_DIR / args.run_name
    has_existing = has_existing_run(run_dir)
    if has_existing and not args.resume and not args.overwrite:
        print(f"!! {run_dir} には既に学習結果がある。続きから学習するなら--resume、"
             f"0から上書きするなら--overwriteを指定すること", file=sys.stderr)
        raise SystemExit(1)

    if args.overwrite and run_dir.exists():
        shutil.rmtree(run_dir)
        has_existing = False
    resuming = args.resume and has_existing

    if resuming:
        # 観測・行動契約が学習済みモデルとズレるとobservation_spaceが合わずクラッシュするので、
        # モデル契約系のCLI引数は無視し、保存済みenv_config.jsonをそのまま使う
        cfg = load_saved_env_config(run_dir)
        if cfg is None:
            print(f"!! {run_dir} にenv_config.jsonが無く再開できない。--overwriteでやり直すこと",
                 file=sys.stderr)
            raise SystemExit(1)
        print(f"→ 既存のenv_config.json({run_dir / 'env_config.json'})を使って再開する。"
             "モデル契約系のCLI引数（--fov-deg等）は無視される")
    else:
        cfg = build_env_config(args)
        if args.no_randomize_lidar:
            cfg.randomize_lidar = False
        if args.no_randomize_dynamics:
            cfg.randomize_dynamics = False

    max_episode_steps = _max_episode_steps(cfg, args.episode_s)

    run_dir.mkdir(parents=True, exist_ok=True)
    if not resuming:
        # `eval_stats.py`/`export_onnx_rl.py`がモデルの観測・行動契約を自動で読めるように、
        # 学習に使った実際のEnvConfigをそのまま保存しておく（CLI引数の再入力に頼ると
        # 値がズレたまま評価・エクスポートしてしまう事故を防ぐ）
        (run_dir / "env_config.json").write_text(
            json.dumps(dataclasses.asdict(cfg), indent=2, ensure_ascii=False), encoding="utf-8")

    env_fns = [make_train_env(cfg, max_episode_steps, seed=args.seed + i)
              for i in range(args.n_envs)]
    vec_env: VecEnv = (SubprocVecEnv(env_fns) if args.n_envs > 1 else DummyVecEnv(env_fns))
    eval_env = make_eval_vec_env(cfg, max_episode_steps)

    if resuming:
        resume_path = find_latest_checkpoint(run_dir)
        if resume_path is None:
            print(f"!! {run_dir} に再開対象（checkpoints/またはfinal_model.zip）が無い。"
                 "--overwriteで0から始めること", file=sys.stderr)
            raise SystemExit(1)
        print(f"→ {resume_path} から再開する")
        model = PPO.load(str(resume_path), env=vec_env, device=args.device,
                         tensorboard_log=str(run_dir / "tb"))
    else:
        model = build_model(vec_env, cfg, args, tb_log=run_dir / "tb")

    callbacks = [
        EvalCallback(
            eval_env,
            best_model_save_path=str(run_dir / "eval"),
            log_path=str(run_dir / "eval"),
            eval_freq=max(1, args.eval_freq // args.n_envs),
            n_eval_episodes=args.n_eval_episodes,
            deterministic=True,
        ),
        CheckpointCallback(
            save_freq=max(1, args.checkpoint_freq // args.n_envs),
            save_path=str(run_dir / "checkpoints"),
            name_prefix="ppo",
        ),
        GeneralizationEvalCallback(
            cfg,
            save_dir=run_dir / "eval",
            eval_freq=max(1, args.gen_eval_freq // args.n_envs),
            n_episodes=args.gen_eval_episodes,
            episode_s=args.episode_s,
            verbose=1,
        ),
    ]

    try:
        model.learn(total_timesteps=args.total_timesteps, callback=callbacks,
                   tb_log_name=args.run_name, reset_num_timesteps=not resuming)
    finally:
        model.save(str(run_dir / "final_model"))
        vec_env.close()
        eval_env.close()


if __name__ == "__main__":
    main()
