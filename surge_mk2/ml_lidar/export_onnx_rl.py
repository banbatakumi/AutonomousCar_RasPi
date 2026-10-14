"""ml_lidar/export_onnx_rl.py — 学習済みPPO方策をONNX化する。

    .venv/bin/python -m ml_lidar.export_onnx_rl --model ml_lidar/runs/v1/eval/best_model.zip --name v1

`models/e2e_lidar/<name>.onnx` ＋ 同名`.json`（`fov_deg`/`max_range`/`max_steer`/
`max_speed`/`steer_rate_max_rad_s`。`raspi/auto/e2e_lidar.py`が読む契約）を書き出す。`in_dim`はJSONに
書かない——`e2e_lidar.py`はONNXグラフ自身のshapeから読む契約になっている
（同ファイルの`_load_from_path()`コメント参照）。`max_steer`は`config/vehicle.toml`
（`VehicleSpec.max_steer`）からそのまま取る——学習側が別の値を持たない設計
（`ml_lidar/env.py`のEnvConfig docstring参照）なので、ここでも同じ値を使う。

## 入出力パリティ検証（保存前に必ず通す）

書き出したONNXモデルを`onnxruntime`でロードし直し、ランダムな観測をいくつも通して
PyTorch側（SB3のpolicy → 決定論的action、`[-1,1]`クリップ込み）の出力と最大絶対誤差を
比較する。**一致しなければ保存を中止する**——過去の落とし穴チェックリスト
「アクションの符号・単位・クリップ位置の不一致」「ONNX変換後の入出力不一致」を
ここで機械的に潰す。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402
import torch as th  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402
from torch import nn  # noqa: E402

from raspi.core.vehicle import Vehicle  # noqa: E402
from sim.vehicle import VehicleSpec  # noqa: E402

from ml_lidar.env import EnvConfig  # noqa: E402
from ml_lidar.eval_stats import (  # noqa: E402
    find_env_config, find_env_config_path, steer_contract_warnings)  # noqa: E402

__all__ = ["PolicyONNXWrapper", "export_model", "check_parity", "parse_args", "main"]

DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "e2e_lidar"
_PARITY_TOL = 1e-4
_PARITY_SAMPLES = 200


class PolicyONNXWrapper(nn.Module):
    """SB3の`ActorCriticPolicy`から、決定論的行動（`[-1,1]`クリップ込み）だけを出す薄いラッパー。

    `model.predict(deterministic=True)`は内部で`distribution.mode()`（未クリップの
    ガウス平均——`squash_output=False`が既定のためtanhは掛からない）→`np.clip`という
    経路を通る（`stable_baselines3.common.policies.BasePolicy.predict`のソース確認済み）。
    ここではその経路をそのままONNXへトレースする。
    """

    def __init__(self, policy) -> None:
        super().__init__()
        if not policy.share_features_extractor:
            raise NotImplementedError(
                "share_features_extractor=False のpolicyは未対応（train_rl.pyは既定のTrueのまま使う）")
        self.features_extractor = policy.features_extractor
        self.mlp_extractor = policy.mlp_extractor
        self.action_net = policy.action_net

    def forward(self, obs: th.Tensor) -> th.Tensor:
        features = self.features_extractor(obs)
        latent_pi = self.mlp_extractor.forward_actor(features)
        mean_actions = self.action_net(latent_pi)
        return th.clamp(mean_actions, -1.0, 1.0)


def export_model(model: PPO, out_path: Path) -> None:
    policy = model.policy
    policy.set_training_mode(False)
    wrapper = PolicyONNXWrapper(policy).eval()

    obs_dim = int(model.observation_space.shape[0])
    dummy = th.zeros(1, obs_dim, dtype=th.float32)
    # external_data=False が必須（`ml_cam/export_onnx.py`と同じ）。既定だと重みが
    # `.<name>.tmp.onnx.data` という別ファイルに出て、rename した<name>.onnxがそれに依存して壊れる
    th.onnx.export(wrapper, (dummy,), str(out_path), input_names=["obs"], output_names=["action"],
                   external_data=False)


def check_parity(model: PPO, onnx_path: Path, *, n_samples: int = _PARITY_SAMPLES,
                 tol: float = _PARITY_TOL, seed: int = 0) -> float:
    import onnxruntime as ort

    wrapper = PolicyONNXWrapper(model.policy).eval()
    obs_space = model.observation_space
    rng = np.random.default_rng(seed)

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name

    max_diff = 0.0
    for _ in range(n_samples):
        obs = rng.uniform(obs_space.low, obs_space.high).astype(np.float32)
        with th.no_grad():
            torch_out = wrapper(th.from_numpy(obs[None, :])).numpy()
        onnx_out = session.run(None, {input_name: obs[None, :]})[0]
        max_diff = max(max_diff, float(np.max(np.abs(torch_out - onnx_out))))
    if max_diff > tol:
        raise RuntimeError(f"ONNX入出力パリティ検証に失敗: 最大誤差 {max_diff:.2e} > 許容 {tol:.2e}")
    return max_diff


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ml_lidar PPOモデルをONNX化する")
    p.add_argument("--model", required=True, type=Path)
    p.add_argument("--name", required=True, help="`models/e2e_lidar/<name>.onnx`の<name>")
    p.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    # env_config.jsonが見つからないときのフォールバック既定値（train_rl.pyと同じ）
    p.add_argument("--fov-deg", type=float, default=270.0)
    p.add_argument("--max-range", type=float, default=10.0)
    p.add_argument("--max-speed", type=float, default=2.0)
    p.add_argument("--steer-rate-max-rad-s", type=float, default=1.0,
                   help="[rad/s] train_rl.pyの--steer-rate-max-rad-sと同じ(v13、"
                        "env_config.jsonが見つからない場合のフォールバック)")
    p.add_argument("--max-steer", type=float, default=None,
                   help="学習したときの最大舵角[rad]（契約 JSON の max_steer）。省略は今の "
                        "config/vehicle.toml の値。vehicle.toml が学習後に変わった run を書き出すとき用")
    p.add_argument("--steer-link", type=float, nargs=2, default=None, metavar=("GAIN", "CUBIC"),
                   help="学習したときの STM32 のステアのリンクの換算（契約 JSON の steer_link）。省略は今の "
                        "config/vehicle.toml の [control]。★2026-10-08 より前の vehicle.toml（max_steer "
                        "0.524）で学習したモデルを今から書き出すなら `--steer-link 1 0` を付けること★")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    cfg = find_env_config(args.model)
    if cfg is None:
        print(f"!! {args.model} の近くにenv_config.jsonが見つからない。CLI既定値にフォールバックする",
             file=sys.stderr)
        cfg = EnvConfig(fov_deg=args.fov_deg, max_range=args.max_range,
                        max_speed=args.max_speed,
                        steer_rate_max_rad_s=args.steer_rate_max_rad_s)
    # 最大舵角はEnvConfigに持たせていない（`config/vehicle.toml`固定）ので、
    # モデル契約JSONへはここで直接読んで書く
    max_steer = args.max_steer if args.max_steer else VehicleSpec.load().max_steer
    # 舵角の数値の意味（`raspi/auto/e2e_lidar.py` が今の車の路面舵角へ直すのに使う）
    steer_link = list(args.steer_link) if args.steer_link else list(Vehicle.load().steer_link)
    # run が記録した学習時の値と、今書こうとしている値が違えば警告する（無い古い run もその旨）
    for w in steer_contract_warnings(find_env_config_path(args.model), max_steer, steer_link):
        print(f"!! {w}", file=sys.stderr)

    model = PPO.load(str(args.model), device="cpu")

    args.models_dir.mkdir(parents=True, exist_ok=True)
    tmp_path = args.models_dir / f".{args.name}.tmp.onnx"
    final_path = args.models_dir / f"{args.name}.onnx"
    json_path = args.models_dir / f"{args.name}.json"

    export_model(model, tmp_path)
    try:
        max_diff = check_parity(model, tmp_path)
    except Exception:
        tmp_path.unlink(missing_ok=True)
        raise
    print(f"入出力パリティ OK（最大誤差 {max_diff:.2e}）")

    tmp_path.replace(final_path)
    json_path.write_text(json.dumps({
        "fov_deg": cfg.fov_deg,
        "max_range": cfg.max_range,
        "max_steer": max_steer,
        "max_speed": cfg.max_speed,
        "steer_rate_max_rad_s": cfg.steer_rate_max_rad_s,
        "steer_link": steer_link,
        "dt": cfg.dt,                    # [s] 学習時の決定周期（推論側は0.10±0.02外を拒否）
        "steer_tau": cfg.steer_tau,      # [s] 学習時の舵の一次遅れ時定数
    }, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"→ {final_path}\n→ {json_path}")


if __name__ == "__main__":
    main()
