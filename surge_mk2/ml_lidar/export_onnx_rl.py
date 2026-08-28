"""ml_lidar/export_onnx_rl.py — 学習済みPPO方策の行動ヘッドだけをONNX化する。

    .venv/bin/python ml_lidar/export_onnx_rl.py \
        --model ml_lidar/runs/ppo_e2e/best_model.zip \
        --max-speed 1.5 --max-steer 0.45

SB3の`ActorCriticPolicy`は`(actions, values, log_prob)`を返すが、実機推論で要るのは
決定論的な行動（分布の平均）だけなので、それだけを返す薄いラッパーをONNX化する。

**`--max-speed`/`--max-steer`は`train_rl.py`に渡したのと同じ値にすること。**
SB3のcheckpointは方策ネットワークの重みだけを持ち、環境側の行動レンジ（`ml_lidar/env.py`
の`GymSurgeEnv._to_physical()`が使う値）は保存されない。ここでズレると、
`models/e2e_lidar.json`に書く契約と実際に学習した行動レンジが食い違う。

方策のGaussian分布は出力を`[-1,1]`にクリップしない（SACのtanh squashと違う）ので、
**推論側（`raspi/auto/e2e_lidar.py`）で必ず`[-1,1]`へクリップしてから物理単位へ戻すこと**。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # surge_mk2/

import numpy as np  # noqa: E402
import onnxruntime as ort  # noqa: E402
import torch  # noqa: E402
import torch.nn as nn  # noqa: E402
from stable_baselines3 import PPO  # noqa: E402

from raspi.msgs import LIDAR_C_SATURATED_M  # noqa: E402
from sim.gym_env import OBS_DIM  # noqa: E402

__all__ = ["ActionOnlyPolicy", "export", "verify_parity"]


class ActionOnlyPolicy(nn.Module):
    """SB3の`ActorCriticPolicy`から決定論的行動（分布の平均）だけを取り出す。

    `policy(obs, deterministic=True)` は内部で `torch.distributions.Normal` を
    組み立てるが、そのコンストラクタにONNXエクスポータ（torch 2.13の既定である
    dynamoベースのエクスポータ）がつまずくデータ依存の分岐があるため使わない。
    **平均行動は`action_net`（線形層）の出力そのもの**（PPOのGaussian方策は
    SACのようにtanhで押し込めない）なので、分布オブジェクトを経由せず
    `extract_features → mlp_extractor.forward_actor → action_net` だけを直接呼ぶ。
    """

    def __init__(self, policy: nn.Module) -> None:
        super().__init__()
        self.extractor = policy.features_extractor
        self.forward_actor = policy.mlp_extractor.forward_actor
        self.action_net = policy.action_net

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        features = self.extractor(obs)
        latent_pi = self.forward_actor(features)
        return self.action_net(latent_pi)


def export(model_path: Path, out_path: Path, *, max_speed: float, max_steer: float,
          fov_deg: float = 360.0, max_range: float = LIDAR_C_SATURATED_M) -> None:
    model = PPO.load(str(model_path), device="cpu")
    wrapper = ActionOnlyPolicy(model.policy).eval()

    # 推論側（e2e_lidar.py）は常にバッチサイズ1で呼ぶので、動的軸にはしない。
    # **`external_data=False`が必須。** 既定(True)だと重みが `<out_path>.data` という
    # 別ファイルに切り出され、`.onnx` 単体をコピー/配置すると壊れる
    # （実際に踏んだ: モデル名を変えてコピーしたら参照が外れてロードに失敗した）
    dummy = torch.zeros(1, OBS_DIM, dtype=torch.float32)
    torch.onnx.export(wrapper, dummy, str(out_path), input_names=["input"],
                      output_names=["action"], opset_version=18, external_data=False)

    out_path.with_suffix(".json").write_text(json.dumps({
        "fov_deg": fov_deg,
        "max_range": max_range,
        "in_dim": OBS_DIM,
        "input": f"先頭{OBS_DIM - 1}点はscan/max_range、末尾1個はspeed/max_speed（共に[0,1]）",
        "action": "[steer_norm, speed_norm] each in [-1,1] (要クリップ)。"
                  "steer_rad = clip(a0,-1,1) * max_steer 。"
                  "speed_mps = (clip(a1,-1,1) + 1) / 2 * max_speed",
        "max_steer": max_steer,
        "max_speed": max_speed,
    }, indent=2, ensure_ascii=False), encoding="utf-8")


def verify_parity(model_path: Path, onnx_path: Path, *, n_samples: int = 8,
                  atol: float = 1e-4) -> float:
    """PyTorchとONNXRuntimeの出力を数値突合する。差が大きければ例外。

    ONNXモデルはバッチサイズ1固定でエクスポートしている（推論側は常に1件ずつ呼ぶため）
    ので、1件ずつ回す。
    """
    model = PPO.load(str(model_path), device="cpu")
    wrapper = ActionOnlyPolicy(model.policy).eval()
    sess = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])

    rng = np.random.default_rng(0)
    x = rng.uniform(0.0, 1.0, size=(n_samples, OBS_DIM)).astype(np.float32)
    diff = 0.0
    for i in range(n_samples):
        xi = x[i:i + 1]
        with torch.no_grad():
            torch_out = wrapper(torch.from_numpy(xi)).numpy()
        onnx_out = sess.run(None, {"input": xi})[0]
        diff = max(diff, float(np.abs(torch_out - onnx_out).max()))

    if diff > atol:
        raise RuntimeError(f"PyTorchとONNXRuntimeの出力が一致しない（最大差 {diff}）")
    return diff


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", type=Path, default=Path("ml_lidar/runs/ppo_e2e/best_model.zip"))
    ap.add_argument("--out", type=Path, default=Path("models/e2e_lidar.onnx"))
    ap.add_argument("--max-speed", type=float, required=True,
                    help="train_rl.py に渡したのと同じ値にすること")
    ap.add_argument("--max-steer", type=float, required=True,
                    help="train_rl.py に渡したのと同じ値にすること")
    args = ap.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    export(args.model, args.out, max_speed=args.max_speed, max_steer=args.max_steer)
    diff = verify_parity(args.model, args.out)
    print(f"# 完了 → {args.out}（PyTorch/ONNXRuntime 最大差 {diff:.2e}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
