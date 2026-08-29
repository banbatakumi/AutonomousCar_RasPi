"""ml/export_onnx.py — 学習済みモデルを ONNX にエクスポートし、Pi側の前処理契約を固定する。

    python3 ml/export_onnx.py --checkpoint ml/runs/latest/best.pt \\
        --size 224x224 --out ml/runs/latest/model.onnx

**前処理定数（入力解像度・平均/分散・閾値）を `model.json` に書き出す。**
`raspi/nodes/cam_perception_node.py` の `SegmentationModel` がこれと違う値で
前処理すると、学習時と推論時で入力の意味がズレる（train/inference skew）。
値の受け渡しを人手のコピペに頼らないための唯一の場所にする。

エクスポート後、**PyTorch と ONNXRuntime の出力を突き合わせて検証する。**
エクスポートが成功しただけでは何も保証されない——opset や演算子の対応漏れで
静かに違う結果を返すことがあるため、必ず数値で確認する。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import torch  # noqa: E402

from model import DrivableSegModel  # noqa: E402

__all__ = ["export", "verify_parity", "MEAN", "STD", "THRESHOLD"]

#: `ml/dataset.py` の正規化（0-1 = mean 0 / std 255）と
#: `raspi/nodes/cam_perception_node.py` の `SegmentationModel` の既定値に揃えてある。
#: **ここを変えたら両方直すこと。**
MEAN = 0.0
STD = 255.0
THRESHOLD = 0.5


def export(checkpoint: Path, out_path: Path, size: tuple[int, int]) -> None:
    """`checkpoint`（`state_dict`）を `out_path` へエクスポートし、同名 `.json` に
    前処理契約を書く。"""
    w, h = size
    model = DrivableSegModel(pretrained=False)
    model.load_state_dict(torch.load(checkpoint, map_location="cpu"))
    model.eval()

    dummy = torch.zeros(1, 3, h, w)
    torch.onnx.export(
        model, dummy, str(out_path),
        input_names=["input"], output_names=["output"],
        # **18 未満を指定すると失敗する。** torch の dynamo エクスポータ
        # （既定）は `interpolate` を新しい opset の演算子表現に変換してから
        # 下位変換を試みるが、対応できず盛大な警告を出したうえで結局 18 に
        # フォールバックする（実測）。最初から 18 を指定して無駄な変換の
        # 試行錯誤を避ける。**onnxruntime>=1.18 は opset 18 を扱える。**
        opset_version=18,
        # **`external_data=False`が必須。** 既定(True)だと重みが`<out_path>.data`
        # という別ファイルに切り出され、`.onnx`単体をコピー/配置すると壊れる
        # （`ml_lidar/export_onnx_rl.py`が同じ罠を実際に踏んで判明。Pi の
        # `models/`へは`.onnx`だけを配置する運用なので、ここが漏れると
        # cam_perception_node のロードが実機でだけ失敗する）
        external_data=False,
    )

    cfg = {
        "input_size": [w, h],           # [width, height]
        "input_layout": "NCHW",
        "mean": MEAN,
        "std": STD,
        "threshold": THRESHOLD,
        "output": "probability (sigmoid, 0-1); >= threshold means drivable",
    }
    out_path.with_suffix(".json").write_text(json.dumps(cfg, indent=2))


def verify_parity(onnx_path: Path, model, size: tuple[int, int], *,
                  n_samples: int = 4, atol: float = 1e-3, seed: int = 0) -> float:
    """PyTorch と ONNXRuntime の出力差の最大値を返す。`atol` を超えたら例外。"""
    import onnxruntime as ort

    w, h = size
    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    model.eval()
    rng = np.random.default_rng(seed)
    max_err = 0.0
    with torch.no_grad():
        for _ in range(n_samples):
            x_np = rng.random((1, 3, h, w), dtype=np.float32)
            torch_out = model(torch.from_numpy(x_np)).numpy()
            onnx_out = session.run(None, {"input": x_np})[0]
            max_err = max(max_err, float(np.abs(torch_out - onnx_out).max()))
    if max_err > atol:
        raise ValueError(
            f"PyTorch と ONNXRuntime の出力が最大 {max_err:.2e} ずれています"
            f"（許容 {atol}）。opset やモデル構造（補間の実装差など）を見直してください")
    return max_err


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--size", default="224x224")
    ap.add_argument("--out", type=Path, default=Path("ml/runs/latest/model.onnx"))
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    export(args.checkpoint, args.out, (w, h))

    model = DrivableSegModel(pretrained=False)
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    err = verify_parity(args.out, model, (w, h))
    print(f"# 書き出し完了: {args.out}（PyTorch比 最大誤差 {err:.2e}）")
    print(f"# 前処理契約: {args.out.with_suffix('.json')}")
    print(f"# Pi 側: cam_perception_node.py --model {args.out.name} "
          f"--input-size {w}x{h}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
