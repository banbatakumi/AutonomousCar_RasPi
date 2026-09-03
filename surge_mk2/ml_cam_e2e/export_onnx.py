"""ml_cam_e2e/export_onnx.py — 学習済み回帰モデルを ONNX にエクスポートし、Pi側の契約を固定する。

    python3 ml_cam_e2e/export_onnx.py --checkpoint ml_cam_e2e/runs/v1/best.pt \\
        --size 224x224 --out models/v1.onnx

`ml_cam/export_onnx.py` と同じ構成（opset 18・`external_data=False`・
PyTorch/ONNXRuntime往復検証）。**前処理契約（入力解像度・平均/分散）に加えて
出力契約（`max_steer`）も `<out>.json` に書く**——`raspi/nodes/cam_e2e_node.py`
がこの値で正規化出力を実際の舵角[rad]へ戻す。ここがズレるとモデルの
学習時の意味と実車での解釈がズレる（train/inference skew の出力版）。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import numpy as np  # noqa: E402
import torch  # noqa: E402

from model import DriveRegressionModel  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402

__all__ = ["export", "verify_parity", "MEAN", "STD"]

#: `ml_cam_e2e/dataset.py` の正規化（0-1 = mean 0 / std 255）と
#: `raspi/nodes/cam_e2e_node.py` の既定値に揃えてある。**変えたら両方直すこと。**
MEAN = 0.0
STD = 255.0


def export(checkpoint: Path, out_path: Path, size: tuple[int, int], *,
          max_steer: float | None = None, note: str = "") -> None:
    """`checkpoint`（`state_dict`）を `out_path` へエクスポートし、同名 `.json` に契約を書く。

    :param max_steer: 出力の正規化基準 [rad]。省略時は `config/vehicle.toml` を読む
        （学習時と車両が変わっていなければ通常これで一致する）
    :param note: `<out_path>.json`に同梱する自由記述の備考（`ml_cam/export_onnx.py`と対称）
    """
    w, h = size
    max_steer = max_steer if max_steer is not None else Vehicle.load().max_steer
    model = DriveRegressionModel(pretrained=False)
    model.load_state_dict(torch.load(checkpoint, map_location="cpu"))
    model.eval()

    dummy = torch.zeros(1, 3, h, w)
    torch.onnx.export(
        model, dummy, str(out_path),
        input_names=["input"], output_names=["output"],
        opset_version=18,          # `ml_cam/export_onnx.py` と同じ理由（18未満は失敗する）
        external_data=False,       # 同上。重みを別ファイルに切り出させない
    )

    cfg = {
        "input_size": [w, h],           # [width, height]
        "input_layout": "NCHW",
        "mean": MEAN,
        "std": STD,
        "max_steer": max_steer,
        "output": "steer_norm (tanh, -1..1); steer_rad = steer_norm * max_steer",
        "note": note,
    }
    out_path.with_suffix(".json").write_text(json.dumps(cfg, indent=2))


def verify_parity(onnx_path: Path, model, size: tuple[int, int], *,
                  n_samples: int = 4, atol: float = 1e-3, seed: int = 0) -> float:
    """PyTorch と ONNXRuntime の出力差の最大値を返す。`atol` を超えたら例外
    （`ml_cam/export_onnx.py` の `verify_parity()` と同じ）。"""
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
            f"（許容 {atol}）")
    return max_err


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--size", default="224x224")
    ap.add_argument("--out", type=Path, default=Path("ml_cam_e2e/runs/latest/model.onnx"))
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    args.out.parent.mkdir(parents=True, exist_ok=True)

    note_path = args.checkpoint.parent / "note.txt"
    note = note_path.read_text(encoding="utf-8") if note_path.exists() else ""

    export(args.checkpoint, args.out, (w, h), note=note)

    model = DriveRegressionModel(pretrained=False)
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    err = verify_parity(args.out, model, (w, h))
    print(f"# 書き出し完了: {args.out}（PyTorch比 最大誤差 {err:.2e}）")
    print(f"# 契約: {args.out.with_suffix('.json')}")
    print(f"# Pi 側: raspi/nodes/cam_e2e_node.py --model {args.out.name} --input-size {w}x{h}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
