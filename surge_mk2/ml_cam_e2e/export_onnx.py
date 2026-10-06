"""ml_cam_e2e/export_onnx.py — 学習済み回帰モデルを ONNX にエクスポートし、Pi側の契約を固定する。

    python3 ml_cam_e2e/export_onnx.py --checkpoint ml_cam_e2e/runs/v1/best.pt \\
        --out models/cam_e2e/v1.onnx

`ml_cam/export_onnx.py` と同じ構成（opset 18・`external_data=False`・
PyTorch/ONNXRuntime往復検証）。**前処理契約（入力解像度・色順・縮小方式・
平均/分散）と出力契約（`max_steer`・`speed_ref`・出力の並び）を `<out>.json`
に書く**——`raspi/nodes/cam_e2e_node.py` の `load_model()` がこれを照合し、
合わないモデルは読み込まない。

解像度と正規化の基準は、チェックポイントと同じ場所の `train_config.json`
（`train.py` が書く）から取る。学習時の値をそのまま使うので、学習タブと
エクスポートタブで同じ値を2回入力させない。
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
from raspi.core.cam_e2e_preproc import PREPROC_VERSION  # noqa: E402
from raspi.nodes.cam_e2e_node import MODEL_OUTPUTS  # noqa: E402

__all__ = ["export", "verify_parity", "read_train_config", "MEAN", "STD"]

#: `ml_cam_e2e/dataset.py` の正規化（0-1 = mean 0 / std 255）と揃えてある。
#: **変えたら両方直すこと。**
MEAN = 0.0
STD = 255.0


def read_train_config(run_dir: Path) -> dict:
    """`run_dir/train_config.json`。無い・読めないなら空の dict。"""
    cfg_path = run_dir / "train_config.json"
    if not cfg_path.exists():
        return {}
    try:
        return json.loads(cfg_path.read_text())
    except (json.JSONDecodeError, OSError):
        return {}


def export(checkpoint: Path, out_path: Path, size: tuple[int, int] | None = None, *,
          max_steer: float | None = None, speed_ref: float | None = None,
          note: str = "") -> tuple[int, int]:
    """`checkpoint`（`state_dict`）を `out_path` へエクスポートし、同名 `.json` に契約を書く。

    `size`・`max_steer`・`speed_ref` を省略すると `train_config.json` の値を使う。
    **どちらにも無ければ例外にする**——以前は `config/vehicle.toml` の現在値へ
    落としていたが、学習後に同定で値が変わると学習時と違う基準で舵角へ戻す
    ことになる（train/inference skew の出力版）。黙って違う値を書くより止める。

    :return: 実際に使った入力解像度 `(width, height)`
    :param note: `<out_path>.json`に同梱する自由記述の備考（`ml_cam/export_onnx.py`と対称）
    """
    cfg = read_train_config(checkpoint.parent)
    if size is None:
        if "input_size" not in cfg:
            raise ValueError(f"{checkpoint.parent}/train_config.json に input_size が無い。"
                             f"--size で指定してください")
        size = (int(cfg["input_size"][0]), int(cfg["input_size"][1]))
    if max_steer is None:
        max_steer = cfg.get("max_steer")
    if speed_ref is None:
        speed_ref = cfg.get("speed_ref")
    if not max_steer or not speed_ref or max_steer <= 0 or speed_ref <= 0:
        raise ValueError(f"max_steer/speed_ref が決まらない（{max_steer}/{speed_ref}）。"
                         f"{checkpoint.parent}/train_config.json が無ければ "
                         f"--max-steer と --speed-ref で指定してください")

    w, h = size
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

    out_path.with_suffix(".json").write_text(json.dumps({
        "input_size": [w, h],           # [width, height]
        "input_layout": "NCHW",
        "color": "RGB",
        "resize": "area",
        "preproc_version": PREPROC_VERSION,
        "mean": MEAN,
        "std": STD,
        "outputs": list(MODEL_OUTPUTS),
        "max_steer": float(max_steer),  # [rad] steer = steer_norm * max_steer
        "speed_ref": float(speed_ref),  # [m/s] speed = speed_norm * speed_ref
        "note": note,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    return size


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
    ap.add_argument("--out", type=Path, default=Path("ml_cam_e2e/runs/latest/model.onnx"))
    ap.add_argument("--size", default=None,
                    help="入力解像度 幅x高さ。省略時は train_config.json の値（通常は省略）")
    ap.add_argument("--max-steer", type=float, default=None,
                    help="操舵の正規化基準[rad]。省略時は train_config.json の値")
    ap.add_argument("--speed-ref", type=float, default=None,
                    help="速度の正規化基準[m/s]。省略時は train_config.json の値")
    args = ap.parse_args()

    size = tuple(int(v) for v in args.size.lower().split("x")) if args.size else None
    args.out.parent.mkdir(parents=True, exist_ok=True)

    note_path = args.checkpoint.parent / "note.txt"
    note = note_path.read_text(encoding="utf-8") if note_path.exists() else ""

    try:
        size = export(args.checkpoint, args.out, size, max_steer=args.max_steer,
                      speed_ref=args.speed_ref, note=note)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2

    model = DriveRegressionModel(pretrained=False)
    model.load_state_dict(torch.load(args.checkpoint, map_location="cpu"))
    err = verify_parity(args.out, model, size)
    print(f"# 書き出し完了: {args.out}（PyTorch比 最大誤差 {err:.2e}）")
    print(f"# 契約: {args.out.with_suffix('.json')}")
    print("# 実車へは tools/deploy.sh で運び、GUI の自動運転タブでこのモデルを選ぶ")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
