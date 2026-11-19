"""ml_cam/export_onnx.py — 学習済みモデルを ONNX にエクスポートし、Pi側の前処理契約を固定する。

    python3 ml_cam/export_onnx.py --checkpoint ml_cam/runs/latest/best.pt \\
        --out ml_cam/runs/latest/model.onnx

**前処理定数（入力解像度・平均/分散・閾値）を `model.json` に書き出す。**
`raspi/nodes/cam_perception_node.py` の `SegmentationModel` がこれと違う値で
前処理すると、学習時と推論時で入力の意味がズレる（train/inference skew）。
値の受け渡しを人手のコピペに頼らないための唯一の場所にする。

エクスポート後、**PyTorch と ONNXRuntime の出力を突き合わせて検証する。**
エクスポートが成功しただけでは何も保証されない——opset や演算子の対応漏れで
静かに違う結果を返すことがあるため、必ず数値で確認する。書き出しの設定・突き合わせ・版の検査は
`ml_common/onnx_export.py`（4本の `export_onnx.py` で共通）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

import numpy as np  # noqa: E402
import torch  # noqa: E402

from ml_common import onnx_export  # noqa: E402
from ml_common.onnx_export import read_train_config  # noqa: E402
from model import DrivableSegModel  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION  # noqa: E402

__all__ = ["export", "verify_parity", "MEAN", "STD", "THRESHOLD"]

#: `ml_cam/dataset.py` の正規化（0-1 = mean 0 / std 255）と
#: `raspi/nodes/cam_perception_node.py` の `SegmentationModel` の既定値に揃えてある。
#: **ここを変えたら両方直すこと。**
MEAN = 0.0
STD = 255.0
THRESHOLD = 0.5


def export(checkpoint: Path, out_path: Path, size: tuple[int, int] | None = None, *,
           note: str = "") -> tuple[int, int]:
    """`checkpoint`（`state_dict`）を `out_path` へエクスポートし、同名 `.json` に
    前処理契約を書く。

    `size` を省くと `train_config.json`（学習時の入力解像度）を使う。**学習時と違う値を渡すと
    例外**にする——ONNX の変換は同じサイズどうしで突き合わせるので、解像度の食い違いは
    `verify_parity` では見つからず、精度だけが静かに落ちる。学習時の前処理の版
    （`preproc_version`）が今と違う・記録が無い場合も例外にする（`onnx_export.check_version`。
    以前は版の無い古い run を警告だけで通していたが、4本の書き出しで扱いを揃えた）。
    :return: 実際に使った入力解像度 `(width, height)`

    :param note: `<out_path>.json`に同梱する自由記述の備考（どんな変更をしたか・
        どのコースか等）。GUIのモデル選択に表示される（`ml_lidar/export_onnx_rl.py`
        の`note`と対称。`--checkpoint`と同じディレクトリの
        `note.txt`から`main()`が拾う）。
    """
    train_cfg = read_train_config(checkpoint.parent)
    trained = train_cfg.get("input_size")
    if size is None:
        if trained is None:
            raise ValueError(f"{checkpoint.parent}/train_config.json に input_size が無い。"
                             "--size で指定してください")
        size = (int(trained[0]), int(trained[1]))
    elif trained is not None and tuple(int(v) for v in trained) != tuple(size):
        raise ValueError(f"--size {size[0]}x{size[1]} が学習時の入力解像度 "
                         f"{int(trained[0])}x{int(trained[1])} と違う（精度が静かに落ちる）")
    onnx_export.check_version("前処理", train_cfg.get("preproc_version"), PREPROC_VERSION)
    w, h = size
    model = DrivableSegModel(pretrained=False)
    model.load_state_dict(torch.load(checkpoint, map_location="cpu"))
    model.eval()

    onnx_export.export(model, np.zeros((1, 3, h, w), dtype=np.float32), out_path)

    cfg = {
        "input_size": [w, h],           # [width, height]
        "input_layout": "NCHW",
        "resize": "area",               # 学習は `ml_cam/dataset.py` の `resize_area`（面積平均）
        "preproc_version": PREPROC_VERSION,
        "mean": MEAN,
        "std": STD,
        "threshold": THRESHOLD,
        "output": "probability (sigmoid, 0-1); >= threshold means drivable",
        "note": note,
    }
    onnx_export.write_contract(out_path, cfg)
    return size


def verify_parity(onnx_path: Path, model, size: tuple[int, int], *,
                  n_samples: int = 4, atol: float = 1e-3, seed: int = 0) -> float:
    """PyTorch と ONNXRuntime の出力差の最大値を返す。`atol` を超えたら例外（`ml_common.onnx_export`）。"""
    w, h = size
    return onnx_export.verify_parity(onnx_path, model, lambda rng: (rng.random((1, 3, h, w), dtype=np.float32),),
                                     n_samples=n_samples, atol=atol, seed=seed)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--size", default=None,
                    help="入力解像度 WxH。省くと学習時の値（train_config.json）。違う値は拒否")
    ap.add_argument("--out", type=Path, default=Path("ml_cam/runs/latest/model.onnx"))
    args = ap.parse_args()

    size = tuple(int(v) for v in args.size.lower().split("x")) if args.size else None
    args.out.parent.mkdir(parents=True, exist_ok=True)

    # ★--checkpointと同じディレクトリのnote.txt（ml_cam/app.pyの備考欄が書く）があれば
    # そのままcfgの"note"へ運ぶ（ml_lidar/export_onnx_rl.pyと同じ流儀）
    note_path = args.checkpoint.parent / "note.txt"
    note = note_path.read_text(encoding="utf-8") if note_path.exists() else ""

    w, h = export(args.checkpoint, args.out, size, note=note)

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
