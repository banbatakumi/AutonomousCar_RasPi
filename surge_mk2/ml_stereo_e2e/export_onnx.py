"""ml_stereo_e2e/export_onnx.py — 学習済みモデルを ONNX にし、Pi 側の契約（同名の `.json`）を書く。

    .venv/bin/python -m ml_stereo_e2e.export_onnx --run ml_stereo_e2e/runs/v1 --name v1
    # → models/stereo_e2e/v1.onnx と v1.json。実車へは tools/deploy.sh

契約は `<run>/train_config.json`（`train.py` が書く）からそのまま写す（`ml_e2e/export_onnx.py` と同じ理由で、
今の `vehicle.toml` では埋めない）。`raspi/nodes/stereo_e2e_node.py` の `load_model()` がこの契約を照合し、合わないモデルは
読まない。書き出した後、PyTorch と ONNXRuntime の出力を突き合わせ、実車と同じ `load_model()` で読めることを確かめる。

書き出しの手順（版の検査・契約の写し方）は `ml_e2e.export_onnx.export_run` と共通で、ここは2番目の入力（視差の窓）に
関わる差分（契約のキー・入力の名前・モデルの組み立て・形を決める入力）だけを渡す。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import torch

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from ml_common import onnx_export  # noqa: E402
from ml_e2e.export_onnx import export_run  # noqa: E402
from ml_stereo_e2e.model import StereoE2EModel  # noqa: E402
from raspi.core.stereo_e2e_preproc import INPUT_NAMES, PREPROC_VERSION  # noqa: E402

__all__ = ["CONTRACT_KEYS", "build_model", "export", "verify_parity", "main", "DEFAULT_MODELS_DIR"]

#: `train_config.json` から契約へ写すキー
CONTRACT_KEYS = ("preproc_version", "image_preproc_version", "input_size", "crop_x",
                 "disp_hw", "disp_num_disp", "disp_rows", "disp_scale", "disp_version", "disp_algo", "disp_block",
                 "disp_uniqueness", "disp_crop_cols", "disp_col_f", "disp_col_cx", "k", "ds", "xy_scale", "max_steer", "speed_ref", "yaw_rate_ref", "steer_link",
                 "maps", "train_records")
DEFAULT_MODELS_DIR = _ROOT / "models" / "stereo_e2e"


def disp_input_hw(cfg: dict) -> tuple[int, int]:
    """モデルに入る視差の窓の大きさ（帯 `disp_hw` の両端 `disp_crop_cols` 列を落とした大きさ）。"""
    dh, dw = cfg["disp_hw"]
    return int(dh), int(dw) - 2 * int(cfg["disp_crop_cols"])


def build_model(cfg: dict) -> StereoE2EModel:
    return StereoE2EModel(disp_hw=disp_input_hw(cfg), k=int(cfg["k"]), xy_scale=float(cfg["xy_scale"]),
                          pretrained=False)


def _dummy(cfg: dict, rng: np.random.Generator | None = None) -> tuple[np.ndarray, ...]:
    w, h = cfg["input_size"]
    dh, dw = disp_input_hw(cfg)
    shapes = ((1, 6, h, w), (1, 2, dh, dw), (1, 3))
    if rng is None:
        return (*(np.zeros(s, dtype=np.float32) for s in shapes), np.ones((1, 3), dtype=np.float32))
    avail = (rng.random((1, 3)) > 0.3).astype(np.float32)
    return (*(rng.random(s, dtype=np.float32) for s in shapes), avail)


def _load(model: StereoE2EModel, path: Path) -> None:
    model.load_state_dict(torch.load(path, map_location="cpu"))


def export(run_dir: Path, out_path: Path, *, checkpoint: str = "best.pt", note: str = "") -> dict:
    """`run_dir` の重みを `out_path` へ書き、同名の `.json` に契約を書く。契約の dict を返す。"""
    return export_run(run_dir, out_path, preproc_version=PREPROC_VERSION, contract_keys=CONTRACT_KEYS,
                      input_names=INPUT_NAMES, build=build_model, load=_load, dummy=_dummy,
                      checkpoint=checkpoint, note=note)


def verify_parity(onnx_path: Path, model: StereoE2EModel, cfg: dict, *, n_samples: int = 4,
                  atol: float = 1e-3, seed: int = 0) -> float:
    """PyTorch と ONNXRuntime の出力差の最大値。`atol` を超えたら例外。入力を伏せた組も混ぜる（`_dummy`）。"""
    return onnx_export.verify_parity(onnx_path, model, lambda rng: _dummy(cfg, rng), input_names=INPUT_NAMES,
                                     n_samples=n_samples, atol=atol, seed=seed)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True, help="`train.py` の出力フォルダ")
    ap.add_argument("--name", required=True, help="モデルの名前（models/stereo_e2e/<名前>.onnx）")
    ap.add_argument("--checkpoint", default="best.pt")
    ap.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    args = ap.parse_args()

    out = args.models_dir / f"{args.name}.onnx"
    note_path = args.run / "note.txt"
    note = note_path.read_text(encoding="utf-8") if note_path.exists() else ""
    try:
        cfg = export(args.run, out, checkpoint=args.checkpoint, note=note)
        model = build_model(cfg)
        _load(model, args.run / args.checkpoint)
        err = verify_parity(out, model, cfg)
        from raspi.nodes.stereo_e2e_node import load_model
        load_model(out)                    # 実車と同じ読み込みが通るか
    except (ValueError, FileNotFoundError) as e:
        print(str(e), file=sys.stderr)
        return 2
    print(f"# 書き出し完了: {out}（PyTorch比 最大誤差 {err:.2e}）")
    print(f"# 契約: {out.with_suffix('.json')}")
    print("# 実車へは tools/deploy.sh で運び、実機 GUI の自動運転タブ「ステレオ E2E」のモデル欄で選ぶ"
          "（「自動」ならいちばん新しい .onnx）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
