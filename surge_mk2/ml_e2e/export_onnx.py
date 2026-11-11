"""ml_e2e/export_onnx.py — 学習済みモデルを ONNX にし、Pi 側の契約（同名の `.json`）を書く。

    .venv/bin/python -m ml_e2e.export_onnx --run ml_e2e/runs/v1 --name v1
    # → models/e2e_fusion/v1.onnx と v1.json。実車へは tools/deploy.sh

契約は `<run>/train_config.json`（`train.py` が書く）からそのまま写す。**`config/vehicle.toml` の今の値では
埋めない**（学習後に同定で値が変わると、学習時と違う基準で物理量へ戻すことになる）。
`raspi/nodes/e2e_fusion_node.py` の `load_model()` がこの契約を照合し、合わないモデルは読まない。

**舵・速度の直接出力があった頃（入出力の版 1）に学習した重みも書き出せる。** 入力と経路点の並びは同じで、
直接出力のヘッドを捨てるだけ（`model.load_weights`）。契約の版は今の版で書く。

書き出した後、PyTorch と ONNXRuntime の出力を突き合わせ、さらに**実車と同じ `load_model()` で読めること**を確かめる。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT))

from ml_e2e.model import FusionModel, load_weights  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_CROP  # noqa: E402
from raspi.core.e2e_fusion_preproc import INPUT_NAMES, PREPROC_VERSION  # noqa: E402

__all__ = ["CONTRACT_KEYS", "build_model", "export", "verify_parity", "main"]

#: `train_config.json` から契約へ写すキー
CONTRACT_KEYS = ("preproc_version", "image_preproc_version", "input_size", "crop_x", "fov_deg", "max_range",
                 "history", "k", "ds", "xy_scale", "max_steer", "speed_ref", "yaw_rate_ref", "steer_link",
                 "maps", "train_records")
#: 学習時の入出力の版のうち、今の版として書き出せるもの（1 = 直接出力のヘッドつき。入力と経路点は同じ）
_EXPORTABLE_VERSIONS = (1, PREPROC_VERSION)
DEFAULT_MODELS_DIR = _ROOT / "models" / "e2e_fusion"


def build_model(cfg: dict) -> FusionModel:
    return FusionModel(lidar_ch=int(cfg["history"]) + 1, lidar_n=int(cfg["fov_deg"]) + 1, k=int(cfg["k"]),
                       xy_scale=float(cfg["xy_scale"]), pretrained=False)


def _dummy(cfg: dict, rng: np.random.Generator | None = None) -> tuple[np.ndarray, ...]:
    w, h = cfg["input_size"]
    shapes = ((1, 3, h, w), (1, int(cfg["history"]) + 1, int(cfg["fov_deg"]) + 1), (1, 3))
    if rng is None:
        return (*(np.zeros(s, dtype=np.float32) for s in shapes), np.ones((1, 3), dtype=np.float32))
    avail = (rng.random((1, 3)) > 0.3).astype(np.float32)
    return (*(rng.random(s, dtype=np.float32) for s in shapes), avail)


def export(run_dir: Path, out_path: Path, *, checkpoint: str = "best.pt", note: str = "") -> dict:
    """`run_dir` の重みを `out_path` へ書き、同名の `.json` に契約を書く。契約の dict を返す。"""
    cfg = json.loads((run_dir / "train_config.json").read_text())
    if cfg.get("preproc_version") not in _EXPORTABLE_VERSIONS:
        raise ValueError(f"学習時の入出力の版 {cfg.get('preproc_version')} が今の版 {PREPROC_VERSION} と違う。"
                         "学習し直してください")
    cfg["preproc_version"] = PREPROC_VERSION
    if cfg.get("image_preproc_version") != PREPROC_VERSION_CROP:
        raise ValueError("学習時の画像の前処理の版が今と違う。学習し直してください")
    missing = [key for key in CONTRACT_KEYS if key not in cfg]
    if missing:
        raise ValueError(f"train_config.json に {missing} が無い")

    model = build_model(cfg)
    load_weights(model, run_dir / checkpoint)
    model.eval()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        model, tuple(torch.from_numpy(a) for a in _dummy(cfg)), str(out_path),
        input_names=list(INPUT_NAMES), output_names=["output"],
        opset_version=18,          # `ml_cam/export_onnx.py` と同じ理由（18未満は失敗する）
        external_data=False,       # 重みを別ファイルに切り出させない
    )
    contract = {key: cfg[key] for key in CONTRACT_KEYS}
    contract["inputs"] = list(INPUT_NAMES)
    contract["best"] = cfg.get("best", {})
    contract["note"] = note
    out_path.with_suffix(".json").write_text(json.dumps(contract, ensure_ascii=False, indent=2), encoding="utf-8")
    return contract


def verify_parity(onnx_path: Path, model: FusionModel, cfg: dict, *, n_samples: int = 4,
                  atol: float = 1e-3, seed: int = 0) -> float:
    """PyTorch と ONNXRuntime の出力差の最大値。`atol` を超えたら例外。入力を伏せた組も混ぜる。"""
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    model.eval()
    rng = np.random.default_rng(seed)
    max_err = 0.0
    with torch.no_grad():
        for _ in range(n_samples):
            arrs = _dummy(cfg, rng)
            ref = model(*(torch.from_numpy(a) for a in arrs)).numpy()
            got = session.run(None, dict(zip(INPUT_NAMES, arrs)))[0]
            max_err = max(max_err, float(np.abs(ref - got).max()))
    if max_err > atol:
        raise ValueError(f"PyTorch と ONNXRuntime の出力が最大 {max_err:.2e} ずれています（許容 {atol}）")
    return max_err


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True, help="`train.py` の出力フォルダ")
    ap.add_argument("--name", required=True, help="モデルの名前（models/e2e_fusion/<名前>.onnx）")
    ap.add_argument("--checkpoint", default="best.pt")
    ap.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    args = ap.parse_args()

    out = args.models_dir / f"{args.name}.onnx"
    note_path = args.run / "note.txt"
    note = note_path.read_text(encoding="utf-8") if note_path.exists() else ""
    try:
        cfg = export(args.run, out, checkpoint=args.checkpoint, note=note)
        model = build_model(cfg)
        load_weights(model, args.run / args.checkpoint)
        err = verify_parity(out, model, cfg)
        from raspi.nodes.e2e_fusion_node import load_model
        load_model(out)                    # 実車と同じ読み込みが通るか
    except (ValueError, FileNotFoundError) as e:
        print(str(e), file=sys.stderr)
        return 2
    print(f"# 書き出し完了: {out}（PyTorch比 最大誤差 {err:.2e}）")
    print(f"# 契約: {out.with_suffix('.json')}")
    print("# 実車へは tools/deploy.sh で運び、実機 GUI の自動運転タブ「E2E 融合」のモデル欄で選ぶ"
          "（「自動」ならいちばん新しい .onnx）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
