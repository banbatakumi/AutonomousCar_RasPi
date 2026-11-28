"""学習した YOLO（`best.pt`）を、車が読む ONNX に書き出す（矢印信号の YOLO④）。

    .venv/bin/python ml_arrow_yolo/export_onnx.py --run ml_arrow_yolo/runs/v1 --out models/arrow_yolo/v1.onnx

書き出すのは ONNX 本体と同名の `.json`（`names`・`imgsz`・`note`）。車側
（`raspi/core/arrow_yolo.py`）はクラス名を `.json` → ONNX のメタデータの順に読み、
GUI のモデル一覧は `.json` の `note` を備考として出す。

書き出した後、車と同じ読み口（`YoloArrowDetector`）で読み直して1枚推論し、読めることを確かめる。
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

ML_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_DIR.parent
sys.path.insert(0, str(ML_DIR))
sys.path.insert(0, str(REPO_ROOT))

__all__ = ["write_sidecar", "verify", "main"]


def write_sidecar(onnx_path: Path, names: list[str], imgsz: int, note: str) -> Path:
    p = onnx_path.with_suffix(".json")
    p.write_text(json.dumps({"names": names, "imgsz": imgsz, "note": note}, ensure_ascii=False,
                            indent=2) + "\n", encoding="utf-8")
    return p


def verify(onnx_path: Path) -> str:
    """車と同じ読み口で読み、黒い画像を1枚推論する。読めなければ例外。結果の説明を返す。"""
    import numpy as np

    from raspi.core.arrow_yolo import YoloArrowDetector

    det = YoloArrowDetector(min_period_s=0.0)
    if not det.load(onnx_path):
        raise RuntimeError(det.error)
    det.detect(np.zeros((480, 640, 3), np.uint8))
    return f"names={det.names} imgsz={det.imgsz} 推論 {det.last_infer_ms:.1f}ms（この計算機）"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True, help="models/arrow_yolo/<名前>.onnx")
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--opset", type=int, default=12)
    args = ap.parse_args()

    from dataset import RunPaths

    paths = RunPaths(args.run)
    if not paths.best.is_file():
        print(f"学習済みの重みがありません: {paths.best}（③学習を先に）", flush=True)
        return 1
    try:
        from ultralytics import YOLO
    except ModuleNotFoundError:
        print("ultralytics が入っていません。`pip install -r ml_arrow_yolo/requirements.txt` してください",
              flush=True)
        return 1

    model = YOLO(str(paths.best))
    exported = Path(model.export(format="onnx", imgsz=args.imgsz, opset=args.opset,
                                 dynamic=False, simplify=False))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(exported, args.out)
    names = [model.names[k] for k in sorted(model.names)]
    note_path = paths.run_dir / "note.txt"
    note = note_path.read_text(encoding="utf-8").strip() if note_path.is_file() else ""
    sidecar = write_sidecar(args.out, names, args.imgsz, note)
    print(f"# ONNX → {args.out}\n# 設定 → {sidecar}", flush=True)
    try:
        print(f"# 読み直し OK: {verify(args.out)}", flush=True)
    except Exception as e:  # noqa: BLE001
        print(f"# 読み直しに失敗: {e}", flush=True)
        return 1
    print("# Pi の models/arrow_yolo/ に .onnx と .json を置き、GUI の「矢印信号」→「YOLO」で選ぶ", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
