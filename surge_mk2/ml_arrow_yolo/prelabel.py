"""前に作った ONNX で、まだラベルの無い画像に下書きの枠を付ける（矢印信号の YOLO②の手間を減らす）。

    .venv/bin/python ml_arrow_yolo/prelabel.py --run ml_arrow_yolo/runs/v2 --model models/arrow_yolo/v1.onnx

下書きは `runs/<名前>/drafts/` に書き、**確定済みのラベル（`labels/`）には触らない**。②ラベル付けで
下書きを見て、直してから確定する（確定するまで学習には使わない。`dataset.py` の約束）。
推論は車と同じ読み口（`raspi/core/arrow_yolo.py`、onnxruntime）で、ultralytics は要らない。

下書きのクラス番号は、モデルのクラス名を今のモデル名の `classes.txt` に名前で突き合わせて付け直す
（名前が無いクラスの検出は捨てる）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ML_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_DIR.parent
sys.path.insert(0, str(ML_DIR))
sys.path.insert(0, str(REPO_ROOT))

from dataset import Box, RunPaths, list_images, read_classes, write_labels  # noqa: E402

__all__ = ["prelabel", "main"]


def prelabel(run_dir: Path, model_path: Path, *, conf: float = 0.3, overwrite: bool = False,
             detector=None) -> tuple[int, int]:
    """`(下書きを書いた枚数, 検出が無く空の下書きにした枚数)`。`detector` はテストが差し込む口。"""
    import cv2

    from raspi.core.arrow_yolo import YoloArrowDetector

    paths = RunPaths(run_dir)
    classes = read_classes(paths)
    if detector is None:
        detector = YoloArrowDetector(conf_min=conf, min_period_s=0.0)
        if not detector.load(model_path):
            raise SystemExit(f"モデルが読めません: {detector.error}")
    remap = {i: classes.index(n) for i, n in enumerate(detector.names) if n in classes}
    written = empty = 0
    for img in list_images(paths):
        if paths.label_path(img).is_file():
            continue                                  # 確定済みには触らない
        if paths.draft_path(img).is_file() and not overwrite:
            continue
        frame = cv2.imread(str(img), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        boxes = [Box(remap[c], *box) for box, _score, c in detector.detect(frame) if c in remap]
        write_labels(paths.draft_path(img), boxes)
        written += 1
        empty += not boxes
    return written, empty


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--model", type=Path, required=True, help="models/arrow_yolo/<名前>.onnx")
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--overwrite", action="store_true", help="既にある下書きも付け直す")
    args = ap.parse_args()
    n, empty = prelabel(args.run, args.model, conf=args.conf, overwrite=args.overwrite)
    print(f"# 下書き {n}枚（うち検出なし {empty}枚）→ {RunPaths(args.run).drafts}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
