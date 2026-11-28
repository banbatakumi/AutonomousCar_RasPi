"""矢印信号の YOLO を学習する（ultralytics。Mac 側）。

    .venv/bin/python ml_arrow_yolo/train.py --run ml_arrow_yolo/runs/v1 --epochs 100 --imgsz 320

`runs/<名前>/labels/` に**確定したラベルがある画像だけ**を使い（`dataset.py` の約束）、記録ごとに
学習と検証へ分けて `dataset/data.yaml` を書いてから ultralytics に渡す。結果は
`runs/<名前>/train/`、一番良い重みを `runs/<名前>/best.pt` に写す。

## 決めてある拡張

- **左右反転はしない（`fliplr=0`）。** 反転すると左の矢印が右の矢印の絵になり、ラベルと逆になる
- **色の拡張は弱める（`hsv_h=0.005`・`hsv_s=0.3`・`hsv_v=0.3`）。** 大会の看板は「薄い色」だけが有効で、
  濃い色は別の意味を持ちうる。色を大きく振ると薄い・濃いの区別が学べない
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dataset import RunPaths, list_images, read_classes, split_by_source, write_dataset_yaml  # noqa: E402

__all__ = ["TRAIN_AUG", "prepare", "main"]

#: ultralytics に渡す拡張の設定（モジュールの docstring 参照）
TRAIN_AUG = {"fliplr": 0.0, "flipud": 0.0, "hsv_h": 0.005, "hsv_s": 0.3, "hsv_v": 0.3}


def prepare(run_dir: Path, val_frac: float) -> tuple[Path, int, int]:
    """確定済みの画像を分けて `data.yaml` を書く。`(yaml, 学習の枚数, 検証の枚数)`。"""
    paths = RunPaths(run_dir)
    labeled = [p for p in list_images(paths) if paths.label_path(p).is_file()]
    if not labeled:
        raise SystemExit("確定したラベルが1枚もありません（②ラベル付けで確定してください）")
    train, val = split_by_source(labeled, val_frac)
    return write_dataset_yaml(paths, train, val, read_classes(paths)), len(train), len(val)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", type=Path, required=True)
    ap.add_argument("--model", default="yolo11n.pt", help="元にする重み（初回は自動でダウンロードされる）")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--val-frac", type=float, default=0.2)
    ap.add_argument("--device", default="", help="空なら自動（Mac は mps）。cpu を指定してもよい")
    ap.add_argument("--patience", type=int, default=30, help="この epoch 数よくならなければ打ち切る")
    args = ap.parse_args()

    yaml, n_train, n_val = prepare(args.run, args.val_frac)
    print(f"# 学習 {n_train}枚・検証 {n_val}枚（{yaml}）", flush=True)

    try:
        from ultralytics import YOLO
    except ModuleNotFoundError:
        print("ultralytics が入っていません。`pip install -r ml_arrow_yolo/requirements.txt` してください",
              flush=True)
        return 1

    model = YOLO(args.model)

    def on_epoch_end(trainer) -> None:
        m = trainer.metrics or {}
        # 学習が終わった後に best.pt で検証し直すときも呼ばれる（epoch が epochs を超える）
        head = (f"epoch {trainer.epoch + 1}/{trainer.epochs}" if trainer.epoch < trainer.epochs
                else "最終評価（best.pt）")
        print(f"{head}"
              f"  mAP50 {m.get('metrics/mAP50(B)', 0):.3f}"
              f"  mAP50-95 {m.get('metrics/mAP50-95(B)', 0):.3f}"
              f"  P {m.get('metrics/precision(B)', 0):.3f}  R {m.get('metrics/recall(B)', 0):.3f}",
              flush=True)

    model.add_callback("on_fit_epoch_end", on_epoch_end)
    kw = dict(data=str(yaml), epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
              project=str(args.run.resolve()), name="train", exist_ok=True,
              patience=args.patience, plots=True, **TRAIN_AUG)
    if args.device:
        kw["device"] = args.device
    model.train(**kw)

    best = args.run / "train" / "weights" / "best.pt"
    if not best.is_file():
        print(f"# best.pt ができていません（{best}）", flush=True)
        return 1
    shutil.copy2(best, RunPaths(args.run).best)
    print(f"# 重み → {RunPaths(args.run).best}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
