"""ml/preview.py — 学習済みモデルの推論結果を、実車に乗せる前に目で確認する。

    python3 ml/preview.py ml/data/frames --model ml/runs/latest/model.onnx

**推論には `raspi/nodes/cam_perception_node.py` の `SegmentationModel` を
そのまま使う。** ここだけ別の前処理（リサイズ方式・正規化）で確認しても、
「実車に乗せたときにどう見えるか」の答えにならないため、実行経路を
1つに揃えている。

## 操作

    n / p    次/前のフレームへ
    q        終了

`<frame_stem>_mask.png`（`ml/annotate.py` の出力）が同じディレクトリに
あれば、正解マスクとの差分を色分け表示し、IoU も画面に出す
（緑=正解一致・青=過検出・赤=見落とし）。無ければ推論結果だけを
緑オーバーレイで表示する。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

from raspi.nodes.cam_perception_node import SegmentationModel, _resize_nearest  # noqa: E402

__all__ = ["load_model_config", "build_model", "compute_iou", "compare_masks"]


def load_model_config(onnx_path: Path) -> dict:
    """`<onnx_path>` と同名の `.json`（前処理契約）があれば読む。無ければ空。"""
    cfg_path = onnx_path.with_suffix(".json")
    if cfg_path.exists():
        return json.loads(cfg_path.read_text())
    return {}


def build_model(onnx_path: Path) -> SegmentationModel:
    """`cam_perception_node._load_model_by_name` と同じ読み方をする。"""
    cfg = load_model_config(onnx_path)
    w, h = cfg.get("input_size", [224, 224])
    return SegmentationModel(str(onnx_path), input_size=(int(w), int(h)),
                             mean=float(cfg.get("mean", 0.0)),
                             std=float(cfg.get("std", 255.0)),
                             threshold=float(cfg.get("threshold", 0.5)))


def compute_iou(pred: np.ndarray, gt: np.ndarray) -> float:
    """走行可能領域（True）のIoU。両方とも空なら「一致」扱い（`ml/train.py` の
    `iou_score` と同じ考え方）。"""
    inter = int(np.logical_and(pred, gt).sum())
    union = int(np.logical_or(pred, gt).sum())
    if union == 0:
        return 1.0
    return inter / union


def compare_masks(pred: np.ndarray, gt: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`(true_positive, false_positive, false_negative)` の3枚のbool配列。

    false_positive=モデルだけが走行可能と判定（過検出）、
    false_negative=正解にはあるがモデルが見落とした領域。
    """
    tp = pred & gt
    fp = pred & ~gt
    fn = gt & ~pred
    return tp, fp, fn


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames_dir", type=Path)
    ap.add_argument("--model", required=True, help="ONNXモデルのパス（同名 .json があれば前処理契約として使う）")
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        print("opencv-python が入っていません。`pip install -r ml/requirements.txt`",
              file=sys.stderr)
        return 2

    frame_paths = sorted(p for p in args.frames_dir.glob("*.jpg"))
    if not frame_paths:
        print("プレビュー対象のフレームが見つかりません", file=sys.stderr)
        return 1

    print(f"# モデルを読み込み中: {args.model}")
    model = build_model(Path(args.model))

    win = "preview  [n/p:次/前 q:終了]"
    cv2.namedWindow(win)

    idx = 0
    while 0 <= idx < len(frame_paths):
        path = frame_paths[idx]
        image_bgr = cv2.imread(str(path))
        if image_bgr is None:
            print(f"# 読み込めない: {path}", file=sys.stderr)
            idx += 1
            continue
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        h, w = image_rgb.shape[:2]
        mask = _resize_nearest(model.infer(image_rgb), w, h)

        label = f"{idx + 1}/{len(frame_paths)} {path.name}"
        disp = image_bgr.copy()
        overlay = disp.copy()

        gt_path = args.frames_dir / f"{path.stem}_mask.png"
        if gt_path.exists():
            gt = np.asarray(Image.open(gt_path).convert("L")) > 127
            tp, fp, fn = compare_masks(mask, gt)
            overlay[tp] = (0, 255, 0)
            overlay[fp] = (255, 0, 0)
            overlay[fn] = (0, 0, 255)
            label += f"  IoU={compute_iou(mask, gt):.3f}"
        else:
            overlay[mask] = (0, 255, 0)

        disp = cv2.addWeighted(disp, 0.5, overlay, 0.5, 0)
        cv2.putText(disp, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow(win, disp)

        advance = 1
        while True:
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q'):
                cv2.destroyAllWindows()
                return 0
            if key == ord('n'):
                advance = 1
                break
            if key == ord('p'):
                advance = -1
                break
        idx += advance

    cv2.destroyAllWindows()
    print("# 全フレーム終了")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
