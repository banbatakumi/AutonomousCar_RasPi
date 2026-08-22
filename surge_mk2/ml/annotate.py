"""ml/annotate.py — SAM 併用の半自動アノテーションツール。

    python3 ml/annotate.py ml/data/frames \\
        --checkpoint sam_vit_b_01ec64.pth --model-type vit_b

走行可能な床を1クリックすると、その領域のマスクを SAM（Segment Anything）が
提案する。フルスクラッチのポリゴン描画ツールではなく、境界追跡という重い
部分を SAM に任せ、人間はクリックと確認だけをする設計。

## 操作

    左クリック        走行可能（前景）の点を追加
    Shift + 左クリック  除外（背景）の点を追加——誤って広がった領域を狭める
    z                 直前の点を取り消す
    Enter             現在のマスクを保存して次のフレームへ
    n / p             保存せずに次/前のフレームへ
    q                 終了

保存先は `<frames_dir>/<frame_stem>_mask.png`（0/255 の2値。`ml/dataset.py`
が読む場所とファイル名の約束）。

## 事前準備

SAM のチェックポイントは同梱されないので別途ダウンロードしておくこと。
軽量な ViT-B が第一候補（他に vit_l・vit_h もある。大きいほど精度は上がるが
1クリックあたりの推論が重くなる）:

    https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

__all__ = ["AnnotationSession", "save_mask"]


class AnnotationSession:
    """1フレームぶんの点集め→マスク確定の状態機械。

    **SAM 呼び出し以外は何も知らない**（`predictor` はダックタイピングで
    差し替え可能——`set_image(image_rgb)` と
    `predict(point_coords=, point_labels=, multimask_output=True) -> (masks, scores, logits)`
    を持つオブジェクトなら何でもよい）。cv2 のウィンドウ処理と分離してあるので、
    実物の SAM チェックポイントや画面が無くてもテストできる。
    """

    def __init__(self, predictor) -> None:
        self.predictor = predictor
        self.points: list[tuple[int, int]] = []
        self.labels: list[int] = []          # 1=前景（走行可能） 0=背景（除外）

    def load_image(self, image_rgb: np.ndarray) -> None:
        self.predictor.set_image(image_rgb)
        self.clear()

    def clear(self) -> None:
        self.points = []
        self.labels = []

    def add_point(self, x: int, y: int, *, foreground: bool = True) -> None:
        self.points.append((x, y))
        self.labels.append(1 if foreground else 0)

    def undo(self) -> None:
        if self.points:
            self.points.pop()
            self.labels.pop()

    def predict_mask(self) -> np.ndarray | None:
        """現在の点集合から2値マスク（bool, HxW、True=走行可能）を作る。

        点が1つも無ければ `None`（＝「まだ何も指定していない」）。
        SAM は複数候補を返す（`multimask_output=True`）ので、**スコア最大の
        ものを採用する**——1点だけのクリックだと候補が曖昧になりやすく、
        複数出させてから選ぶ方が安定する。
        """
        if not self.points:
            return None
        pts = np.array(self.points)
        labels = np.array(self.labels)
        masks, scores, _ = self.predictor.predict(
            point_coords=pts, point_labels=labels, multimask_output=True)
        best = int(np.argmax(scores))
        return masks[best].astype(bool)


def save_mask(mask: np.ndarray, out_path: Path) -> None:
    """2値マスク（True=走行可能）を 0/255 の 8bit PNG として保存する。"""
    arr = (mask.astype(np.uint8)) * 255
    Image.fromarray(arr, mode="L").save(out_path)


def _load_sam(checkpoint: str, model_type: str, device: str):
    from segment_anything import SamPredictor, sam_model_registry

    if model_type not in sam_model_registry:
        raise ValueError(f"未知の model-type: {model_type!r}"
                         f"（候補 {', '.join(sam_model_registry)}）")
    sam = sam_model_registry[model_type](checkpoint=checkpoint)
    sam.to(device)
    return SamPredictor(sam)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames_dir", type=Path)
    ap.add_argument("--checkpoint", required=True, help="SAM のチェックポイント (.pth)")
    ap.add_argument("--model-type", default="vit_b",
                    choices=("default", "vit_b", "vit_l", "vit_h"))
    ap.add_argument("--device", default="cpu", help="cpu / mps / cuda")
    ap.add_argument("--skip-labeled", action="store_true",
                    help="既にマスクがあるフレームは飛ばす")
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        print("opencv-python が入っていません。`pip install -r ml/requirements.txt`",
              file=sys.stderr)
        return 2

    frame_paths = sorted(p for p in args.frames_dir.glob("*.jpg"))
    if args.skip_labeled:
        frame_paths = [p for p in frame_paths
                      if not (args.frames_dir / f"{p.stem}_mask.png").exists()]
    if not frame_paths:
        print("アノテーション対象のフレームが見つかりません", file=sys.stderr)
        return 1

    print(f"# {len(frame_paths)}枚を読み込み中。SAM モデルを準備しています…")
    predictor = _load_sam(args.checkpoint, args.model_type, args.device)
    session = AnnotationSession(predictor)

    win = "annotate  [左:走行可能 shift+左:除外 z:undo Enter:保存 n/p:次/前 q:終了]"
    cv2.namedWindow(win)

    def on_mouse(event, x, y, flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN:
            foreground = not (flags & cv2.EVENT_FLAG_SHIFTKEY)
            session.add_point(x, y, foreground=foreground)

    cv2.setMouseCallback(win, on_mouse)

    idx = 0
    while 0 <= idx < len(frame_paths):
        path = frame_paths[idx]
        image_bgr = cv2.imread(str(path))
        if image_bgr is None:
            print(f"# 読み込めない: {path}", file=sys.stderr)
            idx += 1
            continue
        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        session.load_image(image_rgb)

        advance = 1
        while True:
            mask = session.predict_mask()
            disp = image_bgr.copy()
            if mask is not None:
                overlay = disp.copy()
                overlay[mask] = (0, 255, 0)
                disp = cv2.addWeighted(disp, 0.6, overlay, 0.4, 0)
            for (px, py), lb in zip(session.points, session.labels):
                cv2.circle(disp, (px, py), 4, (0, 255, 0) if lb else (0, 0, 255), -1)
            cv2.putText(disp, f"{idx + 1}/{len(frame_paths)} {path.name}",
                       (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(win, disp)
            key = cv2.waitKey(20) & 0xFF

            if key == ord('q'):
                cv2.destroyAllWindows()
                return 0
            if key == ord('z'):
                session.undo()
                continue
            if key in (13, 10):                       # Enter
                if mask is not None:
                    save_mask(mask, args.frames_dir / f"{path.stem}_mask.png")
                    print(f"# 保存: {path.stem}_mask.png")
                advance = 1
                break
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
