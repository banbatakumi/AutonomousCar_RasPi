"""ml_cam_e2e/preview.py — 学習済みモデルの推論結果を、実車に乗せる前に目で確認する。

    python3 ml_cam_e2e/preview.py ml_cam_e2e/runs/v1/frames --model models/v1.onnx

**推論には `raspi/nodes/cam_e2e_node.py` の `RegressionModel` をそのまま使う。**
`ml_cam/preview.py` と同じ理由——「実車に乗せたときにどう見えるか」の答えに
するため、実行経路を1つに揃えている。

画面には推論した操舵角を矢印で重畳表示する。`manifest.csv`（`extract_pairs.py`
の出力）にそのフレームの実際の操舵角があれば、比較のため2本目の矢印を出す
（緑=モデル出力・青=実際に人間が切っていた角度）。

## 操作

    n / p    次/前のフレームへ
    q        終了
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # repo root

from raspi.nodes.cam_e2e_node import RegressionModel  # noqa: E402

__all__ = ["load_model_config", "build_model", "load_actual_steer", "arrow_endpoint"]


def load_model_config(onnx_path: Path) -> dict:
    cfg_path = onnx_path.with_suffix(".json")
    if cfg_path.exists():
        return json.loads(cfg_path.read_text())
    return {}


def build_model(onnx_path: Path) -> RegressionModel:
    cfg = load_model_config(onnx_path)
    w, h = cfg.get("input_size", [224, 224])
    return RegressionModel(str(onnx_path), input_size=(int(w), int(h)),
                           mean=float(cfg.get("mean", 0.0)),
                           std=float(cfg.get("std", 255.0)),
                           max_steer=float(cfg.get("max_steer", 0.524)))


def load_actual_steer(frames_dir: Path) -> dict[str, float]:
    """`manifest.csv` の `file → target_steer` の対応表。無ければ空 dict。"""
    manifest = frames_dir / "manifest.csv"
    out: dict[str, float] = {}
    if not manifest.exists():
        return out
    with open(manifest) as f:
        for row in csv.DictReader(f):
            out[row["file"]] = float(row["target_steer"])
    return out


def arrow_endpoint(origin: tuple[int, int], steer_rad: float, length: float,
                   *, gain: float = 3.0) -> tuple[int, int]:
    """矢印の先端座標。**`steer_rad` をそのまま画角に使うと振れ幅が小さすぎて
    見えないので `gain` 倍して誇張する**（あくまで目視確認用の表示で、実車の
    ステアリングジオメトリを表しているわけではない）。反時計回り正の `steer_rad`
    は画面上では左（x が小さい方）へ傾く。
    """
    ox, oy = origin
    angle = steer_rad * gain
    x = ox - length * math.sin(angle)
    y = oy - length * math.cos(angle)
    return int(round(x)), int(round(y))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frames_dir", type=Path)
    ap.add_argument("--model", required=True, help="ONNXモデルのパス（同名 .json を前処理・出力契約として使う）")
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        print("opencv-python が入っていません。`pip install -r ml_cam_e2e/requirements.txt`",
              file=sys.stderr)
        return 2

    frame_paths = sorted(p for p in args.frames_dir.glob("*.jpg"))
    if not frame_paths:
        print("プレビュー対象のフレームが見つかりません", file=sys.stderr)
        return 1

    print(f"# モデルを読み込み中: {args.model}")
    model = build_model(Path(args.model))
    actual_steer = load_actual_steer(args.frames_dir)

    win = "preview  [n/p:次/前 q:終了]  緑=モデル出力 青=実際の操舵"
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
        steer_norm = model.infer(image_rgb)
        steer_rad = steer_norm * model.max_steer

        h, w = image_bgr.shape[:2]
        origin = (w // 2, h - 10)
        disp = image_bgr.copy()

        label = f"{idx + 1}/{len(frame_paths)} {path.name}  pred={math.degrees(steer_rad):+.1f}deg"
        pred_end = arrow_endpoint(origin, steer_rad, h * 0.4)
        cv2.arrowedLine(disp, origin, pred_end, (0, 255, 0), 2, tipLength=0.2)

        actual = actual_steer.get(path.name)
        if actual is not None:
            label += f"  actual={math.degrees(actual):+.1f}deg"
            actual_end = arrow_endpoint(origin, actual, h * 0.4)
            cv2.arrowedLine(disp, origin, actual_end, (255, 0, 0), 2, tipLength=0.2)

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
