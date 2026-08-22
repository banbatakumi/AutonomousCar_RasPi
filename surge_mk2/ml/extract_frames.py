"""ml/extract_frames.py — `.mcap` から走行時のカメラフレームを取り出す。

GUIの「ログ」タブ（`gui/src/views/LogView.tsx` の `.mcap` セクション、
「画像を含める」を ON にした録画）で録った `.mcap` には、`/viz/image/front`・
`/viz/image/rear` に `foxglove.CompressedImage`（JPEG・base64）として画像が
入っている（`raspi/rec/mcap_log.py`）。**Pi 側に新しい記録コードを足す必要は
無い**——ここではその中身を JPEG ファイルとして書き出すだけ。

    python3 ml/extract_frames.py logs/run1.mcap logs/run2.mcap \\
        --out ml/data/frames --cam front

出力先に `manifest.csv`（ファイル名・元の `.mcap`・カメラ・撮像時刻）を
追記していく。**実行するたびに追記される**ので、複数回の走行で集めた
フレームを同じ `--out` にまとめてよい。
"""

from __future__ import annotations

import argparse
import base64
import csv
import json
import sys
from pathlib import Path

try:
    from mcap.reader import make_reader
except ImportError:
    print("mcap が入っていません。`pip install -r ml/requirements.txt` してください",
          file=sys.stderr)
    raise

__all__ = ["extract_one", "VIZ_IMAGE_PREFIX"]

#: `raspi/rec/mcap_log.py` の `VIZ_IMAGE_PREFIX` と同じ値。**そちらが正**なので、
#: 値がズレたら両方直すこと（テストで往復を確認している）
VIZ_IMAGE_PREFIX = "/viz/image/"


def extract_one(mcap_path: Path, out_dir: Path, cams: set[str],
                writer: csv.writer) -> int:
    """1つの `.mcap` からフレームを書き出し、書いた枚数を返す。"""
    n = 0
    topics = [VIZ_IMAGE_PREFIX + c for c in cams]
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(topics=topics):
            cam = channel.topic[len(VIZ_IMAGE_PREFIX):]
            obj = json.loads(message.data)
            jpg = base64.b64decode(obj["data"])
            t_ns = message.log_time
            name = f"{mcap_path.stem}_{cam}_{t_ns}.jpg"
            (out_dir / name).write_bytes(jpg)
            writer.writerow([name, mcap_path.name, cam, t_ns])
            n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("ml/data/frames"))
    ap.add_argument("--cam", choices=("front", "rear", "both"), default="front",
                    help="取り出すカメラ（既定 front。無制限部門の MVP は前方のみ想定）")
    args = ap.parse_args()

    cams = {"front", "rear"} if args.cam == "both" else {args.cam}
    args.out.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out / "manifest.csv"
    is_new = not manifest_path.exists()

    total = 0
    with open(manifest_path, "a", newline="") as mf:
        w = csv.writer(mf)
        if is_new:
            w.writerow(["file", "source_mcap", "cam", "t_capture_ns"])
        for p in args.mcap_files:
            if not p.exists():
                print(f"# skip: {p}（見つからない）", file=sys.stderr)
                continue
            n = extract_one(p, args.out, cams, w)
            print(f"# {p.name}: {n}枚")
            total += n

    print(f"# 合計 {total}枚 → {args.out}/manifest.csv に追記")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
