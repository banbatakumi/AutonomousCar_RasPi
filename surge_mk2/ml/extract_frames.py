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

`--min-interval-ms` で間引きができる（既定 0 = 全件出力）。走行中の映像は
前後フレームがほとんど同じ構図になりがちで、全件出すと後続のアノテーション
（`ml/annotate.py`）の手間がそのまま比例して増える。カメラ側の実効fpsより
大きい間隔を指定すれば、そのぶんアノテーション対象を減らせる。
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
                writer: csv.writer, *, min_interval_ns: int = 0) -> int:
    """1つの `.mcap` からフレームを書き出し、書いた枚数を返す。

    `min_interval_ns` を指定すると、カメラごとに直前に書き出したフレームから
    その時間未満しか経っていないフレームを飛ばす（既定 0 = 間引きなし）。
    """
    n = 0
    last_t_by_cam: dict[str, int] = {}
    topics = [VIZ_IMAGE_PREFIX + c for c in cams]
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(topics=topics):
            cam = channel.topic[len(VIZ_IMAGE_PREFIX):]
            t_ns = message.log_time
            last_t = last_t_by_cam.get(cam)
            if last_t is not None and t_ns - last_t < min_interval_ns:
                continue
            obj = json.loads(message.data)
            jpg = base64.b64decode(obj["data"])
            name = f"{mcap_path.stem}_{cam}_{t_ns}.jpg"
            (out_dir / name).write_bytes(jpg)
            writer.writerow([name, mcap_path.name, cam, t_ns])
            last_t_by_cam[cam] = t_ns
            n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("ml/data/frames"))
    ap.add_argument("--cam", choices=("front", "rear", "both"), default="front",
                    help="取り出すカメラ（既定 front。無制限部門の MVP は前方のみ想定）")
    ap.add_argument("--min-interval-ms", type=int, default=0,
                    help="この間隔未満のフレームは間引く（既定 0 = 全件出力）")
    args = ap.parse_args()

    cams = {"front", "rear"} if args.cam == "both" else {args.cam}
    min_interval_ns = args.min_interval_ms * 1_000_000
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
            n = extract_one(p, args.out, cams, w, min_interval_ns=min_interval_ns)
            print(f"# {p.name}: {n}枚")
            total += n

    print(f"# 合計 {total}枚 → {args.out}/manifest.csv に追記")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
