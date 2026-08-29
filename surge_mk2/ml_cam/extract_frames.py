"""ml_cam/extract_frames.py — `.mcap` から走行時のカメラフレームを取り出す。

GUIの「ログ」タブ（`gui/src/views/LogView.tsx` の `.mcap` セクション、
「画像を含める」を ON にした録画）で録った `.mcap` には、`/viz/image/front`・
`/viz/image/rear` に `foxglove.CompressedImage`（JPEG・base64）として画像が
入っている（`raspi/rec/mcap_log.py`）。**Pi 側に新しい記録コードを足す必要は
無い**——ここではその中身を JPEG ファイルとして書き出すだけ。

    python3 ml_cam/extract_frames.py logs/run1.mcap logs/run2.mcap \\
        --out ml_cam/data/frames --cam front

出力先に `manifest.csv`（ファイル名・元の `.mcap`・カメラ・撮像時刻）を
追記していく。**実行するたびに追記される**ので、複数回の走行で集めた
フレームを同じ `--out` にまとめてよい。

間引きは2通り。`--min-interval-ms`は時間間隔（既定 0 = 全件出力）、
`--target-count`は合計の欲しい枚数（複数 `.mcap`・カメラをまたいで
均等に間引く）。走行中の映像は前後フレームがほとんど同じ構図になりがちで、
全件出すと後続のアノテーション（`ml_cam/annotate.py`）の手間がそのまま
比例して増える。どちらか一方だけ指定できる。
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
    print("mcap が入っていません。`pip install -r ml_cam/requirements.txt` してください",
          file=sys.stderr)
    raise

__all__ = ["extract_one", "count_messages", "VIZ_IMAGE_PREFIX"]

#: `raspi/rec/mcap_log.py` の `VIZ_IMAGE_PREFIX` と同じ値。**そちらが正**なので、
#: 値がズレたら両方直すこと（テストで往復を確認している）
VIZ_IMAGE_PREFIX = "/viz/image/"


def count_messages(mcap_path: Path, cams: set[str]) -> int:
    """対象カメラのメッセージ数を数える（JPEGデコード・書き出しはしない軽い走査）。

    `--target-count` の間引き幅（stride）を決めるための下見に使う。
    """
    topics = [VIZ_IMAGE_PREFIX + c for c in cams]
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        return sum(1 for _ in reader.iter_messages(topics=topics))


def extract_one(mcap_path: Path, out_dir: Path, cams: set[str],
                writer: csv.writer, *, min_interval_ns: int = 0,
                keep_ratio: float = 1.0,
                acc_by_cam: dict[str, float] | None = None,
                ) -> tuple[int, dict[str, float]]:
    """1つの `.mcap` からフレームを書き出す。`(書いた枚数, 更新後の acc_by_cam)` を返す。

    間引きは2通りのうち片方だけ効く。`keep_ratio < 1.0` なら誤差蓄積法
    （Bresenham 式）で間引く：カメラごとに独立した蓄積値へ毎メッセージ
    `keep_ratio` を足し、1.0 を超えたら採用してその分を引く。カメラ別に
    蓄積するので `--cam both` で前後が交互に並んでいても偏らず、割り算の
    余りで丸めないので目標枚数からの誤差もカメラごとに高々1枚に収まる
    （通し番号を `stride` で割った余りで間引く方式だと、上記どちらも起こり得た）。
    複数ファイルをまたぐときは `acc_by_cam` に前ファイルの終了値を渡して続ける。
    `keep_ratio == 1.0` なら `min_interval_ns` を使い、カメラごとに直前に
    書き出したフレームからその時間未満しか経っていないフレームを飛ばす
    （既定 0 = 間引きなし）。
    """
    n = 0
    acc_by_cam = dict(acc_by_cam) if acc_by_cam else {}
    last_t_by_cam: dict[str, int] = {}
    topics = [VIZ_IMAGE_PREFIX + c for c in cams]
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(topics=topics):
            cam = channel.topic[len(VIZ_IMAGE_PREFIX):]
            t_ns = message.log_time
            if keep_ratio < 1.0:
                acc = acc_by_cam.get(cam, 0.0) + keep_ratio
                take = acc >= 1.0
                acc_by_cam[cam] = acc - 1.0 if take else acc
            else:
                last_t = last_t_by_cam.get(cam)
                take = last_t is None or t_ns - last_t >= min_interval_ns
            if take:
                obj = json.loads(message.data)
                jpg = base64.b64decode(obj["data"])
                name = f"{mcap_path.stem}_{cam}_{t_ns}.jpg"
                (out_dir / name).write_bytes(jpg)
                writer.writerow([name, mcap_path.name, cam, t_ns])
                last_t_by_cam[cam] = t_ns
                n += 1
    return n, acc_by_cam


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap_files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("ml_cam/data/frames"))
    ap.add_argument("--cam", choices=("front", "rear", "both"), default="front",
                    help="取り出すカメラ（既定 front。無制限部門の MVP は前方のみ想定）")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--min-interval-ms", type=int, default=0,
                       help="この間隔未満のフレームは間引く（既定 0 = 全件出力）")
    group.add_argument("--target-count", type=int, default=0,
                       help="全 .mcap 合計でこの枚数に近づくよう均等に間引く")
    args = ap.parse_args()

    cams = {"front", "rear"} if args.cam == "both" else {args.cam}
    args.out.mkdir(parents=True, exist_ok=True)
    manifest_path = args.out / "manifest.csv"
    is_new = not manifest_path.exists()

    existing = [p for p in args.mcap_files if p.exists()]
    for p in args.mcap_files:
        if p not in existing:
            print(f"# skip: {p}（見つからない）", file=sys.stderr)

    keep_ratio = 1.0
    if args.target_count > 0:
        # 下見パス：JPEGデコード・書き出しをしない軽い走査で合計メッセージ数を数え、
        # 全ファイル・全カメラ通しで目標枚数に近づく採用割合を決める
        total_messages = sum(count_messages(p, cams) for p in existing)
        keep_ratio = min(1.0, args.target_count / total_messages) if total_messages else 1.0

    min_interval_ns = args.min_interval_ms * 1_000_000
    total = 0
    acc_by_cam: dict[str, float] = {}
    with open(manifest_path, "a", newline="") as mf:
        w = csv.writer(mf)
        if is_new:
            w.writerow(["file", "source_mcap", "cam", "t_capture_ns"])
        for p in existing:
            n, acc_by_cam = extract_one(p, args.out, cams, w, min_interval_ns=min_interval_ns,
                                        keep_ratio=keep_ratio, acc_by_cam=acc_by_cam)
            print(f"# {p.name}: {n}枚")
            total += n

    print(f"# 合計 {total}枚 → {args.out}/manifest.csv に追記")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
