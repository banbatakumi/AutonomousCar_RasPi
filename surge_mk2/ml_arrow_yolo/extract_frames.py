"""走行記録（.mcap）から前方カメラの JPEG を切り出す（矢印信号の YOLO の学習データ①）。

    .venv/bin/python ml_arrow_yolo/extract_frames.py logs/x.mcap ... --out ml_arrow_yolo/runs/v1/images

`/viz/image/front`（`raspi/rec/mcap_log.py` が書く `foxglove.CompressedImage`）を読み、
`<記録名>__<時刻ns>.jpg` で書き出す。**画像はデコードし直さない**（車が撮ったバイトのまま）。

`--min-interval-ms` より短い間隔のコマは飛ばす（止まった車の前で同じ絵が何百枚も並ぶのを防ぐ）。
同じ名前の画像が既にあれば書き直さない（記録を足して抽出し直しても、付けたラベルは消えない）。
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from pathlib import Path

__all__ = ["VIZ_IMAGE_PREFIX", "extract_one", "main"]

#: `raspi/rec/mcap_log.py` の `VIZ_IMAGE_PREFIX` と同じ値
VIZ_IMAGE_PREFIX = "/viz/image/"


def extract_one(mcap_path: Path, out_dir: Path, *, cam: str = "front",
                min_interval_ms: int = 200) -> tuple[int, int]:
    """1本の記録を切り出す。`(書いた枚数, 既にあって飛ばした枚数)` を返す。"""
    from mcap.reader import make_reader

    out_dir.mkdir(parents=True, exist_ok=True)
    topic = VIZ_IMAGE_PREFIX + cam
    min_gap = int(min_interval_ms) * 1_000_000
    last_t = None
    written = existed = 0
    with open(mcap_path, "rb") as f:
        for _schema, _channel, message in make_reader(f).iter_messages(topics=[topic],
                                                                         log_time_order=True):
            t = message.log_time
            if last_t is not None and t - last_t < min_gap:
                continue
            last_t = t
            try:
                jpeg = base64.b64decode(json.loads(message.data)["data"])
            except (ValueError, KeyError, TypeError):
                continue
            out = out_dir / f"{mcap_path.stem}__{t:019d}.jpg"
            if out.exists():
                existed += 1
                continue
            out.write_bytes(jpeg)
            written += 1
    return written, existed


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True, help="画像の書き出し先（runs/<名前>/images）")
    ap.add_argument("--cam", default="front", help="カメラ（front＝右・主カメラ / left）")
    ap.add_argument("--min-interval-ms", type=int, default=200, help="これより短い間隔のコマは飛ばす")
    args = ap.parse_args()

    total = 0
    for p in args.mcap:
        if not p.is_file():
            print(f"# skip: {p} が無い", flush=True)
            continue
        try:
            n, existed = extract_one(p, args.out, cam=args.cam, min_interval_ms=args.min_interval_ms)
        except Exception as e:  # noqa: BLE001 — 壊れた記録が1本あっても残りは続ける
            print(f"# skip: {p.name}: {e}", flush=True)
            continue
        total += n
        print(f"{p.name}: {n}枚" + (f"（既にある {existed}枚は飛ばした）" if existed else ""), flush=True)
    print(f"# 合計 {total}枚 → {args.out}", flush=True)
    if total == 0:
        print("# 画像が1枚も取れませんでした。記録を「画像付き」で録ったか確認してください", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
