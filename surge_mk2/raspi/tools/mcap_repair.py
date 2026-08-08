"""記録途中で切れた `.mcap` を読める形に直す。

    python3 -m raspi.tools.mcap_repair logs/surge_20260808_233813.mcap
    python3 -m raspi.tools.mcap_repair logs/*.mcap --inplace

## なぜ要るのか

**MCAP は最後に `finish()` を呼んで初めて索引（要約セクション）が書かれる。**
電源が落ちて `logger_node` が終了処理を通らなかった場合、データ本体は
書けているのに索引が無く、**普通のリーダは開く前に例外で落ちる**:

    RecordLengthLimitExceeded: unknown (opcode 0) record has length ...

`.sfl` は追記のみ・索引なしなので尻切れでも前半が必ず読めるが、
`.mcap` はそうではない。**Pi には原因不明の再起動の実績がある**（PROGRESS.md）ので、
カメラ画像を丸ごと失わないためにこの復旧経路を用意しておく。

## やること

先頭から素直にレコードを舐め、**切れたところで打ち切って**新しいファイルに
書き直す（索引付き）。読めた最後のメッセージまでは完全に救える。

**記録中のファイルにも使える。** 索引が無いという意味では同じ状態なので、
走行中に「今どこまで録れているか」を覗くのにも使える。
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

__all__ = ["repair"]


def repair(src: str | Path, dst: str | Path) -> dict:
    """`src` を先頭から読めるだけ読んで `dst` に書き直す。

    :returns: `{"messages": 件数, "topics": {...}, "truncated": bool}`
    :raises RuntimeError: `mcap` が入っていない
    """
    try:
        from mcap.exceptions import McapError
        from mcap.records import Channel, Message, Metadata, Schema
        from mcap.stream_reader import StreamReader
        from mcap.writer import Writer
    except ImportError as e:                  # pragma: no cover - 環境依存
        raise RuntimeError(f"mcap が入っていません: {e}") from e

    src, dst = Path(src), Path(dst)
    schemas: dict[int, int] = {}
    channels: dict[int, int] = {}
    topics: dict[str, int] = {}
    n = 0
    truncated = False

    with open(src, "rb") as fin, open(dst, "wb") as fout:
        w = Writer(fout)
        w.start(profile="", library="surge-mk2 (repaired)")
        try:
            for rec in StreamReader(fin).records:
                # **同じ id は二度登録しない。** 正常な MCAP は Schema/Channel を
                # データ部と要約部の両方に持つので、素直に舐めると同じトピックの
                # チャネルが2本でき、片方が0件になる
                if isinstance(rec, Schema):
                    if rec.id not in schemas:
                        schemas[rec.id] = w.register_schema(
                            name=rec.name, encoding=rec.encoding, data=rec.data)
                elif isinstance(rec, Channel):
                    if rec.id not in channels:
                        channels[rec.id] = w.register_channel(
                            topic=rec.topic, message_encoding=rec.message_encoding,
                            schema_id=schemas.get(rec.schema_id, 0),
                            metadata=rec.metadata)
                elif isinstance(rec, Message):
                    cid = channels.get(rec.channel_id)
                    if cid is None:
                        continue              # チャネル定義より先に来ることは無いはず
                    w.add_message(channel_id=cid, log_time=rec.log_time,
                                  data=rec.data, publish_time=rec.publish_time,
                                  sequence=rec.sequence)
                    n += 1
                elif isinstance(rec, Metadata):
                    w.add_metadata(rec.name, rec.metadata)
        except (McapError, struct.error, EOFError, ValueError):
            # 尻切れ。**ここまでは救えている**ので、黙って打ち切って閉じる
            truncated = True
        finally:
            w.finish()

    # トピック別の件数は、直した後のファイルを読めば正確に取れる
    from mcap.reader import make_reader

    with open(dst, "rb") as f:
        summary = make_reader(f).get_summary()
        if summary is not None:
            counts = summary.statistics.channel_message_counts
            # **`.get` で引く。** チャネル定義だけ残ってメッセージが1件も
            # 救えなかったトピックがありうる（切れた位置による）
            for cid, ch in summary.channels.items():
                topics[ch.topic] = counts.get(cid, 0)

    return {"messages": n, "topics": topics, "truncated": truncated}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", type=Path, nargs="+", help="直す .mcap")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="出力先（既定は `<名前>.repaired.mcap`）")
    ap.add_argument("--inplace", action="store_true",
                    help="直したもので元を置き換える（元は .bak に残す）")
    args = ap.parse_args()

    if args.out is not None and len(args.paths) > 1:
        print("-o は入力1本のときだけ使えます", file=sys.stderr)
        return 2

    rc = 0
    for p in args.paths:
        if not p.exists():
            print(f"ファイルがありません: {p}", file=sys.stderr)
            rc = 2
            continue
        out = args.out or p.with_suffix(".repaired.mcap")
        try:
            r = repair(p, out)
        except RuntimeError as e:
            print(f"!! {e}", file=sys.stderr)
            return 2
        state = "尻切れを打ち切って復旧" if r["truncated"] else "正常（索引を作り直した）"
        print(f"{p.name} → {out.name}  {r['messages']}件  {state}")
        for topic, n in sorted(r["topics"].items()):
            print(f"  {topic:24} {n:8}")
        if args.inplace:
            p.replace(p.with_suffix(p.suffix + ".bak"))
            out.replace(p)
            print(f"  置き換えた（元は {p.name}.bak）")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
