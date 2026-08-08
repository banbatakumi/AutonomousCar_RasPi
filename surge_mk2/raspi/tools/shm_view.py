"""共有メモリリングを**別プロセスから**読む。camera_node の読み手側の検証。

    python3 -m raspi.tools.shm_view surge_cam0
    python3 -m raspi.tools.shm_view surge_cam0 --duration 10
    python3 -m raspi.tools.shm_view surge_cam0 --save shot.png   # 画を目で確認する

見るもの:

- **本当にゼロコピーか** — numpy 配列が自前のデータを持っていないこと
- **配信の遅れ** — `now - t_capture`。カメラの撮像から読み手が触るまで
- **取りこぼし** — 読み手が遅いと seq が飛ぶ。リング段数が足りているかの判断材料
- **上書き検出** — 読んでいる最中に書き換えられた回数（seqlock が捕まえた数）

`--save` は PNG で書き出す（zlib は標準ライブラリ。外部依存なし）。
**配管が通っただけでなく画素が正しいこと**は、最後は目で見ないと分からない。
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import FrameRing  # noqa: E402


def write_png(path: Path, arr) -> None:
    """numpy 配列を PNG にする。標準ライブラリだけで完結させる。"""
    import numpy as np

    a = np.asarray(arr)
    if a.ndim == 2:
        a = a[:, :, None]
    h, w, ch = a.shape
    if ch == 1:
        color, depth = 0, 1
    elif ch == 3:
        color, depth = 2, 3
    elif ch == 4:
        color, depth = 6, 4
    else:
        raise ValueError(f"チャンネル数 {ch} は PNG にできません")

    # 各行の先頭にフィルタ種別 0 を挟む（PNG の仕様）
    raw = b"".join(b"\x00" + a[y, :, :depth].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">2I5B", w, h, 8, color, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6))
           + chunk(b"IEND", b""))
    path.write_bytes(png)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", default="surge_cam0",
                    help="共有メモリの名前（既定 surge_cam0）")
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--hz", type=float, default=30.0, help="読み出す頻度")
    ap.add_argument("--save", type=Path, default=None, help="PNG に書き出す")
    args = ap.parse_args()

    try:
        ring = FrameRing.attach(args.name)
    except FileNotFoundError:
        print(f"共有メモリ {args.name} がありません。camera_node は動いていますか？",
              file=sys.stderr)
        return 2

    print(f"接続: {ring}")
    print(f"  {ring.width}x{ring.height} {ring.fmt} ×{ring.n_slots}枚 "
          f"({ring.nbytes / 1e6:.1f}MB)")

    seen = 0
    torn = 0
    missed = 0
    last_seq = 0
    lat_ms: list[float] = []
    zero_copy: bool | None = None
    saved = False

    period = 1.0 / args.hz
    end = time.monotonic() + args.duration
    next_t = time.monotonic()
    while time.monotonic() < end:
        now = time.monotonic()
        if now < next_t:
            time.sleep(min(next_t - now, 0.005))
            continue
        next_t += period

        ref = ring.latest()
        if ref is None:
            continue
        if ref.desc.seq == last_seq:
            continue                     # まだ新しい画が来ていない

        arr = ref.as_array()
        if zero_copy is None:
            zero_copy = not arr.flags["OWNDATA"]
        # 実際に画素へ触る（触らないとゼロコピーの検証にならない）
        checksum = int(arr[::64, ::64].sum())
        want_save = bool(args.save) and not saved
        if want_save:
            write_png(args.save, arr)

        # **画素を使い終わってから** seqlock を再検査する
        ok = ref.still_valid()
        arr = None          # 共有メモリへの参照を手放す（後述の close 制約）
        if not ok:
            torn += 1
            continue
        if want_save:
            print(f"  保存: {args.save}（チェックサム {checksum}）")
            saved = True

        if last_seq and ref.desc.seq > last_seq + 1:
            missed += ref.desc.seq - last_seq - 1
        last_seq = ref.desc.seq
        seen += 1
        lat_ms.append((time.clock_gettime_ns(time.CLOCK_MONOTONIC)
                       - ref.desc.t_capture_ns) / 1e6)

    # **close する前に参照を全部手放す。** numpy 配列や memoryview が生きていると
    # 共有メモリを閉じられず、後始末で BufferError になる
    ref = arr = None
    ring.close()

    print(f"\n=== {args.duration:.0f}秒の結果 ===")
    if not seen:
        print("フレームを1枚も受け取れませんでした")
        return 1
    lat_ms.sort()
    print(f"  受信          : {seen}枚 ({seen / args.duration:.1f}fps)")
    print(f"  未取得        : {missed}枚（読み手が間に合わなかった分）")
    print(f"  上書き検出    : {torn}回（seqlock が捕まえた分）")
    print(f"  撮像→読み手   : 中央値 {lat_ms[len(lat_ms)//2]:.1f}ms  "
          f"最小 {lat_ms[0]:.1f}ms  最大 {lat_ms[-1]:.1f}ms")
    print(f"  ゼロコピー    : {'はい（共有メモリを直接見ている）' if zero_copy else '**いいえ**'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
