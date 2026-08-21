"""logger_node — 内部バスを丸ごと MCAP に記録する（`docs/architecture.md` §11）。

    .venv/bin/python -m raspi.nodes.logger_node                  # logs/surge_*.mcap
    .venv/bin/python -m raspi.nodes.logger_node --image-hz 0     # 画像を記録しない
    .venv/bin/python -m raspi.nodes.logger_node --duration 30 -o /tmp/t.mcap

## なぜ `.sfl` があるのに要るのか

**`.sfl` は UART を流れたものしか持っていない。** LiDAR も車両状態も UART 経由
なので `.sfl` に入っているが、**カメラ画像はどこにも記録されていない**
（共有メモリに置いて捨てているだけ）。Phase 0 の達成条件「全データが記録できる」を
満たすには、バス側にもう1つ記録点が要る。

役割分担:

| | `.sfl`（io_node） | `.mcap`（このノード） |
|---|---|---|
| 中身 | UART の生バイト | 解釈済みのトピック（画像を含む） |
| 目的 | 異常の証拠・再生 | 解析・Foxglove で目視 |
| 依存 | stdlib のみ | `mcap` |

**両方を同時に回すのが既定。** 片方だけでは「CRC エラーの証拠」か
「カメラに何が写っていたか」のどちらかが失われる。

## 購読ポリシーは RELIABLE

ロガーは**取りこぼしてはいけない**側（§6.2）。`CONFLATE` を使うと 50Hz の
`vehicle_state` がポーリング周期ぶんしか残らず、記録が虫食いになる。

例外は画像で、`image/front` の `ImageRef`（数百バイト）は全部記録するが、
**JPEG に焼くのは `--image-hz` に間引く。** 30Hz×2台をそのまま焼くと
毎分 100MB を超え、エンコードだけで CPU を1コア食う。

## ディスクの目安

**ほぼ全部が画像。** 数値だけなら zstd がよく効いて 1MB/分に満たない
（Mac で 50Hz+10Hz を1分ぶん書いて 148KB）。画像は圧縮済みなので素通りする。

**Pi 実機の実測は 10.3MB/分**（640×480 の JPEG が約 20KB、5Hz×2台）。
`.sfl` の 2.5MB/分 と合わせて **約 12.8MB/分 = 1日 18GB** になるので、
`surge-logclean.timer` は毎時走って「7日より古いもの」と
「合計 8GB を超えたぶん（古い順）」を消す。

- 画像を減らす → `--image-hz 2`（約 5MB/分）/ `--image-hz 0`（画像なし）

書き込み自体は軽い（Mac で実時間の 1200倍速。1分ぶんを 0.05秒で書く）ので、
**気にするのは CPU ではなくカードの残量**。

## `hb/logger` は publish しない

このノードは購読専用。**バスに口を持たせると `zbus` の endpoint 表に
ノードを1つ増やすことになる**が、それを見る `safety_node` はまだ無い。
生死は systemd（`systemctl status surge-logger`）で見る。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import RELIABLE, Subscriber  # noqa: E402
from raspi.core.jpeg import RingJpeg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_CMD,
    TOPIC_DIAG_LINK,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_IMAGE_REAR,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
)
from raspi.rec.mcap_log import McapLog, default_mcap_path  # noqa: E402

__all__ = ["LoggerNode", "DEFAULT_TOPICS", "IMAGE_TOPICS"]

NS = 1_000_000_000

#: 記録するトピック。**`image/` は接頭辞購読**にして両カメラをまとめて取る
DEFAULT_TOPICS = [
    TOPIC_VEHICLE_STATE,
    TOPIC_SCAN,
    TOPIC_DIAG_LINK,
    TOPIC_CMD,
    TOPIC_HB_PREFIX,
    "image/",
]
IMAGE_TOPICS = (TOPIC_IMAGE_FRONT, TOPIC_IMAGE_REAR)

#: バスを覗く間隔。50Hz の `vehicle_state` に対して十分細かい
POLL_MS = 20

DEFAULT_IMAGE_HZ = 5.0


def _mcap_topic(bus_topic: str) -> str:
    """バスのトピック名 → MCAP のトピック名。**先頭に `/` を付けるだけ。**

    Foxglove の慣習に合わせる。中身は変えないので、`.mcap` を見れば
    バス上のトピック名がそのまま分かる。
    """
    return "/" + bus_topic


class LoggerNode:
    """バスを購読して MCAP に書く。

    :param path: 出力先 `.mcap`
    :param topics: 購読するバストピック
    :param image_hz: JPEG に焼く頻度（カメラ1台あたり）。0 で画像を記録しない
    :param jpeg_quality: JPEG 品質
    :param compression: MCAP の圧縮（zstd / lz4 / none）
    :param viz: Foxglove 用の `/viz/*` トピックも書くか。
        **False にしても数値は全部残る**（Foxglove で絵にならなくなるだけ）
    """

    def __init__(self, path: str | Path, *, topics=None,
                 image_hz: float = DEFAULT_IMAGE_HZ, jpeg_quality: int = 70,
                 compression: str = "zstd", viz: bool = True) -> None:
        self.topics = list(topics if topics is not None else DEFAULT_TOPICS)
        self.image_hz = image_hz
        self.viz = viz

        self._jpeg = RingJpeg(jpeg_quality) if image_hz > 0 else None
        if self._jpeg is not None and not self._jpeg.ok:
            self._jpeg = None                 # エンコーダ無し＝画像なしで続行

        self.log = McapLog(path, compression=compression, metadata={
            "node": "logger_node",
            "topics": ",".join(self.topics),
            "image_hz": image_hz if self._jpeg is not None else 0,
            "jpeg": self._jpeg.impl if self._jpeg is not None else "none",
        })
        self.sub = Subscriber({t: RELIABLE for t in self.topics})

        self._running = False
        self.images_written = 0
        self.t_start = time.monotonic()
        #: カメラ役割名 → 次に JPEG を焼く時刻 [ns]
        self._next_image_ns: dict[str, int] = {}
        #: カメラ役割名 → 最後に見た `ImageRef`
        self._last_ref: dict[str, object] = {}

    # ── 本体 ──

    def run(self, duration_s: float | None = None, status_cb=None) -> None:
        self._running = True
        self.t_start = time.monotonic()
        end = self.t_start + duration_s if duration_s else None
        next_status = 0.0

        try:
            while self._running:
                now_wall = time.monotonic()
                if end is not None and now_wall >= end:
                    break

                for topic, msg in self.sub.poll(POLL_MS):
                    self._on_msg(topic, msg)
                self._pump_images()

                if status_cb and now_wall >= next_status:
                    status_cb(self)
                    next_status = now_wall + 0.5
        finally:
            self._running = False

    def _on_msg(self, topic: str, msg) -> None:
        self.log.write(_mcap_topic(topic), msg)
        if topic in IMAGE_TOPICS:
            # `ImageRef` 自体は全部書く（フレーム時刻の記録は軽い）。
            # 画素を焼くかどうかは `_pump_images` が別途決める
            self._last_ref[msg.cam or topic.split("/")[-1]] = msg
        elif self.viz and topic == TOPIC_SCAN:
            self.log.write_viz_scan(msg)

    def _pump_images(self) -> None:
        """`--image-hz` の間隔で共有メモリから読んで JPEG を書く。

        **`ImageRef` のスロットではなくリングの最新を読む**（`core.jpeg`）。
        間引いている以上、キューに溜まった古い参照を掘り返す意味がない。
        """
        if self._jpeg is None:
            return
        now = time.monotonic_ns()
        period = int(NS / self.image_hz)
        for cam, ref in self._last_ref.items():
            if now < self._next_image_ns.get(cam, 0):
                continue
            got = self._jpeg.encode_latest(ref.shm_name, expect_seq=ref.ring_seq)
            self._next_image_ns[cam] = now + period
            if got is None:
                continue
            jpg, t_capture = got
            self.log.write_viz_image(jpg, cam, t_capture)
            self.images_written += 1

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        self.sub.close()
        if self._jpeg is not None:
            self._jpeg.close()
        self.log.close()

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self.t_start


# ── CLI ────────────────────────────────────────────────────────────────

def _status(node: LoggerNode) -> None:
    el = max(node.elapsed, 1e-9)
    n = node.log.counts
    # **必ず stderr。** `-o -` のとき stdout は MCAP そのものなので、
    # human 向けの1行でも混ぜたらファイルが壊れる
    sys.stderr.write(
        f"\r{el:6.1f}s  {node.log.size_text:>8}  "
        f"state={n.get('/vehicle_state', 0)} scan={n.get('/scan', 0)} "
        f"img={node.images_written} 計{node.log.written}件 "
        f"({node.log.written / el:.0f}/s)    ")
    sys.stderr.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-o", "--out", default=None,
                    help="出力先（既定 logs/surge_<日時>.mcap）。"
                         "**`-` で標準出力**（SD に書かずに PC へ流す）")
    ap.add_argument("--image-hz", type=float, default=DEFAULT_IMAGE_HZ,
                    help=f"JPEG を焼く頻度（カメラ1台あたり。既定 {DEFAULT_IMAGE_HZ}）。"
                         "0 で画像を記録しない")
    ap.add_argument("--jpeg-quality", type=int, default=70)
    ap.add_argument("--compression", default="zstd", choices=["zstd", "lz4", "none"])
    ap.add_argument("--no-viz", action="store_true",
                    help="Foxglove 用の /viz/* を書かない（数値は全部残る）")
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    out = args.out or default_mcap_path()
    # `-o -` は「SD に書かない」ためのモード。人間向けの出力は全部 stderr に寄せる
    say = (lambda *a: print(*a, file=sys.stderr)) if str(out) == "-" else print
    try:
        node = LoggerNode(out, image_hz=args.image_hz,
                          jpeg_quality=args.jpeg_quality,
                          compression=args.compression, viz=not args.no_viz)
    except RuntimeError as e:                 # mcap が無い
        print(f"!! {e}", file=sys.stderr)
        return 2

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: node.stop())

    say(f"# logger_node -> {'標準出力（SD に書かない）' if str(out) == '-' else out}")
    say(f"# 購読: {' '.join(node.topics)}")
    say("# 画像: " + (f"{args.image_hz}Hz/台 ({node._jpeg.impl})"
                      if node._jpeg is not None else "記録しない"))
    say("# Foxglove Studio で開ける（/viz/scan が点群、/viz/image/* が映像）\n")

    try:
        node.run(duration_s=args.duration,
                 status_cb=None if args.quiet else _status)
    finally:
        el = node.elapsed
        node.close()
        say("\n\n=== 記録した ===")
        say(f"{out}  {node.log.size_text}  {el:.1f}s")
        for topic, n in sorted(node.log.counts.items()):
            say(f"  {topic:22} {n:8}  ({n / max(el, 1e-9):6.1f}/s)")
        jp = node._jpeg
        if jp is not None and jp.encoded:
            # **二重エンコードを測るための数字**（2026-08-21 のレビュー 🟡7）。
            # 同じフレームを telemetry_node も焼いている。ここの
            # `CPU/枚 × image_hz × カメラ台数` が無視できない大きさなら、
            # camera_node で1回だけ焼いて配る形に変える価値がある。
            # **測る前に直さない**（誰も見ていない間の CPU がむしろ増える形になりうる）
            say(f"  JPEG: {jp.encoded}枚  {jp.cpu_per_frame_ms:.1f}ms/枚 "
                f"(CPU 計 {jp.encode_cpu_s:.1f}s = 実時間の "
                f"{100 * jp.encode_cpu_s / max(el, 1e-9):.1f}%)  [{jp.impl}]")
        if jp is not None and (jp.torn or jp.errors or jp.reattached):
            say(f"  画像の取りこぼし: 上書き {jp.torn} / エラー {jp.errors}"
                f" / 掴み直し {jp.reattached}")
            # **数だけでは原因が分からない。** 理由をそのまま出す
            if jp.last_error:
                say(f"  直近の理由: {jp.last_error}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
