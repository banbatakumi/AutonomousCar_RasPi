"""生フレームログ `.sfl`（SURGE Frame Log）の記録と読み出し。

**何を記録するか** — UART を流れたフレームそのもの（送受信の両方）を、Pi の
CLOCK_MONOTONIC タイムスタンプ付きでそのまま並べる。デコードも解釈もしない。

**なぜデコードしないか** — デコード済みのきれいなメッセージには、CRC エラー・
SEQ 逆行・リンク断といった「異常の証拠」が残らない。デバッグで本当に欲しいのは
そこなので、記録は生のまま持ち、解釈は再生時に何度でもやり直せるようにする。
protocol.toml が変わっても、過去のログを新しいパーサで読み直せる。

**なぜ MCAP ではないか** — MCAP は Foxglove で開ける最終形として正しいが、
それはトピック（意味のあるメッセージ）の器であり、バスと msgspec 型が決まってから。
実時間で回る io_node には外部依存を持ち込まず、stdlib だけで書けるこの形式にして、
MCAP へは後段のエクスポータで変換する。

ファイル構造（すべてリトルエンディアン）::

    ┌ ヘッダ 32B ────────────────────────────────────────┐
    │ 0  8  magic  b"SURGEFL1"                            │
    │ 8  2  format_version u16                            │
    │ 10 2  header_size    u16                            │
    │ 12 8  t0_mono_ns     u64  記録開始 CLOCK_MONOTONIC  │
    │ 20 8  t0_unix_ns     u64  記録開始 CLOCK_REALTIME   │
    │ 28 4  reserved                                      │
    └─────────────────────────────────────────────────────┘
    ┌ レコード 14B + payload（以降これの繰り返し）────────┐
    │ 0  1  kind   u8   0=RX 1=TX 2=META 3=EVENT          │
    │ 1  1  ptype  u8   パケット TYPE（META/EVENT は 0）  │
    │ 2  1  seq    u8                                     │
    │ 3  1  flags  u8   予約                              │
    │ 4  8  t_ns   u64  Pi CLOCK_MONOTONIC（絶対値）      │
    │ 12 2  length u16                                    │
    │ 14 .. payload                                       │
    └─────────────────────────────────────────────────────┘

時刻を t0 からの相対ではなく **CLOCK_MONOTONIC の絶対値**で持つのは、
TELEMETRY 内の STM32 時刻や TimeSync の推定と突き合わせるときに変換を挟みたく
ないため。t0 はヘッダにあるので相対時刻はいつでも出せる。

記録中に電源が落ちても、**末尾が途中で切れているだけ**で前半は必ず読める
（追記のみ・インデックス無し）。Reader は尻切れを `truncated` として黙って許す。
"""

from __future__ import annotations

import json
import struct
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO, NamedTuple

__all__ = [
    "MAGIC",
    "FORMAT_VERSION",
    "Kind",
    "LogRecord",
    "FileHeader",
    "FrameLogWriter",
    "FrameLogReader",
    "default_log_path",
]

MAGIC = b"SURGEFL1"
FORMAT_VERSION = 1
HEADER_SIZE = 32
REC_HEADER_SIZE = 14

_S_HEADER = struct.Struct("<8sHHQQ4x")
_S_REC = struct.Struct("<BBBBQH")

#: 時刻ベースの flush が効かなくても、この件数で必ず書き出す
_FLUSH_MAX_RECORDS = 4096


class Kind:
    RX = 0      #: STM32 → Pi。CRC 検証を通ったフレーム
    TX = 1      #: Pi → STM32
    META = 2    #: JSON。セッション情報（ファイル先頭に1個）
    EVENT = 3   #: JSON。リンク統計スナップショットなど時刻付きの出来事

    NAMES = {RX: "RX", TX: "TX", META: "META", EVENT: "EVENT"}


class FileHeader(NamedTuple):
    format_version: int
    header_size: int
    t0_mono_ns: int
    t0_unix_ns: int


class LogRecord(NamedTuple):
    kind: int
    t_ns: int          #: Pi CLOCK_MONOTONIC [ns]
    type: int          #: パケット TYPE（RX/TX のみ意味を持つ）
    seq: int
    payload: bytes

    @property
    def is_frame(self) -> bool:
        return self.kind in (Kind.RX, Kind.TX)

    def decode(self):
        """パケットの dataclass に変換する。フレームでない/未知の TYPE なら None。"""
        if not self.is_frame:
            return None
        from ..proto import packets
        cls = packets.BY_TYPE.get(self.type)
        return cls.decode(self.payload) if cls is not None else None

    def json(self) -> dict[str, Any]:
        """META/EVENT の中身。フレームなら空 dict。"""
        if self.is_frame or not self.payload:
            return {}
        return json.loads(self.payload.decode("utf-8"))


def default_log_path(directory: str | Path = "logs", prefix: str = "surge") -> Path:
    """`logs/surge_20260807_153012.sfl` のようなパスを作る（ディレクトリも作る）。

    ファイル名は**ローカル壁時計**にする。単調時刻はセッション間で比較できず、
    人間がログを探すときの手がかりにならないため。
    """
    d = Path(directory)
    d.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    return d / f"{prefix}_{stamp}.sfl"


# ── 記録 ────────────────────────────────────────────────────────────────

@dataclass(slots=True)
class _WriterStats:
    rx: int = 0
    tx: int = 0
    events: int = 0
    bytes_written: int = 0


class FrameLogWriter:
    """`.sfl` への追記。単一スレッドから呼ぶ前提（ロック無し）。

    io_node の実時間ループから叩かれるので、1レコードあたりの仕事は
    「struct.pack 1回 + BufferedWriter への write 2回」に抑えてある。
    fsync はしない（コストが跳ね上がる）。代わりに一定間隔で flush して、
    電源断で失う量を `flush_interval_s` 秒ぶんに限定する。

    :param path: 出力先
    :param meta: 先頭 META に載せるセッション情報（ポート・ボーレート・fw_id など）
    :param flush_interval_s: OS へ書き出す間隔 [s]。0 で毎レコード flush
    :param buffer_size: BufferedWriter のバッファ [B]
    """

    def __init__(self, path: str | Path, meta: dict[str, Any] | None = None,
                 *, flush_interval_s: float = 1.0, buffer_size: int = 256 * 1024) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stats = _WriterStats()
        self._flush_interval_ns = int(flush_interval_s * 1e9)
        self._next_flush_ns = 0
        self._since_flush = 0
        self._closed = False

        self.t0_mono_ns = time.monotonic_ns()
        self.t0_unix_ns = time.time_ns()

        self._f: BinaryIO = open(self.path, "wb", buffering=buffer_size)
        self._f.write(_S_HEADER.pack(MAGIC, FORMAT_VERSION, HEADER_SIZE,
                                     self.t0_mono_ns, self.t0_unix_ns))
        self.stats.bytes_written = HEADER_SIZE

        base = {
            "t0_unix_ns": self.t0_unix_ns,
            "t0_mono_ns": self.t0_mono_ns,
            "t0_local": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
        }
        base.update(meta or {})
        self._write(Kind.META, self.t0_mono_ns, 0, 0,
                    json.dumps(base, ensure_ascii=False).encode("utf-8"))

    # ── 書き込み ──

    def write_rx(self, t_ns: int, pkt_type: int, seq: int, payload: bytes) -> None:
        self._write(Kind.RX, t_ns, pkt_type, seq, payload)
        self.stats.rx += 1

    def write_tx(self, t_ns: int, pkt_type: int, seq: int, payload: bytes) -> None:
        self._write(Kind.TX, t_ns, pkt_type, seq, payload)
        self.stats.tx += 1

    def write_event(self, t_ns: int, name: str, data: dict[str, Any] | None = None) -> None:
        """時刻付きの出来事を JSON で残す。

        リンク統計・ハンドシェイク結果・モード遷移など、フレームには現れないが
        後で「なぜそうなったか」を辿るのに要る情報を入れる。
        """
        body = {"name": name}
        if data:
            body.update(data)
        self._write(Kind.EVENT, t_ns, 0, 0,
                    json.dumps(body, ensure_ascii=False, default=str).encode("utf-8"))
        self.stats.events += 1

    def _write(self, kind: int, t_ns: int, ptype: int, seq: int, payload: bytes) -> None:
        if self._closed:
            raise ValueError("クローズ済みの FrameLogWriter に書き込もうとしました")
        n = len(payload)
        if n > 0xFFFF:
            raise ValueError(f"payload が長すぎます: {n}")
        self._f.write(_S_REC.pack(kind, ptype & 0xFF, seq & 0xFF, 0, t_ns, n))
        if n:
            self._f.write(payload)
        self.stats.bytes_written += REC_HEADER_SIZE + n

        # flush の判定は呼び出し側の t_ns に乗る（時刻取得を増やさないため）。
        # ただし t_ns が単調でない・想定外に小さいと永久に flush されなくなるので、
        # レコード数の歯止めを併置する。記録が黙って失われるより安い。
        self._since_flush += 1
        if (self._flush_interval_ns <= 0 or t_ns >= self._next_flush_ns
                or self._since_flush >= _FLUSH_MAX_RECORDS):
            self._f.flush()
            self._next_flush_ns = t_ns + self._flush_interval_ns
            self._since_flush = 0

    # ── 後始末 ──

    def flush(self) -> None:
        if not self._closed:
            self._f.flush()

    def close(self) -> None:
        """終了イベントを書いてから閉じる。

        正常終了したログには必ず `close` イベントが入るので、
        読む側は「途中で電源が落ちたログ」かどうかを判定できる。
        """
        if self._closed:
            return
        try:
            self.write_event(time.monotonic_ns(), "close", {
                "rx": self.stats.rx,
                "tx": self.stats.tx,
                "events": self.stats.events,
                "duration_s": round((time.monotonic_ns() - self.t0_mono_ns) / 1e9, 3),
            })
            self._f.flush()
        finally:
            self._closed = True
            self._f.close()

    def __enter__(self) -> "FrameLogWriter":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


# ── 読み出し ────────────────────────────────────────────────────────────

class FrameLogReader:
    """`.sfl` の逐次読み出し。

    全レコードをメモリに載せない（長時間ログは数百 MB になりうる）。
    複数回まわしたいときは `list(reader)` するか、開き直すこと。

    :ivar header: ファイルヘッダ
    :ivar meta: 先頭 META の中身（無ければ空 dict）
    :ivar truncated: 末尾が途中で切れていた場合 True（イテレート後に確定）
    """

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self._f: BinaryIO = open(self.path, "rb", buffering=256 * 1024)
        self.truncated = False

        raw = self._f.read(HEADER_SIZE)
        if len(raw) < HEADER_SIZE:
            self._f.close()
            raise ValueError(f"{self.path}: ヘッダが読めません（{len(raw)}B）")
        magic, ver, hdr_len, t0_mono, t0_unix = _S_HEADER.unpack(raw)
        if magic != MAGIC:
            self._f.close()
            raise ValueError(f"{self.path}: .sfl ファイルではありません (magic={magic!r})")
        if ver > FORMAT_VERSION:
            self._f.close()
            raise ValueError(f"{self.path}: 未対応のフォーマット版 {ver} > {FORMAT_VERSION}")
        # 将来ヘッダが伸びたときのために宣言長ぶん読み飛ばす（前方互換）
        if hdr_len > HEADER_SIZE:
            self._f.read(hdr_len - HEADER_SIZE)
        self.header = FileHeader(ver, hdr_len, t0_mono, t0_unix)

        self.meta: dict[str, Any] = {}
        self._peeked: LogRecord | None = None
        first = self._read_one()
        if first is None:
            pass
        elif first.kind == Kind.META:
            self.meta = first.json()
        else:
            self._peeked = first   # META 無しのログも読めるようにしておく

    # ── イテレーション ──

    def __iter__(self) -> Iterator[LogRecord]:
        if self._peeked is not None:
            rec, self._peeked = self._peeked, None
            yield rec
        while True:
            rec = self._read_one()
            if rec is None:
                return
            yield rec

    def _read_one(self) -> LogRecord | None:
        raw = self._f.read(REC_HEADER_SIZE)
        if len(raw) < REC_HEADER_SIZE:
            self.truncated = len(raw) > 0
            return None
        kind, ptype, seq, _flags, t_ns, n = _S_REC.unpack(raw)
        payload = self._f.read(n) if n else b""
        if len(payload) < n:
            self.truncated = True
            return None
        return LogRecord(kind, t_ns, ptype, seq, payload)

    # ── 便利メソッド ──

    def rel_s(self, t_ns: int) -> float:
        """記録開始からの経過秒。"""
        return (t_ns - self.header.t0_mono_ns) / 1e9

    def frames(self, kind: int | None = None) -> Iterator[LogRecord]:
        """RX/TX フレームだけを流す。kind を渡せば片方向だけ。"""
        for rec in self:
            if rec.is_frame and (kind is None or rec.kind == kind):
                yield rec

    def close(self) -> None:
        self._f.close()

    def __enter__(self) -> "FrameLogReader":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
