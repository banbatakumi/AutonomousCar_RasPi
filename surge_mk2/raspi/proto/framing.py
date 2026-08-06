"""UART フレームの組み立てと抽出。

フレーム構造（docs/uart_protocol.md §3）::

    0xAA 0x55 | TYPE | SEQ | LEN | PAYLOAD (LEN bytes) | CRC16 (LE)
     0    1      2      3     4     5                    5+LEN

CRC 計算範囲は TYPE から PAYLOAD 末尾まで（LEN + 3 バイト）。

パケットの中身の定義は generated/packets.py（protocol.toml から生成）にあり、
このモジュールは中身を知らずにフレーミングだけを担当する。
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import NamedTuple

from .generated.packets import (
    BY_TYPE,
    CRC_CHECK_VALUE,
    EXPECTED_LEN,
    FRAME_OVERHEAD,
    HEADER_SIZE,
    MAX_PAYLOAD,
    SYNC,
)

__all__ = [
    "Frame",
    "FrameParser",
    "FrameEncoder",
    "RxStats",
    "crc16_ccitt",
    "build_frame",
]


# ── CRC-16/CCITT-FALSE ──────────────────────────────────────────────────

def _make_crc_table() -> tuple[int, ...]:
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _make_crc_table()


def crc16_ccitt(data: bytes | bytearray | memoryview) -> int:
    """CRC-16/CCITT-FALSE。poly=0x1021 init=0xFFFF 反転なし xorout=0x0000。

    検査値: ``crc16_ccitt(b"123456789") == 0x29B1``
    """
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ _CRC_TABLE[(crc >> 8) ^ b]
    return crc


# 実装ミスに即座に気づけるよう import 時に検証する。
# ここを間違えると通信が一切成立しないので、遅延させる意味がない。
assert crc16_ccitt(b"123456789") == CRC_CHECK_VALUE, "CRC 実装が仕様と一致しません"


# ── 組み立て ────────────────────────────────────────────────────────────

def build_frame(pkt_type: int, seq: int, payload: bytes = b"") -> bytes:
    """1フレームを組み立てる。"""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload が長すぎます: {len(payload)} > {MAX_PAYLOAD}")
    body = bytes((pkt_type, seq & 0xFF, len(payload))) + payload
    crc = crc16_ccitt(body)
    return SYNC + body + bytes((crc & 0xFF, crc >> 8))


class FrameEncoder:
    """送信側。SEQ のローリングカウンタを持つ。

    SEQ は送信方向ごとに1本の通し番号で、受信側はこれの欠番でロスを検出する。
    """

    __slots__ = ("_seq",)

    def __init__(self, start_seq: int = 0) -> None:
        self._seq = start_seq & 0xFF

    @property
    def seq(self) -> int:
        """次に送る SEQ。"""
        return self._seq

    def encode(self, packet) -> bytes:
        """パケットの dataclass をフレームにする。"""
        return self.encode_raw(packet.TYPE, packet.encode())

    def encode_raw(self, pkt_type: int, payload: bytes = b"") -> bytes:
        frame = build_frame(pkt_type, self._seq, payload)
        self._seq = (self._seq + 1) & 0xFF
        return frame


# ── 抽出 ────────────────────────────────────────────────────────────────

class Frame(NamedTuple):
    """CRC 検証を通ったフレーム1個。"""

    type: int
    seq: int
    payload: bytes

    def decode(self):
        """対応する dataclass に変換する。未知の TYPE なら None。"""
        cls = BY_TYPE.get(self.type)
        return cls.decode(self.payload) if cls is not None else None


@dataclass(slots=True)
class RxStats:
    """受信側の統計。STM32 側の STATS パケットと並べて GUI に出す。

    片方向だけ増えるのか両方向なのかで原因の切り分けが変わるため、
    上り下りの両方を見られるようにしておくことに意味がある。
    """

    frame_ok: int = 0
    crc_error: int = 0
    len_error: int = 0
    unknown_type: int = 0
    wrong_direction: int = 0
    packet_loss: int = 0
    resync_bytes: int = 0  #: SYNC を探して読み飛ばしたバイト数

    def as_dict(self) -> dict[str, int]:
        return {f.name: getattr(self, f.name) for f in self.__dataclass_fields__.values()}


@dataclass(slots=True)
class FrameParser:
    """バイトストリームからフレームを切り出す。

    1バイトずつ read すると syscall が支配的になるため、呼び出し側は
    ``ser.read(max(1, ser.in_waiting))`` のように塊で読んで feed すること。

    :param expect_types: 受信を許可する TYPE 集合（方向規約の検査用）。
        None なら検査しない。
    """

    expect_types: frozenset[int] | None = None
    stats: RxStats = field(default_factory=RxStats)
    _buf: bytearray = field(default_factory=bytearray, repr=False)
    _last_seq: int | None = field(default=None, repr=False)

    def feed(self, data: bytes) -> list[Frame]:
        """受信バイト列を投入し、取り出せたフレームを返す。"""
        self._buf.extend(data)
        return list(self._extract_all())

    def reset(self) -> None:
        """バッファと SEQ 追跡をクリアする（再接続時に呼ぶ）。"""
        self._buf.clear()
        self._last_seq = None

    # ── 内部 ──

    def _extract_all(self) -> Iterator[Frame]:
        while True:
            frame = self._extract_one()
            if frame is _NEED_MORE:
                return
            if frame is not None:
                yield frame

    def _extract_one(self):
        """1フレーム分の処理を進める。

        戻り値: Frame / None（破棄したので続行）/ _NEED_MORE（データ待ち）
        """
        buf = self._buf

        idx = buf.find(SYNC)
        if idx < 0:
            # SYNC の1バイト目だけ来ている可能性があるので末尾1バイトは残す
            drop = max(0, len(buf) - 1)
            if drop:
                del buf[:drop]
                self.stats.resync_bytes += drop
            return _NEED_MORE
        if idx > 0:
            del buf[:idx]
            self.stats.resync_bytes += idx

        if len(buf) < HEADER_SIZE:
            return _NEED_MORE

        pkt_type, seq, plen = buf[2], buf[3], buf[4]

        known = pkt_type in EXPECTED_LEN
        if known:
            expected = EXPECTED_LEN[pkt_type]
            # expected が None なのは可変長（LOG）のみ
            if expected is not None and plen != expected:
                self.stats.len_error += 1
                return self._discard_sync()

        total = FRAME_OVERHEAD + plen
        if len(buf) < total:
            return _NEED_MORE

        if crc16_ccitt(buf[2:HEADER_SIZE + plen]) != (buf[5 + plen] | buf[6 + plen] << 8):
            self.stats.crc_error += 1
            return self._discard_sync()

        payload = bytes(buf[HEADER_SIZE:HEADER_SIZE + plen])
        del buf[:total]

        # CRC が通った時点で SEQ は信用できる。未知の TYPE でもロス計上には使う。
        if self._last_seq is not None:
            self.stats.packet_loss += (seq - self._last_seq - 1) & 0xFF
        self._last_seq = seq

        if not known:
            self.stats.unknown_type += 1
            return None
        if self.expect_types is not None and pkt_type not in self.expect_types:
            self.stats.wrong_direction += 1
            return None

        self.stats.frame_ok += 1
        return Frame(pkt_type, seq, payload)

    def _discard_sync(self):
        """壊れたフレームの SYNC を捨てる。

        2バイトだけ捨てるのがポイント。フレーム全体を読み飛ばすと、
        壊れた LEN を信用したことになり、その中に埋もれた本物の SYNC を見逃す。
        """
        del self._buf[:len(SYNC)]
        return None


class _NeedMore:
    __slots__ = ()

    def __repr__(self) -> str:  # pragma: no cover
        return "<NEED_MORE>"


_NEED_MORE = _NeedMore()
