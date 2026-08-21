"""STM32 との UART リンク。pyserial + フレーミングの薄いラッパ。

単一スレッドのループから poll()/send() を叩く前提（ロック無し）。
受信時刻 T4 と送信時刻 T1 を ns 単位で捉え、時刻同期に渡せるようにする。
"""

from __future__ import annotations

import time
from collections.abc import Iterable
from typing import NamedTuple

import serial

from ..core.cleanup import quiet_close
from ..proto import FrameEncoder, FrameParser
from ..proto.generated.packets import HEADER_SIZE, S2P_TYPES


class RxFrame(NamedTuple):
    type: int
    seq: int
    payload: bytes
    rx_ns: int          # この読み取りが完了した Pi 単調時刻

    def decode(self):
        from ..proto import packets
        cls = packets.BY_TYPE.get(self.type)
        return cls.decode(self.payload) if cls is not None else None


class SerialLink:
    """UART の開閉・送受信を担う。

    :param port: 通常 ``/dev/serial0``（GPIO14/15）。名前直書きは避ける
    :param read_timeout: read のブロッキング上限 [s]。ループの応答性を決める

    :ivar on_tx: 送信フックを差すと ``(t_ns, type, seq, payload)`` で呼ばれる。
        送信フレームの SEQ はエンコーダの内部にしか無く、呼び出し側からは
        見えない。記録は横断的な関心事なのでコールバックで外に出す。
    """

    def __init__(self, port: str = "/dev/serial0", baud: int = 250_000,
                 read_timeout: float = 0.005) -> None:
        self._ser = serial.Serial(port, baud, timeout=read_timeout)
        self._parser = FrameParser(expect_types=S2P_TYPES)
        self._enc = FrameEncoder()
        self.port = port
        self.baud = baud
        self.on_tx = None

    # ── 受信 ──

    def poll(self) -> list[RxFrame]:
        """来ているバイトを読み、復元できたフレームを返す。

        read は最低1バイトを read_timeout まで待つ。データが流れている間は
        in_waiting 分をまとめて読むので syscall 数を抑えられる。
        """
        n = self._ser.in_waiting
        data = self._ser.read(n if n else 1)
        rx_ns = time.monotonic_ns()
        if not data:
            return []
        return [RxFrame(f.type, f.seq, f.payload, rx_ns)
                for f in self._parser.feed(data)]

    @property
    def stats(self):
        return self._parser.stats

    # ── 送信 ──

    def send(self, packet) -> int:
        """パケットを送り、1バイト目を書き込む直前の Pi 時刻 [ns] を返す。

        PING の T1 に使う。write は OS バッファに積むだけなので厳密な線上時刻では
        ないが、Python 層で取れる最善。
        """
        return self._write(self._enc.encode(packet))

    def send_raw(self, pkt_type: int, payload: bytes = b"") -> int:
        return self._write(self._enc.encode_raw(pkt_type, payload))

    def _write(self, frame: bytes) -> int:
        t_ns = time.monotonic_ns()
        self._ser.write(frame)
        if self.on_tx is not None:
            # frame = SYNC(2) TYPE SEQ LEN PAYLOAD CRC(2)
            self.on_tx(t_ns, frame[2], frame[3], frame[HEADER_SIZE:HEADER_SIZE + frame[4]])
        return t_ns

    # ── 後始末 ──

    def reset_input(self) -> None:
        self._ser.reset_input_buffer()
        self._parser.reset()

    def close(self) -> None:
        with quiet_close(f"SerialLink の pyserial ({self._ser.port})"):
            self._ser.close()

    def __enter__(self) -> "SerialLink":
        return self

    def __exit__(self, *exc: Iterable) -> None:
        self.close()
