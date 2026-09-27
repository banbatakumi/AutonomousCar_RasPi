"""UART プロトコル。

パケット定義は protocol.toml が唯一の正で、generated/ 以下は生成物。
定義を変えたら ``python3 raspi/proto/generate.py`` で再生成すること。
"""

from .framing import (
    Frame,
    FrameEncoder,
    FrameParser,
    RxStats,
    build_frame,
    crc16_ccitt,
)
from .generated.packets import *  # noqa: F401,F403
from .generated import packets

#: STM32 との UART のボーレート [bps]（8N1）。**ファームの USART1 と必ず一致させる**
#: （MainF446RE_V3 `Core/Src/usart.c`）。2026-09-26 に 250000 → 1000000（遅延の短縮。
#: APB2 45MHz で USARTDIV=2.8125、誤差0%）。実機・シム・補助ツールはここを参照する
UART_BAUD = 1_000_000

__all__ = [
    "UART_BAUD",
    "Frame",
    "FrameEncoder",
    "FrameParser",
    "RxStats",
    "build_frame",
    "crc16_ccitt",
    "packets",
]
