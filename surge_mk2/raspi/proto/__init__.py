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

__all__ = [
    "Frame",
    "FrameEncoder",
    "FrameParser",
    "RxStats",
    "build_frame",
    "crc16_ccitt",
    "packets",
]
