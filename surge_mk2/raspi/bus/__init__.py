"""内部バス。

- **画像**は共有メモリリング（`shm_ring`）。ZMQ には参照だけを流す
- **それ以外**は ZeroMQ PUB/SUB（`zbus`）

`zbus` は pyzmq / msgspec を要求する。`shm_ring` は stdlib のみで動くので、
**pyzmq が無い環境でも `from raspi.bus import FrameRing` は通る**ようにしてある
（io_node は診断時に外部依存なしで動かせることに意味がある）。
"""

from .shm_ring import FrameDesc, FrameRef, FrameRing

__all__ = ["FrameDesc", "FrameRef", "FrameRing"]

try:                                        # pyzmq / msgspec があるときだけ
    from .zbus import (                     # noqa: F401
        LATEST,
        RELIABLE,
        Policy,
        Publisher,
        Subscriber,
        endpoint_for_node,
        endpoints_for_topic,
    )
except ImportError:                         # pragma: no cover - 環境依存
    HAS_ZBUS = False
else:
    HAS_ZBUS = True
    __all__ += ["Publisher", "Subscriber", "Policy", "LATEST", "RELIABLE",
                "endpoint_for_node", "endpoints_for_topic"]

__all__ += ["HAS_ZBUS"]
