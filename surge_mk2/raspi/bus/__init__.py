"""内部バス。

画像は共有メモリリング（`shm_ring`）、それ以外は ZeroMQ の予定
（`docs/architecture.md` §6.2）。ZeroMQ ラッパはまだ無い。
"""

from .shm_ring import FrameDesc, FrameRef, FrameRing

__all__ = ["FrameDesc", "FrameRef", "FrameRing"]
