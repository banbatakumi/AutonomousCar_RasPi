"""BackendBase 再エクスポート。

抽象クラスの定義そのものは core.interfaces に集約している。バックエンド実装は
このモジュール経由でインポートすることで依存方向を明確にする。
"""

from core.interfaces import BackendBase

__all__ = ["BackendBase"]
