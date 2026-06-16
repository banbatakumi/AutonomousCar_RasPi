"""BackendBase 抽象クラス。

実体は core/interfaces.py に定義されている契約をそのまま再公開する。
バックエンド実装（real.py / sim/）はここから継承する。
"""
from core.interfaces import BackendBase

__all__ = ["BackendBase"]
