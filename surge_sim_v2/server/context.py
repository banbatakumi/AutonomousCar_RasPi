"""アプリケーションコンテキスト。

main.py が組み立てた依存（SharedState / Controller / Logger / バックエンド /
コース一覧）を保持し、FastAPI ルート・ブロードキャスタから参照する。
モジュールグローバルに 1 つだけ生成して使う。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from core.controller import Controller
from core.logger import Logger, LogReplayer
from core.shared_state import SharedState


@dataclass
class AppContext:
    shared_state: SharedState
    controller: Controller
    logger: Logger
    courses: dict = field(default_factory=dict)
    is_sim: bool = True
    saved_maps_dir: str = "saved_maps"
    # 地図保存・読込・リセットのフック（Phase3 で SLAM と接続）
    on_map_save: Callable[[str], str] | None = None
    on_map_reset: Callable[[], None] | None = None
    on_map_load: Callable[[str], None] | None = None
    on_course_change: Callable[[str], None] | None = None


_ctx: AppContext | None = None


def set_context(ctx: AppContext) -> None:
    global _ctx
    _ctx = ctx


def get_context() -> AppContext:
    if _ctx is None:
        raise RuntimeError("AppContext が初期化されていません")
    return _ctx
