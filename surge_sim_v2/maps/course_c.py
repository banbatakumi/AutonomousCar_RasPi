"""高難度コース（ロングストレート＋スイーパー＋シケイン＋エセスの大型周回路）。

中心線を平滑化して左右に半幅オフセットし壁を自動生成（_track.build_loop）。
"""
from __future__ import annotations

from ._track import build_loop

COURSE_NAME = "Grand Circuit"

_CENTER_PTS = [
    (1.5, 1.0),
    (6.0, 0.9),    # ロングストレート（下）
    (7.3, 1.8),    # 右スイーパー
    (7.3, 3.2),
    (6.2, 3.8),    # 右シケイン（S字）
    (7.2, 4.6),
    (6.3, 5.6),    # 右上コーナー
    (4.5, 5.8),    # 上ストレート
    (3.2, 5.0),    # エセス（S字）
    (4.0, 4.2),
    (2.8, 3.7),
    (1.5, 4.3),    # 左スイーパー
    (0.8, 3.0),    # 左ストレート
    (1.4, 1.7),    # スタート手前へ
]

WALLS, START_POSE, CENTER_LINE = build_loop(_CENTER_PTS, width=1.0)
