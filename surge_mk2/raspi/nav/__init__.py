"""自己位置推定・地図・経路生成の部品（`docs/architecture.md` §14 Phase 3/4）。

**ここは純粋計算だけ。** バスも WebSocket も GUI も設定ファイルも知らない
（`raspi/auto/__init__.py` の約束と同じ）。入力は `raspi/msgs/types.py` の
メッセージと numpy 配列、出力も数値だけ。

    deskew      点群のモーションスキュー補正（走りながら測った1周を1瞬に直す）
    grid        占有格子。ヒット/ミスの回数を数え、動く物を壁として確定させない
    scanmatch   相関スキャンマッチ。地図に対して今の点群が一番合う姿勢を探す
    slam        上3つの束ね。**SLAM エンジンを隠すのはここだけ**

`raspi/auto/` との分担は「1 planner = 1つの走らせ方」を保つため。planner
（`raspi/auto/raceline.py`）はこれらを組み合わせる状態機械であって、
アルゴリズムそのものはここに置く。

## なぜ外部の SLAM ライブラリを使わないか

BreezySLAM（CoreSLAM）を検討して**採らなかった**。理由は導入の手間ではなく、
中身がこのプロジェクトの安全規約と噛み合わないこと。

- 誤差勾配が取れないと `exit(1)` でプロセスを殺す。走行中に planning_node が
  traceback も残さず消えるのは車載コードとして受け入れられない
- `hole_width_mm/2` より近い点を捨てる仕様が、道幅 0.9m のコースと両立しない
- **`dist == 0` を「自由空間」として彫る。** `sector_seen=False`（受信できなかった
  30点）を渡すと、見えていない方向を「空き」として地図に書く。
  `raspi/auto/follow_the_gap.py:19-32` で確立した規約を SLAM 層でだけ破ることになる
- 内蔵の歪み補正が「添字が増える向きに時間が進む」前提で、こちらと逆
  （`raspi/msgs/types.py` の `Scan` 参照）

numpy で書いても 1周期 5〜15ms に収まる見込み（レイ彫り 0.5ms・スキャンマッチ 0.5ms）で、
10Hz に対して十分。**全部読めることの方が価値が大きい**と判断した。
"""

from __future__ import annotations

from . import centerline, obstacles
from .centerline import Centerline
from .deskew import Points, deskew
from .grid import OccGrid, dilate, pack_trinary
from .obstacles import Obstacle
from .purepursuit import Pursuit, PursuitConfig, follow
from .raceline import RaceLine, optimize
from .scanmatch import MatchResult, match
from .slam import Slam, SlamConfig, SlamUpdate

__all__ = [
    "Centerline",
    "MatchResult",
    "Obstacle",
    "OccGrid",
    "Points",
    "Pursuit",
    "PursuitConfig",
    "RaceLine",
    "Slam",
    "SlamConfig",
    "SlamUpdate",
    "centerline",
    "deskew",
    "dilate",
    "follow",
    "match",
    "obstacles",
    "optimize",
    "pack_trinary",
]
