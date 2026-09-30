"""自己位置推定・地図・経路生成の部品（`docs/architecture.md` §14 Phase 3/4）。

**ここは純粋計算だけ。** バスも WebSocket も GUI も設定ファイルも知らない
（`raspi/auto/__init__.py` の約束と同じ）。入力は `raspi/msgs/types.py` の
メッセージと numpy 配列、出力も数値だけ。

    deskew      点群のモーションスキュー補正（走りながら測った1周を1瞬に直す）
    grid        占有格子。ヒット/ミスの回数を数え、動く物を壁として確定させない
    raceline    最小曲率最適化と速度プロファイル（車体非依存の共通ライブラリ）

**SLAM 本体は `slam2d/`（車体非依存ライブラリ）に統合済み。** ここにあった
自作の `scanmatch`/`slam`（旧 `auto/raceline.py` 用）は学習用の基礎基盤で、
現行の `slam2d_raceline`/`slam2d_route` planner には使われておらず削除した。

**このパッケージの `__init__.py` はサブモジュールを再エクスポートしない**
（issue #22）。`raspi.nav.roadgraph`（cv2使用箇所を除き軽量）や
`raspi.nav.ipm`（単体0.3ms）だけを使うノードでも、`from raspi.nav import ...`
と書くだけで全サブモジュールが芋づる式に読み込まれてしまうため。
使う側はサブモジュールを直接 import すること::

    from raspi.nav.grid import OccGrid, dilate, pack_trinary
    from raspi.nav.deskew import deskew, Points
    from raspi.nav.centerline import Centerline
    from raspi.nav.obstacles import Obstacle
    from raspi.nav.ipm import CameraExtrinsics, ...
"""

from __future__ import annotations
