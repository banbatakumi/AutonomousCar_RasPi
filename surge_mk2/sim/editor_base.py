"""コースエディタ（`sim/editor.py`）の定数と、モードごとの部品が共有する小物。

`sim/editor.py` をモードごと（`editor_centerline`・`editor_wall`・`editor_image`）に分けたとき、
どのモードからも読む定数をここへ寄せた（循環 import を避けるため）。
"""

from __future__ import annotations

import math

INIT_W, INIT_H = 1400, 900
MIN_W, MIN_H = 900, 620
PANEL_W = 340

#: グリッドスナップの間隔の選択肢 [m]
GRID_SPACINGS = (0.25, 0.5, 1.0)
#: 頂点スナップの半径（スクリーン座標 [px]）。**常時ON**——ループを閉じるための機能
VERTEX_SNAP_PX = 14.0
#: これを超えて動かしたら「クリック」ではなく「ドラッグ（円弧）」とみなす[px]
DRAG_ARC_PX = 6.0

DEFAULT_WIDTH = 1.0
WIDTH_STEP = 0.05
CENTER_RESOLUTION = 0.02

DEFAULT_WALL_THICKNESS = 0.03
WALL_THICKNESS_STEP = 0.01
WALL_RESOLUTION = 0.02
WALL_MARGIN = 0.4

IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".bmp", ".webp", ".gif")
#: 画像モードの既定値。壁の外形の幅 [m]（読み込み直後の仮の縮尺。実寸は人が合わせる）
IMG_DEFAULT_WIDTH_M = 10.0
IMG_TOOLS = (("eyedrop", "スポイト"), ("blob", "消す(塊)"),
             ("erase", "消す(矩形)"), ("fill", "塗る(矩形)"))

#: 障害物の直径の選択肢 [m]
OBSTACLE_DIAMETERS = (0.1, 0.2, 0.3, 0.5)

#: 何も描いていないときの表示範囲 [m]（原点付近）。長い直線も引けるよう、
#: 頂点ぴったりよりだいぶ広めに取る
_EMPTY_BOUNDS = (-2.0, -2.0, 10.0, 8.0)
_BOUNDS_MARGIN = 0.6

#: マウスホイール1ノッチあたりの拡大率
_ZOOM_STEP = 1.15
#: 拡大率の範囲 [px/m]
_MIN_SCALE = 8.0
_MAX_SCALE = 250.0


def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Btn:
    __slots__ = ("rect", "kind", "value", "label")

    def __init__(self, rect, kind, value, label):
        self.rect, self.kind, self.value, self.label = rect, kind, value, label
