"""デフォルト長方形コース。

外周 6m x 4m、コース幅 1m の長方形リング。
外周壁と内周壁の両方を線分リストで定義する。座標単位 [m]。
"""

COURSE_NAME = "Default Rectangle"


def _rect_edges(x0, y0, x1, y1):
    """長方形 (x0,y0)-(x1,y1) の4辺を線分リストとして返す。"""
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


# 外周 6m x 4m、内周は各辺から 1m 内側（コース幅 1m）
WALLS = _rect_edges(0.0, 0.0, 6.0, 4.0) + _rect_edges(1.0, 1.0, 5.0, 3.0)

# 下側ストレート上、東向き（トラック方向に整列）
START_POSE = (1.5, 0.5, 0.0)

# Phase2 用のカンニング中心線（リングの中央 = 各辺から 0.5m）
CENTER_LINE = [
    (0.5, 0.5), (5.5, 0.5), (5.5, 3.5), (0.5, 3.5),
]
