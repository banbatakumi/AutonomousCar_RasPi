"""L字型コース。

L字に折れ曲がった周回コース。コース幅は全周 1m。
外周・内周ともにL字輪郭を成し、内周は外周から 1m 内側にオフセットしている。
"""

COURSE_NAME = "L-Shape Course"


def _segments(points):
    """頂点列(閉ループ)を線分リストへ変換する。"""
    segs = []
    for i in range(len(points)):
        segs.append((points[i], points[(i + 1) % len(points)]))
    return segs


# 外周L字輪郭（反時計回り）
_OUTER_PTS = [
    (0.0, 0.0),
    (5.0, 0.0),
    (5.0, 3.0),
    (3.0, 3.0),
    (3.0, 5.0),
    (0.0, 5.0),
]

# 内周L字輪郭（外周から1m内側）
_INNER_PTS = [
    (1.0, 1.0),
    (4.0, 1.0),
    (4.0, 2.0),
    (2.0, 2.0),
    (2.0, 4.0),
    (1.0, 4.0),
]

WALLS = _segments(_OUTER_PTS) + _segments(_INNER_PTS)

# 下側ストレート中央（y=0〜1の通路の中心）、East向き
START_POSE = (2.5, 0.5, 0.0)  # x[m], y[m], heading[deg]

# コース中心線（経路追従用のウェイポイント、反時計回りの閉ループ）
# 外周から0.5m内側 = コース幅1mの中央を通るL字ループ
CENTER_LINE = [
    (0.5, 0.5),
    (4.5, 0.5),
    (4.5, 2.5),
    (2.5, 2.5),
    (2.5, 4.5),
    (0.5, 4.5),
]
