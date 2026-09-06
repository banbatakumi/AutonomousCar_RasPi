"""頂点ベースのスケッチ — Fusion のスケッチのように頂点をクリック/ドラッグでつないで
ループを作る。`sim/editor.py` の中心線モード・壁モード共通のジオメトリ層。

## 円弧は「直線 + 接線円弧」のペア

ドラッグ操作は次のように解釈する（Fusion のスケッチの line/arc ツールと同じ）。

    クリック          → 直前の頂点からの直線。終点はクリック位置
    ドラッグ          → 直前の頂点から**ドラッグ開始位置までは直線**、
                        そこから**ドラッグを離した位置まで接線円弧**

つまり1回のドラッグは頂点を2つ（ドラッグ開始位置・終点）増やし、辺を2本
（直線・円弧）足す。円弧はドラッグ開始位置における直線の向きに**必ず接する**
ので、始点の向きを自由に選べる3点円弧と違って「どちら向きの弧にするか」の
曖昧さが無い——始点・終点・接線方向の3条件から円が一意に決まる
（`_tangent_arc_geometry`）。これにより円弧は常に直前の辺と滑らかにつながり、
円弧の手前に `turn` を挟む必要が無い（`turn` が要るのは直線同士の鋭角な
継ぎ目だけ）。

## 直線同士の折れ角は `["turn", deg]` で表現する

`sim/track.py` の `path` 形式は元々「直線は長さだけ・向きが変わるのは円弧だけ」という
前提（並びに沿ってタートルグラフィクスのように進む）だった。だが頂点を自由にクリック
して作る折れ線は、円弧を挟まず直線同士が鋭角に折れることが普通にある（矩形コース等）。
この継ぎ目は点を打たずその場で向きだけ変える `turn` 区間（`sim/track.py::centerline()`
に追加済み）で表現する——`straight`/`arc` だけの語彙を保つより、任意の折れ線を
そのまま表現できる方が素直だと判断した。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import track

__all__ = ["Edge", "Loop", "snap_to_grid", "snap_to_vertex",
           "sample_loop", "loop_to_path", "path_to_loop"]

Point = tuple[float, float]

#: これ未満の弧長変化・移動量は「実質ゼロ」として直線扱いにする（ゼロ割対策）
_DEGENERATE = 1e-9


@dataclass
class Edge:
    kind: str                          #: "line" か "arc"


@dataclass
class Loop:
    """頂点+辺の順序付きループ。`edges[i]` は `vertices[i] -> vertices[i+1]`。
    `closed` なら末尾の辺が `vertices[-1] -> vertices[0]` を結ぶ（**重複する
    始点は持たない**——`vertices` には始点を1回しか入れない）。

    `start_heading` は `path_to_loop()` で復元したループにだけ入る。**最初の辺が
    円弧のとき**、その接線元になる「直前の辺」が無いので、元ファイルの
    `origin` の向きをここに保持しておく（`loop_to_path()` 参照）。
    エディタが素で作るループは常に最初の辺が直線になる（ドラッグは必ず
    「直線→円弧」の順で頂点を増やすため）ので、`None` のままでよい。
    """

    vertices: list[Point] = field(default_factory=list)
    edges: list[Edge] = field(default_factory=list)
    start_heading: float | None = None

    def __len__(self) -> int:
        return len(self.vertices)

    @property
    def closed(self) -> bool:
        return bool(self.vertices) and len(self.edges) == len(self.vertices)

    def add_line(self, pt: Point) -> None:
        if self.vertices:
            self.edges.append(Edge("line"))
        self.vertices.append(pt)

    def add_arc(self, end: Point) -> None:
        if not self.vertices:
            raise ValueError("最初の頂点は直線でしか置けません")
        self.edges.append(Edge("arc"))
        self.vertices.append(end)

    def close(self, as_arc: bool = False) -> None:
        """終点を始点へつないで閉じる。`as_arc=True` ならその区間を接線円弧にする。"""
        if len(self.vertices) < 2:
            raise ValueError("閉じるには最低2頂点必要です")
        if self.closed:
            raise ValueError("すでに閉じています")
        self.edges.append(Edge("arc" if as_arc else "line"))

    def undo(self) -> None:
        """直前の操作を取り消す。閉じた直後なら閉じるのをやめ、辺があれば
        その辺と終点の頂点を消し、頂点1個だけならそれも消す。
        """
        if self.closed:
            self.edges.pop()
        elif self.edges:
            self.edges.pop()
            self.vertices.pop()
        elif self.vertices:
            self.vertices.pop()


def _tangent_arc_geometry(v_from: Point, v_to: Point, tangent_in: float
                          ) -> tuple[float, list] | None:
    """`v_from` で向き `tangent_in` に接し、`v_to` を通る円弧の
    (出口の向き[rad], path区間の列)。円弧にできない（`v_to` が接線の延長上に
    ある、または `v_from`==`v_to`）なら `None`——呼び出し側は直線にフォールバック
    すること。

    円の中心は「`v_from` における接線と直交する」という条件から
    `center = v_from + k * n`（`n` は接線を+90°回した左法線）の形に限られる。
    これに「`v_to` も同じ円周上にある」を足すと `k` が一次方程式で一意に決まる
    （`k` の符号がそのまま左右どちら向きの弧かを表す——`track.py::centerline()`
    の円弧が同じ式 `center = start ∓ r*n` で組み立てられているのと同じ規約）。
    """
    nx, ny = -math.sin(tangent_in), math.cos(tangent_in)
    vx, vy = v_to[0] - v_from[0], v_to[1] - v_from[1]
    v_sq = vx * vx + vy * vy
    dot_nv = nx * vx + ny * vy
    if v_sq < _DEGENERATE or abs(dot_nv) < _DEGENERATE:
        return None

    k = v_sq / (2.0 * dot_nv)
    r = abs(k)
    cx, cy = v_from[0] + k * nx, v_from[1] + k * ny
    a0 = math.atan2(v_from[1] - cy, v_from[0] - cx)
    a1 = math.atan2(v_to[1] - cy, v_to[0] - cx)
    if k > 0:
        delta = (a1 - a0) % (2 * math.pi)          # 反時計回り
    else:
        delta = -((a0 - a1) % (2 * math.pi))       # 時計回り
    if abs(delta) < 1e-6:
        return None
    exit_ = tangent_in + delta
    return exit_, [["arc", r, math.degrees(delta)]]


def _edge_geoms(loop: Loop) -> list[tuple[float, float, list]]:
    """各辺の (入りの向き[rad], 出の向き[rad], path区間の列) を先頭から順に計算する。

    円弧は直前の辺の出口の向きに必ず接するので、直線と違って単独では決まらない
    （`_tangent_arc_geometry` の `tangent_in` が要る）。**最初の辺**だけは
    「直前の辺」が無いので、`loop.start_heading`（`path_to_loop()` 由来）か、
    直線ならそれ自身の向きから決める。エディタが素で作るループは最初の辺が
    必ず直線になるため、この分岐が問題になるのは既存ファイルの読み込みだけ。
    """
    n = len(loop.vertices)
    edge_count = len(loop.edges)
    geoms: list[tuple[float, float, list]] = []
    heading = loop.start_heading
    for i in range(edge_count):
        v_from = loop.vertices[i % n]
        v_to = loop.vertices[(i + 1) % n]
        if loop.edges[i].kind == "arc":
            if heading is None:
                raise ValueError(
                    "最初の辺が円弧のループは start_heading が必要です（path_to_loop 経由で復元してください）")
            arc = _tangent_arc_geometry(v_from, v_to, heading)
            if arc is not None:
                exit_, segs = arc
                geoms.append((heading, exit_, segs))
                heading = exit_
                continue
            # 縮退（延長線上の点）: 直線として扱う
        enter = exit_ = math.atan2(v_to[1] - v_from[1], v_to[0] - v_from[0])
        length = math.hypot(v_to[0] - v_from[0], v_to[1] - v_from[1])
        geoms.append((enter, exit_, [["straight", length]]))
        heading = exit_
    return geoms


def loop_to_path(loop: Loop) -> tuple[tuple[float, float, float], list]:
    """頂点/円弧ループ → `sim/track.py` のタートル形式 `(origin, path)`。

    直線同士の折れ角は `["turn", deg]` を挟んで表現する（モジュール docstring 参照）。
    円弧は常に直前の辺と滑らかにつながる（`_edge_geoms`）ので、円弧の手前に
    `turn` が入ることはない。

    **閉ループの始点の向きは「最後の辺を抜けた向き」にする。** 単純に辺0の向きを
    始点の向きにすると、始点＝終点でも「そこで鋭角に折れる」コースでは
    `pts[0].yaw` と `pts[-1].yaw` が一致せず、`sim/track.py::build()` の
    loop 警告（向きが合っていない）が誤検知する。始点の向きを「1つ前の辺
    （＝最後の辺）から到着した向き」にそろえておけば、辺0の手前にも他の頂点と
    同じように `turn` が挟まり、`pts[-1].yaw` は定義上 `pts[0].yaw` と一致する
    （閉じていないループはこの補正が意味を持たないので辺0自身の向きのまま）。
    """
    n = len(loop.vertices)
    edge_count = len(loop.edges)
    if n < 2 or edge_count == 0:
        raise ValueError("2頂点・1辺以上必要です")

    geoms = _edge_geoms(loop)

    if loop.start_heading is not None:
        origin_yaw = loop.start_heading
    elif loop.closed:
        origin_yaw = geoms[-1][1]
    else:
        origin_yaw = geoms[0][0]

    path: list = []
    heading = origin_yaw
    for enter, exit_, segs in geoms:
        dturn = math.degrees((enter - heading + math.pi) % (2 * math.pi) - math.pi)
        if abs(dturn) > 1e-6:
            path.append(["turn", dturn])
        path.extend(segs)
        heading = exit_

    ox, oy = loop.vertices[0]
    return (ox, oy, origin_yaw), path


def path_to_loop(origin: tuple[float, float, float], path: list) -> Loop:
    """`loop_to_path()` の逆変換。既存コース（centerline/wall どちらの `path` 形式も
    共通）をエディタで再編集するために使う。`path` の円弧は既に半径・角度が
    明示されているので、ここでは単純に前へ進めるだけでよい（円が一意に決まる
    条件を解く必要はない——それは `loop_to_path()` の仕事）。
    """
    x, y, yaw = origin
    loop = Loop(start_heading=yaw)
    loop.add_line((x, y))
    for seg in path:
        kind = seg[0]
        if kind == "straight":
            length = float(seg[1])
            nx, ny = x + math.cos(yaw) * length, y + math.sin(yaw) * length
            loop.add_line((nx, ny))
            x, y = nx, ny
        elif kind == "arc":
            r = float(seg[1])
            deg = float(seg[2])
            rad = math.radians(deg)
            s = 1.0 if deg >= 0 else -1.0
            cx = x - s * r * math.sin(yaw)
            cy = y + s * r * math.cos(yaw)
            a0 = yaw - s * math.pi / 2
            nx, ny = cx + r * math.cos(a0 + rad), cy + r * math.sin(a0 + rad)
            loop.add_arc((nx, ny))
            x, y, yaw = nx, ny, yaw + rad
        elif kind == "turn":
            yaw = yaw + math.radians(float(seg[1]))
        else:
            raise ValueError(f"未知の区間: {seg!r}")

    # 閉ループなら終点は始点とほぼ一致するはず。重複を畳んで `closed` にする
    if len(loop.vertices) > 1:
        gap = math.hypot(loop.vertices[-1][0] - loop.vertices[0][0],
                         loop.vertices[-1][1] - loop.vertices[0][1])
        if gap < 0.05:
            loop.vertices.pop()
    return loop


def sample_loop(loop: Loop, step: float) -> np.ndarray:
    """ループを世界座標の点列 `(N,3)`（x, y, 向き）へサンプリングする。
    プレビュー描画・壁のラスタライズの両方がこれ経由で点を得る——
    保存フォーマット（`loop_to_path`）と実際に描画される形が食い違う心配がない。
    """
    origin, path = loop_to_path(loop)
    return track.centerline(path, origin[0], origin[1], origin[2], step)


def snap_to_grid(pt: Point, spacing: float) -> Point:
    """グリッド間隔へスナップ。`spacing <= 0` ならそのまま返す（スナップOFF）。"""
    if spacing <= 0:
        return pt
    return (round(pt[0] / spacing) * spacing, round(pt[1] / spacing) * spacing)


def snap_to_vertex(pt_screen: Point, vertices_screen: list[Point],
                   pixel_radius: float) -> int | None:
    """`pt_screen` に最も近い頂点が `pixel_radius` [px] 以内にあればその添字を返す。

    **スクリーン座標で判定する。** ワールド座標の固定半径にすると、ズームで
    見た目のスナップ範囲が変わってしまう（近くに見えるのに吸着しない/その逆）。
    """
    best_i: int | None = None
    best_d2 = pixel_radius * pixel_radius
    for i, v in enumerate(vertices_screen):
        d2 = (v[0] - pt_screen[0]) ** 2 + (v[1] - pt_screen[1]) ** 2
        if d2 <= best_d2:
            best_d2, best_i = d2, i
    return best_i
