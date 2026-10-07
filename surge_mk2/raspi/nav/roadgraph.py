"""占有格子 → 道路グラフ（分岐・合流のあるコースを経路として扱うための骨組み）。

`nav/centerline.py` は「地図作成で走った1周の軌跡」から中心線を作るので、
**走らなかった枝は経路に現れない**し、分岐・合流では法線のレイが枝の奥へ抜けて
道幅を過大に測る。ここでは地図全体から「どこが道で、どこで分かれるか」を
グラフとして取り出し、経路の選択（`nav/route.py`）を地図の上の探索問題にする。

## 作り方（GVD/スケルトン系。Oleynikova 2018・tuw_voronoi_graph・ForzaETH と同じ系統）

1. 3値地図の「空き」を取り出す。**未知は壁として扱う**（走ったことが無い場所に
   経路を引かない）。小さな未知の穴は closing で塞ぐ（壁は塞がない）
2. 壁（＋未知）までの距離 `d` を EDT で出し、`d >= keep`（車体中心が居られる
   領域＝C-space）だけを残す。**狭くて通れない道はここで自然に消える**
3. 出発点を含む連結成分だけを取り、Zhang-Suen で細線化する
4. 細線の画素を「分岐（近傍≥3）」「端点（近傍1）」「それ以外（エッジの途中）」
   に分け、エッジを辿って折れ線にする
5. **ひげ（短い行き止まり）を除き、近い分岐をまとめ、次数2の節点を消す**——
   細線化は壁の小さな凹凸ごとに枝を生やすので、これをしないと節点が数百になる

## ★ 階段の偽の分岐は「次数2の節点」として消す

Zhang-Suen の出力は斜めの階段で2画素幅になる箇所があり、そこでは8近傍の数が
3になる。交差数（周囲の 0→1 の回数）で判定すれば階段は避けられるが、**斜めに
接続する本物の T 字路まで取りこぼす**（toyota2 で分岐4つのうち大半が消えた）。
近傍数で広めに拾い、両側に鎖が1本ずつしか付かない候補（次数2）を
`_Builder._dissolve()` で消す方が確実。

## Zhang-Suen は numpy で自前実装

`cv2.ximgproc.thinning` は contrib 版にしか無く、開発機（Mac）の cv2 には無い。
2つの実装を環境で使い分けると、**同じ地図から違うグラフが出る**ので1本化した。
C-space は細いので反復は十数回で終わる（640×640 で数十ms）。

## 回廊ラベル（`label`）

空きの各セルを「いちばん近いエッジ」に割り当てたもの。経路が通らない枝の
セルを壁とみなす**回廊格子**（`CorridorGrid`）を作るのに使う（分岐の口で
道幅を過大に測らないため、`nav/route.py` 参照）。

ユークリッドの最近傍（`cv2.distanceTransformWithLabels`）だと**薄い壁の向こうの
エッジ**に割り当たることがあるので、空きセルの中だけを4近傍で膨張させる
（障害物を回り込む測地距離の近似）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# cv2 はここでは import しない（issue #22）。line_perception_node 等、道路グラフを
# 使わないノードもこのモジュールを import 経由で引き込みうるため、実際に使う
# 関数の中で import する（cv2 は onnxruntime 等と同じ扱い）。

__all__ = ["RoadGraph", "Edge", "Node", "build_graph", "CorridorGrid",
           "trinary_from_bool"]


@dataclass
class Node:
    xy: tuple[float, float]
    #: 交わるエッジの番号（自己ループは2回入る）
    edges: list[int] = field(default_factory=list)


@dataclass
class Edge:
    u: int                                 #: 始点の節点
    v: int                                 #: 終点の節点
    xy: np.ndarray                         #: (N, 2) u→v の折れ線 [m]
    length: float                          #: [m]
    #: 通ってよい向き。+1 = u→v のみ、-1 = v→u のみ、0 = どちらでも。
    #: 地図作成で走った向きから決める（逆走禁止、`_directions`）
    dir_allowed: int = 0


@dataclass
class RoadGraph:
    nodes: list[Node]
    edges: list[Edge]
    resolution: float
    origin: tuple[float, float]
    #: (H, W) int32。空きセルが属するエッジの番号、どれにも属さなければ -1
    label: np.ndarray
    #: (H, W) float32。壁（＋未知）までの距離 [m]
    clearance: np.ndarray

    def to_cell(self, x, y):
        col = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(np.int64)
        row = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(np.int64)
        return col, row


# ── 細線化 ──

def _neighbors(p: np.ndarray) -> list[np.ndarray]:
    """P2..P9（上から時計回り）の8近傍。`p` は外周1画素を0で埋めた配列。"""
    return [p[:-2, 1:-1], p[:-2, 2:], p[1:-1, 2:], p[2:, 2:],
            p[2:, 1:-1], p[2:, :-2], p[1:-1, :-2], p[:-2, :-2]]


def _crossings(nb: list[np.ndarray]) -> np.ndarray:
    """周囲を1周したときの 0→1 の変化の回数（交差数）。"""
    a = np.zeros_like(nb[0], dtype=np.int32)
    for k in range(8):
        a += ((nb[k] == 0) & (nb[(k + 1) % 8] == 1))
    return a


def thin(mask: np.ndarray, max_iter: int = 500) -> np.ndarray:
    """Zhang-Suen の細線化（Zhang & Suen 1984）。

    局所演算なので前景の外接矩形だけを回す（地図の格子は道の外側がほとんどで、
    全体で回すと数倍遅い）。結果は全体で回した場合と同じ。
    """
    rows = np.flatnonzero(mask.any(axis=1))
    if not len(rows):
        return np.zeros(mask.shape, dtype=bool)
    cols = np.flatnonzero(mask.any(axis=0))
    r0, r1, c0, c1 = rows[0], rows[-1] + 1, cols[0], cols[-1] + 1
    out = np.zeros(mask.shape, dtype=bool)
    out[r0:r1, c0:c1] = _thin(mask[r0:r1, c0:c1], max_iter)
    return out


def _thin(mask: np.ndarray, max_iter: int) -> np.ndarray:
    img = np.pad(mask.astype(np.uint8), 1)
    for _ in range(max_iter):
        changed = False
        for step in (0, 1):
            nb = _neighbors(img)
            p2, p3, p4, p5, p6, p7, p8, p9 = nb
            b = sum(n.astype(np.int32) for n in nb)
            a = _crossings(nb)
            if step == 0:
                c1 = (p2 * p4 * p6) == 0
                c2 = (p4 * p6 * p8) == 0
            else:
                c1 = (p2 * p4 * p8) == 0
                c2 = (p2 * p6 * p8) == 0
            core = img[1:-1, 1:-1]
            rm = (core == 1) & (b >= 2) & (b <= 6) & (a == 1) & c1 & c2
            if rm.any():
                core[rm] = 0
                changed = True
        if not changed:
            break
    return img[1:-1, 1:-1].astype(bool)


# ── 細線 → グラフ ──

#: 8近傍のうち互いに隣り合う組（リング上で連続する組＋角を挟んだ4近傍同士）
_RING_ADJ = [(k, (k + 1) % 8) for k in range(8)] + [(k, (k + 2) % 8) for k in (0, 2, 4, 6)]


def _n_components(ring: list[int]) -> int:
    """8近傍（P2..P9）の立っている画素が、8連結で何個の塊に分かれるか。"""
    parent = list(range(8))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b in _RING_ADJ:
        if ring[a] and ring[b]:
            parent[find(a)] = find(b)
    return len({find(i) for i in range(8) if ring[i]})


def despur_staircase(skel: np.ndarray) -> np.ndarray:
    """斜めの階段で2画素幅になった箇所の余分な画素を消す。

    近傍2以上で、消しても周りのつながりが変わらない（近傍が8連結で1塊）画素が
    それ。直線の途中（近傍が2塊）や本物の分岐（3塊）は消えない。1画素ずつ順に
    消す（並列に消すと隣り合う2画素を同時に消して線が切れる）。
    """
    sk = skel.copy()
    h, w = sk.shape
    p = np.pad(sk.astype(np.uint8), 1)
    count = sum(n.astype(np.int32) for n in _neighbors(p))
    rr, cc = np.nonzero(sk & (count >= 3))
    for r, c in zip(rr.tolist(), cc.tolist()):
        ring = []
        for dr, dc in _OFFS:
            r2, c2 = r + dr, c + dc
            ring.append(int(0 <= r2 < h and 0 <= c2 < w and sk[r2, c2]))
        if sum(ring) >= 2 and _n_components(ring) == 1:
            sk[r, c] = False
    return sk



_OFFS = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
#: 4近傍を先に辿る（階段の角で斜めに近道して画素を取りこぼさないため）
_WALK_ORDER = [(-1, 0), (0, 1), (1, 0), (0, -1), (-1, 1), (1, 1), (1, -1), (-1, -1)]


def _trace(skel: np.ndarray) -> tuple[list[list[tuple[int, int]]], list[list[tuple[int, int]]]]:
    """細線を節点の画素群とエッジの画素列に分ける。

    戻り値の `nodes[k]` は節点 k の画素、`chains[k]` はエッジ k の画素列
    （順序付き）。エッジの両端がどの節点に触れるかは `_attach` で決める。
    """
    import cv2
    p = np.pad(skel.astype(np.uint8), 1)
    nb = _neighbors(p)
    count = sum(n.astype(np.int32) for n in nb)
    # 近傍3以上を分岐の候補にする。階段の偽の分岐も拾うが、それは両側に鎖が
    # 1本ずつしか付かない（次数2）ので `_Builder._dissolve()` が消す
    node_mask = skel & ((count >= 3) | (count <= 1))

    n_lab, lab = cv2.connectedComponents(node_mask.astype(np.uint8), connectivity=8)
    nodes: list[list[tuple[int, int]]] = [[] for _ in range(n_lab - 1)]
    rr, cc = np.nonzero(node_mask)
    for r, c in zip(rr.tolist(), cc.tolist()):
        nodes[lab[r, c] - 1].append((r, c))

    rest = skel & ~node_mask
    h, w = skel.shape
    n_ch, ch_lab = cv2.connectedComponents(rest.astype(np.uint8), connectivity=8)
    members: list[list[tuple[int, int]]] = [[] for _ in range(n_ch - 1)]
    rr, cc = np.nonzero(rest)
    for r, c in zip(rr.tolist(), cc.tolist()):
        members[ch_lab[r, c] - 1].append((r, c))

    chains: list[list[tuple[int, int]]] = []
    for k, pts in enumerate(members):
        mine = ch_lab == k + 1

        def nbr_list(r: int, c: int) -> list[tuple[int, int]]:
            out = []
            for dr, dc in _WALK_ORDER:
                r2, c2 = r + dr, c + dc
                if 0 <= r2 < h and 0 <= c2 < w and mine[r2, c2]:
                    out.append((r2, c2))
            return out

        # 鎖の端（鎖の中で近傍1以下）から辿る。端の無い鎖は節点の無い周回
        start = next((q for q in pts if len(nbr_list(*q)) <= 1), pts[0])
        seen = {start}
        chain = [start]
        cur = start
        while True:
            nxt = next((q for q in nbr_list(*cur) if q not in seen), None)
            if nxt is None:
                break
            seen.add(nxt)
            chain.append(nxt)
            cur = nxt
        chains.append(chain)
    return nodes, chains


def _attach(chain: list[tuple[int, int]], node_of: np.ndarray) -> tuple[int, int]:
    """鎖の両端に8近傍で触れている節点。触れていなければ -1。"""
    h, w = node_of.shape

    def touch(r: int, c: int, avoid: int = -2) -> int:
        best = -1
        for dr, dc in _OFFS:
            r2, c2 = r + dr, c + dc
            if 0 <= r2 < h and 0 <= c2 < w and node_of[r2, c2] >= 0:
                if node_of[r2, c2] != avoid:
                    return int(node_of[r2, c2])
                best = int(node_of[r2, c2])
        return best

    a = touch(*chain[0])
    # 1画素の鎖でも、もう一方の端は**別の節点**を優先する（同じ節点を返すと長さ0の
    # 自己ループとして捨てられ、1画素の橋でつながる2つの節点が切れる）。他に節点が
    # 無ければ `a` が返る
    b = touch(*chain[-1], avoid=a)
    return a, b


# ── 公開関数 ──

def trinary_from_bool(occupied: np.ndarray) -> np.ndarray:
    """真値の占有格子（シムのコース）を3値（1=空き 2=占有）にする。テスト・ベンチ用。"""
    return np.where(occupied, 2, 1).astype(np.uint8)


def build_graph(trinary: np.ndarray, *, resolution: float, origin: tuple[float, float],
                keep: float, seed: tuple[float, float] | None = None,
                traj: np.ndarray | None = None, spur_len: float = 0.8,
                merge_dist: float = 0.35) -> RoadGraph:
    """3値地図から道路グラフを作る。

    :param keep: 車体中心が壁からこれ以上離れていないと通れない [m]
        （車体半幅＋余裕）。これより狭い道はグラフから消える
    :param seed: この点を含む領域だけを使う（地図作成の出発点）。None なら最大の連結成分
    :param traj: 地図作成の軌跡 `(N, 2+)`。エッジを通ってよい向きを決めるのに使う
    :param spur_len: これより短い行き止まりは細線化のひげとして消す [m]
    :param merge_dist: これより短いエッジで結ばれた節点は1つにまとめる [m]
    """
    import cv2
    res = float(resolution)
    free = trinary == 1
    unknown = trinary == 0
    # 小さな未知の穴（スキャンの死角）だけを塞ぐ。壁は塞がない
    k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_CLOSE, k3).astype(bool)
    free = free | (closed & unknown)
    free = cv2.morphologyEx(free.astype(np.uint8), cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8)).astype(bool)

    clear = cv2.distanceTransform(free.astype(np.uint8), cv2.DIST_L2, 5).astype(np.float32) * res
    cspace = clear >= keep

    n_lab, lab = cv2.connectedComponents(cspace.astype(np.uint8), connectivity=8)
    if n_lab <= 1:
        raise ValueError("通れる領域が無い（道幅が車体に対して狭すぎるか、地図が空）")
    pick = _pick_component(lab, n_lab, seed, resolution=res, origin=origin)
    cspace = lab == pick
    # 空き領域の側も、選んだ C-space を含む連結成分だけにする（壁の外の空きを除く）
    _, lab2 = cv2.connectedComponents(free.astype(np.uint8), connectivity=4)
    rr, cc = np.nonzero(cspace)
    free = lab2 == lab2[rr[0], cc[0]]

    skel = despur_staircase(thin(cspace))
    nodes_px, chains = _trace(skel)

    h, w = skel.shape
    node_of = np.full((h, w), -1, dtype=np.int32)
    nodes: list[Node] = []
    for k, px in enumerate(nodes_px):
        arr = np.array(px)
        for r, c in px:
            node_of[r, c] = k
        nodes.append(Node(xy=_cell_center(arr[:, 1].mean(), arr[:, 0].mean(), res, origin)))

    raw_edges: list[tuple[int, int, np.ndarray]] = []
    for ch in chains:
        a, b = _attach(ch, node_of)
        arr = np.array(ch)
        xy = np.column_stack(_cell_center(arr[:, 1], arr[:, 0], res, origin))
        if a < 0 and b < 0:
            if len(ch) < 8:
                continue
            # 節点の無い周回（分岐の無いオーバル）。鎖の最初の画素に節点を置く
            a = b = len(nodes)
            nodes.append(Node(xy=(float(xy[0, 0]), float(xy[0, 1]))))
            xy = np.vstack([xy, xy[:1]])
        elif a < 0 or b < 0:
            # 片側だけ節点に触れていない＝鎖の途中で途切れた小片。捨てる
            continue
        raw_edges.append((a, b, xy))

    g = _Builder(nodes, raw_edges)
    g.simplify(spur_len=spur_len, merge_dist=merge_dist)
    graph_nodes, graph_edges = g.finish(clear, res, origin)

    label = _label_cells(free, graph_edges, h, w, res, origin)
    graph = RoadGraph(nodes=graph_nodes, edges=graph_edges, resolution=res,
                      origin=(float(origin[0]), float(origin[1])), label=label,
                      clearance=clear)
    if traj is not None and len(traj) >= 2:
        _directions(graph, np.asarray(traj, dtype=np.float64)[:, :2])
    return graph


def _cell_center(col, row, res: float, origin: tuple[float, float]):
    x = origin[0] + (np.asarray(col, dtype=np.float64) + 0.5) * res
    y = origin[1] + (np.asarray(row, dtype=np.float64) + 0.5) * res
    if np.ndim(x) == 0:
        return float(x), float(y)
    return x, y


def _pick_component(lab: np.ndarray, n_lab: int, seed, *, resolution: float,
                    origin: tuple[float, float]) -> int:
    if seed is not None:
        c = int(math.floor((seed[0] - origin[0]) / resolution))
        r = int(math.floor((seed[1] - origin[1]) / resolution))
        h, w = lab.shape
        if 0 <= r < h and 0 <= c < w and lab[r, c] > 0:
            return int(lab[r, c])
        # 出発点が C-space の外（壁寄りに置いた等）。いちばん近い成分を取る
        rr, cc = np.nonzero(lab > 0)
        if len(rr):
            k = int(np.argmin((rr - r) ** 2 + (cc - c) ** 2))
            return int(lab[rr[k], cc[k]])
    sizes = np.bincount(lab.ravel(), minlength=n_lab)
    sizes[0] = 0
    return int(np.argmax(sizes))


class _Builder:
    """ひげの除去・節点の統合・次数2の節点の消去を、変化が無くなるまで繰り返す。"""

    def __init__(self, nodes: list[Node], edges: list[tuple[int, int, np.ndarray]]) -> None:
        self.pos = {i: np.array(n.xy) for i, n in enumerate(nodes)}
        self.edges: dict[int, list] = {}
        for k, (a, b, xy) in enumerate(edges):
            self.edges[k] = [a, b, xy]
        self._next = len(edges)

    def _adj(self) -> dict[int, list[int]]:
        adj: dict[int, list[int]] = {i: [] for i in self.pos}
        for k, (a, b, _) in self.edges.items():
            adj[a].append(k)
            adj[b].append(k)
        return adj

    @staticmethod
    def _len(xy: np.ndarray) -> float:
        return float(np.hypot(*np.diff(xy, axis=0).T).sum()) if len(xy) > 1 else 0.0

    def simplify(self, *, spur_len: float, merge_dist: float) -> None:
        for _ in range(50):
            changed = self._prune(spur_len) | self._merge(merge_dist) | self._dissolve()
            if not changed:
                break
        # 孤立した節点を消す
        adj = self._adj()
        for i in [i for i, e in adj.items() if not e]:
            del self.pos[i]

    def _prune(self, spur_len: float) -> bool:
        adj = self._adj()
        changed = False
        for k, (a, b, xy) in list(self.edges.items()):
            if a == b:
                # 細線化が角に残す小さな輪。本物の周回はこれより長い
                if self._len(xy) < spur_len:
                    del self.edges[k]
                    changed = True
                    adj = self._adj()
                continue
            da, db = len(adj[a]), len(adj[b])
            # 行き止まりのひげ（片端が次数1で、もう片端が分岐）
            if (da == 1) != (db == 1) and self._len(xy) < spur_len:
                del self.edges[k]
                changed = True
                adj = self._adj()
            elif da == 1 and db == 1 and self._len(xy) < spur_len:
                del self.edges[k]      # 孤立した短い線
                changed = True
                adj = self._adj()
        return changed

    def _merge(self, merge_dist: float) -> bool:
        """短いエッジで結ばれた2つの分岐を1つの節点にまとめる（十字路が2つの
        T字に割れるのを直す）。"""
        adj = self._adj()
        for k, (a, b, xy) in list(self.edges.items()):
            if a == b or self._len(xy) >= merge_dist:
                continue
            if len(adj[a]) < 3 or len(adj[b]) < 3:
                continue
            # 折れ線の端は `finish()` が節点の位置へつなぎ直すので、ここは付け替えだけ
            self.pos[a] = (self.pos[a] + self.pos[b]) / 2.0
            del self.edges[k]
            for e in self.edges.values():
                if e[0] == b:
                    e[0] = a
                if e[1] == b:
                    e[1] = a
            del self.pos[b]
            return True
        return False

    def _dissolve(self) -> bool:
        """次数2の節点を消して両側のエッジをつなぐ。"""
        adj = self._adj()
        for n, es in adj.items():
            if len(es) != 2 or es[0] == es[1]:
                continue
            k1, k2 = es
            a1, b1, xy1 = self.edges[k1]
            a2, b2, xy2 = self.edges[k2]
            # k1 を「? → n」、k2 を「n → ?」の向きにそろえる
            if b1 != n:
                a1, b1, xy1 = b1, a1, xy1[::-1]
            if a2 != n:
                a2, b2, xy2 = b2, a2, xy2[::-1]
            xy = np.vstack([xy1, self.pos[n][None, :], xy2])
            del self.edges[k1]
            del self.edges[k2]
            self.edges[self._next] = [a1, b2, xy]
            self._next += 1
            del self.pos[n]
            return True
        return False

    def finish(self, clear: np.ndarray, res: float,
               origin: tuple[float, float]) -> tuple[list[Node], list[Edge]]:
        ids = sorted(self.pos)
        remap = {old: new for new, old in enumerate(ids)}
        nodes = [Node(xy=(float(self.pos[i][0]), float(self.pos[i][1]))) for i in ids]
        edges: list[Edge] = []
        h, w = clear.shape
        for k in sorted(self.edges):
            a, b, xy = self.edges[k]
            # 端点を節点の位置にそろえ、重複点を落とす
            xy = np.vstack([self.pos[a][None, :], xy, self.pos[b][None, :]])
            keep = np.ones(len(xy), dtype=bool)
            keep[1:] = np.hypot(*np.diff(xy, axis=0).T) > 1e-6
            xy = xy[keep]
            if len(xy) < 2:
                continue
            e = Edge(u=remap[a], v=remap[b], xy=xy, length=self._len(xy))
            nodes[e.u].edges.append(len(edges))
            nodes[e.v].edges.append(len(edges))
            edges.append(e)
        return nodes, edges


def _label_cells(free: np.ndarray, edges: list[Edge], h: int, w: int, res: float,
                 origin: tuple[float, float]) -> np.ndarray:
    """空きセルを最寄りのエッジに割り当てる（空きの中だけを4近傍で膨張、docstring参照）。"""
    import cv2
    lab = np.zeros((h, w), dtype=np.float32)        # 0=未割り当て、k+1=エッジk
    for k, e in enumerate(edges):
        pts = np.column_stack([(e.xy[:, 0] - origin[0]) / res,
                               (e.xy[:, 1] - origin[1]) / res]).astype(np.int32)
        cv2.polylines(lab, [pts.reshape(-1, 1, 2)], False, float(k + 1), 1)
    lab[~free] = 0
    cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    for _ in range(max(h, w)):
        todo = free & (lab == 0)
        if not todo.any():
            break
        grown = cv2.dilate(lab, cross)
        fill = todo & (grown > 0)
        if not fill.any():
            break
        lab[fill] = grown[fill]
    return (lab.astype(np.int32) - 1)


def _directions(graph: RoadGraph, traj: np.ndarray) -> None:
    """地図作成の軌跡がエッジを進んだ向きを数え、一方向だけならその向きに固定する。"""
    h, w = graph.label.shape
    col, row = graph.to_cell(traj[:, 0], traj[:, 1])
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    votes = np.zeros((len(graph.edges), 2), dtype=np.int64)     # [+向き, −向き]
    prev_e, prev_s = -1, 0.0
    cum = [np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(e.xy, axis=0).T))])
           for e in graph.edges]
    for i in range(len(traj)):
        if not ok[i]:
            prev_e = -1
            continue
        e = int(graph.label[row[i], col[i]])
        if e < 0:
            prev_e = -1
            continue
        xy = graph.edges[e].xy
        j = int(np.argmin((xy[:, 0] - traj[i, 0]) ** 2 + (xy[:, 1] - traj[i, 1]) ** 2))
        s = float(cum[e][j])
        if e == prev_e:
            ds = s - prev_s
            # 自己ループの継ぎ目（s が1周ぶん跳ぶ）は数えない
            if 1e-3 < abs(ds) < 0.5 * graph.edges[e].length:
                votes[e, 0 if ds > 0 else 1] += 1
        prev_e, prev_s = e, s
    for k, e in enumerate(graph.edges):
        fwd, back = votes[k]
        tot = fwd + back
        if tot >= 5:
            if fwd >= 0.9 * tot:
                e.dir_allowed = +1
            elif back >= 0.9 * tot:
                e.dir_allowed = -1


class CorridorGrid:
    """経路が通るエッジの回廊の外を壁とみなす格子（`raycast` だけを持つ）。

    `nav/centerline.measure()` / `nav/raceline.optimize()` は格子を
    ダックタイピングで受け、使うのは `raycast()` だけなので、これを渡せば
    **分岐の口で道幅を測りすぎない**（選ばなかった枝の奥までレイが抜けない）。

    ## 分岐点の周り `reach` [m] は、選ばなかった枝の口も回廊に含める

    含めないと回廊の境界（2本のエッジの二等分線）が分岐点で中心線に接し、
    **曲がる余地を最適化から奪う**。近道のように分岐点どうしが近いと、
    「西向き→北へ90°→東へ90°」の S 字が 0.8m の中に押し込まれ、半径19cm
    （車の最小旋回半径 約40cmの半分）のレーシングラインが出て曲がり切れなかった
    （toyota2 の sim.bench、2026-09-29）。口は本当に空いている場所なので使ってよい。
    **枝の奥まで測りすぎないよう、分岐点から `reach` までの空きに限る**
    （空きの中だけを膨張させる＝壁の向こうへは広がらない）。
    """

    def __init__(self, base, graph: RoadGraph, edge_ids, node_ids,
                 reach: float = 0.8) -> None:
        import cv2
        self._base = base
        allowed = np.isin(graph.label, np.asarray(sorted(set(edge_ids)), dtype=np.int32))
        free = graph.label >= 0
        seeds = np.zeros(graph.label.shape, dtype=np.uint8)
        h, w = graph.label.shape
        for n in set(node_ids):
            c, r = graph.to_cell(*graph.nodes[n].xy)
            if 0 <= r < h and 0 <= c < w:
                seeds[int(r), int(c)] = 1
        # 分岐点から空きの中だけを reach まで広げる（4近傍＝壁の斜めの隙間を抜けない）
        near = seeds.astype(bool) & free
        cross = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        for _ in range(int(math.ceil(reach / graph.resolution))):
            grown = cv2.dilate(near.astype(np.uint8), cross).astype(bool) & free
            if (grown == near).all():
                break
            near = grown
        blocked = free & ~(allowed | near)
        wall = base.wall_mask()
        if wall.shape != blocked.shape:
            raise ValueError("地図とグラフの格子の大きさが違う")
        self.mask = wall | blocked
        self.resolution = base.resolution
        self.origin = base.origin

    def wall_mask(self) -> np.ndarray:
        return self.mask

    def base_wall_mask(self) -> np.ndarray:
        """回廊の外を含まない、本当の壁（車体の検査用、`raceline._BodyCheck`）。"""
        return self._base.wall_mask()

    def raycast(self, ox, oy, angles, max_range, mask=None, fill: int = 1):
        return self._base.raycast(ox, oy, angles, max_range,
                                  mask=self.mask if mask is None else mask, fill=fill)
