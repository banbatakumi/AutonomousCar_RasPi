"""道路グラフ上の経路計画 — 経由点を順に通る周回と、今の位置からゴールまで。

`nav/roadgraph.py` のグラフの上で経路を選び、`nav/centerline.py` /
`nav/raceline.py` にそのまま渡せる折れ線にする。

## 経由点で経路を選ぶ（nav2 route server・Duckietown と同じ考え方）

経由点 `W0, W1, …, Wn-1` を順に最短路でつなぎ、`Wn-1 → W0` で閉じる。
分岐のどちらへ行くかは「行きたい枝の上に経由点を1つ置く」ことで決まる。
エッジのコストをいじって誘導するより、**何が選ばれるかが人間に読める**。

## ★ 状態は「どのエッジをどちら向きに走っているか」

節点だけを状態にした Dijkstra は、Y 字の分岐で「左の枝から入って右の枝へ
折り返す」経路を平気で返す（節点を通過しているだけに見える）。アッカーマン車は
その場で折り返せないので、状態を有向エッジにして**節点での曲がり角が
`turn_max` を超える遷移を禁じる**（turn-restricted routing）。

**逆走禁止**: `Edge.dir_allowed`（地図作成で走った向き）と逆には通さない。

## 経由点の向き

経由点はいちばん近いエッジに吸着させる。そのエッジをどちら向きに通るかは
全経由点の組み合わせを DP で選ぶ（最短の周回になる向き）。経由点に `yaw` を
与えればその向きに固定する（地図作成の軌跡から自動で置く経由点はこれ）。

## 回廊の上で中心線とレーシングラインを作る

経路の折れ線（細線化の結果なので道の真ん中を通っている）を `centerline.build()`
に「走った跡」として渡し、**`CorridorGrid`（選ばなかった枝を壁とみなす格子）**
の上で幅を測る。分岐の口でレイが枝の奥へ抜けて幅を測りすぎるのを防ぐため。
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from . import centerline as cl_mod
from . import raceline as rl_mod
from .roadgraph import CorridorGrid, RoadGraph

__all__ = ["Waypoint", "RoutePath", "RouteError", "plan_loop", "plan_to", "build_raceline",
           "waypoints_from_traj", "snap", "tightest_radius",
           "enumerate_loops", "avoid_edges", "lap_time"]


class RouteError(ValueError):
    """経路が作れない（経由点が道から遠い、つながっていない、など）。**理由は日本語。**"""


@dataclass
class Waypoint:
    x: float
    y: float
    #: 通る向き [rad]。None なら向きは問わない（周回が最短になる向きを選ぶ）
    yaw: float | None = None


@dataclass
class _Snap:
    edge: int
    s: float                               #: エッジの始点 u からの弧長 [m]
    xy: tuple[float, float]
    tangent: tuple[float, float]           #: u→v 向きの単位接ベクトル


@dataclass
class RoutePath:
    xy: np.ndarray                         #: (N, 2) 経路の折れ線（閉路なら始点に戻る手前まで）
    closed: bool
    #: 通る有向エッジ `(edge, dir)` の列（重複あり、順番どおり）
    steps: list[tuple[int, int]] = field(default_factory=list)
    length: float = 0.0

    @property
    def edge_ids(self) -> list[int]:
        return sorted({e for e, _ in self.steps})

    def node_ids(self, graph: RoadGraph) -> list[int]:
        out = set()
        for e, _ in self.steps:
            out.add(graph.edges[e].u)
            out.add(graph.edges[e].v)
        return sorted(out)


# ── エッジの幾何 ──

class _Geom:
    """エッジごとの弧長と、節点を出入りするときの向き。

    ★ 向きは節点から `look`（1m）先までの弦で測る。0.5m だと、平行な2車線が
    分かれる Y 字（toyota2 の上側）の根元では両方の車線がまだ同じ向きに見え、
    **壁の先端を回り込む U ターン**が「60°の曲がり」として通ってしまった。
    """

    def __init__(self, graph: RoadGraph, look: float = 1.0) -> None:
        self.g = graph
        self.cum = []
        self.out_h = []                # [edge][0]= u を出る向き（+）, [1]= v を出る向き（−）
        self.in_h = []                 # [edge][0]= v に入る向き（+）, [1]= u に入る向き（−）
        for e in graph.edges:
            c = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(e.xy, axis=0).T))])
            self.cum.append(c)
            L = float(c[-1])
            la = min(look, 0.45 * L)
            p0, p1 = e.xy[0], self.at(len(self.cum) - 1, la)
            q0, q1 = self.at(len(self.cum) - 1, L - la), e.xy[-1]
            self.out_h.append((_heading(p0, p1), _heading(q1, q0)))
            self.in_h.append((_heading(q0, q1), _heading(p1, p0)))

    def at(self, e: int, s: float) -> np.ndarray:
        c = self.cum[e]
        xy = self.g.edges[e].xy
        s = float(np.clip(s, 0.0, c[-1]))
        return np.array([np.interp(s, c, xy[:, 0]), np.interp(s, c, xy[:, 1])])

    def length(self, e: int) -> float:
        return float(self.cum[e][-1])

    def sub(self, e: int, d: int, s_a: float, s_b: float) -> np.ndarray:
        """エッジ `e` を向き `d` で `s_a` から `s_b` まで（s は u からの弧長）。"""
        c = self.cum[e]
        xy = self.g.edges[e].xy
        lo, hi = (s_a, s_b) if d > 0 else (s_b, s_a)
        inner = (c > lo) & (c < hi)
        pts = np.vstack([self.at(e, lo)[None, :], xy[inner], self.at(e, hi)[None, :]])
        return pts if d > 0 else pts[::-1]


def _heading(a, b) -> float:
    return math.atan2(float(b[1] - a[1]), float(b[0] - a[0]))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ── 吸着 ──

def snap(graph: RoadGraph, x: float, y: float, radius: float = 0.6) -> _Snap:
    """いちばん近いエッジ上の点。`radius` より遠ければ RouteError。"""
    best = (math.inf, -1, -1)
    for k, e in enumerate(graph.edges):
        d2 = (e.xy[:, 0] - x) ** 2 + (e.xy[:, 1] - y) ** 2
        j = int(np.argmin(d2))
        if d2[j] < best[0]:
            best = (float(d2[j]), k, j)
    if best[1] < 0 or math.sqrt(best[0]) > radius:
        raise RouteError(f"({x:.2f}, {y:.2f}) の近く {radius:.1f}m 以内に道が無い")
    k, j = best[1], best[2]
    e = graph.edges[k]
    c = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(e.xy, axis=0).T))])
    a, b = max(0, j - 2), min(len(e.xy) - 1, j + 2)
    t = e.xy[b] - e.xy[a]
    t = t / max(float(np.hypot(*t)), 1e-9)
    return _Snap(edge=k, s=float(c[j]), xy=(float(e.xy[j, 0]), float(e.xy[j, 1])),
                 tangent=(float(t[0]), float(t[1])))


def _dirs(graph: RoadGraph, sp: _Snap, yaw: float | None) -> list[int]:
    allowed = graph.edges[sp.edge].dir_allowed
    ds = [d for d in (+1, -1) if allowed == 0 or allowed == d]
    if yaw is not None:
        ds = [d for d in ds
              if d * (sp.tangent[0] * math.cos(yaw) + sp.tangent[1] * math.sin(yaw)) > 0.0]
    return ds


# ── 探索 ──

def _leg(graph: RoadGraph, geo: _Geom, a: _Snap, da: int, b: _Snap, db: int,
         turn_max: float, forbidden: frozenset[int] = frozenset()
         ) -> tuple[float, list[tuple[int, int]]] | None:
    """`a` を向き `da` で出て `b` を向き `db` で通るまでの最短路（曲がり角の制約付き）。"""
    # 同じエッジを同じ向きで、b が前方にある
    if a.edge == b.edge and da == db:
        ds = (b.s - a.s) * da
        if ds > 1e-6:
            return ds, [(a.edge, da)]

    def exit_node(e: int, d: int) -> int:
        return graph.edges[e].v if d > 0 else graph.edges[e].u

    def entry_node(e: int, d: int) -> int:
        return graph.edges[e].u if d > 0 else graph.edges[e].v

    start_cost = (geo.length(a.edge) - a.s) if da > 0 else a.s
    best_cost = math.inf
    best_state = None
    dist: dict[tuple[int, int], float] = {(a.edge, da): start_cost}
    prev: dict[tuple[int, int], tuple[int, int] | None] = {(a.edge, da): None}
    pq = [(start_cost, a.edge, da)]
    while pq:
        g, e, d = heapq.heappop(pq)
        if g > dist.get((e, d), math.inf) + 1e-9:
            continue
        if g >= best_cost:
            break
        n = exit_node(e, d)
        h_in = geo.in_h[e][0 if d > 0 else 1]
        for e2 in set(graph.nodes[n].edges):
            if e2 in forbidden:
                continue
            ed = graph.edges[e2]
            for d2 in (+1, -1):
                if entry_node(e2, d2) != n:
                    continue
                if ed.dir_allowed and ed.dir_allowed != d2:
                    continue
                h_out = geo.out_h[e2][0 if d2 > 0 else 1]
                if abs(_wrap(h_out - h_in)) > turn_max:
                    continue
                if e2 == b.edge and d2 == db:
                    to_b = b.s if d2 > 0 else geo.length(e2) - b.s
                    if g + to_b < best_cost:
                        best_cost = g + to_b
                        best_state = (e, d)
                g2 = g + geo.length(e2)
                if g2 < dist.get((e2, d2), math.inf) - 1e-9:
                    dist[(e2, d2)] = g2
                    prev[(e2, d2)] = (e, d)
                    heapq.heappush(pq, (g2, e2, d2))
    if best_state is None:
        return None
    chain = []
    cur: tuple[int, int] | None = best_state
    while cur is not None:
        chain.append(cur)
        cur = prev.get(cur)
    chain.reverse()
    return best_cost, chain + [(b.edge, db)]


def _legs_xy(geo: _Geom, a: _Snap, b: _Snap, steps: list[tuple[int, int]]) -> np.ndarray:
    if len(steps) == 1:
        e, d = steps[0]
        return geo.sub(e, d, a.s, b.s)
    parts = []
    e, d = steps[0]
    parts.append(geo.sub(e, d, a.s, geo.length(e) if d > 0 else 0.0))
    for e, d in steps[1:-1]:
        parts.append(geo.sub(e, d, 0.0, geo.length(e)) if d > 0
                     else geo.sub(e, d, geo.length(e), 0.0))
    e, d = steps[-1]
    parts.append(geo.sub(e, d, 0.0 if d > 0 else geo.length(e), b.s))
    return np.vstack(parts)


def _dedup(xy: np.ndarray) -> np.ndarray:
    if len(xy) < 2:
        return xy
    keep = np.ones(len(xy), dtype=bool)
    keep[1:] = np.hypot(*np.diff(xy, axis=0).T) > 1e-4
    return xy[keep]


def plan_loop(graph: RoadGraph, waypoints: list[Waypoint], *, turn_max_deg: float = 100.0,
              snap_radius: float = 0.6,
              avoid: list[tuple[float, float]] | None = None) -> RoutePath:
    """経由点を順に通って最初の経由点へ戻る周回。

    :param avoid: 避ける点。いちばん近いエッジ（道）を使わない（`avoid_edges`）
    """
    if not waypoints:
        raise RouteError("経由点が無い")
    if not graph.edges:
        raise RouteError("道路グラフにエッジが無い")
    turn_max = math.radians(turn_max_deg)
    geo = _Geom(graph)
    forbidden = avoid_edges(graph, avoid)
    snaps = [snap(graph, w.x, w.y, snap_radius) for w in waypoints]
    for i, sp in enumerate(snaps):
        if sp.edge in forbidden:
            raise RouteError(f"経由点{i + 1}が避ける道の上にある")
    dirs = [_dirs(graph, sp, w.yaw) for sp, w in zip(snaps, waypoints)]
    for i, ds in enumerate(dirs):
        if not ds:
            raise RouteError(f"経由点{i + 1}を指定の向きで通れない（逆走になる）")
    n = len(snaps)

    best_total = math.inf
    best_plan: list[list[tuple[int, int]]] | None = None
    for d0 in dirs[0]:
        # DP: cost[d] = W0(d0) から Wi(d) までの最短
        cost = {d0: (0.0, [])}
        for i in range(1, n + 1):
            k = i % n
            targets = [d0] if k == 0 else dirs[k]
            new: dict[int, tuple[float, list]] = {}
            for dk in targets:
                for dp, (cp, legs) in cost.items():
                    leg = _leg(graph, geo, snaps[i - 1], dp, snaps[k], dk, turn_max,
                               forbidden)
                    if leg is None:
                        continue
                    c = cp + leg[0]
                    if c < new.get(dk, (math.inf,))[0]:
                        new[dk] = (c, legs + [leg[1]])
            cost = new
            if not cost:
                break
        if d0 in cost and cost[d0][0] < best_total:
            best_total, best_plan = cost[d0]
    if best_plan is None:
        raise RouteError("経由点を順に通る周回が見つからない"
                         "（道がつながっていないか、曲がり切れない分岐がある）")

    parts = []
    steps: list[tuple[int, int]] = []
    for i, legsteps in enumerate(best_plan):
        parts.append(_legs_xy(geo, snaps[i], snaps[(i + 1) % n], legsteps))
        steps.extend(legsteps)
    xy = _dedup(np.vstack(parts))
    if len(xy) >= 2 and np.hypot(*(xy[-1] - xy[0])) < 1e-3:
        xy = xy[:-1]
    return RoutePath(xy=xy, closed=True, steps=steps, length=best_total)


def plan_to(graph: RoadGraph, pose: tuple[float, float, float], goal: Waypoint, *,
            turn_max_deg: float = 100.0, snap_radius: float = 0.6,
            min_len: float = 0.0, end_at_goal: bool = True,
            extend: float = 0.0,
            avoid: list[tuple[float, float]] | None = None) -> RoutePath:
    """今の位置（今の向きで進む）からゴールまでの開いた経路。

    :param min_len: ゴールがこれより近い（すぐ後ろにある等）なら1周回ってでも
        そこへ行く経路を返す（止まるのに必要な距離が取れないため）
    :param end_at_goal: False なら道の上の最寄り点で終わる（駐車枠のように
        道から外れたゴールは、そこから先を `park_to_point` に任せる）
    :param extend: `end_at_goal=False` のとき、最寄り点からさらに進行方向へこれだけ
        進んだ所で終わる [m]（バック駐車は枠を通り過ぎてから後退で入れるため）。
        エッジの端で頭打ち
    """
    turn_max = math.radians(turn_max_deg)
    geo = _Geom(graph)
    forbidden = avoid_edges(graph, avoid)
    a = snap(graph, pose[0], pose[1], snap_radius)
    da_list = _dirs(graph, a, pose[2])
    if not da_list:
        raise RouteError("今の向きでは今いる道を進めない（逆走になる）")
    b = snap(graph, goal.x, goal.y, snap_radius)
    best = None
    for db in _dirs(graph, b, goal.yaw) or []:
        leg = _leg(graph, geo, a, da_list[0], b, db, turn_max, forbidden)
        if leg is not None and leg[0] < min_len:
            # 近すぎる: いったん先へ進んでから戻ってくる経路（1周）を探す
            leg = _leg_via_loop(graph, geo, a, da_list[0], b, db, turn_max, min_len,
                                forbidden)
        if leg is not None and (best is None or leg[0] < best[0]):
            best = leg
    if best is None:
        raise RouteError("ゴールへ行く経路が見つからない")
    tail = [np.array([[goal.x, goal.y]])] if end_at_goal else []
    length = best[0]
    if not end_at_goal and extend > 0.0:
        e, d = best[1][-1]
        s_end = float(np.clip(b.s + d * extend, 0.0, geo.length(e)))
        tail = [geo.sub(e, d, b.s, s_end)[1:]]
        length += abs(s_end - b.s)
    xy = _dedup(np.vstack([np.array([[pose[0], pose[1]]]), _legs_xy(geo, a, b, best[1])]
                          + tail))
    return RoutePath(xy=xy, closed=False, steps=best[1], length=length)


def _leg_via_loop(graph, geo, a, da, b, db, turn_max, min_len, forbidden=frozenset()):
    """`a` から先へ `min_len` 以上進んだ点を経由して `b` へ。"""
    ahead = min(geo.length(a.edge), a.s + min_len) if da > 0 else max(0.0, a.s - min_len)
    mid = _Snap(edge=a.edge, s=ahead, xy=tuple(geo.at(a.edge, ahead)), tangent=a.tangent)
    first = _leg(graph, geo, a, da, mid, da, turn_max, forbidden)
    second = _leg(graph, geo, mid, da, b, db, turn_max, forbidden)
    if first is None or second is None:
        return None
    steps = first[1][:-1] + second[1]
    return first[0] + second[0], steps


# ── 避ける道・全周回の数え上げ ──

def avoid_edges(graph: RoadGraph, avoid, radius: float = 0.6) -> frozenset[int]:
    """避ける点それぞれにいちばん近いエッジ（`radius` 以内）の集合。

    **ルールで通れない道**（大会の近道・信号で指定された時だけ通る狭い道など）は
    地図からは分からないので、人が地図の上で「ここは通らない」と指すための口。
    """
    out = set()
    for x, y in avoid or ():
        try:
            out.add(snap(graph, float(x), float(y), radius).edge)
        except RouteError:
            continue            # 道から遠い点は無視（地図を作り直して道が動いた等）
    return frozenset(out)


def enumerate_loops(graph: RoadGraph, start: Waypoint, *, turn_max_deg: float = 100.0,
                    avoid: list[tuple[float, float]] | None = None,
                    max_edges: int = 24, max_loops: int = 64,
                    snap_radius: float = 0.6) -> list[RoutePath]:
    """スタート（`start`、向き付き）から出て、スタートへ戻る周回ルートを全部挙げる。

    **同じ道（エッジ、向きを問わず）は1周で1回まで**。向きだけで区別すると、
    地図作成で走っていない道（通ってよい向きが決まらない）を往復する「同じ車線を
    逆走して戻る」周回まで数えてしまった（toyota2 の真値で 12 通り中 8 通り）。曲がり角の上限・逆走禁止・
    避ける道は `plan_loop` と同じ。コースの道路グラフはエッジ数十なので全列挙で足りる
    （toyota2 で数通り）。念のため `max_edges`（1周のエッジ数）・`max_loops` で打ち切る。
    """
    turn_max = math.radians(turn_max_deg)
    geo = _Geom(graph)
    forbidden = avoid_edges(graph, avoid)
    a = snap(graph, start.x, start.y, snap_radius)
    if a.edge in forbidden:
        raise RouteError("スタートが避ける道の上にある")
    dirs = _dirs(graph, a, start.yaw)
    if not dirs:
        raise RouteError("スタートの向きでは今いる道を進めない（逆走になる）")
    d0 = dirs[0]
    start_cost = (geo.length(a.edge) - a.s) if d0 > 0 else a.s
    end_cost = a.s if d0 > 0 else geo.length(a.edge) - a.s

    def exit_node(e: int, d: int) -> int:
        return graph.edges[e].v if d > 0 else graph.edges[e].u

    def entry_node(e: int, d: int) -> int:
        return graph.edges[e].u if d > 0 else graph.edges[e].v

    loops: list[tuple[float, list[tuple[int, int]]]] = []

    def dfs(state: tuple[int, int], steps: list[tuple[int, int]], used: set, cost: float):
        if len(loops) >= max_loops or len(steps) > max_edges:
            return
        e, d = state
        n = exit_node(e, d)
        h_in = geo.in_h[e][0 if d > 0 else 1]
        for e2 in sorted(set(graph.nodes[n].edges)):
            if e2 in forbidden:
                continue
            ed = graph.edges[e2]
            for d2 in (+1, -1):
                if entry_node(e2, d2) != n or (ed.dir_allowed and ed.dir_allowed != d2):
                    continue
                if abs(_wrap(geo.out_h[e2][0 if d2 > 0 else 1] - h_in)) > turn_max:
                    continue
                if (e2, d2) == (a.edge, d0):
                    loops.append((cost + end_cost, steps + [(e2, d2)]))
                    continue
                if e2 in used:
                    continue
                used.add(e2)
                dfs((e2, d2), steps + [(e2, d2)], used, cost + geo.length(e2))
                used.discard(e2)

    dfs((a.edge, d0), [(a.edge, d0)], {a.edge}, start_cost)
    out = []
    for length, steps in sorted(loops, key=lambda t: t[0]):
        xy = _dedup(_legs_xy(geo, a, a, steps))
        if len(xy) >= 2 and np.hypot(*(xy[-1] - xy[0])) < 1e-3:
            xy = xy[:-1]
        out.append(RoutePath(xy=xy, closed=True, steps=steps, length=length))
    return out


def lap_time(rl: rl_mod.RaceLine) -> float:
    """速度プロファイルどおりに1周した時間 [s]（見積もり、周回の比較に使う）。"""
    n = len(rl)
    ds = (np.hypot(*(np.roll(rl.xy, -1, axis=0) - rl.xy).T) if rl.closed
          else np.append(np.hypot(*np.diff(rl.xy, axis=0).T), 0.0))
    v = np.maximum(rl.v, 1e-3)
    vn = np.roll(v, -1) if rl.closed else np.append(v[1:], v[-1:])
    return float(np.sum(2.0 * ds[:n] / (v + vn)))


# ── 走った跡 → 経由点 ──

def waypoints_from_traj(traj: np.ndarray, spacing: float = 2.0) -> list[Waypoint]:
    """地図作成の1周の軌跡から、`spacing` ごとに向き付きの経由点を置く。

    経由点を1つも置いていない地図の既定経路にする。**分岐の無いコースでは
    今までどおり「走った1周」と同じ経路になる**。
    """
    xy = np.asarray(traj, dtype=np.float64)[:, :2]
    if len(xy) < 2:
        return []
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    out = []
    for t in np.arange(0.0, max(total - 0.5 * spacing, 1e-6), spacing):
        j = int(np.searchsorted(s, t))
        j = min(max(j, 0), len(xy) - 1)
        a, b = xy[max(0, j - 3)], xy[min(len(xy) - 1, j + 3)]
        out.append(Waypoint(float(xy[j, 0]), float(xy[j, 1]), _heading(a, b)))
    return out


# ── 車が曲がり切れるか ──

#: 曲がり具合を測る弦の長さ [m]。点の刻み（0.1m）の3点で測ると、1点の折れで
#: 半径が半分に見える（toyota2 で 0.1m 窓 16cm・0.3m 窓 27cm・0.5m 窓 38cm）
_TIGHT_WINDOW_M = 0.5


def tightest_radius(rl: rl_mod.RaceLine, step: float = 0.10) -> tuple[float, int]:
    """レーシングラインのいちばん急な所の半径 [m] と、その添字（弦 0.5m で測る）。"""
    k = max(1, int(round(_TIGHT_WINDOW_M / 2 / step)))
    kap = rl_mod.chord_curvature(rl.xy, k, rl.closed)
    i = int(np.argmax(kap))
    return (1.0 / kap[i] if kap[i] > 1e-9 else math.inf), i


# ── 経路 → レーシングライン ──

def build_raceline(base_grid, graph: RoadGraph, route: RoutePath, *, half_width: float,
                   margin: float, lam: float, passes: int, v_max: float, v_min: float,
                   a_lat: float, a_accel: float, a_brake: float,
                   front_overhang: float = 0.0, rear_overhang: float = 0.0,
                   step: float = 0.10, max_width: float = 3.0,
                   v_start: float | None = None, v_end: float | None = None,
                   kappa_max: float | None = None, footprint=None,
                   time_iters: int = 0) -> tuple[rl_mod.RaceLine, cl_mod.Centerline]:
    """経路 → 中心線 → レーシングライン（回廊の上で、モジュール docstring 参照）。

    :param kappa_max: 車の曲がれる限界の曲率 [1/m]（`tan(最大舵角)/ホイールベース`）。
        与えると、限界に近い区間で横Gの余裕を削る（`raceline.speed_profile`）
    """
    corridor = CorridorGrid(base_grid, graph, route.edge_ids, route.node_ids(graph))
    if route.closed:
        cl = cl_mod.build(corridor, route.xy, step=step, max_width=max_width)
    else:
        cl = cl_mod.build_open(corridor, route.xy, step=step, max_width=max_width)
    if len(cl) < 5:
        raise RouteError(f"経路が短すぎる（{len(cl)}点）")
    rl = rl_mod.optimize(corridor, cl, half_width=half_width, margin=margin, lam=lam,
                         passes=passes, v_max=v_max, v_min=v_min, a_lat=a_lat,
                         a_accel=a_accel, a_brake=a_brake, max_width=max_width,
                         front_overhang=front_overhang, rear_overhang=rear_overhang,
                         v_start=v_start, v_end=v_end, kappa_max=kappa_max,
                         footprint=footprint, time_iters=time_iters)
    return rl, cl
