"""Hybrid A* — 非ホロノミック拘束つきの格子探索。**任意形状の迂回路を見つける。**

## なぜ Reeds-Shepp の候補選択では足りないのか

`park_to_point`は当初「Reeds-Sheppの候補を長さ順に見て、衝突しない最初のものを
選ぶ」方式だった。回避できるのは**候補パターンの範囲**だけで、保証が無い。
レイキャスト点群（遮蔽込み）60通りでの回避経路発見率（2026-09-16実測）:

    縦列駐車  47/60 (78%)
    車庫入れ  26/60 (43%)   ← 半分以上で回避経路が見つからない

Hybrid A*は**解像度完備**（resolution-complete）——格子の解像度で表現できる
経路が存在すれば見つける。これが「候補が尽きたら諦める」との決定的な差。

Dolgov, Thrun, Montemerlo, Diebel, "Practical Search Techniques in Path
Planning for Autonomous Driving", 2008（DARPA Urban Challengeの実装）の構成を
採る。要点は3つで、**どれを落としても実用速度にならない**:

1. **連続状態＋離散格子**。ノードは連続姿勢`(x,y,yaw)`を持ち、重複判定だけ
   格子`(ix,iy,iyaw)`で行う。純粋な格子探索と違い、得られる経路が実際に
   走行可能（各枝が車両の運動そのもの）
2. **解析展開**（analytic expansion）。一定間隔で目標へReeds-Sheppを撃ち、
   衝突しなければ即接続する。これが無いと、目標姿勢にyawまで合わせる最後の
   詰めで探索が発散する
3. **二重ヒューリスティック**。「障害物を無視したRS長」と「非ホロノミックを
   無視した2D測地距離（Dijkstra）」の**max**を使う。前者だけでは行き止まりに
   誘い込まれ、後者だけでは向きの拘束を無視して過小評価する。**どちらも真の
   コスト以下なので、maxを取っても許容的（admissible）**

## コスト設計

経路長だけを最小化すると「壁を舐める・切り返しを繰り返す」経路が出る。
駐車で人が嫌がるのはそこなので、明示的にコストへ入れる:

- `reverse_cost` … 後退の割増（前進で済むなら前進する）
- `gear_change_cost` … **切り返し1回ぶん**。バンビが以前「切り返しが無い」と
  指摘した項目を、今度は明示的なコストとして扱えるようにしたもの
- `steer_change_cost` … 舵を切り替える回数（がくがくした経路を避ける）
- `clearance_weight` … `clearance_target`より壁に近い区間に罰則。ESDFが
  連続量のクリアランスを返すので、これが素直に書ける

## 衝突判定は`LocalMap`のESDF＋円被覆

`local_map.LocalMap.body_clearance()`が**姿勢の配列**を受け取れるので、
1ノードの全後継（既定14本）×中間サンプルを**1回のnumpy呼び出し**で判定する。
点群を直接なめる方式ではこの規模の探索は回らない。

**壁は硬い拘束、未知はコスト。** 未知セルを禁止にすると、駐車枠の奥が
遮蔽で見えていない段階で目標へ到達できなくなる（瞬時スキャンでは目標周辺の
3割が未観測、`local_map.py`のdocstring参照）。逆に未知を空きと同視すると
見えていない場所へ楽観的に突っ込む。**硬い拘束は壁だけに置き、未知は
`unknown_weight`でコストとして避ける**のが両立する唯一の形。
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np

from .local_map import LocalMap
from .reeds_shepp import (PathSegment, ReedsSheppPath, candidate_paths,
                          sample_path_array, shortest_path)
from .se2 import wrap_angle

__all__ = ["HybridConfig", "HybridResult", "plan"]


@dataclass
class HybridConfig:
    """探索の設定。**`turning_radius`は最小旋回半径に余裕を持たせた値を渡す。**

    実舵角の限界ぴったりで計画すると追従誤差を舵で詰める余地が無くなる
    （`park_to_point`が既定で`0.9 * max_steer`相当を渡す）。
    """

    turning_radius: float
    #: 重複判定の格子。細かすぎると同じ場所を何度も展開し、粗すぎると
    #: 狭い隙間を「同じノード」と潰してしまう
    xy_resolution: float = 0.05
    yaw_resolution: float = math.radians(15.0)
    #: プリミティブ1本の弧長 [m]。格子解像度の2〜3倍が目安（短すぎると
    #: 同じ格子内に留まって展開が無駄になる）
    step: float = 0.12
    #: 片側の舵角段数。後継数 = (2*n_steer+1) * 2（前進・後退）
    n_steer: int = 3
    reverse_cost: float = 1.6
    gear_change_cost: float = 0.6
    steer_change_cost: float = 0.15
    #: 壁までの必要余裕 [m]。これを割る姿勢は**通さない**（硬い拘束）
    margin: float = 0.05
    #: これより壁に近い区間に罰則を掛ける [m]
    clearance_target: float = 0.15
    clearance_weight: float = 0.8
    #: 未知領域への罰則。**禁止ではなくコスト**（モジュールdocstring参照）
    unknown_weight: float = 0.5
    goal_xy_tol: float = 0.04
    goal_yaw_tol: float = math.radians(4.0)
    #: 目標姿勢の余裕がこれだけ負（＝わずかに食い込んで見える）でも受け入れる [m]。
    #: ★格子1cm＋円被覆1〜2cmの保守性があるので、**実際には入る駐車枠が
    #: 「数mm食い込む」と読まれることがある**。ここを0にすると、その配置で
    #: 「目標姿勢が障害物と重なっています」と即失敗して永久に動けない
    goal_slack: float = 0.03
    #: 展開の上限。超えたら失敗（呼び出し側は直前の経路を保持する）
    max_expansions: int = 4000
    #: 探索の時間予算 [s]。**実機ではこれが実質的な上限になる。**
    #: `plan()`は`planning_node`の周期ループの中で同期的に呼ばれるので、
    #: ここで長くブロックすると指令が途絶え、デッドマン（150ms、
    #: `config/vehicle.toml`の`cmd_deadman_ms`）でDISARMに落ちる。
    #: 予算切れは「失敗」として返し、車両は直前の経路を維持する
    time_budget_s: float = 0.25
    #: 何回展開ごとに目標へRSを撃つか
    analytic_interval: int = 4
    #: 解析展開で試す旋回半径の倍率。**大回りも試す**——最小半径だけだと
    #: 塞がれている配置で接続できない
    analytic_radii: tuple[float, ...] = (1.0, 1.5, 2.5)
    #: 目標からこの距離 [m] より遠いノードでは解析展開を試さない。
    #: ★遠方のRS経路はほぼ確実に壁を貫くので、掃引の判定コストが丸損になる
    #: （2026-09-16実測: 解析展開が探索時間の8割を占めていた）
    analytic_range: float = 2.0
    #: 1回の解析展開で掃引判定する候補の上限（長さの短い順）
    analytic_candidates: int = 6
    #: ヒューリスティック用の粗い格子 [m]。細かくすると正確だが
    #: Dijkstraが重い（0.10mなら6m四方で3600セル≒数十ms）
    heuristic_resolution: float = 0.10
    #: 目標からこの距離 [m] 以内でだけRSヒューリスティックを計算する。
    #: ★RS長の計算は9語族×4対称変換＝36個の生成器呼び出しで、**全ノードに
    #: 掛けると探索時間の大半を食う**（2026-09-16実測で最大10秒）。遠方では
    #: 2D測地距離の方がほぼ常に大きいので、RS項を省いても案内の質は落ちない
    #: （省略は過小評価側なので許容性も保たれる）
    rs_heuristic_range: float = 1.5


@dataclass
class HybridResult:
    path: ReedsSheppPath
    #: 解が見つかったか。Falseなら`path`は空
    ok: bool
    expansions: int
    #: どう終わったか（診断・GUI表示用）
    how: str = ""
    #: 経路に沿った最小クリアランス [m]（壁まで）
    clearance: float = 0.0
    #: この探索が実際に使った硬い拘束の閾値 [m]。**負になりうる**
    #: （目標が壁に詰まっている配置では`goal_slack`のぶん緩める）。
    #: ★呼び出し側の安全判定は**この値と同じ閾値を使わなければならない**
    #: ——計画が受け入れた経路を安全側が拒否すると、制動と再計画が交代して
    #: 永久に進まない（2026-09-16、縦列駐車でこの症状を実測）
    margin: float = 0.0


@dataclass(order=True)
class _Node:
    f: float
    #: heapqの比較でPoseまで見に行かせないための連番
    seq: int
    g: float = field(compare=False)
    pose: tuple[float, float, float] = field(compare=False)
    curvature: float = field(compare=False)
    gear: int = field(compare=False)
    parent: "_Node | None" = field(compare=False, default=None)
    #: 親からこのノードまでの弧長 [m]。経路の復元に使う
    ds: float = field(compare=False, default=0.0)


def _advance(x: float, y: float, yaw: float, curvature: float,
             ds: float) -> tuple[float, float, float]:
    """曲率一定の弧を`ds`（符号付き）進める。"""
    if abs(curvature) < 1e-9:
        return x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw
    dyaw = ds * curvature
    r = 1.0 / curvature
    return (x + r * (math.sin(yaw + dyaw) - math.sin(yaw)),
            y - r * (math.cos(yaw + dyaw) - math.cos(yaw)),
            wrap_angle(yaw + dyaw))


class _Heuristic2D:
    """非ホロノミックを無視した2D測地距離（Dijkstra）。**行き止まりを見抜く。**

    RS長だけのヒューリスティックは障害物を無視するので、袋小路の奥にある
    目標へ「まっすぐ近い」ノードを高く評価し、探索がそこへ吸い込まれる。
    測地距離は壁を迂回した実距離を返すのでこれが起きない。

    粗い格子（既定10cm）で作る。**点ロボットとして扱い、必要余裕ぶん
    膨張させた自由空間**で測る（車体の向きはここでは無視する——向きの
    拘束は真のコストを増やす方向なので、無視しても過小評価のまま＝許容的）。
    """

    def __init__(self, lmap: LocalMap, goal: tuple[float, float],
                 cfg: HybridConfig, margin: float) -> None:
        self.margin = margin
        res = cfg.heuristic_resolution
        n = max(4, int(round(lmap.size_m / res)))
        self.res = res
        self.n = n
        self.origin = (lmap.center[0] - lmap.size_m / 2.0,
                       lmap.center[1] - lmap.size_m / 2.0)
        cc, rr = np.meshgrid(np.arange(n), np.arange(n), indexing="xy")
        xs = self.origin[0] + (cc + 0.5) * res
        ys = self.origin[1] + (rr + 0.5) * res
        # 点として通れるか。車体半幅ぶん（円被覆の半径）膨張させる
        clear = lmap.clearance(xs, ys)
        free = clear >= (lmap.circle_radius + self.margin)
        self.cost = self._dijkstra(free, goal)

    def _dijkstra(self, free: np.ndarray, goal: tuple[float, float]) -> np.ndarray:
        n, res = self.n, self.res
        cost = np.full((n, n), np.inf)
        gc = int((goal[0] - self.origin[0]) / res)
        gr = int((goal[1] - self.origin[1]) / res)
        if not (0 <= gc < n and 0 <= gr < n):
            return cost
        # **目標セルが膨張後の自由空間に入らないことは普通に起こる**（壁に
        # 詰める駐車ではむしろ普通）。その場合は目標セルだけ強制的に開ける
        # ——開けないとヒューリスティックが全域∞になり探索が盲目になる
        free = free.copy()
        free[gr, gc] = True
        cost[gr, gc] = 0.0
        heap = [(0.0, gr, gc)]
        diag = math.sqrt(2.0) * res
        straight = res
        while heap:
            c, r0, c0 = heapq.heappop(heap)
            if c > cost[r0, c0]:
                continue
            for dr, dc, w in ((-1, 0, straight), (1, 0, straight),
                              (0, -1, straight), (0, 1, straight),
                              (-1, -1, diag), (-1, 1, diag),
                              (1, -1, diag), (1, 1, diag)):
                r1, c1 = r0 + dr, c0 + dc
                if not (0 <= r1 < n and 0 <= c1 < n) or not free[r1, c1]:
                    continue
                nc = c + w
                if nc < cost[r1, c1]:
                    cost[r1, c1] = nc
                    heapq.heappush(heap, (nc, r1, c1))
        return cost

    def __call__(self, x: float, y: float) -> float:
        c = int((x - self.origin[0]) / self.res)
        r = int((y - self.origin[1]) / self.res)
        if not (0 <= c < self.n and 0 <= r < self.n):
            return math.inf
        return float(self.cost[r, c])


def _merge(prims: list[tuple[int, float, float]]) -> ReedsSheppPath:
    """連続する同じ（gear, curvature）のプリミティブを1区間へまとめる。

    まとめないと`park_to_point`の追従が「区間1/40」のような細切れになり、
    区間ごとの減速処理が意味を失う。
    """
    segs: list[PathSegment] = []
    for gear, curv, length in prims:
        if segs and segs[-1].gear == gear and abs(segs[-1].curvature - curv) < 1e-9:
            segs[-1] = PathSegment(gear, curv, segs[-1].length + length)
        else:
            segs.append(PathSegment(gear, curv, length))
    return ReedsSheppPath(segments=tuple(segs))


def _reconstruct(node: _Node) -> list[tuple[int, float, float]]:
    prims: list[tuple[int, float, float]] = []
    cur: _Node | None = node
    while cur is not None and cur.parent is not None:
        prims.append((cur.gear, cur.curvature, cur.ds))
        cur = cur.parent
    prims.reverse()
    return prims


def plan(start: tuple[float, float, float], goal: tuple[float, float, float],
         lmap: LocalMap, cfg: HybridConfig) -> HybridResult:
    """`start`から`goal`への経路を探す。座標は`lmap`と同じ基準フレーム。

    見つかった経路は`ReedsSheppPath`（`(gear, curvature, length)`の列）として
    返るので、**`park_to_point`の追従はそのまま使える**。
    """
    if not lmap.ready:
        return HybridResult(ReedsSheppPath(()), False, 0, "地図が未整備")

    # ── 目標姿勢の検証と、余裕の整合 ──
    # **駐車は意図的に壁へ詰める動作**なので、目標の余裕が`cfg.margin`より
    # 小さいことは普通に起こる。そのまま`margin`を硬い拘束にすると、目標へ
    # 到達する経路が構造的に存在しなくなり、展開上限まで無駄に探索する
    # （2026-09-16実測: 縦列駐車で目標余裕3.0cm・margin5.0cmが噛み合わず0/8）。
    # ユーザーが選んだ詰め方は尊重し、**必要なぶんだけ margin を緩める**
    goal_clear = float(lmap.body_clearance(*goal))
    if goal_clear <= -cfg.goal_slack:
        return HybridResult(ReedsSheppPath(()), False, 0,
                            f"目標姿勢が障害物と重なっています（余裕{goal_clear * 100:.1f}cm）")
    margin = min(cfg.margin, max(-cfg.goal_slack, goal_clear - 0.005))

    curvatures = _curvature_levels(cfg)
    h2d = _Heuristic2D(lmap, goal[:2], cfg, margin)

    def h(pose: tuple[float, float, float]) -> float:
        hd = h2d(pose[0], pose[1])
        d = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        if d > cfg.rs_heuristic_range:
            # 遠方はRS項を省く（上記`rs_heuristic_range`の理由）。
            # 粗格子で塞がれて見える場合だけ直線距離で代替する
            return d if math.isinf(hd) else hd
        hr = shortest_path(pose, goal, cfg.turning_radius).length
        return hr if math.isinf(hd) else max(hd, hr)

    seq = 0
    root = _Node(f=0.0, seq=seq, g=0.0, pose=start, curvature=0.0, gear=1)
    root.f = h(start)
    heap = [root]
    best: dict[tuple[int, int, int], float] = {_key(start, cfg): 0.0}
    expansions = 0

    t_start = time.perf_counter()
    while heap and expansions < cfg.max_expansions:
        # 時間予算。32展開ごとにしか時計を見ない（`perf_counter`自体が
        # 1回0.1µs程度あり、1展開0.1msの探索では無視できない比率になる）
        if (expansions & 31) == 0 and time.perf_counter() - t_start > cfg.time_budget_s:
            return HybridResult(ReedsSheppPath(()), False, expansions,
                                f"時間予算{cfg.time_budget_s * 1000:.0f}msに到達",
                                0.0, margin)
        node = heapq.heappop(heap)
        k = _key(node.pose, cfg)
        if node.g > best.get(k, math.inf) + 1e-9:
            continue
        expansions += 1

        # ── 目標に十分近ければ終わり ──
        if (math.hypot(node.pose[0] - goal[0], node.pose[1] - goal[1]) <= cfg.goal_xy_tol
                and abs(wrap_angle(node.pose[2] - goal[2])) <= cfg.goal_yaw_tol):
            path = _merge(_reconstruct(node))
            return HybridResult(path, True, expansions, "格子探索で到達",
                                _clearance(lmap, start, path), margin)

        # ── 解析展開: 目標へRSを撃ってみる ──
        if (expansions % cfg.analytic_interval == 1
                and math.hypot(node.pose[0] - goal[0],
                               node.pose[1] - goal[1]) <= cfg.analytic_range):
            hit = _try_analytic(node.pose, goal, lmap, cfg, margin)
            if hit is not None:
                path = _merge(_reconstruct(node) + _as_prims(hit))
                return HybridResult(path, True, expansions, "解析展開で接続",
                                    _clearance(lmap, start, path), margin)

        # ── 後継の一括生成と一括衝突判定 ──
        for child in _successors(node, curvatures, lmap, cfg, margin):
            ck = _key(child.pose, cfg)
            if child.g >= best.get(ck, math.inf) - 1e-9:
                continue
            best[ck] = child.g
            seq += 1
            child.seq = seq
            child.f = child.g + h(child.pose)
            if math.isinf(child.f):
                continue
            heapq.heappush(heap, child)

    return HybridResult(ReedsSheppPath(()), False, expansions,
                        f"展開上限{cfg.max_expansions}に到達", 0.0, margin)


def _curvature_levels(cfg: HybridConfig) -> list[float]:
    kmax = 1.0 / cfg.turning_radius
    n = max(1, cfg.n_steer)
    return [kmax * i / n for i in range(-n, n + 1)]


def _key(pose: tuple[float, float, float], cfg: HybridConfig) -> tuple[int, int, int]:
    return (int(math.floor(pose[0] / cfg.xy_resolution)),
            int(math.floor(pose[1] / cfg.xy_resolution)),
            int(math.floor(wrap_angle(pose[2]) / cfg.yaw_resolution)))


#: プリミティブ1本を何点でサンプルして衝突判定するか。弧長12cm・車体37cmなら
#: 端点2点で十分（間をすり抜ける障害物は車体より小さい）
_SAMPLES = 2


def _successors(node: _Node, curvatures: list[float], lmap: LocalMap,
                cfg: HybridConfig, margin: float) -> list[_Node]:
    """1ノードの全後継を作り、**1回のnumpy呼び出しで衝突判定**する。"""
    x0, y0, yaw0 = node.pose
    poses: list[tuple[float, float, float]] = []
    meta: list[tuple[int, float]] = []
    samples: list[tuple[float, float, float]] = []
    for gear in (1, -1):
        for curv in curvatures:
            ds = cfg.step * gear
            p = (x0, y0, yaw0)
            mid = []
            for i in range(_SAMPLES):
                p = _advance(*p, curv, ds / _SAMPLES)
                mid.append(p)
            poses.append(p)
            meta.append((gear, curv))
            samples.extend(mid)

    arr = np.asarray(samples, dtype=np.float64)
    wall = lmap.body_clearance(arr[:, 0], arr[:, 1], arr[:, 2])
    #: 今いる場所の余裕。**すでに食い込んでいる状態から脱出できるようにする**
    here = float(lmap.body_clearance(*node.pose))
    unk = lmap.body_clearance(arr[:, 0], arr[:, 1], arr[:, 2], include_unknown=True)
    wall = wall.reshape(len(poses), _SAMPLES)
    unk = unk.reshape(len(poses), _SAMPLES)

    out: list[_Node] = []
    for i, (gear, curv) in enumerate(meta):
        w = float(wall[i].min())
        if w < margin:
            # 壁は硬い拘束。**ただし既に食い込んでいる場所からは、余裕が
            # 増える向きの動きだけは許す**——そうしないと、追従誤差で車体が
            # 壁際に入り込んだ瞬間に全後継が却下され、その場から一歩も
            # 動けなくなる（2026-09-16、車庫入れで「食い込み5cm」を報告し
            # 続けて固まる挙動を実測）
            if not (here < margin and w > here + 1e-4):
                continue
        cost = cfg.step * (cfg.reverse_cost if gear < 0 else 1.0)
        if gear != node.gear:
            cost += cfg.gear_change_cost
        if abs(curv - node.curvature) > 1e-9:
            cost += cfg.steer_change_cost
        short = max(0.0, cfg.clearance_target - w)
        cost += cfg.clearance_weight * short
        # 未知は禁止せずコストで避ける
        unk_short = max(0.0, cfg.clearance_target - float(unk[i].min()))
        cost += cfg.unknown_weight * unk_short
        out.append(_Node(f=0.0, seq=0, g=node.g + cost, pose=poses[i],
                         curvature=curv, gear=gear, parent=node, ds=cfg.step))
    return out


def _try_analytic(pose: tuple[float, float, float], goal: tuple[float, float, float],
                  lmap: LocalMap, cfg: HybridConfig,
                  margin: float) -> ReedsSheppPath | None:
    """`pose`から目標へRSを撃ち、衝突しない最短の候補を返す。

    **複数の旋回半径を試す。** 最小半径だけだと、その弧が塞がれている配置で
    接続できず、探索が最後の詰めで発散する。掃引の判定は重いので、
    全半径の候補を長さでまとめて並べ、短い方から`analytic_candidates`本だけ見る。
    """
    cands: list[ReedsSheppPath] = []
    for k in cfg.analytic_radii:
        cands.extend(c for c in candidate_paths(pose, goal, cfg.turning_radius * k)
                     if c.segments)
    cands.sort(key=lambda c: c.length)
    for cand in cands[:cfg.analytic_candidates]:
        poses = sample_path_array(pose, cand, step=cfg.xy_resolution)
        if lmap.path_clearance(poses) >= margin:
            return cand
    return None


def _as_prims(path: ReedsSheppPath) -> list[tuple[int, float, float]]:
    return [(s.gear, s.curvature, s.length) for s in path.segments]


def _clearance(lmap: LocalMap, start: tuple[float, float, float],
               path: ReedsSheppPath) -> float:
    if not path.segments:
        return 0.0
    return lmap.path_clearance(sample_path_array(start, path, step=0.03))
