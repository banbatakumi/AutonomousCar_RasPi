"""経路選択走行（`slam2d`版）— 分岐のあるコースを、経由点のグループで経路を選んで走る。

`slam2d_raceline.Slam2dRaceLine` の派生。**地図作成（EXPLORE）・自己位置の復元
（LOCATE）・地図の保存はそのまま使い、経路の作り方（BUILD）と走り方（RACE）だけを
差し替える。** 分岐の無いコースで両者を並べて比べられるよう、別 planner にしてある。

## 何が違うか

| | `slam2d_raceline` | `slam2d_route`（これ） |
|---|---|---|
| 経路の元 | 地図作成の最後の1周の軌跡 | 地図全体の道路グラフ（`nav/roadgraph.py`） |
| 分岐 | 走った枝しか無い | 経由点で選ぶ（`nav/route.py`） |
| 道幅の測り方 | 地図全体でレイ | 選んだ経路の回廊だけでレイ（分岐の口で測りすぎない） |
| 走行中の切替 | なし | グループA〜D（GUI・信号認識、`nav/route_switch.py`） |
| 終わり方 | 走り続ける | ミッション（n周／t秒 → 停止点で停止 or 駐車） |

経由点を1つも置いていない間は**自動経路**（キー `"auto"`）で走る。スタート（地図作成の
出発点とその向き）から出て戻る**周回ルートを道路グラフ上で全部挙げ**（`route.enumerate_loops`）、
それぞれレーシングラインを引いて**見積もりのラップタイムが最短のもの**を選ぶ（地図作成で
走った1周には頼らない）。ルールで通れない道は経路の設定の `avoid`（避ける点）で除く。
**自動経路の経由点は経路の設定（グループA〜D）には書き込まない**——経由点は人間が全部決める
（バンビの指示、2026-09-29）。**自動経路は経由点のグループがあっても作り続け、A〜D と同じく
走行中に選べる**（「自動」ボタン、`request_route("auto")`。以前はグループを置くと消えて戻せなかった）。

## 重い計算はワーカースレッドで

道路グラフ（Pi で数百ms）と経路ごとのレーシングライン（1本 数十〜数百ms）は
`plan()`（10Hz）の中で回すと指令が途切れる（`cmd_deadman_ms` 150ms で DISARM）。
ワーカースレッド1本に投げ、`plan()` は毎周期出来上がりを覗くだけにする。
ワーカーは地図の**写し**（3値から作り直した `OccGrid`）だけを触る。

## 停止と駐車

ミッションが終わると、今の位置から停止点までの**開いた経路**をグラフで引き
（`route.plan_to`）、終点で0 m/s になる速度で走る。終点の手前 `creep_dist` は
`creep_speed` で這い、残り距離が「今の速度で止まれる距離」を切ったら制動する。
`mode="park"` の停止点では、道の上の最寄り点で止まってから `ParkToPoint`
（Reeds-Shepp＋Hybrid A*、LiDAR の局所地図）へ引き継ぐ。**静止してから渡す**のは、
`park_to_point` が低速・局所地図前提の設計だから。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..msgs.types import AutoMap, AutoState, Scan, VehicleState
from ..nav import centerline as cl_mod
from ..nav import raceline as rl_mod
from ..nav.purepursuit import nearest_index
from ..nav.roadgraph import RoadGraph, build_graph
from ..nav.route import (RouteError, RoutePath, Waypoint, build_raceline, enumerate_loops,
                         lap_time, plan_loop, plan_to, tightest_radius, waypoints_from_traj)
from ..nav.route_switch import Mission, RouteSwitcher
from . import mapstore
from ._slam2d_nav import occgrid_from_trinary
from .base import ParamSpec
from .park_to_point import ParkToPoint
from .route_config import AUTO_KEY, ROUTE_KEYS, RouteConfig, Stop
from .slam2d_raceline import (DETOUR, DONE, GROUP_LINE, PATH_STEP, SPEED_KEYS, Slam2dRaceLine,
                              forward_traj, line_kwargs)

__all__ = ["Slam2dRoute"]

#: GUI の組（`ParamSpec.group`）
GROUP_ROUTE = "本番走行: 経路の切替・停止"
#: 停止点に着いて止まっている / 駐車（`ParkToPoint`）に引き継いだ
STOPPED, PARK = "STOPPED", "PARK"
#: 停止経路のキー（`RouteSwitcher.active_key`）
_STOP_KEY = "stop"
#: 駐車の停止点は道から離れている（駐車枠の中）ので、吸着の半径を広く取る
_PARK_SNAP_M = 2.5
#: 停止点への経路を作れなかったとき、次に試すまでの時間 [s]
_STOP_RETRY_S = 1.0
#: 停止経路を計算している間に車が進むと見込む時間 [s]（`_compute_stop` の「近すぎる」の判定）
_STOP_LEAD_S = 0.4


@dataclass
class _BuildOut:
    graph: RoadGraph
    routes: dict[str, rl_mod.RaceLine] = field(default_factory=dict)
    centerlines: dict[str, cl_mod.Centerline] = field(default_factory=dict)
    paths: dict[str, RoutePath] = field(default_factory=dict)
    errors: dict[str, str] = field(default_factory=dict)
    #: 経路はできたが気をつけること（曲がり切れない恐れのある急な所など）
    warnings: dict[str, str] = field(default_factory=dict)
    #: 自動経路をどう選んだか（候補の数とラップタイム、GUI の表示用）
    auto_note: str = ""


#: 経路の作り直しが要る設定（走行中に変わったら作り直す、`plan()`）。`graph_margin` は
#: 道路グラフから作り直す（`Slam2dRoute._submit`）
_LINE_KEYS = SPEED_KEYS + ("line_margin", "turn_max_deg", "graph_margin")

#: 経路の切替で「新しい経路の上に居る」とみなす横ずれ [m]（`RouteSwitcher.step` の `tol`）。
#: ★以前は GUI のスライダだったが、これは「どれだけ待つか」ではなく「別々に最適化した経路どうしの
#: 重なりの誤差（数cm）を飲む幅」で、乗り換えは押した後の最初の乗れる周期に起きる。上げると
#: 別の枝の上から乗り換えて経路へ斜めに突っ込む（安全弁）、下げると重なり区間でも乗れなくなるので
#: 触る理由が無く定数にした
SWITCH_TOL = 0.15


def _line_kwargs(p: dict[str, float], vehicle) -> dict:
    return dict(line_kwargs(p, vehicle), step=PATH_STEP)


def _compute_routes(trinary: np.ndarray, resolution: float, origin: tuple[float, float],
                    seed, traj, groups: dict[str, list[Waypoint]], p: dict[str, float],
                    vehicle, graph: RoadGraph | None, *, auto_start: Waypoint | None = None,
                    avoid: list[tuple[float, float]] | None = None) -> _BuildOut:
    """ワーカースレッドで走る。**引数の写しだけを触る**（planner の状態を読まない）。

    `auto_start` があれば、グループとは別に自動経路（キー `auto`）を全周回の中の最速から選ぶ。
    """
    grid = occgrid_from_trinary(trinary, resolution=resolution, origin=origin, seq=0)
    if graph is None:
        graph = build_graph(trinary, resolution=resolution, origin=origin,
                            keep=vehicle.half_width + p["graph_margin"], seed=seed, traj=traj)
    out = _BuildOut(graph=graph)
    kw = _line_kwargs(p, vehicle)
    if auto_start is not None:
        _fastest_loop(out, grid, graph, auto_start, avoid, p, kw, vehicle)
    for key, wps in groups.items():
        try:
            rp = plan_loop(graph, wps, turn_max_deg=p["turn_max_deg"], avoid=avoid)
            rl, cl = build_raceline(grid, graph, rp, **kw)
        except (RouteError, ValueError, np.linalg.LinAlgError) as e:
            out.errors[key] = str(e)
            continue
        except Exception as e:              # noqa: BLE001
            # 1つのグループの想定外の失敗で、他のグループまで作れなくしない
            out.errors[key] = f"内部エラー: {e!r}"
            continue
        _put(out, key, rl, cl, rp, vehicle)
    return out


def _put(out: _BuildOut, key: str, rl, cl, rp, vehicle) -> None:
    out.routes[key], out.centerlines[key], out.paths[key] = rl, cl, rp
    r, i = tightest_radius(rl, PATH_STEP)
    r_min = 1.0 / vehicle.kappa_max
    if r < 1.1 * r_min:
        x, y = rl.xy[i]
        out.warnings[key] = (f"({x:.1f}, {y:.1f}) の半径{r * 100:.0f}cmは車の最小旋回半径"
                             f"{r_min * 100:.0f}cmに近い（最低速度で通る。外れるなら経由点を見直す）")


#: 自動経路で、見積もりの速い順に本来の設定で引き直して曲がれるか確かめる本数
_FULL_CHECKS = 4
#: 自動経路で比べる周回ルートの上限（Pi で1本 数百ms。これ以上はワーカーでも待たせすぎる）
_MAX_AUTO_LOOPS = 32


def _fastest_loop(out: _BuildOut, grid, graph: RoadGraph, start: Waypoint, avoid,
                  p: dict[str, float], kw: dict, vehicle) -> None:
    """スタートから出て戻る周回ルートを全部挙げ、見積もりのラップタイムが最短のものを
    自動経路にする（モジュール docstring）。"""
    try:
        loops = enumerate_loops(graph, start, turn_max_deg=p["turn_max_deg"], avoid=avoid,
                                max_loops=_MAX_AUTO_LOOPS)
    except RouteError as e:
        out.errors[AUTO_KEY] = str(e)
        return
    if not loops:
        out.errors[AUTO_KEY] = "スタートから戻ってくる周回ルートが無い（避ける点・曲がり角の上限を見直す）"
        return
    # 候補どうしは安い設定（最小曲率・車体外形の検査なし）で並べ、速い順に本来の設定で
    # 引き直して確かめる。**全候補を本来の設定で引くと Pi で数十秒かかる**（候補は最大
    # 32本、最小時間への寄せと外形の検査で1本あたり10倍前後重い）ので `_FULL_CHECKS` 本まで
    quick = {**kw, "time_iters": 0, "footprint": None}
    ranked = []
    last_err = ""
    for rp in loops:
        try:
            rl, cl = build_raceline(grid, graph, rp, **quick)
        except Exception as e:              # noqa: BLE001  1本の失敗で全部を止めない
            last_err = repr(e)
            continue
        ranked.append((lap_time(rl), rl, cl, rp))
    if not ranked:
        out.errors[AUTO_KEY] = f"周回ルートはあるがレーシングラインを引けなかった（{last_err}）"
        return
    ranked.sort(key=lambda t: t[0])

    # ★ 車の最小旋回半径より急な所がある周回は、曲がれる周回がある限り選ばない。
    # 見積もりのラップは最低速度で通る前提なので速く見えるが、実際には曲がり切れずに
    # 経路から外れて止まる（地図作成の誤差で近道の S 字が歪んだ toyota2 の地図で、
    # 半径34cm＜40cm の近道を「最速」に選んで外れた。sim.bench 2026-09-29）。
    # **判定は本来の設定で引いた線で行う**——安い設定（最小曲率のみ）の線は角が急で、
    # 曲がれる周回（本来の線で半径61cm）まで 34cm と出て全部が「曲がれない」になった
    r_car = 1.0 / vehicle.kappa_max
    full = []
    for *_, rp in ranked[:_FULL_CHECKS]:
        try:
            rl, cl = build_raceline(grid, graph, rp, **kw)
        except Exception as e:              # noqa: BLE001
            # 安い線は車体外形を検査していないので走らせない。この候補は除く
            last_err = repr(e)
            continue
        r_tight, _ = tightest_radius(rl, PATH_STEP)
        full.append((r_tight < r_car, lap_time(rl), rl, cl, rp))
    if not full:
        out.errors[AUTO_KEY] = ("速い周回ルートのどれにも本来の設定でレーシングラインを"
                                f"引けなかった（{last_err}）")
        return
    full.sort(key=lambda t: (t[0], t[1]))
    tight, t_best, rl, cl, rp = full[0]
    _put(out, AUTO_KEY, rl, cl, rp, vehicle)
    others = sorted(t for bad, t, *_ in full[1:] if bad == tight)
    n_bad = sum(1 for bad, *_ in full if bad)
    out.auto_note = (f"自動経路: 周回ルート{len(ranked)}通りを比べて最速（見積もり{t_best:.1f}s"
                     + (f"、次点{others[0]:.1f}s" if others else "")
                     + ("。★確かめた周回すべてに車の最小旋回半径より急な所がある" if tight
                        else f"。曲がり切れない周回{n_bad}通りは除いた" if n_bad else "")
                     + "）")


def _compute_stop(trinary: np.ndarray, resolution: float, origin: tuple[float, float],
                  graph: RoadGraph, pose: tuple[float, float, float], stop: Stop,
                  v_now: float, p: dict[str, float], vehicle,
                  avoid: list[tuple[float, float]] | None = None,
                  start_edges: frozenset[int] | None = None) -> tuple[rl_mod.RaceLine, str]:
    """停止点への開いた経路と、気をつけること（無ければ空文字）。ワーカースレッドで走る。"""
    grid = occgrid_from_trinary(trinary, resolution=resolution, origin=origin, seq=0)
    park = stop.mode == "park"
    goal = Waypoint(stop.x, stop.y, None if park else stop.yaw)
    # 停止点が近すぎる（今の速度から止まり切れない）なら1周回ってから止まる。
    # ★固定 1.2m だと、周回を数えた直後（2m/s）に 1.2m 先の停止点へ向かって減速し切れず、
    # 止まる位置が 18cm ずれた（toyota2 の sim.bench、2026-09-29）
    v = max(0.0, v_now)
    # 計算している間（数百ms）に進むぶん（`_STOP_LEAD_S`）も足す。経路は今の位置から引く
    need = (v * v / (2.0 * max(p["a_brake"], 0.1)) + p["creep_dist"] + 0.5
            + _STOP_LEAD_S * v)
    rp = plan_to(graph, pose, goal, turn_max_deg=p["turn_max_deg"],
                 snap_radius=_PARK_SNAP_M if park else 0.6,
                 min_len=need, end_at_goal=not park,
                 extend=p["park_lead"] if park else 0.0, avoid=avoid,
                 start_edges=start_edges)
    kw = _line_kwargs(p, vehicle)
    kw["v_min"] = p["creep_speed"]
    rl, _ = build_raceline(grid, graph, rp, v_start=v_now, v_end=p["creep_speed"], **kw)
    # 停止点が道の中心から横に離れていると、最後が真横への折れになる（吸着点→停止点は
    # 中心線に垂直）。曲がり切れない半径なら知らせる（止まる位置がそのぶんずれる）
    warn = ""
    r, _i = tightest_radius(rl, PATH_STEP)
    r_min = 1.0 / vehicle.kappa_max
    if r < r_min:
        warn = (f"停止点への経路に半径{r * 100:.0f}cmの所がある（車の最小旋回半径"
                f"{r_min * 100:.0f}cm）。停止点を道の中心へ寄せると止まる位置が合う")
    return rl, warn


class Slam2dRoute(Slam2dRaceLine):
    id = "slam2d_route"
    name = "経路選択走行(slam2d)"
    description = ("slam2dの地図全体から道路グラフを作り、経由点のグループで分岐を選んで周回する。"
                   "走行中の切替・n周後の停止/駐車に対応")
    #: GUI が地図パネル・地図ライブラリ・経由点エディタを出す planner（`registry.catalog`）
    map_ui = True
    routes_ui = True
    #: 経路は道路グラフから自前で作り直す（`_submit`）
    _reline_on_load = False

    params = Slam2dRaceLine.params + (
        ParamSpec(group=GROUP_LINE, key="graph_margin", label="道路グラフの余裕", min=0.0, max=0.2,
                  default=0.02, step=0.01, unit="m",
                  note="車体半幅にこれを足した幅が取れない道はグラフから消える（通れない扱い）"),
        ParamSpec(group=GROUP_LINE, key="turn_max_deg", label="分岐で曲がれる角度", min=45.0, max=150.0,
                  default=100.0, step=5.0, unit="°",
                  note="★節点でこれより急に向きを変える経路は選ばない（Y字の枝から枝への"
                       "折り返しを禁じる）。節点から1m先までの向きで測る"),
        ParamSpec(group=GROUP_ROUTE, key="creep_speed", label="停止前の這う速度", min=0.1, max=0.6,
                  default=0.25, step=0.05, unit="m/s", note="停止点の手前ではこれで近づく"),
        ParamSpec(group=GROUP_ROUTE, key="creep_dist", label="這い始める距離", min=0.2, max=2.0,
                  default=0.6, step=0.05, unit="m", note="停止点のこれだけ手前から這う"),
        ParamSpec(group=GROUP_ROUTE, key="park_lead", label="駐車の手前で止まる位置", min=0.0, max=2.0,
                  default=1.0, step=0.1, unit="m",
                  note="駐車の停止点では、道の上で枠に最も近い点からこれだけ先で止まって"
                       "park_to_pointへ渡す（バック駐車は枠を通り過ぎてから後退で入れる）。"
                       "0だと枠の真横で止まり、横へずらす切り返しを繰り返した（sim.benchで実測）"),
        ParamSpec(group=GROUP_ROUTE, key="stop_tol", label="停止の余裕", min=0.0, max=0.3,
                  default=0.02, step=0.01, unit="m",
                  note="残り距離が「止まれる距離＋これ」を切ったら制動する"),
    )

    # ── 状態 ──

    def reset(self) -> None:
        super().reset()
        self._cfg = RouteConfig()
        self._graph: RoadGraph | None = None
        self._routes: dict[str, rl_mod.RaceLine] = {}
        self._route_paths: dict[str, RoutePath] = {}
        self._route_cls: dict[str, cl_mod.Centerline] = {}
        self._route_err: dict[str, str] = {}
        self._route_warn: dict[str, str] = {}
        self._drop_job("_job")
        self._job_again = False
        #: やり直しの計算で道路グラフから作り直すか（`_submit`）
        self._regraph = False
        self._drop_job("_stop_job")
        self._stop_error = ""
        #: 停止点への経路を作れなかった。**この時刻（`_race_t`）までは作り直しに行かない**
        #: （毎周期、重い計算を投げ続けない。`_maybe_finish`）
        self._stop_retry_t = 0.0
        #: RACE の走り出しの初期化を済ませたか（`_race`）
        self._race_started = False
        self._switch = RouteSwitcher()
        self._switch_note = ""
        self._map_name = ""
        self._route_ver = getattr(self, "_route_ver", 0) + 1   # 版は戻さない（配信の判定に使う）
        #: `snapshot()` で最後に詰めた `AutoMap` と、そのときの `_route_ver`
        self._snap_map: AutoMap | None = None
        self._snap_ver = -1
        self._race_t = 0.0
        self._stopping = ""                  #: 向かっている停止点の名前
        self._stop: Stop | None = None
        self._park: ParkToPoint | None = None
        self._note = ""                      #: 設定エラー等（GUI の reason に添える）
        self._p: dict[str, float] | None = None
        #: 今の経路を作った設定（`_LINE_KEYS`）。`None` = まだ作っていない
        self._built_key: tuple | None = None
        self._want_group = ""
        #: 自動経路の元（**経路の設定には書かない**）。ふつうはスタート（地図作成の出発点と
        #: 向き）から全周回を比べる。地図作成の軌跡が無い古い地図だけ、保存した
        #: レーシングラインに沿って置いた経由点で代わりにする
        self._auto_start: Waypoint | None = None
        self._auto_wps: list[Waypoint] = []
        self._auto_note = ""

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        self._p = p
        # ★ 速度・経路の設定が変わったら作り直す（以前は地図を読んだときの設定のまま
        # だった）。できた版へは `_install` が乗れる場所で移る
        if (self._graph is not None and self._built_key is not None
                and self._line_key(p) != self._built_key):
            # `graph_margin` が変わったら道路グラフから作り直す（通れる道そのものが変わる）
            same_graph = float(p["graph_margin"]) == self._built_key[-1]
            self._submit(graph=self._graph if same_graph else None)
        # 出来上がった経路は**どの段でも**取り込む（DONE で経由点を編集して保存した
        # ときも、走り出す前に GUI へ新しい経路を出すため）
        self._poll()
        st = super().plan(scan, vs, p, dt)
        self._fill_route_state(st)
        return st

    # ── 人間・信号からの要求 ──

    def request_load(self, name: str) -> None:
        super().request_load(name)
        if self._load_error:
            return
        self._map_name = name
        self._switch.reset()
        self._routes, self._route_cls, self._route_err, self._route_warn = {}, {}, {}, {}
        self._route_paths = {}
        self._graph = None
        self._drop_job("_job")               # 前の地図の計算は結果を捨てる
        self._job_again = False
        self._drop_job("_stop_job")
        self._stop_retry_t = 0.0
        self._stop_error = self._build_error = self._auto_note = ""
        self._race_started = False
        self._stopping, self._stop, self._park = "", None, None
        self._want_group = ""
        self._switch_note = ""
        self._note = ""
        try:
            self._cfg = RouteConfig.from_json(mapstore.load_routes(name))
        except ValueError as e:
            self._cfg = RouteConfig()
            self._note = f"経路の設定を読めない（{e}）。既定の経路で走る"
        # 自動経路の元（経路の設定には書かない、モジュール docstring）
        self._auto_start = _start_of(self._cfg.explore_traj)
        self._auto_wps = (waypoints_from_traj(self.path.xy)
                          if self._auto_start is None and self.path is not None
                          and len(self.path) >= 20 else [])
        self._submit(graph=None)

    def request_routes(self, text: str) -> None:
        """GUI の経由点エディタが「保存」したときに呼ばれる。"""
        try:
            cfg = RouteConfig.from_json(text)
        except ValueError as e:
            self._note = str(e)
            return
        self._note = ""
        if cfg.explore_traj is None:
            cfg.explore_traj = self._cfg.explore_traj
        rebuild = not cfg.same_groups(self._cfg)
        self._cfg = cfg
        self._stop_retry_t = 0.0            # 停止点・ミッションを直したらすぐ試す
        self._stop_error = ""
        if self._map_name:
            mapstore.save_routes(self._map_name, cfg.to_json())
        # ★ 最初の計算がまだ走っている間（`_graph` が無い）の編集も取りこぼさない。
        #   `_submit` は計算中なら「終わったらやり直す」印を立てるだけ
        if rebuild and (self._graph is not None or self._job is not None):
            self._submit(graph=self._graph)
        if self._build_error and self.phase == DONE:
            self._build_error = ""          # 経路が無いまま DONE に居る。作り直しの結果を待つ
        self._map_dirty = True
        self._route_ver += 1

    def request_route(self, group: str, source: str = "gui") -> None:
        """走行中に経路のグループを切り替える（GUI の A〜D ボタン、信号認識）。"""
        if self._stopping:
            self._switch_note = "停止点へ向かっているので切り替えない"
            return
        self._want_group = group
        line = self._routes.get(group)
        if line is None:
            self._switch_note = f"グループ{group}の経路が無い"
            return
        self._switch_note = ""
        self._switch.request(group, line, source)

    def request_signal(self, value: str, source: str = "signal") -> None:
        """信号認識（`route/select` トピック）の値。`signal_map` でグループに読み替える。

        値そのものがグループ名（A〜D）なら、対応表が無くてもそのグループへ切り替える。
        """
        group = self._cfg.signal_map.get(value) or (value if value in ROUTE_KEYS else "")
        if not group:
            self._switch_note = f"信号『{value}』に対応するグループが無い"
            return
        self.request_route(group, f"{source}:{value}")

    # ── ワーカー ──

    def _submit(self, *, graph: RoadGraph | None) -> None:
        if self._job is not None:
            self._job_again = True          # 走っている計算が終わってからやり直す
            self._regraph = self._regraph or graph is None
            return
        self._regraph = False
        p = self._p or {s.key: s.default for s in self.params}
        self._built_key = self._line_key(p)
        g = self.slam.grid
        seed = None
        traj = self._cfg.explore_traj
        if traj is not None and len(traj):
            seed = (float(traj[0, 0]), float(traj[0, 1]))
        self._job = self._pool().submit(
            _compute_routes, g.trinary().copy(), g.resolution, tuple(g.origin), seed,
            None if traj is None else traj.copy(), self._groups_to_build(), dict(p),
            self.vehicle, graph,
            auto_start=self._auto_start,
            avoid=list(self._cfg.avoid))

    @staticmethod
    def _line_key(p: dict[str, float]) -> tuple:
        return tuple(float(p[k]) for k in _LINE_KEYS)

    def _groups_to_build(self) -> dict[str, list[Waypoint]]:
        """作る経路。経由点のグループ＋自動経路（スタートから全周回を比べる場合は
        `_compute_routes` の `auto_start` 側で作るのでここには入れない）。"""
        out = dict(self._cfg.groups)
        if self._auto_start is None and self._auto_wps:
            out[AUTO_KEY] = list(self._auto_wps)
        return out

    def _route_paths_edges(self, key: str) -> set[int]:
        """経路 `key` が通る道路グラフのエッジ（診断・テスト用）。"""
        rp = self._route_paths.get(key)
        return set(rp.edge_ids) if rp is not None else set()

    def _start_key(self) -> str:
        """走り出す経路。設定の開始（A〜D か自動）→ 無ければ自動経路 → 最初にできた経路。"""
        if self._cfg.active in self._routes:
            return self._cfg.active
        if AUTO_KEY in self._routes:
            return AUTO_KEY
        return next(iter(self._routes), "")

    def _poll(self) -> bool:
        """計算が終わっていれば取り込む。**まだ走っていれば False。**"""
        if self._job is not None:
            if not self._job.done():
                return False
            job, self._job = self._job, None
            try:
                out: _BuildOut = job.result()
            except Exception as e:          # noqa: BLE001
                msg = f"道路グラフを作れなかった: {e}"
                # 走れる経路が既にあるなら（走行中の作り直しの失敗）それで走り続ける
                if self._routes:
                    self._note = msg
                else:
                    self._build_error = msg
            else:
                self._install(out)
            if self._job_again:
                self._job_again = False
                self._submit(graph=None if self._regraph else self._graph)
        if self._stop_job is not None and self._stop_job.done():
            job, self._stop_job = self._stop_job, None
            try:
                line, warn = job.result()
            except Exception as e:          # noqa: BLE001
                # 周回を続け、少し進んでからもう一度試す（車の位置で通るかどうかが変わる。
                # 毎周期は投げない）
                self._stop_error = f"停止点への経路を作れなかった（周回を続けて試し直す）: {e}"
                self._stop_retry_t = self._race_t + _STOP_RETRY_S
                self._stopping = ""
            else:
                self._stop_error = warn
                self._set_active(_STOP_KEY, line)
                # ★ 停止経路は**始点から**追う。1周回ってから止まる経路は終わりの区間が
                #   始めの区間と同じ場所を通るので、最寄り点を全体から探すと終わり側に
                #   貼り付き、1周せずにそのまま止まりに行った
                self._hint = 0
        return True

    def _set_active(self, key: str, line: rl_mod.RaceLine) -> None:
        """保留なしで経路を差し替える（走り出し・停止経路）。GUI へも配り直す。"""
        self._switch.set_active(key, line)
        self._hint = -1
        self._prev_idx = -1
        self._map_dirty = True
        self._route_ver += 1

    def _install(self, out: _BuildOut) -> None:
        self._graph = out.graph
        self._routes, self._route_cls, self._route_err = out.routes, out.centerlines, out.errors
        self._route_paths = out.paths
        self._route_warn = out.warnings
        self._auto_note = out.auto_note
        self._route_ver += 1
        self._map_dirty = True
        if self._routes:
            self._build_error = ""          # 経路ができた（DONE で経由点を直した後など）
        if self._stopping:
            return                          # 停止点へ向かっている間は乗り換えない
        key = self._switch.active_key
        if key in self._routes and self._switch.active is not None:
            # 経由点の編集で今の経路が作り直された。乗れる場所で新しい版へ移る
            self._switch.request(key, self._routes[key], "編集")
        elif (self._switch.active is not None and key != _STOP_KEY
              and self._start_key()):
            # 今の経路が無くなった（自動経路で走っている間に経由点を置いた等）。
            # 開始グループへ乗れる場所で移る
            k = self._start_key()
            self._switch.request(k, self._routes[k], "編集")
        if self._want_group in self._routes and self._want_group != key:
            self._switch.request(self._want_group, self._routes[self._want_group],
                                 self._switch.source)

    # ── BUILD（地図作成の直後） ──

    def _build(self, st: AutoState, p: dict[str, float]) -> AutoState:
        if self._build_error:
            st.reason = self._build_error
            return st
        if self._job is None and self._graph is None:
            lap = self._last_lap()
            if len(lap) < 20:
                self._build_error = f"経路を作れなかった: 軌跡が短すぎる（{len(lap)}点）"
                st.reason = self._build_error
                return st
            self._cfg.explore_traj = RouteConfig.decimate(
                forward_traj(self.slam.trajectory_array()))
            # 自動経路の元（経由点が無い間だけ使う。経路の設定には書かない）
            self._auto_start = _start_of(self._cfg.explore_traj)
            self._auto_wps = [] if self._auto_start is not None else waypoints_from_traj(lap)
            self._submit(graph=None)
        if not self._poll():
            st.reason = "経路を作っている（道路グラフ → 経路 → レーシングライン）"
            return st
        key = self._start_key()
        if not key and not self._build_error:
            errs = "・".join(f"{k}: {v}" for k, v in self._route_err.items())
            self._build_error = f"経路を作れなかった（{errs or '経由点が無い'}）"
        # ★ **経路が1本も作れなくても、地図は保存して DONE へ進む。** 以前は BUILD のまま
        #   固まり、地図は未保存で、経由点を置き直しても進まなかった（数分かけた地図作成が
        #   無駄になる）。経路は DONE で経由点・避ける点を直せば作り直される
        self.path = self._routes.get(key)
        self.centerline = self._route_cls.get(key)
        self.phase = st.phase = DONE
        self._hint = -1
        self._map_dirty = True
        self._saved_map_name = self._auto_save_map()
        if self._saved_map_name:
            self._map_name = self._saved_map_name
            mapstore.save_routes(self._saved_map_name, self._cfg.to_json())
        saved = (f"地図を『{self._saved_map_name}』として保存した。" if self._saved_map_name
                 else f"{self._save_error}。")
        if not key:
            st.reason = f"★{self._build_error}。{saved}経由点・避ける点を見直してください"
            return st
        g = self._graph
        st.reason = (f"経路ができた（道路グラフ: 分岐 {sum(1 for n in g.nodes if len(n.edges) >= 3)}・"
                     f"区間 {len(g.edges)}、"
                     + ((self._auto_note or "経由点が無いので自動経路を作った") if key == AUTO_KEY
                        else f"グループ {'/'.join(self._routes)}") + "）。"
                     + saved + ("経由点を編集するか、走行で開始してください"
                                if self._saved_map_name else ""))
        return st

    def _done(self, st: AutoState) -> AutoState:
        if not self._routes:
            name = f"『{self._saved_map_name}』" if self._saved_map_name else ""
            st.reason = ("経路を作っている" if self._job is not None else
                         f"★走れる経路が無い（{self._build_error or '経由点を置いてください'}）。"
                         f"地図{name}は保存済み。経由点・避ける点を見直してください")
            return st
        return super()._done(st)

    # ── RACE / STOPPED / PARK ──

    def _race(self, st: AutoState, scan: Scan, vs: VehicleState,
              p: dict[str, float], lost: bool) -> AutoState:
        if self.phase == DETOUR:
            return self._detour_step(st, scan, vs, p)
        if self.phase == PARK:
            return self._park_step(st, scan, vs, p)
        if self.phase == STOPPED:
            st.brake = True
            st.ready = True
            st.target_speed = 0.0
            st.reason = f"停止点『{self._stopping}』で停止した"
            return st

        if not self._race_started:
            if not self._routes:
                st.reason = ("経路を準備中" if self._job is not None else
                             f"経路が無い（{self._build_error or self._note or '経由点を置いてください'}）")
                return st
            # ★ 走り出しの初期化は「まだ始めていない」で判定する。LOCATE の間に切替ボタンや
            #   信号が来ると今の経路が先に決まるので、「今の経路が無い」で判定すると初期化が
            #   飛び、前回の走行の経過時間（時間ミッションが即満了）と周回の数えかけが残った
            if self._switch.active is None:
                key = self._start_key()
                self._set_active(key, self._routes[key])
            else:
                self._set_active(self._switch.active_key, self._switch.active)
            self._race_started = True
            self._lap_s = 0.0
            self._joined = False
            self._race_t = 0.0
            self.laps = 0

        coasting = self._localization_ok(st, lost, p)
        if coasting is None:
            return st

        pose = self.slam.pose
        if not coasting and self._avoid is None:
            # 推測航法で続行している間は乗り換えない（乗り換え位置の判定が自己位置頼み）。
            # 障害物を避けている間も乗り換えない（避ける経路の元が変わって外れる）
            sw = self._switch.step(tuple(pose), vs.speed, tol=SWITCH_TOL, a_brake=p["a_brake"])
            if sw.switched:
                self._hint = sw.hint
                self._prev_idx = -1
                self._map_dirty = True
                self._route_ver += 1
            self._switch_note = sw.reason or (self._switch_note if self._switch.pending else "")
        path = self._switch.active
        assert path is not None
        self.path = path
        self._race_t += self._dt

        pp, hit, dist = self._track(st, scan, vs, p, path, pose)

        if path.closed:
            self._count_lap(path, pp.index)
            st.laps = self.laps
            self._maybe_finish(pose, vs, p)

        if self._off_route(st, pp, vs, p):
            return st

        st.ready = True
        st.target_speed = pp.speed
        st.free_ahead = dist if hit is not None else math.inf
        self._join_cap(st, pp, self._avoid_path or path, pose, p)
        if coasting:
            st.target_speed = min(st.target_speed, p["coast_speed"])
        md_note = self._md_fault(st, vs, p)
        if md_note is None:
            return st

        if self._obstacle_response(st, hit, dist, vs, p, path, pp.index):
            return st

        if not path.closed:
            return self._approach_stop(st, path, pose, vs, p)

        st.launch = self._launch_ok(st, vs, hit, coasting, md_note, p)
        st.reason = (f"{self._switch.active_key} {self.laps}周・速度 {st.target_speed:.2f} m/s・"
                     f"横偏差 {pp.cross_track * 100:+.0f}cm"
                     + ("" if self._joined else "・経路に乗るまで減速")
                     + self._avoid_note() + self._coast_note(coasting, p) + md_note)
        return st

    def _maybe_finish(self, pose, vs: VehicleState, p: dict[str, float]) -> None:
        """ミッションが終わったら停止点への経路を（ワーカーで）作り始める。"""
        if (self._stopping or self._graph is None or self._stop_job is not None
                or self._race_t < self._stop_retry_t):
            return
        m = Mission.from_dict(self._cfg.mission)
        if not m.due(self.laps, self._race_t):
            return
        stop = self._cfg.stops.get(m.then)
        if stop is None:
            return
        self._stopping, self._stop = m.then, stop
        self._stop_error = ""
        self._switch.pending, self._switch.pending_key = None, ""
        g = self.slam.grid
        self._stop_job = self._pool().submit(
            _compute_stop, g.trinary().copy(), g.resolution, tuple(g.origin), self._graph,
            self._stop_start(pose, vs), stop, float(vs.speed), dict(p), self.vehicle,
            list(self._cfg.avoid),
            frozenset(self._route_paths_edges(self._switch.active_key)) or None)

    def _stop_start(self, pose, vs: VehicleState) -> tuple[float, float, float]:
        """停止経路を引き始める姿勢: **今の位置**と、今の経路のその場所の向き。

        ★ 計算している間に進むぶん先の点から引かない。以前は今の向きのまま直線で外挿した
        点から引いていて、ヘアピンでは向かい側の道に乗り、そこへ吸着して「逆走になる」で
        失敗した。経路が届くころ車は始点より少し先に居るが、始点から前へ探す（`_poll`）ので
        追える。進むぶんは「近すぎる」の判定の側に足してある（`_compute_stop`）。
        """
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        path = self._switch.active
        if path is not None and path.closed and len(path) >= 3:
            n = len(path)
            j = nearest_index(path, x, y, self._hint, window=n // 4 if self._hint >= 0 else 0)
            a, b = path.xy[j], path.xy[(j + 1) % n]
            yaw = math.atan2(float(b[1] - a[1]), float(b[0] - a[0]))
        return x, y, yaw

    def _approach_stop(self, st: AutoState, path: rl_mod.RaceLine, pose, vs: VehicleState,
                       p: dict[str, float]) -> AutoState:
        rem = _remaining(path, pose, self._hint)
        v = max(0.0, vs.speed)
        # 今の速度で止まれる距離（遅延のあいだに進むぶん＋減速）
        brake_d = v * p["delay_s"] + v * v / (2.0 * max(p["a_brake"], 0.1))
        if rem <= brake_d + p["stop_tol"]:
            st.brake = True
            st.target_speed = 0.0
            if self._stop is not None and self._stop.mode == "park":
                if abs(vs.speed) < 0.03:
                    self._start_park()
                    st.reason = f"停止点『{self._stopping}』の手前で止まった。駐車に移る"
                    return st
                st.reason = f"駐車の手前で止まろうとしている（残り {rem:.2f}m）"
                return st
            if abs(vs.speed) < 0.03:
                self.phase = st.phase = STOPPED
            st.reason = f"停止点『{self._stopping}』で停止（残り {rem * 100:.0f}cm）"
            return st
        if rem <= p["creep_dist"]:
            st.target_speed = min(st.target_speed, p["creep_speed"])
        st.target_speed = max(st.target_speed, min(p["creep_speed"], 0.15))
        st.reason = f"停止点『{self._stopping}』へ向かっている（残り {rem:.2f}m）"
        return st

    def _start_park(self) -> None:
        """道の上で止まった。停止点の姿勢を車体基準に直して `ParkToPoint` へ渡す。"""
        assert self._stop is not None and self._stop.yaw is not None
        x, y, yaw = self.slam.pose
        dx, dy = self._stop.x - x, self._stop.y - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx, ly = c * dx - s * dy, s * dx + c * dy
        lyaw = (self._stop.yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
        self._park = ParkToPoint()
        self._park.request_park_target(lx, ly, lyaw)
        self.phase = PARK

    def _park_step(self, st: AutoState, scan: Scan, vs: VehicleState,
                   p: dict[str, float]) -> AutoState:
        assert self._park is not None
        pp = {s.key: s.default for s in ParkToPoint.params}
        pst = self._park.plan(scan, vs, pp, self._dt)
        pst.mode, pst.planner, pst.phase = self.id, self.name, PARK
        pst.pose_x, pst.pose_y, pst.pose_yaw = st.pose_x, st.pose_y, st.pose_yaw
        pst.match_score = st.match_score
        pst.laps = self.laps
        pst.reason = f"駐車『{self._stopping}』: {pst.reason}"
        return pst

    # ── GUI へ ──

    def _fill_route_state(self, st: AutoState) -> None:
        st.route_active = self._switch.active_key
        st.route_pending = self._switch.pending_key
        st.route_source = self._switch.source
        st.route_groups = sorted(self._routes)
        notes = [x for x in (self._switch_note, self._note, self._stop_error) if x]
        if AUTO_KEY in self._routes and self._auto_note:
            notes.append(self._auto_note)
        notes += [f"{k}: {v}" for k, v in self._route_err.items()]
        notes += [f"{k}: {v}" for k, v in self._route_warn.items()]
        st.route_note = "・".join(notes)
        st.mission = Mission.from_dict(self._cfg.mission).describe(self.laps, self._race_t)
        if self._stopping:
            st.mission = f"停止点『{self._stopping}』へ"

    def snapshot(self) -> AutoMap | None:
        m = super().snapshot()
        if m is None:
            return None
        # 親は変わっていなければ同じ `AutoMap` を返す。経路の版も同じなら詰め直さない
        # （`planning_node` は 10Hz で呼ぶ。グラフと全経路の tolist は毎回やると無駄）
        if m is self._snap_map and self._snap_ver == self._route_ver:
            return m
        # 親が詰め直したとき（`_map_ver`）と、こちらの経路の版が進んだときの両方で変わる値
        m.route_seq = self._map_ver + self._route_ver
        m.route_active = self._switch.active_key
        m.routes_json = self._cfg.to_json(with_traj=False)
        if self._graph is not None:
            xs: list[float] = []
            breaks: list[int] = []
            for e in self._graph.edges:
                pts = e.xy[:: max(1, len(e.xy) // 200)]
                if len(pts) and not np.array_equal(pts[-1], e.xy[-1]):
                    pts = np.vstack([pts, e.xy[-1:]])
                breaks.append(len(xs) // 2)
                xs.extend(np.round(pts, 3).reshape(-1).tolist())
            m.graph_xy, m.graph_breaks = xs, breaks
        m.routes = {k: np.round(v.xy, 3).reshape(-1).tolist() for k, v in self._routes.items()}
        if self._switch.active_key == _STOP_KEY and self._switch.active is not None:
            m.routes[_STOP_KEY] = np.round(self._switch.active.xy, 3).reshape(-1).tolist()
        self._snap_map, self._snap_ver = m, self._route_ver
        return m


def _start_of(traj) -> Waypoint | None:
    """地図作成の軌跡の出発点と向き（自動経路のスタート＝スタート/フィニッシュ）。

    向きは出発点から 0.5m 進んだ点への方向（出発直後の数cmは静止中のゆらぎで向きが暴れる）。
    """
    if traj is None or len(traj) < 2:
        return None
    xy = np.asarray(traj, dtype=np.float64)[:, :2]
    d = np.hypot(xy[:, 0] - xy[0, 0], xy[:, 1] - xy[0, 1])
    far = np.nonzero(d >= 0.5)[0]
    if not len(far):
        return None
    j = int(far[0])
    return Waypoint(float(xy[0, 0]), float(xy[0, 1]),
                    math.atan2(xy[j, 1] - xy[0, 1], xy[j, 0] - xy[0, 0]))


def _remaining(path: rl_mod.RaceLine, pose, hint: int = -1) -> float:
    """開いた経路の終点まで、今の位置から経路に沿った残り距離 [m]。

    `Pursuit.remaining` は予測位置・点の刻み単位なので、停止判定には粗い。
    最寄り点からの残りを点の刻みで数え、最寄り点との前後のずれで補正する。

    :param hint: 追っている添字（`follow` の結果）。与えるとその周りだけで最寄り点を探す。
        ★1周回ってから止まる経路は始めと終わりが同じ場所を通るので、全体から探すと
        走り出した直後に「残りわずか」と出て、その場で制動した
    """
    n = len(path)
    j = nearest_index(path, pose[0], pose[1], hint, window=n // 4 if hint >= 0 else 0)
    step = path.length / max(1, n - 1)
    jj = min(j, n - 2)
    t = path.xy[jj + 1] - path.xy[jj]
    t = t / max(float(np.hypot(*t)), 1e-9)
    along = float((pose[0] - path.xy[j, 0]) * t[0] + (pose[1] - path.xy[j, 1]) * t[1])
    return (n - 1 - j) * step - along

