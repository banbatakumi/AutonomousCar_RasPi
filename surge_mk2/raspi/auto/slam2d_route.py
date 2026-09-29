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

経由点を1つも置いていない地図では、地図作成の最後の1周から自動で経由点を
置いたグループAを作る（**分岐の無いコースでは `slam2d_raceline` と同じ経路になる**）。

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
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field

import numpy as np

from ..msgs.types import AutoMap, AutoState, Scan, VehicleState
from ..nav import centerline as cl_mod
from ..nav import obstacles as obs_mod
from ..nav import raceline as rl_mod
from ..nav.purepursuit import PursuitConfig, follow, nearest_index
from ..nav.roadgraph import RoadGraph, build_graph
from ..nav.route import (RouteError, RoutePath, Waypoint, build_raceline, plan_loop,
                         plan_to, tightest_radius, waypoints_from_traj)
from ..nav.route_switch import Mission, RouteSwitcher, lap_crossed
from . import mapstore
from ._slam2d_nav import occgrid_from_trinary
from .base import ParamSpec
from .park_to_point import ParkToPoint
from .route_config import GROUPS, RouteConfig, Stop
from .slam2d_raceline import DONE, MAX_TRACK_WIDTH, PATH_STEP, Slam2dRaceLine

__all__ = ["Slam2dRoute"]

#: 停止点に着いて止まっている / 駐車（`ParkToPoint`）に引き継いだ
STOPPED, PARK = "STOPPED", "PARK"
#: 停止経路のキー（`RouteSwitcher.active_key`）
_STOP_KEY = "stop"
#: 駐車の停止点は道から離れている（駐車枠の中）ので、吸着の半径を広く取る
_PARK_SNAP_M = 2.5


@dataclass
class _BuildOut:
    graph: RoadGraph
    routes: dict[str, rl_mod.RaceLine] = field(default_factory=dict)
    centerlines: dict[str, cl_mod.Centerline] = field(default_factory=dict)
    paths: dict[str, RoutePath] = field(default_factory=dict)
    errors: dict[str, str] = field(default_factory=dict)
    #: 経路はできたが気をつけること（曲がり切れない恐れのある急な所など）
    warnings: dict[str, str] = field(default_factory=dict)


def _kappa_max(vehicle) -> float:
    """車の曲がれる限界の曲率 [1/m]（最小旋回半径の逆数）。"""
    return math.tan(vehicle.max_steer) / vehicle.wheelbase


def _line_kwargs(p: dict[str, float], vehicle) -> dict:
    return dict(half_width=vehicle.half_width, margin=p["line_margin"], lam=p["line_lam"],
                passes=int(p["line_passes"]), v_max=p["v_max"], v_min=p["v_min"],
                a_lat=p["a_lat"], a_accel=p["a_accel"], a_brake=p["a_brake"],
                front_overhang=vehicle.front_overhang, rear_overhang=vehicle.rear_overhang,
                step=PATH_STEP, max_width=MAX_TRACK_WIDTH, kappa_max=_kappa_max(vehicle))


def _compute_routes(trinary: np.ndarray, resolution: float, origin: tuple[float, float],
                    seed, traj, groups: dict[str, list[Waypoint]], p: dict[str, float],
                    vehicle, graph: RoadGraph | None) -> _BuildOut:
    """ワーカースレッドで走る。**引数の写しだけを触る**（planner の状態を読まない）。"""
    grid = occgrid_from_trinary(trinary, resolution=resolution, origin=origin, seq=0)
    if graph is None:
        graph = build_graph(trinary, resolution=resolution, origin=origin,
                            keep=vehicle.half_width + p["graph_margin"], seed=seed, traj=traj)
    out = _BuildOut(graph=graph)
    kw = _line_kwargs(p, vehicle)
    for key, wps in groups.items():
        try:
            rp = plan_loop(graph, wps, turn_max_deg=p["turn_max_deg"])
            rl, cl = build_raceline(grid, graph, rp, **kw)
        except (RouteError, ValueError, np.linalg.LinAlgError) as e:
            out.errors[key] = str(e)
            continue
        except Exception as e:              # noqa: BLE001
            # 1つのグループの想定外の失敗で、他のグループまで作れなくしない
            out.errors[key] = f"内部エラー: {e!r}"
            continue
        out.routes[key], out.centerlines[key], out.paths[key] = rl, cl, rp
        r, i = tightest_radius(rl, PATH_STEP)
        r_min = 1.0 / _kappa_max(vehicle)
        if r < 1.1 * r_min:
            x, y = rl.xy[i]
            out.warnings[key] = (f"({x:.1f}, {y:.1f}) の半径{r * 100:.0f}cmは車の最小旋回半径"
                                 f"{r_min * 100:.0f}cmに近い（最低速度で通る。外れるなら経由点を見直す）")
    return out


def _compute_stop(trinary: np.ndarray, resolution: float, origin: tuple[float, float],
                  graph: RoadGraph, pose: tuple[float, float, float], stop: Stop,
                  v_now: float, p: dict[str, float], vehicle) -> rl_mod.RaceLine:
    grid = occgrid_from_trinary(trinary, resolution=resolution, origin=origin, seq=0)
    park = stop.mode == "park"
    goal = Waypoint(stop.x, stop.y, None if park else stop.yaw)
    # 停止点が近すぎる（今の速度から止まり切れない）なら1周回ってから止まる。
    # ★固定 1.2m だと、周回を数えた直後（2m/s）に 1.2m 先の停止点へ向かって減速し切れず、
    # 止まる位置が 18cm ずれた（toyota2 の sim.bench、2026-09-29）
    v = max(0.0, v_now)
    need = v * v / (2.0 * max(p["a_brake"], 0.1)) + p["creep_dist"] + 0.5
    rp = plan_to(graph, pose, goal, turn_max_deg=p["turn_max_deg"],
                 snap_radius=_PARK_SNAP_M if park else 0.6,
                 min_len=need, end_at_goal=not park,
                 extend=p["park_lead"] if park else 0.0)
    kw = _line_kwargs(p, vehicle)
    kw["v_min"] = p["creep_speed"]
    rl, _ = build_raceline(grid, graph, rp, v_start=v_now, v_end=p["creep_speed"], **kw)
    return rl


class Slam2dRoute(Slam2dRaceLine):
    id = "slam2d_route"
    name = "経路選択走行(slam2d)"
    description = ("slam2dの地図全体から道路グラフを作り、経由点のグループで分岐を選んで周回する。"
                   "走行中の切替・n周後の停止/駐車に対応")
    #: GUI が地図パネル・地図ライブラリ・経由点エディタを出す planner（`registry.catalog`）
    map_ui = True
    routes_ui = True

    params = Slam2dRaceLine.params + (
        ParamSpec(key="graph_margin", label="道路グラフの余裕", min=0.0, max=0.2,
                  default=0.02, step=0.01, unit="m",
                  note="車体半幅にこれを足した幅が取れない道はグラフから消える（通れない扱い）"),
        ParamSpec(key="turn_max_deg", label="分岐で曲がれる角度", min=45.0, max=150.0,
                  default=100.0, step=5.0, unit="°",
                  note="★節点でこれより急に向きを変える経路は選ばない（Y字の枝から枝への"
                       "折り返しを禁じる）。節点から1m先までの向きで測る"),
        ParamSpec(key="switch_tol", label="乗り換えを許す横ずれ", min=0.05, max=0.5,
                  default=0.15, step=0.01, unit="m",
                  note="経路の切替は、新しい経路からこれ以内に居る周期まで待つ"),
        ParamSpec(key="creep_speed", label="停止前の這う速度", min=0.1, max=0.6,
                  default=0.25, step=0.05, unit="m/s", note="停止点の手前ではこれで近づく"),
        ParamSpec(key="creep_dist", label="這い始める距離", min=0.2, max=2.0,
                  default=0.6, step=0.05, unit="m", note="停止点のこれだけ手前から這う"),
        ParamSpec(key="park_lead", label="駐車の手前で止まる位置", min=0.0, max=2.0,
                  default=1.0, step=0.1, unit="m",
                  note="駐車の停止点では、道の上で枠に最も近い点からこれだけ先で止まって"
                       "park_to_pointへ渡す（バック駐車は枠を通り過ぎてから後退で入れる）。"
                       "0だと枠の真横で止まり、横へずらす切り返しを繰り返した（sim.benchで実測）"),
        ParamSpec(key="stop_tol", label="停止の余裕", min=0.0, max=0.3,
                  default=0.02, step=0.01, unit="m",
                  note="残り距離が「止まれる距離＋これ」を切ったら制動する"),
    )

    def __init__(self) -> None:
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="route")
        super().__init__()

    # ── 状態 ──

    def reset(self) -> None:
        super().reset()
        self._cfg = RouteConfig()
        self._graph: RoadGraph | None = None
        self._routes: dict[str, rl_mod.RaceLine] = {}
        self._route_cls: dict[str, cl_mod.Centerline] = {}
        self._route_err: dict[str, str] = {}
        self._route_warn: dict[str, str] = {}
        self._job: Future | None = None
        self._job_again = False
        self._stop_job: Future | None = None
        self._stop_error = ""
        self._switch = RouteSwitcher()
        self._switch_note = ""
        self._map_name = ""
        self._route_ver = getattr(self, "_route_ver", 0) + 1   # 版は戻さない（配信の判定に使う）
        self._race_t = 0.0
        self._prev_idx = -1
        #: 前回数えてから経路に沿って進んだ距離 [m]（周回の数え間違い防止）
        self._lap_s = 0.0
        self._stopping = ""                  #: 向かっている停止点の名前
        self._stop: Stop | None = None
        self._park: ParkToPoint | None = None
        self._note = ""                      #: 設定エラー等（GUI の reason に添える）
        self._p: dict[str, float] | None = None
        self._want_group = ""

    def close(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)

    def plan(self, scan: Scan, vs: VehicleState | None,
             p: dict[str, float], dt: float) -> AutoState:
        self._p = p
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
        self._graph = None
        self._job = None                     # 前の地図の計算は結果を捨てる
        self._job_again = False
        self._stop_job = None
        self._stopping, self._stop, self._park = "", None, None
        self._want_group = ""
        self._switch_note = ""
        self._note = ""
        try:
            self._cfg = RouteConfig.from_json(mapstore.load_routes(name))
        except ValueError as e:
            self._cfg = RouteConfig()
            self._note = f"経路の設定を読めない（{e}）。既定の経路で走る"
        if not self._cfg.groups and self.path is not None and len(self.path) >= 20:
            # 経由点の無い地図（旧形式）: 保存済みレーシングラインを1周として経由点を置く
            self._cfg.groups = {"A": waypoints_from_traj(self.path.xy)}
            self._cfg.active = "A"
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
        if self._map_name:
            mapstore.save_routes(self._map_name, cfg.to_json())
        if rebuild and self._graph is not None:
            self._submit(graph=self._graph)
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
        group = self._cfg.signal_map.get(value) or (value if value in GROUPS else "")
        if not group:
            self._switch_note = f"信号『{value}』に対応するグループが無い"
            return
        self.request_route(group, f"{source}:{value}")

    # ── ワーカー ──

    def _submit(self, *, graph: RoadGraph | None) -> None:
        if self._job is not None:
            self._job_again = True          # 走っている計算が終わってからやり直す
            return
        p = self._p or {s.key: s.default for s in self.params}
        g = self.slam.grid
        seed = None
        traj = self._cfg.explore_traj
        if traj is not None and len(traj):
            seed = (float(traj[0, 0]), float(traj[0, 1]))
        self._job = self._pool.submit(
            _compute_routes, g.trinary().copy(), g.resolution, tuple(g.origin), seed,
            None if traj is None else traj.copy(), dict(self._cfg.groups), dict(p),
            self.vehicle, graph)

    def _poll(self) -> bool:
        """計算が終わっていれば取り込む。**まだ走っていれば False。**"""
        if self._job is not None:
            if not self._job.done():
                return False
            job, self._job = self._job, None
            try:
                out: _BuildOut = job.result()
            except Exception as e:          # noqa: BLE001
                self._build_error = f"道路グラフを作れなかった: {e}"
                return True
            self._install(out)
            if self._job_again:
                self._job_again = False
                self._submit(graph=self._graph)
        if self._stop_job is not None and self._stop_job.done():
            job, self._stop_job = self._stop_job, None
            try:
                line = job.result()
            except Exception as e:          # noqa: BLE001
                self._stop_error = f"停止点への経路を作れなかった: {e}"
                self._stopping = ""
            else:
                self._switch.set_active(_STOP_KEY, line)
                self._hint = -1
                self._prev_idx = -1
        return True

    def _install(self, out: _BuildOut) -> None:
        self._graph = out.graph
        self._routes, self._route_cls, self._route_err = out.routes, out.centerlines, out.errors
        self._route_warn = out.warnings
        self._route_ver += 1
        self._map_dirty = True
        key = self._switch.active_key
        if key in self._routes and self._switch.active is not None:
            # 経由点の編集で今の経路が作り直された。乗れる場所で新しい版へ移る
            self._switch.request(key, self._routes[key], "編集")
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
            self._cfg.explore_traj = RouteConfig.decimate(self.slam.trajectory_array())
            if not self._cfg.groups:
                self._cfg.groups = {"A": waypoints_from_traj(lap)}
                self._cfg.active = "A"
            self._submit(graph=None)
        if not self._poll():
            st.reason = "経路を作っている（道路グラフ → 経路 → レーシングライン）"
            return st
        if self._build_error:
            st.reason = self._build_error
            return st
        key = self._cfg.active if self._cfg.active in self._routes else next(iter(self._routes), "")
        if not key:
            errs = "・".join(f"{k}: {v}" for k, v in self._route_err.items())
            self._build_error = f"経路を作れなかった（{errs or '経由点が無い'}）"
            st.reason = self._build_error
            return st
        self.path = self._routes[key]
        self.centerline = self._route_cls[key]
        self.phase = st.phase = DONE
        self._hint = -1
        self._map_dirty = True
        self._saved_map_name = self._auto_save_map()
        if self._saved_map_name:
            self._map_name = self._saved_map_name
            mapstore.save_routes(self._saved_map_name, self._cfg.to_json())
        g = self._graph
        st.reason = (f"経路ができた（道路グラフ: 分岐 {sum(1 for n in g.nodes if len(n.edges) >= 3)}・"
                     f"区間 {len(g.edges)}、グループ {'/'.join(self._routes)}）。"
                     + (f"地図を『{self._saved_map_name}』として保存した。"
                        "経由点を編集するか、走行で開始してください"
                        if self._saved_map_name else self._save_error))
        return st

    # ── RACE / STOPPED / PARK ──

    def _race(self, st: AutoState, scan: Scan, vs: VehicleState,
              p: dict[str, float], lost: bool) -> AutoState:
        self._poll()
        if self.phase == PARK:
            return self._park_step(st, scan, vs, p)
        if self.phase == STOPPED:
            st.brake = True
            st.ready = True
            st.target_speed = 0.0
            st.reason = f"停止点『{self._stopping}』で停止した"
            return st

        if self._switch.active is None:
            if not self._routes:
                st.reason = ("経路を準備中" if self._job is not None else
                             f"経路が無い（{self._build_error or self._note or '経由点を置いてください'}）")
                return st
            key = self._cfg.active if self._cfg.active in self._routes else next(iter(self._routes))
            self._switch.set_active(key, self._routes[key])
            self._hint = -1
            self._prev_idx = -1
            self._lap_s = 0.0
            self._joined = False
            self._race_t = 0.0
            self.laps = 0

        if lost or st.match_score < p["min_score"]:
            st.reason = (f"自己位置が信用できない（一致度 {st.match_score:.2f} < "
                         f"{p['min_score']:.2f}）")
            return st

        pose = self.slam.pose
        sw = self._switch.step(tuple(pose), vs.speed, tol=p["switch_tol"], a_brake=p["a_brake"])
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

        cfg = PursuitConfig(
            wheelbase=self.vehicle.wheelbase, max_steer=self.vehicle.max_steer,
            lookahead_k=p["look_k"], lookahead_min=p["look_min"], delay_s=p["delay_s"])
        pp = follow(path, pose, vs.speed, vs.steer_actual, cfg, hint=self._hint)
        self._hint = pp.index
        st.target_steer = pp.steer
        st.heading = pp.steer
        st.cross_track = pp.cross_track
        st.target_x, st.target_y = pp.target

        if path.closed:
            n = len(path)
            if self._prev_idx >= 0:
                self._lap_s += ((pp.index - self._prev_idx) % n) * path.length / n
            # 添字が末尾→先頭へ回っても、半周も進んでいなければ数えない（走り出しの
            # 位置が経路の終端の直前だと、動いた瞬間に「1周」と数えてしまう）
            if lap_crossed(self._prev_idx, pp.index, n) and self._lap_s > 0.5 * path.length:
                self.laps += 1
                self._lap_s = 0.0
            self._prev_idx = pp.index
            self._maybe_finish(pose, vs, p)

        if self._off_route(st, pp, vs, p):
            return st

        self._obs_pad = p["obstacle_pad"]
        self._update_obstacles(scan)
        st.obstacles = [v for o in self._obs for v in (o.x, o.y, o.r)]
        n = len(path)
        hit, dist = obs_mod.blocking(
            self._obs, path.xy, pp.index,
            half_width=self.vehicle.half_width + p["line_margin"],
            ahead_m=p["obstacle_stop"] + self.vehicle.front_overhang,
            step=path.length / (n if path.closed else max(1, n - 1)),
            skip_m=self.vehicle.front_overhang, closed=path.closed)

        st.ready = True
        st.target_speed = pp.speed
        st.free_ahead = dist if hit is not None else math.inf
        self._join_cap(st, pp, path, pose, p)

        stop_at = p["obstacle_stop"]
        if (stop_at > 0.0 and hit is not None
                and dist - self.vehicle.front_overhang <= stop_at):
            st.brake = True
            st.target_speed = 0.0
            st.reason = f"前方 {max(0.0, dist - self.vehicle.front_overhang):.1f}m に障害物。停止"
            return st

        if not path.closed:
            return self._approach_stop(st, path, pose, vs, p)

        st.reason = (f"{self._switch.active_key} {self.laps}周・速度 {st.target_speed:.2f} m/s・"
                     f"横偏差 {pp.cross_track * 100:+.0f}cm"
                     + ("" if self._joined else "・経路に乗るまで減速"))
        return st

    def _maybe_finish(self, pose, vs: VehicleState, p: dict[str, float]) -> None:
        """ミッションが終わったら停止点への経路を（ワーカーで）作り始める。"""
        if self._stopping or self._graph is None:
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
        # 計算している間に進む距離ぶん先の姿勢から引く（計算は数百msかかる）
        lead = max(0.0, vs.speed) * 0.4
        start = (pose[0] + lead * math.cos(pose[2]), pose[1] + lead * math.sin(pose[2]), pose[2])
        g = self.slam.grid
        self._stop_job = self._pool.submit(
            _compute_stop, g.trinary().copy(), g.resolution, tuple(g.origin), self._graph,
            start, stop, float(vs.speed), dict(p), self.vehicle)

    def _approach_stop(self, st: AutoState, path: rl_mod.RaceLine, pose, vs: VehicleState,
                       p: dict[str, float]) -> AutoState:
        rem = _remaining(path, pose)
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
        m.route_seq = self._route_ver
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
        return m


def _remaining(path: rl_mod.RaceLine, pose) -> float:
    """開いた経路の終点まで、今の位置から経路に沿った残り距離 [m]。

    `Pursuit.remaining` は予測位置・点の刻み単位なので、停止判定には粗い。
    最寄り点からの残りを点の刻みで数え、最寄り点との前後のずれで補正する。
    """
    n = len(path)
    j = nearest_index(path, pose[0], pose[1])
    step = path.length / max(1, n - 1)
    jj = min(j, n - 2)
    t = path.xy[jj + 1] - path.xy[jj]
    t = t / max(float(np.hypot(*t)), 1e-9)
    along = float((pose[0] - path.xy[j, 0]) * t[0] + (pose[1] - path.xy[j, 1]) * t[1])
    return (n - 1 - j) * step - along

