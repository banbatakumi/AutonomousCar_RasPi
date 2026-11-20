"""planner をシムの真値と突き合わせて評価する。**バスも GUI もプロセスも使わない。**

    .venv/bin/python -m sim.bench --course normal
    .venv/bin/python -m sim.bench --course course1 --mode slam2d_raceline --time 240
    .venv/bin/python -m sim.bench --mode ftg --set explore_speed=0.6
    .venv/bin/python -m sim.bench --course toyota2 --mode slam2d_route \
        --routes sim/routes/toyota2.json --switch 200:B
    .venv/bin/python -m sim.bench --course toyota2 --map toyota2 --mode slam2d_route \
        --time 120 --trace /tmp/t.csv          # 保存済み地図から走る（地図作成を飛ばす）

## ラップタイム（真値のゲート）

RACE で最初に走り出した真値の位置に、進行方向と直交するゲート線を置き、それを
前向きに跨いだ時刻（100Hz で補間）でラップを計る。**プランナーの周回カウンタには
頼らない**（planner の数え方は経路の最寄り点の添字で、時刻の分解能が 0.1s しか無い）。1周目はスタンディング
スタートを含むので、比較には2周目以降（フライング）を使う。

`io_node` と同じことを 1 プロセスの中でやる:

    SimLink.poll() → フレーム → ScanAssembler / StateBuilder → planner.plan()
                  → command_from_cmd() → SimLink.send()

**実機と同じ変換コードを通す**（`BusBridge`＝`ScanAssembler` の鏡像反転・`StateBuilder`、
`LinkTracker`＋`TimeSync` による STM32 時刻→Pi 時刻の換算、`command_from_cmd` のクランプ）ので、
ここで走れば `sim.run` でも走る。PING を 5Hz で送って時刻同期を育てる（収束するまでは実機と同じく
受信時刻で代用する）。

`--map` の走行は、地図を**真のコースへ重ねて**（`slam_bench.align_map`）自己位置の誤差と
経路からの真の横偏差を出す（planner の自己申告ではない）。

## なぜ別に用意するのか

`sim.run` は 4 プロセスを立ち上げてブラウザで engage する。**自己位置の誤差が
何 cm かを知りたいだけのときに重すぎる。** しかもシムの真値は UDP の私設
チャンネルにしか出ないので、GUI を開かないと見られない。

ここは真値を直に読めるので、**SLAM の推定と真値を毎周期突き合わせられる**。
実機では絶対に手に入らない数字なので、これが座標系と精度を確定する場所になる。

## `--allow-arm` に相当するものは無い

シム専用の入口で、動かす対象が numpy の中の車だけなので ARM を人間に握らせる
意味が無い（`sim/run.py` が `--allow-arm` を既定で付けているのと同じ理由）。
**このファイルを実機のコードから import してはいけない。**

## 限界

- **壁時計で回る**: 計画・指令の周期は `time.monotonic_ns()` で刻むので、ホストの負荷で結果が
  変わる（決定論的でない）。`advance_to` は 1 回 0.25s で頭打ちなので、それ以上ホストが止まると
  シムの時間が遅れる。指令の間隔が仮想 STM32 の途絶（100ms）を超えた走行は「捨てる走行」の
  印が付く（`Result.report`）。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from raspi.auto.registry import PLANNERS, make_planner, merged_params  # noqa: E402
from raspi.core.bus_bridge import BusBridge  # noqa: E402
from raspi.core.link_tracker import LinkTracker  # noqa: E402
from raspi.msgs import DriveCmd, command_from_cmd  # noqa: E402
from raspi.msgs.types import TOPIC_SCAN, TOPIC_VEHICLE_STATE  # noqa: E402
from raspi.nav.body_mask import body_ranges, mask_body  # noqa: E402
from raspi.proto.generated import packets  # noqa: E402

from .bench_events import BenchEvents  # noqa: E402
from .bench_faults import (  # noqa: E402
    KidnapRunner,
    blank_scan,
    carve_rects,
    parse_kidnap,
    parse_place,
    parse_rect,
)
from .bench_record import TraceWriter  # noqa: E402
from .bench_report import Result  # noqa: E402
from .course import list_courses  # noqa: E402
from .link import create_sim_link  # noqa: E402
from .moving import update_all  # noqa: E402
from .params import REAL_VIBRATION  # noqa: E402

__all__ = ["Bench", "BenchOptions", "Result"]

NS = 1_000_000_000
CMD_HZ = 100
#: PING の間隔（`io_node` の `PING_HZ`）
PING_PERIOD_NS = NS // 5


class _Capture:
    """`BusBridge` の出力（バスの代わり）。最後に出た車両状態と、組み上がった点群を受ける。"""

    def __init__(self) -> None:
        self.vs = None
        self.scan = None

    def send(self, topic, obj) -> None:
        if topic == TOPIC_VEHICLE_STATE:
            self.vs = obj
        elif topic == TOPIC_SCAN:
            self.scan = obj


@dataclass
class BenchOptions:
    """`Bench` の走らせ方（コース・モード・planner のパラメータ以外の全部）。

    `main` の引数が 38 個あり、`Bench` へ並べて渡していたのを 1 つにまとめた。
    各欄の意味は `main` の `--` 引数の help と `Bench.__init__` の注釈を見る。
    """

    max_speed: float = 3.0
    max_steer: float = 0.524
    model: str | None = None
    quiet: bool = False
    seed: int = 0
    routes: dict | None = None
    events: list[tuple[float, str, str]] | None = None
    auto_race: bool = True
    map_name: str | None = None
    trace: Path | None = None
    locate_hint: tuple[float, float] | None = None
    obstacles: list[tuple[float, float, float]] | None = None
    movers: list[str] | None = None
    vehicle: dict[str, float] | None = None
    control: dict[str, float] | None = None
    surface: dict | None = None
    assume: dict[str, float] | None = None
    grip_file: Path | None = None
    grip_cfg: dict[str, float] | None = None
    sim_params: dict[str, float] | None = None
    state_dir: Path | None = None
    hijacks: list[tuple[float, float, float, float]] | None = None
    plan_slowdown: float = 1.0
    #: `--kidnap`: (走行開始からの秒, DX, DY, DYAW[deg], 持ち上げ[s])
    kidnaps: list[tuple] | None = None
    #: `--wall-gap`: 世界の壁を消す矩形 (x0, y0, x1, y1)
    wall_gaps: list[tuple[float, float, float, float]] | None = None


class Bench:
    def __init__(self, course: str | Path, mode: str, params: dict[str, float],
                 opts: BenchOptions | None = None) -> None:
        """:param model: `reload_if_changed(name)`を持つplanner（今のところ`e2e_lidar`
            だけ）へ渡すモデル名。`raspi/nodes/planning_node.py`の`_apply_e2e_model()`
            と同じダックタイピング（`E2ELidar`をここでimportして特別扱いしない）。
            **これが無いと`--mode e2e_lidar`はモデル未ロードのまま何も動かない**
            （GUI/`e2e/model`トピック経由でしかモデルを選べないので、
            `sim.bench`単体で検証するために必要）
        """
        o = opts or BenchOptions()
        self._course = course
        self._setup_maps(o.map_name, o.grip_file, o.state_dir)
        self._setup_sim(o)
        self._setup_planner(mode, o)
        self._init_state(mode, params, o)

    def _setup_maps(self, map_name: str | None, grip_file: Path | None,
                    state_dir: Path | None) -> None:
        # 地図を作る planner は BUILD の終わりに地図を自動保存する。**実機の
        # `saved_maps/` を bench の地図で埋めない**よう、保存先を一時ディレクトリへ
        import tempfile

        import shutil

        from raspi.auto import mapstore
        self._real_maps_dir = mapstore.MAPS_DIR
        self._tmp_maps_dir = mapstore.MAPS_DIR = Path(tempfile.mkdtemp(prefix="bench_maps_"))
        #: `--map`: 保存済み地図の**写し**を一時ディレクトリへ置いて読む（経路の設定を
        #: 保存し直しても実機の `saved_maps/` を書き換えない）
        self._map_name = map_name
        if map_name:
            src = self._real_maps_dir / f"{map_name}.npz"
            if not src.is_file():
                raise SystemExit(f"地図が無い: {src}")
            for suf in (".npz", ".json", ".routes.json"):
                f = self._real_maps_dir / f"{map_name}{suf}"
                if f.is_file():
                    shutil.copy(f, mapstore.MAPS_DIR / f.name)
            # 学習した横グリップは、実機の `saved_maps/` のものは使わず `--grip` で渡したものだけ読む
            # （bench の路面は実機と別物）
            if grip_file is not None:
                shutil.copy(grip_file, mapstore.MAPS_DIR / f"{map_name}.grip.json")
            # `--state`: 前の bench が残した地図の学習値・仕上げた経路の写しを置く（2 回目の走行の再現）
            self._state_dir = state_dir
            if state_dir is not None:
                for suf in (".grip.json", ".lines.pkl"):
                    f = state_dir / f"{map_name}{suf}"
                    if f.is_file():
                        shutil.copy(f, mapstore.MAPS_DIR / f.name)

    def _setup_sim(self, o: BenchOptions) -> None:
        """真値の側（シムの車・路面・仮想 STM32 の設定・障害物・他車）を作る。"""
        vehicle, sim_params, surface = o.vehicle, o.sim_params, o.surface
        # `--vehicle`: **真値の車だけ**諸元を変える（planner は `config/vehicle.toml` のまま）。
        # 同定値が外れていたとき（グリップが低い・舵が届かない等）に走りが壊れないかを見る
        spec = None
        if vehicle:
            from dataclasses import fields, replace

            from .vehicle import VehicleSpec
            known = {f.name for f in fields(VehicleSpec)}
            bad = sorted(set(vehicle) - known)
            if bad:
                raise SystemExit(f"--vehicle: 知らない諸元 {', '.join(bad)}")
            spec = replace(VehicleSpec.load(), **vehicle)
        params_sim = None
        if sim_params:
            from dataclasses import fields as _fields, replace as _replace

            from .params import SimParams
            bad = sorted(set(sim_params) - {f.name for f in _fields(SimParams)})
            if bad:
                raise SystemExit(f"--sim: 知らない項目 {', '.join(bad)}")
            params_sim = _replace(SimParams.load(), **sim_params)
        self.link = create_sim_link(self._course, seed=o.seed, with_channel=False, spec=spec,
                                    params=params_sim)
        # `--mu`・`--surface`・`--mu-at`: 真値の路面を場所と時刻で変える（`sim/surface.py`）。
        # コースの床（JSON の `"surface"`）が土台で、`--mu` はそれを置き換える
        self.surface = self.link.sim.surface
        if surface:
            from .surface import Surface
            sf = self.surface = self.link.sim.surface or Surface(self.link.sim.spec.mu)
            if surface.get("base"):
                sf.set_base(*surface["base"])
            for d in surface.get("discs", []):
                sf.add_disc(*d)
            for c in surface.get("changes", []):
                sf.add_change(*c)
            self.link.sim.surface = sf
        # 実機では io_node が `[control]` を CONFIG_SET で STM32 へ入れる。bench は io_node を
        # 通さないので、車速PIの加速度フィードフォワード（★v0.21）だけ直に入れる
        from raspi.core.control_params import load_control_params
        ctl = {**load_control_params(), **(o.control or {})}
        self.sim = self.link.sim
        # 車に効かないもの（TC・TV の係数など）も値を持たせる。launch_exit_torque_nm・速度 FF は車へ、
        # TC・ABS の有効/無効は旗の出し方に効く
        from raspi.core.control_params import OVERRIDABLE
        self.sim.vehicle.set_speed_ff(0.0, 0.0)
        for key, val in ctl.items():
            pid = OVERRIDABLE.get(key)
            if pid is not None:
                self.sim.apply_config(pid, float(val))
        if o.obstacles:
            # 障害物はシムの世界（LiDAR と衝突判定が見る格子）にだけ刻む。`--map` と
            # 併用すると凍結地図には無い物が経路の上に居る状況になる
            from .track import stamp_discs
            c = self.sim.course
            stamp_discs(c.grid, c.origin, c.resolution, list(o.obstacles))
            c._padded_grid = None
        if o.wall_gaps:
            # 壁の欠け: シムの世界からだけ消す（`--map` の凍結地図には残る）
            c = self.sim.course
            n = carve_rects(c.grid, c.origin, c.resolution, list(o.wall_gaps))
            c._padded_grid = None
            self._wall_gap_cells = n
        self._movers = []
        self._mv_col = 0
        if o.movers:
            from .moving import parse_mover
            self._movers = [parse_mover(m) for m in o.movers]
            self.sim.course._padded_grid = None

    def _setup_planner(self, mode: str, o: BenchOptions) -> None:
        """planner と、planning_node と同じ前処理（車体の内側の点・点群のノイズ）を作る。"""
        model, assume, grip_cfg = o.model, o.assume, o.grip_cfg
        self.planner = make_planner(mode)
        # planning_node と同じく、車体の内側の点は planner に渡さない（`raspi/nav/body_mask.py`）
        from raspi.core.vehicle import Vehicle
        _veh = Vehicle.load()
        self._body = body_ranges(_veh.footprint, _veh.lidar_x, _veh.lidar_y)
        # 点群のノイズの除去も planning_node と同じ（`raspi/nav/scan_filter.py`）
        from raspi.nav.scan_filter import ScanFilter, config_from_items
        self._scan_filter = ScanFilter(config_from_items(_veh.lidar_filter),
                                       lidar_x=_veh.lidar_x, lidar_y=_veh.lidar_y)
        self.scan_dropped = 0
        if self.planner is None:
            raise SystemExit(f"知らないモード: {mode!r}（候補 {', '.join(PLANNERS)}）")
        if model:
            reload_fn = getattr(self.planner, "reload_if_changed", None)
            if reload_fn is None:
                raise SystemExit(f"{mode!r} はモデル選択に対応していない"
                                 f"（reload_if_changedを持たない）")
            reload_fn(model)
        if assume:
            # `--assume`: **planner が信じている車**の諸元だけ変える（真値の車はそのまま）。
            # 「真値を知っている固定設定」の基準を作るのに使う
            from dataclasses import replace as _rep
            try:
                self.planner.vehicle = _rep(self.planner.vehicle, **assume)
            except TypeError as e:
                raise SystemExit(f"--assume: {e}")
        learner = getattr(self.planner, "_grip", None)
        if grip_cfg and learner is not None:
            for k, val in grip_cfg.items():
                if not hasattr(learner.cfg, k):
                    raise SystemExit(f"--grip-set: 知らない項目 {k}")
                setattr(learner.cfg, k, type(getattr(learner.cfg, k))(val))

    def _init_state(self, mode: str, params: dict[str, float], o: BenchOptions) -> None:
        """走行中に数える状態・受信経路・真値の起点・要求・ゲート・記録を用意する。"""
        self._grip_n = 0
        self._grip_open: list[dict] = []
        self._grip_slip: list[tuple[float, float]] = []    # 直近の (時刻, 真値の超過率)
        self._grip_lv = -1.0
        self._long_n = 0
        self._last_cmd_ns = 0
        self._grip_t0: float | None = None
        self._race_t0: float | None = None
        #: 走行開始から `(t, dur, steer, speed)` の間、指令を乗っ取る（人が押した・ぶつかって弾かれた代わり。
        #: 経路から外れた・壁に当たった所から戻れるかを見る）
        self._hijacks = sorted(o.hijacks or [])
        #: `--kidnap`（`sim/bench_faults.py`）
        self._kidnap = KidnapRunner(list(o.kidnaps or [])) if o.kidnaps else None
        self._wrong_xy: tuple[float, float] | None = None
        self.mode = mode
        self.params = merged_params(mode, params, hidden=True)
        self.max_speed = o.max_speed
        #: `plan()` を何倍遅く見せるか（Pi の計算の遅さ。`--plan-slowdown`）
        self.plan_slowdown = max(1.0, float(o.plan_slowdown))
        self.max_steer = o.max_steer
        self.quiet = o.quiet

        # `io_node` と同じ受信経路（時刻同期つき）。バスの代わりに `_Capture` が結果を受ける
        self._cap = _Capture()
        self.bridge = BusBridge(self._cap, clock=time.monotonic_ns)
        self.tracker = LinkTracker(on_telemetry=self.bridge.on_telemetry,
                                   on_frame=self.bridge.on_frame,
                                   on_time_reset=self._on_time_reset)
        self.bridge.to_pi_ns = self.tracker.sync.to_pi_ns
        self._ping_seq = 0
        self._next_ping_ns = 0
        self.vs = None
        self.res = Result()

        # 真値の起点。**SLAM の原点は「最初の姿勢」**なので、絶対座標ではなく
        # 起点からの移動量どうしを比べる（`test_nav.py` と同じ考え方）
        v = self.sim.vehicle
        self._origin = (v.x, v.y, v.yaw)
        #: `--map` の地図の原点の世界姿勢（RACE に入ってから `_align_map` が決める）
        self._map_origin: tuple[float, float, float] | None = None
        self._last_lap_t = 0.0
        self._prev_laps = 0
        self._prev_phase = ""

        self._truth_log: list[tuple[str, int, str, float, float]] = []
        routes = o.routes
        self._routes_world = routes
        if routes is not None:
            if getattr(self.planner, "request_routes", None) is None:
                raise SystemExit(f"{mode!r} は経路の設定（--routes）に対応していない")
            if not o.map_name:
                self.planner.request_routes(json.dumps(self._to_map_frame(routes)))
        #: 人の代わりに出す要求（`--map` の読み込み・DONE での「走行」・`--switch`/`--signal`）
        self._events = BenchEvents(self.planner, map_name=o.map_name, routes_world=routes,
                                   to_map_frame=self._to_map_frame, locate_hint=o.locate_hint,
                                   auto_race=o.auto_race, events=o.events,
                                   notes=self.res.notes, quiet=self.quiet)

        # 真値のゲート（モジュール docstring）
        self._gate: tuple[float, float, float, float] | None = None
        self._gate_prev: tuple[float, float, float] | None = None   # (t, 沿う距離, 横)
        self._gate_last_t: float | None = None
        self._gate_odom = 0.0
        self._race_col0: int | None = None
        #: `--trace` の CSV（`sim/bench_record.py`）
        self._trace = TraceWriter(o.trace, self.planner, self.sim) if o.trace else None
        # 車体外形の輪郭と、真の壁からの距離場（`_clearance`）
        from scipy.ndimage import distance_transform_edt

        from .course import _outline
        c = self.sim.course
        # `distance_transform_edt` はセル中心どうしの距離なので、壁の面までは約半セル短い。
        # 半セル引く（負にはしない）
        self._edt = np.maximum(
            distance_transform_edt(~c.grid.astype(bool)) * c.resolution - c.resolution / 2, 0.0)
        self._outline = _outline(np.asarray(self.sim.spec.footprint, dtype=np.float64),
                                 c.resolution / 2)
        self._lap_clear = math.inf
        self._prev_avoid = None
        self._prev_detour = False
        self._prev_reason_head = ""
        #: 障害物まわりの判断の移り変わり（`--obs-log` で出す）
        self._obs_log: list[str] = []
        self._feed_vs = getattr(self.planner, "on_vehicle_state", None)
        self._tick_fn = getattr(self.planner, "tick", None)

    def _on_time_reset(self, rx_ns: int) -> None:
        self.bridge.state_builder.reset()
        self.bridge.scans.reset()

    def _send_ping(self, now_ns: int) -> None:
        self._ping_seq = (self._ping_seq + 1) & 0xFFFFFFFF
        t1 = self.link.send(packets.Ping(ping_id=self._ping_seq))
        self.tracker.note_ping_sent(self._ping_seq, t1)
        self._next_ping_ns = now_ns + PING_PERIOD_NS

    # ── 座標 ──

    def _to_map(self, x: float, y: float, yaw: float | None = None, origin=None):
        """シムの世界座標 → SLAM の map フレーム（起点が原点、起点の向きが +x）。
        `origin` は map の原点の世界姿勢（既定は bench の起点。`--map` の地図は別）。"""
        ox, oy, oyaw = origin or self._origin
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        dx, dy = x - ox, y - oy
        mx, my = dx * c - dy * s, dx * s + dy * c
        if yaw is None:
            return mx, my, None
        return mx, my, (yaw - oyaw + math.pi) % (2 * math.pi) - math.pi

    def _to_map_frame(self, cfg: dict) -> dict:
        """`--routes` の JSON（シムの世界座標で書く）を map フレームへ直す。"""
        out = json.loads(json.dumps(cfg))
        for k, pts in (out.get("groups") or {}).items():
            conv = []
            for p in pts:
                mx, my, myaw = self._to_map(p[0], p[1], p[2] if len(p) > 2 else None)
                conv.append([mx, my] + ([] if myaw is None else [myaw]))
            out["groups"][k] = conv
        out["avoid"] = [list(self._to_map(q[0], q[1])[:2]) for q in (out.get("avoid") or [])]
        # 避ける範囲は map フレームでも軸に平行な矩形なので、回した4隅を囲む矩形にする
        rects = []
        for x0, y0, x1, y1 in out.get("avoid_rects") or []:
            cs = [self._to_map(x, y)[:2] for x, y in ((x0, y0), (x1, y0), (x0, y1), (x1, y1))]
            xs, ys = [c[0] for c in cs], [c[1] for c in cs]
            rects.append([min(xs), min(ys), max(xs), max(ys)])
        out["avoid_rects"] = rects
        for name, st in (out.get("stops") or {}).items():
            mx, my, myaw = self._to_map(st["x"], st["y"], st.get("yaw"))
            st["x"], st["y"], st["yaw"] = mx, my, myaw
        return out

    # ── 真値 ──

    def _truth_rel(self) -> tuple[float, float, float]:
        """map フレームでの真の姿勢。`--map` なら、地図を真のコースへ重ねた原点（`_align_map`）基準。"""
        v = self.sim.vehicle
        ox, oy, oyaw = self._map_origin or self._origin
        dx, dy = v.x - ox, v.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        return (dx * c - dy * s, dx * s + dy * c,
                (v.yaw - oyaw + math.pi) % (2 * math.pi) - math.pi)

    # ── 走行 ──

    def run(self, duration_s: float) -> Result:
        t0 = time.monotonic()
        self._next_cmd_ns = time.monotonic_ns()
        self._cmd = DriveCmd(mode=0)
        self._last_plan_ns = 0
        self._next_tick_ns = self._next_cmd_ns
        self._rearm = False
        # 仮想 STM32 は起動直後、ARM を一度外すまで ARM を受け付けない。最初の指令を ARM にすると
        # 断られて入れ直しになる（実機では人が ARM を入れる前に DISARM が流れている）ので、先に DISARM を送る
        self.link.send(command_from_cmd(DriveCmd(mode=0), allow_arm=True,
                                        max_speed=self.max_speed, max_steer=self.max_steer))
        while time.monotonic() - t0 < duration_s:
            t_poll = time.perf_counter()
            frames = self.link.poll()
            if self._movers:
                self._step_movers(t0)
            if self._kidnap is not None:
                self._kidnap.step(time.monotonic() - t0, self._race_t0, self.sim.vehicle)
            self._took("シム", t_poll)
            for f in frames:
                self._on_frame(f, t0)

            now = time.monotonic_ns()
            if now >= self._next_ping_ns:
                self._send_ping(now)
            if now >= self._next_tick_ns:
                # 点群の合間の追従（`planning_node._tick` と同じ。50Hz）
                self._next_tick_ns = now + NS // 50
                self._step_tick()
            if now >= self._next_cmd_ns:
                self._send_cmd(now, t0)

        self._finish_run()
        return self.res

    def _took(self, key: str, t_start: float) -> None:
        """ループの各部の最大 [ms]（どこで詰まったかを見る）。走り出してから数える。"""
        if self._gate is not None:
            slow = self.res.slow_ms
            slow[key] = max(slow.get(key, 0.0), (time.perf_counter() - t_start) * 1000)

    def _step_movers(self, t0: float) -> None:
        """他車（`--mover`）を動かして世界の格子へ刻み、接触を数える。"""
        c = self.sim.course
        path = getattr(self.planner, "path", None)
        for m in self._movers:
            # ★ 経路の計算（仕上げ）が終わってから結び付ける。最初の経路（仕上げ前・速度の低い線）に結ぶと、
            #   他車はそれを自車と違う線・遅い速度で走り続け、結ぶタイミング次第で結果が大きく変わった
            #   （同じ設定で 2 周目以降の平均 12.3 秒と 16.2 秒）
            busy = getattr(self.planner, "_job", None) is not None
            if (m.center is None and path is not None and len(path) >= 3 and not busy
                    and getattr(path, "closed", True)
                    and (self._map_origin is not None or not self._map_name)):
                ox, oy, oyaw = self._map_origin or self._origin
                cs, sn = math.cos(oyaw), math.sin(oyaw)
                q = np.asarray(path.xy, dtype=float)
                w = np.stack([ox + q[:, 0] * cs - q[:, 1] * sn,
                              oy + q[:, 0] * sn + q[:, 1] * cs], axis=1)
                m.bind(w, tuple(self.sim.course.start[:2]), np.asarray(path.v, dtype=float))
                m.t_start += time.monotonic() - t0      # 経路ができた時刻から数える
        update_all(self._movers, c.grid, c.origin, c.resolution, time.monotonic() - t0)
        c._padded_grid = None
        st_ = getattr(self, "_last_st", None)
        if st_ is not None and os.environ.get("MOVER_LOG") and st_.reason != getattr(self, "_mv_reason", None) \
                and any(k in st_.reason for k in ("障害", "避け")):
            v_ = self.sim.vehicle
            print(f"  {time.monotonic() - t0:5.1f}s v={v_.speed:.2f} car({v_.x:.2f},{v_.y:.2f}) "
                  f"mover({self._movers[0].xy[0]:.2f},{self._movers[0].xy[1]:.2f}) {st_.reason}", flush=True)
        if st_ is not None:
            self._mv_reason = st_.reason
        ncol = self.sim.vehicle.collisions
        if ncol == self._mv_col:
            self._mv_speed = float(self.sim.vehicle.speed)
        if ncol != self._mv_col:
            self._mv_col = ncol
            v_ = self.sim.vehicle
            # 車体の中心（後輪軸から前へ半分）から見て、いちばん近い他車が前か後ろか
            ctr = 0.5 * (max(x for x, _ in self.sim.spec.footprint) + min(x for x, _ in self.sim.spec.footprint))
            cx = v_.x + ctr * math.cos(v_.yaw)
            cy = v_.y + ctr * math.sin(v_.yaw)
            near = min(self._movers, key=lambda m: math.hypot(m.xy[0] - cx, m.xy[1] - cy))
            dist = math.hypot(near.xy[0] - cx, near.xy[1] - cy)
            if dist < 0.6:
                # どちらが当てたか: 相手へ向かう速さ（車は接触で速度が 0 になるので直前の値）
                rx, ry = (near.xy[0] - cx) / dist, (near.xy[1] - cy) / dist
                sp = getattr(self, "_mv_speed", 0.0)
                car_in = sp * (math.cos(v_.yaw) * rx + math.sin(v_.yaw) * ry)
                mv_in = -abs(near.u_now) * (math.cos(near.yaw) * rx + math.sin(near.yaw) * ry)
                if mv_in > car_in:
                    self.res.mover_hit_behind += 1
                else:
                    self.res.mover_hit_front += 1
            st_ = getattr(self, "_last_st", None)
            print(f"  {time.monotonic() - t0:5.1f}s 衝突 車({v_.x:.2f},{v_.y:.2f}) 向き{math.degrees(v_.yaw):.0f}° "
                  f"速度{v_.speed:.2f} "
                  + " ".join(f"mover({m.xy[0]:.2f},{m.xy[1]:.2f}) 向き{math.degrees(m.yaw):.0f}°"
                             for m in self._movers)
                  + (f" [{st_.phase}] {st_.reason}" if st_ is not None else ""), flush=True)

    def _on_frame(self, f, t0: float) -> None:
        """受信した 1 フレーム。点群が組み上がったら計画し、指令（`self._cmd`）を作り直す。"""
        self._cap.scan = None
        msg = self.tracker.feed(f.rx_ns, f.type, f.seq, f.payload)
        if isinstance(msg, packets.Telemetry):
            self.vs = self._cap.vs
            # `planning_node` と同じく、届いた順に全部渡す（点ごとの脱スキューと、
            # 点群の合間の姿勢の前進に使う。渡さないと SLAM は
            # 「1周のあいだ twist は一定」の近似になる）
            if self._feed_vs is not None:
                t_vs = time.perf_counter()
                self._feed_vs(self.vs)
                self._took("車両状態", t_vs)
            return
        if isinstance(msg, packets.Log) and msg.message.startswith(b"ARM ignored"):
            # ホストが詰まって COMMAND が 100ms 途切れると、仮想 STM32 はファームと同じく
            # ARM を一度外すまで受け付けない。実機では io_node が GUI の ARM を下ろさせ、
            # 人が入れ直す。ベンチは人の代わりに1回だけ外して入れ直す
            self._rearm = True
            return
        if not isinstance(msg, (packets.LidarSector, packets.LidarSectorI,
                                packets.LidarSectorC)):
            return
        scan = self._cap.scan
        if scan is None:
            return

        now = time.monotonic_ns()
        dt = (now - self._last_plan_ns) / NS if self._last_plan_ns else 0.0
        self._last_plan_ns = now
        if self._kidnap is not None and self._kidnap.holding:
            scan = blank_scan(scan)          # 持ち上げている間は何も見えない
        scan = mask_body(scan, self._body)
        scan = self._scan_filter.apply(scan, self.vs)
        self.scan_dropped += self._scan_filter.dropped
        t_plan = time.perf_counter()
        st = self.planner.plan(scan, self.vs, self.params, dt)
        if self.plan_slowdown > 1.0:
            # Pi の遅さ: 壁時計で回るので、待つ間も仮想 STM32 の車は進む（指令が古くなる）
            time.sleep((time.perf_counter() - t_plan) * (self.plan_slowdown - 1.0))
        self.res.plan_ms.append((time.perf_counter() - t_plan) * 1000)
        self.res.plans += 1
        self._took("計画", t_plan)
        t_now = time.monotonic() - t0
        t_rec = time.perf_counter()
        self._record(st, t_now)
        self._events.step(st, t_now)
        self._took("記録", t_rec)
        self._cmd = self._cmd_from(st)

    def _cmd_from(self, st) -> DriveCmd:
        # `planning_node._cmd_from` と同じ変換に、実機で `telemetry_node._merge_auto` が
        # 載せる GUI のレート制限（`[safety] cmd_*_limit`）を足す。★ 載せないと
        # `steer_rate_limit=0` をファームが「最大舵角/秒」（0.52rad/s、実機の 7.0 の
        # 1/13）と読み、切り返しに舵が間に合わず経路から外れた
        sp = self.sim.spec
        accel = sp.cmd_accel_limit_m_s2
        if st.accel_limit > 0:
            accel = min(accel, st.accel_limit) if accel > 0 else st.accel_limit
        if not st.ready or st.brake:
            return DriveCmd(mode=2, arm=True, brake=True,
                            target_steer=st.target_steer,
                            steer_rate_limit=sp.cmd_steer_rate_limit_rad_s,
                            brake_torque=st.brake_torque if st.ready else 0.0)
        return DriveCmd(mode=2, arm=True, target_speed=st.target_speed,
                        target_steer=st.target_steer, accel_limit=accel,
                        steer_rate_limit=sp.cmd_steer_rate_limit_rad_s,
                        launch=st.launch)

    def _step_tick(self) -> None:
        """点群の合間の追従（`planning_node._tick` と同じ）。指令の舵・速度を上書きする。"""
        cmd = self._cmd
        st_last = getattr(self, "_last_st", None)
        if (self._tick_fn is not None and st_last is not None and st_last.ready
                and not cmd.brake and cmd.mode == 2):
            t_tick = time.perf_counter()
            out = self._tick_fn(self.vs)
            self._took("追従", t_tick)
            if out is not None:
                steer, speed, ramp = out
                if steer is not None:
                    cmd.target_steer = float(steer)
                if speed is not None:
                    cmd.target_speed = float(speed)
                    lim = self.sim.spec.cmd_accel_limit_m_s2
                    cmd.accel_limit = (min(lim, ramp) if lim > 0 else ramp) if ramp > 0 \
                        else lim

    def _send_cmd(self, now: int, t0: float) -> None:
        """指令を送る（100Hz）。乗っ取り（`--hijack`）と ARM の入れ直しはここで差し替える。"""
        cmd_period = NS // CMD_HZ
        cmd = self._cmd
        # 指令の間隔（ホストが詰まって `cmd_deadman_ms` を超えると、仮想 STM32 が車を止める。
        # その走行は捨てる——走りの問題ではない）
        # （走っている間だけ数える。地図の読み込み・ラインの計算で止まっている間は関係ない）
        if self._last_cmd_ns and self._gate is not None and abs(self.sim.vehicle.speed) > 0.3:
            self.res.cmd_gap_ms = max(self.res.cmd_gap_ms, (now - self._last_cmd_ns) / 1e6)
        self._last_cmd_ns = now
        self._next_cmd_ns = now + cmd_period
        out_cmd = cmd
        hj = self._hijack(time.monotonic() - t0)
        if hj is not None:
            out_cmd = DriveCmd(mode=2, arm=True, target_speed=hj[1], target_steer=hj[0],
                               accel_limit=self.sim.spec.cmd_accel_limit_m_s2,
                               steer_rate_limit=self.sim.spec.cmd_steer_rate_limit_rad_s)
        if self._rearm:
            out_cmd = DriveCmd(mode=0)       # ARM を一度外す（次の周期から元の指令に戻る）
            self._rearm = False
            self.res.rearms += 1
        self.link.send(command_from_cmd(
            out_cmd, allow_arm=True,
            max_speed=self.max_speed, max_steer=self.max_steer))
        t_smp = time.perf_counter()
        self._sample(time.monotonic() - t0, cmd, cmd_period / NS)
        self._took("集計", t_smp)

    def _finish_run(self) -> None:
        """走り終えたときの集計（衝突・回避の計算時間・理想ラップ）と記録の後始末。"""
        self.res.collisions = self.sim.vehicle.collisions
        if self._kidnap is not None:
            self.res.kidnaps = [k.line() for k in self._kidnap.items]
            self.res.kidnap_items = list(self._kidnap.items)
        hist = getattr(self.planner, "_avoid_ms_hist", None)
        if hist:
            self.res.avoid_ms = list(hist)     # 追い抜きの計画は回した周期ごとの時間
        if self._race_col0 is not None:
            self.res.race_collisions = self.sim.vehicle.collisions - self._race_col0
        self.res.distance = float(self.sim.vehicle.odom_front[0])
        path = getattr(self.planner, "path", None)
        if path is not None and getattr(path, "closed", False) and len(path) >= 3:
            ds = np.hypot(*(np.roll(path.xy, -1, axis=0) - path.xy).T)
            v = np.maximum(path.v, 1e-3)
            self.res.ideal_lap = float(np.sum(2.0 * ds / (v + np.roll(v, -1))))
            self.res.path_len = float(path.length)
        if self._trace is not None:
            self._trace.close()

    def _hijack(self, t: float) -> tuple[float, float] | None:
        """`--hijack` の窓の中なら `(舵角, 速度)`。走行開始（`_race_t0`）から数える。"""
        if self._race_t0 is None:
            return None
        tr = t - self._race_t0
        for t1, dur, steer, speed in self._hijacks:
            if t1 <= tr < t1 + dur:
                if not getattr(self, "_hj_on", False) and not self.quiet:
                    print(f"  {t:5.1f}s 指令を乗っ取る（{dur:.1f}s・舵 {steer:+.2f}・{speed:.2f}m/s）", flush=True)
                self._hj_on = True
                return steer, speed
        self._hj_on = False
        return None

    def _sample(self, t: float, cmd: DriveCmd, dt: float) -> None:
        """指令周期（100Hz）で真値を見る: ゲートのラップ・最高速・制動・記録。"""
        st = getattr(self, "_last_st", None)
        if st is None or st.phase not in ("RACE", "DETOUR"):
            return
        v = self.sim.vehicle
        if self._gate is None:
            if not st.ready:
                return
            self._gate = (v.x, v.y, math.cos(v.yaw), math.sin(v.yaw))
            self.res.notes.append(f"RACE 開始 地図座標 ({st.pose_x:.2f}, {st.pose_y:.2f})"
                                  f"（--locate-hint に使える）")
            self._gate_last_t = t
            self._gate_odom = float(v.odom_front[0])
            self._race_col0 = v.collisions
            self._race_t0 = t
        self._sample_grip(t, st, v)
        gx, gy, tx, ty = self._gate
        along = (v.x - gx) * tx + (v.y - gy) * ty
        lat = -(v.x - gx) * ty + (v.y - gy) * tx
        prev = self._gate_prev
        self._gate_prev = (t, along, lat)
        clear = self._clearance(v.x, v.y, v.yaw)
        self.res.race_min_clear = min(self.res.race_min_clear, clear)
        self._lap_clear = min(self._lap_clear, clear)
        if (prev is not None and prev[1] < 0.0 <= along and abs(lat) < 1.0
                and float(v.odom_front[0]) - self._gate_odom > 3.0):
            tc = prev[0] + (t - prev[0]) * (-prev[1]) / max(along - prev[1], 1e-9)
            self.res.gate_laps.append(tc - self._gate_last_t)
            self.res.lap_min_clear.append(self._lap_clear)
            self._lap_clear = math.inf
            self._gate_last_t = tc
            self._gate_odom = float(v.odom_front[0])
            if not self.quiet:
                print(f"  {t:5.1f}s 真値ラップ {self.res.gate_laps[-1]:.2f}s", flush=True)
        self.res.race_vmax = max(self.res.race_vmax, float(v.speed))
        if cmd.brake:
            self.res.race_brake_s += dt
        r = self.res
        r.race_n += 1
        if abs(cmd.target_steer) >= 0.98 * self.sim.spec.max_steer:
            r.race_steer_sat += 1
        r.race_a_lat.append(abs(float(v.accel_lateral)))
        if not cmd.brake:
            if cmd.target_speed > v.speed + 0.3:
                r.race_accel.append(float(v.accel_x))
            elif cmd.target_speed < v.speed - 0.3:
                r.race_decel.append(-float(v.accel_x))
        if abs(v.speed) < 0.05:
            r.stop_s += dt
        pl = self.planner
        path_now = getattr(pl, "_avoid_path", None) or getattr(pl, "path", None)
        hint = getattr(pl, "_hint", -1)
        if path_now is not None and 0 <= hint < len(path_now):
            r.race_over.append(float(v.speed) - float(path_now.v[hint]))
        if self._movers:
            self._sample_movers(v, pl, hint, dt)
        av, dt_on = getattr(pl, "_avoid", None), getattr(pl, "_detour", None) is not None
        on = av is not None and getattr(av, "obstacle", None) is not None
        if getattr(pl, "_avoid_how", "") == "追い抜き":
            # 追い抜きの計画は毎周期引き直すので、避けていない → 避けている の切り替わりで数える
            prev_on = self._prev_avoid is not None and getattr(self._prev_avoid, "obstacle", None) is not None
            r.avoid_starts += on and not prev_on
        elif on and av is not self._prev_avoid:
            r.avoid_starts += 1
            r.avoid_elastic += getattr(pl, "_avoid_how", "") == "引き直し"
            r.avoid_ms.append(float(getattr(pl, "_avoid_ms", 0.0)))
        self._prev_avoid = av
        if dt_on:
            r.detour_s += dt
            if not self._prev_detour:
                r.detour_starts += 1
        self._prev_detour = dt_on
        if self._trace is not None:
            self._trace.row(t, st, v, cmd, clear)

    def _sample_grip(self, t: float, st, v) -> None:
        """横グリップの学習の状態を真値と突き合わせる（100Hz）。"""
        pl = self.planner
        g = getattr(pl, "_grip", None)
        if g is None or not getattr(pl, "_grip_on", False):
            return
        r = self.res
        r.grip_on = True
        tr = t - (self._race_t0 or t)
        if self._grip_t0 is None:
            # 学習側の時刻（車両状態の時刻）と bench の時刻のずれ
            self._grip_t0 = (self.vs.t_capture / 1e9 - tr) if self.vs is not None else 0.0
        slip = float(v.slip_frac)
        self._grip_slip.append((t, slip))
        while self._grip_slip and self._grip_slip[0][0] < t - 0.4:
            self._grip_slip.pop(0)
        clear = self._clearance(v.x, v.y, v.yaw)
        for e in self._grip_open:
            e["cross"] = max(e["cross"], abs(st.cross_track))
            e["clear"] = min(e["clear"], clear)
            e["over"] = max(e["over"], slip)
        self._grip_open = [e for e in self._grip_open if t - e["_t"] < 1.5]
        while self._grip_n < len(g.events):
            src = g.events[self._grip_n]
            self._grip_n += 1
            e = {"t": tr, "_t": t, "kind": src["kind"], "a": src["a"], "applied": src["applied"],
                 "over": max(s for _, s in self._grip_slip), "cross": abs(st.cross_track),
                 "clear": clear}
            r.grip_events.append(e)
            self._grip_open.append(e)
            if not self.quiet:
                print(f"  {t:5.1f}s ★横の安全装置 {src['kind']} 横加速度 {src['a']:.2f}（上限 "
                      f"{src['applied']:.2f}）真値の超過 {e['over'] * 100:+.1f}%", flush=True)
        if g.converged and r.grip_conv_t is None:
            r.grip_conv_t, r.grip_conv_laps = tr, len(r.gate_laps)
        elif not g.converged and r.grip_conv_t is not None and not g.down_only:
            r.grip_conv_t = r.grip_conv_laps = None      # 収束が解けた（路面が変わった等）
        if abs(g.map.level - self._grip_lv) > 1e-6:
            self._grip_lv = g.map.level
            r.grip_levels.append((tr, g.map.level))
        r.grip_level, r.grip_ceil = g.map.level, g.map.ceil
        r.grip_state = g.state(pl._grip_gate)
        r.grip_steps = (len(g.steps), sum(1 for x in g.steps if x["ok"]))
        r.grip_true = float(v.spec.mu * v.grip_lat * 9.81)
        lg, sp = g.long, v.spec
        ev = [{**e, "t": e["t"] - self._grip_t0} for e in lg.events]
        trail = r.grip_long.get("trail", [])
        if not trail or abs(trail[-1][1] - g.map.accel) + abs(trail[-1][2] - g.map.decel) > 1e-6:
            trail.append((tr, g.map.accel, g.map.decel))
        # 真値: 路面で決まる上限と、目標車速を動かす傾きの上限の小さい方
        acc_true = min(sp.drive_accel_m_s2 * v.grip_long - sp.rolling_resistance,
                       sp.cmd_accel_limit_m_s2)
        dec_true = min(sp.speed_decel_m_s2 * v.grip_long + sp.rolling_resistance,
                       sp.cmd_accel_limit_m_s2)
        r.grip_long = {"acc": g.map.accel, "dec": g.map.decel, "windows": lg.windows, "events": ev,
                       "acc_env": list(g.map.acc), "dec_env": list(g.map.dec),
                       "trail": trail, "acc_true": acc_true, "dec_true": dec_true,
                       "done": (True, lg.dec_done)}

    def _sample_movers(self, v, pl, hint: int, dt: float) -> None:
        """他車との距離（真値）・抜いた回数・近くで抑えられた時間を数える。"""
        r = self.res
        co, si = math.cos(v.yaw), math.sin(v.yaw)
        b = self._outline
        wx = v.x + b[:, 0] * co - b[:, 1] * si
        wy = v.y + b[:, 0] * si + b[:, 1] * co
        near = False
        rel = getattr(self, "_mv_rel", None) or [None] * len(self._movers)
        for i, m in enumerate(self._movers):
            if m.center is None:
                continue
            mx, my = m.xy
            if m.length > 0.0:
                mc, ms = math.cos(m.yaw), math.sin(m.yaw)
                lx = np.abs((wx - mx) * mc + (wy - my) * ms) - 0.5 * m.length
                ly = np.abs(-(wx - mx) * ms + (wy - my) * mc) - m.r
                d = np.where((lx > 0) | (ly > 0), np.hypot(np.maximum(lx, 0), np.maximum(ly, 0)),
                             np.maximum(lx, ly))
                d = float(d.min())
            else:
                d = float((np.hypot(wx - mx, wy - my) - m.r).min())
            r.mover_min_clear = min(r.mover_min_clear, d)
            fwd = (mx - v.x) * co + (my - v.y) * si
            lat = -(mx - v.x) * si + (my - v.y) * co
            near |= 0.0 < fwd < 3.0 and abs(lat) < 1.0
            if rel[i] is not None and rel[i] > 0.0 >= fwd and abs(lat) < 1.0 and m.u_now > 0:
                r.mover_passes += 1
            rel[i] = fwd if abs(lat) < 1.0 else None
        self._mv_rel = rel
        base = getattr(pl, "path", None)
        if near and base is not None and 0 <= hint < len(base) and \
                float(v.speed) < float(base.v[hint]) - 0.3:
            r.mover_slow_s += dt

    def _clearance(self, x: float, y: float, yaw: float) -> float:
        """車体外形（輪郭）と真の壁との最小距離 [m]。0 = 接触。"""
        c = self.sim.course
        co, si = math.cos(yaw), math.sin(yaw)
        b = self._outline
        wx = x + b[:, 0] * co - b[:, 1] * si
        wy = y + b[:, 0] * si + b[:, 1] * co
        cols = np.clip(((wx - c.origin[0]) / c.resolution).astype(np.int32), 0,
                       self._edt.shape[1] - 1)
        rows = np.clip(((wy - c.origin[1]) / c.resolution).astype(np.int32), 0,
                       self._edt.shape[0] - 1)
        return float(self._edt[rows, cols].min())

    def _record(self, st, t: float) -> None:
        self._last_st = st
        if st.reason != self._prev_reason_head and ("障害物" in st.reason or "迂回" in st.reason[:4]):
            if len(self._obs_log) < 400:
                self._obs_log.append(f"{t:6.1f}s v={self.sim.vehicle.speed:.2f} {st.reason[:150]}")
        self._prev_reason_head = st.reason
        v = self.sim.vehicle
        self._truth_log.append((st.phase, st.laps, getattr(st, "route_active", ""), v.x, v.y))
        if self._map_name and self._map_origin is None and st.phase in ("RACE", "DETOUR") and st.ready:
            self._align_map(st)
        if not self._map_name or self._map_origin is not None:
            # 保存済み地図の map フレームは bench の起点と無関係なので、`--map` は地図を真のコースへ
            # 重ねてから比べる（RACE に入る前は比べない）
            tx, ty, tyaw = self._truth_rel()
            self.res.pose_err.append(math.hypot(st.pose_x - tx, st.pose_y - ty))
            d = (st.pose_yaw - tyaw + math.pi) % (2 * math.pi) - math.pi
            self.res.yaw_err.append(abs(math.degrees(d)))
            if st.phase in ("RACE", "DETOUR", "LOCATE") and self._map_origin is not None:
                pe = self.res.pose_err[-1]
                step = 0.0 if self._wrong_xy is None else math.hypot(v.x - self._wrong_xy[0], v.y - self._wrong_xy[1])
                if pe > 0.30 and step < 0.5:          # 0.5m 超の 1 周期の跳びは置き直し（走った距離ではない）
                    self.res.wrong_pose_dist += step
                self._wrong_xy = (v.x, v.y)
                if self._kidnap is not None:
                    self._kidnap.observe(t, pe, self.res.yaw_err[-1], v.x, v.y)
            if self._map_name and st.phase in ("RACE", "DETOUR"):
                dist = self._path_distance(tx, ty)
                if dist is not None:
                    self.res.cross_true.append(dist)
        if st.phase == "RACE":
            self.res.cross.append(st.cross_track)
        if st.laps > self._prev_laps:
            self.res.lap_times.append(t - self._last_lap_t)
            self._last_lap_t = t
            self._prev_laps = st.laps

        # EXPLORE段が終わった瞬間（地図構築完了、直後の姿勢）の誤差を1回だけ残す。
        # RACE段の誤差は凍結地図に対する追跡精度でしかないため、
        # 「地図構築中にどれだけドリフトしたか」はここでしか直接見えない
        if self._prev_phase == "EXPLORE" and st.phase != "EXPLORE" and self.res.pose_err:
            self.res.explore_end_pose_err = self.res.pose_err[-1]
            self.res.explore_end_yaw_err = self.res.yaw_err[-1]
        self._prev_phase = st.phase

        self.res.laps = st.laps
        self.res.phase = st.phase
        self.res.reason = st.reason

        if not self.quiet and self.res.plans % 20 == 0:
            err = (f"誤差 {self.res.pose_err[-1] * 100:5.1f}cm {self.res.yaw_err[-1]:4.1f}°"
                   if self.res.pose_err else f"速度 {v.speed:4.2f}m/s")
            print(f"  {t:5.1f}s [{st.phase or '-'}] 真値({v.x:5.2f},{v.y:5.2f}) {err}  "
                  f"一致度 {st.match_score:.2f}  {st.reason}", flush=True)

    def _align_map(self, st) -> None:
        """`--map` の地図を真のコースへ剛体で重ね、地図の原点の世界姿勢を決める。

        初期値は「いまの真の姿勢と planner の自己位置が一致する原点」。そこから壁の重なりで
        詰める（`slam_bench.align_map`）ので、自己位置の誤差は壁の重ね合わせには入らない。"""
        v = self.sim.vehicle
        oyaw = v.yaw - st.pose_yaw
        c, s = math.cos(oyaw), math.sin(oyaw)
        guess = (v.x - (st.pose_x * c - st.pose_y * s), v.y - (st.pose_x * s + st.pose_y * c), oyaw)
        slam = getattr(self.planner, "slam", None)
        if slam is None:
            self._map_origin = guess
            return
        from .slam_bench import align_map
        self._map_origin = align_map(slam.grid, self.sim.course, guess)
        d = math.hypot(self._map_origin[0] - guess[0], self._map_origin[1] - guess[1])
        self.res.notes.append(f"地図を真のコースへ重ねた（初期値から {d * 100:.0f}cm・"
                              f"{math.degrees(self._map_origin[2] - guess[2]):+.1f}°）")

    def _path_distance(self, x: float, y: float) -> float | None:
        """map フレームの点 `(x, y)` から、いま追っている経路（折れ線）までの距離 [m]。"""
        path = getattr(self.planner, "path", None)
        if path is None or len(path) < 2:
            return None
        xy = np.asarray(path.xy, dtype=np.float64)
        a = xy
        b = np.roll(xy, -1, axis=0) if getattr(path, "closed", False) else xy[1:]
        if not getattr(path, "closed", False):
            a = xy[:-1]
        ab = b - a
        l2 = np.maximum((ab ** 2).sum(axis=1), 1e-12)
        t = np.clip(((np.array([x, y]) - a) * ab).sum(axis=1) / l2, 0.0, 1.0)
        proj = a + ab * t[:, None]
        return float(np.sqrt(((proj - np.array([x, y])) ** 2).sum(axis=1)).min())

    def map_quality(self) -> str:
        """出来上がった地図を**真のコースと重ねて**採点する。

        姿勢の誤差だけを見ていると「地図がどれくらい歪んでいるか」が分からない。
        実機の GUI で崩れて見えた地図が、数字の上では悪く見えないこともある。

        - **正確さ**: 地図に立てた壁のうち、真の壁の近くにあるものの割合
        - **網羅**: 走った付近の真の壁のうち、地図に立ったものの割合

        SLAM の原点は「最初の姿勢」なので、真のコースを起点の車体座標へ移してから比べる。
        """
        planner = getattr(self.planner, "slam", None)
        if planner is None:
            return "—"
        g = planner.grid
        wall = g.wall_mask()
        if not wall.any():
            return "地図が空"

        course = self.sim.course
        ox, oy, oyaw = self._origin
        c, s = math.cos(-oyaw), math.sin(-oyaw)

        # 地図の壁セル → 世界座標（シムの座標系）
        rows, cols = np.nonzero(wall)
        mx, my = g.to_world(cols, rows)
        wx = ox + mx * math.cos(oyaw) - my * math.sin(oyaw)
        wy = oy + mx * math.sin(oyaw) + my * math.cos(oyaw)

        # 真の壁からの距離。**壁を 1 セル膨らませた版と比べる**（量子化のぶん）
        tol = 2
        occ = course.grid
        col = ((wx - course.origin[0]) / course.resolution).astype(np.int32)
        row = ((wy - course.origin[1]) / course.resolution).astype(np.int32)
        ok = ((col >= tol) & (col < occ.shape[1] - tol)
              & (row >= tol) & (row < occ.shape[0] - tol))
        near = np.zeros(col.size, dtype=bool)
        for dr in range(-tol, tol + 1):
            for dc in range(-tol, tol + 1):
                near[ok] |= occ[row[ok] + dr, col[ok] + dc]
        good = float(near.sum()) / max(1, near.size)
        return (f"壁 {int(wall.sum())} セル / 正確さ {good * 100:.0f}% "
                f"（真の壁から {tol * course.resolution * 100:.0f}cm 以内）")

    def route_report(self) -> None:
        """走行中（RACE）に真値で通った道路グラフのエッジを周回ごとに数える。

        グラフは**真のコース**から作る（SLAM の地図とは別物。番号の意味は
        `raspi/nav/roadgraph.py` を真値に当てたもの）。停止点に止まったなら
        停止誤差も出す。
        """
        from raspi.core.vehicle import Vehicle
        from raspi.nav.roadgraph import build_graph, trinary_from_bool

        course = self.sim.course
        veh = Vehicle.load()
        try:
            g = build_graph(trinary_from_bool(course.grid), resolution=course.resolution,
                            origin=course.origin, keep=veh.half_width + 0.02,
                            seed=self._origin[:2])
        except ValueError as e:
            self.res.notes.append(f"真値の道路グラフを作れない: {e}")
            return
        h, w = g.label.shape
        laps: dict[int, tuple[str, list[int]]] = {}
        for phase, lap, key, x, y in self._truth_log:
            if phase != "RACE":
                continue
            c, r = g.to_cell(x, y)
            if not (0 <= c < w and 0 <= r < h):
                continue
            e = int(g.label[r, c])
            if e < 0:
                continue
            k, seq = laps.setdefault(lap, (key, []))
            if not seq or seq[-1] != e:
                seq.append(e)
            laps[lap] = (k or key, seq)
        self.res.lap_edges = [laps[k] for k in sorted(laps)]
        if self._routes_world is not None and self.res.phase in ("STOPPED", "PARK"):
            then = (self._routes_world.get("mission") or {}).get("then", "")
            stop = (self._routes_world.get("stops") or {}).get(then)
            if stop is not None:
                v = self.sim.vehicle
                self.res.stop_err = math.hypot(v.x - stop["x"], v.y - stop["y"])
                mx, my, _ = self._to_map(stop["x"], stop["y"])
                st = self._last_st
                self.res.stop_err_est = math.hypot(st.pose_x - mx, st.pose_y - my)

    def save_grip(self, dst: Path) -> str:
        """学習した横グリップ（一時ディレクトリの `<地図>.grip.json`）を `dst` へ写す（`--save-grip`）。"""
        import shutil

        from raspi.auto import mapstore
        save = getattr(self.planner, "_grip_save", None)
        if save is not None:
            save()
        name = getattr(self.planner, "_grip_name", "")
        src = mapstore.MAPS_DIR / f"{name}.grip.json"
        if not name or not src.is_file():
            return "学習した横グリップが無いので保存しない"
        shutil.copy(src, dst)
        return f"学習した横グリップを {dst} に保存した"

    def save_built_map(self, name: str) -> str:
        """bench が作った地図を実機の `saved_maps/` へ `name` で写す（`--save-map`）。"""
        import shutil

        from raspi.auto import mapstore
        src = getattr(self.planner, "_saved_map_name", "")
        if not src:
            return "地図ができていないので保存しない"
        self._real_maps_dir.mkdir(parents=True, exist_ok=True)
        for suf in (".npz", ".json", ".routes.json"):
            f = mapstore.MAPS_DIR / f"{src}{suf}"
            if f.is_file():
                shutil.copy(f, self._real_maps_dir / f"{name}{suf}")
        return f"地図を {self._real_maps_dir / name}.npz に保存した"

    def close(self) -> None:
        close = getattr(self.planner, "close", None)
        if close is not None:
            close()
        self.link.close()
        # 書き換えた保存先を戻し、一時ディレクトリを消す
        import shutil

        from raspi.auto import mapstore
        mapstore.MAPS_DIR = self._real_maps_dir
        shutil.rmtree(self._tmp_maps_dir, ignore_errors=True)


def _error_preset(name: str) -> dict[str, float]:
    """`--errors` の車速・ジャイロの誤差を `SimParams` の欄に直す。値は `slam_bench.SensorModel`
    の preset から読む（二重に持たない）。"""
    if name == "none":
        return {}
    from .slam_bench import SensorModel
    m = SensorModel.preset(name)
    return {"speed_scale_err": m.speed_scale - 1.0, "gyro_scale_err": m.gyro_scale - 1.0,
            "gyro_bias_dps": m.gyro_bias_dps, "gyro_noise_dps": m.gyro_noise_dps}


def _build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="normal", help="コース名 or パス")
    ap.add_argument("--mode", default="slam2d_raceline", choices=list(PLANNERS))
    ap.add_argument("--time", type=float, default=180.0, help="走らせる秒数")
    ap.add_argument("--seed", type=int, default=0, help="シムの乱数シード（LiDARノイズ等）")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="planner のパラメータを上書き（複数可）")
    # 実機の io_node は STM32 の LIMITS（5m/s）で切る。3.0 のままだと `v_max` を 3 より上げても効かない
    ap.add_argument("--max-speed", type=float, default=5.0)
    ap.add_argument("--max-steer", type=float, default=0.524)
    ap.add_argument("--model", default=None,
                    help="reload_if_changedに対応するplanner（e2e_lidar）へ渡すモデル名"
                         "（例: v2 → models/e2e_lidar/v2.onnx）。他のモードでは無視される")
    ap.add_argument("--routes", default=None,
                    help="経路の設定 JSON（`raspi/auto/route_config.py` の形、**シムの世界座標**で書く）。"
                         "slam2d_route 用")
    ap.add_argument("--switch", action="append", default=[], metavar="T:GROUP",
                    help="時刻 T [s] に経路グループを切り替える（複数可）")
    ap.add_argument("--signal", action="append", default=[], metavar="T:VALUE",
                    help="時刻 T [s] に信号認識の値を送る（`signal_map` で読み替え、複数可）")
    ap.add_argument("--no-auto-race", action="store_true",
                    help="地図作成が終わっても走り出さない（既定は bench が「走行」を押す）")
    ap.add_argument("--map", default=None,
                    help="saved_maps/ の地図名。地図作成を飛ばし、読み込んで LOCATE から走る")
    ap.add_argument("--save-map", default=None, metavar="NAME",
                    help="bench が作った地図を saved_maps/NAME として残す")
    ap.add_argument("--locate-hint", default=None, metavar="X,Y",
                    help="--map のとき LOCATE に渡すおおよその位置（地図座標）。対称なコースで速く確実にする")
    ap.add_argument("--trace", default=None, help="RACE 中の速度などを CSV で書く（100Hz）")
    ap.add_argument("--obstacle", action="append", default=[], metavar="X,Y,R",
                    help="シムの世界座標に円柱の障害物を置く（繰り返し可。地図には入れないなら --map と併用）")
    ap.add_argument("--mover", action="append", default=[], metavar="S0,SPEED[,D,R,T0,L,K]",
                    help="中心線に沿って動く円柱（他車の代わり。繰り返し可）。S0=出発点から先の弧長[m]、"
                         "SPEED=速度[m/s]（負で逆走）、D=横ずれ[m]、R=半径[m]（矩形なら半幅）、T0=動き出す時刻[s]、"
                         "L=長さ[m]（0 で円盤、正で車の形の矩形）、K=正なら経路の速度プロファイル×K で走る"
                         "（SPEED は上限、0 で上限なし。コーナーで落ちる相手）")
    ap.add_argument("--vehicle", action="append", default=[], metavar="KEY=VALUE",
                    help="真値の車だけ諸元を上書き（`sim.vehicle.VehicleSpec` の欄名。例 mu=0.37、複数可）。"
                         "planner は config/vehicle.toml のまま")
    ap.add_argument("--control", action="append", default=[], metavar="KEY=VALUE",
                    help="**シムの STM32 だけ** `[control]` の値を上書き（launch_exit_torque_nm・"
                         "speed_ff_*・tc_enable などは仮想 STM32 が持つ）。planner は config/vehicle.toml のまま")
    ap.add_argument("--mu", default=None, metavar="MU[:MU_LONG]",
                    help="真値の路面の摩擦係数（一様）。`--vehicle mu=` と違い、加速の上限も同じ割合で変える。"
                         "MU_LONG を付けると前後だけ別の値（前後と横で摩擦が違う路面）")
    ap.add_argument("--surface", action="append", default=[], metavar="X,Y,R,MU[,MU_LONG]",
                    help="シムの世界座標 (X, Y) の半径 R の円の中だけ摩擦を変える（繰り返し可）")
    ap.add_argument("--mu-at", action="append", default=[], metavar="T:MU[:MU_LONG]",
                    help="bench の時刻 T [s] から一様の摩擦を変える（繰り返し可）")
    ap.add_argument("--assume", action="append", default=[], metavar="KEY=VALUE",
                    help="**planner が信じている車**の諸元だけ上書き（`raspi.core.vehicle.Vehicle` の欄名。"
                         "例 mu=0.55）。真値の車はそのまま")
    ap.add_argument("--grip", default=None, metavar="FILE",
                    help="学習した横グリップの JSON を --map の地図の横へ置いてから走る（保存値からの再開）")
    ap.add_argument("--save-grip", default=None, metavar="FILE",
                    help="走り終えたときの学習した横グリップを FILE へ書く")
    ap.add_argument("--state", default=None, metavar="DIR",
                    help="--map の地図の学習値（.grip.json）と仕上げた経路の写し（.lines.pkl）を DIR から読み、"
                         "走り終えたら DIR へ書く（同じ地図の 2 回目の走行を再現する）")
    ap.add_argument("--grip-set", action="append", default=[], metavar="KEY=VALUE",
                    help="学習の定数を上書き（`raspi.nav.grip.GripConfig` の欄名。上げ幅の実測用）")
    ap.add_argument("--sim", action="append", default=[], metavar="KEY=VALUE",
                    help="センサ誤差などシムの設定を上書き（`sim.params.SimParams` の欄名）")
    ap.add_argument("--real-noise", action="store_true",
                    help="実機で測った走行中の揺れ（ヨーレート・車速・実舵角）をテレメトリに足す"
                         "（`sim.params.REAL_VIBRATION`。`--sim` の指定が優先）")
    ap.add_argument("--errors", default="none", choices=["none", "real", "harsh"],
                    help="閉ループのセンサ誤差（`sim.slam_bench` の real・harsh と同じ車速・ジャイロの"
                         "倍率とバイアス）。既定 none = 今まで通り。`--sim` の指定が優先")
    ap.add_argument("--obs-log", action="store_true",
                    help="障害物まわりの判断（reason の移り変わり）を時刻つきで出す")
    ap.add_argument("--hijack", action="append", default=[], metavar="T:DUR:STEER:SPEED",
                    help="走行開始から T 秒後に DUR 秒だけ指令を舵 STEER [rad]・速度 SPEED [m/s] に乗っ取る"
                         "（経路から外れた・ぶつかった所から戻れるかを見る）")
    ap.add_argument("--plan-slowdown", type=float, default=1.0, metavar="K",
                    help="plan() を K 倍遅く見せる（その間も車は進む）。Mac は Pi より速いので、Pi で同じ planner を"
                         "回した時間との比を入れると計算の遅れを Pi 相当にできる。計画 p99 の表示も K 倍込み")
    ap.add_argument("--kidnap", action="append", default=[], metavar="T:DX:DY:DYAW[:HOLD]",
                    help="走行開始から T 秒で真の車を車体基準で (DX,DY)m・DYAW° 動かす。HOLD 秒は持ち上げ"
                         "（その姿勢で止め、LiDAR は何も返さない）。人が車を置き直す状況（sim/bench_faults.py）")
    ap.add_argument("--place", action="append", default=[], metavar="T:X:Y:YAW[:HOLD]",
                    help="走行開始から T 秒で真の車を世界座標の姿勢 (X,Y,YAW°) へ置き直す（HOLD は --kidnap と同じ）")
    ap.add_argument("--wall-gap", action="append", default=[], metavar="X0,Y0,X1,Y1",
                    help="シムの世界の壁を矩形で消す（世界座標。--map の凍結地図には残る＝壁の欠け）")
    ap.add_argument("--quiet", action="store_true")
    return ap


def _resolve_course(name: str) -> Path | None:
    """コース名かパス → コースのファイル。無ければ候補を出して None。"""
    course = Path(name)
    if not course.exists():
        found = [p for p in list_courses() if p.stem == name]
        if not found:
            names = ", ".join(sorted({p.stem for p in list_courses()}))
            print(f"コースが無い: {name}（候補 {names}）", file=sys.stderr)
            return None
        course = found[0]
    return course


def _options_from_args(args: argparse.Namespace) -> BenchOptions:
    """コマンドラインの引数 → `BenchOptions`。"""
    routes = json.loads(Path(args.routes).read_text()) if args.routes else None
    events: list[tuple[float, str, str]] = []
    for kind, items in (("route", args.switch), ("signal", args.signal)):
        for it in items:
            t, _, v = it.partition(":")
            events.append((float(t), kind, v))

    def pairs(items):
        return {k.strip(): float(v) for k, _, v in (it.partition("=") for it in items)}

    def nums(text, sep):
        return tuple(float(x) for x in text.split(sep))

    surface: dict = {}
    if args.mu:
        surface["base"] = nums(args.mu, ":")
    if args.surface:
        surface["discs"] = [nums(d, ",") for d in args.surface]
    if args.mu_at:
        surface["changes"] = [nums(c, ":") for c in args.mu_at]
    grip_cfg = pairs(args.grip_set)

    return BenchOptions(
        max_speed=args.max_speed,
        max_steer=args.max_steer, model=args.model, quiet=args.quiet, seed=args.seed,
        routes=routes, events=events, auto_race=not args.no_auto_race,
        map_name=args.map, trace=Path(args.trace) if args.trace else None,
        locate_hint=(tuple(float(v) for v in args.locate_hint.split(","))
                     if args.locate_hint else None),
        obstacles=[tuple(float(v) for v in o.split(",")) for o in args.obstacle],
        movers=args.mover,
        vehicle={k.strip(): float(v) for k, _, v in
                 (kv.partition("=") for kv in args.vehicle)},
        control={k.strip(): float(v) for k, _, v in
                 (kv.partition("=") for kv in args.control)},
        surface=surface or None, assume=pairs(args.assume),
        grip_file=Path(args.grip) if args.grip else None, grip_cfg=grip_cfg,
        sim_params={**(REAL_VIBRATION if args.real_noise else {}),
                    **_error_preset(args.errors), **pairs(args.sim)},
        state_dir=Path(args.state) if args.state else None,
        hijacks=[nums(h, ":") for h in args.hijack],
        kidnaps=[parse_kidnap(k) for k in args.kidnap] + [parse_place(k) for k in args.place],
        wall_gaps=[parse_rect(r) for r in args.wall_gap],
        plan_slowdown=args.plan_slowdown)


def _save_after_run(b: Bench, args: argparse.Namespace, res: Result) -> None:
    """走り終えた後の保存（`--save-map`・`--save-grip`・`--state`）。"""
    if args.save_map:
        res.notes.append(b.save_built_map(args.save_map))
    if args.save_grip:
        res.notes.append(b.save_grip(Path(args.save_grip)))
    from raspi.auto import mapstore as _ms
    save = getattr(b.planner, "_grip_save", None)
    if save is not None:
        save()
    if args.state and args.map:
        import shutil as _sh
        _st = Path(args.state)
        _st.mkdir(parents=True, exist_ok=True)
        for suf in (".grip.json", ".lines.pkl"):
            f = _ms.MAPS_DIR / f"{args.map}{suf}"
            if f.is_file():
                _sh.copy(f, _st / f.name)
        res.notes.append(f"地図の学習値・経路の写しを {_st} に書いた")


def main() -> int:
    args = _build_parser().parse_args()
    course = _resolve_course(args.course)
    if course is None:
        return 2

    params: dict[str, float] = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        params[k.strip()] = float(v)

    b = Bench(course, args.mode, params, _options_from_args(args))
    print(f"# {course.name} / {args.mode} / {args.time:.0f}s / seed={args.seed}")
    try:
        res = b.run(args.time)
    except KeyboardInterrupt:
        res = b.res
    quality = b.map_quality()
    b.route_report()
    _save_after_run(b, args, res)
    b.close()
    if args.obs_log:
        print("\n=== 障害物まわりの判断 ===")
        print("\n".join(b._obs_log))
    print("\n=== 結果 ===")
    print(res.report())
    print("地図       " + quality)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
