"""planner をシムの真値と突き合わせて評価する。**バスも GUI もプロセスも使わない。**

    .venv/bin/python -m sim.bench --course circuit
    .venv/bin/python -m sim.bench --course courses/1.json --mode raceline --time 240
    .venv/bin/python -m sim.bench --mode ftg --set explore_speed=0.6
    .venv/bin/python -m sim.bench --course toyota2 --mode slam2d_route \
        --routes sim/routes/toyota2.json --switch 200:B
    .venv/bin/python -m sim.bench --course toyota2 --map toyota2 --mode slam2d_route \
        --time 120 --trace /tmp/t.csv          # 保存済み地図から走る（地図作成を飛ばす）

## ラップタイム（真値のゲート）

RACE で最初に走り出した真値の位置に、進行方向と直交するゲート線を置き、それを
前向きに跨いだ時刻（100Hz で補間）でラップを計る。**プランナーの周回カウンタには
頼らない**（`slam2d_raceline` は RACE の周回を数えない）。1周目はスタンディング
スタートを含むので、比較には2周目以降（フライング）を使う。

`io_node` と同じことを 1 プロセスの中でやる:

    SimLink.poll() → フレーム → ScanAssembler / StateBuilder → planner.plan()
                  → command_from_cmd() → SimLink.send()

**実機と同じ変換コードを通す**（`ScanAssembler` の鏡像反転、整数スケール換算、
`command_from_cmd` のクランプ）ので、ここで走れば `sim.run` でも走る。

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
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402

from raspi.auto import PLANNERS, make_planner, merged_params  # noqa: E402
from raspi.msgs import DriveCmd, ScanAssembler, StateBuilder, command_from_cmd  # noqa: E402
from raspi.proto.generated import packets  # noqa: E402

from .course import list_courses  # noqa: E402
from .link import create_sim_link  # noqa: E402

__all__ = ["Bench", "Result"]

NS = 1_000_000_000
CMD_HZ = 100


class Result:
    """1回の走行の結果。**数字は全部ここに集める。**"""

    def __init__(self) -> None:
        self.plans = 0
        self.plan_ms: list[float] = []
        self.pose_err: list[float] = []        #: SLAM と真値の**移動量**の差 [m]
        self.yaw_err: list[float] = []         #: 同 向き [deg]
        self.cross: list[float] = []           #: 経路からの横偏差 [m]（RACE 段のみ）
        self.collisions = 0
        self.laps = 0
        self.phase = ""
        self.reason = ""
        self.lap_times: list[float] = []
        self.distance = 0.0
        #: EXPLORE段が終わった瞬間（地図構築完了時点）の自己位置・向き誤差。
        #: RACE段の誤差は「凍結地図に対する追跡精度」でしかなく、地図自体が
        #: 歪んでいるかどうかはこの数字でしか直接見えない
        #: （`/Users/banbatakumi/.claude/plans/slam2d-slam-slam2d-imu-slam-slam2d-imu-nifty-aurora.md`
        #: フェーズ1参照）
        self.explore_end_pose_err: float | None = None
        self.explore_end_yaw_err: float | None = None
        #: 経路選択走行（`slam2d_route`）の検証。周回ごとに真値で通った道路グラフの
        #: エッジ列（真のコースから作ったグラフの番号）と、その周に走っていた経路
        self.lap_edges: list[tuple[str, list[int]]] = []
        #: 停止点に止まったときの、真値の位置と停止点の距離 [m]。**自己位置の誤差を含む**
        self.stop_err: float | None = None
        #: 同じく推定位置（map フレーム）と停止点の距離 [m]。**止める制御の精度**
        #: （実機では停止点を SLAM の地図の上で置くので、こちらが「狙った所に止まれたか」）
        self.stop_err_est: float | None = None
        self.notes: list[str] = []
        #: 真値のゲートで計ったラップ [s]（先頭はスタンディングスタートを含む）
        self.gate_laps: list[float] = []
        self.race_vmax = 0.0                   #: RACE 中の真値の最高速度 [m/s]
        self.race_brake_s = 0.0                #: RACE 中に制動を出していた時間 [s]
        self.race_collisions = 0               #: RACE 中の衝突
        #: RACE 中の車体外形と真の壁との最小距離 [m]（周回ごと・全体）。衝突の手前の余裕
        self.race_min_clear = math.inf
        self.lap_min_clear: list[float] = []
        self.ideal_lap: float | None = None    #: 速度プロファイルどおりの1周 [s]
        self.path_len: float | None = None

    def report(self) -> str:
        def stat(v, unit, scale=1.0):
            if not v:
                return "—"
            a = [x * scale for x in v]
            return (f"平均 {statistics.fmean(a):.2f}{unit} "
                    f"中央 {statistics.median(a):.2f}{unit} "
                    f"最大 {max(a):.2f}{unit}")

        out = [
            f"計画       {self.plans} 回  1周期 {stat(self.plan_ms, 'ms')}",
            f"自己位置   {stat(self.pose_err, 'cm', 100)}",
            f"向き       {stat(self.yaw_err, '°')}",
            f"横偏差     {stat([abs(c) for c in self.cross], 'cm', 100)}",
            f"周回       {self.laps} 周  走行 {self.distance:.1f}m  衝突 {self.collisions}",
        ]
        if self.explore_end_pose_err is not None:
            out.append(
                f"EXPLORE終了時点  自己位置 {self.explore_end_pose_err * 100:.1f}cm  "
                f"向き {self.explore_end_yaw_err:.1f}°")
        if self.lap_times:
            out.append(
                "ラップ     " + " / ".join(f"{t:.1f}s" for t in self.lap_times))
        for i, (key, edges) in enumerate(self.lap_edges):
            out.append(f"周{i + 1:<2} 経路{key or '-'}  通ったエッジ {'→'.join(map(str, edges))}")
        if self.stop_err is not None:
            out.append(f"停止誤差   真値 {self.stop_err * 100:.1f}cm（自己位置の誤差を含む）・"
                       f"推定 {self.stop_err_est * 100:.1f}cm（止める制御の精度）")
        if self.gate_laps:
            fly = self.gate_laps[1:]
            best = f"最良 {min(fly):.2f}s 平均 {statistics.fmean(fly):.2f}s" if fly else "—"
            out.append("真値ラップ " + " / ".join(f"{t:.2f}s" for t in self.gate_laps)
                       + f"  （2周目以降 {best}）")
        if self.ideal_lap is not None:
            out.append(f"理想ラップ {self.ideal_lap:.2f}s（プロファイルどおり・"
                       f"1周 {self.path_len:.1f}m）")
        out.append(f"RACE       最高速 {self.race_vmax:.2f}m/s  制動 {self.race_brake_s:.1f}s  "
                   f"衝突 {self.race_collisions}  車体と壁の最小距離 "
                   f"{self.race_min_clear * 100:.1f}cm")
        if self.lap_min_clear:
            out.append("周ごとの最小距離 " + " / ".join(f"{c * 100:.1f}cm"
                                                      for c in self.lap_min_clear))
        out.extend(self.notes)
        out.append(f"最後の状態 [{self.phase}] {self.reason}")
        return "\n".join(out)


class Bench:
    def __init__(self, course: str | Path, mode: str, params: dict[str, float], *,
                 max_speed: float = 3.0, max_steer: float = 0.524,
                 model: str | None = None, quiet: bool = False, seed: int = 0,
                 routes: dict | None = None, events: list[tuple[float, str, str]] | None = None,
                 auto_race: bool = True, map_name: str | None = None,
                 trace: Path | None = None,
                 locate_hint: tuple[float, float] | None = None,
                 obstacles: list[tuple[float, float, float]] | None = None) -> None:
        """:param model: `reload_if_changed(name)`を持つplanner（今のところ`e2e_lidar`
            だけ）へ渡すモデル名。`raspi/nodes/planning_node.py`の`_apply_e2e_model()`
            と同じダックタイピング（`E2ELidar`をここでimportして特別扱いしない）。
            **これが無いと`--mode e2e_lidar`はモデル未ロードのまま何も動かない**
            （2026-08-29追加。GUI/`e2e/model`トピック経由でしかモデルを選べず、
            `sim.bench`単体では検証できなかった穴を埋める）
        """
        # 地図を作る planner は BUILD の終わりに地図を自動保存する。**実機の
        # `saved_maps/` を bench の地図で埋めない**よう、保存先を一時ディレクトリへ
        import tempfile

        import shutil

        from raspi.auto import mapstore
        self._real_maps_dir = mapstore.MAPS_DIR
        mapstore.MAPS_DIR = Path(tempfile.mkdtemp(prefix="bench_maps_"))
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
        self.link = create_sim_link(course, seed=seed, with_channel=False)
        self.sim = self.link.sim
        if obstacles:
            # 障害物はシムの世界（LiDAR と衝突判定が見る格子）にだけ刻む。`--map` と
            # 併用すると凍結地図には無い物が経路の上に居る状況になる
            from .track import stamp_discs
            c = self.sim.course
            stamp_discs(c.grid, c.origin, c.resolution, list(obstacles))
            c._padded_grid = None
        self.planner = make_planner(mode)
        if self.planner is None:
            raise SystemExit(f"知らないモード: {mode!r}（候補 {', '.join(PLANNERS)}）")
        if model:
            reload_fn = getattr(self.planner, "reload_if_changed", None)
            if reload_fn is None:
                raise SystemExit(f"{mode!r} はモデル選択に対応していない"
                                 f"（reload_if_changedを持たない）")
            reload_fn(model)
        self.mode = mode
        self.params = merged_params(mode, params)
        self.max_speed = max_speed
        self.max_steer = max_steer
        self.quiet = quiet

        self.asm = ScanAssembler()
        self.state = StateBuilder()
        self.vs = None
        self.res = Result()

        # 真値の起点。**SLAM の原点は「最初の姿勢」**なので、絶対座標ではなく
        # 起点からの移動量どうしを比べる（`test_nav.py` と同じ考え方）
        v = self.sim.vehicle
        self._origin = (v.x, v.y, v.yaw)
        self._last_lap_t = 0.0
        self._prev_laps = 0
        self._prev_phase = ""

        #: DONE（地図作成が終わって人の「走行」を待つ段）で、自動保存した地図を
        #: 読み込ませて走り出させる（GUI の「レーシングライン走行」を押すのと同じ）
        self.auto_race = auto_race
        #: `(時刻, 種類, 値)`。種類は "route"（`request_route`）/"signal"（`request_signal`）
        self.events = sorted(events or [])
        self._truth_log: list[tuple[str, int, str, float, float]] = []
        self._routes_world = routes
        if routes is not None:
            if getattr(self.planner, "request_routes", None) is None:
                raise SystemExit(f"{mode!r} は経路の設定（--routes）に対応していない")
            if not map_name:
                self.planner.request_routes(json.dumps(self._to_map_frame(routes)))
        #: `--map` の読み込みは最初の `plan()` の後に行う（`slam2d_route` は
        #: `plan()` で受けたパラメータで経路を作るため）
        self._load_pending = bool(map_name)
        self._locate_hint = locate_hint

        # 真値のゲート（モジュール docstring）
        self._gate: tuple[float, float, float, float] | None = None
        self._gate_prev: tuple[float, float, float] | None = None   # (t, 沿う距離, 横)
        self._gate_last_t: float | None = None
        self._gate_odom = 0.0
        self._race_col0: int | None = None
        self._trace_f = open(trace, "w") if trace else None
        if self._trace_f:
            self._trace_f.write("t,phase,x,y,yaw,v,cmd_v,brake,prof_v,idx,cross,clear\n")
        # 車体外形の輪郭と、真の壁からの距離場（`_clearance`）
        from scipy.ndimage import distance_transform_edt

        from .course import _outline
        c = self.sim.course
        self._edt = distance_transform_edt(~c.grid.astype(bool)) * c.resolution
        self._outline = _outline(np.asarray(self.sim.spec.footprint, dtype=np.float64),
                                 c.resolution / 2)
        self._lap_clear = math.inf

    # ── 座標 ──

    def _to_map(self, x: float, y: float, yaw: float | None = None):
        """シムの世界座標 → SLAM の map フレーム（起点が原点、起点の向きが +x）。"""
        ox, oy, oyaw = self._origin
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
        for name, st in (out.get("stops") or {}).items():
            mx, my, myaw = self._to_map(st["x"], st["y"], st.get("yaw"))
            st["x"], st["y"], st["yaw"] = mx, my, myaw
        return out

    # ── 真値 ──

    def _truth_rel(self) -> tuple[float, float, float]:
        """起点から見た真の姿勢（起点の車体座標系）。"""
        v = self.sim.vehicle
        ox, oy, oyaw = self._origin
        dx, dy = v.x - ox, v.y - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        return (dx * c - dy * s, dx * s + dy * c,
                (v.yaw - oyaw + math.pi) % (2 * math.pi) - math.pi)

    # ── 走行 ──

    def run(self, duration_s: float) -> Result:
        t0 = time.monotonic()
        next_cmd = time.monotonic_ns()
        cmd_period = NS // CMD_HZ
        cmd = DriveCmd(mode=0)
        last_plan_ns = 0

        while time.monotonic() - t0 < duration_s:
            for f in self.link.poll():
                msg = f.decode()
                if isinstance(msg, packets.Telemetry):
                    self.vs = self.state.build(msg, f.rx_ns)
                    continue
                if not isinstance(msg, (packets.LidarSector, packets.LidarSectorI,
                                        packets.LidarSectorC)):
                    continue
                scan = self.asm.feed(msg, f.rx_ns)
                if scan is None:
                    continue

                now = time.monotonic_ns()
                dt = (now - last_plan_ns) / NS if last_plan_ns else 0.0
                last_plan_ns = now
                t_plan = time.perf_counter()
                st = self.planner.plan(scan, self.vs, self.params, dt)
                self.res.plan_ms.append((time.perf_counter() - t_plan) * 1000)
                self.res.plans += 1
                t_now = time.monotonic() - t0
                self._record(st, t_now)
                self._step_events(st, t_now)

                # `planning_node._cmd_from` と同じ変換
                if not st.ready or st.brake:
                    cmd = DriveCmd(mode=2, arm=True, brake=True,
                                   target_steer=st.target_steer,
                                   brake_torque=st.brake_torque if st.ready else 0.0)
                else:
                    cmd = DriveCmd(mode=2, arm=True, target_speed=st.target_speed,
                                   target_steer=st.target_steer,
                                   accel_limit=st.accel_limit)

            now = time.monotonic_ns()
            if now >= next_cmd:
                next_cmd = now + cmd_period
                self.link.send(command_from_cmd(
                    cmd, allow_arm=True,
                    max_speed=self.max_speed, max_steer=self.max_steer))
                self._sample(time.monotonic() - t0, cmd, cmd_period / NS)

        self.res.collisions = self.sim.vehicle.collisions
        if self._race_col0 is not None:
            self.res.race_collisions = self.sim.vehicle.collisions - self._race_col0
        self.res.distance = float(self.sim.vehicle.odom_front[0])
        path = getattr(self.planner, "path", None)
        if path is not None and getattr(path, "closed", False) and len(path) >= 3:
            ds = np.hypot(*(np.roll(path.xy, -1, axis=0) - path.xy).T)
            v = np.maximum(path.v, 1e-3)
            self.res.ideal_lap = float(np.sum(2.0 * ds / (v + np.roll(v, -1))))
            self.res.path_len = float(path.length)
        if self._trace_f:
            self._trace_f.close()
        return self.res

    def _sample(self, t: float, cmd: DriveCmd, dt: float) -> None:
        """指令周期（100Hz）で真値を見る: ゲートのラップ・最高速・制動・記録。"""
        st = getattr(self, "_last_st", None)
        if st is None or st.phase != "RACE":
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
        if self._trace_f:
            path = getattr(self.planner, "path", None)
            i = getattr(self.planner, "_hint", -1)
            pv = float(path.v[i]) if path is not None and 0 <= i < len(path) else float("nan")
            self._trace_f.write(
                f"{t:.3f},{st.phase},{v.x:.4f},{v.y:.4f},{v.yaw:.4f},{v.speed:.4f},"
                f"{cmd.target_speed:.3f},{int(cmd.brake)},{pv:.3f},{i},{st.cross_track:.4f},"
                f"{clear:.4f}\n")

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

    def _step_events(self, st, t: float) -> None:
        if self._load_pending:
            self._load_pending = False
            if self._routes_world is not None:
                # 経路の設定は地図と一緒に読ませる（読み込み後に渡すと、道路グラフが
                # できる前なので経由点のグループが作られない）
                from raspi.auto import mapstore
                cfg = self._to_map_frame(self._routes_world)
                # 地図作成の軌跡（自動経路のスタートと逆走禁止の元）は地図側のものを残す
                # （GUI の保存 `request_routes` と同じ。消すと自動経路が保存した線をなぞるだけになる）
                try:
                    old = json.loads(mapstore.load_routes(self._map_name) or "{}")
                except ValueError:
                    old = {}
                if "explore_traj" in old and "explore_traj" not in cfg:
                    cfg["explore_traj"] = old["explore_traj"]
                mapstore.save_routes(self._map_name, json.dumps(cfg))
            self.planner.request_load(self._map_name)
            err = getattr(self.planner, "_load_error", "")
            if err:
                raise SystemExit(err)
            if self._locate_hint is not None:
                self.planner.request_locate_hint(*self._locate_hint)
            if not self.quiet:
                print(f"  {t:5.1f}s 保存済み地図『{self._map_name}』を読み込んだ", flush=True)
            return
        if (self.auto_race and st.phase == "DONE"
                and getattr(self.planner, "_saved_map_name", "")):
            name = self.planner._saved_map_name
            if not self.quiet:
                print(f"  {t:5.1f}s 地図『{name}』で走行を開始する（bench が押す）", flush=True)
            self.planner.request_load(name)
        while self.events and self.events[0][0] <= t:
            _, kind, value = self.events.pop(0)
            fn = getattr(self.planner, "request_route" if kind == "route" else "request_signal",
                         None)
            if fn is not None:
                fn(value, "bench")
                self.res.notes.append(f"{t:5.1f}s {kind} → {value}")
                if not self.quiet:
                    print(f"  {t:5.1f}s 切替要求 {kind}={value}", flush=True)

    def _record(self, st, t: float) -> None:
        self._last_st = st
        v = self.sim.vehicle
        self._truth_log.append((st.phase, st.laps, getattr(st, "route_active", ""), v.x, v.y))
        if not self._map_name:
            # 保存済み地図の map フレームは bench の起点と無関係なので比べられない
            tx, ty, tyaw = self._truth_rel()
            self.res.pose_err.append(math.hypot(st.pose_x - tx, st.pose_y - ty))
            d = (st.pose_yaw - tyaw + math.pi) % (2 * math.pi) - math.pi
            self.res.yaw_err.append(abs(math.degrees(d)))
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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="normal", help="コース名 or パス")
    ap.add_argument("--mode", default="slam2d_raceline", choices=list(PLANNERS))
    ap.add_argument("--time", type=float, default=180.0, help="走らせる秒数")
    ap.add_argument("--seed", type=int, default=0, help="シムの乱数シード（LiDARノイズ等）")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="planner のパラメータを上書き（複数可）")
    ap.add_argument("--max-speed", type=float, default=3.0)
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
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    course = Path(args.course)
    if not course.exists():
        found = [p for p in list_courses() if p.stem == args.course]
        if not found:
            names = ", ".join(sorted({p.stem for p in list_courses()}))
            print(f"コースが無い: {args.course}（候補 {names}）", file=sys.stderr)
            return 2
        course = found[0]

    params: dict[str, float] = {}
    for kv in args.set:
        k, _, v = kv.partition("=")
        params[k.strip()] = float(v)

    routes = json.loads(Path(args.routes).read_text()) if args.routes else None
    events: list[tuple[float, str, str]] = []
    for kind, items in (("route", args.switch), ("signal", args.signal)):
        for it in items:
            t, _, v = it.partition(":")
            events.append((float(t), kind, v))

    b = Bench(course, args.mode, params, max_speed=args.max_speed,
              max_steer=args.max_steer, model=args.model, quiet=args.quiet, seed=args.seed,
              routes=routes, events=events, auto_race=not args.no_auto_race,
              map_name=args.map, trace=Path(args.trace) if args.trace else None,
              locate_hint=(tuple(float(v) for v in args.locate_hint.split(","))
                           if args.locate_hint else None),
              obstacles=[tuple(float(v) for v in o.split(",")) for o in args.obstacle])
    print(f"# {course.name} / {args.mode} / {args.time:.0f}s / seed={args.seed}")
    try:
        res = b.run(args.time)
    except KeyboardInterrupt:
        res = b.res
    quality = b.map_quality()
    b.route_report()
    if args.save_map:
        res.notes.append(b.save_built_map(args.save_map))
    b.close()
    print("\n=== 結果 ===")
    print(res.report())
    print("地図       " + quality)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
