"""planner をシムの真値と突き合わせて評価する。**バスも GUI もプロセスも使わない。**

    .venv/bin/python -m sim.bench --course circuit
    .venv/bin/python -m sim.bench --course courses/1.json --mode raceline --time 240
    .venv/bin/python -m sim.bench --mode ftg --set explore_speed=0.6

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
        if self.lap_times:
            out.append(
                "ラップ     " + " / ".join(f"{t:.1f}s" for t in self.lap_times))
        out.append(f"最後の状態 [{self.phase}] {self.reason}")
        return "\n".join(out)


class Bench:
    def __init__(self, course: str | Path, mode: str, params: dict[str, float], *,
                 max_speed: float = 3.0, max_steer: float = 0.524,
                 model: str | None = None, quiet: bool = False) -> None:
        """:param model: `reload_if_changed(name)`を持つplanner（今のところ`e2e_lidar`
            だけ）へ渡すモデル名。`raspi/nodes/planning_node.py`の`_apply_e2e_model()`
            と同じダックタイピング（`E2ELidar`をここでimportして特別扱いしない）。
            **これが無いと`--mode e2e_lidar`はモデル未ロードのまま何も動かない**
            （2026-08-29追加。GUI/`e2e/model`トピック経由でしかモデルを選べず、
            `sim.bench`単体では検証できなかった穴を埋める）
        """
        self.link = create_sim_link(course, with_channel=False)
        self.sim = self.link.sim
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
                self._record(st, time.monotonic() - t0)

                if not st.ready or st.brake:
                    cmd = DriveCmd(mode=2, arm=True, brake=True,
                                   target_steer=st.target_steer)
                else:
                    cmd = DriveCmd(mode=2, arm=True, target_speed=st.target_speed,
                                   target_steer=st.target_steer)

            now = time.monotonic_ns()
            if now >= next_cmd:
                next_cmd = now + cmd_period
                self.link.send(command_from_cmd(
                    cmd, allow_arm=True,
                    max_speed=self.max_speed, max_steer=self.max_steer))

        self.res.collisions = self.sim.vehicle.collisions
        self.res.distance = float(self.sim.vehicle.odom_front[0])
        return self.res

    def _record(self, st, t: float) -> None:
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
        self.res.laps = st.laps
        self.res.phase = st.phase
        self.res.reason = st.reason

        if not self.quiet and self.res.plans % 20 == 0:
            print(f"  {t:5.1f}s [{st.phase or '-'}] 誤差 "
                  f"{self.res.pose_err[-1] * 100:5.1f}cm "
                  f"{self.res.yaw_err[-1]:4.1f}°  一致度 {st.match_score:.2f}  "
                  f"{st.reason}", flush=True)

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

    def close(self) -> None:
        self.link.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", default="circuit", help="コース名 or パス")
    ap.add_argument("--mode", default="raceline", choices=list(PLANNERS))
    ap.add_argument("--time", type=float, default=180.0, help="走らせる秒数")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="planner のパラメータを上書き（複数可）")
    ap.add_argument("--max-speed", type=float, default=3.0)
    ap.add_argument("--max-steer", type=float, default=0.524)
    ap.add_argument("--model", default=None,
                    help="reload_if_changedに対応するplanner（e2e_lidar）へ渡すモデル名"
                         "（例: v2 → models/e2e_lidar/v2.onnx）。他のモードでは無視される")
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

    b = Bench(course, args.mode, params, max_speed=args.max_speed,
              max_steer=args.max_steer, model=args.model, quiet=args.quiet)
    print(f"# {course.name} / {args.mode} / {args.time:.0f}s")
    try:
        res = b.run(args.time)
    except KeyboardInterrupt:
        res = b.res
    quality = b.map_quality()
    b.close()
    print("\n=== 結果 ===")
    print(res.report())
    print("地図       " + quality)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
