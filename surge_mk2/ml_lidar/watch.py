"""ml_lidar/watch.py — 学習済みモデルの観戦（可視化のみ・学習には一切関与しない）。

    .venv/bin/python -m ml_lidar.watch --model v1                          # ランダムコース
    .venv/bin/python -m ml_lidar.watch --model v1 --course sim/courses/course1.json

実車側の推論コード（`raspi/auto/e2e_lidar.py`の`E2ELidar`）をそのまま呼び出して走らせる
——学習ループ用の`ml_lidar/env.py`の正規化済みaction空間を経由せず、実車と全く同じ
`plan()`の入出力契約で駆動することで、「今まさに実車に載せるコード」の挙動を目視できる。

`sim/courses/`の自作コース（`--course`で指定）は学習プロセスに一切関与しない確認専用。指定しなければ`course_gen`のランダムコースを使う。

## 最高速度（`--max-speed`）

実機のGUIの`E2ELidar`「最高速度」既定は1.0m/sで、学習時の`max_speed`（2.0）より低い
＝**学習分布の外**。省略時はこの実機既定のまま走らせる（実機で触らないときの挙動を見る）。
`--max-speed model`でモデルの契約JSONが持つ学習時の値、`--max-speed 1.5`のように数値で
上書きもできる。学習どおりの走りを見たいときは`model`、実機の見え方を確かめたいときは省略。

オレンジの破線は理想ライン（MCL、`sim/raceline.py`の`compute_raceline_xy`）。
学習には一切関与しない診断用オーバーレイで、どのモデルでも常に描く——
「このモデルが理想ラインからどれだけ離れているか」を目視する比較材料。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ML_LIDAR_DIR = Path(__file__).resolve().parent
REPO_ROOT = ML_LIDAR_DIR.parent
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402

from raspi.auto.e2e_lidar import E2ELidar  # noqa: E402
from raspi.msgs.convert import ScanAssembler  # noqa: E402
from raspi.msgs.types import Scan, VehicleState  # noqa: E402
from sim.course import Course  # noqa: E402
from sim.lidar import VirtualLidar  # noqa: E402
from sim.params import SimParams  # noqa: E402
from sim.raceline import compute_raceline_xy  # noqa: E402
from sim.vehicle import DriveInput, VehicleModel, VehicleSpec  # noqa: E402

from ml_lidar.course_gen import random_walk_loop_course  # noqa: E402
from ml_lidar.env import EnvConfig  # noqa: E402
from ml_lidar.sim_support import pump_lidar_until_scan, stm_us  # noqa: E402
from ml_lidar.viz_support import catchup_step_count, set_japanese_font  # noqa: E402

__all__ = ["Watcher", "parse_args", "main"]

NS = 1_000_000_000
DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "e2e_lidar"


class Watcher:
    """`E2ELidar`を実車と同じ`plan()`契約で駆動する、可視化専用の最小シム。

    `ml_lidar/env.py`とほぼ同じ骨格（`VirtualLidar`＋`ScanAssembler`のLiDAR中間案）
    だが、行動は正規化済みaction空間ではなく`E2ELidar.plan()`が返す物理量
    （`target_steer`/`target_speed`/`brake`）をそのまま`VehicleModel`に渡す。
    """

    def __init__(self, model_path: Path, course: Course, *, dt: float = EnvConfig().dt,
                physics_substep: float = 0.005, params: SimParams | None = None,
                max_speed: float | str | None = None) -> None:
        """:param max_speed: `E2ELidar`の「最高速度」パラメータ（GUIのスライダ相当）。
            `None`＝GUIの既定値（実機で何も触らないときと同じ。学習時の`max_speed`より
            低いことが多く、学習分布の外になる）、数値＝その値、`"model"`＝モデルの契約JSONが
            持つ学習時の`max_speed`。実機の挙動を見たいときは`None`、学習どおりの走りを
            見たいときは`"model"`
        """
        self.dt = dt
        self.physics_substep = physics_substep
        self.course = course
        spec = VehicleSpec.load()
        self.vehicle = VehicleModel(spec, course.start)
        self.body = course.body_samples(spec.footprint)
        self.lidar = VirtualLidar(course, spec, params or SimParams(), seed=0)
        self.assembler = ScanAssembler()
        self.scan: Scan | None = None
        self.t_ns = 0
        self.planner = E2ELidar(model_path=model_path)
        self.planner.reset()
        if max_speed == "model":
            self._plan_overrides = {"max_speed": self.planner._model_max_speed}
        elif max_speed is None:
            self._plan_overrides = {}
        else:
            self._plan_overrides = {"max_speed": float(max_speed)}
        self._pump_lidar()

    def _pump_lidar(self) -> None:
        sub_ns = int(round(self.physics_substep * NS))

        def _step():
            self.t_ns += sub_ns
            self._poll()
            return self.scan

        pump_lidar_until_scan(_step, self.physics_substep)

    def _poll(self) -> None:
        for gen_ns, pkt in self.lidar.poll(self.t_ns, self.vehicle, stm_us):
            scan = self.assembler.feed(pkt, gen_ns)
            if scan is not None:
                self.scan = scan

    def step(self) -> dict:
        """1制御周期ぶん進める。`E2ELidar.plan()`の戻り値をそのまま反映する。"""
        vs = VehicleState(speed=self.vehicle.speed_measured, steer_actual=self.vehicle.steer_actual)
        params = E2ELidar.merged(self._plan_overrides)
        state = self.planner.plan(self.scan, vs, params, self.dt)

        sp = self.vehicle.spec
        # 実機のCOMMANDと同じくGUIのレート制限を載せ、制御遅延ぶん前の指令を効かせる
        # （`ml_lidar/env.py`の`step()`と同じ扱い）
        cmd = DriveInput(armed=True, brake=state.brake,
                         target_speed=0.0 if state.brake else state.target_speed,
                         target_steer=state.target_steer,
                         accel_limit=sp.cmd_accel_limit_m_s2,
                         steer_rate_limit=sp.cmd_steer_rate_limit_rad_s)

        n_sub = max(1, int(round(self.dt / self.physics_substep)))
        sub_dt = self.dt / n_sub
        n_delay = min(n_sub, int(round(sp.control_latency_s / sub_dt)))
        prev_cmd = getattr(self, "_applied_cmd", None) or cmd
        self._applied_cmd = cmd
        collided = False
        for i in range(n_sub):
            self.vehicle.apply(prev_cmd if i < n_delay else cmd)
            self.vehicle.step(sub_dt)
            self.t_ns += int(round(sub_dt * NS))
            hit = self.course.collides(self.vehicle.x, self.vehicle.y, self.vehicle.yaw, self.body)
            self.vehicle.note_collision(hit)
            self._poll()
            if hit:
                collided = True
                break

        return {"collided": collided, "ready": state.ready, "reason": state.reason,
               "target_speed": state.target_speed, "target_steer": state.target_steer}


def _load_course(args: argparse.Namespace) -> Course:
    if args.course:
        return Course.load(args.course)
    rng = np.random.default_rng(args.seed)
    return random_walk_loop_course(rng)


def run(args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Polygon

    set_japanese_font()
    plt.style.use("dark_background")

    course = _load_course(args)
    model_path = args.models_dir / f"{args.model}.onnx"
    watcher = Watcher(model_path, course, dt=args.dt, max_speed=_parse_max_speed(args.max_speed))
    footprint = np.asarray(VehicleSpec.load().footprint)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    h, w = course.grid.shape
    extent = (course.origin[0], course.origin[0] + w * course.resolution,
             course.origin[1], course.origin[1] + h * course.resolution)
    ax.imshow(course.grid, extent=extent, origin="lower", cmap="Greys",
             vmin=0, vmax=1, alpha=0.6)

    # 理想ライン(MCL、道幅内で曲率二乗和を最小化した参照軌道)を薄く重ねる。
    # 学習には一切関与しない診断用オーバーレイなので、報酬に理想ラインを使わない
    # 現行の設定(v21以降)でも常に描く——「このモデルが理想ラインからどれだけ
    # 離れているか」を目視する比較材料として有益なため
    if course.centerline is not None:
        vehicle_half_width_m = max(abs(p[1]) for p in VehicleSpec.load().footprint)
        raceline_xy = compute_raceline_xy(course.centerline, course.width,
                                          vehicle_half_width_m=vehicle_half_width_m,
                                          course=course)
        closed = np.vstack([raceline_xy, raceline_xy[:1]])
        ax.plot(closed[:, 0], closed[:, 1], color="tab:orange", linewidth=1.2,
               linestyle="--", zorder=3, label="理想ライン(MCL)")
        ax.legend(loc="upper right", fontsize=8)

    body_poly = Polygon(np.zeros((4, 2)), closed=True, facecolor="tab:blue",
                        edgecolor="white", zorder=5)
    ax.add_patch(body_poly)
    lidar_scatter = ax.scatter([], [], s=3, c="tab:red", zorder=4)
    title = ax.set_title("")

    # 直前フレームからの実経過時間ぶんだけシムを進める（fixed-timestep-with-catchup）。
    # 「毎フレーム必ず1step」だと、描画・CPU競合でフレーム間隔が伸びた瞬間だけ
    # 見かけの速度が遅くなり、次に間隔が戻ると急に追いつく——これが「かくつく」の正体
    last_time = [time.monotonic()]

    def _update(_frame):
        now = time.monotonic()
        real_dt = now - last_time[0]
        last_time[0] = now
        n_steps = catchup_step_count(real_dt, args.dt)

        info = watcher.step()
        for _ in range(n_steps - 1):
            if info["collided"]:
                break
            info = watcher.step()

        v = watcher.vehicle
        c, s = np.cos(v.yaw), np.sin(v.yaw)
        rot = np.array([[c, -s], [s, c]])
        body_poly.set_xy(footprint @ rot.T + np.array([v.x, v.y]))

        if watcher.scan is not None:
            pts = []
            for deg in range(0, 360, 2):
                d = watcher.scan.dist[deg]
                if d > 0:
                    a = v.yaw + np.radians(deg)
                    pts.append((v.x + d * np.cos(a), v.y + d * np.sin(a)))
            lidar_scatter.set_offsets(np.asarray(pts) if pts else np.empty((0, 2)))

        status = "衝突" if info["collided"] else ("走行中" if info["ready"] else "待機")
        title.set_text(f"{status} | v={v.speed:.2f}m/s | {info['reason']}")
        if info["collided"]:
            anim.event_source.stop()
        return body_poly, lidar_scatter, title

    anim = FuncAnimation(fig, _update, interval=max(1, int(args.dt * 1000)), blit=False)
    plt.show()


def _parse_max_speed(value: str | None) -> float | str | None:
    """`--max-speed`の値。`None`・`"model"`はそのまま、数値は`float`にする。"""
    if value is None or value == "model":
        return value
    return float(value)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ml_lidar 観戦（実車と同じplan()契約で駆動）")
    p.add_argument("--model", required=True, help="`models/e2e_lidar/<name>.onnx`の<name>")
    p.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    p.add_argument("--course", type=Path, default=None,
                   help="未指定なら`course_gen`のランダムコース。指定するなら`sim/courses/*.json`")
    p.add_argument("--seed", type=int, default=0, help="ランダムコース時のみ使う")
    # 既定は`EnvConfig.dt`に追従させる（観戦だけ別の周期で回すと、
    # 実車・学習と違う応答を見て判断してしまう）
    p.add_argument("--dt", type=float, default=EnvConfig().dt)
    p.add_argument("--max-speed", default=None,
                   help="E2ELidarの最高速度[m/s]。省略＝GUIの既定値（実機で触らないときと同じ。"
                        "学習時のmax_speedより低く、学習分布の外）。`model`＝モデルの契約JSONの"
                        "学習時の値。数値ならその値")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    run(parse_args(argv))


if __name__ == "__main__":
    main()
