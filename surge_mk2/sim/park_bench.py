"""`park_to_point`（LiDARクリック指定の自律駐車）の評価ベンチ。

    .venv/bin/python -m sim.park_bench --scenario all --n 10
    .venv/bin/python -m sim.park_bench --scenario garage --n 20 --errors real
    .venv/bin/python -m sim.park_bench --scenario parallel --n 4 --plot

## 旧版の致命的な欠陥（2026-09-16に判明）

旧`park_bench.py`は`Scan`を**ループの外で1回だけ**作って毎周期同じものを
plannerへ渡していた。障害物は車体ローカル座標に固定されるので**車と一緒に
動き**、近づくことも通り過ぎることもない。「障害物あり全パターン通過」という
検証結果は、実質「障害物が存在しない場合の検証」だった。さらに:

- **衝突を一度も数えていなかった**。plannerが「完了」と言えば成功で、
  途中で壁を擦っていても分からない
- `ScanAssembler`を迂回していた（鏡像戻し・セクタ欠損・ノイズ・遅延が入らない）
- `VehicleModel`は`yaw_rate`・オドメトリを**ノイズもバイアスもゼロ**で出すので、
  デッドレコニングのドリフトという最大の脅威が測れていなかった

## 今の構成

    park_world（駐車配置をCourseとして生成）
      → VirtualLidar（毎周期レイキャスト・セクタ単位・ノイズ・欠損・遅延）
      → ScanAssembler（実機と同じ鏡像戻し・欠測セクタの扱い）
      → ParkToPoint.plan()
      → VehicleModel（操舵むだ時間・1次遅れ・レート制限・グリップ限界）
      → Course.collides()で**真値の衝突判定**、clearance_field()で**真値の余裕**

センサ誤差（`SensorErrors`）はここで注入する。`VehicleModel`本体には手を
入れない——シミュレータの物理モデルは「真値」を持つ役で、センサの不完全さを
混ぜると他のベンチ（`sim.bench`・RL学習）の意味が変わってしまうため。

## 成否は「plannerの申告」ではなく「真値」で決める

`ok`の条件は **phase=="完了" かつ 無衝突 かつ 真値の姿勢誤差が許容内**。
デッドレコニングがドリフトすると planner は誤った場所で「完了」と言うので、
申告を信じた評価では品質が測れない（これが旧版の最大の問題だった）。
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from raspi.auto.park_to_point import ParkToPoint  # noqa: E402
from raspi.msgs import SPEED_DEADBAND_MPS  # noqa: E402
from raspi.msgs.convert import ScanAssembler  # noqa: E402
from raspi.msgs.types import VehicleState  # noqa: E402

from . import park_world as W  # noqa: E402
from .lidar import VirtualLidar  # noqa: E402
from .params import SimParams  # noqa: E402
from .vehicle import DriveInput, VehicleModel, VehicleSpec  # noqa: E402

__all__ = ["SensorErrors", "RunResult", "run_one", "run_batch"]

DT = 0.02          #: 車両モデルの積分刻み [s]


@dataclass
class SensorErrors:
    """デッドレコニングを狂わせる誤差。**実機で必ず存在するもの。**

    `VehicleModel`は真値を出すので、ここで意図的に汚してから
    `VehicleState`へ詰める。`park_to_point`は`yaw_rate`の積分と
    `odom_center`の差分だけで姿勢を推定するため、この2つが効く。
    """

    #: ジャイロのゼロ点ずれ [°/s]。**駐車の絶対精度を決める最大の要因**。
    #: 40s動作なら0.5°/sで20°ずれる。MPU6050の温度ドリフトはこの程度出る
    gyro_bias_dps: float = 0.0
    gyro_noise_dps: float = 0.0            #: ジャイロの白色ノイズ 1σ [°/s]
    #: オドメトリのスケール誤差（1.0=誤差なし）。車輪半径の誤差・摩耗・
    #: 空気圧相当の変化。前輪アナログ絶対角エンコーダなのでスリップより
    #: スケールの方が支配的
    odom_scale: float = 1.0
    #: エンコーダ量子化 [m]。`odom_dist`はi32・0.1mm/LSBで送られる
    #: （`raspi/proto/protocol.toml`）
    odom_tick_m: float = 1e-4

    @classmethod
    def preset(cls, name: str) -> "SensorErrors":
        if name == "none":
            return cls()
        if name == "real":
            # ★実測値ではない暫定値。MPU6050の据え置きバイアスと、
            # 車輪半径30mmに対する±0.6mm相当のスケール誤差を見込んだもの
            return cls(gyro_bias_dps=0.5, gyro_noise_dps=0.3,
                       odom_scale=1.02, odom_tick_m=1e-4)
        if name == "harsh":
            return cls(gyro_bias_dps=1.5, gyro_noise_dps=0.8,
                       odom_scale=1.05, odom_tick_m=1e-3)
        raise ValueError(f"unknown preset: {name}")


@dataclass
class RunResult:
    scenario: str
    ok: bool                       #: 真値で判定した成否（下記3条件すべて）
    reached: bool                  #: 真値の姿勢誤差が許容内か
    no_collision: bool
    phase: str                     #: plannerの最終phase
    t: float                       #: 所要時間 [s]
    switches: int                  #: 前後進の切り替え回数
    collisions: int                #: 衝突に入った回数（真値）
    min_clear: float               #: 走行中の最小クリアランス [m]（真値）
    err_pos: float                 #: 真値の位置誤差 [m]
    err_yaw_deg: float             #: 真値の向き誤差 [°]
    believed_pos: float            #: plannerが信じていた残り距離 [m]
    #: **自己位置推定そのものの誤差**（planner の推定 vs 真値）[m] / [°]。
    #: 成否は5cmの許容誤差の境界でばたつくので、推定の良し悪しはこちらで測る
    est_err_pos: float
    est_err_yaw_deg: float
    est_err_pos_max: float
    final_pose: tuple[float, float, float]
    params: dict
    log: list[dict]


def _stm_us(t_ns: int) -> int:
    return (t_ns // 1000) & 0xFFFFFFFF


def _world_to_local(pose: tuple[float, float, float],
                    target: tuple[float, float, float]) -> tuple[float, float, float]:
    """`target`（世界座標）を`pose`基準のローカル座標へ。GUIのクリックと同じ形。"""
    px, py, pyaw = pose
    c, s = math.cos(-pyaw), math.sin(-pyaw)
    dx, dy = target[0] - px, target[1] - py
    return (c * dx - s * dy, s * dx + c * dy,
            (target[2] - pyaw + math.pi) % (2 * math.pi) - math.pi)


def run_one(sc: W.ParkScenario, *, errors: SensorErrors | None = None,
            max_time: float = 45.0, params: dict[str, float] | None = None,
            seed: int = 0) -> RunResult:
    """1配置ぶん走らせる。

    `sc.target`（世界座標）を開始姿勢基準のローカル座標へ直して
    `request_park_target()`へ渡す——**GUIのクリックと同じ情報量**にするため。
    以降 planner は世界座標を一切知らない。
    """
    err = errors or SensorErrors()
    spec = VehicleSpec.load()
    sim_p = SimParams()
    veh = VehicleModel(spec, start=sc.start)
    lidar = VirtualLidar(sc.course, spec, sim_p, seed=seed)
    asm = ScanAssembler()
    rng = np.random.default_rng(seed)

    pp = ParkToPoint()
    p = pp.merged(params or {})
    pp.request_park_target(*_world_to_local(sc.start, sc.target))

    body = sc.course.body_samples(W.footprint())
    field = W.clearance_field(sc.course)

    t = 0.0
    t_ns = 0
    queue: list[tuple[int, object]] = []
    odom_accum = 0.0
    prev_odom_front = veh.odom_front[0]
    yaw_bias = math.radians(err.gyro_bias_dps)
    yaw_sigma = math.radians(err.gyro_noise_dps)

    last_speed_cmd = 0.0
    last_steer_cmd = 0.0
    prev_reverse: bool | None = None
    switches = 0
    collisions = 0
    was_colliding = False
    steps = 0
    min_clear = math.inf
    est_last = (0.0, 0.0)
    est_max = 0.0
    t_last_plan = 0.0
    st = None
    log: list[dict] = []

    while t < max_time:
        # ── LiDAR（セクタ単位・伝送遅延つき） ──
        for gen_ns, pkt in lidar.poll(t_ns, veh, _stm_us):
            queue.append((gen_ns, pkt))
        scan = None
        keep: list[tuple[int, object]] = []
        for gen_ns, pkt in queue:
            if gen_ns <= t_ns:
                done = asm.feed(pkt, gen_ns)
                if done is not None:
                    scan = done
            else:
                keep.append((gen_ns, pkt))
        queue = keep

        # ── 1周そろったら plan()（実機と同じ 10Hz 相当） ──
        if scan is not None:
            dt_plan = t - t_last_plan
            t_last_plan = t
            d_front_true = veh.odom_front[0] - prev_odom_front
            prev_odom_front = veh.odom_front[0]
            # スケール誤差 → 量子化（累積値をLSBに丸める、実機と同じ順序）
            odom_accum += d_front_true * err.odom_scale * math.cos(veh.steer_actual)
            odom_q = (round(odom_accum / err.odom_tick_m) * err.odom_tick_m
                      if err.odom_tick_m > 0 else odom_accum)
            yaw_rate_meas = veh.yaw_rate + yaw_bias + (
                float(rng.normal(0.0, yaw_sigma)) if yaw_sigma > 0 else 0.0)
            speed_meas = veh.speed
            vs = VehicleState(speed=speed_meas, yaw_rate=yaw_rate_meas,
                              steer_actual=veh.steer_actual, odom_center=odom_q,
                              stopped=abs(speed_meas) < SPEED_DEADBAND_MPS)
            if dt_plan > 0:
                st = pp.plan(scan, vs, p, dt_plan)
                # planner の推定姿勢（基準フレーム＝開始姿勢）と真値の差
                if pp._park_pose is not None:
                    tx, ty, tyaw = _world_to_local(sc.start, (veh.x, veh.y, veh.yaw))
                    est_err = math.hypot(pp._park_pose[0] - tx, pp._park_pose[1] - ty)
                    est_err_yaw = abs((pp._park_pose[2] - tyaw + math.pi)
                                      % (2 * math.pi) - math.pi)
                    est_last = (est_err, math.degrees(est_err_yaw))
                    est_max = max(est_max, est_err)
                if prev_reverse is not None and st.park_reverse != prev_reverse:
                    switches += 1
                prev_reverse = st.park_reverse
                last_speed_cmd = 0.0 if st.brake else st.target_speed
                last_steer_cmd = st.target_steer
                log.append(dict(t=t, x=veh.x, y=veh.y, yaw=math.degrees(veh.yaw),
                                rho=st.park_rho, v=last_speed_cmd,
                                steer=math.degrees(last_steer_cmd),
                                phase=st.phase, reverse=st.park_reverse,
                                reason=st.reason))
                if st.phase in ("完了", "失敗"):
                    break

        veh.apply(DriveInput(armed=True, brake=(last_speed_cmd == 0.0),
                             target_speed=last_speed_cmd, target_steer=last_steer_cmd))
        veh.step(DT)
        t += DT
        t_ns += int(DT * 1e9)

        # ── 真値の衝突とクリアランス ──
        # 衝突は毎ステップ見る（見落とすと評価の意味が無い）が、クリアランスは
        # 数千点×2000ステップで割に合わないので5ステップ(0.1m/s換算で1cm)ごと
        hit = sc.course.collides(veh.x, veh.y, veh.yaw, body)
        if hit and not was_colliding:
            collisions += 1
        was_colliding = hit
        steps += 1
        if hit:
            min_clear = 0.0
        elif steps % 5 == 0:
            min_clear = min(min_clear, W.body_clearance(
                sc.course, field, veh.x, veh.y, veh.yaw, body))

    err_pos = math.hypot(veh.x - sc.target[0], veh.y - sc.target[1])
    err_yaw = abs((veh.yaw - sc.target[2] + math.pi) % (2 * math.pi) - math.pi)
    reached = err_pos <= p["pos_tol_m"] and math.degrees(err_yaw) <= p["yaw_tol_deg"]
    phase = st.phase if st is not None else "no-plan"
    return RunResult(
        scenario=sc.name, ok=(phase == "完了" and collisions == 0 and reached),
        reached=reached, no_collision=(collisions == 0), phase=phase, t=t,
        switches=switches, collisions=collisions,
        min_clear=(0.0 if min_clear is math.inf else min_clear),
        err_pos=err_pos, err_yaw_deg=math.degrees(err_yaw),
        believed_pos=(st.park_rho if st is not None else 0.0),
        est_err_pos=est_last[0], est_err_yaw_deg=est_last[1],
        est_err_pos_max=est_max,
        final_pose=(veh.x, veh.y, veh.yaw), params=sc.params, log=log)


def run_batch(names: list[str], n: int, *, seed: int = 0,
              errors: SensorErrors | None = None, max_time: float = 45.0,
              params: dict[str, float] | None = None) -> list[tuple[W.ParkScenario, RunResult]]:
    out = []
    for name in names:
        rng = random.Random(seed)
        for i in range(n):
            sc = W.make(name, rng)
            out.append((sc, run_one(sc, errors=errors, max_time=max_time,
                                    params=params, seed=seed * 1000 + i)))
    return out


def _summary(results: list[tuple[W.ParkScenario, RunResult]]) -> None:
    by: dict[str, list[RunResult]] = {}
    for _, r in results:
        by.setdefault(r.scenario, []).append(r)
    print(f"\n{'配置':10s} {'成功':>7s} {'到達':>7s} {'無衝突':>7s} "
          f"{'最小余裕':>8s} {'位置誤差':>8s} {'向き誤差':>8s} "
          f"{'推定誤差':>8s} {'推定向き':>8s} {'推定最大':>8s} {'切返':>4s} {'時間':>6s}")
    for name, rs in by.items():
        n = len(rs)
        print(f"{name:10s} {sum(r.ok for r in rs):3d}/{n:<3d} "
              f"{sum(r.reached for r in rs):3d}/{n:<3d} "
              f"{sum(r.no_collision for r in rs):3d}/{n:<3d} "
              f"{np.mean([r.min_clear for r in rs]) * 100:7.1f}cm "
              f"{np.mean([r.err_pos for r in rs]) * 100:7.1f}cm "
              f"{np.mean([r.err_yaw_deg for r in rs]):7.1f}° "
              f"{np.mean([r.est_err_pos for r in rs]) * 100:7.1f}cm "
              f"{np.mean([r.est_err_yaw_deg for r in rs]):7.1f}° "
              f"{np.mean([r.est_err_pos_max for r in rs]) * 100:7.1f}cm "
              f"{np.mean([r.switches for r in rs]):4.1f} "
              f"{np.mean([r.t for r in rs]):5.1f}s")
    all_r = [r for _, r in results]
    print(f"{'合計':10s} {sum(r.ok for r in all_r):3d}/{len(all_r):<3d}")


def _plot(results: list[tuple[W.ParkScenario, RunResult]], out: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib未インストールのため--plotはスキップ)")
        return
    n = len(results)
    cols = min(4, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 4 * rows), squeeze=False)
    for i, (sc, r) in enumerate(results):
        ax = axes[i // cols][i % cols]
        ax.imshow(sc.course.grid, origin="lower", cmap="Greys",
                  extent=(0, W.WORLD_M, 0, W.WORLD_M))
        ax.plot([pt["x"] for pt in r.log], [pt["y"] for pt in r.log], "-",
                lw=1.2, color="tab:blue")
        ax.plot(*sc.start[:2], "gs", ms=5)
        ax.annotate("", xy=(sc.target[0] + 0.2 * math.cos(sc.target[2]),
                            sc.target[1] + 0.2 * math.sin(sc.target[2])),
                    xytext=sc.target[:2],
                    arrowprops=dict(arrowstyle="->", color="red", lw=1.5))
        ax.set_xlim(1.0, 4.6)
        ax.set_ylim(0.9, 3.6)
        ax.set_aspect("equal")
        # ★タイトルはASCIIで書く。matplotlibの既定フォントに日本語グリフが
        # 無く、豆腐（□）になって読めない（デバッグ用の図で本末転倒）
        ax.set_title(f"{sc.name} {'OK' if r.ok else 'NG'}\n"
                     f"err {r.err_pos * 100:.1f}cm/{r.err_yaw_deg:.0f}deg  "
                     f"clear {r.min_clear * 100:.1f}cm  hit {r.collisions}",
                     fontsize=8)
    for j in range(n, rows * cols):
        axes[j // cols][j % cols].axis("off")
    plt.tight_layout()
    plt.savefig(out, dpi=110)
    print(f"saved: {out.resolve()}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", default="all",
                    help="parallel|garage|angled|deadend|all（カンマ区切り可）")
    ap.add_argument("--n", type=int, default=10, help="各配置の試行数")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--errors", default="none", help="none|real|harsh")
    ap.add_argument("--max-time", type=float, default=45.0)
    ap.add_argument("--param", action="append", default=[],
                    help="plannerのパラメータ上書き（例 --param cruise_speed=0.2）")
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--verbose", action="store_true", help="1件ごとに結果を出す")
    args = ap.parse_args()

    names = (list(W.SCENARIOS) if args.scenario == "all"
             else [s.strip() for s in args.scenario.split(",")])
    params = {}
    known = {sp.key for sp in ParkToPoint.params}
    for kv in args.param:
        k, v = kv.split("=")
        # ★知らないキーは**黙って無視させない。** `merged()`は宣言済みの
        # キーしか読まないので、パラメータをリネームした後に古い名前を
        # 渡しても無反応になり、「効いているつもりで効いていない」計測を
        # してしまう（実際に`obstacle_max_range`のリネームで踏んだ）
        if k not in known:
            raise SystemExit(f"知らないパラメータ: {k}\n"
                             f"使えるのは: {', '.join(sorted(known))}")
        params[k] = float(v)

    results = run_batch(names, args.n, seed=args.seed,
                        errors=SensorErrors.preset(args.errors),
                        max_time=args.max_time, params=params or None)
    if args.verbose:
        for sc, r in results:
            print(f"{r.scenario:9s} {'OK ' if r.ok else 'NG '} {r.phase:4s} "
                  f"t={r.t:5.1f}s sw={r.switches} coll={r.collisions} "
                  f"clear={r.min_clear * 100:5.1f}cm "
                  f"err={r.err_pos * 100:5.1f}cm/{r.err_yaw_deg:4.1f}° "
                  f"(planner申告 {r.believed_pos * 100:.1f}cm) {sc.params}")
    _summary(results)
    if args.plot:
        _plot(results, Path("park_bench_result.png"))


if __name__ == "__main__":
    main()
