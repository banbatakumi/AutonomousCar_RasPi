"""実機の記録を横グリップの学習（`raspi/nav/grip.py`）に流し直して、前兆の判定を確かめる。

    python3 -m raspi.tools.grip_replay ~/Downloads/surge_20261012_122614.mcap
    python3 -m raspi.tools.grip_replay run.mcap --pin 4.9          # 上限を固定して流す
    python3 -m raspi.tools.grip_replay run.mcap --export run.npz   # テスト用の小さな記録に抜く
    python3 -m raspi.tools.grip_replay run.mcap --learner time --plan-lap 5.9   # タイムで決める版

## 何のためか

`GripLearner` の前兆の判定は 100Hz の車両状態（車速・ヨーレート・実舵角）だけで決まり、
自己位置にも指令にも依らない。だから**記録をそのまま流せば、実機で何が前兆と判定されたかが
手元で再現できる**。シム（`sim.bench`）の車は雑音が実機よりずっと小さく、しきい値をシムだけで
決めると実機で誤検出する（2026-10-12: 滑っていない車で前兆 46 回、横の上限が下限の 1.0m/s² まで
落ちて 1 周 5.7s → 13.1s）。

- **滑っていない記録では前兆 0 が合格**。`--pin` で上限を固定すると、「その上限で走っていたと
  したら」の判定になる（記録の車がその横加速度を実際に出していた区間だけが試行になる）
- 既定の `--learner events` は前兆で決める元の学習器（`GripLearner`）。`--learner time` は planner が
  既定で使う `TimeLearner` で、安全装置（前兆・リアの滑り・横偏差の増え）の判定を流せる。
  周回タイムによる段の採否には計画のラップ時間が要るが記録には無いので、`--plan-lap` で与えたときだけ
  採否も流れる（開ループなので、採った段で記録の車が速くなることはない）
- 流し直しは**開ループ**: 前兆で目標速度を落としても、記録の車速は変わらない。学習値が
  どこまで上がるかは見られない（それはシムで見る）。見るのは誤検出の数と、下がり方

## 入力

`.mcap`（logger_node の記録）か、`--export` で抜いた `.npz`（`raspi/tests/data/` の回帰テスト用）。
学習を止める区間（`gate`）は、記録の `auto/state.reason` に学習の注記があればそこから、無ければ
「通常の周回で、舵が上限でない」で決める。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path
from typing import NamedTuple

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.nav import grip as grip_mod  # noqa: E402

#: 舵がこの割合以上なら「舵が上限」（`slam2d_raceline._grip_end` と同じ）
STEER_GATE = 0.97
_GATE_RE = re.compile(r"（停止: (.*?)・前兆")


class Record(NamedTuple):
    """学習の流し直しに要る列だけを抜いた記録。時刻は記録の最初からの秒。"""

    t: np.ndarray           #: 車両状態（100Hz）の時刻 [s]
    v: np.ndarray           #: 車速 [m/s]
    w: np.ndarray           #: ヨーレート [rad/s]
    steer: np.ndarray       #: 実舵角 [rad]
    tc: np.ndarray          #: TC が介入しているか
    st_t: np.ndarray        #: planner の状態（10Hz）の時刻 [s]
    x: np.ndarray           #: 自己位置 [m]
    y: np.ndarray
    yaw: np.ndarray
    cross: np.ndarray       #: 経路からの横偏差 [m]
    gate: np.ndarray        #: 学習を止める周期か（1 = 止める）
    laps: np.ndarray


def load(path: str | Path, vehicle: Vehicle | None = None) -> Record:
    path = Path(path)
    if path.suffix == ".npz":
        with np.load(path, allow_pickle=False) as z:
            return Record(**{k: np.asarray(z[k]) for k in Record._fields})
    return _load_mcap(path, vehicle or Vehicle.load())


def _load_mcap(path: Path, vehicle: Vehicle) -> Record:
    import json

    from mcap.reader import make_reader

    vs: list[tuple] = []
    st: list[tuple] = []
    with open(path, "rb") as f:
        for _, ch, m in make_reader(f).iter_messages(topics=["/vehicle_state", "/auto/state"]):
            d = json.loads(m.data)
            t = d["t_capture"] / 1e9
            if ch.topic == "/vehicle_state":
                vs.append((t, d["speed"], d["yaw_rate"], d["steer_actual"], bool(d.get("tc_active"))))
                continue
            reason = d.get("reason") or ""
            race = d.get("phase") == "RACE" and d.get("engaged") and d.get("ready")
            if not race:
                gate = True
            elif "学習 横" in reason:
                gate = _GATE_RE.search(reason) is not None
            else:
                gate = (abs(d.get("target_steer") or 0.0) >= STEER_GATE * vehicle.max_steer
                        or bool(d.get("brake")) or "乗るまで" in reason)
            st.append((t, d.get("pose_x") or 0.0, d.get("pose_y") or 0.0, d.get("pose_yaw") or 0.0,
                       d.get("cross_track") or 0.0, gate, d.get("laps") or 0))
    if not vs or not st:
        raise ValueError("vehicle_state か auto/state が無い")
    vs.sort(key=lambda r: r[0])
    st.sort(key=lambda r: r[0])
    t0 = min(vs[0][0], st[0][0])
    a = np.asarray([r[:4] for r in vs], dtype=np.float64)
    s = np.asarray([r[:5] for r in st], dtype=np.float64)
    return Record(t=a[:, 0] - t0, v=a[:, 1].astype(np.float32), w=a[:, 2].astype(np.float32),
                  steer=a[:, 3].astype(np.float32), tc=np.asarray([r[4] for r in vs], dtype=np.uint8),
                  st_t=s[:, 0] - t0, x=s[:, 1].astype(np.float32), y=s[:, 2].astype(np.float32),
                  yaw=s[:, 3].astype(np.float32), cross=s[:, 4].astype(np.float32),
                  gate=np.asarray([r[5] for r in st], dtype=np.uint8),
                  laps=np.asarray([r[6] for r in st], dtype=np.int32))


def cut(rec: Record, start_s: float = 0.0, end_s: float | None = None) -> Record:
    """`[start_s, end_s]` の区間だけ残す。"""
    end = math.inf if end_s is None else end_s
    a = (rec.t >= start_s) & (rec.t <= end)
    b = (rec.st_t >= start_s) & (rec.st_t <= end)
    return Record(*[x[a] for x in rec[:5]], *[x[b] for x in rec[5:]])


def export(rec: Record, path: str | Path) -> None:
    np.savez_compressed(path, **rec._asdict())


class Result(NamedTuple):
    learner: grip_mod.GripLearner      #: `--learner time` なら `TimeLearner`
    #: (時刻, 横の全体の値) の移り変わり（変わったときだけ）
    levels: list[tuple[float, float]]
    #: 学習が見ていた時間 [s]（`gate` が空の周期の合計）
    seen_s: float


def replay(rec: Record, vehicle: Vehicle | None = None, *, init: float | None = None,
           max_level: float = 6.0, pin: float | None = None, step_up: float = 0.06,
           cfg: dict[str, float] | None = None, learner: str = "events",
           plan_lap: float = 0.0) -> Result:
    """記録を `GripLearner` に流す（モジュール docstring）。

    :param init: 横の出発点 [m/s²]（既定は planner と同じ、同定値の 0.85 倍）
    :param pin: 横の全体の値をこの値に固定して流す [m/s²]（学習値は動かさず、判定だけを見る）
    :param cfg: `GripConfig` の上書き
    :param learner: "events"（前兆で決める `GripLearner`）か "time"（`TimeLearner`）
    :param plan_lap: "time" のとき、計画のラップ時間 [s]。0 なら周回タイムの採否は流れない
    """
    v = vehicle or Vehicle.load()
    if learner not in ("events", "time"):
        raise ValueError(f"learner は events か time: {learner}")
    cls = grip_mod.TimeLearner if learner == "time" else grip_mod.GripLearner
    g = cls(yaw_wn=v.yaw_natural_freq_rad_s, yaw_zeta=v.yaw_damping,
                             rolling_resistance=v.rolling_resistance)
    g.cfg.step_up = step_up
    for k, val in (cfg or {}).items():
        if not hasattr(g.cfg, k):
            raise ValueError(f"GripConfig に無い項目: {k}")
        setattr(g.cfg, k, type(getattr(g.cfg, k))(val))
    start = pin if pin is not None else (init if init is not None else 0.85 * v.a_lat_max)
    sm, wb = v.steer_map, v.wheelbase
    levels: list[tuple[float, float]] = []
    j, n_st = 0, len(rec.st_t)
    gate, x, y, yaw, laps, t_prev = "記録の前", 0.0, 0.0, 0.0, None, None
    seen = 0.0
    for i in range(len(rec.t)):
        t = float(rec.t[i])
        while j < n_st and rec.st_t[j] <= t:
            x, y, yaw = float(rec.x[j]), float(rec.y[j]), float(rec.yaw[j])
            gate = "止めている" if rec.gate[j] else ""
            g.configure(init_level=start, max_level=max(max_level, start), down_only=False,
                        lat_use=1.0)
            g.observe_cross(0.0 if gate else float(rec.cross[j]))
            if plan_lap > 0.0 and isinstance(g, grip_mod.TimeLearner):
                g.observe_plan(plan_lap)
            if laps is not None and int(rec.laps[j]) != laps:
                g.lap(t)
            laps = int(rec.laps[j])
            j += 1
        if laps is None:
            continue
        sp, w = float(rec.v[i]), float(rec.w[i])
        if t_prev is not None and 0.0 < t - t_prev < 0.1:
            # planner と同じく、姿勢は車速とジャイロで今へ進める
            dt = t - t_prev
            yaw += w * dt
            x += sp * math.cos(yaw) * dt
            y += sp * math.sin(yaw) * dt
            if not gate:
                seen += dt
        t_prev = t
        if pin is not None:
            _pin(g, pin)
        kappa = math.tan(sm.effective(float(rec.steer[i]))) / wb
        g.observe(t, x, y, sp, w, kappa, gate=gate, tc_active=bool(rec.tc[i]))
        lv = g.map.level
        if not levels or abs(lv - levels[-1][1]) > 1e-6:
            levels.append((t, lv))
    return Result(g, levels, seen)


def _pin(g: grip_mod.GripLearner, level: float) -> None:
    """全体の値と各セルの前兆の値を元へ戻す（判定だけを見る。前兆の記録 `events` は残る）。"""
    m = g.map
    m.level = g.conf = level
    m.bad = 0.0
    g._prov = None
    g.converged = False
    for c in m.cells.values():
        c.bad = 0.0


def format_result(name: str, res: Result, *, verbose: bool = True) -> str:
    g = res.learner
    kinds: dict[str, int] = {}
    for e in g.events:
        kinds[e["kind"]] = kinds.get(e["kind"], 0) + 1
    lv = [x for _, x in res.levels] or [0.0]
    out = [f"{name}: 前兆 {len(g.events)}回 {kinds or ''}  横 {lv[0]:.2f} → {lv[-1]:.2f}m/s²"
           f"（最小 {min(lv):.2f}・最大 {max(lv):.2f}）  試行 {g.trials}（通過 {g.passes}）  "
           f"学習が見た時間 {res.seen_s:.0f}s"]
    if isinstance(g, grip_mod.TimeLearner):
        out.append(f"  タイムで決める版: 段の判断 {len(g.steps)}回（採用 {sum(1 for x in g.steps if x['ok'])}）"
                   f"・測った周 {'あり' if g._measured else 'なし（--plan-lap で計画のラップ時間を与える）'}"
                   f"・働いた上限 {g.map.ceil:.2f}m/s²" if g.map.ceil > 0.0 else
                   f"  タイムで決める版: 段の判断 {len(g.steps)}回・安全装置が働いた上限は無い")
    if verbose:
        for k, e in enumerate(g.events, 1):
            out.append(f"  #{k:<2d} {e['t']:6.1f}s {e['kind']}  横加速度 {e['a']:.2f}・値 {e['b']:.2f}・"
                       f"上限 {e['applied']:.2f}m/s²  不足 {e['excess']:.3f}"
                       f"{'  ★想定外' if e['surprise'] else ''}  ({e['x']:+.2f}, {e['y']:+.2f})")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", type=Path, nargs="+", help=".mcap または .npz")
    ap.add_argument("--init", type=float, default=None, help="横の出発点 [m/s²]（既定は同定値の 0.85 倍）")
    ap.add_argument("--max", type=float, default=6.0, help="上げてよい上限 [m/s²]")
    ap.add_argument("--pin", type=float, default=None, help="横の上限を固定して流す [m/s²]")
    ap.add_argument("--start", type=float, default=0.0, help="開始位置 [s]")
    ap.add_argument("--end", type=float, default=None, help="終了位置 [s]")
    ap.add_argument("--learner", choices=("events", "time"), default="events",
                    help="events = 前兆で決める GripLearner、time = planner の既定の TimeLearner")
    ap.add_argument("--plan-lap", type=float, default=0.0,
                    help="--learner time のとき、計画のラップ時間 [s]（周回タイムの採否を流す）")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                    help="GripConfig の上書き（例: --set dev_thr=0.2）")
    ap.add_argument("--export", type=Path, default=None, help="抜いた記録を .npz に書く（入力 1 本）")
    ap.add_argument("-q", "--quiet", action="store_true", help="前兆を 1 つずつ出さない")
    args = ap.parse_args()
    if args.export is not None and len(args.paths) > 1:
        print("--export は入力 1 本のときだけ使えます", file=sys.stderr)
        return 2
    try:
        cfg = {k: float(val) for k, val in (s.split("=", 1) for s in args.set)}
    except ValueError:
        print("--set は KEY=VALUE（数値）で書きます", file=sys.stderr)
        return 2
    vehicle = Vehicle.load()
    rc = 0
    for p in args.paths:
        try:
            rec = cut(load(p, vehicle), args.start, args.end)
            if args.export is not None:
                export(rec, args.export)
            res = replay(rec, vehicle, init=args.init, max_level=args.max, pin=args.pin, cfg=cfg,
                         learner=args.learner, plan_lap=args.plan_lap)
        except (OSError, ValueError, KeyError) as e:
            print(f"!! {p}: {e}", file=sys.stderr)
            rc = 1
            continue
        print(format_result(p.name, res, verbose=not args.quiet))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
