"""制御パラメータの最適化。

制御ごとに「調整する項目・場面・コスト」をまとめた `Problem` を作り、あり得る車の組
（`plant.variants()`）すべてで場面を回して、**平均と最悪の中間**が最小になる値を探す
（平均だけだと、ある1つの条件で破綻する値が選ばれる）。

## スリップ率の目標は最適化しない（2026-10-05、実機の同定を受けて）

TC・ABS で調整するのは**ゲイン（kp・ki）だけ**。スリップ率の目標（`tc_slip_target`・`abs_slip_target`）は
「前後の力」と「横グリップ」のどちらを取るかという**方針**で、最適な値というものが無い——人が
`tradeoff()` の表（同定したタイヤの曲線から、目標ごとに前後力と残る横グリップ）を見て決める。
最初は目標も一緒に探し、横グリップの損に重みを掛けて釣り合わせていた。実機のタイヤは滑らせても
前後力が落ちない形（`tyre_c`≈1.0）で、その重みでは「ABS のゲインを下げてロックさせた方が止まる」が
最適になった（効率 0.91→0.97 と引き換えに、滑っていた割合 0.03→0.58）。後輪だけで制動する車で
後輪をロックさせるとスピンするが、このモデルは横滑りを表せず、その損をコストに入れられない。

## コスト（小さいほど良い。場面×車 1つぶん）

- **目標からの行き過ぎ**（`slip_over`: 目標を超えた分の時間平均 ÷ 目標）と、深く滑っていた時間の割合
  （スリップ率の絶対値 > 0.3。**目標は 0.25 以下の前提**）× `overshoot`。これが主
- 効率の損 × `efficiency`（小さい重み）: 一定トルクの最良（`scenarios.oracle()`）に対して何割遠回り
  したか。目標より下に留まりすぎないための項で、主にしてはいけない——**効率を主にすると、積分を
  弱くして目標より上に居座るゲインが選ばれる**（目標 0.1 に対して平均 0.19 で滑り、そのぶん止まる距離が
  縮む。2026-10-05 に実際に出た）
- トルクの暴れ × 小さい重み（同じ性能なら滑らかな方を選ぶ）
- してはいけないこと（定速・定常旋回での介入、浮いた輪の吹け上がり、ABS のフォールバック）は大きな罰

探索は対数スケールの差分進化（`scipy.optimize.differential_evolution`、乱数の種は固定）。
"""

from __future__ import annotations

import math
import multiprocessing
import os
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
from scipy.optimize import differential_evolution

from . import scenarios as S
from .fw import Firmware
from .plant import Plant, variants

__all__ = ["Problem", "Tuning", "evaluate", "optimise", "tc_problem", "abs_problem",
           "PROBLEMS", "Weights", "tradeoff", "SLIP_TARGET_RANGE"]


@dataclass(frozen=True)
class Weights:
    #: 目標からの行き過ぎ（`slip_over`）と、深く滑っていた時間の割合の重み
    overshoot: float = 1.0
    #: 効率の損の重み。行き過ぎより十分小さくする（冒頭の説明）
    efficiency: float = 0.3
    #: トルクの暴れ [N·m/s] の重み
    chatter: float = 0.01
    #: してはいけないことの罰の倍率
    penalty: float = 5.0


@dataclass(frozen=True)
class Problem:
    key: str
    label: str
    #: 調整する項目 → (下限, 上限)。0 を含まない項目は対数スケールで探す
    bounds: dict[str, tuple[float, float]]
    scenarios: tuple[S.Scenario, ...]
    #: (場面, 物差し, 一定トルクの最良の距離, 重み) → コスト
    cost: Callable[[S.Scenario, dict[str, float], float, Weights], float]
    #: 一定トルクの最良（効率の分母）が要る場面
    needs_oracle: frozenset[str] = frozenset()
    variants: Callable[[Plant], list[tuple[str, Plant]]] = variants


@dataclass
class Tuning:
    problem: Problem
    values: dict[str, float]
    cost: float
    #: 調整前（渡した初期値）のコスト
    baseline_cost: float
    baseline: dict[str, float]
    #: 場面のキー → (調整前の物差しの平均, 調整後の物差しの平均)。物差しは車の組での平均
    table: dict[str, tuple[dict[str, float], dict[str, float]]] = field(default_factory=dict)
    #: 車の名前 → (調整前, 調整後) のコスト
    by_variant: dict[str, tuple[float, float]] = field(default_factory=dict)
    evaluations: int = 0


_WORKER: dict = {}


def _worker_init(git_ref: str | None) -> None:
    _WORKER["fw"] = Firmware(git_ref)


def _run_variant(fw: Firmware, plant: Plant, problem: Problem, oracle: dict[str, float],
                 values: dict[str, float], weights: Weights):
    out = {}
    for sc in problem.scenarios:
        m = S.run(fw, plant, sc, values).metrics
        orc = oracle.get(sc.key, float("nan"))
        if orc == orc:
            m["efficiency"] = (orc / m["distance"] if sc.kind == "brake"
                               else m["distance"] / orc) if m["distance"] > 0 else 0.0
        out[sc.key] = (problem.cost(sc, m, orc, weights), m)
    return out


def _worker_run(args):
    problem_key, plant, oracle, values, weights = args
    return _run_variant(_WORKER["fw"], plant, PROBLEMS[problem_key](), oracle, values, weights)


def _worker_oracle(args):
    problem_key, plant = args
    problem = PROBLEMS[problem_key]()
    return {sc.key: S.oracle(_WORKER["fw"], plant, sc) for sc in problem.scenarios
            if sc.key in problem.needs_oracle}


class _Evaluator:
    """場面×車を回してコストを出す。車ごとに別プロセスへ振る（`workers` > 1 のとき）。"""

    def __init__(self, fw: Firmware, plant: Plant, problem: Problem, weights: Weights,
                 workers: int | None = None) -> None:
        self.fw, self.problem, self.weights = fw, problem, weights
        self.variants = problem.variants(plant)
        if workers is None:
            workers = min(4, os.cpu_count() or 1, len(self.variants))
        # 別プロセスでは `PROBLEMS` から問題を作り直すので、登録済みの問題のときだけ並列にする
        registered = problem.key in PROBLEMS and PROBLEMS[problem.key]().scenarios == problem.scenarios
        self.pool = (multiprocessing.get_context("spawn").Pool(workers, _worker_init, (fw.git_ref,))
                     if workers > 1 and registered else None)
        if self.pool is not None:
            oracles = self.pool.map(_worker_oracle, [(problem.key, v) for _, v in self.variants])
        else:
            oracles = [{sc.key: S.oracle(fw, v, sc) for sc in problem.scenarios
                        if sc.key in problem.needs_oracle} for _, v in self.variants]
        self.oracle = {vname: o for (vname, _), o in zip(self.variants, oracles)}
        self.count = 0

    def close(self) -> None:
        if self.pool is not None:
            self.pool.terminate()
            self.pool = None

    def detail(self, values: dict[str, float]):
        """車 → 場面 → (コスト, 物差し)。"""
        self.count += 1
        if self.pool is not None:
            res = self.pool.map(_worker_run, [(self.problem.key, v, self.oracle[vname], values, self.weights)
                                              for vname, v in self.variants])
        else:
            res = [_run_variant(self.fw, v, self.problem, self.oracle[vname], values, self.weights)
                   for vname, v in self.variants]
        return {vname: r for (vname, _), r in zip(self.variants, res)}

    def cost(self, values: dict[str, float]) -> float:
        d = self.detail(values)
        per_variant = [float(np.mean([c for c, _ in sc.values()])) for sc in d.values()]
        return 0.5 * float(np.mean(per_variant)) + 0.5 * float(np.max(per_variant))


def evaluate(fw: Firmware, plant: Plant, problem: Problem, values: dict[str, float],
             weights: Weights = Weights(), workers: int | None = 1):
    """`values` のコストと、車 → 場面 → (コスト, 物差し)。"""
    ev = _Evaluator(fw, plant, problem, weights, workers)
    try:
        d = ev.detail(values)
    finally:
        ev.close()
    per_variant = [float(np.mean([c for c, _ in sc.values()])) for sc in d.values()]
    return 0.5 * float(np.mean(per_variant)) + 0.5 * float(np.max(per_variant)), d


def _mean_metrics(detail, key: str) -> dict[str, float]:
    ms = [d[key][1] for d in detail.values()]
    return {k: float(np.mean([m[k] for m in ms if k in m])) for k in ms[0]}


def optimise(fw: Firmware, plant: Plant, problem: Problem, start: dict[str, float],
             weights: Weights = Weights(), max_iter: int = 25, pop: int = 8, seed: int = 0,
             progress: Callable[[int, float], None] | None = None,
             workers: int | None = None) -> Tuning:
    """`start`（今の値。調整しない項目も含めてよい）から始めて `problem.bounds` の項目を調整する。"""
    ev = _Evaluator(fw, plant, problem, weights, workers)
    try:
        return _optimise(ev, problem, start, max_iter, pop, seed, progress)
    finally:
        ev.close()


def _optimise(ev: _Evaluator, problem: Problem, start: dict[str, float], max_iter: int, pop: int,
              seed: int, progress) -> Tuning:
    names = list(problem.bounds)
    logs = [problem.bounds[n][0] > 0.0 for n in names]
    lo = [math.log(problem.bounds[n][0]) if lg else problem.bounds[n][0] for n, lg in zip(names, logs)]
    hi = [math.log(problem.bounds[n][1]) if lg else problem.bounds[n][1] for n, lg in zip(names, logs)]

    def decode(x) -> dict[str, float]:
        v = dict(start)
        for n, lg, xi in zip(names, logs, x):
            v[n] = math.exp(xi) if lg else float(xi)
        return v

    def encode(v: dict[str, float]) -> list[float]:
        return [min(max(math.log(max(v[n], 1e-12)) if lg else v[n], a), b)
                for n, lg, a, b in zip(names, logs, lo, hi)]

    gen = [0]

    def callback(xk, convergence=0.0):
        gen[0] += 1
        if progress is not None:
            progress(gen[0], ev.cost(decode(xk)))

    base_detail = ev.detail(start)
    base_cost = ev.cost(start)
    res = differential_evolution(lambda x: ev.cost(decode(x)), list(zip(lo, hi)), x0=encode(start),
                                 maxiter=max_iter, popsize=pop, seed=seed, tol=1e-3, polish=False,
                                 init="sobol", callback=callback)
    best = decode(res.x)
    best_cost = float(res.fun)
    if best_cost >= base_cost:           # 今の値より良くならなかった
        best, best_cost = dict(start), base_cost
    best_detail = ev.detail(best)
    t = Tuning(problem, {n: best[n] for n in names}, best_cost, base_cost,
               {n: start[n] for n in names}, evaluations=ev.count)
    for sc in problem.scenarios:
        t.table[sc.key] = (_mean_metrics(base_detail, sc.key), _mean_metrics(best_detail, sc.key))
    for vname in base_detail:
        t.by_variant[vname] = (float(np.mean([c for c, _ in base_detail[vname].values()])),
                               float(np.mean([c for c, _ in best_detail[vname].values()])))
    return t


# ── 制御ごとの問題 ────────────────────────────────────────────────────────

def _pick(*keys: str) -> tuple[S.Scenario, ...]:
    by = {s.key: s for s in S.ALL}
    return tuple(by[k] for k in keys)


#: 人が選ぶスリップ率の目標の範囲。下は前輪の読みの誤差（タイヤ径・1回転周期の誤差で合わせて数%。
#: これより下だと定速で誤介入する）、上は「深く滑っていた」の物差し（0.3）より十分下
SLIP_TARGET_RANGE = (0.08, 0.25)


def tradeoff(plant: Plant, targets: tuple[float, ...] = (0.08, 0.10, 0.15, 0.20, 0.25, 0.40)):
    """スリップ率の目標 → (前後力が上限の何割出るか, 摩擦円で残る横グリップの割合)。

    同定したタイヤの曲線 `sin(C·atan(B·κ))` から。前後力を f 割使うと、横に残るのは √(1−f²)。
    """
    kappa = np.linspace(0.0, 1.5, 1501)
    curve = np.sin(plant.tyre_c * np.arctan(plant.tyre_b * kappa))
    peak = float(curve.max())
    out = []
    for t in targets:
        f = min(1.0, math.sin(plant.tyre_c * math.atan(plant.tyre_b * t)) / peak)
        out.append((t, f, math.sqrt(max(0.0, 1.0 - f * f))))
    return out

#: 浮いた輪の周速がこれを超えた分を罰する [m/s]
_LIFT_WHEEL_OK_M_S = 1.0


def _slip_cost(sc: S.Scenario, m: dict[str, float], orc: float, w: Weights) -> float:
    c = w.chatter * m["chatter"]
    if sc.key in ("cruise", "cruise_turn"):
        return c + w.penalty * (m["tc_active"] + m["abs_active"] + m["lift_active"])
    if sc.key == "lift":
        return c + w.penalty * max(0.0, m["wheel_peak"] - _LIFT_WHEEL_OK_M_S) / _LIFT_WHEEL_OK_M_S
    c += w.overshoot * (m["slip_over"] + m["slipping"])
    if "efficiency" in m:
        c += w.efficiency * max(0.0, 1.0 - m["efficiency"])
    if sc.kind == "brake" and not m.get("stopped", 1.0):
        c += w.penalty
    return c


def tc_problem() -> Problem:
    return Problem(
        "tc", "TC・片輪浮き対策",
        {"tc_kp_nm_per_m_s": (0.005, 1.0), "tc_ki_nm_per_m": (0.1, 50.0)},
        _pick("launch", "launch_pi", "roll", "accel_mu_drop", "accel_split", "lift", "corner_exit",
              "decel_pi", "decel_pi_corner", "cruise", "cruise_turn"),
        _slip_cost, frozenset({"launch", "roll", "accel_mu_drop", "corner_exit"}))


def abs_problem() -> Problem:
    return Problem(
        "abs", "ABS",
        {"abs_kp_nm_per_m_s": (0.005, 1.0), "abs_ki_nm_per_m": (0.1, 50.0)},
        _pick("brake", "brake_soft", "brake_mu_drop", "brake_split"),
        _slip_cost, frozenset({"brake", "brake_soft", "brake_mu_drop"}))


#: TV は入れていない（効きが小さく、限界での挙動をこの車両モデルが表せないので最適化できない。
#: `fit.py` 冒頭）。TV のゲインは `vehicle.toml` の `[control]` を手で決める
PROBLEMS: dict[str, Callable[[], Problem]] = {"tc": tc_problem, "abs": abs_problem}
