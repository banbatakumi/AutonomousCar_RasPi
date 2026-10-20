"""校正 8: TC・ABS のゲイン（`raspi/auto/calib_tc_gain.py` の記録）→ `tc_kp/ki`・`abs_kp/ki`。設計は `docs/calibration.md` §4.8。

片道ごとに、試験が `fw_overrides` で入れたゲイン（交差周波数 ωc と PI の零点の比）を記録から読み戻し、その片道の発進（TC）・
制動（ABS）を評価する。ゲインは基準 `kp = ωc·J/r`、`ki = kp·ωc/ratio`（`raspi/auto/calib_tc_gain.base_gains`）。

## 評価（片道・種類ごと。`traction.py` と同じ指標）

- 発振: 区間内のスリップ率の揺れが `SLIP_STD_MAX` を超え、一つの周波数に集まり（`PEAK_SHARE_MAX`）、その周波数が `OSC_FREQ_MIN_HZ` 以上
  （それより遅い揺れはゲインが低くて目標に保てていないだけ）で、**手前の片道の最小の揺れ（高周波の大きさ）の `OSC_GROWTH` 倍を超える**。
  実機は TC・ABS のゲインによらず 20〜30Hz の揺れが常にあり（TC・ABS を切ったランプでも制動側は 0.03m/s）、ゲインを上げても増えなかった
  ので、「一つの周波数に集まる」だけでは発振と言えない（2026-10-20 に判明）
- 平均のスリップ率が目標（校正 7）の ±`SLIP_TOL` 以内
- 加速度・減速度の中央値（性能）、始まりの行き過ぎ（目標の 2 倍を超えた時間）

## 採り方

1. **ゲイン余裕 2 倍**: 採る ωc は、比 4 の梯子で最初に発振した ωc の半分以下（発振が出なければ梯子の最大の半分以下）
2. その範囲の片道（**比を変えた片道も含む**。I を強くすると低い ωc でも目標に保てることがある）のうち、発振せず平均スリップ率が目標の
   ±`SLIP_TOL` 以内のものから、加速度・減速度が最良の `_PERF_KEEP` 以上を保つ最大の ωc を採る（高すぎるとトルクが暴れて性能が落ちる）
3. 同じ ωc の中では、始まりの行き過ぎの時間が最も短い比（差が小さければ 4 に近い方）
4. kp・ki を書く。範囲に目標に保てた片道が無ければ不合格（ゲインを上げると発振し、下げると保てない）
"""

from __future__ import annotations

import numpy as np

from raspi.auto._calib_common import phase_parts
from raspi.auto.calib_tc_gain import KEYS, OSC_GROWTH, WHEEL_RADIUS_M, WIN, is_oscillation, jitter_rms

from . import tyre
from .encoder import _stm_time
from .log import Log
from .result import Result
from .speed import body_distance
from .traction import SLIP_TOL, _TARGET_V, _chunks, _onsets, _is, peak_share

__all__ = ["analyze", "LegResult"]

#: 梯子の比（`CalibTcGain.ZERO_RATIO`）と、比を同じとみなす幅
_BASE_RATIO = 4.0
_RATIO_TOL = 0.2
#: 性能（加速度・減速度）が、発振せず目標に保てた中の最良のこの割合以上を保つ中から選ぶ
_PERF_KEEP = 0.95
_KIND = {"tc": ("drive", False), "abs": ("brake", True)}


class LegResult:
    """片道・種類ごとの評価。"""

    def __init__(self, leg: int, kind: str, wc: float, ratio: float, mean: float, std: float, share: float,
                 freq: float, jit: float, perf: float, over_s: float) -> None:
        self.leg, self.kind, self.wc, self.ratio = leg, kind, wc, ratio
        self.mean, self.std, self.share, self.freq, self.jit = mean, std, share, freq, jit
        self.perf, self.over_s = perf, over_s
        #: 発振か。手前の片道との比べで決まる（`evaluate_legs` の最後）
        self.osc = False

    def label(self) -> str:
        return f"ωc {self.wc:.0f}rad/s・比 {self.ratio:.1f}"


def _applied(log: Log, t_wall: float, key: str) -> float:
    return tyre._override_at(log, t_wall, key)


def evaluate_legs(log: Log, J: float, targets: dict[str, float]) -> list[LegResult]:
    t = _stm_time(log)
    x = body_distance(log)
    v = np.gradient(x, t)
    slip = np.asarray(log.vs["tc_slip"], dtype=float)
    out: list[LegResult] = []
    for kind, (stage, select_low) in _KIND.items():
        kp_key, ki_key = KEYS[kind]
        for name, i0, _ in log.segments(lambda p, s=stage: _is(p, s, None)):
            leg = (phase_parts(name) or ("", -1))[1]
            kp, ki = _applied(log, log.t[i0] + 0.2, kp_key), _applied(log, log.t[i0] + 0.2, ki_key)
            if not (np.isfinite(kp) and np.isfinite(ki)) or kp <= 0 or ki <= 0:
                continue
            wc = kp * WHEEL_RADIUS_M / J
            ratio = kp * wc / ki
            ch = _chunks(log, stage, t, x, v, leg)
            if not ch:
                continue
            lam, inner, acc, vv = [], [], [], []
            for part, a in ch:
                sl, sr = slip[part, 0], slip[part, 1]
                s = np.abs(np.minimum(sl, sr) if select_low else np.maximum(sl, sr))
                lam.append(float(np.mean(s)))
                inner.append(float(np.std(s)))
                acc.append(abs(a))
                vv.append(float(np.mean(np.abs(v[part]))))
            lam, vv = np.array(lam), np.array(vv)
            fast = vv >= _TARGET_V
            mean = float(np.mean(lam[fast])) if fast.any() else float(np.mean(lam))
            std = float(np.max(inner))
            def series_of(part, sl=select_low):
                a, b = slip[part, 0], slip[part, 1]
                return np.abs(np.minimum(a, b) if sl else np.maximum(a, b))
            share, freq = peak_share(log, stage, series_of, v, t, leg, WIN)
            jit = jitter_rms(series_of(np.concatenate([p for p, _ in ch])))
            ons = _onsets(log, stage, slip, select_low, leg)
            over = max((float(np.sum(o[1] > 2 * targets[kind])) * 0.01 for o in ons), default=0.0)
            out.append(LegResult(leg, kind, wc, ratio, mean, std, share, freq, jit, float(np.median(acc)), over))
    # 発振: 揺れは TC・ABS のゲインによらず常にあるので、手前の片道の最小の揺れより増えたものだけを発振とする
    for kind in _KIND:
        base = None
        for l in sorted((l for l in out if l.kind == kind), key=lambda l: l.leg):
            l.osc = is_oscillation(l.std, l.share, l.freq, l.jit, base)
            if np.isfinite(l.jit):
                base = l.jit if base is None else min(base, l.jit)
    return out


def _pick(res: Result, kind: str, name: str, legs: list[LegResult], target: float, J: float) -> None:
    rows = sorted([l for l in legs if l.kind == kind], key=lambda l: l.leg)
    if not rows:
        res.fail(f"{name}: 片道の記録が無い（`calib_tc_gain` の記録か確かめる）")
        return
    for l in rows:
        res.info(f"{name}・片道 {l.leg // 2 + 1}: {l.label()} → スリップ率 {l.mean:.3f}（目標 {target:.3f}）・揺れ {l.std:.3f}（高周波 {l.jit:.3f}）"
                 f"{'・発振（' + format(l.freq, '.0f') + 'Hz に ' + format(l.share, '.0%') + '、手前の ' + format(OSC_GROWTH, '.1f') + ' 倍超）' if l.osc else ''}"
                 f"・{'加速度' if kind == 'tc' else '減速度'} {l.perf:.2f}m/s²・始まりの行き過ぎ {l.over_s * 1000:.0f}ms")
    ladder = [l for l in rows if abs(l.ratio - _BASE_RATIO) < _RATIO_TOL]
    first_osc = next((l for l in ladder if l.osc), None)
    # ゲイン余裕 2 倍: 採る ωc は、梯子で最初に発振した ωc の半分以下（発振が出なければ、梯子の最大の半分以下。最大は 2 倍の側が
    # 試されていない）。比は問わない（比を変えた片道も候補。I を強くすると低い ωc でも目標に保てることがある）
    limit = (first_osc.wc if first_osc else max(l.wc for l in rows)) / 2.0 * 1.001
    good = [l for l in rows if not l.osc and abs(l.mean - target) <= SLIP_TOL and l.wc <= limit]
    if not good:
        res.fail(f"{name}: 発振の手前（ωc {limit:.0f}rad/s 以下）で、目標（{target:.3f}±{SLIP_TOL}）に保てた片道が無い。ゲインを上げると発振し、"
                 "下げると目標に保てない。校正 7 の目標を下げる（山の手前に置く）か、路面・タイヤ・MD の状態を確かめる")
        return
    # 性能が最良に近い中で最大の交差周波数（高すぎるとトルクが暴れて性能が落ちる）。同じ ωc の中では始まりの行き過ぎが最も短い比
    best = max(l.perf for l in good)
    keep = [l for l in good if l.perf >= _PERF_KEEP * best]
    wc_sel = max(l.wc for l in keep)
    at_wc = [l for l in keep if abs(l.wc - wc_sel) <= 0.5 + 1e-3 * wc_sel]
    best_over = min(l.over_s for l in at_wc)
    sel = min((l for l in at_wc if l.over_s <= best_over + 0.02), key=lambda l: abs(np.log(l.ratio / _BASE_RATIO)))
    ratio = sel.ratio
    if first_osc is None:
        res.ok(f"{name}: 梯子の最大まで発振しなかった。2 倍の余裕が確かめられた中で採る: {sel.label()}")
    else:
        res.ok(f"{name}: ωc {first_osc.wc:.0f}rad/s で発振 → 半分以下で採る: {sel.label()}（発振の {first_osc.wc / sel.wc:.1f} 分の 1）")
    same = sorted((l for l in good if abs(l.wc - sel.wc) <= 0.5 + 1e-3 * sel.wc), key=lambda l: l.ratio)
    if len(same) > 1:
        res.info(f"{name}: 同じ ωc での比の比較: " + "・".join(f"比 {l.ratio:.1f} → 行き過ぎ {l.over_s * 1000:.0f}ms・{l.perf:.2f}m/s²" for l in same))
    kp = sel.wc * J / WHEEL_RADIUS_M
    ki = kp * sel.wc / ratio
    kp_key, ki_key = KEYS[kind]
    res.values[f"control.{kp_key}"] = round(kp, 5)
    res.values[f"control.{ki_key}"] = round(ki, 4)
    res.notes[f"control.{kp_key}"] = f"ωc {sel.wc:.0f}rad/s（発振する ωc の半分以下で、目標に保てて性能が最良に近いもの）"
    res.notes[f"control.{ki_key}"] = f"零点 = 交差/{ratio:.1f}"
    xs = [l.wc for l in ladder]
    res.plot(f"{name}: 揺れ（横軸: 交差周波数 [rad/s]）", "区間内の揺れ", xs, [l.std for l in ladder])


def analyze(log: Log, consts: tyre.Consts | None = None) -> Result:
    res = Result("校正 8: TC・ABS のゲイン")
    if log.aborted:
        res.warn(f"試験は途中で中止された（{log.aborted}）。残った片道で解析する")
    c = consts or tyre.Consts.from_toml()
    # 目標スリップ率は試験が上書きした値（記録の指令に残る。戻る片道の指令には上書きが無いので、あるものを探す）
    def tested(key: str) -> float:
        return next((float(o[key]) for o in log.cmd_overrides if key in o), float("nan"))
    tc_t, abs_t = tested("tc_slip_target"), tested("abs_slip_target")
    if not (np.isfinite(tc_t) and np.isfinite(abs_t)):
        tc_t = log.control_params.get("tc_slip_target", 0.09)
        abs_t = log.control_params.get("abs_slip_target", 0.18)
        res.warn("記録の指令に目標スリップ率の上書きが無い。STM32 に入っていた値を使う")
    res.info(f"後輪の慣性 {c.J * 1e6:.1f}e-6kg·m²（校正 6）から基準のゲイン kp = ωc·J/r・ki = kp·ωc/比 を作った片道を評価する"
             f"（試験の間の目標スリップ率: TC {tc_t:.3f}・ABS {abs_t:.3f}）")
    targets = {"tc": tc_t, "abs": abs_t}
    legs = evaluate_legs(log, c.J, targets)
    _pick(res, "tc", "TC（全開の発進）", legs, tc_t, c.J)
    _pick(res, "abs", "ABS（最大の制動）", legs, abs_t, c.J)
    return res
