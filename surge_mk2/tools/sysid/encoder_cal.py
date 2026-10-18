"""エンコーダ校正試験（`raspi/auto/sysid_encoder.py`）の記録 → `[control]` の `enc_*`（前輪エンコーダの校正）。

    .venv/bin/python -m tools.sysid.encoder_cal 記録.mcap [記録.mcap ...]            # 結果を表示するだけ
    .venv/bin/python -m tools.sysid.encoder_cal 記録.mcap --apply                    # vehicle.toml へ書く

## 何から何を求めるか

| 求めるもの | 材料 | 基準 |
|---|---|---|
| 出力の範囲 `enc_*_lo`・`hi` | 惰行中の `front_angle`（校正前の生の読み） | 1回転で読みが戻る段差の大きさ（`hi − lo`） |
| 周期誤差 `enc_*_h1s…h2c` | 同上 | 惰行中の真の回転は滑らか（区間ごとの3次式） |

`front_angle` は STM32 が今使っている校正に依らない生の読み（★プロトコル v0.27）なので、何度録り直しても
同じ基準で求め直せる（前の結果に足し込むのではない）。車輪の半径は実測値（`vehicle.toml` の `wheel_radius`）のまま
で、ここでは求めない（周期誤差は角度の関数なので、半径に依らない）。

## 周期誤差のモデル（STM32 `lib/encoder_cal/encoder_cal.h` と同じ）

ADC のフルスケールに対する比 u（`front_angle`÷2π）は、真の角度 θ に対して

    u = lo + (hi − lo)/2π · (θ + e(θ))  （1回転ごとに hi から lo へ戻る）
    e(θ) = h1s·sinθ + h1c·cosθ + h2s·sin2θ + h2c·cos2θ

惰行の区間ごとに θ(t) を3次式とおき、戻った回数 n を数えて

    u = 3次式(t) − n·(hi − lo) + (hi − lo)/2π · e(θ)

を最小二乗で解く（θ は前の反復の値で評価して線形にする）。`hi − lo` は戻りの段差そのもの、`lo` は角度の
原点を決めるだけ（STM32 も同じ `lo` で角度にするので、どこに置いても結果の角度の差は同じ）。

**定速の巡航を使わない理由**: 車速PIはエンコーダの読み（周期誤差を含む）に合わせてトルクを揺らすので、
真の回転にも同じ周期のうねりが乗り、周期誤差が小さく出る。惰行中は何も読み返さない。

"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from . import fit, toml_update

__all__ = ["Cal", "Record", "Result", "load_record", "analyze", "CONTROL_KEYS", "main"]

TWO_PI = 2.0 * math.pi
SIDES = ("left", "right")
_JP = {"left": "左", "right": "右"}

#: 書き込むキー（`[control]`。STM32 `control_params.h` の `enc_*`）
CONTROL_KEYS = tuple(f"enc_{s}_{k}" for s in SIDES for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c"))

#: 惰行に入ってから捨てる時間 [s]（MD のトルクが 0 へ落ちきるまで。指令の到着の遅れも含む）
_COAST_SKIP_S = 0.12
#: 使う惰行の区間の最短 [s] と、これより遅いサンプルは使わない [m/s]
_COAST_MIN_S = 0.4
_COAST_MIN_SPEED = 0.15
#: 周期誤差の当てはめで外れ値とみなす残差（ロバストな σ の何倍）
_OUTLIER_SIGMA = 5.0
#: 速さの違う惰行どうしで周期誤差の係数が食い違ったとみなす差 [rad]（と標準誤差の何倍）
_SPEED_MISMATCH_RAD = math.radians(0.2)
_SPEED_MISMATCH_SIGMA = 4.0


# ── 校正のモデル（STM32 `lib/encoder_cal/encoder_cal.h` の写し）─────────────

@dataclass
class Cal:
    lo: float = 0.0
    hi: float = 1.0
    h1s: float = 0.0
    h1c: float = 0.0
    h2s: float = 0.0
    h2c: float = 0.0

    @property
    def h(self) -> np.ndarray:
        return np.array([self.h1s, self.h1c, self.h2s, self.h2c])

    def span(self) -> float:
        s = self.hi - self.lo
        return s if s > 0.5 else 1.0

    def ripple(self, theta: np.ndarray) -> np.ndarray:
        return _harm(theta) @ self.h

    def angle(self, u: np.ndarray) -> np.ndarray:
        """ADC のフルスケールに対する比 → 真の角度 0〜2π（`EncoderCal_Angle` と同じ2回の不動点反復）。"""
        theta_m = np.mod((np.asarray(u, dtype=float) - self.lo) / self.span() * TWO_PI, TWO_PI)
        theta = theta_m
        for _ in range(2):
            theta = theta_m - self.ripple(theta)
        return np.mod(theta, TWO_PI)

    def amplitude(self) -> float:
        g = np.linspace(0.0, TWO_PI, 360, endpoint=False)
        return float(np.max(np.abs(self.ripple(g))))

    @classmethod
    def from_params(cls, params: dict[str, float], side: str) -> "Cal":
        d = cls()
        return cls(**{k: float(params.get(f"enc_{side}_{k}", getattr(d, k)))
                      for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c")})


def _harm(theta: np.ndarray) -> np.ndarray:
    theta = np.asarray(theta, dtype=float)
    return np.stack([np.sin(theta), np.cos(theta), np.sin(2 * theta), np.cos(2 * theta)], axis=-1)


def _wrap_pi(a: np.ndarray) -> np.ndarray:
    return (a + math.pi) % TWO_PI - math.pi


# ── 記録 ────────────────────────────────────────────────────────────────────

#: サンプルごとの指令の区分
OTHER, RUN, COAST, BRAKE = range(4)


@dataclass
class Record:
    """エンコーダ校正試験の記録（時刻はログ先頭からの経過 [s]）。"""

    t: np.ndarray                   #: (n,)
    u: np.ndarray                   #: (n, 2) 前輪エンコーダの生の読み（ADC÷フルスケール）[FL, FR]
    speed: np.ndarray               #: (n,) STM32 の車速（区間の切り分けにだけ使う）
    phase: np.ndarray               #: (n,) `OTHER`・`RUN`・`COAST`・`BRAKE`


def _phase_of(cmd: dict) -> int:
    if cmd.get("brake"):
        return BRAKE
    if cmd.get("torque_mode"):
        return COAST if abs(float(cmd.get("target_torque", 0.0))) < 1e-6 else OTHER
    return RUN


def build_record(vs_msgs: Sequence[tuple[float, dict]], cmds: Sequence[tuple[float, dict]]) -> Record:
    """時刻 [s]（同じ時間軸）つきの `/vehicle_state`・`/cmd`（dict）→ `Record`。ベンチも使う。"""
    vs_msgs = sorted(vs_msgs, key=lambda x: x[0])
    cmds = sorted(cmds, key=lambda x: x[0])
    if not vs_msgs:
        raise ValueError("記録に /vehicle_state がありません")
    t0 = vs_msgs[0][0]
    ct = np.array([c[0] for c in cmds])
    n = len(vs_msgs)
    t = np.empty(n)
    u = np.empty((n, 2))
    speed = np.empty(n)
    phase = np.empty(n, dtype=int)
    for i, (ti, vs) in enumerate(vs_msgs):
        j = int(np.searchsorted(ct, ti, side="right")) - 1
        t[i] = ti - t0
        fa = vs.get("front_angle") or [0.0, 0.0]
        u[i] = [float(fa[0]) / TWO_PI, float(fa[1]) / TWO_PI]
        speed[i] = float(vs.get("speed", 0.0))
        phase[i] = _phase_of(cmds[j][1]) if j >= 0 else OTHER
    return Record(t=t, u=u, speed=speed, phase=phase)


def load_record(path: str | Path) -> Record:
    """mcap → `Record`。時刻は `fit.load_log` と同じくメッセージの中の単調時刻。"""
    from mcap.reader import make_reader

    vs_msgs: list[tuple[float, dict]] = []
    cmds: list[tuple[float, dict]] = []
    with open(path, "rb") as f:
        for _schema, channel, message in make_reader(f).iter_messages(topics=["/cmd", "/vehicle_state"]):
            obj = json.loads(message.data)
            if channel.topic == "/vehicle_state":
                vs_msgs.append((int(obj.get("t_capture", 0)) / fit.NS, obj))
            else:
                cmds.append((int(obj.get("t_pub", 0)) / fit.NS, obj))
    if not vs_msgs or not all(t for t, _ in vs_msgs):
        raise ValueError("記録の /vehicle_state に t_capture がありません（古い形式の記録は使えません）")
    return build_record(vs_msgs, cmds)


# ── 区間 ────────────────────────────────────────────────────────────────────

def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """True が続く区間 `[a, b)` の列。"""
    m = np.concatenate([[False], np.asarray(mask, dtype=bool), [False]])
    d = np.diff(m.astype(int))
    return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1)))


def coast_segments(rec: Record) -> list[np.ndarray]:
    """惰行の区間（頭の `_COAST_SKIP_S` を捨て、`_COAST_MIN_SPEED` 以上で同じ向きに動いている所）の添字の列。"""
    out = []
    for a, b in _runs(rec.phase == COAST):
        idx = np.arange(a, b)
        idx = idx[rec.t[idx] >= rec.t[a] + _COAST_SKIP_S]
        if idx.size == 0:
            continue
        sgn = np.sign(np.median(rec.speed[idx]))
        idx = idx[rec.speed[idx] * sgn >= _COAST_MIN_SPEED]
        # 途中で遅くなって抜けたら、そこまで（1区間の中は連続している前提で3次式を当てる）
        if idx.size and np.any(np.diff(idx) > 1):
            idx = idx[:np.flatnonzero(np.diff(idx) > 1)[0] + 1]
        if idx.size >= 10 and rec.t[idx[-1]] - rec.t[idx[0]] >= _COAST_MIN_S:
            out.append(idx)
    return out


# ── 出力の範囲・周期誤差 ──────────────────────────────────────────────────

@dataclass
class WheelFit:
    cal: Cal
    #: 周期誤差の係数の標準誤差 [rad]（h1s, h1c, h2s, h2c）
    h_se: np.ndarray
    #: 戻りの段差 hi − lo の標準誤差
    span_se: float
    #: 当てはめの残差の RMS [rad]（周期誤差を除いた後の読みのばらつき）
    resid_rad: float
    n_wraps: int
    n_samples: int


def _wraps(u: np.ndarray) -> np.ndarray:
    """読みが hi→lo（前へ回る）なら +1、lo→hi なら −1 を数えた累積（各サンプル時点）。"""
    d = np.diff(u)
    step = (d < -0.5).astype(int) - (d > 0.5).astype(int)
    return np.concatenate([[0], np.cumsum(step)])


def _fit_wheel_once(rec: Record, segs: list[np.ndarray], col: int, lo: float,
                    iters: int = 5) -> tuple[WheelFit, np.ndarray, np.ndarray]:
    us = [rec.u[idx, col] for idx in segs]
    ns = [_wraps(x) for x in us]
    tt = []
    for idx in segs:
        t = rec.t[idx]
        tt.append((t - t.mean()) / max(np.ptp(t), 1e-3))
    n_seg = len(segs)
    rows = sum(len(x) for x in us)
    P = np.zeros((rows, 4 * n_seg))
    r0 = 0
    for k, x in enumerate(tt):
        P[r0:r0 + len(x), 4 * k:4 * k + 4] = np.stack([np.ones_like(x), x, x ** 2, x ** 3], axis=1)
        r0 += len(x)
    u = np.concatenate(us)
    n = np.concatenate(ns).astype(float)
    span, h = 1.0, np.zeros(4)
    use = np.ones(rows, dtype=bool)
    theta = np.zeros(rows)
    coef_cov = np.zeros((5, 5))
    resid = np.zeros(rows)
    for it in range(iters):
        cal = Cal(lo=lo, hi=lo + span, h1s=h[0], h1c=h[1], h2s=h[2], h2c=h[3])
        theta_m = (u + n * span - lo) / span * TWO_PI
        theta = theta_m
        for _ in range(3):
            theta = theta_m - cal.ripple(theta)
        H = _harm(theta)
        X = np.concatenate([P, -n[:, None], H], axis=1)
        sol, *_ = np.linalg.lstsq(X[use], u[use], rcond=None)
        resid = u - X @ sol
        s = 1.4826 * float(np.median(np.abs(resid[use]))) + 1e-7
        span = float(sol[4 * n_seg]) if np.any(n != 0) else 1.0
        if not np.any(n != 0):
            # 1回も戻らなかった: 段差が決まらない（列が全部0）。lstsq は 0 を返すので恒等に戻す
            span = 1.0
        h = sol[4 * n_seg + 1:] * TWO_PI / span
        if it >= 1:
            use = np.abs(resid) < _OUTLIER_SIGMA * s
        if it == iters - 1:
            Xu = X[use]
            sig2 = float(np.mean(resid[use] ** 2))
            tau = len(resid[use]) / fit.n_eff(resid[use])
            try:
                cov = np.linalg.pinv(Xu.T @ Xu) * sig2 * tau
            except np.linalg.LinAlgError:
                cov = np.zeros((X.shape[1], X.shape[1]))
            coef_cov = cov[4 * n_seg:, 4 * n_seg:]
    span_se = math.sqrt(max(coef_cov[0, 0], 0.0))
    h_se = np.sqrt(np.maximum(np.diag(coef_cov)[1:], 0.0)) * TWO_PI / span
    cal = Cal(lo=lo, hi=lo + span, h1s=float(h[0]), h1c=float(h[1]), h2s=float(h[2]), h2c=float(h[3]))
    resid_rad = float(np.sqrt(np.mean(resid[use] ** 2))) * TWO_PI / span
    n_wraps = int(sum(np.abs(np.diff(x)).sum() for x in ns))
    return WheelFit(cal, h_se, span_se, resid_rad, n_wraps, int(use.sum())), u[use], theta[use]


def fit_wheel(rec: Record, segs: list[np.ndarray], side: str) -> WheelFit:
    """片輪の出力の範囲と周期誤差。`lo` は惰行中の読みの最小（外れ値を除く）に置く。"""
    if not segs:
        raise ValueError("惰行の区間がありません（エンコーダ校正試験の記録か、惰行が短すぎないか確かめてください）")
    col = SIDES.index(side)
    first, u_in, _ = _fit_wheel_once(rec, segs, col, 0.0)
    span = first.cal.span()
    lo = float(np.min(u_in)) if u_in.size else 0.0
    # 範囲は 0〜1 の中（STM32 の範囲 lo 0〜0.2・hi 0.8〜1）。段差が 1 に近いと最小が少し浮くので詰める
    lo = min(max(lo, 0.0), max(1.0 - span, 0.0))
    return _fit_wheel_once(rec, segs, col, lo)[0]


# ── まとめ ──────────────────────────────────────────────────────────────────

@dataclass
class Result:
    values: dict[str, float]
    notes: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    wheels: dict[str, WheelFit] = field(default_factory=dict)


def _merge(records: Sequence[Record]) -> Record:
    if len(records) == 1:
        return records[0]
    parts, off = [], 0.0
    for r in records:
        parts.append((r, off))
        off += float(r.t[-1]) + 10.0       # 記録の間に十分な隙間（区間が繋がらない）
    t = np.concatenate([r.t + o for r, o in parts])
    phase = np.concatenate([r.phase for r, _ in parts])
    # 記録の継ぎ目は「その他」にして区間を切る
    starts = np.cumsum([0] + [len(r.t) for r, _ in parts[:-1]])
    phase[starts] = OTHER
    speed = np.concatenate([r.speed for r, _ in parts])
    speed[starts] = 1.0
    return Record(t=t, u=np.concatenate([r.u for r, _ in parts]), speed=speed, phase=phase)


def _speed_split(rec: Record, segs: list[np.ndarray]) -> tuple[list[np.ndarray], list[np.ndarray]]:
    v = np.array([float(np.median(np.abs(rec.speed[idx]))) for idx in segs])
    mid = float(np.median(v)) if v.size else 0.0
    return [s for s, x in zip(segs, v) if x < mid], [s for s, x in zip(segs, v) if x >= mid]


def analyze(records: Record | Sequence[Record]) -> Result:
    """記録（1本以上。同じ車）→ `[control]` に書く値と所見。"""
    recs = [records] if isinstance(records, Record) else list(records)
    rec = _merge(recs)
    segs = coast_segments(rec)
    out = Result(values={})
    # 区間ごとの境目は記録の継ぎ目を跨がない（`_merge`）
    for side in SIDES:
        wf = fit_wheel(rec, segs, side)
        out.wheels[side] = wf
        c = wf.cal
        out.values.update({f"enc_{side}_{k}": round(float(getattr(c, k)), 6)
                           for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c")})
        out.notes.append(
            f"前輪エンコーダ（{_JP[side]}）: 出力の範囲 {c.lo:.4f}〜{c.hi:.4f}（段差の標準誤差 {wf.span_se:.4f}）、"
            f"周期誤差 ±{math.degrees(c.amplitude()):.2f}°（係数の標準誤差 {math.degrees(float(np.max(wf.h_se))):.3f}°）、"
            f"残りのばらつき {math.degrees(wf.resid_rad):.3f}°（惰行 {len(segs)} 区間・{wf.n_samples} 点・"
            f"戻り {wf.n_wraps} 回）")
        if wf.n_wraps == 0:
            out.warnings.append(f"★{_JP[side]}前輪: 惰行中に1回転もせず、出力の範囲が決まらない（hi−lo=1 のまま）")
    # 速さで分けて当てはめ直し、係数が角度だけの関数か確かめる
    slow, fast = _speed_split(rec, segs)
    if len(slow) >= 2 and len(fast) >= 2:
        for side in SIDES:
            a, b = fit_wheel(rec, slow, side), fit_wheel(rec, fast, side)
            diff = np.abs(a.cal.h - b.cal.h)
            se = np.sqrt(a.h_se ** 2 + b.h_se ** 2) + 1e-12
            worst = int(np.argmax(diff / se))
            msg = (f"周期誤差（{_JP[side]}）の速さによる違い: 最大 {math.degrees(float(diff[worst])):.3f}°"
                   f"（標準誤差の {float(diff[worst] / se[worst]):.1f} 倍）")
            if diff[worst] > _SPEED_MISMATCH_RAD and diff[worst] / se[worst] > _SPEED_MISMATCH_SIGMA:
                msg = "★" + msg + "。角度だけの関数でない誤差（振動・ADC の遅れ）が混ざっている"
                out.warnings.append(msg)
            out.notes.append(msg)
    for w in out.warnings:
        if w not in out.notes:
            out.notes.append(w)
    for key, value in out.values.items():
        lo, hi = toml_update.CONTROL_RANGES[key]
        if not lo <= value <= hi:
            out.warnings.append(f"★{key}={value:.5g} は STM32 の受け付ける範囲 {lo}〜{hi} の外")
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mcap", nargs="+")
    ap.add_argument("--apply", action="store_true", help="config/vehicle.toml の [control] へ書く")
    ap.add_argument("--toml", default=str(fit.DEFAULT_TOML))
    a = ap.parse_args(argv)
    res = analyze([load_record(p) for p in a.mcap])
    for n in res.notes:
        print(n)
    print()
    for k in CONTROL_KEYS:
        if k in res.values:
            print(f"{k} = {res.values[k]}")
    if a.apply:
        if res.warnings:
            print("\n★警告があるので書き込みません（記録を確かめて録り直してください）")
            return 1
        changed = toml_update.apply_control(a.toml, res.values)
        print(f"\n{a.toml} を更新: {', '.join(changed) or '変更なし'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
