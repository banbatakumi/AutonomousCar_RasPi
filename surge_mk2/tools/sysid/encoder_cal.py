"""エンコーダ校正試験（`raspi/auto/sysid_encoder.py`）の記録 → `[control]` の `enc_*`・`front_radius_*`・`rear_radius_*`。

    .venv/bin/python -m tools.sysid.encoder_cal 記録.mcap [記録.mcap ...]            # 結果を表示するだけ
    .venv/bin/python -m tools.sysid.encoder_cal 記録.mcap --apply                    # vehicle.toml へ書く

## 何から何を求めるか

| 求めるもの | 材料 | 基準 |
|---|---|---|
| 出力の範囲 `enc_*_lo`・`hi` | 惰行中の `front_angle`（校正前の生の読み） | 1回転で読みが戻る段差の大きさ（`hi − lo`） |
| 周期誤差 `enc_*_h1s…h2c` | 同上 | 惰行中の真の回転は滑らか（区間ごとの3次式） |
| 前輪の半径 `front_radius_*` | 止まっている間の LiDAR のスキャン（端と端）と、その間の補正後の回転角 | スキャンどうしを合わせた移動量 |
| 後輪の半径 `rear_radius_*` | 惰行中の後輪の周速と前輪の周速（補正後の角度×上で求めた半径） | トルク 0 なのでタイヤが滑らない |

`front_angle` は STM32 が今使っている校正に依らない生の読み（★プロトコル v0.27）なので、何度録り直しても
同じ基準で求め直せる（前の結果に足し込むのではない）。後輪の周速は STM32 が今の `rear_radius_*` を掛けた値
なので、記録の `/diag/link` の `control_params` から今の値を読んで割り戻す。

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

## 前輪の半径

止まっている間（各片道の端）の前輪の角度は動かないので、時刻の合わせ方に依らない。片道ぶんの補正後の回転角
Δθ と、端どうしのスキャンを合わせた前車軸の中心の移動量 s から r = s/Δθ（左右は回頭ぶん ±トレッド/2·Δψ を
足し引き）。スキャンの照合の精度（数mm）に対して片道 2m、往復 4回で約0.1%。
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from raspi.core.vehicle import Vehicle
from raspi.msgs.types import Scan
from raspi.nav.deskew import Points, deskew
from raspi.nav.scan2scan import match_scans

from . import fit, toml_update

__all__ = ["Cal", "Record", "Result", "load_record", "analyze", "CONTROL_KEYS", "main"]

TWO_PI = 2.0 * math.pi
SIDES = ("left", "right")
_JP = {"left": "左", "right": "右"}

#: 書き込むキー（`[control]`。STM32 `control_params.h` の `enc_*`・`*_radius_*_m`）
CONTROL_KEYS = tuple(
    [f"enc_{s}_{k}" for s in SIDES for k in ("lo", "hi", "h1s", "h1c", "h2s", "h2c")]
    + [f"{a}_radius_{s}_m" for a in ("front", "rear") for s in SIDES])

#: 惰行に入ってから捨てる時間 [s]（MD のトルクが 0 へ落ちきるまで。指令の到着の遅れも含む）
_COAST_SKIP_S = 0.12
#: 使う惰行の区間の最短 [s] と、これより遅いサンプルは使わない [m/s]
_COAST_MIN_S = 0.4
_COAST_MIN_SPEED = 0.15
#: 止まっているとみなす |speed| [m/s] と、止まっている区間の最短 [s]・頭で捨てる時間 [s]
_HOLD_SPEED = 0.01
_HOLD_MIN_S = 0.6
_HOLD_SKIP_S = 0.3
#: 1サンプルの間に読みが動ける上限 [rad]（1m/s・10ms で 0.33rad、取りこぼし1回で倍）。これを超えて
#: 行って戻るサンプルは、ADC の平均の窓が 0/2π の境目を跨いだ読み（`ReadAveragedRaw`）とみなす
_SPIKE_RAD = 0.9
#: 周期誤差の当てはめで外れ値とみなす残差（ロバストな σ の何倍）
_OUTLIER_SIGMA = 5.0
#: 速さの違う惰行どうしで周期誤差の係数が食い違ったとみなす差 [rad]（と標準誤差の何倍）
_SPEED_MISMATCH_RAD = math.radians(0.2)
_SPEED_MISMATCH_SIGMA = 4.0
#: 後輪の周速の遅れとして許す範囲 [s]（MD 側のフィルタ。これを超えたら何かがおかしい）
_REAR_LAG_MAX_S = 0.05
#: プロトコル v0.27（0x001B）より前のファームは後輪の周速に名目の半径を掛けていた
_PROTO_RADIUS = 0x001B
_NOMINAL_RADIUS_M = 0.030


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


def unwrap_angle(theta: np.ndarray) -> np.ndarray:
    """0〜2π に畳まれた角度の列 → 連続した角度（隣どうしの差を ±π に畳んで足す。STM32 の積算と同じ）。

    ADC の平均の窓が 0/2π の境目を跨いだ1サンプルだけ飛んだ読み（行って戻る）は、前後の中点に置き換える
    （そのまま足すと、行きと帰りの差の畳み方しだいで1回転ぶん数え違える）。"""
    th = np.asarray(theta, dtype=float).copy()
    if th.size < 2:
        return th
    for i in range(1, th.size - 1):
        a = _wrap_pi(th[i] - th[i - 1])
        b = _wrap_pi(th[i + 1] - th[i])
        if abs(a) > _SPIKE_RAD and abs(b) > _SPIKE_RAD and abs(_wrap_pi(th[i + 1] - th[i - 1])) < _SPIKE_RAD:
            th[i] = th[i - 1] + 0.5 * _wrap_pi(th[i + 1] - th[i - 1])
    return th[0] + np.concatenate([[0.0], np.cumsum(_wrap_pi(np.diff(th)))])


# ── 記録 ────────────────────────────────────────────────────────────────────

#: サンプルごとの指令の区分
OTHER, RUN, COAST, BRAKE = range(4)


@dataclass
class Record:
    """エンコーダ校正試験の記録（時刻はログ先頭からの経過 [s]）。"""

    t: np.ndarray                   #: (n,)
    u: np.ndarray                   #: (n, 2) 前輪エンコーダの生の読み（ADC÷フルスケール）[FL, FR]
    rear: np.ndarray                #: (n, 2) 後輪の周速 [m/s] [RL, RR]（STM32 が `rear_mult` の半径を掛けた値）
    speed: np.ndarray               #: (n,) STM32 の車速（区間の切り分けにだけ使う）
    steer: np.ndarray               #: (n,) 路面舵角 [rad]
    yaw_rate: np.ndarray            #: (n,) [rad/s]
    phase: np.ndarray               #: (n,) `OTHER`・`RUN`・`COAST`・`BRAKE`
    #: スキャン `(測り終えた時刻, base_link の点群)`
    scans: list[tuple[float, Points]] = field(default_factory=list)
    #: 記録したときに STM32 が後輪の周速に掛けていた半径 [m] [RL, RR]。None は記録から分からない
    rear_radius: tuple[float, float] | None = (_NOMINAL_RADIUS_M, _NOMINAL_RADIUS_M)


def _phase_of(cmd: dict) -> int:
    if cmd.get("brake"):
        return BRAKE
    if cmd.get("torque_mode"):
        return COAST if abs(float(cmd.get("target_torque", 0.0))) < 1e-6 else OTHER
    return RUN


def build_record(vs_msgs: Sequence[tuple[float, dict]], cmds: Sequence[tuple[float, dict]],
                 scans: Sequence[tuple[float, Points]] = (),
                 rear_radius: tuple[float, float] | None = (_NOMINAL_RADIUS_M, _NOMINAL_RADIUS_M)) -> Record:
    """時刻 [s]（同じ時間軸）つきの `/vehicle_state`・`/cmd`（dict）・点群 → `Record`。ベンチも使う。"""
    vs_msgs = sorted(vs_msgs, key=lambda x: x[0])
    cmds = sorted(cmds, key=lambda x: x[0])
    if not vs_msgs:
        raise ValueError("記録に /vehicle_state がありません")
    t0 = vs_msgs[0][0]
    ct = np.array([c[0] for c in cmds])
    n = len(vs_msgs)
    t = np.empty(n)
    u = np.empty((n, 2))
    rear = np.empty((n, 2))
    speed, steer, yaw, phase = (np.empty(n) for _ in range(4))
    for i, (ti, vs) in enumerate(vs_msgs):
        j = int(np.searchsorted(ct, ti, side="right")) - 1
        t[i] = ti - t0
        fa = vs.get("front_angle") or [0.0, 0.0]
        u[i] = [float(fa[0]) / TWO_PI, float(fa[1]) / TWO_PI]
        ws = vs.get("wheel_speed") or [0.0] * 4
        rear[i] = [float(ws[2]), float(ws[3])] if len(ws) >= 4 else [0.0, 0.0]
        speed[i] = float(vs.get("speed", 0.0))
        steer[i] = float(vs.get("steer_actual", 0.0))
        yaw[i] = float(vs.get("yaw_rate", 0.0))
        phase[i] = _phase_of(cmds[j][1]) if j >= 0 else OTHER
    return Record(t=t, u=u, rear=rear, speed=speed, steer=steer, yaw_rate=yaw, phase=phase.astype(int),
                  scans=sorted(((ts - t0, p) for ts, p in scans), key=lambda x: x[0]),
                  rear_radius=rear_radius)


def load_record(path: str | Path, vehicle: Vehicle | None = None) -> Record:
    """mcap → `Record`。時刻は `fit.load_log` と同じくメッセージの中の単調時刻。"""
    from mcap.reader import make_reader

    veh = vehicle or Vehicle.load()
    vs_msgs: list[tuple[float, dict]] = []
    cmds: list[tuple[float, dict]] = []
    scans: list[tuple[float, Points]] = []
    params: dict[str, float] = {}
    proto = None
    with open(path, "rb") as f:
        for _schema, channel, message in make_reader(f).iter_messages(
                topics=["/cmd", "/vehicle_state", "/scan", "/diag/link"]):
            obj = json.loads(message.data)
            if channel.topic == "/vehicle_state":
                vs_msgs.append((int(obj.get("t_capture", 0)) / fit.NS, obj))
            elif channel.topic == "/cmd":
                cmds.append((int(obj.get("t_pub", 0)) / fit.NS, obj))
            elif channel.topic == "/diag/link":
                proto = proto if proto is not None else obj.get("protocol_version")
                params.update({k: float(v) for k, v in (obj.get("control_params") or {}).items()
                               if isinstance(v, (int, float))})
            else:
                scan = Scan(**{k: obj[k] for k in ("dist", "sector_seen", "sector_t_ns", "sector_dur_us",
                                                   "seq", "t_capture", "saturated") if k in obj})
                pts = deskew(scan, 0.0, 0.0, mount_x=veh.lidar_x, mount_y=veh.lidar_y)
                durs = obj.get("sector_dur_us") or []
                t_done = max((int(x) + (int(durs[i]) * 1000 if i < len(durs) else 0)
                              for i, x in enumerate(obj.get("sector_t_ns", [])) if x), default=0)
                if t_done:
                    scans.append((t_done / fit.NS, pts))
    if not vs_msgs or not all(t for t, _ in vs_msgs):
        raise ValueError("記録の /vehicle_state に t_capture がありません（古い形式の記録は使えません）")
    keys = ("rear_radius_left_m", "rear_radius_right_m")
    if all(k in params for k in keys):
        rr: tuple[float, float] | None = (params[keys[0]], params[keys[1]])
    elif proto is not None and proto < _PROTO_RADIUS:
        rr = (_NOMINAL_RADIUS_M, _NOMINAL_RADIUS_M)
    else:
        rr = None
    return build_record(vs_msgs, cmds, scans, rr)


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


def hold_segments(rec: Record) -> list[np.ndarray]:
    """止まっている区間（試験の頭の静止と、各片道の端の制動）。頭の `_HOLD_SKIP_S` は捨てる。"""
    out = []
    still = (np.abs(rec.speed) < _HOLD_SPEED) & (rec.phase != COAST)
    for a, b in _runs(still):
        idx = np.arange(a, b)
        idx = idx[rec.t[idx] >= rec.t[a] + _HOLD_SKIP_S]
        if idx.size >= 5 and rec.t[idx[-1]] - rec.t[idx[0]] >= _HOLD_MIN_S - _HOLD_SKIP_S:
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


# ── 前輪の半径（スキャン）────────────────────────────────────────────────

@dataclass
class Leg:
    t0: float
    t1: float
    #: 補正後の回転角 [rad] [FL, FR]（符号つき。取付向きの符号のまま）
    dtheta: tuple[float, float]
    #: 前車軸の中心の移動量 [m]（進んだ向きが正）と回頭 [rad]
    s: float
    dyaw: float
    ok: bool
    inlier: float


def _hold_angle(theta_u: np.ndarray, idx: np.ndarray) -> float:
    return float(np.median(theta_u[idx]))


def _scans_in(rec: Record, idx: np.ndarray, n: int = 3) -> list[Points]:
    a, b = rec.t[idx[0]], rec.t[idx[-1]]
    pts = [p for t, p in rec.scans if a <= t <= b]
    if len(pts) <= n:
        return pts
    mid = len(pts) // 2
    return pts[max(0, mid - n // 2):max(0, mid - n // 2) + n]


def legs(rec: Record, cals: dict[str, Cal], vehicle: Vehicle, r_guess: float) -> list[Leg]:
    """止まっている区間どうし（＝片道）ごとの、前輪の回転角とスキャンで測った移動量。"""
    holds = hold_segments(rec)
    th = {s: unwrap_angle(cals[s].angle(rec.u[:, i])) for i, s in enumerate(SIDES)}
    bias = float(np.median(np.concatenate([rec.yaw_rate[h] for h in holds]))) if holds else 0.0
    pf = np.array([vehicle.wheelbase, 0.0])
    out = []
    for h0, h1 in zip(holds, holds[1:]):
        d = tuple(_hold_angle(th[s], h1) - _hold_angle(th[s], h0) for s in SIDES)
        a, b = h0[-1], h1[0]
        sgn = float(np.sign(np.sum(rec.speed[a:b])) or 1.0)
        dyaw_gyro = float(np.sum((rec.yaw_rate[a:b] - bias) * np.diff(rec.t[a:b + 1])))
        s_guess = sgn * r_guess * 0.5 * (abs(d[0]) + abs(d[1]))
        prev, cur = _scans_in(rec, h0), _scans_in(rec, h1)
        res = []
        for p in prev:
            for c in cur:
                m = match_scans(p, c, (s_guess, 0.0, dyaw_gyro), iters=40,
                                prior_xy=0.2, prior_yaw=math.radians(3.0))
                if m.ok:
                    res.append((m.dx, m.dy, m.dyaw, m.inlier))
        if not res:
            out.append(Leg(rec.t[a], rec.t[b], d, s_guess, dyaw_gyro, False, 0.0))
            continue
        dx, dy, dyaw, inl = (float(np.median([r[k] for r in res])) for k in range(4))
        c, s_ = math.cos(dyaw), math.sin(dyaw)
        moved = np.array([c * pf[0] - s_ * pf[1] + dx, s_ * pf[0] + c * pf[1] + dy]) - pf
        out.append(Leg(rec.t[a], rec.t[b], d, sgn * float(np.hypot(*moved)), dyaw, True, inl))
    return out


def front_radii(lg: list[Leg], track: float) -> tuple[dict[str, float], dict[str, float], list[str]]:
    """片道ごとの (移動量, 回転角) → 左右の前輪の半径と標準誤差。"""
    notes = []
    good = [x for x in lg if x.ok and abs(x.s) > 0.3]
    if not good:
        raise ValueError("止まっている間のスキャンどうしが合わず、前輪の半径を決められません（前後に壁・物が"
                         "見える場所で録り直してください）")
    r, se = {}, {}
    for i, side in enumerate(SIDES):
        off = (-0.5 if side == "left" else 0.5) * track
        path = np.array([abs(x.s + off * x.dyaw) for x in good])
        ang = np.array([abs(x.dtheta[i]) for x in good])
        r[side] = float(path.sum() / ang.sum())
        per = path / ang
        se[side] = float(np.std(per, ddof=1) / math.sqrt(len(per))) if len(per) > 1 else math.nan
        fwd = [p for p, x in zip(per, good) if x.s > 0]
        rev = [p for p, x in zip(per, good) if x.s < 0]
        if fwd and rev:
            notes.append(f"前輪の半径（{_JP[side]}）: 前進 {np.mean(fwd) * 1000:.3f}mm・後退 {np.mean(rev) * 1000:.3f}mm"
                         f"（片道 {len(per)} 本のばらつき ±{np.std(per) * 1000:.3f}mm）")
    if len(good) < len(lg):
        notes.append(f"★スキャンが合わなかった片道 {len(lg) - len(good)} 本を除いた（{len(lg)} 本中）")
    return r, se, notes


# ── 後輪の半径（惰行）────────────────────────────────────────────────────

def rear_radii(rec: Record, segs: list[np.ndarray], cals: dict[str, Cal], r_front: dict[str, float],
               vehicle: Vehicle) -> tuple[dict[str, float], list[str]]:
    """惰行中の後輪の周速 = (後輪が今の半径で報告する倍率) × 真の周速 を当てはめる。"""
    notes: list[str] = []
    if rec.rear_radius is None:
        raise ValueError("記録に STM32 の rear_radius_*（/diag/link の control_params）が無く、後輪の周速に"
                         "掛かっていた半径が分からない。/diag/link を含めて録り直してください")
    holds = hold_segments(rec)
    bias = float(np.median(np.concatenate([rec.yaw_rate[h] for h in holds]))) if holds else 0.0
    half = 0.5 * vehicle.track
    th = {s: unwrap_angle(cals[s].angle(rec.u[:, i])) for i, s in enumerate(SIDES)}
    xs: dict[str, list[np.ndarray]] = {s: [] for s in SIDES}
    ys: dict[str, list[np.ndarray]] = {s: [] for s in SIDES}
    for idx in segs:
        t = rec.t[idx]
        # 前輪の周速（取付向きの符号は進む向きにそろえる）
        sgn = float(np.sign(np.median(rec.speed[idx])))
        vf = []
        for s in SIDES:
            w = fit._odom_slope(t, th[s][idx])
            vf.append(np.abs(w) * r_front[s])
        v_fc = 0.5 * (vf[0] + vf[1])
        v_rc = sgn * v_fc * np.cos(rec.steer[idx])
        omega = rec.yaw_rate[idx] - bias
        a = fit._odom_slope(t, v_rc)
        # 区間の端は微分の窓が片側だけになるので捨てる
        keep = (t >= t[0] + fit._ODOM_DIFF_HALF_S) & (t <= t[-1] - fit._ODOM_DIFF_HALF_S)
        for i, s in enumerate(SIDES):
            v = v_rc + (-half if s == "left" else half) * omega
            xs[s].append(np.stack([v, a], axis=1)[keep])
            ys[s].append(rec.rear[idx, i][keep])
    out = {}
    for i, s in enumerate(SIDES):
        X, y = np.concatenate(xs[s]), np.concatenate(ys[s])
        # 倍率は遅れを持たないモデルで決める。遅れの列（加速度）は惰行中ほぼ一定で、倍率と分けにくく
        # 値がばらつく（ベンチで ±0.3%）。遅れ τ があると倍率は τ·a/v（5ms・0.27m/s²・0.6m/s で 0.2%）
        # ずれるが、後輪の半径はスリップ率の基準（TC の目標 20%）なので効かない
        k = float(X[:, 0] @ y / (X[:, 0] @ X[:, 0]))
        (k2, c), *_ = np.linalg.lstsq(X, y, rcond=None)
        out[s] = float(rec.rear_radius[i] / k)
        lag = -c / k2
        notes.append(f"後輪の半径（{_JP[s]}）: {out[s] * 1000:.3f}mm（記録時の設定 {rec.rear_radius[i] * 1000:.3f}mm。"
                     f"周速の遅れの目安 {lag * 1000:+.1f}ms）")
        if abs(lag) > _REAR_LAG_MAX_S:
            notes.append(f"★後輪（{_JP[s]}）の周速が前輪に {lag * 1000:+.0f}ms ずれている。MD の速度の読みか"
                         "惰行の判定がおかしい")
    return out, notes


# ── まとめ ──────────────────────────────────────────────────────────────────

@dataclass
class Result:
    values: dict[str, float]
    notes: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    wheels: dict[str, WheelFit] = field(default_factory=dict)
    legs: list[Leg] = field(default_factory=list)


def _merge(records: Sequence[Record]) -> Record:
    if len(records) == 1:
        return records[0]
    rr = {r.rear_radius for r in records}
    if len(rr) > 1:
        raise ValueError("記録によって STM32 の rear_radius_* が違います。同じ設定で録った記録だけを選んでください")
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
    return Record(t=t, u=np.concatenate([r.u for r, _ in parts]),
                  rear=np.concatenate([r.rear for r, _ in parts]), speed=speed,
                  steer=np.concatenate([r.steer for r, _ in parts]),
                  yaw_rate=np.concatenate([r.yaw_rate for r, _ in parts]), phase=phase,
                  scans=[(ts + o, p) for r, o in parts for ts, p in r.scans], rear_radius=rr.pop())


def _speed_split(rec: Record, segs: list[np.ndarray]) -> tuple[list[np.ndarray], list[np.ndarray]]:
    v = np.array([float(np.median(np.abs(rec.speed[idx]))) for idx in segs])
    mid = float(np.median(v)) if v.size else 0.0
    return [s for s, x in zip(segs, v) if x < mid], [s for s, x in zip(segs, v) if x >= mid]


def analyze(records: Record | Sequence[Record], vehicle: Vehicle | None = None) -> Result:
    """記録（1本以上。同じ車・同じ設定）→ `[control]` に書く値と所見。"""
    recs = [records] if isinstance(records, Record) else list(records)
    veh = vehicle or Vehicle.load()
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
    cals = {s: out.wheels[s].cal for s in SIDES}
    lg = legs(rec, cals, veh, veh.wheel_radius)
    out.legs = lg
    try:
        rf, rf_se, notes = front_radii(lg, veh.track)
        out.notes += notes
        for s in SIDES:
            out.values[f"front_radius_{s}_m"] = round(rf[s], 7)
            out.notes.append(f"前輪の半径（{_JP[s]}）: {rf[s] * 1000:.3f}mm ± {rf_se[s] * 1000:.3f}mm"
                             f"（名目 {veh.wheel_radius * 1000:.1f}mm から {(rf[s] / veh.wheel_radius - 1) * 100:+.2f}%）")
    except ValueError as e:
        out.warnings.append(f"★{e}")
        rf = {s: veh.wheel_radius for s in SIDES}
    try:
        rr, notes = rear_radii(rec, segs, cals, rf, veh)
        out.notes += notes
        out.values.update({f"rear_radius_{s}_m": round(rr[s], 7) for s in SIDES})
    except ValueError as e:
        out.warnings.append(f"★{e}")
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
