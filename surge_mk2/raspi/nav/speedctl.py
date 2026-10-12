"""速度指令 — 経路の速度プロファイルを、STM32 の速度PIの遅れを見込んで前倒しして送る。

## なぜ要るか

STM32 は目標速度をランプ（3.0m/s²）でならし、PI（kp・ki）でトルクにする。比例項だけで見ると
加速度 `a` を出すには「目標 − 車速」が `a × T_v`（`T_v = 1/(車体の利得×kp)` ≒ 0.16s）だけ要る。
**経路の速度をそのまま送ると、車速はその分だけ遅れる。** 加速では遅いだけだが、減速では
コーナーへ速すぎる速度で入る（実車ヘアピン進入で目標 0.7 に対し 1.2〜1.3m/s。
ファームの写し `sim.vehicle.SpeedController` に 1 次元で当てても +0.48m/s）。

## やっていること（`SpeedFF.command`）

    目標 = v(s') + T_v·HP[a(s')] + k_fb·(v(s) − 車速)

- `v(s)`・`a(s)`: 経路の速度プロファイルと、その傾きから出した加速度（`a = v·dv/ds`）
- `T_v·a`: PI の比例項の逆算。等加速度で走っている間の遅れを消す
- `HP[·]`: 時定数 `kp/ki` のハイパス。長く続く加減速では PI の積分が比例項を引き継ぐので、
  足したままにすると行き過ぎる（線形の範囲では、これが PI＋車体の厳密な逆モデルになる）
- `s'`: **減速は `lead_s` 秒ぶん先、加速は `lead_acc_s` 秒ぶん先**の点。目標を急に下げても
  STM32 のランプが 3.0m/s² でしか追わないので、比例項が溜まるのに時間がかかる。減速だけ
  早めに始め、加速は早めない（コーナーの出口で早く踏むと、まだ曲がっている所で速度が出る）。
  2つの候補のうち**目標が低くなる方**を採る
- `k_fb`: 車速がプロファイルから外れたぶんを目標に足し戻す（同定値のずれ・TC の介入の保険。
  「今の速度では次のコーナーに間に合わない」とき、目標がさらに下がって PI が最大減速へ張り付く）

値は `Vehicle`（`config/vehicle.toml` の同定値）から決まり、手で合わせる数字は `lead` の係数だけ
（`config_from_vehicle`）。`raspi/tests/test_speedctl.py` がファームの写しに対する追従を確かめる。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .raceline import RaceLine

__all__ = ["SpeedFFConfig", "SpeedFF", "config_from_vehicle", "path_accel", "arc_index",
           "catch_up", "ref_command"]

#: 減速の前倒し `lead_s` を速度ループの時定数の何倍にするか。ファームの写しに対する 1 次元の実験
#: （直線＋一定速コーナー、指令 50Hz・遅れ 25ms）で、0.65 倍（0.1s）がコーナー進入の出過ぎ・追従の
#: 誤差とも最小だった（0 倍: 出過ぎ +0.49m/s、0.65 倍: +0.12、1.3 倍: 早く落としすぎて 1 周 +2%）。
#: kp を 0.3 に・加減速を 85% にした車でも同じ所が最良
LEAD_RATIO = 0.65
#: 加速の前倒し [s]（指令が届くまでの遅れぶんだけ）
LEAD_ACC_S = 0.05


class SpeedFFConfig(NamedTuple):
    tau_v: float = 0.0          #: 速度PIの比例項の時定数 [s]（0 = 前倒ししない）
    reset_s: float = 0.0        #: 積分が比例項を引き継ぐ時定数 kp/ki [s]（0 = ハイパスなし）
    lead_s: float = 0.0         #: 減速の前倒し [s]
    lead_acc_s: float = 0.0     #: 加速の前倒し [s]
    k_fb: float = 1.0           #: 車速のずれを目標へ戻す重み


def config_from_vehicle(vehicle, k_fb: float = 1.0) -> SpeedFFConfig:
    tau = float(vehicle.speed_loop_tau_s)
    return SpeedFFConfig(tau_v=tau, reset_s=float(vehicle.speed_loop_reset_s),
                         lead_s=LEAD_RATIO * tau, lead_acc_s=LEAD_ACC_S if tau > 0.0 else 0.0,
                         k_fb=k_fb)


def path_accel(path: RaceLine) -> np.ndarray:
    """経路の各点 i → i+1 の加速度 [m/s²]（速度プロファイルの傾きから。`a = Δ(v²)/(2Δs)`）。"""
    n = len(path)
    if n < 2:
        return np.zeros(n)
    v = np.asarray(path.v, dtype=np.float64)
    if path.closed:
        nxt = np.roll(path.xy, -1, axis=0)
        v1 = np.roll(v, -1)
    else:
        nxt = np.vstack([path.xy[1:], path.xy[-1:]])
        v1 = np.append(v[1:], v[-1])
    ds = np.maximum(np.hypot(*(nxt - path.xy).T), 1e-6)
    a = (v1 * v1 - v * v) / (2.0 * ds)
    if not path.closed:
        a[-1] = 0.0
    return a


#: STM32 が目標車速の傾きに掛けるローパスの時定数 [s]（**ファームの定数** `drive.h` の
#: DRIVE_SPEED_FF_TAU_S。変えたらここと `sim/vehicle.py` の `SpeedController.FF_TAU_S` も直す）
FW_FF_TAU_S = 0.02
#: ファームの加速度フィードフォワードが有効なときの、目標車速のランプの下限 [m/s²]（`ref_command`）
REF_ACCEL_MIN = 0.3
#: 同じく、ランプをプロファイルの加速度の何倍にするか（1 より少し大きく。目標が先へ逃げない）
REF_ACCEL_GAIN = 1.1
#: 同じく、車速のずれをこの時間で詰められるランプまで許す [s]
REF_CATCH_S = 0.15


def ref_command(path: RaceLine, pos: float, v_now: float, *, lead_s: float,
                acc: np.ndarray | None = None) -> tuple[float, float]:
    """ファームの加速度フィードフォワード（★v0.21）が有効なときの `(目標速度, ランプの加速度)`。

    STM32 は目標車速を `accel_limit` の傾きで動かし、**その傾きに要るトルクを先に足す**
    （`drive.c`、`[control]` の `speed_ff_*`）。だから Pi は「プロファイルそのもの」を渡せばよい:

    - 目標速度 = 指令が届くころ（`lead_s` 秒後）に居る場所のプロファイルの速度
    - ランプ = そこでのプロファイルの加速度（の `REF_ACCEL_GAIN` 倍）

    STM32 の目標はプロファイルと同じ傾きで動き、車速PIは残ったずれだけを直す。`SpeedFF` の
    前倒し（経験則の係数）・比例項の逆算・ハイパスは要らない。車速がプロファイルから外れている
    ときは、その差を `REF_CATCH_S` で詰められるランプまで許す（速すぎるときに減速が遅れない）。
    """
    n = len(path)
    if n < 2:
        return (float(path.v[0]) if n else 0.0), 0.0
    closed = path.closed
    step = max(path.length / (n if closed else max(1, n - 1)), 1e-6)
    if acc is None:
        acc = path_accel(path)
    q = pos + max(0.0, v_now) * lead_s / step
    v_ref = _lerp(path.v, q, closed)
    a = abs(_lerp(acc, q, closed))
    ramp = max(REF_ACCEL_MIN, REF_ACCEL_GAIN * a, abs(v_ref - v_now) / REF_CATCH_S)
    return max(0.0, v_ref), ramp


#: `catch_up` が先を見る距離 [m]
CATCH_UP_M = 6.0


def catch_up(path: RaceLine, index: int, v_now: float, *, a_accel: float, a_grip,
             ahead_m: float = CATCH_UP_M) -> RaceLine:
    """今の車速から**摩擦円の中で加速して**プロファイルに追いつくまでの速度を付けた経路。

    ★ 経路の速度は「1周を通して走っている」前提で引いてある。迂回の後・走り出し・障害物で減速
    した後は、車速がそれよりずっと低い。そのまま追わせると（`SpeedFF` の `k_fb`）速度PIが全開に
    なり、次のコーナーへ**全開で加速しながら**入る——横に要る力と前後に使う力が同時に限界を
    超え、外へ膨らんで壁に当たった（sim.bench の toyota2、迂回の後）。目標を下げる
    だけでは、速度PIの積分が溜まっていて加速が止まらない。そこで `index` から先を、今の車速から
    出発して「横に使っているぶんを引いた加速度」で届く速度に置き換える（前向きの 1 パス）。
    コーナーの手前で加速が自然に細り、プロファイルに合流した先は元のまま。

    点の数と添字は元の経路と同じ。`index` より後ろ・`ahead_m` より先は元の速度。
    """
    n = len(path)
    if n < 3:
        return path
    closed = path.closed
    step = max(path.length / (n if closed else max(1, n - 1)), 1e-6)
    kap = np.abs(path.kappa) if len(path.kappa) == n else np.zeros(n)
    v = np.array(path.v, dtype=np.float64, copy=True)
    # `a_grip` は点ごとの配列でもよい（走行中に学習した横グリップ、`nav/grip.py`）
    grip = np.broadcast_to(np.asarray(a_grip, dtype=np.float64), (n,))
    vi = max(float(v_now), 0.05)
    i = index % n if closed else min(max(index, 0), n - 1)
    for _ in range(max(1, int(ahead_m / step))):
        if vi >= v[i]:
            break                              # プロファイルに追いついた
        v[i] = vi
        ay = vi * vi * float(kap[i])
        g = float(grip[i])
        r = 1.0 - ay * ay / (g * g) if g > 1e-9 else 1.0
        vi = math.sqrt(vi * vi + 2.0 * (a_accel * math.sqrt(r) if r > 0.0 else 0.0) * step)
        i += 1
        if closed:
            i %= n
        elif i >= n:
            break
    return RaceLine(**{**path._asdict(), "v": v})


def arc_index(path: RaceLine, i: int, x: float, y: float) -> float:
    """点 `(x, y)` の経路上の位置を「添字＋端数」で返す（最寄り点 `i` の前後の線分へ射影）。

    添字だけで読むと 0.1m 刻みで速度が段になり、50Hz で読むたびに加速度が跳ねる。
    """
    n = len(path)
    if n < 2:
        return float(i)
    xy = path.xy

    def seg(a: int, b: int) -> tuple[float, float]:
        ax, ay = xy[a]
        dx, dy = xy[b, 0] - ax, xy[b, 1] - ay
        d2 = dx * dx + dy * dy
        t = ((x - ax) * dx + (y - ay) * dy) / d2 if d2 > 1e-12 else 0.0
        t = max(0.0, min(1.0, t))
        return t, (ax + t * dx - x) ** 2 + (ay + t * dy - y) ** 2

    if path.closed:
        prev, nxt = (i - 1) % n, (i + 1) % n
    else:
        prev, nxt = max(i - 1, 0), min(i + 1, n - 1)
    best = float(i)
    best_d = math.inf
    if nxt != i:
        t, d = seg(i, nxt)
        best, best_d = i + t, d
    if prev != i:
        t, d = seg(prev, i)
        if d < best_d:
            best = (i - 1) + t
    return best


def _lerp(arr: np.ndarray, pos: float, closed: bool) -> float:
    n = len(arr)
    if closed:
        pos %= n
        i = int(pos)
        f = pos - i
        return float(arr[i] * (1.0 - f) + arr[(i + 1) % n] * f)
    pos = max(0.0, min(float(n - 1), pos))
    i = min(int(pos), n - 2) if n >= 2 else 0
    f = pos - i
    return float(arr[i] * (1.0 - f) + arr[min(i + 1, n - 1)] * f)


class SpeedFF:
    """速度指令の前倒し（モジュール docstring）。経路が変わったら加速度の表を作り直す。"""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._hp = 0.0
        self._a_prev: float | None = None
        self._v_ref: np.ndarray | None = None      # 加速度の表を作った `path.v`（同一性で見る）
        self._acc: np.ndarray | None = None

    def command(self, path: RaceLine, pos: float, v_now: float, dt: float,
                cfg: SpeedFFConfig, a_room: float | None = None) -> float:
        """目標速度 [m/s]。

        :param pos: 今の車の経路上の位置（`arc_index`）
        :param dt: 前回呼んでからの時間 [s]（ハイパスの刻み。0.5s を超えたら状態を捨てる）
        :param a_room: 今この場所で前へ使ってよい加速度 [m/s²]（摩擦円の残り）。与えると、
            目標を「車速 ＋ `tau_v`×`a_room`」までに抑える——★ 車速がプロファイルよりずっと低い
            とき（迂回の後・走り出し・減速した後）、`k_fb` の項が大きな目標を出して速度PIを全開に
            する。プロファイルは「そこを巡航で通る」前提で引いてあるので、コーナーの中でも全開に
            なり、横のグリップが足りずに外へ膨らんで壁に当たった（sim.bench の toyota2、迂回の後）。プロファイルどおりに走っている間は効かない（同じ摩擦円で引いてある）
        """
        n = len(path)
        if n < 2:
            return float(path.v[0]) if n else 0.0
        if self._v_ref is not path.v:
            self._acc = path_accel(path)
            self._v_ref = path.v
            self._a_prev = None                # 別の経路の加速度との差をハイパスに入れない
        acc = self._acc
        closed = path.closed
        step = max(path.length / (n if closed else max(1, n - 1)), 1e-6)
        v_here = _lerp(path.v, pos, closed)
        if cfg.tau_v <= 0.0:
            return v_here

        v_abs = max(0.0, v_now)
        best: tuple[float, float, float] | None = None
        for lead in (cfg.lead_s, cfg.lead_acc_s):
            q = pos + v_abs * lead / step
            vq, aq = _lerp(path.v, q, closed), _lerp(acc, q, closed)
            cand = vq + cfg.tau_v * aq
            if best is None or cand < best[0]:
                best = (cand, vq, aq)
        _, v_ref, a = best

        if dt <= 0.0 or dt > 0.5 or self._a_prev is None:
            self._hp = a                       # 走り出し・間が空いた後は「今の加速度が始まった所」
        elif cfg.reset_s > 0.0:
            k = math.exp(-dt / cfg.reset_s)
            self._hp = k * (self._hp + a - self._a_prev)
        else:
            self._hp = a
        self._a_prev = a
        tgt = v_ref + cfg.tau_v * self._hp + cfg.k_fb * (v_here - v_now)
        if a_room is not None:
            tgt = min(tgt, v_now + cfg.tau_v * max(a_room, 0.0))
        return max(0.0, tgt)
