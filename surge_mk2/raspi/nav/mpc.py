"""MPC 経路追従 — 経路座標の線形化自転車モデルで、舵角の列を最適化する。

Pure Pursuit（`purepursuit.py`）の代わりに舵だけを決める。速度は経路の速度プロファイルを
そのまま使う（速度PI・TC は STM32）。`follow()` と同じ引数・同じ `Pursuit` を返すので、
planner が差し替えるだけで比較できる。

## モデル（base_link = 後輪車軸中心）

経路に対する横偏差 `e_y`（左が正）と向きの偏差 `e_psi`、舵角 `δ`（実効舵角）:

    e_y'   = v·sin(e_psi)
    e_psi' = v·tan(δ)/L − κ_ref·v/(1 − κ_ref·e_y)

経路の曲率 `κ_ref` の先で `δ_ref = atan(L·κ_ref)`（曲率を丸ごと打ち消す舵角）の周りに
線形化する。`e=0, δ=δ_ref` が平衡点なので、フィードフォワードは別に要らない。

    e_{k+1} = A_k·e_k + B_k·(δ_k − δ_ref_k)
    A = [[1, v·dt], [−v·dt·κ², 1]],  B = [0, v·dt·sec²(δ_ref)/L]

舵の1次遅れ（τ≈1ms）は無視し、**むだ時間は初期状態を `delay_s` 先へ進めて**扱う
（`purepursuit._predict`。Pure Pursuit と同じ遅延補償）。

## 最適化

決定変数は各ステップの舵角 `δ_0..δ_{N-1}`（実効舵角）。コストは

    Σ (q_y·e_y² + q_psi·e_psi²)（最終ステップは ×3） + r_u·Σ(δ−δ_ref)² + r_d·Σ(Δδ)²

を `numpy` だけで凝縮した QP（N×N）にし、箱制約（`|δ| ≤ 限界`、前回の舵角から舵レートで
届く範囲）を**射影ニュートン法**（有効制約を固定して残りを解く）で解く。FISTA は条件数が悪く40反復でも収束せず、舵が振動した。scipy・cvxpy は Pi の依存に無いので使わない。
まず制約なしの解析解を箱へ射影して初期値にするので、制約が効かない場面（ほとんど）は
その場で収束する。前回解は1つずらして次周期の初期値の候補にする。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

from .purepursuit import Pursuit, PursuitConfig, SteerMap, _predict, follow, nearest_index
from .raceline import RaceLine

__all__ = ["MPCConfig", "MPCFollower"]

#: モデルに使う速度の下限 [m/s]（停止中・量子化された低速で時間刻みの弧長が潰れないように）
V_FLOOR = 0.3


class MPCConfig(NamedTuple):
    wheelbase: float = 0.23
    max_steer: float = 0.46175             #: 指令舵角の上限 [rad]（`Vehicle.max_steer`）
    steer_rate: float = 7.0                #: 舵レートの上限 [rad/s]
    horizon: int = 20                      #: 予測ステップ数 N
    dt: float = 0.1                        #: 1ステップの時間 [s]
    delay_s: float = 0.06                  #: 遅延補償の時間 [s]
    period: float = 0.1                    #: 制御周期 [s]（LiDAR 10Hz。出した指令を覚える間隔）
    q_y: float = 100.0                     #: 横偏差の重み [1/m²]
    q_psi: float = 30.0                    #: 向き偏差の重み [1/rad²]
    r_u: float = 0.1                       #: 経路の曲率に要る舵角からのずれの重み
    r_d: float = 5.0                       #: 舵角の変化（ステップ間）の重み
    terminal: float = 3.0                  #: 最終ステップの状態重みの倍率
    iters: int = 12                        #: 射影ニュートンの反復上限
    steer_map: SteerMap = SteerMap()
    servo_gain: float = 1.0                #: 指令→実舵角の定常ゲイン（箱の上限に使う）
    #: 速度を読む先読み時間 [s]（`PursuitConfig.speed_preview_s`）
    speed_preview_s: float | None = None


class MPCFollower:
    """前回解を持つ MPC 追従。planner が1つ持ち、`reset()` で捨てる。"""

    def __init__(self) -> None:
        self._warm: np.ndarray | None = None
        #: 出した実効舵角の履歴 `(出した時刻 [s], 舵角)`（新しい順）。遅延の間にまだ効いていない
        #: 指令を予測に入れる。★ 時刻で持つ。「呼ばれるたびに1周期進む」
        #: 前提の並びだと、同じ周期に2回呼ばれる（回避経路へ乗り換えた周期）と履歴が2つ進み、
        #: 呼ぶ周期を変える（点群の合間の 50Hz）こともできない
        self._sent: list[tuple[float, float]] = []
        self._clock = 0.0                  # `now` を渡さない呼び出し用の内部時計
        #: 直近の解の診断（GUI・ベンチ用）
        self.last_iters = 0
        self.last_cost = 0.0

    def reset(self) -> None:
        self._warm = None
        self._sent = []
        self._clock = 0.0

    def _predict_delay(self, pose, v, delta_now, cfg: MPCConfig, now: float):
        """`delay_s` 先の姿勢。まだ効いていない自分の指令を順に当てはめる（Smith 予測器）。

        指令は `delay_s` 遅れて効く。`age` 秒前に出した指令は、`delay_s − age` 後に効き始める
        （`purepursuit._predict` は今の舵角のまま進めるので、舵が速く動くと予測が外れて振動した）。
        """
        x, y, yaw = pose
        t, delta = 0.0, delta_now
        pend = [(cfg.delay_s - (now - ts), d) for ts, d in self._sent]
        pend = [(te, d) for te, d in reversed(pend) if te > 1e-9]   # 効き始めの早い順
        for te, d in pend + [(cfg.delay_s, None)]:
            te = min(te, cfg.delay_s)
            if te > t:
                x, y, yaw = _predict(x, y, yaw, v, delta, cfg.wheelbase, te - t)
                t = te
            if d is not None:
                delta = d
        return x, y, yaw

    # ── 経路の切り出し ──

    @staticmethod
    def _horizon(path: RaceLine, i: int, speeds: np.ndarray, cfg: MPCConfig):
        """各ステップの（経路の添字, 速度, 曲率, 経路の向き）。弧長 `v·dt` ずつ進める。"""
        n = len(path)
        step = max(path.length / (n if path.closed else max(1, n - 1)), 1e-6)
        idx = np.empty(cfg.horizon, dtype=int)
        past_end = np.zeros(cfg.horizon, dtype=bool)
        s = 0.0
        for k in range(cfg.horizon):
            j = i + int(round(s / step))
            idx[k] = j % n if path.closed else min(j, n - 1)
            past_end[k] = (not path.closed) and j > n - 1
            v = max(float(path.v[idx[k]]), V_FLOOR)
            speeds[k] = v
            s += v * cfg.dt
        kap = path.kappa[idx].astype(float)
        kap = np.where(past_end, 0.0, kap)        # 開経路の終点の先は直進（`follow` と同じ）
        return idx, kap

    def follow(self, path: RaceLine, pose: tuple[float, float, float], v_now: float,
               steer_now: float, cfg: MPCConfig, *, hint: int = -1,
               now: float | None = None) -> Pursuit:
        """:param now: 今の時刻 [s]（単調増加なら何でもよい。車両状態の取得時刻を渡す）。
            `None` なら「呼ばれるたびに `cfg.period` 進む」とみなす（単体の閉ループ試験用）"""
        if now is None:
            self._clock += cfg.period
            now = self._clock
        sm = cfg.steer_map
        pcfg = PursuitConfig(wheelbase=cfg.wheelbase, max_steer=cfg.max_steer,
                             delay_s=cfg.delay_s, speed_preview_s=cfg.speed_preview_s,
                             steer_map=sm)
        # 添字・横偏差・速度・残りは Pure Pursuit と同じ定義（比較できるように）
        base = follow(path, pose, v_now, steer_now, pcfg, hint=hint)

        delta_now = sm.effective(steer_now)
        px, py, pyaw = self._predict_delay(pose, v_now, delta_now, cfg, now)
        n = len(path)
        i = nearest_index(path, px, py, hint, window=n // 4 if hint >= 0 else 0)
        if path.closed:
            t = path.xy[(i + 1) % n] - path.xy[i - 1]
        else:
            t = path.xy[min(i + 1, n - 1)] - path.xy[max(i - 1, 0)]
        tn = math.hypot(*t)
        if tn < 1e-9:
            return base
        ty = math.atan2(t[1], t[0])
        e_y = (t[0] * (py - path.xy[i, 1]) - t[1] * (px - path.xy[i, 0])) / tn
        e_psi = (pyaw - ty + math.pi) % (2.0 * math.pi) - math.pi
        e_psi = max(-1.2, min(1.2, e_psi))        # 線形化の範囲外へ飛ばさない

        N = cfg.horizon
        v = np.empty(N)
        idx, kap = self._horizon(path, i, v, cfg)
        L = cfg.wheelbase
        d_ref = np.arctan(L * kap)
        sec2 = 1.0 + (L * kap) ** 2
        a_dt = v * cfg.dt

        # 予測行列 X = F·e0 + G·u（u = δ − δ_ref、X は k=1..N の (e_y, e_psi)）
        F = np.zeros((2 * N, 2))
        G = np.zeros((2 * N, N))
        a_prev = np.eye(2)
        g_prev = np.zeros((2, N))
        for k in range(N):
            A = np.array([[1.0, a_dt[k]], [-a_dt[k] * kap[k] ** 2, 1.0]])
            a_prev = A @ a_prev
            g_prev = A @ g_prev
            g_prev[1, k] += a_dt[k] * sec2[k] / L
            F[2 * k:2 * k + 2] = a_prev
            G[2 * k:2 * k + 2] = g_prev

        w = np.tile([float(cfg.q_y), float(cfg.q_psi)], N)
        w[-2:] *= cfg.terminal
        x_off = F @ np.array([e_y, e_psi]) - G @ d_ref
        GtW = G.T * w
        D = np.eye(N) - np.eye(N, k=-1)
        c = np.zeros(N)
        c[0] = -delta_now
        H = GtW @ G + cfg.r_u * np.eye(N) + cfg.r_d * (D.T @ D)
        g = GtW @ x_off - cfg.r_u * d_ref + cfg.r_d * (D.T @ c)

        # 箱制約: 絶対値の限界と、今の舵角から舵レートで届く範囲
        lim = cfg.max_steer * max(abs(cfg.servo_gain), 1e-6)
        lo_abs, hi_abs = sm.effective(-lim), sm.effective(lim)
        reach = cfg.steer_rate * cfg.dt * np.arange(1, N + 1)
        lo = np.maximum(lo_abs, delta_now - reach)
        hi = np.minimum(hi_abs, delta_now + reach)
        hi = np.maximum(hi, lo)

        free = np.linalg.solve(H, -g)
        d = np.clip(free, lo, hi)
        self.last_iters = 0
        if not np.array_equal(d, free):           # 制約が効く → 射影ニュートン法
            if self._warm is not None and len(self._warm) == N:
                w0 = np.clip(self._warm, lo, hi)
                if 0.5 * w0 @ H @ w0 + g @ w0 < 0.5 * d @ H @ d + g @ d:
                    d = w0

            def cost(x):
                return 0.5 * x @ H @ x + g @ x

            for it in range(cfg.iters):
                grad = H @ d + g
                eps = 1e-9
                act = ((d <= lo + eps) & (grad > 0)) | ((d >= hi - eps) & (grad < 0))
                fr = ~act
                self.last_iters = it + 1
                if not fr.any():
                    break
                step = np.zeros(N)
                step[fr] = np.linalg.solve(H[np.ix_(fr, fr)], -grad[fr])
                if np.max(np.abs(step)) < 1e-6:
                    break
                c0, a_ = cost(d), 1.0
                while a_ > 1e-3:                   # 射影しながら下がる所まで刻む
                    nd = np.clip(d + a_ * step, lo, hi)
                    if cost(nd) < c0 - 1e-12:
                        break
                    a_ *= 0.5
                else:
                    break
                d = nd
        # 前回解は、次に呼ばれるまでに 1 刻み（`dt`）進むなら 1 つずらす。刻みより短い間隔で
        # 呼ばれる（50Hz）なら、ずらさない方が近い
        last = self._sent[0][0] if self._sent else None
        step = last is None or now - last >= 0.5 * cfg.dt
        self._warm = np.r_[d[1:], d[-1]] if step else d.copy()
        if last is not None and abs(now - last) < 1e-4:
            self._sent[0] = (now, float(d[0]))     # 同じ時刻にもう一度呼ばれた（経路を乗り換えた）
        else:
            self._sent.insert(0, (now, float(d[0])))
        self._sent = [(ts, x) for ts, x in self._sent if now - ts < cfg.delay_s + 1e-6][:16]
        self.last_cost = float(0.5 * d @ H @ d + g @ d)

        steer = sm.command(float(d[0]), cfg.max_steer)

        # GUI に描く点: 予測軌道の中ほど（経路の点を横偏差ぶん法線へずらす）
        X = (F @ np.array([e_y, e_psi]) + G @ (d - d_ref)).reshape(N, 2)
        m = N // 2
        j = idx[m]
        tj = path.xy[(j + 1) % n] - path.xy[j - 1] if path.closed else \
            path.xy[min(j + 1, n - 1)] - path.xy[max(j - 1, 0)]
        tjn = max(math.hypot(*tj), 1e-9)
        tgt = (float(path.xy[j, 0] - tj[1] / tjn * X[m, 0]),
               float(path.xy[j, 1] + tj[0] / tjn * X[m, 0]))
        return base._replace(steer=steer, target=tgt,
                             lookahead=float(np.sum(v[:m + 1]) * cfg.dt))
