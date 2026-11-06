"""走行中の学習 — 舵の効きが横加速度とともに落ちる度合い（アンダーステア勾配 K）をコースで決める。

舵の換算（`purepursuit.SteerMap`）は校正 2 が低速（0.45m/s）で決める。タイヤが横に滑らない速さなので、
曲率 = tan(δ)/L がそのまま成り立つ。速く曲がるとタイヤの横滑り角のぶん同じ舵でも曲がり方が足りなくなる:

    曲率 κ = tan(δ) / (L + K·v²)        （`sim/vehicle.py` の `understeer_gradient` と同じ式）

**K はタイヤと床の組み合わせで変わる**（車に固有ではない）ので、校正ではなく走りながら決める（`nav/grip.py` と
同じ方針）。MPC・Pure Pursuit は「欲しい曲率 → 舵」の換算にこの K を入れる（`SteerMap.understeer`）。
2026-11-02 の実機（K を入れていない MPC）は、横加速度 3〜5m/s² のコーナーで経路の外へ中央 4〜5cm 膨らんでいた。

## 推定

上の式を `tan(δ) − L·κ = K·v²·κ = K·a_y` と書き直し、標本ごとに

    y = tan(δ) − L·r/v     （δ は校正の換算を通した舵角、r はヨーレート、v は車速）
    x = v·r               （横加速度）

として `y = c + K·x` を当てはめる。**ただし最小二乗ではなく操作変数法**（操作変数 z = v²·tan(δ)/L、舵から
出る横加速度）。ジャイロの雑音が x と y の両方に（逆向きに）入るので、最小二乗だと K が小さく偏る（実機の記録で
約半分に出た）。z はジャイロを含まないので、この偏りが消える。切片 c は舵の中立のずれ・ジャイロのバイアスを吸う
だけで、補正には使わない（中立は校正 2 の値）。

- 舵角は**ヨーの遅れ（2ζ/ωn、校正 3）だけ前**の値を使う。舵が動いている間（`steer_rate_max` 超）は使わない
- 古い標本は、使った標本の時間で `memory_s` の時定数で忘れる（止まっている間は忘れない）
- 横加速度が `ay_strong` 以上の標本が `strong_min_s` 秒ぶん溜まるまでは、推定を使わず前の値（地図に保存した値、
  無ければ 0）のまま。cam_e2e の記録のように横をあまり使わない走りでは K が定まらない（前半と後半で符号が変わった）
- 推定は `[k_min, k_max]` に収める
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

__all__ = ["UndersteerConfig", "UndersteerLearner", "K_MIN", "K_MAX", "K_FORMAT"]

#: 保存値・推定の範囲 [rad/(m/s²)]。実機 2026-11-02 の記録で約 0.002
K_MIN = -0.002
K_MAX = 0.01
#: 保存値（`GripMap.understeer`）の標本の選び方の版。違う版の保存値は読まない（0 から学び直す）。
#: 1 = 横グリップの学習のゲート（舵が上限など）で標本を捨てていた。横加速度の大きい区間ほど落ちて K が小さく
#: 偏った（2026-11-06 の記録で 0.52e-3 を保存。選び直すと約 2e-3）
K_FORMAT = 2


@dataclass
class UndersteerConfig:
    v_min: float = 1.0                #: これより遅い標本は使わない [m/s]（低速は K の効きが v² で消える）
    steer_rate_max: float = 0.6       #: 舵がこれより速く動いている標本は使わない [rad/s]（0.1s の差で見る）
    resid_max: float = 0.15           #: |y| がこれを超える標本は使わない [rad]（スピン・衝突）
    memory_s: float = 60.0            #: 忘れる時定数（使った標本の時間）[s]
    ay_strong: float = 2.5            #: 「横を使っている」とみなす横加速度 [m/s²]
    strong_min_s: float = 8.0         #: その標本がこれだけ溜まるまでは推定を使わない [s]
    k_min: float = K_MIN
    k_max: float = K_MAX


class UndersteerLearner:
    """`observe()` を 100Hz で呼び、`k` を追従の舵の換算に使う。"""

    def __init__(self, steer_map, lag_s: float = 0.0, cfg: UndersteerConfig | None = None) -> None:
        """:param steer_map: 校正の舵の換算（`Vehicle.steer_map`。`understeer` は見ない）
        :param lag_s: 舵角 → 曲率のヨーの遅れ [s]（`2ζ/ωn`、校正 3）"""
        self.sm = steer_map._replace(understeer=0.0)
        self.L = float(steer_map.wheelbase)
        self.lag_s = max(0.0, float(lag_s))
        self.cfg = cfg or UndersteerConfig()
        self.reset()

    def reset(self, prior: float = 0.0) -> None:
        """学習を捨てる。`prior` は推定が使えるようになるまで使う値（地図に保存した K）。"""
        self.prior = max(self.cfg.k_min, min(self.cfg.k_max, float(prior)))
        self._s = [0.0] * 6               # Σ1, Σz, Σx, Σy, Σzx, Σzy（忘却つき）
        self.strong_s = 0.0               #: 横を使った標本の時間（忘却つき）[s]
        self._hist: deque[tuple[float, float]] = deque()
        self._t_last: float | None = None
        self.estimate: float | None = None   #: 直近の推定（範囲に収める前。使えなければ None）
        self.offset = 0.0                 #: 切片 c [rad]（診断用）
        self.samples = 0

    @property
    def ready(self) -> bool:
        """推定を使ってよいか（横を使った標本が十分溜まった）。"""
        return self.estimate is not None and self.strong_s >= self.cfg.strong_min_s

    @property
    def k(self) -> float:
        """追従に使う K [rad/(m/s²)]。"""
        if self.ready:
            return max(self.cfg.k_min, min(self.cfg.k_max, self.estimate))
        return self.prior

    def _steer_at(self, t: float) -> float | None:
        """時刻 `t` の実舵角（それ以前で最も新しい標本）。履歴が足りなければ None。"""
        h = self._hist
        if not h or h[0][0] > t:
            return None
        out = h[0][1]
        for ts, s in h:
            if ts > t:
                break
            out = s
        return out

    def observe(self, t: float, v: float, yaw_rate: float, steer_actual: float) -> None:
        """車両状態 1 つぶん。標本を選ぶのはここの条件（車速・舵の速さ・残差）だけ。

        **外から「この区間は使わない」と選ばない。** 横グリップの学習のゲート（舵が上限など）で選んでいた頃は、
        アンダーステアの大きい標本ほど落ちて K が小さく偏った（`K_FORMAT` の 1）。"""
        cfg = self.cfg
        h = self._hist
        h.append((t, float(steer_actual)))
        while h and h[0][0] < t - self.lag_s - 0.3:
            h.popleft()
        dt = 0.0 if self._t_last is None else min(max(t - self._t_last, 0.0), 0.05)
        self._t_last = t
        if v < cfg.v_min or dt <= 0.0:
            return
        t_d = t - self.lag_s
        d = self._steer_at(t_d)
        d_prev = self._steer_at(t_d - 0.1)
        if d is None or d_prev is None or abs(d - d_prev) > cfg.steer_rate_max * 0.1:
            return
        tan_d = math.tan(self.sm.effective(d))
        y = tan_d - self.L * yaw_rate / v
        if abs(y) > cfg.resid_max:
            return
        x = v * yaw_rate
        z = v * v * tan_d / self.L
        lam = math.exp(-dt / cfg.memory_s)
        s = self._s
        for i, val in enumerate((dt, dt * z, dt * x, dt * y, dt * z * x, dt * z * y)):
            s[i] = s[i] * lam + val
        self.strong_s = self.strong_s * lam + (dt if abs(x) >= cfg.ay_strong else 0.0)
        self.samples += 1
        w = s[0]
        mz, mx, my = s[1] / w, s[2] / w, s[3] / w
        czx = s[4] / w - mz * mx
        czy = s[5] / w - mz * my
        if czx > 0.5:                     # 横加速度の振れが小さすぎると比が定まらない [(m/s²)²]
            self.estimate = czy / czx
            self.offset = my - self.estimate * mx
