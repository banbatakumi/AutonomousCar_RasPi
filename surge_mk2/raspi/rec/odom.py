"""推測航法（車速＋ジャイロの積分）。**記録と可視化のための自己位置**。

`logger_node` と `tools/sfl2mcap.py` が `vehicle_state` を受けるたびに呼び、
MCAP の `/odom` と Foxglove 用の `/tf`（odom→base_link）を作る（`rec/viz.py`）。
planner が動いていない記録でも「車がどこを走ったか」を残すのが目的で、
**走行制御には使わない**（制御側の姿勢は各 planner が自前で持つ）。

## 入力は `speed` と `yaw_rate` だけ

- `speed` … 車体中心線方向に射影済みの車速。そのまま前進量になる
- `yaw_rate` … IMU。地磁気が無いので絶対方位は出ない（原点は記録開始時の向き）

`odom_center`（前輪の累積距離の差分）を積分する手もあるが、前輪エンコーダは
分解能が粗く 100Hz の差分は段々になる。`speed` は STM32 側で平滑化されているので
こちらを使う。

## ジャイロのゼロ点ずれは静止中に学ぶ（ZUPT）

`stopped` の間は「車は回っていない」と分かっているので、ジャイロの読みが
そのままゼロ点ずれになる。**時定数で寄せる**（1サンプルあたりの係数にすると
100Hz と 50Hz の記録で効きが変わる）。静止中は位置も方位も動かさない。

学ばない場面が2つある。取り込むと走り出した後ずっと曲がり続ける。

- **止まった直後**（`STILL_SETTLE_S` 未満）… `stopped` は車速のデッドバンドで
  決まるので、減速の最後は「止まった扱いだが実際はまだ転がって曲がっている」。
  シム（`sim.slam_bench` の誤差なし条件）で、ここを学ぶと真値 0 のゼロ点ずれを
  -0.05°/s と誤って学んだ。**ただし `stopped` は静止付近でばたつく**（前輪エンコーダの
  符号が揺れる。`VehicleState.stopped`）ので、1サンプル外れただけで数え直すと
  いつまでも学べない（シムの harsh 条件で一度も学べなかった）。`MOVING_RESET_S`
  以上続けて動いたときだけ数え直す
- **読みが大きいとき**（手で車を持ち上げて回した等）
"""

from __future__ import annotations

import math

import msgspec

__all__ = ["DeadReckoner", "OdomPose"]

NS = 1_000_000_000

#: これより間が空いたら積分しない [s]。100Hz の記録で 5 周期ぶんの欠け。
#: 長い欠けを1回の dt で積分すると、その間の旋回が直線に化けて大きく外す
MAX_DT_S = 0.05

#: 静止中のゼロ点ずれ学習の時定数 [s]。**短いほど短い停車でも収束するが、ノイズを拾う**。
#: シム（3秒停車してから周回）で 2.0 だと真値の7〜8割しか学べなかった
BIAS_TAU_S = 1.0

#: 止まってからこれだけ経つまでは学ばない [s]（モジュール docstring）
STILL_SETTLE_S = 0.5

#: これだけ続けて動いたら「止まってからの時間」を数え直す [s]（モジュール docstring）
MOVING_RESET_S = 0.1

#: 静止中にこれを超える回頭 [rad/s] を読んだら学習しない（手で回した等）
BIAS_MAX_RATE = 0.2

#: ゼロ点ずれの上限 [rad/s]。MPU6050 のゼロ点ずれは通常 0.05rad/s 未満
BIAS_LIMIT = 0.3


class OdomPose(msgspec.Struct):
    """推測航法の1サンプル（MCAP の `/odom`。Foxglove の Plot パネル用）。

    **フレームは `odom`**: 記録開始時の車両位置・向きが原点、x=前・y=左・反時計回り正。
    """

    #: 元の `vehicle_state.t_capture` [ns]。**以下の実数は書き出し用に丸めてある**
    #: （位置 0.1mm・角度 1e-5rad。`DeadReckoner` の内部状態は丸めない）
    t_capture: int = 0
    x: float = 0.0                         #: [m]
    y: float = 0.0                         #: [m]
    yaw: float = 0.0                       #: [rad] ±π に丸めた値
    yaw_unwrapped: float = 0.0             #: [rad] 丸めない累積回頭（Plot で見やすい）
    v: float = 0.0                         #: 積分に使った車速 [m/s]
    omega: float = 0.0                     #: ゼロ点ずれを引いたヨーレート [rad/s]
    gyro_bias: float = 0.0                 #: 推定したゼロ点ずれ [rad/s]
    dist: float = 0.0                      #: 累積走行距離 [m]（後退も正で数える）
    stopped: bool = True
    #: 静止中に学習した累積時間 [s]。**0 のまま走り出したらゼロ点ずれは未較正**
    bias_learn_s: float = 0.0
    #: `MAX_DT_S` を超える欠けで積分を飛ばした回数
    gaps: int = 0


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class DeadReckoner:
    """`VehicleState` を順に食わせて `OdomPose` を返す。

        dr = DeadReckoner()
        for st in states:
            pose = dr.update(st)
    """

    def __init__(self, *, max_dt_s: float = MAX_DT_S, bias_tau_s: float = BIAS_TAU_S,
                 bias_max_rate: float = BIAS_MAX_RATE,
                 still_settle_s: float = STILL_SETTLE_S) -> None:
        self.max_dt_s = max_dt_s
        self.bias_tau_s = bias_tau_s
        self.still_settle_s = still_settle_s
        self.bias_max_rate = bias_max_rate
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0                     #: 丸めない累積値
        self.dist = 0.0
        self.bias = 0.0
        self.bias_learn_s = 0.0
        self.gaps = 0
        self._still_s = 0.0                #: 止まってからの時間 [s]
        self._moving_s = 0.0               #: 続けて動いている時間 [s]
        self._t_prev: int | None = None
        self._v_prev = 0.0
        self._w_prev = 0.0

    def update(self, st) -> OdomPose:
        """1サンプル進める。`st` は `VehicleState`（`t_capture` `speed` `yaw_rate` `stopped`）。"""
        t = st.t_capture or st.t_pub
        v = 0.0 if st.stopped else float(st.speed)
        raw_w = float(st.yaw_rate)

        dt = 0.0
        if self._t_prev is not None:
            dt = (t - self._t_prev) / NS
            if dt <= 0.0:
                dt = 0.0                   # 同時刻・逆行は積分しない
            elif dt > self.max_dt_s:
                self.gaps += 1
                dt = 0.0
        self._t_prev = t

        if st.stopped:
            self._moving_s = 0.0
            self._still_s += dt
        else:
            self._moving_s += dt
            if self._moving_s >= MOVING_RESET_S:
                self._still_s = 0.0
        if st.stopped and self._still_s >= self.still_settle_s \
                and abs(raw_w - self.bias) < self.bias_max_rate and dt > 0.0:
            a = min(1.0, dt / self.bias_tau_s)
            self.bias = max(-BIAS_LIMIT, min(BIAS_LIMIT,
                                             self.bias + a * (raw_w - self.bias)))
            self.bias_learn_s += dt
        w = raw_w - self.bias
        # 静止中は回っていないとみなす。**ただし大きく回されたら追う**（手で回した等）
        if st.stopped and abs(w) < self.bias_max_rate:
            w = 0.0

        if dt > 0.0:
            # 台形則: 周期の両端の平均で回し、中間の向きへ進める
            w_mid = 0.5 * (w + self._w_prev)
            v_mid = 0.5 * (v + self._v_prev)
            yaw_mid = self.yaw + 0.5 * w_mid * dt
            self.x += v_mid * math.cos(yaw_mid) * dt
            self.y += v_mid * math.sin(yaw_mid) * dt
            self.yaw += w_mid * dt
            self.dist += abs(v_mid) * dt
        self._v_prev = v
        self._w_prev = w

        # **書き出す値だけ丸める**（内部の積分は丸めない）。JSON に全桁書くと
        # 1件 290B になり、100Hz で記録の数値部分の大半を占めた（実機 sysid 記録で測定）
        return OdomPose(t_capture=t, x=round(self.x, 4), y=round(self.y, 4),
                        yaw=round(_wrap(self.yaw), 5), yaw_unwrapped=round(self.yaw, 5),
                        v=round(v, 4), omega=round(w, 5), gyro_bias=round(self.bias, 6),
                        dist=round(self.dist, 4), stopped=bool(st.stopped),
                        bias_learn_s=round(self.bias_learn_s, 3), gaps=self.gaps)
