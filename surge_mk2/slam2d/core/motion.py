"""運動予測の抽象化 — 「次はどこにいるか」を車体・センサ構成に依らず扱う。

`MotionModel`という薄い抽象を挟み、外部センサ構成（ジャイロ+速度、車輪速+IMU、
あるいは推測航法を一切持たず等速度モデルだけに頼る、等）を差し替え可能にする。

`current_twist()`と積分（`integrate_twist`）を分けているのは、脱スキュー
（`core/deskew.py`）に渡すtwistと、推測航法の1周期ぶんの移動予測に使う
twistを**同じ値**にする必要があるから。SLAMの1周期の処理は「まずtwistで
脱スキューし、脱スキュー後の点群の時刻差から正確な`dt`（span）を確定し、
そのspanで積分する」という順序になるため、`predict(dt)`のように`dt`を
先に要求するAPIだと噛み合わない。`current_twist()`は呼ぶたびに新しい値を
返しうるので、**1周期内で1回だけ呼ぶこと**。

## 予測の不確かさは「走った量」に比例させる

`prediction_covariance()`は位置合わせ（`core/register.py`）の事前分布の重みに
なる。固定値にすると、止まっている間も高速で走っている間も同じだけ信用する
ことになる。実際の推測航法の誤差は**走った距離・回った角度に比例する部分**
（車輪半径の誤差・ジャイロの倍率誤差）と**時間に比例する部分**（ジャイロの
ゼロ点ずれ）でできているので、そのとおりに組む（`OdometryNoise`）。

## 推測航法の系統誤差をオンラインで学ぶ

点群の位置合わせで測った移動量と、推測航法の予測を突き合わせて:

- **ジャイロのゼロ点ずれ**（`GyroBiasEstimator`）: 回頭の差
- **車速の倍率誤差**（`SpeedScaleEstimator`）: 進行方向の移動量の比

を学ぶ。後者は**進行方向が点群で観測できているときだけ**更新する（通路の
直線区間では点群は進行方向の位置を決められず、位置合わせの結果は推測航法の
予測そのものになる。そこで比を取っても 1.0 が返るだけで、学習が止まるどころか
正しい値から引き戻される）。

## ★ ジャイロのゼロ点ずれは「止まっているとき」にしか素直に測れない

走行中の回頭は、**地図作成中は直近のキーフレームの点群（局所地図）を基準に
測っている**。ゼロ点ずれで姿勢がゆっくり回ると、その姿勢で焼いた局所地図も
一緒に回るので、点群は「ずれていない」と答える。実測（`sim/slam_bench.py`
normal）でも、真のゼロ点ずれ0.5°/sに対して走行中の学習値は0.01°/sにしか
ならなかった。**止まっている間はヨーレートの読みがそのままゼロ点ずれ**なので、
`GyroBiasEstimator.update_stationary()`で速く学ぶ（ZUPT）。実車は ARM →
engage の操作の間ずっと止まっているので、走り出す前にここで較正が終わる。
走行中に残るぶんの回転ドリフトは、ループ閉じ（`backend/`）が直す担当。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .types import Cov3, Pose2D, Twist2D

__all__ = ["MotionModel", "OdometryNoise", "ConstantVelocityModel", "ExternalTwistModel",
           "GyroBiasEstimator", "SpeedScaleEstimator"]


@dataclass(frozen=True, slots=True)
class OdometryNoise:
    """推測航法の1周期ぶんの誤差の見積もり（1σ）。車体座標系。

    既定値は「MPU6050 + 前輪エンコーダ、較正は大ざっぱ」を想定した控えめな値。
    **小さすぎると点群が推測航法に負け、大きすぎると通路で推測航法が効かない。**
    """

    #: 進行方向: 距離の倍率誤差 [比] と、距離によらない下限 [m]
    trans_rel: float = 0.05
    trans_min: float = 0.003
    #: 横方向（非ホロノミック車両ではほぼ横滑りだけ）[比]
    lat_rel: float = 0.02
    #: 回頭: 回った角度の倍率誤差 [比] と、時間に比例するゼロ点ずれ [rad/s]
    yaw_rel: float = 0.03
    yaw_rate: float = math.radians(1.0)

    def covariance(self, twist: Twist2D, dt: float) -> Cov3:
        dt = max(dt, 1e-3)
        dist = math.hypot(twist.vx, twist.vy) * dt
        sx = self.trans_min + self.trans_rel * dist
        sy = self.trans_min + self.lat_rel * dist
        syaw = self.yaw_rate * dt + self.yaw_rel * abs(twist.yaw_rate) * dt
        return np.diag([sx * sx, sy * sy, syaw * syaw])


class MotionModel(Protocol):
    """次の姿勢を予測し、観測（スキャンマッチ等の結果）で内部状態を更新する。"""

    def current_twist(self) -> Twist2D:
        """次の1周期で使うtwist推定値。`core/deskew.py`にそのまま渡せる。"""
        ...

    def prediction_covariance(self, dt: float) -> Cov3:
        """`dt`秒ぶんの予測の不確かさ（3x3共分散、**前の姿勢の車体座標系**）。"""
        ...

    def update(self, measured_delta: Pose2D, dt: float, info: Cov3 | None = None,
               raw_delta: Pose2D | None = None, absolute: bool = False) -> None:
        """観測された相対姿勢を使って内部状態（速度推定・バイアス等）を更新する。

        :param info: その観測が点群でどれだけ拘束されていたか（車体座標系の
            情報行列）。None なら「分からない」
        :param raw_delta: 同じ区間の**補正前**のセンサ値から積んだ相対姿勢。
            ジャイロのゼロ点ずれ・車速の倍率の学習に使う（無ければ直近の
            生サンプル×dtで代用する）
        :param absolute: その観測が**動かない基準**（凍結地図）に対するものか。
            地図を作りながらの観測は自分の推定で焼いた局所地図が基準なので、
            系統誤差の学習には使えない（モジュールdocstringのZUPTの節）
        """
        ...

    def correct(self, twist: Twist2D) -> Twist2D:
        """生のセンサ値に、学習済みの補正（バイアス・倍率）を適用する。"""
        ...


class ConstantVelocityModel:
    """車輪もIMUも要らない、自分の推定姿勢の履歴だけを使う等速度モデル。

    1次遅れで速度推定を均し、マッチ結果のばらつきがそのまま次の予測を揺らして
    発散しないようにする。センサが無いぶん予測の不確かさは大きく取る。
    """

    def __init__(self, *, vel_alpha: float = 0.5,
                 noise: OdometryNoise = OdometryNoise(trans_rel=0.3, trans_min=0.02,
                                                      lat_rel=0.3, yaw_rel=0.3,
                                                      yaw_rate=math.radians(10.0))) -> None:
        self._vel_alpha = vel_alpha
        self._noise = noise
        self.twist = Twist2D(0.0, 0.0, 0.0)

    def current_twist(self) -> Twist2D:
        return self.twist

    def prediction_covariance(self, dt: float) -> Cov3:
        return self._noise.covariance(self.twist, dt)

    def correct(self, twist: Twist2D) -> Twist2D:
        return twist

    def update(self, measured_delta: Pose2D, dt: float, info: Cov3 | None = None,
               raw_delta: Pose2D | None = None, absolute: bool = False) -> None:
        dt = max(dt, 1e-3)
        vx = measured_delta.x / dt
        vy = measured_delta.y / dt
        yaw_rate = measured_delta.yaw / dt
        a = self._vel_alpha
        self.twist = Twist2D(
            self.twist.vx + a * (vx - self.twist.vx),
            self.twist.vy + a * (vy - self.twist.vy),
            self.twist.yaw_rate + a * (yaw_rate - self.twist.yaw_rate),
        )


class ExternalTwistModel:
    """外部センサ（ジャイロ+速度、車輪速+IMU等）から得たtwistを使う。

    `twist_source`はcoreの外（車体固有のアダプタ層）が用意する、引数無しで
    現在のtwistを返す呼び出し可能オブジェクト。`bias_estimator`・
    `scale_estimator`を渡すと、読みを補正し、`update()`で観測とのずれから学ぶ。
    """

    def __init__(self, twist_source, *, bias_estimator: "GyroBiasEstimator | None" = None,
                 scale_estimator: "SpeedScaleEstimator | None" = None,
                 noise: OdometryNoise = OdometryNoise(),
                 still_dist: float = 0.004, still_yaw: float = math.radians(0.3)) -> None:
        #: 「止まっている」とみなす1周期あたりの移動量・回頭
        self._still_dist = still_dist
        self._still_yaw = still_yaw
        self._twist_source = twist_source
        self._bias = bias_estimator
        self._scale = scale_estimator
        self._noise = noise
        self._last_raw = Twist2D(0.0, 0.0, 0.0)
        self._last = Twist2D(0.0, 0.0, 0.0)

    @property
    def bias(self) -> float:
        return self._bias.bias if self._bias is not None else 0.0

    @property
    def scale(self) -> float:
        return self._scale.scale if self._scale is not None else 1.0

    def correct(self, twist: Twist2D) -> Twist2D:
        yaw_rate = (self._bias.corrected_yaw_rate(twist.yaw_rate)
                    if self._bias is not None else twist.yaw_rate)
        k = self._scale.scale if self._scale is not None else 1.0
        return Twist2D(twist.vx * k, twist.vy * k, yaw_rate)

    def current_twist(self) -> Twist2D:
        raw = self._twist_source()
        self._last_raw = raw
        self._last = self.correct(raw)
        return self._last

    def prediction_covariance(self, dt: float) -> Cov3:
        return self._noise.covariance(self._last, dt)

    def update(self, measured_delta: Pose2D, dt: float, info: Cov3 | None = None,
               raw_delta: Pose2D | None = None, absolute: bool = False) -> None:
        if dt <= 1e-6:
            return
        raw_yaw_rate = (raw_delta.yaw / dt) if raw_delta is not None else self._last_raw.yaw_rate
        raw_fwd = raw_delta.x if raw_delta is not None else self._last_raw.vx * dt
        # ZUPT: 車速の読みも点群も「動いていない」と言っているなら、
        # ヨーレートの読みはそのままゼロ点ずれ
        still = (abs(raw_fwd) < self._still_dist and abs(measured_delta.x) < self._still_dist
                 and abs(measured_delta.y) < self._still_dist
                 and abs(measured_delta.yaw) < self._still_yaw)
        if self._bias is not None and still:
            self._bias.update_stationary(raw_yaw_rate)
            return
        if not absolute:
            # 地図を作りながらの観測は「自分で焼いた局所地図」が基準なので、
            # ゼロ点ずれ・倍率の学習には使えない（モジュールdocstringのZUPTの節）
            return
        if self._bias is not None:
            ok_yaw = info is None or info[2, 2] >= (1.0 / math.radians(0.5)) ** 2
            if ok_yaw:
                self._bias.update(raw_yaw_rate, measured_delta.yaw / dt)
        if self._scale is not None and info is not None:
            self._scale.update(raw_fwd, measured_delta.x, info[0, 0])


class GyroBiasEstimator:
    """ジャイロのバイアスを「回頭を2通りで測って引き算」で推定する。

    `ジャイロの読み − 実際に採用した回頭`ではなく、`ジャイロの読み − 点群が
    独立に測った回頭`で作る。採用値が予測（＝バイアス込み）にほぼ一致して
    しまう構成だと差が消えて収束しないため。

    速く更新すると、点群側のマッチのばらつきがバイアスに化けてそれがまた
    予測を狂わせる循環に入るため、`alpha`は小さく保つ（既定は10Hzで時定数10秒相当）。
    """

    def __init__(self, *, alpha: float = 0.01, alpha_still: float = 0.15,
                 limit: float = 0.5) -> None:
        self._alpha = alpha
        self._alpha_still = alpha_still
        self._limit = limit
        self.bias = 0.0
        #: 静止中に較正した回数（診断用。0 のまま走り出したら較正できていない）
        self.still_updates = 0

    def corrected_yaw_rate(self, raw_yaw_rate: float) -> float:
        return raw_yaw_rate - self.bias

    def update(self, raw_yaw_rate: float, measured_yaw_rate: float) -> None:
        err = raw_yaw_rate - measured_yaw_rate - self.bias
        self.bias = max(-self._limit, min(self._limit, self.bias + self._alpha * err))

    def update_stationary(self, raw_yaw_rate: float) -> None:
        """止まっている間の更新。**読みがそのままゼロ点ずれ**なので速く寄せる。"""
        self.bias = max(-self._limit, min(self._limit,
                                          self.bias + self._alpha_still * (raw_yaw_rate - self.bias)))
        self.still_updates += 1


class SpeedScaleEstimator:
    """車速の読みに掛ける倍率を「点群で測った前進量 ÷ 読みの前進量」で推定する。

    **進行方向が点群で拘束されているときだけ**更新する（モジュールdocstring）。
    判定は位置合わせの情報行列の進行方向成分`info_xx`（車体座標）で、
    `1/σ²`が`min_info`以上、つまり進行方向の位置が`1/sqrt(min_info)`[m]より
    よく決まっているときだけ。
    """

    def __init__(self, *, alpha: float = 0.02, min_dist: float = 0.02,
                 min_info: float = (1.0 / 0.005) ** 2,
                 limits: tuple[float, float] = (0.8, 1.25)) -> None:
        self._alpha = alpha
        self._min_dist = min_dist
        self._min_info = min_info
        self._limits = limits
        self.scale = 1.0
        self.updates = 0

    def update(self, raw_forward: float, measured_forward: float, info_xx: float) -> None:
        if abs(raw_forward) < self._min_dist or info_xx < self._min_info:
            return
        ratio = measured_forward / raw_forward
        if not (self._limits[0] <= ratio <= self._limits[1]):
            return
        self.scale += self._alpha * (ratio - self.scale)
        self.scale = max(self._limits[0], min(self._limits[1], self.scale))
        self.updates += 1
