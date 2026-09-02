"""運動予測の抽象化 — 「次はどこにいるか」を車体・センサ構成に依らず扱う。

`raspi/nav/slam.py`の推測航法（ジャイロ+速度の相補フィルタ、ジャイロバイアス
推定）は、車輪オドメトリを使わないという設計判断自体は車体非依存の考え方
だが、実装は「ジャイロ+速度センサ」という具体的なセンサ構成に固定されていた。
ここでは`MotionModel`という薄い抽象を挟み、外部センサ構成（ジャイロ+速度、
車輪速+IMU、あるいは推測航法を一切持たず等速度モデルだけに頼る、等）を
差し替え可能にする。

`current_twist()`と積分（`integrate_twist`）を分けているのは、脱スキュー
（`core/deskew.py`）に渡すtwistと、推測航法の1周期ぶんの移動予測に使う
twistを**同じ値**にする必要があるから。SLAMの1周期の処理は「まずtwistで
脱スキューし、脱スキュー後の点群の時刻差から正確な`dt`（span）を確定し、
そのspanで積分する」という順序になるため、`predict(dt)`のように`dt`を
先に要求するAPIだと、`dt`が決まる前にtwistが必要になる場面と噛み合わない。
`current_twist()`は引数を取らず、呼ぶたびに新しい値を返しうるので、
**1周期内で1回だけ呼ぶこと**。
"""

from __future__ import annotations

from typing import Protocol

import numpy as np

from .types import Cov3, Pose2D, Twist2D

__all__ = ["MotionModel", "ConstantVelocityModel", "ExternalTwistModel", "GyroBiasEstimator"]

#: 推測航法の予測が外れうる大きさの既定値。`ConstantVelocityModel`はセンサを
#: 一切持たないため、この値は「何も分からないなりの控えめな見積もり」でしかない
_DEFAULT_COV = np.diag([0.05, 0.05, 0.05]) ** 2


class MotionModel(Protocol):
    """次の姿勢を予測し、観測（スキャンマッチ等の結果）で内部状態を更新する。"""

    def current_twist(self) -> Twist2D:
        """次の1周期で使うtwist推定値。`core/deskew.py`にそのまま渡せる。"""
        ...

    def prediction_covariance(self, dt: float) -> Cov3:
        """`dt`秒ぶんの予測の不確かさ（3x3共分散）。"""
        ...

    def update(self, measured_delta: Pose2D, dt: float) -> None:
        """観測された相対姿勢を使って内部状態（速度推定等）を更新する。"""
        ...


class ConstantVelocityModel:
    """車輪もIMUも要らない、自分の推定姿勢の履歴だけを使う等速度モデル。

    `raspi/nav/slam.py`の既定動作（`speed`/`yaw_rate`が渡されないときの
    フォールバック）と同じ考え方。1次遅れで速度推定を均し、マッチ結果の
    ばらつきがそのまま次の予測を揺らして発散しないようにする。
    """

    def __init__(self, *, vel_alpha: float = 0.5, prediction_cov: Cov3 = _DEFAULT_COV) -> None:
        self._vel_alpha = vel_alpha
        self._cov = prediction_cov
        self.twist = Twist2D(0.0, 0.0, 0.0)

    def current_twist(self) -> Twist2D:
        return self.twist

    def prediction_covariance(self, dt: float) -> Cov3:
        return self._cov * max(dt, 1e-3)

    def update(self, measured_delta: Pose2D, dt: float) -> None:
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
    現在のtwistを返す呼び出し可能オブジェクト。`bias_estimator`を渡すと、
    ジャイロのヨーレートに`GyroBiasEstimator`の補正を適用し、`update()`で
    観測とのずれからバイアスを学習する（`raspi/nav/slam.py`と同じ設計）。
    """

    def __init__(self, twist_source, *, bias_estimator: "GyroBiasEstimator | None" = None,
                 prediction_cov: Cov3 = _DEFAULT_COV) -> None:
        self._twist_source = twist_source
        self._bias = bias_estimator
        self._cov = prediction_cov
        self._last_raw_yaw_rate = 0.0

    def current_twist(self) -> Twist2D:
        raw = self._twist_source()
        self._last_raw_yaw_rate = raw.yaw_rate
        yaw_rate = (self._bias.corrected_yaw_rate(raw.yaw_rate)
                    if self._bias is not None else raw.yaw_rate)
        return Twist2D(raw.vx, raw.vy, yaw_rate)

    def prediction_covariance(self, dt: float) -> Cov3:
        return self._cov * max(dt, 1e-3)

    def update(self, measured_delta: Pose2D, dt: float) -> None:
        if self._bias is not None and dt > 1e-6:
            self._bias.update(self._last_raw_yaw_rate, measured_delta.yaw / dt)


class GyroBiasEstimator:
    """ジャイロのバイアスを「回頭を2通りで測って引き算」で推定する。

    `raspi/nav/slam.py`の設計をそのまま一般化: `ジャイロの読み − 実際に採用した
    回頭`ではなく、`ジャイロの読み − 点群が独立に測った回頭`で作る。採用値は
    予測（＝バイアス込み）にほぼ一致してしまい、前者だと差が消えて収束しない。

    速く更新すると、点群側のマッチのばらつきがバイアスに化けてそれがまた
    予測を狂わせる循環に入るため、`alpha`は小さく保つ（既定は10Hzで時定数10秒相当）。
    """

    def __init__(self, *, alpha: float = 0.01, limit: float = 0.1) -> None:
        self._alpha = alpha
        self._limit = limit
        self.bias = 0.0

    def corrected_yaw_rate(self, raw_yaw_rate: float) -> float:
        return raw_yaw_rate - self.bias

    def update(self, raw_yaw_rate: float, measured_yaw_rate: float) -> None:
        err = raw_yaw_rate - measured_yaw_rate - self.bias
        self.bias = max(-self._limit, min(self._limit, self.bias + self._alpha * err))
