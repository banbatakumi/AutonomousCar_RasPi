"""slam2d全体で共有する型。**車体・センサに一切依存しない。**

`Pose2D`はSE(2)（平面上の位置+向き）の最小表現で、姿勢の合成・逆変換・相対姿勢
計算をここに集約する。以前の`raspi/nav/`実装ではこの手の`cos/sin`合成式が
`scanmatch.py`・`scan2scan.py`・`slam.py`の各所に重複して書かれていたため、
ここに一本化した。

`RawScan`は`raspi/nav/deskew.py`の暗黙の前提（LD06の360点固定長・セクタ単位
でしか時刻が無い）を持たない。**点数は可変**で、各点が個別の取得時刻
（`t_point_ns`）を持てる形にしてあるので、点ごとに時刻を持たないセンサは
全点へ同じ時刻を複製すればよい（`RawScan`側では区別しない）。
"""

from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

__all__ = [
    "Pose2D", "RawScan", "ScanPoints", "Twist2D", "Cov3",
    "wrap_angle", "compose", "inverse", "between", "integrate_twist",
]

#: これ以下のヨーレートは直進として扱う [rad/s]。dx/dyaw が発散するのを避ける
_STRAIGHT_EPS = 1e-4

#: 3x3の共分散/情報行列（並進x, 並進y, 回頭yawの順）。型チェック用のエイリアスに過ぎない
Cov3 = np.ndarray


class Pose2D(NamedTuple):
    """平面上の姿勢 (x, y, yaw)。yaw は rad、反時計回り正。"""

    x: float
    y: float
    yaw: float


class Twist2D(NamedTuple):
    """車体座標系での速度 (vx, vy, yaw_rate)。

    非ホロノミック車両(前後方向にしか進めない)なら`vy=0`に固定して使えばよいが、
    型としては横滑りやオムニホイール等も表現できるよう分離してある。
    """

    vx: float
    vy: float
    yaw_rate: float


class RawScan(NamedTuple):
    """脱スキュー前の1周分の点群（センサ座標系）。

    `raspi/nav/deskew.py`の`Points`と違い、こちらは**脱スキュー前**の生データ。
    360点固定を要求しないので、LD06以外の任意角度分解能のLiDARを表現できる。
    """

    angles: np.ndarray      #: (N,) [rad]。センサ座標系、反時計回り正
    ranges: np.ndarray      #: (N,) [m]
    valid: np.ndarray       #: (N,) bool。False の点は「観測できなかった」（`dist==0`相当）
    saturated: np.ndarray   #: (N,) bool。True なら測距上限に張り付いた飽和点(壁として打たない)
    t_point_ns: np.ndarray  #: (N,) int64。各点の取得時刻。点ごとの時刻が無ければ全点同一値でよい

    def __len__(self) -> int:
        return int(self.angles.size)


class ScanPoints(NamedTuple):
    """脱スキュー済みの点群（共通の基準フレーム、`t_ref_ns`時点）。

    `raspi/nav/deskew.py`の`Points`と同じ設計（車体非依存）。`hit[i]`がFalseの
    点は「そこまでは空きだが、終端に壁があるとは限らない」（飽和点）。
    """

    x: np.ndarray           #: (N,) [m]
    y: np.ndarray           #: (N,) [m]
    hit: np.ndarray         #: (N,) bool。True なら終端に壁を打つ
    t_ref_ns: int            #: この点群が「いつの姿勢」のものか
    corrected: bool          #: 脱スキューを実際に掛けたか

    def __len__(self) -> int:
        return int(self.x.size)


def wrap_angle(a: float) -> float:
    """角度を (-pi, pi] へ畳む。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """`a`の座標系で表された`b`を、`a`の親座標系（世界座標）へ変換する（a∘b）。

    `b`が「`a`から見た相対姿勢」のとき、`compose(a, b)`は世界座標での絶対姿勢になる。
    """
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(
        a.x + c * b.x - s * b.y,
        a.y + s * b.x + c * b.y,
        wrap_angle(a.yaw + b.yaw),
    )


def inverse(a: Pose2D) -> Pose2D:
    """`a`の逆変換。`compose(a, inverse(a))`は恒等姿勢 `(0, 0, 0)` になる。"""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(-(c * a.x + s * a.y), -(-s * a.x + c * a.y), wrap_angle(-a.yaw))


def between(a: Pose2D, b: Pose2D) -> Pose2D:
    """`a`から見た`b`の相対姿勢。`compose(a, between(a, b))`は`b`に一致する。"""
    return compose(inverse(a), b)


def integrate_twist(twist: Twist2D, dt: float) -> Pose2D:
    """車体座標系での一定`twist`を`dt`秒ぶん積分し、相対姿勢デルタを返す。

    非ホロノミック車両（`vy=0`固定）なら円弧、`vy!=0`（オムニホイール等）
    でも一定twist前提の厳密解になる。直線近似ではなく円弧で積むのは、
    高速走行時に直線近似だと初期値が数cmずれ、スキャンマッチの粗探索の
    格子1つぶんを食ってしまうため（`raspi/nav/deskew.py`の`integrate_pose`
    と同じ理由）。
    """
    vx, vy, w = twist.vx, twist.vy, twist.yaw_rate
    if abs(w) < _STRAIGHT_EPS:
        return Pose2D(vx * dt, vy * dt, 0.0)
    wt = w * dt
    s, c = math.sin(wt), math.cos(wt)
    dx = (vx * s + vy * (c - 1.0)) / w
    dy = (vx * (1.0 - c) + vy * s) / w
    return Pose2D(dx, dy, wrap_angle(wt))
