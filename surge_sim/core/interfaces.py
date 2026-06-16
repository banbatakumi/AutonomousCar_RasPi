"""SURGE Mark.2 共通インターフェース定義。

シミュレータと実機で共通利用する全データクラスとバックエンド抽象クラスを
ここに集約する。実機・シミュレータ・各制御モジュールはこのファイルの型のみに
依存することで、コードの共通化を実現する。

座標系の約束:
    - x, y: ワールド座標 [m]
    - heading: [deg] 0=East(+x方向)、反時計回りを正とする
    - 角度は基本 deg で扱い、内部計算で必要に応じて rad に変換する
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np


# ---------------------------------------------------------------------------
# データクラス群
# ---------------------------------------------------------------------------
@dataclass
class VehicleState:
    """車両の真の状態（シミュレータの物理演算が保持する）。"""

    x: float = 0.0                # [m]
    y: float = 0.0                # [m]
    heading: float = 0.0         # [deg] 0=East, 反時計回り正
    speed: float = 0.0           # [m/s]
    acceleration: float = 0.0    # [m/s^2]
    steer_angle: float = 0.0     # [deg]
    timestamp: float = 0.0       # [s]


@dataclass
class LidarScan:
    """LD06 1スキャン分のデータ（360点）。"""

    distances: np.ndarray        # [m] 360要素
    angles: np.ndarray           # [deg] 360要素
    timestamp: float = 0.0       # [s]


@dataclass
class ControlCommand:
    """制御ループが下位（STM32 / 物理演算）へ送る指令。"""

    target_speed: float = 0.0    # [m/s]
    target_steer: float = 0.0    # [deg]
    timestamp: float = 0.0       # [s]


@dataclass
class LocalizationResult:
    """自己位置推定の結果。"""

    x: float = 0.0               # [m]
    y: float = 0.0               # [m]
    heading: float = 0.0         # [deg]
    confidence: float = 0.0      # 0.0〜1.0
    source: str = "cheat"        # "cheat" | "slam" | "ekf"
    timestamp: float = 0.0       # [s]


@dataclass
class CourseMap:
    """コース形状（境界・中心線・レーシングライン）。"""

    left_wall: np.ndarray        # shape(N,2) 左壁点列 [m]
    right_wall: np.ndarray       # shape(N,2) 右壁点列 [m]
    center_line: np.ndarray      # shape(N,2) 中心線点列 [m]
    racing_line: np.ndarray      # shape(N,2) レーシングライン（Phase4で追加）
    width_profile: np.ndarray    # shape(N,) 各点でのコース幅 [m]


@dataclass
class OccupancyGrid:
    """SLAM占有格子地図。"""

    grid: np.ndarray             # shape(H,W) 0=free, 1=occupied, -1=unknown
    resolution: float            # [m/cell]
    origin_x: float              # [m] グリッド原点のワールド座標
    origin_y: float              # [m]
    timestamp: float = 0.0       # [s]


# ---------------------------------------------------------------------------
# バックエンド抽象クラス
# ---------------------------------------------------------------------------
class BackendBase(ABC):
    """シミュレータ／実機を抽象化する共通インターフェース。

    制御ロジックはこのインターフェースのみに依存する。
    """

    @abstractmethod
    def get_lidar_scan(self) -> LidarScan:
        """最新のLiDARスキャンを取得する。"""

    @abstractmethod
    def send_command(self, cmd: ControlCommand) -> None:
        """制御指令を下位システムへ送る。"""

    @abstractmethod
    def get_vehicle_state(self) -> VehicleState:
        """車両状態を取得する（シミュレータは真値、実機は推定／オドメトリ）。"""

    @abstractmethod
    def step(self, dt: float) -> None:
        """内部状態を dt 秒進める（実機ではno-op）。"""

    @abstractmethod
    def reset(self) -> None:
        """初期状態へリセットする。"""
