"""全データクラス・抽象クラス・DriveMode定義。

実機・シミュレーション共通の「データ契約」。core/ 以下の制御ロジックは
このモジュールのデータクラスにのみ依存し、バックエンドの実体は知らない。
"""
from __future__ import annotations

from dataclasses import dataclass
from abc import ABC, abstractmethod
from enum import Enum

import numpy as np


class DriveMode(Enum):
    MANUAL = "manual"
    MAP_BUILDING = "map_building"
    AUTONOMOUS = "autonomous"


@dataclass
class VehicleState:
    x: float                    # [m]
    y: float                    # [m]
    heading: float              # [deg] 0=East、反時計回り正
    speed: float                # [m/s]
    acceleration: float         # [m/s^2]
    steer_angle: float          # [deg]
    timestamp: float            # [s]


@dataclass
class LidarScan:
    distances: np.ndarray       # [m] 360要素
    angles: np.ndarray          # [deg] 360要素
    timestamp: float


@dataclass
class ControlCommand:
    target_speed: float         # [m/s]
    target_steer: float         # [deg]
    timestamp: float


@dataclass
class LocalizationResult:
    x: float
    y: float
    heading: float              # [deg]
    confidence: float           # 0.0〜1.0
    source: str                 # "cheat" | "slam" | "ekf"
    timestamp: float


@dataclass
class OccupancyGrid:
    grid: np.ndarray            # shape(H,W) 0=free,100=occupied,-1=unknown
    resolution: float           # [m/cell]
    origin_x: float             # [m] グリッド原点のワールド座標
    origin_y: float             # [m]
    timestamp: float


@dataclass
class CourseMap:
    left_wall: np.ndarray       # shape(N,2) [m]
    right_wall: np.ndarray      # shape(N,2) [m]
    center_line: np.ndarray     # shape(N,2) [m]
    racing_line: np.ndarray     # shape(N,2) [m]（Phase4で追加）
    width_profile: np.ndarray   # shape(N,) [m]


@dataclass
class ConnectionStatus:
    websocket_connected: bool
    last_received_at: float     # [s] Unix時刻
    uart_connected: bool        # 実機のみ有効
    lidar_receiving: bool       # LiDARデータ受信中か
    stm32_connected: bool       # STM32通信状態（実機のみ）
    latency_ms: float           # WebSocket往復レイテンシ [ms]


@dataclass
class SystemState:
    """WebSocketで配信する全状態をまとめたクラス"""
    mode: DriveMode
    vehicle: VehicleState
    lidar: LidarScan
    localization: LocalizationResult
    slam_map: OccupancyGrid | None
    course_map: CourseMap | None
    connection: ConnectionStatus
    is_paused: bool
    is_recording: bool
    speed_multiplier: float     # シミュ専用（実機では1.0固定）
    autonomous_running: bool    # 自律走行中フラグ
    timestamp: float


class BackendBase(ABC):
    @abstractmethod
    def get_lidar_scan(self) -> LidarScan: ...

    @abstractmethod
    def send_command(self, cmd: ControlCommand) -> None: ...

    @abstractmethod
    def get_vehicle_state(self) -> VehicleState: ...

    @abstractmethod
    def step(self, dt: float) -> None: ...

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def get_connection_status(self) -> ConnectionStatus: ...
