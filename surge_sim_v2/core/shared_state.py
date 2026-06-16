"""全スレッドが読み書きする中央データストア。

以下のスレッドから同時アクセスされる：
- 制御ループスレッド（50Hz）：vehicle/lidar書き込み、command読み込み
- WebSocket配信スレッド（20Hz）：全データ読み込み
- pygameレンダラースレッド（60Hz）：vehicle/lidar読み込み（シミュのみ）
- FastAPIスレッド：command書き込み、mode書き込み

すべての読み書きは self._lock（threading.Lock）で保護してスレッドセーフを保証する。
"""
from __future__ import annotations

import threading
import time

import numpy as np

from .interfaces import (
    ConnectionStatus,
    ControlCommand,
    CourseMap,
    DriveMode,
    LidarScan,
    LocalizationResult,
    OccupancyGrid,
    SystemState,
    VehicleState,
)


def _empty_vehicle() -> VehicleState:
    return VehicleState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, time.time())


def _empty_lidar() -> LidarScan:
    angles = np.arange(360, dtype=float)
    distances = np.full(360, 12.0, dtype=float)
    return LidarScan(distances=distances, angles=angles, timestamp=time.time())


def _empty_localization() -> LocalizationResult:
    return LocalizationResult(0.0, 0.0, 0.0, 0.0, "cheat", time.time())


def _empty_connection() -> ConnectionStatus:
    return ConnectionStatus(
        websocket_connected=False,
        last_received_at=time.time(),
        uart_connected=False,
        lidar_receiving=False,
        stm32_connected=False,
        latency_ms=0.0,
    )


class SharedState:
    """スレッドセーフな中央データストア。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()

        self._vehicle: VehicleState = _empty_vehicle()
        self._lidar: LidarScan = _empty_lidar()
        self._localization: LocalizationResult = _empty_localization()
        self._slam_map: OccupancyGrid | None = None
        self._course_map: CourseMap | None = None
        self._connection: ConnectionStatus = _empty_connection()

        self._command: ControlCommand | None = None
        self._mode: DriveMode = DriveMode.MANUAL

        self._is_paused: bool = False
        self._is_recording: bool = False
        self._speed_multiplier: float = 1.0
        self._autonomous_running: bool = False
        self._emergency_stop: bool = False

    # ---- 書き込み ---------------------------------------------------------
    def update_vehicle(self, state: VehicleState) -> None:
        with self._lock:
            self._vehicle = state

    def update_lidar(self, scan: LidarScan) -> None:
        with self._lock:
            self._lidar = scan

    def update_slam_map(self, grid: OccupancyGrid) -> None:
        with self._lock:
            self._slam_map = grid

    def update_localization(self, result: LocalizationResult) -> None:
        with self._lock:
            self._localization = result

    def update_course_map(self, course_map: CourseMap) -> None:
        with self._lock:
            self._course_map = course_map

    def update_connection(self, status: ConnectionStatus) -> None:
        with self._lock:
            self._connection = status

    def set_command(self, cmd: ControlCommand) -> None:
        with self._lock:
            self._command = cmd

    def set_mode(self, mode: DriveMode) -> None:
        with self._lock:
            self._mode = mode

    def set_paused(self, paused: bool) -> None:
        with self._lock:
            self._is_paused = paused

    def set_recording(self, recording: bool) -> None:
        with self._lock:
            self._is_recording = recording

    def set_speed_multiplier(self, multiplier: float) -> None:
        with self._lock:
            self._speed_multiplier = float(multiplier)

    def set_autonomous_running(self, running: bool) -> None:
        with self._lock:
            self._autonomous_running = running

    def set_emergency_stop(self, active: bool) -> None:
        with self._lock:
            self._emergency_stop = active

    # ---- 読み込み ---------------------------------------------------------
    def get_command(self) -> ControlCommand | None:
        with self._lock:
            return self._command

    def get_mode(self) -> DriveMode:
        with self._lock:
            return self._mode

    def get_vehicle(self) -> VehicleState:
        with self._lock:
            return self._vehicle

    def get_lidar(self) -> LidarScan:
        with self._lock:
            return self._lidar

    def get_localization(self) -> LocalizationResult:
        with self._lock:
            return self._localization

    def get_slam_map(self) -> OccupancyGrid | None:
        with self._lock:
            return self._slam_map

    def get_course_map(self) -> CourseMap | None:
        with self._lock:
            return self._course_map

    def is_paused(self) -> bool:
        with self._lock:
            return self._is_paused

    def is_recording(self) -> bool:
        with self._lock:
            return self._is_recording

    def is_emergency_stop(self) -> bool:
        with self._lock:
            return self._emergency_stop

    def get_speed_multiplier(self) -> float:
        with self._lock:
            return self._speed_multiplier

    def get_autonomous_running(self) -> bool:
        with self._lock:
            return self._autonomous_running

    def get_system_state(self) -> SystemState:
        """全状態のスナップショットを返す（ロック内で一括コピー）。"""
        with self._lock:
            return SystemState(
                mode=self._mode,
                vehicle=self._vehicle,
                lidar=self._lidar,
                localization=self._localization,
                slam_map=self._slam_map,
                course_map=self._course_map,
                connection=self._connection,
                is_paused=self._is_paused,
                is_recording=self._is_recording,
                speed_multiplier=self._speed_multiplier,
                autonomous_running=self._autonomous_running,
                timestamp=time.time(),
            )
