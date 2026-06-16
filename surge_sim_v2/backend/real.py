"""実機UARTバックエンド（骨格実装）。

クラス構造・初期化・定数定義・パケット仕様は完全に実装する。
実際の UART 送受信ループ（get_lidar_scan/get_vehicle_state/step）は
NotImplementedError とし、Phase5（実機統合）で埋める。

送信パケット仕様：
[0xAA][0x01][speed_H][speed_L][steer_H][steer_L][CRC8]
  - speed：mm/s 符号付き16bit ビッグエンディアン
  - steer：0.01deg単位 符号付き16bit ビッグエンディアン
  - CRC8：0x01以降の全バイトのXOR

受信パケット仕様（STM32からのLiDAR構造化データ）：
[0xBB][LEN][n_points(1B)][angle_0_H][angle_0_L][dist_0_H][dist_0_L]...[CRC8]
  - angle：0.01deg単位 符号なし16bit ビッグエンディアン
  - dist：mm単位 符号なし16bit ビッグエンディアン
"""
from __future__ import annotations

import struct
import threading
import time

import numpy as np

from backend.base import BackendBase
from core.interfaces import ConnectionStatus, ControlCommand, LidarScan, VehicleState
from core.shared_state import SharedState

BAUDRATE = 250000
HEADER_TX = 0xAA
HEADER_RX = 0xBB
COMMAND_ID = 0x01

# ウォッチドッグ閾値 [s]
RX_TIMEOUT_S = 0.5


class RealBackend(BackendBase):
    def __init__(self, port: str, baudrate: int = BAUDRATE,
                 shared_state: SharedState | None = None) -> None:
        self.port = port
        self.baudrate = baudrate
        self.shared = shared_state

        self._serial = None            # serial.Serial を Phase5 で開く
        self._last_rx_at = time.time()
        self._uart_connected = False
        self._stm32_connected = False
        self._lidar_receiving = False

        self._watchdog_thread: threading.Thread | None = None
        self._watchdog_running = False

    # ---- パケットエンコード/デコード -------------------------------------
    def _encode_command(self, cmd: ControlCommand) -> bytes:
        """ControlCommand を送信バイト列に変換する。"""
        speed_mm = int(round(cmd.target_speed * 1000.0))        # m/s -> mm/s
        steer_cd = int(round(cmd.target_steer * 100.0))         # deg -> 0.01deg
        speed_mm = max(-32768, min(32767, speed_mm))
        steer_cd = max(-32768, min(32767, steer_cd))
        body = struct.pack(">Bhh", COMMAND_ID, speed_mm, steer_cd)
        crc = self._calc_crc8(body)
        return bytes([HEADER_TX]) + body + bytes([crc])

    def _decode_lidar_packet(self, data: bytes) -> LidarScan:
        """STM32 からの LiDAR 構造化パケットを LidarScan に変換する。

        data: [0xBB][LEN][n_points][angle_H][angle_L][dist_H][dist_L]...[CRC8]
        """
        if len(data) < 4 or data[0] != HEADER_RX:
            raise ValueError("invalid LiDAR packet header")
        n_points = data[2]
        payload = data[3:3 + n_points * 4]
        crc_recv = data[3 + n_points * 4]
        if self._calc_crc8(data[1:3 + n_points * 4]) != crc_recv:
            raise ValueError("CRC mismatch")

        angles = np.zeros(n_points, dtype=float)
        dists = np.zeros(n_points, dtype=float)
        for i in range(n_points):
            off = i * 4
            angle_cd, dist_mm = struct.unpack_from(">HH", payload, off)
            angles[i] = angle_cd / 100.0      # 0.01deg -> deg
            dists[i] = dist_mm / 1000.0       # mm -> m
        return LidarScan(distances=dists, angles=angles, timestamp=time.time())

    @staticmethod
    def _calc_crc8(data: bytes) -> int:
        """0x01 以降（ヘッダを除く）全バイトの XOR。"""
        crc = 0
        for b in data:
            crc ^= b
        return crc & 0xFF

    # ---- ウォッチドッグ ---------------------------------------------------
    def start_watchdog(self) -> None:
        """最後の受信から RX_TIMEOUT_S 経過で緊急停止コマンドを書き込む。"""
        if self._watchdog_running:
            return
        self._watchdog_running = True
        self._watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog_thread.start()

    def stop_watchdog(self) -> None:
        self._watchdog_running = False
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=1.0)

    def _watchdog_loop(self) -> None:
        while self._watchdog_running:
            if time.time() - self._last_rx_at > RX_TIMEOUT_S:
                # 通信断 → 緊急停止指令を SharedState に書き込む
                if self.shared is not None:
                    self.shared.set_command(
                        ControlCommand(target_speed=0.0, target_steer=0.0,
                                       timestamp=time.time())
                    )
                    self.shared.set_emergency_stop(True)
                self._uart_connected = False
                self._stm32_connected = False
                self._lidar_receiving = False
            time.sleep(0.1)

    # ---- BackendBase 実装 -------------------------------------------------
    def send_command(self, cmd: ControlCommand) -> None:
        # packet = self._encode_command(cmd); self._serial.write(packet)
        raise NotImplementedError("Phase5（実機UART送信）で実装")

    def get_lidar_scan(self) -> LidarScan:
        raise NotImplementedError("Phase5（実機UART受信）で実装")

    def get_vehicle_state(self) -> VehicleState:
        raise NotImplementedError("Phase5（オドメトリ/推定）で実装")

    def step(self, dt: float) -> None:
        raise NotImplementedError("Phase5（受信ポーリング）で実装")

    def reset(self) -> None:
        raise NotImplementedError("Phase5で実装")

    def get_connection_status(self) -> ConnectionStatus:
        now = time.time()
        return ConnectionStatus(
            websocket_connected=True,
            last_received_at=self._last_rx_at,
            uart_connected=self._uart_connected,
            lidar_receiving=self._lidar_receiving and (now - self._last_rx_at) < RX_TIMEOUT_S,
            stm32_connected=self._stm32_connected,
            latency_ms=0.0,
        )
