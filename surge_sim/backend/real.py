"""実機UARTバックエンド（スタブ）。

Raspberry Pi から STM32 へ UART(250000bps) で制御指令を送り、STM32 が取得した
LD06 LiDARデータを受信する。実装はPhase以降。ここではシグネチャと
プロトコル仕様のみを定義する。

送信パケット仕様:
    [0xAA][0x01][speed_H][speed_L][steer_H][steer_L][CRC8]
    - 0xAA   : ヘッダ
    - 0x01   : コマンドID（制御指令）
    - speed  : mm/s 符号付き16bit ビッグエンディアン
    - steer  : 0.01deg単位 符号付き16bit ビッグエンディアン
    - CRC8   : ヘッダ(0xAA)以外の全バイトのXOR
"""

from __future__ import annotations

import serial  # noqa: F401  (実機で使用)

from core.interfaces import (
    BackendBase,
    ControlCommand,
    LidarScan,
    VehicleState,
)

# UART定数
BAUDRATE = 250000
PACKET_HEADER = 0xAA
CMD_ID_CONTROL = 0x01


def _crc8_xor(data: bytes) -> int:
    """ヘッダ以外の全バイトのXORによるCRC8。"""
    crc = 0
    for b in data:
        crc ^= b
    return crc & 0xFF


def _encode_command(cmd: ControlCommand) -> bytes:
    """ControlCommand を送信パケットへエンコードする（参考実装）。

    speed[mm/s], steer[0.01deg] を符号付き16bitビッグエンディアンで格納し、
    末尾にCRC8(XOR)を付与する。
    """
    speed_mm = int(round(cmd.target_speed * 1000.0))
    steer_cd = int(round(cmd.target_steer * 100.0))
    payload = bytes([
        CMD_ID_CONTROL,
        (speed_mm >> 8) & 0xFF, speed_mm & 0xFF,
        (steer_cd >> 8) & 0xFF, steer_cd & 0xFF,
    ])
    crc = _crc8_xor(payload)
    return bytes([PACKET_HEADER]) + payload + bytes([crc])


class RealBackend(BackendBase):
    """実機UARTバックエンド（Phase以降で実装）。"""

    def __init__(self, port: str = "/dev/serial0", baudrate: int = BAUDRATE,
                 timeout: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None  # type: ignore[assignment]

    def get_lidar_scan(self) -> LidarScan:
        raise NotImplementedError("実機バックエンドは未実装")

    def send_command(self, cmd: ControlCommand) -> None:
        raise NotImplementedError("実機バックエンドは未実装")

    def get_vehicle_state(self) -> VehicleState:
        raise NotImplementedError("実機バックエンドは未実装")

    def step(self, dt: float) -> None:
        raise NotImplementedError("実機バックエンドは未実装")

    def reset(self) -> None:
        raise NotImplementedError("実機バックエンドは未実装")
