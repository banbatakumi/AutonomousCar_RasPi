"""バスに流れるメッセージ型と、UART パケットからの変換。

型は `types.py`、スケール換算と射影は `convert.py`。
**単位は SI。km/h や度への変換は GUI の表示直前だけ**（`docs/architecture.md` §5.1）。
"""

from .convert import (
    DISARM_COMMAND,
    FAULT_FLAGS,
    LIDAR_C_SATURATED_M,
    MAX_BRAKE_TORQUE_NM,
    MAX_TARGET_TORQUE_NM,
    SPEED_DEADBAND_MPS,
    ScanAssembler,
    StateBuilder,
    command_from_cmd,
    decode_flags,
)
from .types import (
    TOPIC_AUTO_CMD,
    TOPIC_AUTO_CTRL,
    TOPIC_AUTO_MAP,
    TOPIC_AUTO_STATE,
    TOPIC_CMD,
    TOPIC_DIAG_LINK,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_IMAGE_REAR,
    TOPIC_LOG_CTRL,
    TOPIC_SCAN,
    TOPIC_TYPES,
    TOPIC_VEHICLE_STATE,
    AutoCtrl,
    AutoMap,
    AutoState,
    DriveCmd,
    Heartbeat,
    ImageRef,
    LinkDiag,
    LogCtrl,
    MsgBase,
    Scan,
    VehicleState,
    type_for_topic,
)

__all__ = [
    "MsgBase", "VehicleState", "Scan", "DriveCmd", "LinkDiag", "ImageRef",
    "Heartbeat", "LogCtrl", "AutoCtrl", "AutoState", "AutoMap",
    "TOPIC_VEHICLE_STATE", "TOPIC_SCAN", "TOPIC_CMD", "TOPIC_DIAG_LINK",
    "TOPIC_IMAGE_FRONT", "TOPIC_IMAGE_REAR", "TOPIC_HB_PREFIX", "TOPIC_LOG_CTRL",
    "TOPIC_AUTO_CTRL", "TOPIC_AUTO_CMD", "TOPIC_AUTO_STATE", "TOPIC_AUTO_MAP",
    "TOPIC_TYPES", "type_for_topic",
    "StateBuilder", "ScanAssembler", "decode_flags", "command_from_cmd",
    "DISARM_COMMAND", "FAULT_FLAGS", "SPEED_DEADBAND_MPS", "LIDAR_C_SATURATED_M",
    "MAX_BRAKE_TORQUE_NM", "MAX_TARGET_TORQUE_NM",
]
