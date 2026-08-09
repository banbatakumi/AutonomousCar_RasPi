"""UART パケット → バスのメッセージ。**スケール換算が存在する唯一の場所。**

`proto/generated/packets.py` は整数の生値をそのまま持つ。ここで SI に直す。
換算をノードごとに書くと、必ずどこか1箇所でスケールを間違え、しかも
「値が10倍おかしい」という形でしか症状が出ないので発見が遅れる。

ここには2つ、**単なる単位換算では済まない処理**が入る。

1. **前輪オドメトリの射影**（`uart_protocol.md` §5.3）。
   前輪は操舵輪なので、走行距離は車体中心線距離の 1/cos(δ) 倍になる。
   **射影は累積値ではなく1周期ぶんの差分に対して行う。** 累積値に cos を掛けると、
   直進 100m のあとハンドルを 60° 切った瞬間に累積距離が半分になる。
2. **LiDAR セクタ12個の組み立て**。1周が揃うまで待ち、欠けたセクタは
   「見えなかった」として残す。**0m 埋めにすると「そこに壁がある」と読める。**
   ここで**センサ座標 → 車両座標の鏡像反転**も行う（`ScanAssembler` の解説）。
"""

from __future__ import annotations

import math

from ..proto import packets
from .types import DriveCmd, Scan, VehicleState

__all__ = ["StateBuilder", "ScanAssembler", "SPEED_DEADBAND_MPS",
           "decode_flags", "FAULT_FLAGS", "command_from_cmd", "DISARM_COMMAND",
           "MAX_BRAKE_TORQUE_NM"]

# ── スケール（uart_protocol.md §5.3。protocol.toml の META と一致させること） ──
_SPEED = 1e-3          # m/s
_YAW_RATE = 1e-3       # rad/s
_ANGLE = 1e-4          # rad
_ODOM = 1e-4           # m (0.1mm/LSB)
_ACCEL = 1e-3          # m/s²
_CURRENT = 1e-3        # A (mA/LSB)
_TORQUE = 1e-4         # N·m
_BATT_V = 0.05         # V
_BATT_A_DRIVE = 0.05   # A
_BATT_A_SIGNAL = 0.02  # A
_ULTRASONIC = 0.02     # m (2cm/LSB)
_LIDAR_MM = 1e-3       # m
_LIDAR_C = 0.02        # m (2cm/LSB、255 は飽和)

# センサ角 sector_idx*30+i [deg] → 車両角 (360 - センサ角) % 360 [deg] の対応表。
# 毎セクタ (120Hz) 組み立て直す意味がないので先に引いておく
_DEG = [[(360 - s * 30 - i) % 360 for i in range(30)] for s in range(12)]

#: 圧縮フォーマットの `255` が意味する距離。**「5.10m ちょうど」ではなく「5.10m 以上」**
LIDAR_C_SATURATED_M = 255 * _LIDAR_C

#: 前輪エンコーダはアナログ絶対角の ADC 読みで、静止付近は符号がばたつく
#: （`uart_protocol.md` §5.3）。生値は残したまま `stopped` で示す
SPEED_DEADBAND_MPS = 0.05

_I32_SPAN = 1 << 32

#: 後輪1輪あたりの制動トルクの上限 [N·m]（STM32 の `DRIVE_MAX_BRAKE_TORQUE_NM`）。
#: **STM32 側でもクランプされるが、GUI のスライダ上限をここから引くために持つ。**
#: 超えた値を送ると黙って丸められ、「スライダを上げても効きが変わらない」に見える
MAX_BRAKE_TORQUE_NM = 0.075

#: 立っていたら名前で持ち回る fault。ビットのまま流すと GUI 側で意味を再定義する羽目になる
FAULT_FLAGS = {
    "drive_overcurrent": packets.FLG_FAULT_DRIVE_OVERCURRENT,
    "signal_overcurrent": packets.FLG_FAULT_SIGNAL_OVERCURRENT,
    "drive_undervoltage": packets.FLG_FAULT_DRIVE_UNDERVOLTAGE,
    "signal_undervoltage": packets.FLG_FAULT_SIGNAL_UNDERVOLTAGE,
}


def _diff_i32(new: int, old: int) -> int:
    """32bit ラップを考慮した差分。

    `odom_dist` は 0.1mm × 2^32 = 429.5km でラップするのでミニカーでは到達しないが、
    負値からの再起動などで巨大な差分を作らないように必ず畳んでおく。
    """
    return ((new - old + (1 << 31)) % _I32_SPAN) - (1 << 31)


def decode_flags(flags: int) -> dict:
    """`TELEMETRY.flags` を名前付きに開く。"""
    return {
        "mode": flags & packets.FLG_MODE_MASK,
        "armed": bool(flags & packets.FLG_ARMED),
        "estop_active": bool(flags & packets.FLG_ESTOP_ACTIVE),
        "uart_timeout": bool(flags & packets.FLG_UART_TIMEOUT),
        "tc_active": bool(flags & packets.FLG_TC_ACTIVE),
        "tv_active": bool(flags & packets.FLG_TV_ACTIVE),
        "imu_ok": bool(flags & packets.FLG_IMU_OK),
        "lidar_ok": bool(flags & packets.FLG_LIDAR_OK),
        "steer_center_valid": bool(flags & packets.FLG_STEER_CENTER_VALID),
        "drive_power_locked": bool(flags & packets.FLG_DRIVE_POWER_LOCKED),
        "faults": [n for n, b in FAULT_FLAGS.items() if flags & b],
    }


class StateBuilder:
    """`Telemetry` → `VehicleState`。**前回値を持つので状態を持つ。**

    射影した累積距離 `odom_center` を進めるために前周期の `odom_dist` が要る。
    関数にできないのはそのため。

    :param max_gap_m: 1周期の前輪移動量がこれを超えたら「連続していない」と見なし、
        `odom_center` を進めない。起動直後・再接続直後に巨大な差分で
        累積距離が飛ぶのを防ぐ（50Hz・3m/s でも 1周期 6cm）
    """

    def __init__(self, max_gap_m: float = 0.5) -> None:
        self.max_gap_m = max_gap_m
        self.odom_center = 0.0
        self._prev_odom: list[int] | None = None
        #: 連続性が切れて `odom_center` を進めなかった回数。診断用
        self.odom_jumps = 0

    def reset(self) -> None:
        self._prev_odom = None

    def build(self, t: packets.Telemetry, t_capture_ns: int) -> VehicleState:
        steer = t.steer_actual * _ANGLE
        speed = t.speed * _SPEED
        cos_d = math.cos(steer)

        # ── 前輪オドメトリの射影（差分に対して行う） ──
        if self._prev_odom is not None:
            d_fl = _diff_i32(t.odom_dist[0], self._prev_odom[0]) * _ODOM
            d_fr = _diff_i32(t.odom_dist[1], self._prev_odom[1]) * _ODOM
            d_front = (d_fl + d_fr) / 2.0
            if abs(d_front) <= self.max_gap_m:
                self.odom_center += d_front * cos_d
            else:
                self.odom_jumps += 1
        self._prev_odom = list(t.odom_dist)

        # ── スリップ（前輪は射影してから引く。忘れると舵角30°で常時15.5%出る） ──
        ws = [w * _SPEED for w in t.wheel_speed]
        slip_front = [ws[0] * cos_d - speed, ws[1] * cos_d - speed]
        slip_rear = [ws[2] - speed, ws[3] - speed]

        # ── MD が無言なら温度を信じない（status が最後の値で固まるため） ──
        comm = [bool(s & packets.MDS_COMM_OK) for s in t.md_status]
        temp: list[int | None] = [
            t.temp[0] if comm[0] else None,
            t.temp[1] if comm[1] else None,
            t.temp[2] if comm[2] else None,
            t.temp[3],                        # MCU は MD ではないので常に有効
        ]

        return VehicleState(
            t_capture=t_capture_ns,
            speed=speed,
            yaw_rate=t.yaw_rate * _YAW_RATE,
            steer_actual=steer,
            steer_cmd_echo=t.steer_cmd_echo * _ANGLE,
            wheel_speed=ws,
            odom_dist=[v * _ODOM for v in t.odom_dist],
            accel=[t.accel_x * _ACCEL, t.accel_y * _ACCEL, t.accel_z * _ACCEL],
            pitch=t.pitch * _ANGLE,
            roll=t.roll * _ANGLE,
            motor_current=[v * _CURRENT for v in t.motor_current],
            torque_cmd=[v * _TORQUE for v in t.torque_cmd],
            temp=temp,
            batt_voltage=[t.batt_voltage_drive * _BATT_V, t.batt_voltage_signal * _BATT_V],
            batt_current=[t.batt_current_drive * _BATT_A_DRIVE,
                          t.batt_current_signal * _BATT_A_SIGNAL],
            us_front=(t.us_front * _ULTRASONIC) if t.us_front else None,
            us_rear=(t.us_rear * _ULTRASONIC) if t.us_rear else None,
            md_status=list(t.md_status),
            flags=t.flags,
            cmd_seq_echo=t.cmd_seq_echo,
            t_stm_us=t.t_us,
            odom_center=self.odom_center,
            slip_front=slip_front,
            slip_rear=slip_rear,
            stopped=abs(speed) < SPEED_DEADBAND_MPS,
            **decode_flags(t.flags),
        )


def _clamp(v: float, lo: int, hi: int) -> int:
    """SI → 整数スケール。**必ず飽和させる。**

    `struct.pack` は範囲外で例外を投げるので、クランプを忘れると
    「舵を切りすぎた瞬間に送信が例外で止まり、STM32 が COMMAND 途絶で
    自動ブレーキに入る」という分かりにくい壊れ方をする。
    """
    return max(lo, min(hi, int(round(v))))


#: 何を送ればよいか分からないときに送るもの。**停止が既定値。**
DISARM_COMMAND = packets.Command(
    mode=packets.Mode.DISARM, flags=0,
    target_speed=0, target_steer=0, accel_limit=0, steer_rate_limit=0,
    brake_torque=0)


def command_from_cmd(cmd: DriveCmd, *, allow_arm: bool = False,
                     max_speed: float | None = None,
                     max_steer: float | None = None) -> packets.Command:
    """`DriveCmd`（SI）→ `COMMAND`(0x10)。

    :param allow_arm: False なら **mode と arm を強制的に DISARM に落とす**。
        GUI から WS・バスを経由してモータが回る経路は、明示的に解禁する日まで
        ここで断つ。判断を1箇所に閉じておかないと「どこかで解禁されていた」が起きる
    :param max_speed: Pi 側の上限 [m/s]。STM32 側にも上限はあるが、
        **上位が壊れたときに下位だけが守る構造にしない**（多層防御）
    """
    if not allow_arm:
        return DISARM_COMMAND

    speed = cmd.target_speed
    steer = cmd.target_steer
    if max_speed is not None:
        speed = max(-max_speed, min(max_speed, speed))
    if max_steer is not None:
        steer = max(-max_steer, min(max_steer, steer))

    flags = 0
    if cmd.arm:
        flags |= packets.CMD_FLG_ARM
    if cmd.brake:
        flags |= packets.CMD_FLG_BRAKE
    if cmd.horn:
        flags |= packets.CMD_FLG_HORN
    if cmd.passing:
        flags |= packets.CMD_FLG_PASSING
    # bit3-4 の2ビット。**3 は予約なので送らない**（STM32 は NORMAL として扱うが、
    # 「予約値を送っておいて向こうの解釈に頼る」形にはしない）
    light = cmd.light_mode if cmd.light_mode in (0, 1, 2) else 0
    flags |= (light << packets.CMD_FLG_LIGHT_SHIFT) & packets.CMD_FLG_LIGHT_MASK

    # mode=3 は予約。送ってはいけない（STM32 は無視して現在のモードを維持する）
    mode = cmd.mode if cmd.mode in (0, 1, 2) else 0
    return packets.Command(
        mode=mode, flags=flags,
        target_speed=_clamp(speed / _SPEED, -32768, 32767),
        target_steer=_clamp(steer / _ANGLE, -32768, 32767),
        accel_limit=_clamp(cmd.accel_limit / _SPEED, 0, 65535),
        steer_rate_limit=_clamp(cmd.steer_rate_limit / _YAW_RATE, 0, 65535),
        brake_torque=_brake_torque_raw(cmd.brake_torque),
    )


def _brake_torque_raw(nm: float) -> int:
    """[N·m] → `brake_torque` の生値。**0 は「未指定＝最大制動」を意味する。**

    そのため、正の値が丸めで 0 に落ちるのを禁じる（最小 1 LSB = 0.0001 N·m に留める）。
    これをやらないと「スライダを最弱にしたら最大制動になった」という、
    操作と挙動が正反対になる壊れ方をする。上限は STM32 でもクランプされるが、
    **どこで頭打ちになったか分からなくなる**ので Pi 側でも掛けておく。
    """
    if nm <= 0.0:
        return 0
    return _clamp(min(nm, MAX_BRAKE_TORQUE_NM) / _TORQUE, 1, 65535)


class ScanAssembler:
    """`LIDAR_SECTOR` 12個 → `Scan` 1周ぶん。

    **セクタ番号が戻った時点で1周の完了とみなす。** 「12個そろったら」にすると、
    1個でも落ちた周が永久に完成せず、次の周と混ざる。

    欠けたセクタは `dist=0`（無効）のまま `sector_seen=False` にする。
    **0 を「距離0の障害物」と解釈してはならない。**

    ## センサ座標 → 車両座標（左右の鏡像反転）

    **LD06 は PCB 面を下にして裏向きに取り付けてある。** 0° は機体前方を向いているが、
    角度が増える向きが上から見て車両座標（反時計回りが正）と逆になるため、
    `sector_idx * 30 + i` をそのまま度として使うと**点群が左右反転する**。
    STM32 側は「センサ基準の角度をそのまま送り、解釈は Pi 側で行う」設計なので
    （`stm32_interface.md`、STM32 の `src/sensing/lidar.h`）、ここで

        車両角 = (360 - センサ角) % 360

    に直す。**`Scan.dist` の添字は車両角**であり、以降のノード・GUI・地図生成は
    反転を意識しなくてよい。

    この反転で1周ぶんの点は**添字の降順に取得される**ことになる（センサ角が
    増える = 車両角が減る）。`sector_t_ns` / `sector_dur_us` / `sector_seen` も
    車両座標のセクタ番号 `11 - sector_idx` へ移して持つが、その結果
    **セクタ内の時刻は添字が大きい側から流れ、30° の境界も1° ずれる**。
    点の時刻は必ず**車両角をセンサ角に戻してから**引くこと（`Scan.sector_dur_us` の式）。
    """

    def __init__(self) -> None:
        self._reset()
        #: 落としたセクタの累計。`lidar_ok` は 300ms の完全途絶しか見ないので、
        #: 「たまに落ちている」はここでしか見えない
        self.sectors_lost = 0
        self.scans = 0

    def _reset(self) -> None:
        self._dist = [0.0] * 360
        self._intensity: list[int] | None = None
        self._saturated: list[bool] | None = None
        self._t_ns = [0] * 12
        self._dur = [0] * 12
        self._seen = [False] * 12
        self._rot = 0.0
        self._fmt = 0
        self._last_idx = -1
        self._any = False

    def _emit(self) -> Scan:
        seen_n = sum(self._seen)
        self.sectors_lost += 12 - seen_n
        self.scans += 1
        # 添字は車両座標なので取得時刻の順に並んでいない。先頭ではなく最小を取る
        t0 = min((t for t in self._t_ns if t), default=0)
        scan = Scan(
            t_capture=t0,
            dist=self._dist,
            sector_t_ns=self._t_ns,
            sector_dur_us=self._dur,
            sector_seen=self._seen,
            rot_speed_dps=self._rot,
            intensity=self._intensity,
            saturated=self._saturated,
            lidar_format=self._fmt,
        )
        self._reset()
        return scan

    def feed(self, msg, t_rx_ns: int) -> Scan | None:
        """セクタを1個取り込む。1周が完成したらその `Scan` を返す。

        :param msg: `LidarSector` / `LidarSectorI` / `LidarSectorC`
        :param t_rx_ns: このセクタ先頭点の Pi 時刻。時刻同期が未収束なら受信時刻でよい
        """
        idx = msg.sector_idx
        if not (0 <= idx < 12):
            return None

        done = None
        if self._any and idx <= self._last_idx:
            done = self._emit()

        deg = _DEG[idx]
        if isinstance(msg, packets.LidarSectorC):
            self._fmt = 2
            if self._saturated is None:
                self._saturated = [False] * 360
            for i, d in enumerate(msg.dist):
                self._dist[deg[i]] = d * _LIDAR_C
                self._saturated[deg[i]] = (d == 255)
        else:
            self._fmt = 1 if isinstance(msg, packets.LidarSectorI) else 0
            for i, d in enumerate(msg.dist):
                self._dist[deg[i]] = d * _LIDAR_MM
            if isinstance(msg, packets.LidarSectorI):
                if self._intensity is None:
                    self._intensity = [0] * 360
                for i, v in enumerate(msg.intensity):
                    self._intensity[deg[i]] = v

        # 反転後のセクタ番号。このパケットの30点は車両角 30*out+1 〜 30*out+30 に入るので、
        # 29点が out に、**先頭の1点 (i=0) だけが隣の out+1 の先頭**に落ちる。
        # 時刻を引くときは車両角をセンサ角に戻すこと（`Scan.sector_dur_us` の式）。
        # 車両角のまま 30*s+j で式を立てると j=0 の点で最大1周ぶん外す
        out = 11 - idx
        self._t_ns[out] = t_rx_ns
        self._dur[out] = msg.duration_us
        self._seen[out] = True
        self._rot = float(msg.rot_speed_dps)
        self._last_idx = idx
        self._any = True
        return done
