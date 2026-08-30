"""仮想 STM32 ファーム — パケットを解釈し、車両モデルを回し、パケットを作る。

実機の STM32 が UART の向こうでやっていることを Python で真似る。**本物のバイト列**を
作って返すので、CRC・フレーミング・SEQ 追跡・スケール換算はすべて実機と同じコードが動く。

## 実機と意図的に違うところ

- **E-Stop は自動発動しない。** GPIO6 ハートビートが Mac に無いため（`--no-heartbeat`
  と同じ扱い）。代わりに `sim.gui` から手で発動できる
- **TC/TV は実装していない。** タイヤモデルが無いので介入する材料が無い
- 電流・温度・電圧は「それらしい」値。負荷に応じて緩く動くだけで物理的な根拠は無い

## 時計は Pi と独立にしてある

`t_us` は仮想 STM32 自身の時計で、起動オフセットとドリフト（既定 +3378ppm ＝ 実車の実測値）
を持つ。**Pi の時計をそのまま返すと時刻同期コードが仕事をしているか確認できない。**
"""

from __future__ import annotations

import math
import random

import numpy as np

from raspi.proto import FrameEncoder, FrameParser, packets
from raspi.proto.generated.packets import P2S_TYPES, PROTOCOL_VERSION

from .params import SimParams
from .vehicle import DriveInput, VehicleModel, VehicleSpec

__all__ = ["VirtualStm32", "FW_ID"]

NS = 1_000_000_000

#: 仮想であることが一目で分かる fw_id。実機のファームと衝突しない値にしてある
FW_ID = 0x51310000            # "S1M"

#: 250000 bps 8N1 = 10 bit/byte。1バイトの線上時間 [ns]。
#: **これを入れないと LiDAR とテレメトリが同時刻に到着し、実機より綺麗になりすぎる**
BYTE_NS = 10 * NS // 250_000  # 40,000 ns

TELEMETRY_HZ = 50
STATS_HZ = 1

#: `COMMAND` がこれだけ途絶したら最大制動（`uart_protocol.md` §5.6、実機と同じ）
COMMAND_TIMEOUT_NS = 100 * 1_000_000

#: `auto_stop`（v0.7）が効き始める距離 [m]
AUTO_STOP_DISTANCE_M = 0.20

#: `LIMITS`(0x0A、★v0.11) が返す値。**実機 STM32 の固定定数
#: （`DRIVE_MAX_SPEED_M_S`/`DRIVE_MAX_ACCEL_M_S2`/`DRIVE_MAX_TORQUE_NM`）と
#: 一致させること。** `docs/uart_protocol.md` §5.12 に記載の値が正
DRIVE_MAX_SPEED_M_S = 5.0
DRIVE_MAX_ACCEL_M_S2 = 3.0
DRIVE_MAX_TORQUE_NM = 0.15   # VehicleModel.MAX_BRAKE_TORQUE_NM と同値

#: 積分の刻み。細かくしても運動学モデルでは意味が薄く、粗いと舵の遅れが崩れる
STEP_S = 0.001

_CMD_SCALE = {k: v[0] for k, v in packets.Command.META.items()}
_TLM_SCALE = {k: v[0] for k, v in packets.Telemetry.META.items()}


def _q(name: str, si: float, lo: int, hi: int) -> int:
    """SI → `TELEMETRY` の整数生値。**必ず飽和させる**（struct.pack が例外を投げる）。"""
    return max(lo, min(hi, int(round(si / _TLM_SCALE[name]))))


class VirtualStm32:
    """:param course: レイキャストと衝突判定に使う。走行中に差し替えられる
    :param lidar: `sim.lidar.VirtualLidar`。None なら LIDAR_SECTOR を出さない
    """

    def __init__(self, spec: VehicleSpec, course, params: SimParams,
                 lidar=None, seed: int = 0) -> None:
        self.spec = spec
        self.course = course
        self.params = params
        self.lidar = lidar
        self.rng = random.Random(seed)
        self._np_rng = np.random.default_rng(seed)

        self.vehicle = VehicleModel(spec, course.start)
        self._body = course.body_samples(spec.footprint)

        self._parser = FrameParser(expect_types=P2S_TYPES)
        self._enc = FrameEncoder()
        self._tx: list[tuple[int, bytes]] = []
        self._wire_free_ns = 0

        self._t0 = 0
        self._now = 0
        self._acc = 0.0
        self._clock_offset_us = self.rng.randrange(0, 1_000_000)

        self._cmd_seq_echo = 0
        self._cmd_ns = -COMMAND_TIMEOUT_NS
        self._cmd_flags = 0
        self._cmd_flags2 = 0
        self._steer_cmd_echo = 0.0
        self._input = DriveInput()

        self.estop_active = False
        self.auto_stop_active = False
        self.side_brake_active = False
        self.uart_timeout = True
        self.mode = packets.Mode.DISARM

        self.us_front = 0.0
        self.us_rear = 0.0
        self._last_collision_check = (0.0, 0.0)

        self._next_tlm = 0
        self._next_stats = 0
        self._boot_versions = 3
        self._boot_limits = 3
        # AUTO_STOP_MARGIN_CM は STM32 実機の既定が15cm。0.0 も範囲内の値として
        # 有効なため他の bool 系パラメータと違って見た目上は不正にならないが、
        # 未初期化のまま「マージン0cm」で動くとsimと実機の既定が食い違うので揃えておく（★v0.12）
        self._config: dict[int, float] = {
            packets.Param.AUTO_STOP_MARGIN_CM: 15.0,
        }
        self._stats = packets.Stats()

    # ── 時計 ──

    def start(self, t_ns: int) -> None:
        self._t0 = self._now = t_ns
        self._wire_free_ns = t_ns
        self._next_tlm = t_ns
        self._next_stats = t_ns

    def stm_us(self, t_ns: int | None = None) -> int:
        """仮想 STM32 の µs 時計。Pi の時計とはオフセットもレートも違う。"""
        t = self._now if t_ns is None else t_ns
        drift = 1.0 + self.params.stm_clock_ppm * 1e-6
        return int(((t - self._t0) * drift) / 1000 + self._clock_offset_us) & 0xFFFFFFFF

    # ── Pi からの受信 ──

    def rx_bytes(self, data: bytes, t_ns: int) -> None:
        for f in self._parser.feed(data):
            self._stats.rx_frame_ok += 1
            self._handle(f, t_ns)
        self._stats.rx_crc_error = self._parser.stats.crc_error
        self._stats.rx_len_error = self._parser.stats.len_error
        self._stats.rx_unknown_type = self._parser.stats.unknown_type

    def _handle(self, frame, t_ns: int) -> None:
        msg = frame.decode()
        if isinstance(msg, packets.Command):
            self._on_command(msg, frame.seq, t_ns)
        elif isinstance(msg, packets.Ping):
            # T2 = 受信時刻、T3 = 送信時刻。どちらも仮想 STM32 の時計で
            self._emit(packets.Pong(ping_id=msg.ping_id,
                                    t_ping_rx_us=self.stm_us(t_ns),
                                    t_pong_tx_us=self.stm_us(t_ns)), t_ns)
        elif isinstance(msg, packets.VersionReq):
            self._emit_version(t_ns)
        elif isinstance(msg, packets.LimitsReq):
            self._emit_limits(t_ns)
        elif isinstance(msg, packets.ConfigSet):
            self._config[msg.param_id] = msg.value
            known = msg.param_id in vars(packets.Param).values()
            self._emit(packets.ConfigAck(
                param_id=msg.param_id, applied=msg.value,
                result=packets.ConfigResult.OK if known else packets.ConfigResult.UNKNOWN_ID), t_ns)
        elif isinstance(msg, packets.ConfigGet):
            self._emit(packets.ConfigAck(
                param_id=msg.param_id, applied=self._config.get(msg.param_id, 0.0),
                result=packets.ConfigResult.OK), t_ns)

    def _on_command(self, c: packets.Command, seq: int, t_ns: int) -> None:
        self._cmd_ns = t_ns
        self._cmd_seq_echo = seq
        self._cmd_flags = c.flags
        self._cmd_flags2 = c.flags2
        self.mode = c.mode if c.mode in (0, 1, 2) else packets.Mode.DISARM

        armed = bool(c.flags & packets.CMD_FLG_ARM) and self.mode != packets.Mode.DISARM
        if self.estop_active:
            armed = False       # E-Stop 中は COMMAND が一切効かない（実機と同じ）

        self._steer_cmd_echo = c.target_steer * _CMD_SCALE["target_steer"]
        self._input = DriveInput(
            armed=armed,
            brake=bool(c.flags & packets.CMD_FLG_BRAKE),
            torque_mode=bool(c.flags & packets.CMD_FLG_TORQUE_MODE),
            target_speed=c.target_speed * _CMD_SCALE["target_speed"],
            target_steer=self._steer_cmd_echo,
            accel_limit=c.accel_limit * _CMD_SCALE["accel_limit"],
            steer_rate_limit=c.steer_rate_limit * _CMD_SCALE["steer_rate_limit"],
            brake_torque=c.brake_torque * _CMD_SCALE["brake_torque"],
            target_torque=c.target_torque * _CMD_SCALE["target_torque"],
        )

    # ── 進行 ──

    def advance_to(self, t_ns: int) -> None:
        """固定ステップで積分し、送るべきパケットをキューに積む。"""
        if self._t0 == 0:
            self.start(t_ns)
        if t_ns <= self._now:
            return
        # 一度に進めすぎない（デバッガで止めた後などに何万ステップも回さない）
        dt_total = min((t_ns - self._now) / NS, 0.25)
        self._now = t_ns

        self._acc += dt_total
        while self._acc >= STEP_S:
            self._acc -= STEP_S
            self._substep(STEP_S)

        if self._boot_versions > 0:
            self._boot_versions -= 1
            self._emit_version(t_ns)
        if self._boot_limits > 0:
            self._boot_limits -= 1
            self._emit_limits(t_ns)

        while t_ns >= self._next_tlm:
            self._emit(self._telemetry(self._next_tlm), self._next_tlm)
            self._next_tlm += NS // TELEMETRY_HZ
        while t_ns >= self._next_stats:
            self._stats.t_us = self.stm_us(self._next_stats)
            self._emit(self._stats, self._next_stats)
            self._next_stats += NS // STATS_HZ

        if self.lidar is not None:
            for gen_ns, pkt in self.lidar.poll(t_ns, self.vehicle, self.stm_us):
                self._emit(pkt, gen_ns)

    def _substep(self, dt: float) -> None:
        v = self.vehicle
        cmd = self._input

        # COMMAND 途絶 → 最大制動（舵角は最後の値を保持する）
        self.uart_timeout = (self._now - self._cmd_ns) > COMMAND_TIMEOUT_NS
        if self.uart_timeout or self.estop_active:
            cmd = DriveInput(armed=False, target_steer=v.steer_actual)

        # v0.13: サイドブレーキ。**個別車輪の位置制御モデルを持たないため、
        # 最大制動として近似する**（実車は速度に関わらず後輪を機械的に位置固定するが、
        # 運動学モデルでは表現できない。実機での動作検証も未了 — `pi_uart_protocol_v0.13_delta.md`）
        self.side_brake_active = bool(self._cmd_flags2 & packets.CMD_FLG2_SIDE_BRAKE) and cmd.armed
        if self.side_brake_active:
            cmd = DriveInput(armed=cmd.armed, brake=True, brake_torque=0.0,
                             target_steer=cmd.target_steer,
                             steer_rate_limit=cmd.steer_rate_limit)

        # 超音波（前後 1 本ずつのレイキャスト）と auto_stop
        self._update_ultrasonic()
        self.auto_stop_active = self._auto_stop_should_brake(cmd)
        if self.auto_stop_active:
            cmd = DriveInput(armed=cmd.armed, brake=True, brake_torque=0.0,
                             target_steer=cmd.target_steer,
                             steer_rate_limit=cmd.steer_rate_limit)

        v.apply(cmd)
        v.step(dt)

        # 衝突判定は 1cm 動くごと。毎ステップ撃つ必要はない
        px, py = self._last_collision_check
        if (v.x - px) ** 2 + (v.y - py) ** 2 >= 1e-4:
            self._last_collision_check = (v.x, v.y)
            v.note_collision(self.course.collides(v.x, v.y, v.yaw, self._body))
        elif v.collided:
            v.note_collision(self.course.collides(v.x, v.y, v.yaw, self._body))

    def _update_ultrasonic(self) -> None:
        v = self.vehicle
        p = self.params
        for name, attr in (("us_front", "us_front"), ("us_rear", "us_rear")):
            sx, sy, syaw = self.spec.sensor_pose(name)
            c, s = math.cos(v.yaw), math.sin(v.yaw)
            wx, wy = v.x + sx * c - sy * s, v.y + sx * s + sy * c
            d = self.course.ray(wx, wy, v.yaw + syaw, p.us_max_range_m)
            if d > 0 and p.us_noise_sigma_m > 0:
                d = max(0.0, d + self.rng.gauss(0.0, p.us_noise_sigma_m))
            setattr(self, attr, d)

    def _auto_stop_should_brake(self, cmd: DriveInput) -> bool:
        """v0.7。**進行方向**の超音波だけを見る。ラッチせず、ヒステリシスも無い。"""
        if not (self._cmd_flags & packets.CMD_FLG_AUTO_STOP) or not cmd.armed:
            return False
        want = cmd.target_torque if cmd.torque_mode else cmd.target_speed
        if want > 0:
            d = self.us_front
        elif want < 0:
            d = self.us_rear
        else:
            return False
        return 0.0 < d < AUTO_STOP_DISTANCE_M

    # ── テレメトリ ──

    def _flags(self, t_ns: int) -> int:
        f = self.mode & packets.FLG_MODE_MASK
        if self._input.armed and not self.estop_active and not self.uart_timeout:
            f |= packets.FLG_ARMED
        if self.estop_active:
            f |= packets.FLG_ESTOP_ACTIVE
        if self.uart_timeout:
            f |= packets.FLG_UART_TIMEOUT
        if self.auto_stop_active:
            f |= packets.FLG_AUTO_STOP_ACTIVE
        if self.side_brake_active:
            f |= packets.FLG_SIDE_BRAKE_ACTIVE
        # v0.14: ウィンカー。点滅周期は暫定 1Hz（0.5s ON / 0.5s OFF）。
        # **実際の周期は STM32 側の実装依存**なので、ここは GUI 配線の確認用の近似
        blink = (t_ns // (NS // 2)) % 2 == 0
        if blink and (self._cmd_flags2 & packets.CMD_FLG2_WINKER_LEFT):
            f |= packets.FLG_WINKER_LEFT_ACTIVE
        if blink and (self._cmd_flags2 & packets.CMD_FLG2_WINKER_RIGHT):
            f |= packets.FLG_WINKER_RIGHT_ACTIVE
        f |= packets.FLG_IMU_OK | packets.FLG_STEER_CENTER_VALID
        if self.lidar is not None:
            f |= packets.FLG_LIDAR_OK
        return f

    def _telemetry(self, t_ns: int) -> packets.Telemetry:
        v = self.vehicle
        p = self.params
        # ★ オドメトリと IMU の誤差。**ここを 0 にすると推測航法が完璧になり、
        #   SLAM を入れる意味がシミュレータ上で消える**（`sim/params.py` 参照）。
        #   距離は倍率誤差（タイヤの摩耗・スリップぶん）＋ノイズ、ヨーレートは bias（方位ドリフトの正体）
        odom = [d * (1.0 + p.odom_scale_err)
                + (self.rng.gauss(0.0, p.odom_noise_m) if p.odom_noise_m > 0 else 0.0)
                for d in v.odom_front]
        yaw_rate = v.yaw_rate + math.radians(
            p.gyro_bias_dps
            + (self.rng.gauss(0.0, p.gyro_noise_dps) if p.gyro_noise_dps > 0 else 0.0))
        load = abs(v.speed) / 3.0                       # 0..1 の目安
        torque = v.cmd.target_torque if v.cmd.torque_mode else load * 0.05
        cur = torque / max(1e-3, self.spec.wheel_radius) * 0.5
        vd = 11.2 - 1.6 * load - 0.05 * self.rng.random()
        vs = 8.2 - 0.1 * self.rng.random()
        temp = 25 + int(20 * load)

        return packets.Telemetry(
            t_us=self.stm_us(t_ns),
            flags=self._flags(t_ns),
            speed=_q("speed", v.speed, -32768, 32767),
            yaw_rate=_q("yaw_rate", yaw_rate, -32768, 32767),
            steer_actual=_q("steer_actual", v.steer_actual, -32768, 32767),
            steer_cmd_echo=_q("steer_cmd_echo", self._steer_cmd_echo, -32768, 32767),
            wheel_speed=[_q("wheel_speed", w, -32768, 32767) for w in v.wheel_speed],
            odom_dist=[_q("odom_dist", d, -(1 << 31), (1 << 31) - 1) for d in odom],
            accel_x=_q("accel_x", v.accel_x, -32768, 32767),
            accel_y=_q("accel_y", v.accel_lateral, -32768, 32767),
            accel_z=_q("accel_z", 9.81, -32768, 32767),
            pitch=0, roll=0,
            motor_current=[_q("motor_current", cur, -32768, 32767),
                           _q("motor_current", cur, -32768, 32767),
                           _q("motor_current", abs(v.steer_actual) * 0.4, -32768, 32767)],
            torque_cmd=[_q("torque_cmd", torque, -32768, 32767)] * 2,
            # TC/TV 未実装（タイヤモデルが無い）なので、スリップは常に0・上限は常に全開で返す
            slip=[0, 0],
            tc_limit_nm=[_q("tc_limit_nm", DRIVE_MAX_TORQUE_NM, -32768, 32767)] * 2,
            temp=[temp, temp, 25 + int(10 * abs(v.steer_actual)), 40],
            batt_voltage_drive=_q("batt_voltage_drive", vd, 0, 255),
            batt_voltage_signal=_q("batt_voltage_signal", vs, 0, 255),
            batt_current_drive=_q("batt_current_drive", abs(cur) * 2, 0, 255),
            batt_current_signal=_q("batt_current_signal", 0.6, 0, 255),
            us_front=_q("us_front", min(self.us_front, 5.1), 0, 255),
            us_rear=_q("us_rear", min(self.us_rear, 5.1), 0, 255),
            md_status=[packets.MDS_RUNNING | packets.MDS_COMM_OK | packets.MDS_LIMIT_SYNCED] * 3,
            cmd_seq_echo=self._cmd_seq_echo,
        )

    def _emit_version(self, t_ns: int) -> None:
        self._emit(packets.Version(protocol_version=PROTOCOL_VERSION,
                                   fw_id=FW_ID, build_epoch=0), t_ns)

    def _emit_limits(self, t_ns: int) -> None:
        """★v0.11。`VERSION` と同じタイミングで自動送信 + `LIMITS_REQ` への応答。"""
        self._emit(packets.Limits(
            max_speed_m_s=DRIVE_MAX_SPEED_M_S,
            max_accel_m_s2=DRIVE_MAX_ACCEL_M_S2,
            max_torque_nm=DRIVE_MAX_TORQUE_NM,
            max_steer_rad=self.spec.max_steer), t_ns)

    # ── 送信キュー ──

    def _emit(self, packet, gen_ns: int) -> None:
        """パケットを線上に載せる。**UART の直列化時間ぶん詰まる。**

        250000bps では TELEMETRY(81B) が 3.2ms、LIDAR_SECTOR(76B) が 3.0ms かかる。
        合計 12.8kB/s ＝ 帯域の 51% を使うので、実機では実際に順番待ちが起きる。
        ここを無視すると「全部が同時刻に届く」不自然に綺麗なシムになる。
        """
        frame = self._enc.encode(packet)
        if self.params.uart_bit_error_rate > 0:
            frame = self._corrupt(frame)
        start = max(self._wire_free_ns, gen_ns)
        end = start + len(frame) * BYTE_NS
        self._wire_free_ns = end
        self._tx.append((end, frame))

    def _corrupt(self, frame: bytes) -> bytes:
        p = self.params.uart_bit_error_rate
        if self.rng.random() >= p * len(frame) * 8:
            return frame
        buf = bytearray(frame)
        i = self.rng.randrange(len(buf))
        buf[i] ^= 1 << self.rng.randrange(8)
        return bytes(buf)

    def due_bytes(self, t_ns: int) -> bytes:
        """到着時刻を過ぎたフレームをまとめて返す。"""
        if not self._tx:
            return b""
        out = bytearray()
        keep = []
        for arrive, frame in self._tx:
            if arrive <= t_ns:
                out += frame
            else:
                keep.append((arrive, frame))
        self._tx = keep
        return bytes(out)

    def next_due_ns(self) -> int | None:
        """次にフレームが到着する時刻。`SimLink.poll()` が「何秒寝てよいか」に使う。"""
        return min((a for a, _ in self._tx), default=None)

    def flush_tx(self) -> None:
        """Pi 側の `reset_input_buffer()` に対応。線上のものを捨てる。"""
        self._tx.clear()

    # ── sim.gui からの操作 ──

    def set_course(self, course) -> None:
        self.course = course
        self._body = course.body_samples(self.spec.footprint)
        self.vehicle.reset(course.start)
        if self.lidar is not None:
            self.lidar.course = course

    def reset_pose(self) -> None:
        self.vehicle.reset(self.course.start)

    def set_estop(self, active: bool) -> None:
        self.estop_active = active
