import struct
import threading
import time
import logging

logger = logging.getLogger(__name__)

HEADER = 0xFF
FOOTER = 0xAA
SEND_SIZE = 6
RECV_SIZE = 28
SEND_INTERVAL = 0.05  # 20Hz

# EMA smoothing factors (0=frozen, 1=no filter)
_ALPHA_SPEED = 0.35   # speed / accel: moderate
_ALPHA_DIST  = 0.50   # distance sensors: responsive but noise-reduced
_ALPHA_VOLT  = 0.15   # voltages: slow-changing → heavy smoothing
_ALPHA_IMU_A = 0.50   # IMU accelerometer: responsive
_ALPHA_IMU_G = 0.30   # IMU pitch/roll: smoothed for display


class UARTHandler:
    def __init__(self, port="/dev/serial0", baudrate=230400):
        self._port = port
        self._baudrate = baudrate
        self._serial = None
        self._lock = threading.Lock()
        self._running = False

        self._command = {
            "do_stop": True,
            "do_brake": False,
            "on_headlight": False,
            "on_hazard": False,
            "play_sound": False,
            "enable_auto_brake": False,
            "mode": 0,
            "move_speed": 0.0,
            "acceleration": 0.0,
            "steer": 0.0,
        }

        self._sensor_data = {
            "speed": 0.0,
            "acceleration": 0.0,
            "dists": [0] * 36,  # cm, 0=out of range; sectors: 0°,10°,...,350°
            "volt_signal": 0.0,
            "volt_power": 0.0,
            "motor_error": False,
            "accel_x": 0.0,  # longitudinal G (forward positive), 4-bit signed × 0.1
            "accel_y": 0.0,  # lateral G (right positive), 4-bit signed × 0.1
            "pitch": 0.0,    # degrees, nose-up positive
            "roll": 0.0,     # degrees, right-bank positive
        }

    def start(self):
        import serial
        self._serial = serial.Serial(self._port, self._baudrate, timeout=0.1)
        self._running = True
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def stop(self):
        self._running = False
        if self._serial:
            self._serial.close()

    def set_command(self, cmd: dict):
        with self._lock:
            self._command.update(cmd)

    def get_sensor_data(self) -> dict:
        with self._lock:
            return dict(self._sensor_data)

    def _build_packet(self) -> bytes:
        with self._lock:
            cmd = dict(self._command)

        flags = 0
        if cmd["do_stop"]:           flags |= 0x01
        if cmd["do_brake"]:          flags |= 0x02
        if cmd["on_headlight"]:      flags |= 0x04
        if cmd["on_hazard"]:         flags |= 0x08
        if cmd["play_sound"]:        flags |= 0x10
        if cmd["enable_auto_brake"]: flags |= 0x20
        flags |= (int(cmd["mode"]) & 0x03) << 6

        move_speed   = max(-128, min(127, round(cmd["move_speed"]   / 0.1)))
        acceleration = max(-128, min(127, round(cmd["acceleration"] / 0.1)))
        steer        = max(-127, min(127, round(cmd["steer"] * 127.0)))

        return struct.pack("BBbbbB", HEADER, flags, move_speed, acceleration, steer, FOOTER)

    def _send_loop(self):
        while self._running:
            try:
                self._serial.write(self._build_packet())
                self._serial.flush()
            except Exception as e:
                logger.error("UART send error: %s", e)
            time.sleep(SEND_INTERVAL)

    def _recv_loop(self):
        buf = bytearray()
        _diag_bytes = 0
        _diag_frames = 0
        _diag_t = time.time()

        while self._running:
            try:
                waiting = self._serial.in_waiting
                chunk = self._serial.read(waiting if waiting > 0 else 1)
                if not chunk:
                    pass
                else:
                    _diag_bytes += len(chunk)
                    buf.extend(chunk)

                    # ヘッダ(0xFF)とフッタ(0xAA)の両方が正しい位置にある最後のフレームを使用
                    last_pos = -1
                    i = 0
                    while i <= len(buf) - RECV_SIZE:
                        if buf[i] == HEADER and buf[i + RECV_SIZE - 1] == FOOTER:
                            last_pos = i
                        i += 1

                    if last_pos >= 0:
                        self._parse_packet(bytes(buf[last_pos:last_pos + RECV_SIZE]))
                        del buf[:last_pos + RECV_SIZE]
                        _diag_frames += 1
                    elif len(buf) > RECV_SIZE * 4:
                        del buf[:len(buf) - RECV_SIZE + 1]

                # 10秒ごとに受信統計をログ出力
                now = time.time()
                if now - _diag_t >= 10.0:
                    logger.info("UART recv: %d bytes, %d frames / 10s", _diag_bytes, _diag_frames)
                    _diag_bytes = 0
                    _diag_frames = 0
                    _diag_t = now

            except Exception as e:
                if self._running:
                    logger.error("UART recv error: %s", e)

    def _parse_packet(self, pkt: bytes):
        # [0xFF, speed, accel, b3..b20(36 nibbles), volt_s, volt_p, motor_err,
        #  accel_xy, pitch, roll, 0xAA]
        speed  = struct.unpack("b", bytes([pkt[1]]))[0] * 0.1
        accel  = struct.unpack("b", bytes([pkt[2]]))[0] * 0.1
        volt_s = pkt[21] * 0.1
        volt_p = pkt[22] * 0.1

        raw = []
        for b in pkt[3:21]:
            raw.append((b >> 4) & 0xF)
            raw.append(b & 0xF)
        dists = [v * 10 for v in raw]  # cm; 0=out of range, else 10-150

        # IMU: accel nibbles (int8_t signed 4-bit, × 0.1g)
        def s4(v): return v - 16 if v >= 8 else v  # 4-bit two's complement
        ax = s4((pkt[24] >> 4) & 0xF) * 0.1
        ay = s4(pkt[24] & 0xF) * 0.1
        pitch = struct.unpack("b", bytes([pkt[25]]))[0] * 1.0  # degrees
        roll  = struct.unpack("b", bytes([pkt[26]]))[0] * 1.0  # degrees

        with self._lock:
            p = self._sensor_data

            def ema(new, old, a):
                return new * a + old * (1 - a)

            self._sensor_data = {
                "speed":        round(ema(speed,  p["speed"],        _ALPHA_SPEED), 2),
                "acceleration": round(ema(accel,  p["acceleration"], _ALPHA_SPEED), 2),
                "dists":        [round(ema(d, p["dists"][i], _ALPHA_DIST)) if d > 0 else 0
                                 for i, d in enumerate(dists)],
                "volt_signal":  round(ema(volt_s, p["volt_signal"],  _ALPHA_VOLT), 1),
                "volt_power":   round(ema(volt_p, p["volt_power"],   _ALPHA_VOLT), 1),
                "motor_error":  bool(pkt[23]),
                "accel_x":      round(ema(ax,    p["accel_x"], _ALPHA_IMU_A), 2),
                "accel_y":      round(ema(ay,    p["accel_y"], _ALPHA_IMU_A), 2),
                "pitch":        round(ema(pitch, p["pitch"],   _ALPHA_IMU_G), 1),
                "roll":         round(ema(roll,  p["roll"],    _ALPHA_IMU_G), 1),
            }
