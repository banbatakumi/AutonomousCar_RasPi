import struct
import threading
import time
import logging

logger = logging.getLogger(__name__)

HEADER = 0xFF
FOOTER = 0xAA
SEND_SIZE = 6
RECV_SIZE = 11
SEND_INTERVAL = 0.05  # 20Hz

# EMA smoothing factors (0=frozen, 1=no filter)
_ALPHA_SPEED = 0.35   # speed / accel: moderate
_ALPHA_DIST  = 0.50   # distance sensors: responsive but noise-reduced
_ALPHA_VOLT  = 0.15   # voltages: slow-changing → heavy smoothing


class UARTHandler:
    def __init__(self, port="/dev/serial0", baudrate=230400):
        self._port = port
        self._baudrate = baudrate
        self._serial = None
        self._lock = threading.Lock()
        self._running = False

        self._command = {
            "do_stop": True,
            "do_remote_control": False,
            "do_brake": False,
            "on_headlight": False,
            "on_hazard": False,
            "move_speed": 0.0,
            "acceleration": 0.0,
            "steer": 0.0,
        }

        self._sensor_data = {
            "speed": 0.0,
            "acceleration": 0.0,
            "dist_front": 0,
            "dist_left": 0,
            "dist_right": 0,
            "dist_back": 0,
            "volt_signal": 0.0,
            "volt_power": 0.0,
            "motor_error": False,
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
        if cmd["do_remote_control"]: flags |= 0x02
        if cmd["do_brake"]:          flags |= 0x04
        if cmd["on_headlight"]:      flags |= 0x08
        if cmd["on_hazard"]:         flags |= 0x10

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
        # [0xFF, speed, accel, dist_f, dist_l, dist_r, dist_b, volt_s, volt_p, motor_err, 0xAA]
        speed  = struct.unpack("b", bytes([pkt[1]]))[0] * 0.1
        accel  = struct.unpack("b", bytes([pkt[2]]))[0] * 0.1
        volt_s = pkt[7] * 0.1
        volt_p = pkt[8] * 0.1

        with self._lock:
            p = self._sensor_data  # previous filtered values

            def ema(new, old, a):
                return new * a + old * (1 - a)

            self._sensor_data = {
                "speed":        round(ema(speed,  p["speed"],        _ALPHA_SPEED), 2),
                "acceleration": round(ema(accel,  p["acceleration"], _ALPHA_SPEED), 2),
                "dist_front":   round(ema(pkt[3], p["dist_front"],   _ALPHA_DIST)),
                "dist_left":    round(ema(pkt[4], p["dist_left"],    _ALPHA_DIST)),
                "dist_right":   round(ema(pkt[5], p["dist_right"],   _ALPHA_DIST)),
                "dist_back":    round(ema(pkt[6], p["dist_back"],    _ALPHA_DIST)),
                "volt_signal":  round(ema(volt_s, p["volt_signal"],  _ALPHA_VOLT), 1),
                "volt_power":   round(ema(volt_p, p["volt_power"],   _ALPHA_VOLT), 1),
                "motor_error":  bool(pkt[9]),
            }
