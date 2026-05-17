import struct
import threading
import time
import logging

logger = logging.getLogger(__name__)

HEADER        = 0xFF
FOOTER        = 0xAA
SEND_SIZE     = 6
RECV_SIZE     = 733  # [0xFF][motor_err][speed][accel][volt_s][volt_p][accel_xy]
                     # [pitch][roll][tmp_l][tmp_r][tmp_s][dis×360×uint16 big-endian mm][0xAA]
SEND_INTERVAL = 0.05  # 20 Hz

# EMA smoothing factors (0=frozen, 1=no filter)
_ALPHA_SPEED = 0.35
_ALPHA_DIST  = 0.50
_ALPHA_VOLT  = 0.15
_ALPHA_IMU_A = 0.50
_ALPHA_IMU_G = 0.30
_ALPHA_TEMP  = 0.10


class UARTHandler:
    def __init__(self, port="/dev/serial0", baudrate=1000000):
        self._port     = port
        self._baudrate = baudrate
        self._serial   = None
        self._lock     = threading.Lock()
        self._running  = False

        self._command = {
            "do_stop":           True,
            "do_brake":          False,
            "on_headlight":      False,
            "on_hazard":         False,
            "play_sound":        False,
            "enable_auto_brake": False,
            "mode":              0,
            "move_speed":        0.0,
            "acceleration":      0.0,
            "steer":             0.0,
        }

        self._sensor_data = {
            "speed":        0.0,
            "acceleration": 0.0,
            "dists":        [0] * 360,  # mm; 360 sectors×1°, 0=範囲外, 最大12000mm
            "volt_signal":  0.0,
            "volt_power":   0.0,
            "motor_error":  False,
            "accel_x":      0.0,
            "accel_y":      0.0,
            "pitch":        0.0,
            "roll":         0.0,
            "temp_left":    0,
            "temp_right":   0,
            "temp_steer":   0,
        }

    def start(self):
        import serial
        self._serial  = serial.Serial(self._port, self._baudrate, timeout=0.1)
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
        buf          = bytearray()
        _diag_bytes  = 0
        _diag_frames = 0
        _diag_t      = time.time()

        while self._running:
            try:
                waiting = self._serial.in_waiting
                chunk   = self._serial.read(waiting if waiting > 0 else 1)
                if chunk:
                    _diag_bytes += len(chunk)
                    buf.extend(chunk)

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

                now = time.time()
                if now - _diag_t >= 10.0:
                    logger.info("UART recv: %d bytes, %d frames / 10s",
                                _diag_bytes, _diag_frames)
                    _diag_bytes  = 0
                    _diag_frames = 0
                    _diag_t      = now

            except Exception as e:
                if self._running:
                    logger.error("UART recv error: %s", e)

    def _parse_packet(self, pkt: bytes):
        # [0xFF][motor_err][speed][accel][volt_s][volt_p][accel_xy][pitch][roll]
        # [tmp_l][tmp_r][tmp_s][dis×360bytes][0xAA]
        motor_err = bool(pkt[1])
        speed  = struct.unpack("b", bytes([pkt[2]]))[0] * 0.1
        accel  = struct.unpack("b", bytes([pkt[3]]))[0] * 0.1
        volt_s = pkt[4] * 0.1
        volt_p = pkt[5] * 0.1

        def s4(v): return v - 16 if v >= 8 else v  # 4-bit two's complement
        ax    = s4((pkt[6] >> 4) & 0xF) * 0.5
        ay    = s4(pkt[6] & 0xF) * 0.5
        pitch = struct.unpack("b", bytes([pkt[7]]))[0] * 1.0
        roll  = struct.unpack("b", bytes([pkt[8]]))[0] * 1.0
        temp_l = pkt[9]
        temp_r = pkt[10]
        temp_s = pkt[11]

        # LiDAR: bytes [12..731] = 720 bytes = 360 × uint16 big-endian, mm, 0=範囲外
        dists = [((pkt[12 + i*2] << 8) | pkt[12 + i*2 + 1]) for i in range(360)]  # mm

        with self._lock:
            p = self._sensor_data

            def ema(new, old, a):
                return new * a + old * (1 - a)

            self._sensor_data = {
                "speed":        round(ema(speed,  p["speed"],        _ALPHA_SPEED), 2),
                "acceleration": round(ema(accel,  p["acceleration"], _ALPHA_SPEED), 2),
                "dists":        [round(ema(d, p["dists"][i], _ALPHA_DIST)) if d > 0 else 0
                                 for i, d in enumerate(dists)],  # 360 sectors
                "volt_signal":  round(ema(volt_s, p["volt_signal"],  _ALPHA_VOLT), 1),
                "volt_power":   round(ema(volt_p, p["volt_power"],   _ALPHA_VOLT), 1),
                "motor_error":  motor_err,
                "accel_x":      round(ema(ax,    p["accel_x"], _ALPHA_IMU_A), 2),
                "accel_y":      round(ema(ay,    p["accel_y"], _ALPHA_IMU_A), 2),
                "pitch":        round(ema(pitch, p["pitch"],   _ALPHA_IMU_G), 1),
                "roll":         round(ema(roll,  p["roll"],    _ALPHA_IMU_G), 1),
                "temp_left":    round(ema(temp_l, p["temp_left"],  _ALPHA_TEMP)),
                "temp_right":   round(ema(temp_r, p["temp_right"], _ALPHA_TEMP)),
                "temp_steer":   round(ema(temp_s, p["temp_steer"], _ALPHA_TEMP)),
            }
