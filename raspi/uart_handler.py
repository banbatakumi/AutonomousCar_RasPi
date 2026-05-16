import struct
import threading
import time
import logging

logger = logging.getLogger(__name__)

HEADER = 0xFF
FOOTER = 0xAA
SEND_SIZE = 6
RECV_SIZE = 11
SEND_INTERVAL = 0.02  # 50Hz


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

        move_speed   = max(-128, min(127, int(cmd["move_speed"]   / 0.1)))
        acceleration = max(-128, min(127, int(cmd["acceleration"] / 0.1)))
        steer        = max(-127, min(127, int(cmd["steer"] * 127.0)))

        return struct.pack("BBbbbB", HEADER, flags, move_speed, acceleration, steer, FOOTER)

    def _send_loop(self):
        while self._running:
            try:
                self._serial.write(self._build_packet())
            except Exception as e:
                logger.error("UART send error: %s", e)
            time.sleep(SEND_INTERVAL)

    def _recv_loop(self):
        buf = bytearray()
        while self._running:
            try:
                chunk = self._serial.read(RECV_SIZE)
                if not chunk:
                    continue
                buf.extend(chunk)
                while len(buf) >= RECV_SIZE:
                    idx = buf.find(HEADER)
                    if idx == -1:
                        buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < RECV_SIZE:
                        break
                    if buf[RECV_SIZE - 1] == FOOTER:
                        self._parse_packet(bytes(buf[:RECV_SIZE]))
                        del buf[:RECV_SIZE]
                    else:
                        del buf[:1]
            except Exception as e:
                logger.error("UART recv error: %s", e)

    def _parse_packet(self, pkt: bytes):
        # [0xFF, speed, accel, dist_f, dist_l, dist_r, dist_b, volt_s, volt_p, motor_err, 0xAA]
        speed = struct.unpack("b", bytes([pkt[1]]))[0] * 0.1
        accel = struct.unpack("b", bytes([pkt[2]]))[0] * 0.1
        with self._lock:
            self._sensor_data = {
                "speed":        round(speed, 2),
                "acceleration": round(accel, 2),
                "dist_front":   pkt[3],
                "dist_left":    pkt[4],
                "dist_right":   pkt[5],
                "dist_back":    pkt[6],
                "volt_signal":  round(pkt[7] * 0.1, 1),
                "volt_power":   round(pkt[8] * 0.1, 1),
                "motor_error":  bool(pkt[9]),
            }
