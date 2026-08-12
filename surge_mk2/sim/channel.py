"""シム ↔ シミュレータGUI の私設チャンネル（UDP ループバック）。

**ZeroMQ の内部バスを使わない。** バスに乗せると `raspi/msgs/types.py` に真値の
メッセージ型を、`raspi/bus/zbus.py` にトピックの持ち主を登録することになり、
実機側の語彙にシミュレータの概念が入り込む。シムの真値は実機には存在しない量なので、
実機コードが名前すら知らない場所に置く。

    sim  --- truth (20Hz) --->  :5599  シミュレータGUI
         <--- ctrl (event) ---  :5598

取りこぼしても再送しない。**真値は履歴に意味が無い**（次が 50ms 後に来る）。

代償: 真値が `.sfl` / mcap ログに残らない。将来「真値つきデータセット」が要るなら
シム側で独立したファイルに書くのが筋で、バスに戻すのではない。
"""

from __future__ import annotations

import math
import os
import socket
from pathlib import Path

import msgspec

from .course import Course, list_courses
from .params import TUNABLES

__all__ = ["SimChannel", "ViewerChannel", "TRUTH_PORT", "CTRL_PORT"]

#: `SURGE_BUS_DIR` と同じ流儀で差し替えられる。2つのシムを並走させたいとき
#: （検証中に、動いているセッションを止めずに確かめたいとき）に使う
TRUTH_PORT = int(os.environ.get("SURGE_SIM_TRUTH_PORT", 5599))
CTRL_PORT = int(os.environ.get("SURGE_SIM_CTRL_PORT", 5598))
HOST = "127.0.0.1"

TRUTH_HZ = 20

_enc = msgspec.msgpack.Encoder()
_dec = msgspec.msgpack.Decoder()


def _udp(bind_port: int | None) -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(False)
    if bind_port is not None:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, bind_port))
    return s


class SimChannel:
    """シム側。真値を投げ、操作を受ける。

    **相手がいなくても動く。** GUI を起動していない／落としたときに UDP の
    送信先が閉じていると ECONNREFUSED が返ることがあるが、無視して走り続ける。
    シミュレーションが GUI に依存してはいけない。
    """

    def __init__(self, course_dir: str | Path | None = None) -> None:
        self.sock = _udp(CTRL_PORT)
        self.course_dir = Path(course_dir) if course_dir else None
        self._next_ns = 0
        self.enabled = True

    def pump(self, t_ns: int, sim) -> None:
        """`SimLink.poll()` から毎回呼ばれる。20Hz に間引いて送る。"""
        if not self.enabled:
            return
        for msg in self._recv():
            self._apply(msg, sim)
        if t_ns < self._next_ns:
            return
        self._next_ns = t_ns + 1_000_000_000 // TRUTH_HZ
        self._send(self._truth(t_ns, sim))

    # ── 送信 ──

    def _truth(self, t_ns: int, sim) -> dict:
        v = sim.vehicle
        scan = sim.lidar.last_scan if sim.lidar is not None else None
        return {
            "t": t_ns,
            "x": v.x, "y": v.y, "yaw": v.yaw,
            "speed": v.speed,
            "steer_actual": v.steer_actual,
            "steer_cmd": sim._steer_cmd_echo,
            "yaw_rate": v.yaw_rate,
            "mode": sim.mode,
            "armed": v.cmd.armed,
            "estop": sim.estop_active,
            "uart_timeout": sim.uart_timeout,
            "auto_stop": sim.auto_stop_active,
            "collided": v.collided,
            "collisions": v.collisions,
            "us_front": sim.us_front,
            "us_rear": sim.us_rear,
            "course": sim.course.name,
            "course_path": str(sim.course.path),
            "courses": [str(p) for p in self._list()],
            "footprint": sim.spec.footprint,
            "lidar_mount": sim.spec.sensor_pose("lidar")[:2],
            # mm の整数にして送る。float 360 個より小さく、描画には十分
            "scan_mm": [int(d * 1000) for d in scan] if scan is not None else [],
            "params": {t.key: getattr(sim.params, t.key) for t in TUNABLES},
        }

    def _list(self) -> list[Path]:
        return list_courses(self.course_dir) if self.course_dir else list_courses()

    def _send(self, payload: dict) -> None:
        try:
            self.sock.sendto(_enc.encode(payload), (HOST, TRUTH_PORT))
        except OSError:
            pass                     # GUI が居ない。走行は続ける

    # ── 受信 ──

    def _recv(self) -> list[dict]:
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                break
            try:
                out.append(_dec.decode(data))
            except msgspec.DecodeError:
                pass
        return out

    def _apply(self, msg: dict, sim) -> None:
        kind = msg.get("type")
        if kind == "set_course":
            try:
                sim.set_course(Course.load(msg["path"]))
            except Exception as e:
                print(f"!! コースを読めない: {e}")
        elif kind == "reset":
            sim.reset_pose()
        elif kind == "estop":
            sim.set_estop(bool(msg.get("value")))
        elif kind == "set_param":
            key = msg.get("key")
            if any(t.key == key for t in TUNABLES):
                setattr(sim.params, key, float(msg["value"]))
        elif kind == "nudge":
            # GUI から車体を少し動かす（壁に埋まったときの脱出用）
            v = sim.vehicle
            v.x += float(msg.get("dx", 0.0)) * math.cos(v.yaw)
            v.y += float(msg.get("dx", 0.0)) * math.sin(v.yaw)
            v.yaw += float(msg.get("dyaw", 0.0))

    def close(self) -> None:
        self.sock.close()


class ViewerChannel:
    """シミュレータGUI 側。真値を受け、操作を投げる。"""

    def __init__(self) -> None:
        self.sock = _udp(TRUTH_PORT)

    def poll(self) -> dict | None:
        """**最新の1件だけ**返す。溜まった古い真値は捨てる（履歴に意味が無い）。"""
        latest = None
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                break
            try:
                latest = _dec.decode(data)
            except msgspec.DecodeError:
                pass
        return latest

    def send(self, **msg) -> None:
        try:
            self.sock.sendto(_enc.encode(msg), (HOST, CTRL_PORT))
        except OSError:
            pass                     # シムが居ない

    def close(self) -> None:
        self.sock.close()
