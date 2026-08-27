"""SimLink — `SerialLink` と同じ顔をした偽リンク。

`IoNode` は link をコンストラクタ注入で受け取る（`io_node.py:100`）ので、
これを差すだけで `io_node` より上流は 1 行も変えずに動く。

**満たすべき約束は 7 つだけ**（`raspi/io/serial_link.py` を参照）:
`poll()` / `send()` / `send_raw()` / `reset_input()` / `close()` / `stats` / `on_tx`。

## poll() が sleep するのを消さないこと

本物の `SerialLink.poll()` は `read_timeout=0.005` でブロックする。io_node の
メインループには sleep が無く、**そこで待つ前提**になっている。SimLink が即座に
返すと CPU を 100% 使う暴走ループになり、シミュレーションの時間精度も落ちる。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

from raspi.io.serial_link import RxFrame
from raspi.proto import FrameEncoder, FrameParser
from raspi.proto.generated.packets import HEADER_SIZE, S2P_TYPES

from .course import Course
from .params import SimParams
from .stm32 import BYTE_NS, VirtualStm32
from .vehicle import VehicleSpec

__all__ = ["SimLink", "create_sim_link"]


class SimLink:
    """:param sim: 裏で回る仮想 STM32
    :param read_timeout: 何も届いていないときに待つ時間 [s]。本物と同じ既定値
    """

    #: `LinkDiag.sim` に載る印。GUI が SIM バッジを出す唯一の根拠。
    #: `SerialLink` 側には何も足さない（`getattr(link, "is_sim", False)` で読む）
    is_sim = True

    def __init__(self, sim: VirtualStm32, read_timeout: float = 0.005,
                 channel=None) -> None:
        self.sim = sim
        self.channel = channel
        self._read_timeout = read_timeout
        self._parser = FrameParser(expect_types=S2P_TYPES)
        self._enc = FrameEncoder()
        self.port = f"sim:{sim.course.name}"
        self.baud = 250_000
        self.on_tx = None
        sim.start(time.monotonic_ns())

    # ── 受信 ──

    def poll(self) -> list[RxFrame]:
        """届いているフレームを返す。無ければ **次の到着時刻まで**（上限 `read_timeout`）待つ。

        本物の `read()` は「1バイト来たら起きる」ので、`read_timeout` を毎回まるごと
        寝てはいけない。固定 5ms 寝ると受信時刻 T4 に平均 2.5ms の下駄が乗り、
        時刻同期の片道遅延推定が実機より数倍大きく出る（実際に 5.8ms になった）。
        """
        data = self._advance_and_read()
        if not data:
            now = time.monotonic_ns()
            due = self.sim.next_due_ns()
            wait = self._read_timeout if due is None else (due - now) / 1e9
            time.sleep(max(0.0, min(self._read_timeout, wait)))
            data = self._advance_and_read()
            if not data:
                return []
        rx_ns = time.monotonic_ns()
        return [RxFrame(f.type, f.seq, f.payload, rx_ns)
                for f in self._parser.feed(data)]

    def _advance_and_read(self) -> bytes:
        now = time.monotonic_ns()
        self.sim.advance_to(now)
        if self.channel is not None:
            self.channel.pump(now, self.sim)
        return self.sim.due_bytes(now)

    @property
    def stats(self):
        return self._parser.stats

    # ── 送信 ──

    def send(self, packet) -> int:
        return self._write(self._enc.encode(packet))

    def send_raw(self, pkt_type: int, payload: bytes = b"") -> int:
        return self._write(self._enc.encode_raw(pkt_type, payload))

    def _write(self, frame: bytes) -> int:
        t_ns = time.monotonic_ns()
        self.sim.advance_to(t_ns)
        # Pi → STM32 も線上時間ぶん遅れて届く。COMMAND(22B) で 0.88ms。
        # ここを 0 にすると PING の往復が非現実的に速くなり、時刻同期の
        # 片道遅延推定が「ほぼ 0」に収束してしまう
        self.sim.rx_bytes(frame, t_ns + len(frame) * BYTE_NS)
        if self.on_tx is not None:
            self.on_tx(t_ns, frame[2], frame[3],
                       frame[HEADER_SIZE:HEADER_SIZE + frame[4]])
        return t_ns

    # ── 後始末 ──

    def reset_input(self) -> None:
        self.sim.flush_tx()
        self._parser.reset()

    def close(self) -> None:
        pass

    def __enter__(self) -> "SimLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def create_sim_link(course_path: str | Path, *, spec_path: str | Path | None = None,
                    params: SimParams | None = None, seed: int = 0,
                    with_lidar: bool = True, with_channel: bool = True) -> SimLink:
    """コースを読み、仮想 STM32 を組み立てて SimLink を返す。

    :param with_channel: シミュレータGUI 向けの UDP 私設チャンネルを開くか。
        GUI が起動していなくても支障はない（送信先が居なければ黙って捨てる）
    """
    spec = VehicleSpec.load(spec_path) if spec_path else VehicleSpec.load()
    course = Course.load(course_path)
    params = params or SimParams()

    lidar = None
    if with_lidar:
        try:
            from .lidar import VirtualLidar
            lidar = VirtualLidar(course, spec, params, seed=seed)
        except ImportError:
            lidar = None

    sim = VirtualStm32(spec, course, params, lidar=lidar, seed=seed)
    # **自作 PNG で一番やりがちな間違い。** 壁に埋まったままだと車が一切動かず、
    # 超音波 2cm → auto_stop が効き続けるという分かりにくい症状になる
    if course.collides(*course.start, sim._body):
        print(f"!! スタート姿勢 {course.start} が壁に埋まっている（{course.path.name}）。"
              f"\n   JSON の start を走行可能な場所に直すこと", file=sys.stderr)

    channel = None
    if with_channel:
        try:
            from .channel import SimChannel
            channel = SimChannel(course_dir=Path(course_path).parent)
        except OSError as e:
            # ポートが埋まっている＝別のシムが動いている。走行自体は続けられる
            print(f"!! シミュレータGUI 用チャンネルを開けない（GUI 無しで続行）: {e}",
                  file=sys.stderr)
    return SimLink(sim, channel=channel)
