"""カメラの実力を測る。**「映った」で終わらせないための道具。**

    .venv/bin/python -m raspi.tools.camera_probe                 # カメラ0を既定設定で
    .venv/bin/python -m raspi.tools.camera_probe --camera 1
    .venv/bin/python -m raspi.tools.camera_probe --both          # 2台同時（共存できるか）
    .venv/bin/python -m raspi.tools.camera_probe --size 1280x720 --fps 30

測る対象と、なぜそれを測るか:

1. **`SensorTimestamp` がどの時計基準か** — これが最重要。
   アーキテクチャは全メッセージが `t_capture`（Pi の `CLOCK_MONOTONIC`）を持つ前提
   （`docs/architecture.md` §6.3）。カメラの時刻が別基準だと、
   LiDAR やオドメトリと融合するときに全部を書き直すことになる。
   STM32 に時刻同期を作ったのと同じ問題がカメラにもある。
2. **取得レイテンシ** — 撮像から Python が配列に触れるまで。制御に使うなら効く
3. **実効 fps とその揺らぎ** — 公称値ではなく実測
4. **CPU 負荷** — io_node と同居できるかの判断材料。Pi 5 は4コア
5. **フレーム落ち** — libcamera がフレームを捨てていないか

**この道具は測るだけで、何も保存しない。** 記録は camera_node の仕事。
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.core.cleanup import quiet_close  # noqa: E402

NS = 1_000_000_000


def _clock_bases() -> dict[str, int]:
    """同じ瞬間の各種時計を読む。`SensorTimestamp` の基準を当てるのに使う。"""
    return {
        "CLOCK_MONOTONIC": time.clock_gettime_ns(time.CLOCK_MONOTONIC),
        "CLOCK_BOOTTIME": time.clock_gettime_ns(time.CLOCK_BOOTTIME),
        "CLOCK_REALTIME": time.clock_gettime_ns(time.CLOCK_REALTIME),
    }


def identify_clock(sensor_ts_ns: int, bases: dict[str, int]) -> tuple[str, int]:
    """`SensorTimestamp` に一番近い時計を選ぶ。

    撮像は「ついさっき」なので、正しい基準との差は
    パイプライン遅延ぶん（数ミリ〜数十ミリ秒）の小さな正の値になるはず。
    基準が違えば差は桁違いになる（REALTIME と MONOTONIC は起動時刻ぶんずれる）。
    """
    diffs = {name: t - sensor_ts_ns for name, t in bases.items()}
    best = min(diffs, key=lambda k: abs(diffs[k]))
    return best, diffs[best]


class Probe:
    def __init__(self, idx: int, size: tuple[int, int], fps: float | None,
                 fmt: str = "RGB888") -> None:
        from picamera2 import Picamera2

        from raspi.nodes.camera_node import full_fov_sensor_size

        self.idx = idx
        self.cam = Picamera2(idx)
        ctrl = {}
        if fps:
            # フレーム時間の上下限を同じ値にして fps を固定する
            us = int(1e6 / fps)
            ctrl["FrameDurationLimits"] = (us, us)
        # main の size に一致するセンサーネイティブモードが望遠クロップのことがある
        # （camera_node.full_fov_sensor_size 参照）。フル画角のモードを明示する
        sensor_size = full_fov_sensor_size(self.cam, size)
        cfg_kwargs = {"main": {"size": size, "format": fmt}, "buffer_count": 6, "controls": ctrl}
        if sensor_size:
            cfg_kwargs["sensor"] = {"output_size": sensor_size}
        cfg = self.cam.create_video_configuration(**cfg_kwargs)
        self.cam.configure(cfg)
        self.size = size
        self.fmt = fmt
        self.samples: list[tuple[int, int, int]] = []   # (sensor_ts, t_after, seq)

    def start(self) -> None:
        self.cam.start()

    def grab(self) -> None:
        """1フレーム取り、時刻を控える。**配列のコピーはしない**（測定を汚さないため）。"""
        req = self.cam.capture_request()
        t_after = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
        try:
            md = req.get_metadata()
            self.samples.append(
                (md.get("SensorTimestamp", 0), t_after, md.get("FrameCount", -1)))
        finally:
            req.release()

    def stop(self) -> None:
        with quiet_close("picamera2 のカメラ（camera_probe）"):
            self.cam.stop()
            self.cam.close()

    # ── 集計 ──

    def report(self) -> None:
        n = len(self.samples)
        if n < 2:
            print(f"  カメラ{self.idx}: サンプルが足りない")
            return
        ts = [s[0] for s in self.samples]
        lat = [(s[1] - s[0]) / 1e6 for s in self.samples]      # ms
        gaps = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]     # ms

        w, h = self.size
        bpp = 3 if self.fmt in ("RGB888", "BGR888") else 1.5
        fps = 1000 / statistics.median(gaps)
        mbps = w * h * bpp * fps / 1e6

        print(f"\n  ── カメラ{self.idx}  {w}x{h} {self.fmt}  {n}フレーム ──")
        print(f"    実効 fps      : {fps:.1f}  "
              f"（フレーム間隔 中央値 {statistics.median(gaps):.2f}ms / "
              f"最大 {max(gaps):.2f}ms / 最小 {min(gaps):.2f}ms）")
        print(f"    取得レイテンシ: 中央値 {statistics.median(lat):.1f}ms  "
              f"最小 {min(lat):.1f}ms  最大 {max(lat):.1f}ms")
        print(f"    帯域          : {mbps:.0f} MB/s（生 RGB 換算）")

        # フレーム落ち: FrameCount が飛んでいないか
        seqs = [s[2] for s in self.samples if s[2] >= 0]
        if len(seqs) > 1:
            dropped = (seqs[-1] - seqs[0]) - (len(seqs) - 1)
            print(f"    フレーム落ち  : {dropped}  "
                  + ("（libcamera 側で捨てられている）" if dropped else "（なし）"))


def run(indices: list[int], size, fps, frames: int) -> int:
    probes = []
    try:
        for i in indices:
            probes.append(Probe(i, size, fps))
    except Exception as e:
        print(f"カメラを開けない: {e}", file=sys.stderr)
        for p in probes:
            p.stop()
        return 2

    for p in probes:
        p.start()
    time.sleep(1.0)          # AE/AWB が落ち着くまで捨てる

    # ── 時計の基準を先に確定させる ──
    probes[0].grab()
    bases = _clock_bases()
    sensor_ts = probes[0].samples[-1][0]
    name, diff = identify_clock(sensor_ts, bases)

    print("=" * 62)
    print("1) SensorTimestamp の時計基準")
    print("=" * 62)
    print(f"  SensorTimestamp = {sensor_ts}")
    for k, v in bases.items():
        d = (v - sensor_ts) / 1e6
        mark = " ← これ" if k == name else ""
        print(f"    {k:16} との差: {d:>16,.1f} ms{mark}")
    ok_clock = name == "CLOCK_MONOTONIC" and 0 <= diff < 500 * 1_000_000
    print(f"\n  判定: " + (
        "**CLOCK_MONOTONIC 基準**。t_capture にそのまま使える" if ok_clock
        else f"**{name} 基準**。t_capture へ変換が要る（差 {diff/1e6:.1f}ms）"))
    probes[0].samples.clear()

    # ── 本計測 ──
    #
    # **カメラごとにスレッドを分ける。** 1スレッドで2台を順にブロッキング取得すると、
    # 2台目は常に1フレーム溜まった状態から取ることになり、レイテンシが
    # 1フレーム分（約33ms）水増しされる。実測で 5.6ms → 38.1ms になった。
    # これは測定の誤りであると同時に、camera_node が避けるべき実装でもある。
    cpu0 = time.process_time()
    t0 = time.monotonic()
    if len(probes) == 1:
        for _ in range(frames):
            probes[0].grab()
    else:
        def loop(p):
            for _ in range(frames):
                p.grab()
        threads = [threading.Thread(target=loop, args=(p,), daemon=True)
                   for p in probes]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
    wall = time.monotonic() - t0
    cpu = time.process_time() - cpu0

    print("\n" + "=" * 62)
    print(f"2) 取得性能（カメラ {len(probes)}台同時）")
    print("=" * 62)
    for p in probes:
        p.report()

    ncpu = os.cpu_count() or 4
    print(f"\n  この process の CPU: {cpu / wall * 100:.0f}% "
          f"(1コア基準、{ncpu}コア中) / 実時間 {wall:.1f}s")
    print("  ※ libcamera の処理は別プロセス(ISP/IPA)なので、"
          "全体負荷は top で別途見ること")

    for p in probes:
        p.stop()
    return 0 if ok_clock else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--both", action="store_true", help="2台同時に回す")
    ap.add_argument("--size", default="640x480", help="例 1280x720")
    ap.add_argument("--fps", type=float, default=None, help="固定したい fps")
    ap.add_argument("--frames", type=int, default=120)
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    indices = [0, 1] if args.both else [args.camera]
    return run(indices, (w, h), args.fps, args.frames)


if __name__ == "__main__":
    raise SystemExit(main())
