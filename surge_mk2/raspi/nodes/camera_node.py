"""camera_node — picamera2 で撮って共有メモリリングに書く。

    .venv/bin/python -m raspi.nodes.camera_node                      # 2台・640x480
    .venv/bin/python -m raspi.nodes.camera_node --cameras 0
    .venv/bin/python -m raspi.nodes.camera_node --size 1640x1232 --duration 30

やること:

- カメラごとに `FrameRing`（共有メモリ N枚）を作り、撮ったフレームを書き込む
- **カメラごとに独立したスレッドで取得する。** 1スレッドで順に取ると、
  2台目は常に1フレーム古いものを掴む（実測 5.6ms → 38.1ms）
- `t_capture` には picamera2 の `SensorTimestamp` をそのまま入れる。
  **これは Pi の CLOCK_MONOTONIC 基準**であることを実機で確認済みなので変換不要
- フレームの説明（`FrameDesc`）をコールバックで渡す。**画素は渡さない**

下流配信（ZeroMQ）はまだ無い。バスができたら `FrameDesc` をそのまま publish する。

共有メモリの名前は `surge_cam0` / `surge_cam1`。読み手は:

    python3 -m raspi.tools.shm_view surge_cam0
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import FrameRing  # noqa: E402

SHM_PREFIX = "surge_cam"
DEFAULT_SLOTS = 8

#: **libcamera の形式名はメモリ上のバイト順ではない。**
#: 32bit ワードにパックしたときの並びを指すので、リトルエンディアンの
#: メモリ上では逆順になる。実測で確認済み: `format="RGB888"` で赤い物を撮ると
#: 配列の ch2 が最大になる（＝ ch0=B, ch1=G, ch2=R）。
#:
#: リングには**メモリ上の実際の並び**を記録する。下流が `fmt` を見て
#: そのまま解釈できることを優先する（名前を信じて色が入れ替わる事故を防ぐ）。
_MEMORY_ORDER = {
    "RGB888": "BGR888",
    "BGR888": "RGB888",
    "XRGB8888": "XBGR8888",
    "XBGR8888": "XRGB8888",
}


def memory_format(libcamera_name: str) -> str:
    """libcamera の形式名 → メモリ上のバイト順の名前。"""
    return _MEMORY_ORDER.get(libcamera_name, libcamera_name)


@dataclass(slots=True)
class CamStats:
    frames: int = 0
    dropped: int = 0            #: libcamera 側で落ちた（FrameCount の飛び）
    write_ns_max: int = 0       #: 共有メモリへの1回の書き込みにかかった最大時間
    write_ns_total: int = 0
    last_t_capture: int = 0
    gaps_ms: list[float] = field(default_factory=list)

    def summary(self, elapsed: float) -> str:
        fps = self.frames / elapsed if elapsed > 0 else 0
        avg_us = (self.write_ns_total / self.frames / 1000) if self.frames else 0
        jitter = ""
        if len(self.gaps_ms) > 2:
            g = sorted(self.gaps_ms)
            jitter = f" 間隔 中央値{g[len(g)//2]:.2f}ms 最大{g[-1]:.2f}ms"
        return (f"{self.frames}枚 {fps:.1f}fps 落ち{self.dropped} "
                f"書込 平均{avg_us:.0f}μs 最大{self.write_ns_max/1000:.0f}μs{jitter}")


class CameraWorker(threading.Thread):
    """カメラ1台ぶんの取得ループ。**1台につき1スレッド。**"""

    def __init__(self, idx: int, size: tuple[int, int], fmt: str, fps: float | None,
                 n_slots: int, on_frame=None) -> None:
        super().__init__(name=f"cam{idx}", daemon=True)
        self.idx = idx
        self.size = size
        self.fmt = fmt
        self.fps = fps
        self.on_frame = on_frame
        self.stats = CamStats()
        self.error: Exception | None = None
        self._running = False
        self._last_frame_id = -1

        from picamera2 import Picamera2

        self.cam = Picamera2(idx)
        ctrl = {}
        if fps:
            us = int(1e6 / fps)
            ctrl["FrameDurationLimits"] = (us, us)
        cfg = self.cam.create_video_configuration(
            main={"size": size, "format": fmt}, buffer_count=6, controls=ctrl)
        self.cam.configure(cfg)

        # 実際に確定した幾何をリングに使う。要求と食い違うことがあるため
        main = self.cam.camera_configuration()["main"]
        self.size = main["size"]
        self.libcamera_fmt = main["format"]
        self.fmt = memory_format(self.libcamera_fmt)      # メモリ上の並びで記録する
        self.ring = FrameRing.create(f"{SHM_PREFIX}{idx}", self.size[0], self.size[1],
                                     self.fmt, n_slots=n_slots)

    def start_camera(self) -> None:
        self.cam.start()

    def run(self) -> None:
        self._running = True
        try:
            while self._running:
                self._grab()
        except Exception as e:                       # 1台落ちても他は回す
            self.error = e
        finally:
            self._running = False

    def _grab(self) -> None:
        req = self.cam.capture_request()
        try:
            md = req.get_metadata()
            t_cap = md.get("SensorTimestamp", 0)
            fid = md.get("FrameCount", -1)
            # make_array は DMA バッファへの view。**release() すると無効になる**ので、
            # 共有メモリへの書き込みは release より前に済ませる。
            # ここで release を先にすると、解放済みメモリを読むことになる
            arr = req.make_array("main")
            if not arr.flags["C_CONTIGUOUS"]:
                arr = _contig(arr)
            t0 = time.monotonic_ns()
            desc = self.ring.write(arr, t_capture_ns=t_cap, frame_id=max(fid, 0))
            dt = time.monotonic_ns() - t0
        finally:
            req.release()

        s = self.stats
        s.frames += 1
        s.write_ns_total += dt
        s.write_ns_max = max(s.write_ns_max, dt)
        if s.last_t_capture and t_cap:
            s.gaps_ms.append((t_cap - s.last_t_capture) / 1e6)
        s.last_t_capture = t_cap
        if self._last_frame_id >= 0 and fid > self._last_frame_id + 1:
            s.dropped += fid - self._last_frame_id - 1
        self._last_frame_id = fid

        if self.on_frame:
            self.on_frame(desc)

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass
        self.ring.unlink()


def _contig(arr):
    import numpy as np

    return np.ascontiguousarray(arr)


class CameraNode:
    def __init__(self, indices: list[int], size, fmt: str, fps, n_slots: int,
                 on_frame=None) -> None:
        self.workers = [CameraWorker(i, size, fmt, fps, n_slots, on_frame)
                        for i in indices]
        self._t_start = 0.0

    def run(self, duration_s: float | None = None, status_cb=None) -> None:
        for w in self.workers:
            w.start_camera()
        time.sleep(0.5)                     # AE/AWB が落ち着くまで
        for w in self.workers:
            w.stats = CamStats()
            w.start()

        self._t_start = time.monotonic()
        next_status = 0.0
        try:
            while any(w.is_alive() for w in self.workers):
                el = time.monotonic() - self._t_start
                if duration_s is not None and el >= duration_s:
                    break
                if status_cb and el >= next_status:
                    status_cb(self)
                    next_status = el + 1.0
                time.sleep(0.05)
        finally:
            self.stop()

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self._t_start if self._t_start else 0.0

    def stop(self) -> None:
        for w in self.workers:
            w.stop()
        for w in self.workers:
            w.join(timeout=2.0)

    def close(self) -> None:
        for w in self.workers:
            w.close()


def _status(node: CameraNode) -> None:
    el = node.elapsed
    parts = [f"cam{w.idx}: {w.stats.frames}枚 "
             f"{w.stats.frames / el if el else 0:.1f}fps" for w in node.workers]
    sys.stdout.write("\r" + " | ".join(parts) + "    ")
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cameras", default="0,1", help="使うカメラ番号（例 0 / 0,1）")
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--fmt", default="RGB888")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--slots", type=int, default=DEFAULT_SLOTS,
                    help="リングの枚数（既定8。読み手が遅れても上書きされにくくなる）")
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    indices = [int(v) for v in args.cameras.split(",") if v.strip()]

    try:
        node = CameraNode(indices, (w, h), args.fmt, args.fps, args.slots)
    except Exception as e:
        print(f"カメラを開けない: {e}", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, lambda *_: node.stop())

    for wk in node.workers:
        print(f"# cam{wk.idx} -> /dev/shm/{wk.ring.name}  "
              f"{wk.ring.width}x{wk.ring.height} {wk.ring.fmt} "
              f"×{wk.ring.n_slots}枚 ({wk.ring._shm.size / 1e6:.1f}MB)")
    print("# 読み手: python3 -m raspi.tools.shm_view "
          f"{SHM_PREFIX}{indices[0]}\n")

    try:
        node.run(duration_s=args.duration,
                 status_cb=None if args.quiet else _status)
    finally:
        el = node.elapsed
        print("\n\n=== 終了時の統計 ===")
        for wk in node.workers:
            print(f"cam{wk.idx}: {wk.stats.summary(el)}")
            if wk.error:
                print(f"  !! 異常終了: {wk.error}")
        node.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
