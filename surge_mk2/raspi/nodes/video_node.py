"""video_node — カメラ映像を mp4 に録る（GUI の 🎥 録画ボタンの中身）。

    .venv/bin/python -m raspi.nodes.video_node --prefix logs/surge_20261016_120000

カメラごとに `<prefix>_front.mp4` / `<prefix>_rear.mp4` を作る（映像が来たカメラだけ）。
SIGINT/SIGTERM で書き終えて終わる——**mp4 は最後に索引を書くので、殺すと壊れる**。
`telemetry_node` は止めるときに `terminate()`（SIGTERM）を使う。

## 30fps の固定フレームに並べ直す

mp4 は固定 fps。カメラの取りこぼしや揺らぎで実時間とずれないよう、**フレームの
`t_capture` から出力のコマ番号を決め**、欠けたコマは直前の画で埋める
（`tools/mcap_video.py` と同じ考え方）。早く来すぎた画（同じコマ）は捨てる。

## 画は共有メモリの最新を読む

バスの `ImageRef` は「新しいフレームが出た」合図にだけ使い、画素は
`RingJpeg.read_latest` で共有メモリからコピーする（JPEG には焼かない＝画質劣化なし）。
コーデックは `avc1`（H.264）を試し、OpenCV のビルドに無ければ `mp4v`
（`tools/mcap_video.py` と同じ順）。Pi 5 に H.264 のハードウェアエンコーダは無い。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import LATEST, Subscriber  # noqa: E402
from raspi.core.jpeg import RingJpeg  # noqa: E402
from raspi.msgs.types import TOPIC_IMAGE_FRONT, TOPIC_IMAGE_REAR  # noqa: E402

NS = 1_000_000_000
DEFAULT_FPS = 30.0
#: 試す順。`avc1` は QuickTime・ブラウザでそのまま再生できる
FOURCCS = ("avc1", "mp4v")
POLL_MS = 10

CAM_TOPICS = {"front": TOPIC_IMAGE_FRONT, "rear": TOPIC_IMAGE_REAR}


class _CamWriter:
    """1台ぶんの mp4。最初のフレームが来たときに開く（映らないカメラのファイルは作らない）。"""

    def __init__(self, path: Path, fps: float) -> None:
        self.path = path
        self.fps = fps
        self.writer = None
        self.t0_ns = 0
        self.written = 0
        self.fourcc = ""

    def _open(self, size: tuple[int, int]) -> None:
        import cv2

        for code in FOURCCS:
            w = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*code), self.fps, size)
            if w.isOpened():
                self.writer, self.fourcc = w, code
                return
            w.release()
        raise RuntimeError(f"mp4 を開けない（{'/'.join(FOURCCS)} とも不可）: {self.path}")

    def add(self, bgr, t_capture_ns: int) -> None:
        if self.writer is None:
            self._open((bgr.shape[1], bgr.shape[0]))
            self.t0_ns = t_capture_ns
        # このフレームが載るべきコマ番号。欠けた分は同じ画で埋め、早すぎる画は捨てる
        slot = round((t_capture_ns - self.t0_ns) / NS * self.fps)
        while self.written <= slot:
            self.writer.write(bgr)
            self.written += 1

    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.writer = None


class VideoNode:
    def __init__(self, prefix: str | Path, *, fps: float = DEFAULT_FPS,
                 cams: tuple[str, ...] = ("front", "rear")) -> None:
        self.fps = fps
        self.cams = cams
        self.sub = Subscriber({CAM_TOPICS[c]: LATEST for c in cams})
        self.jpeg = RingJpeg()                    # `read_latest` を使うだけ
        self.writers = {c: _CamWriter(Path(f"{prefix}_{c}.mp4"), fps) for c in cams}
        self._last_seq: dict[str, tuple[str, int]] = {}
        self._running = False
        self.error: str | None = None
        self.t_start = time.monotonic()

    def stop(self) -> None:
        self._running = False

    def run(self, duration_s: float | None = None) -> None:
        self._running = True
        end = self.t_start + duration_s if duration_s else None
        topic_cam = {CAM_TOPICS[c]: c for c in self.cams}
        while self._running:
            if end is not None and time.monotonic() >= end:
                break
            for topic, ref in self.sub.poll(POLL_MS):
                cam = topic_cam.get(topic)
                if cam is None:
                    continue
                key = (ref.shm_name, ref.ring_seq)
                if self._last_seq.get(cam) == key:
                    continue                      # 新しいフレームではない
                got = self.jpeg.read_latest(ref.shm_name, expect_seq=ref.ring_seq)
                if got is None:
                    continue
                arr, colorspace, desc = got
                self._last_seq[cam] = key
                if colorspace == "RGB":
                    arr = arr[:, :, ::-1]         # OpenCV は BGR
                try:
                    self.writers[cam].add(arr, desc.t_capture_ns)
                except Exception as e:            # noqa: BLE001
                    self.error = f"{cam}: {type(e).__name__}: {e}"
                    self._running = False
                    break

    def close(self) -> None:
        for w in self.writers.values():
            w.close()
        self.jpeg.close()
        self.sub.close()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prefix", required=True,
                    help="出力の接頭辞（`<prefix>_front.mp4` / `<prefix>_rear.mp4`）")
    ap.add_argument("--fps", type=float, default=DEFAULT_FPS)
    ap.add_argument("--cams", default="front,rear")
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    args = ap.parse_args()

    Path(args.prefix).parent.mkdir(parents=True, exist_ok=True)
    node = VideoNode(args.prefix, fps=args.fps, cams=tuple(args.cams.split(",")))
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: node.stop())
    try:
        node.run(duration_s=args.duration)
    finally:
        node.close()
    for cam, w in node.writers.items():
        if w.written:
            print(f"{w.path}  {cam} {w.written}コマ {w.fourcc} "
                  f"{w.written / args.fps:.1f}s", file=sys.stderr)
    if node.error:
        print(f"!! {node.error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
