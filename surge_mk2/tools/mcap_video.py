"""`.mcap` のカメラ画像を mp4 に書き出す（Mac 側）。

    .venv/bin/python -m tools.mcap_video logs/run.mcap            # 前後それぞれ
    .venv/bin/python -m tools.mcap_video logs/run.mcap --cams front --fps 30

`logger_node` が記録した `/viz/image/<cam>`（`foxglove.CompressedImage`、JPEG）を読み、
カメラごとに `<入力>_<cam>.mp4` を作る。GUI は `tools/mcap_video_gui.py`。

Foxglove 自体の動画書き出しは Enterprise プラン限定なので、自前で持つ。

## 固定 fps に並べ直す（記録の間隔は一定ではない）

画像は `--image-hz`（既定 5Hz）に間引いて記録され、間隔も揺れる。mp4 は固定 fps なので、
**出力の各コマに「その時刻までで最新の画像」を置く**（同じ画像を繰り返して埋める）。
こうすると記録が途切れた区間も実時間のまま止まって見え、再生時刻が記録とずれない。

**時間軸は全カメラ共通**（最初の画像〜最後の画像）にしてあるので、前後の動画は
同じ長さになり、並べて再生すれば同期する。
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Sequence

import cv2
import numpy as np
from mcap.reader import NonSeekingReader, make_reader

from raspi.rec.mcap_log import VIZ_IMAGE_PREFIX

__all__ = ["DEFAULT_CAMS", "DEFAULT_FPS", "VideoResult", "default_out_path", "export_videos"]

DEFAULT_CAMS = ("front", "rear")
DEFAULT_FPS = 30.0

#: 試す順。`avc1`（H.264）は QuickTime・ブラウザでそのまま再生できる。
#: OpenCV のビルドによっては使えないので `mp4v` に落とす
_FOURCCS = ("avc1", "mp4v")


@dataclass(frozen=True)
class VideoResult:
    cam: str
    path: Path
    #: 記録に入っていた画像の枚数（動画のコマ数ではない）
    images: int
    duration_s: float
    codec: str
    #: 記録が途中で壊れていて、読めた所までで切り上げたときの理由（正常なら空）
    truncated: str = ""


def default_out_path(src: Path, cam: str, out_dir: Path | None = None) -> Path:
    return (out_dir or src.parent) / f"{src.stem}_{cam}.mp4"


def _open_writer(path: Path, fps: float, size: tuple[int, int]) -> tuple[cv2.VideoWriter, str]:
    for cc in _FOURCCS:
        vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*cc), fps, size)
        if vw.isOpened():
            return vw, cc
        vw.release()
    raise RuntimeError(f"動画を書き出せません（{'/'.join(_FOURCCS)} のどちらも開けない）: {path}")


class _Track:
    """1カメラ分の書き出し。コマ `k` は時刻 `t0 + k/fps` に対応する。"""

    def __init__(self, cam: str, path: Path, fps: float) -> None:
        self.cam = cam
        self.path = path
        self.fps = fps
        self.images = 0
        self.codec = ""
        self._vw: cv2.VideoWriter | None = None
        self._size: tuple[int, int] = (0, 0)
        self._last: np.ndarray | None = None
        self._written = 0

    def add(self, img: np.ndarray, slot: int) -> None:
        if self._vw is None:
            self._size = (img.shape[1], img.shape[0])
            self._vw, self.codec = _open_writer(self.path, self.fps, self._size)
        elif (img.shape[1], img.shape[0]) != self._size:
            img = cv2.resize(img, self._size)
        # 最初の画像より前のコマは、黒ではなく最初の画像で埋める
        self._fill(slot, self._last if self._last is not None else img)
        self._last = img
        self.images += 1

    def finish(self, end_slot: int) -> None:
        if self._vw is None:
            return
        self._fill(end_slot + 1, self._last)
        self._vw.release()
        self._vw = None

    def close(self) -> None:
        if self._vw is not None:
            self._vw.release()
            self._vw = None

    def _fill(self, until: int, img: np.ndarray) -> None:
        while self._written < until:
            self._vw.write(img)
            self._written += 1


def export_videos(src: Path, out_dir: Path | None = None, *,
                  cams: Sequence[str] = DEFAULT_CAMS, fps: float = DEFAULT_FPS,
                  progress: Callable[[float], None] | None = None) -> list[VideoResult]:
    """`src` の画像をカメラごとの mp4 にする。**画像の無いカメラは結果に入らない。**

    :param out_dir: 出力先。省略時は入力と同じフォルダ
    :param progress: 進み具合（0〜1）を受け取る。**総数の分からない記録では呼ばれない**
        （正常に閉じられなかった `.mcap` は summary が無い）
    """
    if fps <= 0:
        raise ValueError(f"fps は正の値にしてください: {fps}")
    tracks = {c: _Track(c, default_out_path(src, c, out_dir), fps) for c in cams}
    topics = [VIZ_IMAGE_PREFIX + c for c in cams]
    t0 = t_last = None
    truncated = ""
    try:
        with open(src, "rb") as f:
            reader = make_reader(f)
            try:
                total = _count_messages(reader, topics)
                ordered = True
            except Exception:  # noqa: BLE001 — 末尾が切れた記録は summary の読み出しで落ちる
                # 先頭から順に読む。**時刻順の並べ替えは頼めない**（全体を読み切ってから
                # 並べる実装なので、壊れた所で1枚も返さずに落ちる）。書いた順＝ほぼ時刻順
                f.seek(0)
                reader, total, ordered = NonSeekingReader(f), 0, False
            messages = enumerate(reader.iter_messages(topics=topics, log_time_order=ordered), 1)
            while True:
                try:
                    n, (_schema, channel, message) = next(messages)
                except StopIteration:
                    break
                except Exception as e:  # noqa: BLE001 — mcap・zstd のどちらの例外も来る
                    # 電源断などで末尾が壊れた記録。**読めた所までを動画にする**
                    # （`raspi/tools/mcap_repair.py` を通さなくても中身を見られるように）
                    if t0 is None:
                        raise
                    truncated = f"{type(e).__name__}: {e}"
                    break
                if progress is not None and total:
                    progress(min(1.0, n / total))
                jpg = base64.b64decode(json.loads(message.data)["data"])
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue                           # 壊れた1枚で全体を止めない
                if t0 is None:
                    t0 = message.log_time
                t_last = max(t_last or 0, message.log_time)
                tracks[channel.topic[len(VIZ_IMAGE_PREFIX):]].add(
                    img, _slot(message.log_time, t0, fps))
        results = []
        for tr in tracks.values():
            if tr.images == 0:
                continue
            tr.finish(_slot(t_last, t0, fps))
            results.append(VideoResult(tr.cam, tr.path, tr.images,
                                       (t_last - t0) / 1e9, tr.codec, truncated))
        return results
    finally:
        for tr in tracks.values():
            tr.close()


def _slot(t_ns: int, t0_ns: int, fps: float) -> int:
    return int((t_ns - t0_ns) * fps // 1_000_000_000)


def _count_messages(reader, topics: list[str]) -> int:
    summary = reader.get_summary()
    if summary is None or summary.statistics is None:
        return 0
    counts = summary.statistics.channel_message_counts
    return sum(counts.get(cid, 0) for cid, ch in summary.channels.items() if ch.topic in topics)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=".mcap のカメラ画像を mp4 に書き出す")
    ap.add_argument("mcap", nargs="+", type=Path)
    ap.add_argument("--out-dir", type=Path, default=None, help="出力先（省略時は入力と同じフォルダ）")
    ap.add_argument("--cams", nargs="+", default=list(DEFAULT_CAMS))
    ap.add_argument("--fps", type=float, default=DEFAULT_FPS)
    args = ap.parse_args(argv)
    for src in args.mcap:
        results = export_videos(src, args.out_dir, cams=args.cams, fps=args.fps)
        if not results:
            print(f"{src.name}: 画像がありません（--image-hz 0 で記録した？）", file=sys.stderr)
        for r in results:
            print(f"{r.path}  {r.images}枚 {r.duration_s:.1f}s {r.codec}")
        if results and results[0].truncated:
            print(f"{src.name}: 記録が途中で壊れています。読めた所までを書き出しました"
                  f"（{results[0].truncated}）", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
