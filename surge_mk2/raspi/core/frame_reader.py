"""共有メモリの画像フレームを読む — `image/*` の `ImageRef` から生の配列を取り出す。

`raspi/core/jpeg.py`（同じセンサ画像を JPEG にする側）と同じ「最新スロットを
読んで、読み終えてから検証する」「`ring_seq` の乖離で作り直しを検出して
attach し直す」作法を、生配列がそのまま要る側（`cam_perception_node.py`・
`line_perception_node.py` 等、推論・CV 処理をする側）向けにまとめたもの。
JPEG 化はしない分だけ `RingJpeg` より薄い。

**この掴み直しの作法を2箇所に書き写さない。** 片方だけ直すと、もう片方が
古い共有メモリを掴んだまま「同じ画像を延々と処理し続ける」（`core/jpeg.py`
の docstring参照）というエラーの出ない壊れ方が残る。
"""

from __future__ import annotations

import numpy as np

from ..msgs import ImageRef
from .cleanup import quiet_close

__all__ = ["FrameReader"]

#: `ImageRef.ring_seq` とこちらが掴んでいる `write_seq` がこれ以上開いていたら、
#: 掴んでいるのは作り直される前の共有メモリだと判断して attach し直す
_STALE_GAP = 1000


class FrameReader:
    """1本の共有メモリリングだけを掴む薄いラッパ。

        reader = FrameReader()
        got = reader.read(ref)          # (arr, t_capture_ns) | None
        reader.close()

    複数カメラを読みたい場合は `ImageRef.shm_name` ごとに1個ずつ持つこと
    （このクラス自体は直近に読んだ1本のリングしか掴まない）。
    """

    def __init__(self) -> None:
        self._ring = None
        self._ring_name = ""

    def close(self) -> None:
        if self._ring is not None:
            with quiet_close("FrameReader のフレームリング"):
                self._ring.close()
            self._ring = None
            self._ring_name = ""

    def read(self, ref: ImageRef) -> tuple[np.ndarray, int] | None:
        """`ref` が指す共有メモリから最新フレームを読む。読めなければ `None`。

        画素を使い終わってから `still_valid()` を確認する（seqlock）。
        **例外は投げない**——1周期読めなくても呼び出し側が「失敗フレーム」に
        落ちて続行できるようにする。
        """
        try:
            from ..bus import FrameRing

            if self._ring is not None and ref.shm_name != self._ring_name:
                self.close()
            if self._ring is None:
                self._ring = FrameRing.attach(ref.shm_name)
                self._ring_name = ref.shm_name
            elif abs(ref.ring_seq - self._ring.write_seq) > _STALE_GAP:
                self.close()
                self._ring = FrameRing.attach(ref.shm_name)
                self._ring_name = ref.shm_name

            frame_ref = self._ring.latest()
            if frame_ref is None:
                return None
            arr = frame_ref.as_array().copy()      # 呼び出し側が保持する間ずっと使うのでコピーする
            t_capture = frame_ref.desc.t_capture_ns
            ok = frame_ref.still_valid()
            if not ok:
                return None
            return arr, t_capture
        except Exception:
            self.close()
            return None
