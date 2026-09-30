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

import time

import numpy as np

from ..msgs import ImageRef
from .cleanup import quiet_close

__all__ = ["FrameReader"]

#: `ImageRef.ring_seq` とこちらが掴んでいる `write_seq` がこれ以上開いていたら、
#: 掴んでいるのは作り直される前の共有メモリだと判断して attach し直す
_STALE_GAP = 1000

#: `t_capture`（センサ取得時刻、CLOCK_MONOTONIC）がこれより古いフレームは
#: 「新鮮」として扱わない。**camera_node が死ぬ／`capture_request()` がハング
#: しても、共有メモリの mapping 自体は生きたままなので `ring.latest()` は
#: 同じフレームを返し続け、`still_valid()` も True のまま**——中身が全く
#: 更新されていなくても、見た目上は正常なフレームに見えてしまう
#: （issue #2）。知覚ノードは20msごとに同じ画像を推論・publishし続け、
#: `planning_node` 側の鮮度判定は `t_pub`（publish時刻）しか見ていないので
#: 「凍った画像に基づく『走れ』」が制動に化けない。ここで撮像時刻そのものの
#: 年齢を見て弾くことで、上流（`camera_node`）が死んでいる限り必ず
#: `failed_frame` 側に落ちるようにする
_MAX_FRAME_AGE_NS = 200_000_000


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
            if time.monotonic_ns() - t_capture > _MAX_FRAME_AGE_NS:
                # 撮像から時間が経ちすぎている＝上流（`camera_node`）が死んでいる
                # か固まっている。共有メモリ自体は読めてしまうので、ここで
                # 明示的に弾かないと「凍った画像」がいつまでも新鮮に見える
                return None
            return arr, t_capture
        except Exception:
            self.close()
            return None
