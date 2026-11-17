"""共有メモリの画素形式の変換を1か所に置く（`ImageRef.fmt`＝メモリ上の実際の並び）。

カメラは `YUV420`（I420: Y 面 H×W、続いて U 面・V 面 各 H/2×W/2）で撮る。色空間は
**sYCC（全範囲の BT.601＝JPEG の YCbCr と同じ）**に固定している（`camera_node`）。

- ステレオは Y 面をそのまま灰色として使う（変換なし）
- GUI・記録の JPEG は Y・U・V 面からそのまま焼く（`core/jpeg.py`。RGB から焼くより速い）
- CNN 系など色が要る側だけ、ここで RGB/BGR に直す

**OpenCV の `COLOR_YUV2BGR_I420` は使わない。** 制限範囲（16〜235）を前提にしており、
sYCC の値を通すと色が平均 4 ずれた（2026-11-17 実機、RGB888 で撮った画との比較）。ここでは
色差を元の大きさに戻してから全範囲の `YCrCb2BGR` を通す（RGB888 の画との差は平均 2.4。
残りは色差を縦横半分にした 4:2:0 の分で、記録の JPEG も同じ 4:2:0）。

`BGR888` などの3〜4バイト形式もそのまま扱える（シム・古い記録・テスト）。並びの判定は従来どおり
「`BGR` で始まるか」（`core/jpeg.py` と同じ規則。変えると4バイト形式の色が入れ替わる）。
"""

from __future__ import annotations

import numpy as np

__all__ = ["YUV420", "is_yuv420", "image_hw", "yuv420_planes", "to_bgr", "to_rgb", "to_gray"]

YUV420 = "YUV420"


def is_yuv420(fmt: str) -> bool:
    return fmt == YUV420


def image_hw(frame: np.ndarray, fmt: str) -> tuple[int, int]:
    """画像としての `(高さ, 幅)`（YUV420 の配列は高さが 1.5 倍あるため）。"""
    if is_yuv420(fmt):
        return frame.shape[0] * 2 // 3, frame.shape[1]
    return frame.shape[0], frame.shape[1]


def yuv420_planes(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """I420 の `(H*3/2, W)` 配列 → `(Y, U, V)` の view。"""
    h, w = frame.shape[0] * 2 // 3, frame.shape[1]
    q = (h // 2) * (w // 2)
    flat = frame.reshape(-1)
    u = flat[h * w: h * w + q].reshape(h // 2, w // 2)
    v = flat[h * w + q: h * w + 2 * q].reshape(h // 2, w // 2)
    return frame[:h], u, v


def _yuv420_to_bgr(frame: np.ndarray) -> np.ndarray:
    import cv2

    y, u, v = yuv420_planes(frame)
    h, w = y.shape
    cr = cv2.resize(v, (w, h), interpolation=cv2.INTER_LINEAR)
    cb = cv2.resize(u, (w, h), interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(cv2.merge([np.ascontiguousarray(y), cr, cb]), cv2.COLOR_YCrCb2BGR)


def to_bgr(frame: np.ndarray, fmt: str) -> np.ndarray:
    """共有メモリの生フレーム → BGR の `(H, W, 3)`。"""
    if is_yuv420(fmt):
        return _yuv420_to_bgr(frame)
    if fmt.startswith("BGR"):
        return frame[..., :3]
    return np.ascontiguousarray(frame[..., 2::-1])


def to_rgb(frame: np.ndarray, fmt: str) -> np.ndarray:
    """共有メモリの生フレーム → RGB の `(H, W, 3)`。BGR 系は入れ替えた view を返す（コピーしない）。"""
    if is_yuv420(fmt):
        return _yuv420_to_bgr(frame)[..., ::-1]
    if fmt.startswith("BGR"):
        return frame[..., 2::-1]
    return frame[..., :3]


def to_gray(frame: np.ndarray, fmt: str) -> np.ndarray:
    """共有メモリの生フレーム → 灰色の `(H, W)`。YUV420 は Y 面の view（変換なし）。"""
    if is_yuv420(fmt):
        return frame[: frame.shape[0] * 2 // 3]
    if frame.ndim == 2:
        return frame
    import cv2

    img = frame[:, :, :3] if frame.shape[2] == 4 else frame
    code = cv2.COLOR_RGB2GRAY if fmt.startswith("RGB") else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(np.ascontiguousarray(img), code)
