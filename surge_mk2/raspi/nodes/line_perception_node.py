"""line_perception_node — 前方カメラの白線を検出し、地面座標の目標点2つに変換する。

    .venv/bin/python -m raspi.nodes.line_perception_node

`camera_node.py` が書く共有メモリ（`image/front` の `ImageRef`）を読み、明るく
無彩色な画素（白線らしさ）を古典的な色しきい値で抜き出し、画面下寄りの帯
（近傍）と中央寄りの帯（遠方）それぞれで重心の画素位置を求める。
`raspi.nav.ipm.pixel_to_ground()` で地面座標へ逆投影した2点を `LineScan`
（`raspi/msgs/types.py`）として `line/cam` へ publish する。

`cam_perception_node.py`（走行可能／不可能セグメンテーション）とは別モジュール
にしてあるのは、**ここは学習済みモデルを一切要らない**ため——白線は「明るく
無彩色」という単純な色特徴で足り、ONNX 推論のコストも学習データも不要。
走行可能領域そのものを知りたい場合（未舗装路の縁・低い障害物など）は引き続き
`cam_perception_node.py` を使う。

## 契約: 見失ったら `near_seen`/`far_seen` を両方 False にする

`raspi/auto/line_trace.py` はこれを「白線が無い」と読み、`ready=False`
（＝制動）に倒す。**中間的な「たぶんある」を作らない**——閾値ぎりぎりの検出を
そのまま座標に変換すると、ノイズで目標点が暴れて舵が振動する
（`_MIN_BAND_FRAC` 未満はその帯に「無い」扱いにする）。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import ImageRef, LineScan, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_LINE_CAM,
    TOPIC_VEHICLE_STATE,
)
from raspi.nav.ipm import CameraExtrinsics, camera_intrinsics, pixel_to_ground  # noqa: E402

__all__ = ["LinePerceptionNode", "white_mask"]

NS = 1_000_000_000
HB_HZ = 10
#: 帯の中で白画素がこれ未満の割合なら「その帯には無い」とみなす。
#: 0 にすると単発のノイズ画素1個でも目標点になり、舵が暴れる
_MIN_BAND_FRAC = 0.01


def white_mask(frame: np.ndarray, *, min_brightness: int = 170,
               max_chroma: int = 40) -> np.ndarray:
    """`(H, W, C)` uint8 → 白線らしい画素の bool マスク。

    HSV へ変換せず、RGB の最小値（明るさの下限）と最大−最小（彩度の近似）
    だけで判定する。白線は「明るく・色が付いていない」の2条件で十分に分離
    でき、`cv2` 依存を増やさずに済む（`cam_perception_node.py` の
    `_resize_nearest` と同じ理由——依存を増やす前にこれで足りるか確かめる）。

    :param min_brightness: RGB各chの最小値がこれ未満なら白ではない
    :param max_chroma: RGBの最大−最小がこれを超えたら色が付いている＝白ではない
    """
    f = frame[..., :3].astype(np.int16)
    lo = f.min(axis=-1)
    hi = f.max(axis=-1)
    return (lo >= min_brightness) & ((hi - lo) <= max_chroma)


def _band_centroid(mask: np.ndarray, v0: int, v1: int) -> tuple[float, float, float] | None:
    """行 `[v0, v1)` の帯における白画素の重心 `(u, v, frac)`。

    帯の中で白画素の割合が `_MIN_BAND_FRAC` 未満なら `None`
    （見えているのがノイズか、その帯には線が無い）。
    """
    v0 = max(0, v0)
    v1 = min(mask.shape[0], v1)
    if v1 <= v0:
        return None
    band = mask[v0:v1]
    rows, cols = np.nonzero(band)
    frac = rows.size / band.size if band.size else 0.0
    if frac < _MIN_BAND_FRAC:
        return None
    return float(cols.mean()), float(v0 + rows.mean()), frac


class LinePerceptionNode:
    """1台の前方カメラ → 白線の目標点2つ（`LineScan`）。

    **`process_frame()` はバス・共有メモリを一切知らない純粋関数。**
    `run()`（実バス配線）とテストの両方がここを通る——
    `cam_perception_node.CamPerceptionNode` と同じ理由（配線とアルゴリズムを
    分けておくと、片方だけをテストで踏める）。
    """

    def __init__(self, *, vehicle: Vehicle | None = None,
                min_brightness: int = 170, max_chroma: int = 40,
                near_band: tuple[float, float] = (0.80, 1.00),
                far_band: tuple[float, float] = (0.55, 0.75)) -> None:
        self.vehicle = vehicle or Vehicle.load()
        v = self.vehicle
        #: 地面からの高さは base_link の z をそのまま使う近似（`ipm.py` docstring参照）
        self.base_ext = CameraExtrinsics(x=v.cam_front_x, y=v.cam_front_y,
                                         height=v.cam_front_z, pitch=v.cam_front_pitch,
                                         yaw=v.cam_front_yaw)
        self.min_brightness = min_brightness
        self.max_chroma = max_chroma
        #: 画面高さに対する割合 `(top, bottom)`。0=最上段、1=最下段
        self.near_band = near_band
        self.far_band = far_band

        self._reader = FrameReader()
        self._running = False

    def close(self) -> None:
        self._reader.close()

    # ── 1周期ぶんの処理（純粋関数。バスを知らない） ──

    def process_frame(self, frame: np.ndarray, *, vs: VehicleState | None = None,
                      t_capture_ns: int = 0, seq: int = 0) -> LineScan:
        """1枚のフレーム → `LineScan`。

        IMU が有効なら `pitch` を実測ぶん補正する（`cam_perception_node.py`
        と同じ式・同じ理由——車体の加減速でピッチが動くと、地平線付近の
        投影誤差が発散するため）。
        """
        mask = white_mask(frame, min_brightness=self.min_brightness,
                          max_chroma=self.max_chroma)
        h, w = mask.shape

        pitch = self.base_ext.pitch
        if vs is not None and vs.imu_ok:
            pitch = self.base_ext.pitch - vs.pitch
        ext = self.base_ext._replace(pitch=pitch)
        intr = camera_intrinsics(self.vehicle.cam_front_hfov, w, h,
                                 self.vehicle.cam_front_bottom_crop)

        st = LineScan(t_capture=t_capture_ns, seq=seq)
        coverages: list[float] = []

        near = _band_centroid(mask, int(self.near_band[0] * h), int(self.near_band[1] * h))
        if near is not None:
            u, vpix, frac = near
            g = pixel_to_ground(u, vpix, intr, ext)
            if g is not None:
                st.near_seen = True
                st.near_x, st.near_y = g
                coverages.append(frac)

        far = _band_centroid(mask, int(self.far_band[0] * h), int(self.far_band[1] * h))
        if far is not None:
            u, vpix, frac = far
            g = pixel_to_ground(u, vpix, intr, ext)
            if g is not None:
                st.far_seen = True
                st.far_x, st.far_y = g
                coverages.append(frac)

        st.seen = st.near_seen or st.far_seen
        st.coverage = max(coverages) if coverages else 0.0
        return st

    def failed_frame(self, *, seq: int = 0) -> LineScan:
        """フレームが読めない周期。**契約＝見失った扱い。**"""
        return LineScan(seq=seq)

    # ── 共有メモリの読み取り（`raspi/core/frame_reader.py` に委譲） ──

    def read_frame(self, ref: ImageRef) -> tuple[np.ndarray, int] | None:
        return self._reader.read(ref)

    # ── ループ（実バス配線） ──

    def stop(self) -> None:
        self._running = False

    def run(self, *, sub, pub, duration_s: float | None = None,
           status_cb=None) -> None:
        self._running = True
        seq = 0
        t_end = time.monotonic() + duration_s if duration_s else None
        next_hb = time.monotonic_ns()
        while self._running:
            if t_end and time.monotonic() >= t_end:
                break
            for _ in sub.poll(20):
                pass  # `latest` を見るだけなので中身の処理は不要
            ref = sub.latest.get(TOPIC_IMAGE_FRONT)
            vs = sub.latest.get(TOPIC_VEHICLE_STATE)
            if ref is None:
                st = self.failed_frame(seq=seq)
            else:
                got = self.read_frame(ref)
                if got is None:
                    st = self.failed_frame(seq=seq)
                else:
                    frame, t_capture = got
                    st = self.process_frame(frame, vs=vs, t_capture_ns=t_capture, seq=seq)
            pub.send(TOPIC_LINE_CAM, st)
            seq += 1

            now = time.monotonic_ns()
            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "line_perception",
                        HbMsg(node="line_perception"))
            if status_cb:
                status_cb(st)


def _parse_band(s: str) -> tuple[float, float]:
    a, b = (float(x) for x in s.split(","))
    return a, b


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-brightness", type=int, default=170,
                    help="RGB各chの最小値がこれ未満なら白ではない")
    ap.add_argument("--max-chroma", type=int, default=40,
                    help="RGBの最大−最小がこれを超えたら色が付いている＝白ではない")
    ap.add_argument("--near-band", default="0.80,1.00",
                    help="近傍帯の画面高さ割合 top,bottom")
    ap.add_argument("--far-band", default="0.55,0.75",
                    help="遠方帯の画面高さ割合 top,bottom")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    node = LinePerceptionNode(min_brightness=args.min_brightness,
                              max_chroma=args.max_chroma,
                              near_band=_parse_band(args.near_band),
                              far_band=_parse_band(args.far_band))

    pub = Publisher("line_perception")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_VEHICLE_STATE: LATEST})

    print(f"# line_perception_node  publish {pub.endpoint}  line/cam へ配信")

    def _shutdown(*_):
        node.stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _shutdown)

    try:
        node.run(sub=sub, pub=pub, duration_s=args.duration)
    finally:
        node.close()
        sub.close()
        pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
