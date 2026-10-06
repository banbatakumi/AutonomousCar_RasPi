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

## 帯の重心は「細い」領域だけを使う（毛布・段ボール等の誤検出対策）

実車確認（2026-09-03）で、`near_band`/`far_band` 内に毛布・カーテン・段ボール
等の大きく明るい面があると、単純な帯内重心は白線ではなくそちらへ引っ張られる
ことが分かった。白線は数cm幅で「細い」という物理的な制約を使い、`_thin_runs()`
で行ごとに幅が `_MAX_LINE_WIDTH_FRAC`（画像幅比）を超える連続領域を除いてから
重心を取る。ただし**同じ室内に他の細長い明るい構造（棚の縁・ドア枠等）が
あると、形状フィルタだけでは区別できない**——これは実コース以外での検証が
原理的に当たらない領域なので、最終確認は実コース上で行うこと。

## 帯は「地面の距離」で決める（`near_range` / `far_range`）

帯を画面高さの割合で決めると、`bottom_crop`・レンズ・取付高さ・ピッチが変わるたびに
地面でない行（空・車体）へ当たる（160° 広角への交換で `far_band` が地平線より上になった、
2026-10-03）。そこで既定は **base_link 前方の距離 `(手前, 奥)` [m]** で持ち、
`ground_to_pixel()` で画像の行へ換算する（校正値・取付位置・`bottom_crop` が効く）。
車体はカメラに対して固定でボンネットの上端より下に写るので、`near_range` の手前側を
その行より奥（距離が大きい側）に置けば車体は入らない。
`near_band` / `far_band`（画面高さ割合）を明示すると、距離指定より優先する。
換算は静的な取付姿勢で行う（走行中のピッチ補正は、行→地面の逆投影側で効く）。

## `line_trace` が選ばれている間だけ認識する（IDLE/ACTIVE）

プロセスとしては常時起動（`surge-line-perception`）だが、`white_mask()` は
フレームが来るたびに毎回回るので上げっぱなしだと CPU・電力を無駄に消費する。
`cam_perception_node.py`（`auto/ctrl` の `mode` がカメラ系モードの間だけ推論する）
と同じ考え方で、`mode == "line_trace"` で、**かつ DISARM中（駐車中）でない**間だけ
実際にフレームを読んで判定する（`raspi/core/auto_gate.cam_infer_active()` に集約、
`cam_perception_node.py`・`cam_e2e_node.py` と共通）。DISARM中はモードが選ばれて
いるだけでは判定しない（2026-09-04、省電力バグとして修正）。非アクティブの間は
`failed_frame()`（見失った扱い）を出すだけで、共有メモリの読み取りすらしない。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.core.auto_gate import IdlePacer, cam_infer_active  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import ImageRef, LineScan, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_LINE_CAM,
    TOPIC_VEHICLE_STATE,
)
from raspi.nav.ipm import (  # noqa: E402
    CameraExtrinsics,
    ground_to_pixel,
    pixel_to_ground,
    vehicle_camera_intrinsics,
)

__all__ = ["LinePerceptionNode", "white_mask"]

NS = 1_000_000_000
HB_HZ = 10
#: 帯の中で検出できた行の割合がこれ未満なら「その帯には無い」とみなす。
#: 0 にすると単発のノイズ画素1個でも目標点になり、舵が暴れる
_MIN_BAND_FRAC = 0.01
#: 白線とみなす連続白画素の最大幅（画像幅に対する割合）。これを超える幅の
#: 連続領域は毛布・段ボール等の面とみなして除く（上のモジュールdocstring参照）。
#: 実車確認で毛布・カーテンの塊は画像幅の9〜13%あった一方、白線は数%に収まる
_MAX_LINE_WIDTH_FRAC = 0.08
#: この `auto/ctrl` モードが選ばれている間だけ認識する（上のモジュールdocstring参照）
_LINE_MODE = "line_trace"


def white_mask(frame: np.ndarray, *, min_brightness: int = 170,
               max_chroma: int = 40) -> np.ndarray:
    """`(H, W, C)` uint8 → 白線らしい画素の bool マスク。

    HSV へ変換せず、RGB の最小値（明るさの下限）と最大−最小（彩度の近似）
    だけで判定する。白線は「明るく・色が付いていない」の2条件で十分に分離
    でき、`cv2` 依存を増やさずに済む（`cam_perception_node.py` の
    `_resize_nearest` と同じ理由——依存を増やす前にこれで足りるか確かめる）。

    :param min_brightness: RGB各chの最小値がこれ未満なら白ではない
    :param max_chroma: RGBの最大−最小がこれを超えたら色が付いている＝白ではない

    ## なぜ `int16` へ拡げず uint8 のまま計算するか（issue #14）

    以前は `frame[..., :3].astype(np.int16)` で640x360x3を一旦 int16
    （1.4MB）に広げてから、長さ3の最終軸に対して `min`/`max`（`axis=-1`）を
    取っていた——これは numpy 的に遅い形で、実測 12.8ms/枚（x86）かかっていた。
    ここでは **チャネルごとに** `minimum`/`maximum` を取る（`hi >= lo` が
    常に成り立つので `hi - lo` を uint8 のまま引いても溢れない）。実測
    0.42ms（約30倍）。**乱数フレームで旧実装と `np.array_equal` の完全一致を
    確認済み**（`raspi/tests/test_line_perception_node.py`）——数値が変わると
    白線検出の閾値が事実上変わってしまうため、単なる高速化のつもりで
    近似値になっていないことをテストで保証している。
    """
    r, g, b = frame[..., 0], frame[..., 1], frame[..., 2]
    lo = np.minimum(np.minimum(r, g), b)
    hi = np.maximum(np.maximum(r, g), b)
    return (lo >= min_brightness) & ((hi - lo) <= max_chroma)


def _thin_runs(row: np.ndarray, max_width_px: int) -> list[float]:
    """1行ぶんの bool 配列から、幅が `max_width_px` 以下の連続領域の中心列を列挙する。

    白線は数cm幅で「細い」という制約を使った形状フィルタ——色（明るく無彩色）
    だけでは毛布・カーテンのような大きな面と区別できないため（上のモジュール
    docstring参照）。
    """
    if not row.any():
        return []
    padded = np.concatenate(([False], row, [False]))
    edges = np.diff(padded.astype(np.int8))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    return [float(s + e) / 2.0 for s, e in zip(starts, ends) if (e - s) <= max_width_px]


def _band_centroid(mask: np.ndarray, v0: int, v1: int,
                   max_width_px: int) -> tuple[float, float, float] | None:
    """行 `[v0, v1)` の帯における細い白領域の重心 `(u, v, frac)`。

    `frac` は帯の中で細い白領域を検出できた**行の割合**。`_MIN_BAND_FRAC`
    未満なら `None`（見えているのがノイズか、その帯には線が無い）。
    """
    v0 = max(0, v0)
    v1 = min(mask.shape[0], v1)
    if v1 <= v0:
        return None
    us: list[float] = []
    vs: list[int] = []
    for r in range(v0, v1):
        for u in _thin_runs(mask[r], max_width_px):
            us.append(u)
            vs.append(r)
    if not vs:
        return None
    frac = len(set(vs)) / (v1 - v0)
    if frac < _MIN_BAND_FRAC:
        return None
    return float(np.mean(us)), float(np.mean(vs)), frac


class LinePerceptionNode:
    """1台の前方カメラ → 白線の目標点2つ（`LineScan`）。

    **`process_frame()` はバス・共有メモリを一切知らない純粋関数。**
    `run()`（実バス配線）とテストの両方がここを通る——
    `cam_perception_node.CamPerceptionNode` と同じ理由（配線とアルゴリズムを
    分けておくと、片方だけをテストで踏める）。
    """

    def __init__(self, *, vehicle: Vehicle | None = None,
                min_brightness: int = 170, max_chroma: int = 40,
                near_range: tuple[float, float] = (1.05, 1.5),
                far_range: tuple[float, float] = (1.6, 3.5),
                near_band: tuple[float, float] | None = None,
                far_band: tuple[float, float] | None = None) -> None:
        self.vehicle = vehicle or Vehicle.load()
        v = self.vehicle
        #: 地面からの高さは base_link の z をそのまま使う近似（`ipm.py` docstring参照）
        self.base_ext = CameraExtrinsics(x=v.cam_front_x, y=v.cam_front_y,
                                         height=v.cam_front_z, pitch=v.cam_front_pitch,
                                         yaw=v.cam_front_yaw)
        self.min_brightness = min_brightness
        self.max_chroma = max_chroma
        #: 帯の指定。既定は base_link 前方の距離 `(手前, 奥)` [m]（モジュール docstring 参照）
        self.near_range = near_range
        self.far_range = far_range
        #: 画面高さに対する割合 `(top, bottom)`（0=最上段、1=最下段）。**明示したときだけ**
        #: 距離指定より優先する（`None` なら距離から換算する）
        self.near_band = near_band
        self.far_band = far_band
        self._bands_cache: dict[tuple[int, int], tuple[tuple[float, float], tuple[float, float]]] = {}

        self._reader = FrameReader()
        self._running = False
        #: 直前周期の ACTIVE/IDLE。切り替わった周期だけログを出すための記憶
        self._active = False
        #: ★省電力化: 直近に処理した `ImageRef.ring_seq`。カメラは30fps・この
        #: ループは約10ms周期（`sub.poll(20)`）で回るので、間を置かずに読むと
        #: 同じフレームを約3回処理してしまう（issue #13）。`ring_seq` が
        #: 変わっていなければ共有メモリの読み取り自体をスキップし、前回の
        #: 結果をそのまま使い回す
        self._last_ring_seq: int = -1
        self._last_scan: LineScan | None = None

    def close(self) -> None:
        self._reader.close()

    def _bands(self, w: int, h: int) -> tuple[tuple[float, float], tuple[float, float]]:
        """`(near, far)` の帯を画面高さ割合 `(top, bottom)` で返す。`(w, h)` ごとに1回だけ換算する。"""
        key = (w, h)
        cached = self._bands_cache.get(key)
        if cached is not None:
            return cached
        intr = vehicle_camera_intrinsics(self.vehicle, "front", w, h)

        def from_range(rng: tuple[float, float]) -> tuple[float, float]:
            rows = []
            for d in rng:
                uv = ground_to_pixel(d, self.base_ext.y, intr, self.base_ext)
                rows.append(None if uv is None else uv[1])
            if None in rows:                        # 地平線より上＝その距離は映らない
                return (0.0, 0.0)
            top, bottom = sorted(rows)
            return (min(max(top / h, 0.0), 1.0), min(max(bottom / h, 0.0), 1.0))

        near = self.near_band if self.near_band is not None else from_range(self.near_range)
        far = self.far_band if self.far_band is not None else from_range(self.far_range)
        self._bands_cache[key] = (near, far)
        return near, far

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
        intr = vehicle_camera_intrinsics(self.vehicle, "front", w, h)

        st = LineScan(t_capture=t_capture_ns, seq=seq)
        coverages: list[float] = []
        max_width_px = max(1, int(_MAX_LINE_WIDTH_FRAC * w))

        near_band, far_band = self._bands(w, h)
        near = _band_centroid(mask, int(near_band[0] * h), int(near_band[1] * h),
                              max_width_px)
        if near is not None:
            u, vpix, frac = near
            g = pixel_to_ground(u, vpix, intr, ext)
            if g is not None:
                st.near_seen = True
                st.near_x, st.near_y = g
                coverages.append(frac)

        far = _band_centroid(mask, int(far_band[0] * h), int(far_band[1] * h),
                             max_width_px)
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
        pacer = IdlePacer()
        while self._running:
            if t_end and time.monotonic() >= t_end:
                break
            for _ in sub.poll(pacer.poll_timeout_ms(self._active)):
                pass  # `latest` を見るだけなので中身の処理は不要

            auto_ctrl = sub.latest.get(TOPIC_AUTO_CTRL)
            vs = sub.latest.get(TOPIC_VEHICLE_STATE)
            active = cam_infer_active(auto_ctrl, vs, (_LINE_MODE,))
            if active != self._active:
                self._active = active
                mode = auto_ctrl.mode if auto_ctrl is not None else "?"
                if active:
                    reason = "選択: 認識開始"
                elif auto_ctrl is None or auto_ctrl.mode != _LINE_MODE:
                    reason = "非選択: 認識停止"
                else:
                    reason = "DISARM: 認識停止"
                print(f"# {mode} {reason}", flush=True)

            reused = False
            if not active:
                # `line_trace` が選ばれていない、またはDISARM中は共有メモリすら読まない
                # （CPU/IO を無駄に使わない。上のモジュールdocstring参照）
                st = self.failed_frame(seq=seq)
            else:
                ref = sub.latest.get(TOPIC_IMAGE_FRONT)
                if ref is None:
                    st = self.failed_frame(seq=seq)
                    self._last_scan = None
                elif ref.ring_seq == self._last_ring_seq and self._last_scan is not None:
                    # ★省電力化: カメラの新しいフレームがまだ来ていない
                    # （上の `_last_ring_seq` docstring参照）。共有メモリの
                    # 読み取り自体をスキップし、前回の結果を使い回す。**publish もしない**
                    # （2026-10-06）。同じ結果を送り直すと `Publisher` が `seq` を振り直すので、
                    # planning_node が「新しい周」と見て判断し直し、`auto/cmd` まで増える
                    # （実機で `line/cam` 130Hz・`auto/cmd` 128Hz。カメラは 30fps）。
                    # 鮮度は planning_node が `t_capture` で見ているので、送らなくても
                    # カメラが止まれば制動に落ちる
                    st = self._last_scan
                    reused = True
                else:
                    got = self.read_frame(ref)
                    if got is None:
                        st = self.failed_frame(seq=seq)
                        self._last_scan = None
                    else:
                        frame, t_capture = got
                        self._last_ring_seq = ref.ring_seq
                        try:
                            st = self.process_frame(frame, vs=vs, t_capture_ns=t_capture, seq=seq)
                        except Exception as e:
                            # 推論(白線検出)側のバグでノード全体を巻き込んで
                            # 落とさない。契約の「見失った扱い」に自然に落とす
                            # （`planning_node._replan()` と同じパターン）
                            print(f"# line_perception process_frame() が例外: {e}",
                                 file=sys.stderr, flush=True)
                            st = self.failed_frame(seq=seq)
                            self._last_scan = None
                        else:
                            self._last_scan = st
            now = time.monotonic_ns()
            # IDLE中は約2Hzに間引く（`IdlePacer`。省電力、2026-10-04）
            if not reused and pacer.should_publish(active, now):
                pub.send(TOPIC_LINE_CAM, st)
            seq += 1

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "line_perception",
                        HbMsg(node="line_perception"))
            if status_cb:
                status_cb(st)
            pacer.idle_sleep(active)


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
    ap.add_argument("--near-range", default="1.05,1.5",
                    help="近傍帯の base_link 前方距離 手前,奥 [m]（既定。車体のボンネットより奥）")
    ap.add_argument("--far-range", default="1.6,3.5",
                    help="遠方帯の base_link 前方距離 手前,奥 [m]")
    ap.add_argument("--near-band", default=None,
                    help="近傍帯の画面高さ割合 top,bottom。指定すると --near-range より優先")
    ap.add_argument("--far-band", default=None,
                    help="遠方帯の画面高さ割合 top,bottom。指定すると --far-range より優先")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    node = LinePerceptionNode(min_brightness=args.min_brightness,
                              max_chroma=args.max_chroma,
                              near_range=_parse_band(args.near_range),
                              far_range=_parse_band(args.far_range),
                              near_band=_parse_band(args.near_band) if args.near_band else None,
                              far_band=_parse_band(args.far_band) if args.far_band else None)

    pub = Publisher("line_perception")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_VEHICLE_STATE: LATEST,
                      TOPIC_AUTO_CTRL: LATEST})

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
