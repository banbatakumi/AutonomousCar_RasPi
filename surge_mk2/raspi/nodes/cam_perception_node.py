"""cam_perception_node — 前方カメラの走行可否セグメンテーションを擬似 `Scan` に変換する。

    .venv/bin/python -m raspi.nodes.cam_perception_node --model model.onnx

`camera_node.py` が書く共有メモリ（`image/front` の `ImageRef`）を読み、ONNX
モデルで「走行可能／不可能」の2値マスクを作り、`raspi.nav.ipm` で地面座標へ
逆投影し、`raspi.nav.grid.OccGrid.raycast()`（既存のレイキャストをそのまま
使う）で角度ごとの距離配列に変換して `scan/cam`（`raspi/msgs/types.py`）へ
publish する。ギャップ探索そのものは書かない——それは
`raspi/auto/follow_the_gap_cam.py` が `FollowTheGap` をそのまま流用する。

## 契約1: `dist` を厳密に 0.0 にしない

`raspi/auto/base.py` の `scan_window()` は `dist[i] == 0.0` を「測距不能→
空き扱い」と読む（LiDAR 特有の穴。`dist==0` が「反射が返らなかった」だけで
射程外が多いという前提に基づく）。カメラ側で「本当に壁」を意味する値が
そこに落ちると誤って「空き」と解釈されるので、常に微小値でクランプする。

## 契約2: 推論が失敗したフレームは全セクタ `sector_seen=False`

LiDAR の「欠測は空きではない」（`follow_the_gap.py` の docstring）と同じ
考え方。フレームが読めない・推論が例外を吐いた周期は「壁」として安全側に
倒す（`Planner.plan()` 側の `ready=False`／停止に自然につながる）。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.base import sector_of_deg  # noqa: E402
from raspi.bus import FrameRing  # noqa: E402
from raspi.core.cleanup import quiet_close  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import ImageRef, Scan, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_SCAN_CAM,
    TOPIC_VEHICLE_STATE,
)
from raspi.nav.grid import OccGrid  # noqa: E402
from raspi.nav.ipm import CameraExtrinsics, camera_intrinsics, project_mask_to_grid  # noqa: E402

__all__ = ["SegmentationModel", "CamPerceptionNode"]

NS = 1_000_000_000
HB_HZ = 10
#: `dist` に絶対に置かない値（契約1）
_MIN_DIST = 0.01
#: 共有メモリの write_seq がこれ以上ズレたら作り直されたとみなす（`core/jpeg.py` と同じ流儀）
_STALE_GAP = 1000


class SegmentationModel:
    """ONNX 推論の薄いラッパ。

    **前処理定数（入力解像度・平均/分散・閾値）はモデルに同梱する契約**
    （`ml/export_onnx.py` が `model.json` に書き出す）。学習時と推論時で
    別々に決め打ちすると、いつか静かにズレる（train/inference skew）。
    ここではまだそのローダを持たず、コンストラクタ引数で明示的に渡す。
    """

    def __init__(self, model_path: str, *, input_size: tuple[int, int] = (224, 224),
                mean: float = 0.0, std: float = 255.0, threshold: float = 0.5) -> None:
        import onnxruntime as ort

        self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.input_size = input_size           # (width, height)
        self.mean = mean
        self.std = std
        self.threshold = threshold

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """`(H, W, C)` uint8 → モデル入力 `(1, C, h, w)` float32。"""
        w, h = self.input_size
        resized = _resize_nearest(frame[..., :3], w, h)
        x = (resized.astype(np.float32) - self.mean) / self.std
        return np.transpose(x, (2, 0, 1))[None, ...]

    def infer(self, frame: np.ndarray) -> np.ndarray:
        """`frame` (H, W, C) → 走行可能マスク（モデル入力解像度の bool 配列、True=走行可能）。"""
        x = self._preprocess(frame)
        out = self.session.run(None, {self.input_name: x})[0]
        prob = np.squeeze(out)
        return prob >= self.threshold


def _resize_nearest(frame: np.ndarray, width: int, height: int) -> np.ndarray:
    """最近傍法の縮小・拡大。**依存を増やさないための最小実装。**

    `cv2.resize`（双線形）より画質は劣るが、走行可否という粗いセグメンテー
    ションでは支障になりにくい想定。実データで画質不足が分かれば
    `raspi/requirements.txt` の `opencv-python-headless`（プレースホルダ済み）
    に切り替える。
    """
    src_h, src_w = frame.shape[:2]
    col = (np.arange(width) * src_w / width).astype(np.int32)
    row = (np.arange(height) * src_h / height).astype(np.int32)
    return frame[row][:, col]


class CamPerceptionNode:
    """1台の前方カメラ → 擬似 `Scan`。

    **`process_frame()` はバス・共有メモリを一切知らない純粋関数。**
    `run()`（実バス配線）とテスト（`raspi/tests/test_cam_perception_node.py`）
    の両方がここを通る——`raspi/auto/` の planner が「バスを知らない純粋な
    計算」であるのと同じ理由（配線とアルゴリズムを分けておくと、片方だけを
    テストで踏める）。
    """

    def __init__(self, *, model: SegmentationModel, vehicle: Vehicle | None = None,
                fov_deg: float = 60.0, max_range: float = 3.0,
                grid_resolution: float = 0.05, grid_size_m: float = 6.0) -> None:
        self.model = model
        self.vehicle = vehicle or Vehicle.load()
        v = self.vehicle
        #: 地面からの高さは base_link の z をそのまま使う近似（`ipm.py` docstring参照）
        self.base_ext = CameraExtrinsics(x=v.cam_front_x, y=v.cam_front_y,
                                         height=v.cam_front_z, pitch=v.cam_front_pitch,
                                         yaw=v.cam_front_yaw)
        self.fov_deg = fov_deg
        self.max_range = max_range
        self.grid_resolution = grid_resolution
        self.grid_size_m = grid_size_m

        half = int(round(fov_deg / 2))
        self._degs = np.arange(-half, half + 1)
        #: カメラの実視野に入る `sector_seen` の添字。**この外は常に False**
        self._sectors = sorted({sector_of_deg(int(d) % 360) for d in self._degs})

        self._ring: FrameRing | None = None
        self._ring_name = ""

    def close(self) -> None:
        if self._ring is not None:
            with quiet_close("cam_perception のフレームリング"):
                self._ring.close()
            self._ring = None

    # ── 1周期ぶんの処理（純粋関数。バスを知らない） ──

    def process_frame(self, frame: np.ndarray, *, vs: VehicleState | None = None,
                      t_capture_ns: int = 0, seq: int = 0) -> Scan:
        """1枚のフレーム → `Scan`。

        IMU が有効なら `pitch` を実測ぶん補正する（`gui/src/render/CameraView.tsx`
        の `drawGuide()` と同じ式・同じ理由——車体の加減速でピッチが動くと、
        地平線付近の投影誤差が発散するため）。
        """
        drivable = self.model.infer(frame)

        pitch = self.base_ext.pitch
        if vs is not None and vs.imu_ok:
            pitch = self.base_ext.pitch - vs.pitch
        ext = self.base_ext._replace(pitch=pitch)

        # **内部パラメータはマスクの実解像度から毎回作る。** モデル入力は
        # 生フレームとは別の解像度（例: 640x480 → 224x224）にリサイズされて
        # いるため、生フレーム基準の f/cx/principal_y をそのまま使うとズレる。
        # `camera_intrinsics()` の式は幅・高さに比例するので、リサイズが
        # アスペクト比を保っている限りこれで正しく縮尺が揃う
        h, w = drivable.shape
        intr = camera_intrinsics(self.vehicle.cam_front_hfov, w, h,
                                 self.vehicle.cam_front_bottom_crop)

        grid = OccGrid(resolution=self.grid_resolution, size_m=self.grid_size_m)
        occ = project_mask_to_grid(drivable, intr, ext, grid, stride=2)

        angles = np.radians(self._degs.astype(np.float64))
        dist = grid.raycast(0.0, 0.0, angles, self.max_range, mask=occ)
        dist = np.maximum(dist, _MIN_DIST)                 # 契約1

        full_dist = [0.0] * 360
        for d, val in zip(self._degs.tolist(), dist.tolist()):
            full_dist[d % 360] = float(val)
        sector_seen = [s in self._sectors for s in range(12)]
        return Scan(dist=full_dist, sector_seen=sector_seen, seq=seq,
                   t_capture=t_capture_ns, rot_speed_dps=0.0)

    def failed_frame(self, *, seq: int = 0) -> Scan:
        """フレームが読めない／推論が失敗した周期。**契約2＝全セクタ壁扱い。**"""
        return Scan(dist=[0.0] * 360, sector_seen=[False] * 12, seq=seq)

    # ── 共有メモリの読み取り（`raspi/core/jpeg.py` の掴み直しと同じ作法） ──

    def read_frame(self, ref: ImageRef) -> tuple[np.ndarray, int] | None:
        """`ref` が指す共有メモリから最新フレームを読む。読めなければ `None`。

        画素を使い終わってから `still_valid()` を確認する（seqlock）。
        **例外は投げない**——1周期読めなくても `failed_frame()` に落ちて
        続行できるようにする。
        """
        try:
            if self._ring is not None and ref.shm_name != self._ring_name:
                self.close()
            if self._ring is None:
                self._ring = FrameRing.attach(ref.shm_name)
                self._ring_name = ref.shm_name
            elif abs(ref.ring_seq - self._ring.write_seq) > _STALE_GAP:
                # 掴んでいるのは作り直される前の共有メモリ（`core/jpeg.py` と同じ検出）
                self.close()
                self._ring = FrameRing.attach(ref.shm_name)
                self._ring_name = ref.shm_name

            frame_ref = self._ring.latest()
            if frame_ref is None:
                return None
            arr = frame_ref.as_array().copy()      # 推論中ずっと保持するのでコピーする
            t_capture = frame_ref.desc.t_capture_ns
            ok = frame_ref.still_valid()
            if not ok:
                return None
            return arr, t_capture
        except Exception:
            self.close()
            return None

    # ── ループ（実バス配線） ──

    def run(self, *, sub, pub, duration_s: float | None = None,
           status_cb=None) -> None:
        seq = 0
        t_end = time.monotonic() + duration_s if duration_s else None
        next_hb = time.monotonic_ns()
        while True:
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
            pub.send(TOPIC_SCAN_CAM, st)
            seq += 1

            now = time.monotonic_ns()
            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "cam_perception",
                        HbMsg(node="cam_perception"))
            if status_cb:
                status_cb(st)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", required=True, help="ONNX モデルのパス")
    ap.add_argument("--input-size", default="224x224")
    ap.add_argument("--mean", type=float, default=0.0)
    ap.add_argument("--std", type=float, default=255.0)
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--fov-deg", type=float, default=60.0)
    ap.add_argument("--max-range", type=float, default=3.0)
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    w, h = (int(v) for v in args.input_size.lower().split("x"))
    model = SegmentationModel(args.model, input_size=(w, h), mean=args.mean,
                             std=args.std, threshold=args.threshold)
    node = CamPerceptionNode(model=model, fov_deg=args.fov_deg, max_range=args.max_range)

    pub = Publisher("cam_perception")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_VEHICLE_STATE: LATEST})

    print(f"# cam_perception_node  publish {pub.endpoint}  scan/cam へ配信")

    running = True

    def _shutdown(*_):
        nonlocal running
        running = False

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
