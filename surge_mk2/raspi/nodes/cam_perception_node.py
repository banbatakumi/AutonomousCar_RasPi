"""cam_perception_node — 前方カメラの走行可否セグメンテーションを擬似 `Scan` に変換する。

    .venv/bin/python -m raspi.nodes.cam_perception_node
    .venv/bin/python -m raspi.nodes.cam_perception_node --model model.onnx   # 開発用に既定モデルを直指定

`camera_node.py` が書く共有メモリ（`image/front` の `ImageRef`）を読み、ONNX
モデルで「走行可能／不可能」の2値マスクを作る。ここから2通りの出力を作る:

- `scan/cam`: `raspi.nav.ipm` で地面座標へ逆投影し、`OccGrid.raycast()` で
  車両原点からの角度ごとの距離配列（擬似 `Scan`）に変換したもの。
  `raspi/auto/follow_the_gap_cam.py` が `FollowTheGap` をそのまま流用する
- `path/cam`（`CamPath`）: 同じ占有格子から `raspi.nav.drivable_path` で前方
  複数距離ごとの走行可能な回廊の中心線を作ったもの。`raspi/auto/cam_centerline.py`
  が使う。`scan/cam` が車両原点1点からの角度スキャンに潰すのに対し、こちらは
  マスクの2次元的な広がり（回廊の幅・中心が前方距離ごとにどう変わるか）を
  残したまま渡す

判断そのもの（ギャップ探索・中心線追従）はここには書かない——`raspi/auto/` の
Planner の責務にする。

## どのモデルを使うかは `cam/model` トピックで決まる（GUI から選ぶ）

起動時に `--model` を渡さなければ、GUI（設定タブ）で選ばれたモデル名を
telemetry_node が `cam/model`（`CamModelCtrl`）で繰り返し流してくるのを待つ。
モデル名は `--models-dir`（既定 `models/`）配下の `<name>.onnx` に解決される
（前処理設定は同名の `<name>.json`。`ml_cam/export_onnx.py` が書く）。
`ftg_cam` を engage する前に、走行開始ボタンを押さずにモデルだけ選び直せる
——プロセスの再起動もSSHも要らない（`reload_if_changed()`）。

## 契約1: `dist` を厳密に 0.0 にしない

`raspi/auto/base.py` の `scan_window()` は `dist[i] == 0.0` を「測距不能→
空き扱い」と読む（LiDAR 特有の穴。`dist==0` が「反射が返らなかった」だけで
射程外が多いという前提に基づく）。カメラ側で「本当に壁」を意味する値が
そこに落ちると誤って「空き」と解釈されるので、常に微小値でクランプする。

## 契約2: 推論が失敗したフレームは全セクタ `sector_seen=False`

LiDAR の「欠測は空きではない」（`follow_the_gap.py` の docstring）と同じ
考え方。フレームが読めない・推論が例外を吐いた周期は「壁」として安全側に
倒す（`Planner.plan()` 側の `ready=False`／停止に自然につながる）。

## カメラ系モードが選ばれている間だけ推論する（IDLE/ACTIVE）

`cam_perception_node` はプロセスとしては常時起動（`surge-cam-perception`）
だが、CNN 推論はカメラフレームが来るたびに回るので上げっぱなしだと CPU・
電力を無駄に消費する。`cam_track_node.py`（`track/roi` の選択が無い間は
NanoTrack を回さない）と同じ考え方で、`auto/ctrl`（`AutoCtrl.mode`。
telemetry_node が GUI の選択を繰り返し流すトピック）を見て
`mode` が `_CAM_MODES`（`ftg_cam`・`cam_centerline`）のいずれかの間だけ
実際にフレームを読んで推論する。**どちらのモードでも同じ推論結果
（`drivable` マスク）を使い回すだけ**なので、片方だけ選ばれていてももう片方の
出力も一緒に計算して構わない（`scan/cam`／`path/cam` を毎回両方 publish する）。
非アクティブの間は `failed_frame()`（契約2の「壁」扱い）／`seen=False` の
`CamPath` を出すだけで、共有メモリの読み取りすらしない。
**`reload_if_changed()` はモード非依存で常に呼ぶ**——カメラ系モードに
切り替えた瞬間から推論を始められるよう、モデルだけは先にロードしておいてよい
（ONNXセッション生成はモデル切替時の一過性コスト）。
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.base import sector_of_deg  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.jpeg import make_encoder  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import AutoCtrl, CamMask, CamPath, ImageRef, Scan, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_CAM_MASK,
    TOPIC_CAM_MODEL,
    TOPIC_CAM_PATH,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_SCAN_CAM,
    TOPIC_VEHICLE_STATE,
)
from raspi.nav.drivable_path import extract_centerline  # noqa: E402
from raspi.nav.grid import OccGrid  # noqa: E402
from raspi.nav.ipm import (  # noqa: E402
    CameraExtrinsics,
    CameraIntrinsics,
    camera_intrinsics,
    project_mask_to_grid,
    project_seen_to_grid,
)

__all__ = ["SegmentationModel", "CamPerceptionNode"]

#: `surge_mk2/`。GUI から選ばれたモデル名を `models/<name>.onnx` に解決する基準
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODELS_DIR = REPO_ROOT / "models"

NS = 1_000_000_000
HB_HZ = 10
#: `dist` に絶対に置かない値（契約1）
_MIN_DIST = 0.01
#: 推論（CNN＋IPM＋raycast）を必要とする自動運転モード。**両方とも同じ推論結果
#: （`drivable` マスク）を使い回すだけ**なので、どちらが選ばれていても推論は1回で足りる
#: （`raspi/auto/follow_the_gap_cam.py` の `id`・`raspi/auto/cam_centerline.py` の `id`）
_CAM_MODES = ("ftg_cam", "cam_centerline")


class SegmentationModel:
    """ONNX 推論の薄いラッパ。

    **前処理定数（入力解像度・平均/分散・閾値）はモデルに同梱する契約**
    （`ml_cam/export_onnx.py` が `model.json` に書き出す）。学習時と推論時で
    別々に決め打ちすると、いつか静かにズレる（train/inference skew）。
    ここではまだそのローダを持たず、コンストラクタ引数で明示的に渡す。
    """

    def __init__(self, model_path: str, *, input_size: tuple[int, int] = (224, 224),
                mean: float = 0.0, std: float = 255.0, threshold: float = 0.5) -> None:
        import onnxruntime as ort

        # ★省電力化: 224x224程度の軽量モデルでは全4コアへ並列化するオーバーヘッドの
        # 方が大きく、かつ他ノード（camera_node/planning_node等）とコアを取り合う。
        # さらに ONNXRuntime は既定でスレッドをスピンウェイトさせ、推論の合間も
        # CPUを回し続けて電力を無駄にする（`session.*.allow_spinning`）ので明示的に切る。
        # 参照: https://onnxruntime.ai/docs/performance/tune-performance/threading.html
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1
        opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
        opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
        self.session = ort.InferenceSession(model_path, sess_options=opts,
                                            providers=["CPUExecutionProvider"])
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

    def __init__(self, *, model: SegmentationModel | None = None,
                models_dir: Path | None = None, loaded_model_name: str = "",
                vehicle: Vehicle | None = None,
                fov_deg: float = 60.0, max_range: float = 3.0,
                grid_resolution: float = 0.05, grid_size_m: float = 6.0,
                infer_hz: float = 10.0, mask_every: int = 1,
                path_x_min: float = 0.3, path_x_step: float = 0.1) -> None:
        #: **`None` は「まだモデルが選ばれていない」。** `run()` はこの間
        #: `failed_frame()` を出し続ける（契約2の「壁扱い」に自然に落ちる）
        self.model = model
        #: GUI が選んだモデル名（`cam/model`）から `models_dir/<name>.onnx` を探す
        self.models_dir = models_dir or DEFAULT_MODELS_DIR
        self._loaded_model_name = loaded_model_name
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
        #: `raspi/auto/cam_centerline.py` 用の中心線サンプリング範囲。
        #: `path_x_min` は車両直前（真下付近・IPM誤差が大きい範囲）を避けるための
        #: 下限。上限は `max_range` を流用する（`scan/cam` と探索範囲を揃える）
        self.path_x_min = path_x_min
        self.path_x_step = path_x_step

        # ★省電力化: 推論（ONNX＋IPM投影＋raycast）はカメラの実フレームレート
        # （既定30fps）ではなく `infer_hz` に間引く。`follow_the_gap_cam.py` の
        # `stale_ms=500` に対して10Hzなら十分余裕があり、間引いた周期は
        # `_last_scan` を seq/t_capture だけ更新して出し続ける（`failed_frame()`
        # にはしない——欠測扱いにすると planner が「壁」と読んで不要に止まる）
        self._infer_period_ns = int(NS / infer_hz) if infer_hz > 0 else 0
        self._last_infer_ns = 0
        self._last_scan: Scan | None = None
        #: `scan/cam` の `_last_scan` と同じ役割（`build_path()` の結果のキャッシュ）
        self._last_path: CamPath | None = None
        #: マスクJPEG（GUIプレビュー専用）のエンコード頻度。JPEGエンコード自体は
        #: CNN推論より軽いので既定は推論と同じ毎回（1）。粗くしたければ上げる
        self._mask_every = max(1, int(mask_every))
        self._infer_count = 0

        half = int(round(fov_deg / 2))
        self._degs = np.arange(-half, half + 1)
        #: カメラの実視野に入る `sector_seen` の添字。**この外は常に False**
        self._sectors = sorted({sector_of_deg(int(d) % 360) for d in self._degs})

        self._reader = FrameReader()

        #: GUI にマスクを重畳表示させるための JPEG エンコーダ（`cam/mask`）。
        #: 共有メモリ・`camera_node.py` とは無関係に、この場でモデル入力解像度の
        #: グレースケール1枚を直接エンコードする（`CamMask` の docstring参照）。
        #: エンコーダが無い環境（`simplejpeg`/`Pillow` どちらも無い）では `None`
        #: のままにし、`encode_mask_jpeg()` は諦めて `None` を返す
        self._encode_jpeg, _ = make_encoder(quality=70)
        #: 直近の `process_frame()` が作った走行可否マスク（bool, HxW）。
        #: **`process_frame()` 自体の戻り値は `Scan` のまま変えていない**——
        #: 既存のテスト・呼び出し側の契約を壊さないための側路（`encode_mask_jpeg()` 参照）
        self._last_drivable: np.ndarray | None = None
        #: `process_frame()` が最後に使った幾何（`build_path()` が使い回す）。
        #: **`build_path()` はバスも共有メモリも知らない純粋関数のままにするため**、
        #: 推論・IPM投影を2度走らせるのではなく、ここに一度だけ計算した結果を置く
        self._last_intr: CameraIntrinsics | None = None
        self._last_ext: CameraExtrinsics | None = None
        self._last_grid: OccGrid | None = None
        self._last_occ: np.ndarray | None = None
        self._running = False
        #: 直前周期の ACTIVE/IDLE。切り替わった周期だけログを出すための記憶
        self._active = False

    def close(self) -> None:
        self._reader.close()

    # ── モデルの切替（GUI からの `cam/model` を受けて呼ばれる） ──

    def _load_model_by_name(self, name: str) -> SegmentationModel:
        """`models_dir/<name>.onnx`（+ 同名 `.json`。前処理設定）を読む。

        `.json` が無ければ `SegmentationModel` の既定値（`ml_cam/dataset.py` の
        正規化と揃えてある 0-1 正規化）で読む——`ml_cam/export_onnx.py` を通さずに
        手で置いた `.onnx` でも動かせるようにするための保険で、
        **積極的に使うことは想定していない**（train/inference skew の温床）。
        """
        onnx_path = self.models_dir / f"{name}.onnx"
        if not onnx_path.exists():
            raise FileNotFoundError(f"モデルが見つかりません: {onnx_path}")
        cfg = {}
        cfg_path = onnx_path.with_suffix(".json")
        if cfg_path.exists():
            cfg = json.loads(cfg_path.read_text())
        w, h = cfg.get("input_size", [224, 224])
        return SegmentationModel(str(onnx_path), input_size=(int(w), int(h)),
                                 mean=float(cfg.get("mean", 0.0)),
                                 std=float(cfg.get("std", 255.0)),
                                 threshold=float(cfg.get("threshold", 0.5)))

    def reload_if_changed(self, desired_name: str) -> bool:
        """`desired_name` が今ロード中のモデルと違えば読み込み直す。

        **失敗しても今のモデルを保持する。** GUI が壊れた・存在しないモデル名を
        選んでも、それだけで走行が止まる（`self.model` が `None` に戻って
        `failed_frame()` の壁扱いになる）よりは、前のモデルで走り続けられる
        方が安全側——ただし選び間違いに気づけるようログには残す。

        切り替えられたら `True`、そのまま／失敗なら `False`。
        """
        if not desired_name or desired_name == self._loaded_model_name:
            return False
        try:
            self.model = self._load_model_by_name(desired_name)
            self._loaded_model_name = desired_name
            print(f"# モデル切替: {desired_name}", flush=True)
            return True
        except Exception as e:                                # noqa: BLE001
            print(f"# モデル読み込み失敗（{desired_name}）: {e}。"
                 f"前のモデルのまま続行", file=sys.stderr, flush=True)
            return False

    # ── 1周期ぶんの処理（純粋関数。バスを知らない） ──

    def process_frame(self, frame: np.ndarray, *, vs: VehicleState | None = None,
                      t_capture_ns: int = 0, seq: int = 0) -> Scan:
        """1枚のフレーム → `Scan`。

        IMU が有効なら `pitch` を実測ぶん補正する（`gui/src/render/CameraView.tsx`
        の `drawGuide()` と同じ式・同じ理由——車体の加減速でピッチが動くと、
        地平線付近の投影誤差が発散するため）。
        """
        drivable = self.model.infer(frame)
        self._last_drivable = drivable

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
        #: `build_path()` が同じフレームの推論結果を使い回すための置き場
        #: （幾何は毎フレーム変わりうる——IMU有効時は `pitch` が補正されるため）
        self._last_intr = intr
        self._last_ext = ext
        self._last_grid = grid
        self._last_occ = occ

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

    def build_path(self, *, seq: int = 0, t_capture_ns: int = 0) -> CamPath:
        """直前の `process_frame()` の結果から走行可能領域の中心線を作る。

        **`process_frame()` の直後にだけ呼ぶこと。** 幾何（`_last_grid` 等）を
        持ち回っているので、`process_frame()` を一度も呼んでいない・直前の呼び出しが
        `failed_frame()` だった周期に呼ぶと `seen=False` の空の `CamPath` を返す
        （契約2と同じ「分からなければ壁／未検出扱い」）。
        """
        if self._last_grid is None or self._last_occ is None or self._last_intr is None:
            return CamPath(seq=seq, t_capture=t_capture_ns)
        seen_grid = project_seen_to_grid(self._last_drivable.shape, self._last_intr,
                                         self._last_ext, self._last_grid, stride=2)
        blocked = (~seen_grid) | self._last_occ
        xs, ys, widths = extract_centerline(self._last_grid, blocked,
                                            x_min=self.path_x_min, x_max=self.max_range,
                                            x_step=self.path_x_step)
        return CamPath(seen=True, xs=xs, ys=ys, widths=widths,
                       seq=seq, t_capture=t_capture_ns)

    def encode_mask_jpeg(self) -> bytes | None:
        """直近の `process_frame()` が作った走行可否マスクを JPEG 化する（`cam/mask` 用）。

        白＝走行可能、黒＝不可のグレースケール1枚（モデル入力解像度のまま、
        例 224×224）。エンコーダが無い環境、または `process_frame()` がまだ
        一度も呼ばれていない（＝モデル未選択）間は `None`——呼び出し元
        （`run()`）はその場合 publish を諦める（カメラ映像そのものと同じ扱い）。
        """
        if self._encode_jpeg is None or self._last_drivable is None:
            return None
        img = (self._last_drivable.astype(np.uint8)) * 255
        return self._encode_jpeg(img, "GRAY")

    # ── 共有メモリの読み取り（`raspi/core/frame_reader.py` に委譲） ──

    def read_frame(self, ref: ImageRef) -> tuple[np.ndarray, int] | None:
        """`ref` が指す共有メモリから最新フレームを読む。読めなければ `None`。"""
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

            model_ctrl = sub.latest.get(TOPIC_CAM_MODEL)
            if model_ctrl is not None:
                self.reload_if_changed(model_ctrl.name)

            auto_ctrl = sub.latest.get(TOPIC_AUTO_CTRL)
            active = auto_ctrl is not None and auto_ctrl.mode in _CAM_MODES
            if active != self._active:
                self._active = active
                mode = auto_ctrl.mode if auto_ctrl is not None else "?"
                print(f"# {mode} {'選択: 推論開始' if active else '非選択: 推論停止'}",
                     flush=True)

            ref = sub.latest.get(TOPIC_IMAGE_FRONT)
            vs = sub.latest.get(TOPIC_VEHICLE_STATE)
            now = time.monotonic_ns()
            #: `Publisher.send()` は呼ぶたびに独自の seq を打ち直す（`bus/zbus.py`
            #: `Publisher._send()`）ので、中身が同じでも publish すれば
            #: planning_node には「新しい周」に見えてしまい、`_replan()` の
            #: 重複排除（`scan.seq == self._last_scan_seq`）が効かない。
            #: 間引きで推論を休んだ周期は **publish 自体を省略する**——
            #: `stale_ms=500` に対して `infer_hz`（既定10Hz=100ms間隔）の実publishで
            #: 十分足りるので、鮮度は損なわない（`scan/cam`・`path/cam` 共通）
            should_publish = True
            path_msg: CamPath | None = None
            if not active:
                # どちらのカメラ系モードも選ばれていない間は共有メモリすら読まない
                # （CPU/IO を無駄に使わない。上のモジュールdocstring参照）
                st = self.failed_frame(seq=seq)
                self._last_scan = None
                self._last_path = None
            elif self.model is None:
                # **契約2の入口。** まだ GUI がモデルを選んでいない（起動直後・
                # 未選択のまま）。フレームが来ていても推論しようがない
                st = self.failed_frame(seq=seq)
                self._last_scan = None
                self._last_path = None
            elif ref is None:
                st = self.failed_frame(seq=seq)
                self._last_scan = None
                self._last_path = None
            else:
                got = self.read_frame(ref)
                if got is None:
                    st = self.failed_frame(seq=seq)
                    self._last_scan = None
                    self._last_path = None
                elif (self._last_scan is not None
                      and now - self._last_infer_ns < self._infer_period_ns):
                    # ★省電力化: まだ間引き周期に達していない。前回の推論結果を
                    # そのまま維持し、publish はしない（上のコメント参照）
                    st = self._last_scan
                    path_msg = self._last_path
                    should_publish = False
                else:
                    frame, t_capture = got
                    self._last_infer_ns = now
                    st = self.process_frame(frame, vs=vs, t_capture_ns=t_capture, seq=seq)
                    self._last_scan = st
                    #: `cam_centerline` が選ばれていなくても、同じ推論結果から
                    #: ただで作れる（下の `_CAM_MODES` docstring参照）ので常に計算する
                    path_msg = self.build_path(seq=seq, t_capture_ns=t_capture)
                    self._last_path = path_msg
                    self._infer_count += 1
                    # マスクは既定で推論と同じ頻度（`mask_every=1`）で配る。GUIの
                    # ライブプレビュー専用で走行判断には使わない。
                    # **failed_frame・間引きの間は送らない**——GUI 側は直前の
                    # マスクが静止して見えるだけで、暴走はしない
                    if self._infer_count == 1 or self._infer_count % self._mask_every == 0:
                        mask_jpeg = self.encode_mask_jpeg()
                        if mask_jpeg is not None:
                            pub.send(TOPIC_CAM_MASK, CamMask(jpeg=mask_jpeg, seq=seq))
            if should_publish:
                pub.send(TOPIC_SCAN_CAM, st)
                pub.send(TOPIC_CAM_PATH, path_msg if path_msg is not None
                         else CamPath(seq=seq))
            seq += 1

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "cam_perception",
                        HbMsg(node="cam_perception"))
            if status_cb:
                status_cb(st)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=None,
                    help="起動時の既定モデル（ONNXパス。省略時は cam/model トピック"
                         "経由のGUI選択を待つ）")
    ap.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR),
                    help="GUIが選んだモデル名から探すディレクトリ（既定 models/）")
    ap.add_argument("--input-size", default="224x224",
                    help="--model 直指定のときだけ使う入力解像度")
    ap.add_argument("--mean", type=float, default=0.0)
    ap.add_argument("--std", type=float, default=255.0)
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--fov-deg", type=float, default=60.0)
    ap.add_argument("--max-range", type=float, default=3.0)
    ap.add_argument("--infer-hz", type=float, default=10.0,
                    help="推論（ONNX＋IPM投影＋raycast）を回す上限頻度。カメラの実"
                         "フレームレートより低くして省電力化する。"
                         "follow_the_gap_cam.py の stale_ms=500 に対して十分な余裕を"
                         "残すこと（既定10Hzなら100ms間隔）")
    ap.add_argument("--mask-every", type=int, default=1,
                    help="cam/mask（GUIプレビュー）を配る間隔。推論N回に1回だけ配る。"
                         "JPEGエンコード自体はCNN推論より軽いので既定は推論と同じ毎回")
    ap.add_argument("--path-x-min", type=float, default=0.3,
                    help="cam_centerline用の中心線サンプリングの前方下限[m]。"
                         "車両直前（IPM誤差が大きい範囲）を避ける")
    ap.add_argument("--path-x-step", type=float, default=0.1,
                    help="cam_centerline用の中心線サンプリング間隔[m]")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    model = None
    if args.model:
        w, h = (int(v) for v in args.input_size.lower().split("x"))
        model = SegmentationModel(args.model, input_size=(w, h), mean=args.mean,
                                 std=args.std, threshold=args.threshold)
    node = CamPerceptionNode(model=model, models_dir=Path(args.models_dir),
                             fov_deg=args.fov_deg, max_range=args.max_range,
                             infer_hz=args.infer_hz, mask_every=args.mask_every,
                             path_x_min=args.path_x_min, path_x_step=args.path_x_step)

    pub = Publisher("cam_perception")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_VEHICLE_STATE: LATEST,
                      TOPIC_CAM_MODEL: LATEST, TOPIC_AUTO_CTRL: LATEST})

    print(f"# cam_perception_node  publish {pub.endpoint}  scan/cam・path/cam へ配信")

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
