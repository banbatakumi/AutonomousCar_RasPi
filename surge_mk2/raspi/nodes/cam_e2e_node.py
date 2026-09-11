"""cam_e2e_node — 前方カメラの画像から操舵角を直接回帰する（模倣学習、幾何変換を経由しない）。

    .venv/bin/python -m raspi.nodes.cam_e2e_node
    .venv/bin/python -m raspi.nodes.cam_e2e_node --model model.onnx   # 開発用に既定モデルを直指定

`camera_node.py` が書く共有メモリ（`image/front` の `ImageRef`）を読み、ONNX
モデルで正規化した操舵角（`steer_norm`、-1..1）を直接推論する。
`cam_perception_node.py`（走行可否セグメンテーション→IPM→擬似`Scan`）と違い、
**幾何変換（IPM）を一切経由しない**——低いカメラ高さでIPMの深度誤差が
拡大する問題を、画像から操舵を直接学習することで迂回するのがこのノードの
存在理由（`docs/development.md` §12.3）。

## 独立の安全策として LiDAR 前方距離も同梱する

模倣学習は「学習データに無い状況」でどう振る舞うか原理的に保証できない
（`raspi/auto/e2e_lidar.py` と同じ理由）。カメラだけに徹すると、正面が
詰まったときに止める根拠が丸ごとモデル出力任せになってしまうので、
**`scan`（LiDAR）も購読して正面付近の最小距離を一緒に publish する。**
低い壁には効かないが（そもそもLiDARが見えない）、それ以外の一般障害物
（人・パイロン・普通の高さの壁）への最後の砦として `raspi/auto/cam_e2e.py`
の `stop_dist` 判定がこの値を使う。

## 契約: 推論が失敗した／モデル未選択のフレームは `ready=False`

`cam_perception_node.py` の契約2（欠測は壁扱い）と同じ考え方。カメラ側が
死んでいても LiDAR 側は独立に更新し続ける——`ready=False` はあくまで
「操舵をこの周期のモデル出力に任せてよいか」の判定であって、安全策の可否とは別。

## カメラ系モードが選ばれている間だけ推論する

`cam_perception_node.py` と同じ節電方針。`cam_e2e_node` はプロセスとしては
常時起動（`surge-cam-e2e`）だが、`auto/ctrl` の `mode` が `cam_e2e` で、**かつ
DISARM中（駐車中）でない**間だけ実際にフレームを読んで推論する
（`raspi/core/auto_gate.cam_infer_active()` に集約、`cam_perception_node.py`・
`line_perception_node.py` と共通）。DISARM中はモードが選ばれているだけでは
推論しない——GUI再読み込みで前回選択モードがそのまま復元されるため、
armed/engaged を見ずにモード選択だけで駆動すると駐車中の待機でも推論が
回り続けてしまう（2026-09-04、省電力バグとして修正）。
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

from raspi.auto.base import scan_window  # noqa: E402
from raspi.core.auto_gate import cam_infer_active  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import AutoCtrl, CamE2ECmd, ImageRef, Scan, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_CAM_E2E_CMD,
    TOPIC_CAM_E2E_MODEL,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
)
from raspi.nodes.cam_perception_node import _resize_nearest  # noqa: E402

__all__ = ["RegressionModel", "CamE2ENode"]

REPO_ROOT = Path(__file__).resolve().parents[2]
#: カメラ用セグメンテーションモデルと同じ `models/` 直下（`cam_e2e/model` という
#: 別トピックで選ぶので混同しない。`raspi/msgs/types.py` の `TOPIC_CAM_E2E_MODEL` 参照）
DEFAULT_MODELS_DIR = REPO_ROOT / "models"

NS = 1_000_000_000
HB_HZ = 10
#: 推論（CNN）を必要とする自動運転モード
_CAM_E2E_MODES = ("cam_e2e",)


class RegressionModel:
    """ONNX 推論の薄いラッパ。出力は `steer_norm`（tanh、-1..1）1個。

    前処理契約（入力解像度・平均/分散）と出力契約（`max_steer`）はモデルに
    同梱する（`ml_cam_e2e/export_onnx.py` が `<name>.json` に書く）。
    `raspi/nodes/cam_perception_node.py` の `SegmentationModel` と同じ設計。
    """

    def __init__(self, model_path: str, *, input_size: tuple[int, int] = (224, 224),
                mean: float = 0.0, std: float = 255.0, max_steer: float = 0.0) -> None:
        import onnxruntime as ort

        # `cam_perception_node.SegmentationModel` と同じ省電力設定
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
        self.max_steer = max_steer

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        w, h = self.input_size
        resized = _resize_nearest(frame[..., :3], w, h)
        x = (resized.astype(np.float32) - self.mean) / self.std
        return np.transpose(x, (2, 0, 1))[None, ...]

    def infer(self, frame: np.ndarray) -> float:
        """`frame` (H, W, C) → `steer_norm`（-1..1。既にモデル側で `tanh` 済み）。"""
        x = self._preprocess(frame)
        out = self.session.run(None, {self.input_name: x})[0]
        return float(np.squeeze(out))


class CamE2ENode:
    """1台の前方カメラ → 操舵の直接回帰 ＋ LiDAR前方距離（安全策）。

    **`process_frame()`/`front_lidar_dist()` はバス・共有メモリを一切知らない
    純粋関数。** `run()`（実バス配線）とテストの両方がここを通る
    （`cam_perception_node.py` と同じ設計方針）。
    """

    def __init__(self, *, model: RegressionModel | None = None,
                models_dir: Path | None = None, loaded_model_name: str = "",
                vehicle: Vehicle | None = None,
                lidar_fov_deg: float = 40.0, lidar_max_range: float = 3.0,
                infer_hz: float = 10.0) -> None:
        #: **`None` は「まだモデルが選ばれていない」。** `run()` はこの間 `ready=False` を出し続ける
        self.model = model
        self.models_dir = models_dir or DEFAULT_MODELS_DIR
        self._loaded_model_name = loaded_model_name
        self.vehicle = vehicle or Vehicle.load()
        self.lidar_fov_deg = lidar_fov_deg
        self.lidar_max_range = lidar_max_range

        # ★省電力化: `cam_perception_node.py` と同じ理由で推論を間引く
        self._infer_period_ns = int(NS / infer_hz) if infer_hz > 0 else 0
        self._last_infer_ns = 0
        self._last_cmd: CamE2ECmd | None = None
        self._running = False
        self._active = False
        self._reader = FrameReader()

    def close(self) -> None:
        self._reader.close()

    # ── モデルの切替 ──

    def _load_model_by_name(self, name: str) -> RegressionModel:
        onnx_path = self.models_dir / f"{name}.onnx"
        if not onnx_path.exists():
            raise FileNotFoundError(f"モデルが見つかりません: {onnx_path}")
        cfg = {}
        cfg_path = onnx_path.with_suffix(".json")
        if cfg_path.exists():
            cfg = json.loads(cfg_path.read_text())
        w, h = cfg.get("input_size", [224, 224])
        return RegressionModel(str(onnx_path), input_size=(int(w), int(h)),
                               mean=float(cfg.get("mean", 0.0)),
                               std=float(cfg.get("std", 255.0)),
                               max_steer=float(cfg.get("max_steer", 0.0)))

    def reload_if_changed(self, desired_name: str) -> bool:
        """`cam_perception_node.CamPerceptionNode.reload_if_changed()` と同じ契約
        （失敗しても今のモデルを保持する）。"""
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

    # ── 1周期ぶんの処理（純粋関数） ──

    def process_frame(self, frame: np.ndarray) -> float:
        """1枚のフレーム → `steer_norm`（-1..1）。`self.model` が必須（呼び出し側が保証）。"""
        return self.model.infer(frame)

    def front_lidar_dist(self, scan: Scan | None) -> tuple[float, bool]:
        """`(前方最小距離[m], 実測に基づくか)`。`scan` が無ければ `(0.0, False)`。

        `raspi/auto/e2e_lidar.py` の `free_ahead` と同じ切り出し方
        （`scan_window()` で前方視野を切り出し、正面±`lidar_fov_deg/2` の最小値を取る）。
        """
        if scan is None:
            return 0.0, False
        w = scan_window(scan, self.lidar_fov_deg, self.lidar_max_range)
        if w.seen_ratio <= 0.0:
            return 0.0, False
        return min(w.dist) if w.dist else 0.0, True

    def failed_cmd(self, *, seq: int = 0, t_capture_ns: int = 0) -> CamE2ECmd:
        """フレームが読めない／モデル未選択の周期。**`ready=False`。**"""
        return CamE2ECmd(ready=False, seq=seq, t_capture=t_capture_ns)

    # ── 共有メモリの読み取り ──

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
                pass

            model_ctrl = sub.latest.get(TOPIC_CAM_E2E_MODEL)
            if model_ctrl is not None:
                self.reload_if_changed(model_ctrl.name)

            auto_ctrl = sub.latest.get(TOPIC_AUTO_CTRL)
            vs = sub.latest.get(TOPIC_VEHICLE_STATE)
            active = cam_infer_active(auto_ctrl, vs, _CAM_E2E_MODES)
            if active != self._active:
                self._active = active
                mode = auto_ctrl.mode if auto_ctrl is not None else "?"
                if active:
                    reason = "選択: 推論開始"
                elif auto_ctrl is None or auto_ctrl.mode not in _CAM_E2E_MODES:
                    reason = "非選択: 推論停止"
                else:
                    reason = "DISARM: 推論停止"
                print(f"# {mode} {reason}", flush=True)

            now = time.monotonic_ns()
            if not active:
                cmd = self.failed_cmd(seq=seq)
                self._last_cmd = None
            elif self.model is None:
                cmd = self.failed_cmd(seq=seq)
                self._last_cmd = None
            else:
                ref = sub.latest.get(TOPIC_IMAGE_FRONT)
                scan = sub.latest.get(TOPIC_SCAN)
                lidar_dist, lidar_seen = self.front_lidar_dist(scan)
                if ref is None:
                    cmd = self.failed_cmd(seq=seq)
                    self._last_cmd = None
                elif (self._last_cmd is not None
                      and now - self._last_infer_ns < self._infer_period_ns):
                    # ★省電力化: 間引き周期内。前回の推論結果（steer）はそのまま
                    # 使い回すが、LiDAR前方距離は安全策なので毎周期更新する
                    prev = self._last_cmd
                    cmd = CamE2ECmd(ready=prev.ready, steer_norm=prev.steer_norm,
                                    model_max_steer=prev.model_max_steer,
                                    lidar_front_dist=lidar_dist, lidar_seen=lidar_seen,
                                    seq=seq, t_capture=prev.t_capture)
                    self._last_cmd = cmd
                else:
                    got = self.read_frame(ref)
                    if got is None:
                        cmd = self.failed_cmd(seq=seq)
                        self._last_cmd = None
                    else:
                        frame, t_capture = got
                        self._last_infer_ns = now
                        try:
                            steer_norm = self.process_frame(frame)
                        except Exception as e:
                            # 推論側のバグでノード全体を巻き込んで落とさない。
                            # 契約の「ready=False」に自然に落とす
                            # （`planning_node._replan()` と同じパターン）
                            print(f"# cam_e2e process_frame() が例外: {e}",
                                 file=sys.stderr, flush=True)
                            cmd = self.failed_cmd(seq=seq, t_capture_ns=t_capture)
                            self._last_cmd = None
                        else:
                            cmd = CamE2ECmd(ready=True, steer_norm=steer_norm,
                                            model_max_steer=self.model.max_steer,
                                            lidar_front_dist=lidar_dist, lidar_seen=lidar_seen,
                                            seq=seq, t_capture=t_capture)
                            self._last_cmd = cmd
            # **`cam_perception_node.py` と違い、間引き周期中も毎回 publish する。**
            # あちらは推論結果（内容）が変わらないので重複排除に任せてよいが、
            # ここは LiDAR 前方距離（安全策）が毎周期変わりうる値で、それを
            # 間引くと「安全策だけ古いまま」になりかねないため
            pub.send(TOPIC_CAM_E2E_CMD, cmd)
            seq += 1

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "cam_e2e", HbMsg(node="cam_e2e"))
            if status_cb:
                status_cb(cmd)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=None,
                    help="起動時の既定モデル（ONNXパス。省略時は cam_e2e/model トピック"
                         "経由のGUI選択を待つ）")
    ap.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    ap.add_argument("--input-size", default="224x224",
                    help="--model 直指定のときだけ使う入力解像度")
    ap.add_argument("--mean", type=float, default=0.0)
    ap.add_argument("--std", type=float, default=255.0)
    ap.add_argument("--max-steer", type=float, default=0.0,
                    help="--model 直指定のときだけ使う出力契約（0ならvehicle.tomlを使う）")
    ap.add_argument("--lidar-fov-deg", type=float, default=40.0)
    ap.add_argument("--lidar-max-range", type=float, default=3.0)
    ap.add_argument("--infer-hz", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    vehicle = Vehicle.load()
    model = None
    if args.model:
        w, h = (int(v) for v in args.input_size.lower().split("x"))
        max_steer = args.max_steer if args.max_steer > 0 else vehicle.max_steer
        model = RegressionModel(args.model, input_size=(w, h), mean=args.mean,
                                std=args.std, max_steer=max_steer)
    node = CamE2ENode(model=model, models_dir=Path(args.models_dir), vehicle=vehicle,
                      lidar_fov_deg=args.lidar_fov_deg, lidar_max_range=args.lidar_max_range,
                      infer_hz=args.infer_hz)

    pub = Publisher("cam_e2e")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_SCAN: LATEST,
                      TOPIC_VEHICLE_STATE: LATEST, TOPIC_CAM_E2E_MODEL: LATEST,
                      TOPIC_AUTO_CTRL: LATEST})

    print(f"# cam_e2e_node  publish {pub.endpoint}  cam_e2e/cmd へ配信")

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
