"""cam_e2e_node — 前方カメラの画像から操舵と速度を直接回帰する（模倣学習、幾何変換を経由しない）。

    .venv/bin/python -m raspi.nodes.cam_e2e_node
    .venv/bin/python -m raspi.nodes.cam_e2e_node --model model.onnx   # 開発用に既定モデルを直指定

`camera_node.py` が書く共有メモリ（`image/front` の `ImageRef`）を読み、ONNX
モデルで正規化した操舵（`steer_norm`、-1..1）と速度（`speed_norm`、0..1）を
直接推論する。`cam_perception_node.py`（走行可否セグメンテーション→IPM→
擬似`Scan`）と違い、**幾何変換（IPM）を一切経由しない**——低いカメラ高さで
IPMの深度誤差が拡大する問題を、画像から操作を直接学習することで迂回するのが
このノードの存在理由（`docs/development.md` §12.3）。

## カメラだけで完結する（2026-10-06）

以前は LiDAR の前方距離を同梱して速度則に使っていたが、「カメラの映像だけで
走る」というモードの目的に合わないので外した。速度もモデルが出す。
模倣学習は学習データに無い状況での挙動を保証できない点は変わらないので、
止める根拠は planner の外にある層に任せる——STM32 の `auto_stop`
（GUI の設定がそのまま通る）、デッドマン、人の操作による解除、そして
`raspi/auto/cam_e2e.py` の速度上限。

## 前処理は `raspi/core/cam_e2e_preproc.py` だけ

学習（`ml_cam_e2e/`）と同じ関数を通す。色順（リングは BGR が既定）は
`ImageRef.fmt` を見てここで RGB に直す——`fmt` が分かるのはこのノードだけ。

## 契約: 推論が失敗した／モデル未選択のフレームは `ready=False`

`cam_perception_node.py` の契約2（欠測は壁扱い）と同じ考え方。

## publish するのは「推論を試みた周期」だけ

`Publisher.send()` は呼ぶたびに seq を打ち直すので、同じ中身を再送すると
planning_node には新しい入力に見えて `plan()` が回る。推論は `--infer-hz`
（既定10Hz）に間引いているので、publish もその周期に合わせる
（`cam_perception_node.py` と同じ。`stale_ms=500` に対して十分な鮮度）。

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

from raspi.core.auto_gate import IdlePacer, cam_infer_active  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION, preprocess, to_rgb  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import CamE2ECmd, ImageRef  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_CAM_E2E_CMD,
    TOPIC_CAM_E2E_MODEL,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_VEHICLE_STATE,
)

__all__ = ["MODEL_OUTPUTS", "RegressionModel", "load_model", "CamE2ENode"]

REPO_ROOT = Path(__file__).resolve().parents[2]
#: セグメンテーション用（`models/` 直下）とは分ける。telemetry_node の
#: `_cam_models_list` は直下の .onnx を全部セグメンテーション用として並べるので、
#: 同じ場所に置くと `ftg_cam` の選択肢に出てしまう（`models/e2e_lidar/` と同じ理由）
DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "cam_e2e"

NS = 1_000_000_000
HB_HZ = 10
#: 推論（CNN）を必要とする自動運転モード
_CAM_E2E_MODES = ("cam_e2e",)
#: モデル出力の並び。同梱 JSON の `outputs` と照合する（`ml_cam_e2e/export_onnx.py` が書く）
MODEL_OUTPUTS = ("steer_norm", "speed_norm")


class RegressionModel:
    """ONNX 推論の薄いラッパ。出力は `(steer_norm, speed_norm)` の2個。

    前処理契約（入力解像度・平均/分散）と出力契約（`max_steer`・`speed_ref`）は
    モデルに同梱する（`ml_cam_e2e/export_onnx.py` が `<name>.json` に書く）。
    `raspi/nodes/cam_perception_node.py` の `SegmentationModel` と同じ設計。
    """

    def __init__(self, model_path: str, *, input_size: tuple[int, int] = (224, 128),
                mean: float = 0.0, std: float = 255.0, max_steer: float = 0.0,
                speed_ref: float = 0.0) -> None:
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
        self.speed_ref = speed_ref

    def infer(self, rgb: np.ndarray) -> tuple[float, float]:
        """**RGB** の `(H, W, 3)` uint8 → `(steer_norm, speed_norm)`。

        色順を直すのは呼び出し側の責任（`to_rgb()`）。ここで `fmt` を知らずに
        推測すると、学習側（JPEG をデコードした RGB）とまた食い違う。
        """
        x = preprocess(rgb, self.input_size, self.mean, self.std)
        out = np.asarray(self.session.run(None, {self.input_name: x})[0]).reshape(-1)
        if out.size != len(MODEL_OUTPUTS):
            raise ValueError(f"モデル出力が{out.size}個（{len(MODEL_OUTPUTS)}個のはず）")
        return float(out[0]), float(out[1])


def load_model(onnx_path: Path) -> RegressionModel:
    """`<name>.onnx` と同梱の `<name>.json` から `RegressionModel` を作る。

    **契約が合わないモデルは読まない。** 出力が2個でない（操舵だけの旧モデル、
    セグメンテーション用を置き間違えた等）、前処理の版が違う、物理量へ戻す
    基準が無い——どれも「読めてしまうが走りがおかしい」になるので例外にする。
    `ml_cam_e2e/eval.py` も同じ関数で読む（実車と同じ経路で評価するため）。
    """
    if not onnx_path.exists():
        raise FileNotFoundError(f"モデルが見つかりません: {onnx_path}")
    cfg_path = onnx_path.with_suffix(".json")
    if not cfg_path.exists():
        raise FileNotFoundError(f"契約ファイルがありません: {cfg_path}")
    cfg = json.loads(cfg_path.read_text())
    if tuple(cfg.get("outputs", ())) != MODEL_OUTPUTS:
        raise ValueError(f"outputs が {list(MODEL_OUTPUTS)} ではない: {cfg.get('outputs')}")
    if cfg.get("preproc_version") != PREPROC_VERSION:
        raise ValueError(f"前処理の版が違う（モデル {cfg.get('preproc_version')}・"
                         f"ノード {PREPROC_VERSION}）。再エクスポートが必要")
    max_steer = float(cfg.get("max_steer", 0.0))
    speed_ref = float(cfg.get("speed_ref", 0.0))
    if max_steer <= 0 or speed_ref <= 0:
        raise ValueError(f"max_steer/speed_ref が正でない: {max_steer}/{speed_ref}")
    w, h = cfg["input_size"]
    return RegressionModel(str(onnx_path), input_size=(int(w), int(h)),
                           mean=float(cfg.get("mean", 0.0)),
                           std=float(cfg.get("std", 255.0)),
                           max_steer=max_steer, speed_ref=speed_ref)


class CamE2ENode:
    """1台の前方カメラ → 操舵と速度の直接回帰。

    **`process_frame()` はバス・共有メモリを一切知らない純粋関数。**
    `run()`（実バス配線）とテストの両方がここを通る
    （`cam_perception_node.py` と同じ設計方針）。
    """

    def __init__(self, *, model: RegressionModel | None = None,
                models_dir: Path | None = None, loaded_model_name: str = "",
                vehicle: Vehicle | None = None, infer_hz: float = 10.0) -> None:
        #: **`None` は「まだモデルが選ばれていない」。** `run()` はこの間 `ready=False` を出し続ける
        self.model = model
        self.models_dir = models_dir or DEFAULT_MODELS_DIR
        self._loaded_model_name = loaded_model_name
        self.vehicle = vehicle or Vehicle.load()

        # ★省電力化: `cam_perception_node.py` と同じ理由で推論を間引く
        self._infer_period_ns = int(NS / infer_hz) if infer_hz > 0 else 0
        self._last_attempt_ns = 0
        self._running = False
        self._active = False
        self._reader = FrameReader()

    def close(self) -> None:
        self._reader.close()

    # ── モデルの切替 ──

    def reload_if_changed(self, desired_name: str) -> bool:
        """`cam_perception_node.CamPerceptionNode.reload_if_changed()` と同じ契約
        （失敗しても今のモデルを保持する）。"""
        if not desired_name or desired_name == self._loaded_model_name:
            return False
        try:
            self.model = load_model(self.models_dir / f"{desired_name}.onnx")
            self._loaded_model_name = desired_name
            print(f"# モデル切替: {desired_name}", flush=True)
            return True
        except Exception as e:                                # noqa: BLE001
            print(f"# モデル読み込み失敗（{desired_name}）: {e}。"
                 f"前のモデルのまま続行", file=sys.stderr, flush=True)
            return False

    # ── 1周期ぶんの処理（純粋関数） ──

    def process_frame(self, frame: np.ndarray, fmt: str = "RGB888") -> tuple[float, float]:
        """1枚の生フレーム → `(steer_norm, speed_norm)`。`self.model` が必須（呼び出し側が保証）。"""
        return self.model.infer(to_rgb(frame, fmt))

    def failed_cmd(self, *, seq: int = 0, t_capture_ns: int = 0) -> CamE2ECmd:
        """フレームが読めない／モデル未選択の周期。**`ready=False`。**

        `t_capture` は省略時「いま」にする。0 のままだと planning_node の鮮度判定
        （`_current()`）が先に「入力が古い」で止めてしまい、planner の
        「モデル未選択」という本当の理由が GUI に出ない。
        """
        return CamE2ECmd(ready=False, seq=seq,
                         t_capture=t_capture_ns or time.monotonic_ns())

    # ── 共有メモリの読み取り ──

    def read_frame(self, ref: ImageRef) -> tuple[np.ndarray, int] | None:
        return self._reader.read(ref)

    def _attempt(self, ref: ImageRef | None, seq: int) -> CamE2ECmd:
        """推論を1回試みる。どこで失敗しても `ready=False` の `CamE2ECmd` を返す。"""
        if self.model is None or ref is None:
            return self.failed_cmd(seq=seq)
        got = self.read_frame(ref)
        if got is None:
            return self.failed_cmd(seq=seq)
        frame, t_capture = got
        try:
            steer_norm, speed_norm = self.process_frame(frame, ref.fmt)
        except Exception as e:
            # 推論側のバグでノード全体を巻き込んで落とさない。
            # 契約の「ready=False」に自然に落とす
            # （`planning_node._replan()` と同じパターン）
            print(f"# cam_e2e process_frame() が例外: {e}", file=sys.stderr, flush=True)
            return self.failed_cmd(seq=seq, t_capture_ns=t_capture)
        return CamE2ECmd(ready=True, steer_norm=steer_norm, speed_norm=speed_norm,
                         model_max_steer=self.model.max_steer,
                         model_speed_ref=self.model.speed_ref,
                         seq=seq, t_capture=t_capture)

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
            # IDLE中は約2Hzに間引く（`IdlePacer`。省電力、2026-10-04）。
            # ACTIVE中は常に True が返るので、下の推論周期で間引く
            idle_publish = pacer.should_publish(active, now)
            cmd: CamE2ECmd | None = None
            if not active:
                self._last_attempt_ns = 0          # ACTIVE に戻った瞬間にすぐ推論する
                if idle_publish:
                    cmd = self.failed_cmd(seq=seq)
            elif now - self._last_attempt_ns >= self._infer_period_ns:
                # 成功・失敗どちらでも1周期に1回だけ publish する
                # （モジュールdocstring「publish するのは推論を試みた周期だけ」）
                self._last_attempt_ns = now
                cmd = self._attempt(sub.latest.get(TOPIC_IMAGE_FRONT), seq)
            if cmd is not None:
                pub.send(TOPIC_CAM_E2E_CMD, cmd)
                seq += 1
                if status_cb:
                    status_cb(cmd)

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "cam_e2e", HbMsg(node="cam_e2e"))
            pacer.idle_sleep(active)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=None,
                    help="起動時の既定モデル（ONNXパス。同名の .json が必要。省略時は "
                         "cam_e2e/model トピック経由のGUI選択を待つ）")
    ap.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    ap.add_argument("--infer-hz", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    model = load_model(Path(args.model)) if args.model else None
    node = CamE2ENode(model=model, models_dir=Path(args.models_dir),
                      vehicle=Vehicle.load(), infer_hz=args.infer_hz)

    pub = Publisher("cam_e2e")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_VEHICLE_STATE: LATEST,
                      TOPIC_CAM_E2E_MODEL: LATEST, TOPIC_AUTO_CTRL: LATEST})

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
