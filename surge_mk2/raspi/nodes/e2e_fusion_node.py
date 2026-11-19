"""e2e_fusion_node — LiDAR＋前方カメラ＋車両状態から、前方の経路点を推論する。

    .venv/bin/python -m raspi.nodes.e2e_fusion_node
    .venv/bin/python -m raspi.nodes.e2e_fusion_node --model models/e2e_fusion/v1.onnx

- **購読**: `scan`（全周）・`image/front`・`vehicle_state`・`auto/ctrl`
- **発行**: `e2e_fusion/plan`（`E2EFusionPlan`）・`hb/e2e_fusion`

学習は Mac の `ml_e2e/`（実機の記録から模倣学習 → ONNX）。入出力の形は `raspi/core/e2e_fusion_preproc.py` が正で、
学習と同じ関数を通る。走らせる側は `raspi/auto/e2e_fusion.py`（経路点を推測航法でずらして追う）。

## 推論のきっかけと時刻

**新しい点群の1周が届いたら推論する**（10Hz）。画像はその時点の最新のコマ。出力の基準時刻（`t_capture`）は
画像の撮影時刻で、点群はそれより 0.07〜0.17s 古い（学習の標本と同じ関係。`ml_e2e/extract.py`）。
画像が読めない周は点群だけで推論し（`avail` の画像を 0。学習でも一定割合で伏せている）、基準時刻は
点群の先頭セクタの時刻＋「画像があった周の、画像と点群の時刻の差」の移動平均（初めは学習の標本の中央
`_SCAN_AGE_NO_IMAGE_S`。固定値だと実車の配信の遅れの変化に追いつかない。レビュー L13）。
点群が途絶えた・両方無いときは `ready=False`。

**点群は `scan` をそのまま使う**（planning_node が planner に渡す前に掛ける車体マスク・ノイズ除去は通さない。
学習の標本が記録の `scan` そのままだから）。読み方（欠測・飽和・測距不能）は `raspi.auto.base.scan_window`。

## モデルの選び方

`models/e2e_fusion/<名前>.onnx` と同名の `.json`（契約。`ml_e2e/export_onnx.py` が書く）。使うモデルは
**GUI の自動運転タブで選ぶ**（telemetry_node が `e2e_fusion/model` で名前を流し続け、`config/e2e_fusion_model.json`
に残す）。GUI で「自動」（空の名前）なら**いちばん新しい `.onnx`**。telemetry_node が居ないときは、同じ設定ファイルを
自分で読む。

- GUI で選んだ名前へは、届いたらすぐ切り替える（telemetry_node が同時に engage を落とす）
- 「自動」のときにいちばん新しいモデルへ切り替えるのは、推論していない間だけ（走行中に、配られたばかりの
  モデルへ勝手に替わらない）

## 該当モードが選ばれている間だけ推論する

`cam_e2e_node.py` と同じ節電方針（`raspi/core/auto_gate.cam_infer_active`）。`auto/ctrl` の `mode` が
`e2e_fusion` で DISARM 中でない間だけ推論し、それ以外は約 2Hz で `ready=False` を出す。

## 画像は左右2台（画像の前処理の版3）

契約の `image_preproc_version` は 3（`cam_e2e_preproc.PREPROC_VERSION_STEREO`）・`cams` は `["front", "left"]` だけを読む。
右（`image/front`）の最新のコマと、左（`image/left`）のリングの同じ瞬間（`STEREO_PAIR_TOL_NS` 以内）の
コマを組にして画像に入れる。**組が揃わない周は画像を伏せて**（`avail` の画像を 0）点群と車両状態だけで推論する
（片目では推論しない。学習でも画像は左右まとめて伏せる）。
"""

from __future__ import annotations

import argparse
import time
from collections import deque
from pathlib import Path


import numpy as np

from raspi.auto.base import scan_window
from raspi.bus import Policy
from raspi.core.cam_e2e_preproc import (
    STEREO_PAIR_TOL_NS,
    preprocess_stereo,
    to_rgb,
)
from raspi.core.e2e_fusion_preproc import (
    INPUT_NAMES,
    PREPROC_VERSION,
    FusionOutput,
    decode_output,
    lidar_input,
    lidar_rows,
    output_size,
    state_input,
)
from raspi.core.onnx_session import make_session
from raspi.nodes.e2e_base import E2ENodeBase, check_contract
from raspi.nodes.e2e_base import pick_model_names as _pick_model_names
from raspi.core.vehicle import Vehicle
from raspi.msgs import E2EFusionPlan, Scan, VehicleState
from raspi.msgs.types import (
    TOPIC_AUTO_CTRL,
    TOPIC_E2E_FUSION_MODEL,
    TOPIC_E2E_FUSION_PLAN,
    TOPIC_IMAGE_FRONT,
    TOPIC_IMAGE_LEFT,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
)
from raspi.core.log import get_logger, setup_logging
from raspi.core.runner import run_node

_log = get_logger("e2e_fusion")

__all__ = ["FusionOnnx", "load_model", "pick_model_name", "pick_model_names", "E2EFusionNode", "MODES"]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "e2e_fusion"
MODEL_CONF = REPO_ROOT / "config" / "e2e_fusion_model.json"

NS = 1_000_000_000
#: 推論を必要とする自動運転モード
MODES = ("e2e_fusion",)
#: 画像が無い周の基準時刻を、点群の先頭セクタの時刻からどれだけ後に置くか [s]（学習の標本の点群の古さの中央）
_SCAN_AGE_NO_IMAGE_S = 0.12
#: 画像と点群の時刻の差の移動平均の重み（10Hz で約 1 秒）と、採る範囲 [s]（外れた組は使わない）
_SCAN_AGE_EMA = 0.1
_SCAN_AGE_RANGE_S = (0.03, 0.3)
#: LD06 の1周 [s]
_SCAN_PERIOD_S = 0.1
#: 最新の点群がこれより古ければ推論しない [s]
_SCAN_STALE_S = 0.3
#: 点群は1周も落とさず受ける（履歴に使う）。キューは数周ぶん
SCAN_POLICY = Policy(conflate=False, hwm=8)


class FusionOnnx:
    """ONNX 推論の薄いラッパ。契約（入出力の形と正規化）はモデル同梱の JSON から読む。"""

    def __init__(self, model_path: str, cfg: dict) -> None:
        self.session = make_session(model_path)
        names = tuple(i.name for i in self.session.get_inputs())
        if names != INPUT_NAMES:
            raise ValueError(f"入力名が {list(INPUT_NAMES)} ではない: {list(names)}")
        w, h = cfg["input_size"]
        self.input_size = (int(w), int(h))
        self.crop_x = float(cfg["crop_x"])
        self.fov_deg = int(cfg["fov_deg"])
        self.max_range = float(cfg["max_range"])
        self.history = int(cfg["history"])
        self.k = int(cfg["k"])
        self.ds = float(cfg["ds"])
        self.xy_scale = float(cfg["xy_scale"])
        self.max_steer = float(cfg["max_steer"])
        self.speed_ref = float(cfg["speed_ref"])
        self.yaw_rate_ref = float(cfg["yaw_rate_ref"])
        # 画像は `[右, 左]` の 6ch（モジュールdocstring）
        self._zero_image = np.zeros((1, 6, self.input_size[1], self.input_size[0]), dtype=np.float32)
        self._zero_lidar = np.zeros((1, self.history + 1, self.fov_deg + 1), dtype=np.float32)

    def image_input(self, rgb: np.ndarray, rgb_left: np.ndarray | None) -> np.ndarray | None:
        """画像の入力。左が無ければ `None`（画像を伏せる。片目では推論しない）。"""
        if rgb_left is None:
            return None
        return preprocess_stereo(rgb, rgb_left, self.input_size, 0.0, 255.0, self.crop_x)

    def state_input(self, speed: float, steer: float, yaw_rate: float) -> np.ndarray:
        return state_input(speed, steer, yaw_rate, speed_ref=self.speed_ref, max_steer=self.max_steer,
                           yaw_rate_ref=self.yaw_rate_ref)

    def infer(self, image: np.ndarray | None, lidar: np.ndarray | None,
              state: np.ndarray | None) -> FusionOutput:
        """前処理済みの入力（無い入力は `None`）→ 物理量の出力。"""
        avail = np.array([[image is not None, lidar is not None, state is not None]], dtype=np.float32)
        feed = {
            "image": image if image is not None else self._zero_image,
            "lidar": lidar if lidar is not None else self._zero_lidar,
            "state": state if state is not None else np.zeros((1, 3), dtype=np.float32),
            "avail": avail,
        }
        out = np.asarray(self.session.run(None, feed)[0]).reshape(-1)
        return decode_output(out, k=self.k, speed_ref=self.speed_ref, xy_scale=self.xy_scale)


def load_model(onnx_path: Path) -> FusionOnnx:
    """`<name>.onnx` と同梱の `<name>.json` から読む。**契約が合わないモデルは読まない**（例外）。"""
    cfg = check_contract(
        onnx_path, preproc_version=PREPROC_VERSION, export_hint="ml_e2e.export_onnx",
        required=("input_size", "crop_x", "fov_deg", "max_range", "history", "k", "ds", "xy_scale",
                  "max_steer", "speed_ref", "yaw_rate_ref"),
        positive=("max_steer", "speed_ref", "yaw_rate_ref", "max_range", "ds", "xy_scale"))
    model = FusionOnnx(str(onnx_path), cfg)
    out_dim = model.session.get_outputs()[0].shape[-1]
    if isinstance(out_dim, int) and out_dim != output_size(model.k):
        raise ValueError(f"出力の長さ {out_dim} が契約（K={model.k}）の {output_size(model.k)} と合わない")
    return model


def pick_model_names(models_dir: Path, conf_path: Path = MODEL_CONF) -> list[str]:
    """使うモデルの候補を優先順に（`e2e_base.pick_model_names`。既定の設定ファイルはこのノードの）。"""
    return _pick_model_names(models_dir, conf_path)


def pick_model_name(models_dir: Path, conf_path: Path = MODEL_CONF) -> str:
    """使うモデルの名前。設定ファイルの指定、無ければいちばん新しい `.onnx`。無ければ ""。"""
    names = pick_model_names(models_dir, conf_path)
    return names[0] if names else ""


class E2EFusionNode(E2ENodeBase[FusionOnnx]):
    """**`infer_once()` はバス・共有メモリを知らない純粋な計算。** `run()`（`E2ENodeBase`）とテストの両方がここを通る。"""

    NAME = "e2e_fusion"
    MODES = MODES
    MODEL_TOPIC = TOPIC_E2E_FUSION_MODEL
    PLAN_TOPIC = TOPIC_E2E_FUSION_PLAN
    MODEL_CONF = MODEL_CONF

    def __init__(self, *, model: FusionOnnx | None = None, models_dir: Path | None = None,
                 model_name: str = "", vehicle: Vehicle | None = None) -> None:
        super().__init__(load_model=lambda p: load_model(p), models_dir=models_dir or DEFAULT_MODELS_DIR,
                         model=model, model_name=model_name)
        self.vehicle = vehicle or Vehicle.load()
        #: 新しい順の `(先頭セクタの時刻, lidar_rows)`
        self._rows: deque[tuple[int, np.ndarray]] = deque(maxlen=4)
        self._last_scan_seq = -1
        self._new_scans: list[Scan] = []
        #: 画像の撮影時刻 − 点群の先頭セクタの時刻 [s] の移動平均（画像が無い周の基準時刻に使う）
        self._scan_age_s = _SCAN_AGE_NO_IMAGE_S

    def on_model_changed(self) -> None:
        self._rows.clear()                     # 視野・距離の上限がモデルごとに違う

    def on_active_changed(self, active: bool) -> None:
        if not active:
            self._rows.clear()

    def on_poll(self, msgs: list) -> None:
        for topic, msg in msgs:
            if topic == TOPIC_SCAN and msg.seq != self._last_scan_seq:
                self._last_scan_seq = msg.seq
                self._new_scans.append(msg)

    # ── 入力 ──

    def push_scan(self, scan: Scan) -> None:
        """新しい点群の1周を履歴に積む（モデルが無ければ何もしない）。"""
        if self.model is None:
            return
        w = scan_window(scan, self.model.fov_deg, self.model.max_range)
        self._rows.appendleft((int(scan.t_capture), lidar_rows(w.dist, w.measured, self.model.max_range)))

    def _lidar(self, now_ns: int) -> tuple[np.ndarray, int] | None:
        """履歴 → モデルの入力と、最新の周の先頭セクタの時刻。最新が古ければ `None`。"""
        if not self._rows or now_ns - self._rows[0][0] > _SCAN_STALE_S * NS:
            return None
        t0 = self._rows[0][0]
        # `h` 周前の点群は、最新から `(h + 0.5)` 周期以内のものだけ使う（学習の標本と同じ規則。
        # 抜けた周は `lidar_input` がそれより新しい周で埋める）
        rows = [r if t0 - t <= (h + 0.5) * _SCAN_PERIOD_S * NS else None
                for h, (t, r) in enumerate(self._rows)]
        return lidar_input(rows, self.model.history), t0

    # ── 1回の推論（純粋な計算） ──

    def infer_once(self, *, rgb: np.ndarray | None, t_image_ns: int, vs: VehicleState | None,
                   now_ns: int, seq: int = 0, rgb_left: np.ndarray | None = None) -> E2EFusionPlan:
        """今の履歴・画像・車両状態から1回推論する。どこで失敗しても `ready=False` と理由を返す。
        `rgb` が右・`rgb_left` が左（どちらかが無ければ画像を伏せる）。"""
        if self.model is None:
            return self.failed("モデルが無い（models/e2e_fusion/ に .onnx と .json を置く）", seq=seq)
        if rgb_left is None:
            rgb = None
        t0 = time.perf_counter()
        got = self._lidar(now_ns)
        if got is None and rgb is None:
            return self.failed("点群も画像も届いていない", seq=seq)
        lidar, t_scan = got if got is not None else (None, 0)
        try:
            image = self.model.image_input(rgb, rgb_left) if rgb is not None else None
            state = (self.model.state_input(vs.speed, vs.steer_actual, vs.yaw_rate)
                     if vs is not None else None)
            out = self.model.infer(image, lidar, state)
        except Exception as e:                                # noqa: BLE001
            _log.info(f"e2e_fusion の推論が例外: {e}")
            return self.failed(f"推論に失敗: {e}", seq=seq)
        if rgb is not None and t_scan:
            age = (t_image_ns - t_scan) / NS
            if _SCAN_AGE_RANGE_S[0] <= age <= _SCAN_AGE_RANGE_S[1]:
                self._scan_age_s += _SCAN_AGE_EMA * (age - self._scan_age_s)
        t_ref = t_image_ns if rgb is not None else t_scan + int(self._scan_age_s * NS)
        return E2EFusionPlan(
            ready=True,
            path_x=out.path_xy[:, 0].tolist(), path_y=out.path_xy[:, 1].tolist(),
            path_v=out.path_v.tolist(), path_b=out.path_b.tolist(), ds=self.model.ds,
            has_image=rgb is not None, has_lidar=lidar is not None, model=self.model_name,
            infer_ms=(time.perf_counter() - t0) * 1000.0, seq=seq, t_capture=t_ref)

    def failed(self, reason: str, *, seq: int = 0) -> E2EFusionPlan:
        """`ready=False`。`t_capture` は「いま」（0 だと planning_node の鮮度判定が先に止め、理由が出ない）。"""
        return E2EFusionPlan(ready=False, reason=reason, model=self.model_name, seq=seq,
                             t_capture=time.monotonic_ns())

    # ── 推論中の1周期（`E2ENodeBase.run` から） ──

    def step(self, latest: dict, vs, now_ns: int, seq: int, last_pub_ns: int) -> E2EFusionPlan | None:
        """新しい点群の周ごとに1回推論する（点群が来ていなければ何も出さない）。"""
        new_scans, self._new_scans = self._new_scans, []
        if not new_scans:
            return None
        for sc in new_scans:
            self.push_scan(sc)
        rgb, rgb_left, t_img = None, None, 0
        ref = latest.get(TOPIC_IMAGE_FRONT)
        # 右も通知された瞬間のコマを読む（リングの最新は通知より新しいことがあり、その左はまだ無い）
        got = (self._reader.read_at(ref, ref.t_capture, STEREO_PAIR_TOL_NS)
               if ref is not None else None)
        if got is not None:
            frame, t_img = got
            rgb = to_rgb(frame, ref.fmt)
            left = latest.get(TOPIC_IMAGE_LEFT)
            if left is not None:
                got_l = self._reader_left.read_at(left, t_img, STEREO_PAIR_TOL_NS)
                if got_l is not None:
                    rgb_left = to_rgb(got_l[0], left.fmt)
        return self.infer_once(rgb=rgb, t_image_ns=t_img, vs=vs, now_ns=now_ns, seq=seq, rgb_left=rgb_left)


def main() -> int:
    setup_logging("e2e_fusion")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=None,
                    help="使うモデルを固定する（ONNX のパス。同名の .json が必要）。省略時は設定ファイルか最新")
    ap.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    node = E2EFusionNode(models_dir=Path(args.models_dir), vehicle=Vehicle.load())
    if args.model:
        node.model = load_model(Path(args.model))
        node.model_name = Path(args.model).stem
    else:
        node.reload_first_usable(pick_model_names(node.models_dir))

    pub = Publisher("e2e_fusion")
    sub = Subscriber({TOPIC_SCAN: SCAN_POLICY, TOPIC_IMAGE_FRONT: LATEST, TOPIC_IMAGE_LEFT: LATEST, TOPIC_VEHICLE_STATE: LATEST,
                      TOPIC_AUTO_CTRL: LATEST, TOPIC_E2E_FUSION_MODEL: LATEST})
    _log.info(f"e2e_fusion_node  publish {pub.endpoint}  モデル: {node.model_name or '（なし）'}")

    run_node(node, lambda: node.run(sub=sub, pub=pub, duration_s=args.duration, fixed_model=bool(args.model)), close=(sub, pub))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
