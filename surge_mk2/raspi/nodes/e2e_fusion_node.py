"""e2e_fusion_node — LiDAR＋前方カメラ＋車両状態から、舵・速度・前方の経路点を推論する。

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
点群の先頭セクタの時刻＋`_SCAN_AGE_NO_IMAGE_S`。点群が途絶えた・両方無いときは `ready=False`。

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
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.base import scan_window  # noqa: E402
from raspi.bus import Policy  # noqa: E402
from raspi.core.auto_gate import IdlePacer, cam_infer_active  # noqa: E402
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_CROP, preprocess, to_rgb  # noqa: E402
from raspi.core.control_params import steer_relink  # noqa: E402
from raspi.core.e2e_fusion_preproc import (  # noqa: E402
    INPUT_NAMES,
    PREPROC_VERSION,
    FusionOutput,
    decode_output,
    lidar_input,
    lidar_rows,
    output_size,
    state_input,
)
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import E2EFusionPlan, Scan, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_E2E_FUSION_MODEL,
    TOPIC_E2E_FUSION_PLAN,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_SCAN,
    TOPIC_VEHICLE_STATE,
)

__all__ = ["FusionOnnx", "load_model", "pick_model_name", "E2EFusionNode", "MODES"]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "e2e_fusion"
MODEL_CONF = REPO_ROOT / "config" / "e2e_fusion_model.json"

NS = 1_000_000_000
HB_HZ = 10
#: 推論を必要とする自動運転モード
MODES = ("e2e_fusion",)
#: 画像が無い周の基準時刻を、点群の先頭セクタの時刻からどれだけ後に置くか [s]（学習の標本の点群の古さの中央）
_SCAN_AGE_NO_IMAGE_S = 0.12
#: LD06 の1周 [s]
_SCAN_PERIOD_S = 0.1
#: 最新の点群がこれより古ければ推論しない [s]
_SCAN_STALE_S = 0.3
#: 推論していない間、モデルの選び直しを見る間隔 [s]
_MODEL_CHECK_S = 2.0
#: 点群は1周も落とさず受ける（履歴に使う）。キューは数周ぶん
SCAN_POLICY = Policy(conflate=False, hwm=8)


class FusionOnnx:
    """ONNX 推論の薄いラッパ。契約（入出力の形と正規化）はモデル同梱の JSON から読む。"""

    def __init__(self, model_path: str, cfg: dict) -> None:
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1
        opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
        opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
        self.session = ort.InferenceSession(model_path, sess_options=opts,
                                            providers=["CPUExecutionProvider"])
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
        link = cfg.get("steer_link")
        self.steer_link = (float(link[0]), float(link[1])) if link else None
        self._zero_image = np.zeros((1, 3, self.input_size[1], self.input_size[0]), dtype=np.float32)
        self._zero_lidar = np.zeros((1, self.history + 1, self.fov_deg + 1), dtype=np.float32)

    def image_input(self, rgb: np.ndarray) -> np.ndarray:
        return preprocess(rgb, self.input_size, 0.0, 255.0, self.crop_x)

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
        return decode_output(out, k=self.k, max_steer=self.max_steer, speed_ref=self.speed_ref,
                             xy_scale=self.xy_scale)


def load_model(onnx_path: Path) -> FusionOnnx:
    """`<name>.onnx` と同梱の `<name>.json` から読む。**契約が合わないモデルは読まない**（例外）。"""
    if not onnx_path.exists():
        raise FileNotFoundError(f"モデルが見つかりません: {onnx_path}")
    cfg_path = onnx_path.with_suffix(".json")
    if not cfg_path.exists():
        raise FileNotFoundError(f"契約ファイルがありません: {cfg_path}")
    cfg = json.loads(cfg_path.read_text())
    if cfg.get("preproc_version") != PREPROC_VERSION:
        raise ValueError(f"入出力の版が違う（モデル {cfg.get('preproc_version')}・ノード {PREPROC_VERSION}）。"
                         "再エクスポートが必要")
    if cfg.get("image_preproc_version") != PREPROC_VERSION_CROP:
        raise ValueError(f"画像の前処理の版が違う（モデル {cfg.get('image_preproc_version')}・"
                         f"ノード {PREPROC_VERSION_CROP}）")
    for key in ("input_size", "crop_x", "fov_deg", "max_range", "history", "k", "ds", "xy_scale",
                "max_steer", "speed_ref", "yaw_rate_ref"):
        if key not in cfg:
            raise ValueError(f"契約に {key} が無い")
    if not 0.0 < float(cfg["crop_x"]) < 0.3:
        raise ValueError(f"crop_x が不正: {cfg['crop_x']}")
    for key in ("max_steer", "speed_ref", "yaw_rate_ref", "max_range", "ds", "xy_scale"):
        if not float(cfg[key]) > 0:
            raise ValueError(f"{key} が正でない: {cfg[key]}")
    model = FusionOnnx(str(onnx_path), cfg)
    out_dim = model.session.get_outputs()[0].shape[-1]
    if isinstance(out_dim, int) and out_dim != output_size(model.k):
        raise ValueError(f"出力の長さ {out_dim} が契約（K={model.k}）の {output_size(model.k)} と合わない")
    return model


def pick_model_name(models_dir: Path, conf_path: Path = MODEL_CONF) -> str:
    """使うモデルの名前。設定ファイルの指定、無ければいちばん新しい `.onnx`。無ければ ""。"""
    try:
        name = str(json.loads(conf_path.read_text()).get("name", ""))
        if name and (models_dir / f"{name}.onnx").is_file():
            return name
    except (OSError, ValueError, AttributeError):
        pass
    try:
        files = [p for p in models_dir.glob("*.onnx") if p.with_suffix(".json").is_file()]
    except OSError:
        return ""
    if not files:
        return ""
    return max(files, key=lambda p: p.stat().st_mtime_ns).stem


class E2EFusionNode:
    """**`infer_once()` はバス・共有メモリを知らない純粋な計算。** `run()`（実バス配線）とテストの両方がここを通る。"""

    def __init__(self, *, model: FusionOnnx | None = None, models_dir: Path | None = None,
                 model_name: str = "", vehicle: Vehicle | None = None) -> None:
        self.model = model
        self.models_dir = models_dir or DEFAULT_MODELS_DIR
        self.model_name = model_name
        self._failed_key: tuple | None = None
        self.vehicle = vehicle or Vehicle.load()
        #: 新しい順の `(先頭セクタの時刻, lidar_rows)`
        self._rows: deque[tuple[int, np.ndarray]] = deque(maxlen=4)
        self._last_scan_seq = -1
        self._running = False
        self._active = False
        self._reader = FrameReader()

    def close(self) -> None:
        self._reader.close()

    # ── モデル ──

    def reload_if_changed(self, name: str) -> bool:
        """`name` のモデルへ切り替える。失敗しても今のモデルを保持し、同じファイルは再試行しない。"""
        if not name or name == self.model_name:
            return False
        path = self.models_dir / f"{name}.onnx"

        def mtime(p: Path):
            try:
                return p.stat().st_mtime_ns
            except OSError:
                return None

        key = (name, mtime(path), mtime(path.with_suffix(".json")))
        if key == self._failed_key:
            return False
        try:
            self.model = load_model(path)
            self.model_name = name
            self._failed_key = None
            self._rows.clear()                 # 視野・距離の上限がモデルごとに違う
            print(f"# モデル切替: {name}", flush=True)
            return True
        except Exception as e:                                # noqa: BLE001
            self._failed_key = key
            print(f"# モデル読み込み失敗（{name}）: {e}。前のモデルのまま続行", file=sys.stderr, flush=True)
            return False

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
                   now_ns: int, seq: int = 0) -> E2EFusionPlan:
        """今の履歴・画像・車両状態から1回推論する。どこで失敗しても `ready=False` と理由を返す。"""
        if self.model is None:
            return self.failed("モデルが無い（models/e2e_fusion/ に .onnx と .json を置く）", seq=seq)
        t0 = time.perf_counter()
        got = self._lidar(now_ns)
        if got is None and rgb is None:
            return self.failed("点群も画像も届いていない", seq=seq)
        lidar, t_scan = got if got is not None else (None, 0)
        try:
            image = self.model.image_input(rgb) if rgb is not None else None
            state = (self.model.state_input(vs.speed, vs.steer_actual, vs.yaw_rate)
                     if vs is not None else None)
            out = self.model.infer(image, lidar, state)
        except Exception as e:                                # noqa: BLE001
            print(f"# e2e_fusion の推論が例外: {e}", file=sys.stderr, flush=True)
            return self.failed(f"推論に失敗: {e}", seq=seq)
        steer = out.steer
        link = self.model.steer_link
        if link is not None:
            # 学習したときの換算の舵角 → 今の車の換算の舵角（同じモータ角になる値）
            steer = steer_relink(steer, link, self.vehicle.steer_link)
        t_ref = t_image_ns if rgb is not None else t_scan + int(_SCAN_AGE_NO_IMAGE_S * NS)
        return E2EFusionPlan(
            ready=True, steer=steer, speed=out.speed,
            path_x=out.path_xy[:, 0].tolist(), path_y=out.path_xy[:, 1].tolist(),
            path_v=out.path_v.tolist(), path_b=out.path_b.tolist(), ds=self.model.ds,
            has_image=rgb is not None, has_lidar=lidar is not None, model=self.model_name,
            infer_ms=(time.perf_counter() - t0) * 1000.0, seq=seq, t_capture=t_ref)

    def failed(self, reason: str, *, seq: int = 0) -> E2EFusionPlan:
        """`ready=False`。`t_capture` は「いま」（0 だと planning_node の鮮度判定が先に止め、理由が出ない）。"""
        return E2EFusionPlan(ready=False, reason=reason, model=self.model_name, seq=seq,
                             t_capture=time.monotonic_ns())

    # ── ループ（実バス配線） ──

    def stop(self) -> None:
        self._running = False

    def run(self, *, sub, pub, duration_s: float | None = None, status_cb=None,
            fixed_model: bool = False) -> None:
        self._running = True
        seq = 0
        t_end = time.monotonic() + duration_s if duration_s else None
        next_hb = time.monotonic_ns()
        next_model_check = 0.0
        pacer = IdlePacer()
        while self._running:
            if t_end and time.monotonic() >= t_end:
                break
            new_scans: list[Scan] = []
            for topic, msg in sub.poll(pacer.poll_timeout_ms(self._active)):
                if topic == TOPIC_SCAN and msg.seq != self._last_scan_seq:
                    self._last_scan_seq = msg.seq
                    new_scans.append(msg)

            auto_ctrl = sub.latest.get(TOPIC_AUTO_CTRL)
            vs = sub.latest.get(TOPIC_VEHICLE_STATE)
            # GUI で選んだモデル（空の名前は「自動」＝下の、推論していない間の選び直しに任せる）
            wanted = sub.latest.get(TOPIC_E2E_FUSION_MODEL)
            chosen = wanted.name if wanted is not None and not fixed_model else ""
            if chosen:
                self.reload_if_changed(chosen)
            active = cam_infer_active(auto_ctrl, vs, MODES)
            if active != self._active:
                self._active = active
                print(f"# e2e_fusion {'推論開始' if active else '推論停止'}", flush=True)
                if not active:
                    self._rows.clear()

            now = time.monotonic_ns()
            plan: E2EFusionPlan | None = None
            if not active:
                # 走っていない間だけモデルを選び直す
                if not fixed_model and not chosen and time.monotonic() >= next_model_check:
                    next_model_check = time.monotonic() + _MODEL_CHECK_S
                    # GUI から「自動」と言われていれば最新、何も届いていなければ設定ファイル → 最新
                    conf = self.models_dir / "__none__.json" if wanted is not None else MODEL_CONF
                    self.reload_if_changed(pick_model_name(self.models_dir, conf))
                if pacer.should_publish(active, now):
                    plan = self.failed("このモードが選ばれていない、または DISARM 中", seq=seq)
            elif new_scans:
                for sc in new_scans:
                    self.push_scan(sc)
                rgb, t_img = None, 0
                ref = sub.latest.get(TOPIC_IMAGE_FRONT)
                got = self._reader.read(ref) if ref is not None else None
                if got is not None:
                    frame, t_img = got
                    rgb = to_rgb(frame, ref.fmt)
                plan = self.infer_once(rgb=rgb, t_image_ns=t_img, vs=vs, now_ns=now, seq=seq)
            if plan is not None:
                pub.send(TOPIC_E2E_FUSION_PLAN, plan)
                seq += 1
                if status_cb:
                    status_cb(plan)

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "e2e_fusion", HbMsg(node="e2e_fusion"))
            pacer.idle_sleep(active)


def main() -> int:
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
        node.reload_if_changed(pick_model_name(node.models_dir))

    pub = Publisher("e2e_fusion")
    sub = Subscriber({TOPIC_SCAN: SCAN_POLICY, TOPIC_IMAGE_FRONT: LATEST, TOPIC_VEHICLE_STATE: LATEST,
                      TOPIC_AUTO_CTRL: LATEST, TOPIC_E2E_FUSION_MODEL: LATEST})
    print(f"# e2e_fusion_node  publish {pub.endpoint}  モデル: {node.model_name or '（なし）'}", flush=True)

    def _shutdown(*_):
        node.stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _shutdown)
    try:
        node.run(sub=sub, pub=pub, duration_s=args.duration, fixed_model=bool(args.model))
    finally:
        node.close()
        sub.close()
        pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
