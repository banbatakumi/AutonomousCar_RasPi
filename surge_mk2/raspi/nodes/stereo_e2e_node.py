"""stereo_e2e_node — 前方ステレオの左右の画像＋視差の帯＋車両状態から、前方の経路点を推論する（LiDAR を使わない）。

    .venv/bin/python -m raspi.nodes.stereo_e2e_node
    .venv/bin/python -m raspi.nodes.stereo_e2e_node --model models/stereo_e2e/v1.onnx

- **購読**: `stereo/disp`（`stereo_node` が照合ごとに出す視差の帯）・`image/front`・`image/left`・`vehicle_state`・
  `auto/ctrl`・`stereo_e2e/model`
- **発行**: `stereo_e2e/plan`（型は `E2EFusionPlan`）・`hb/stereo_e2e`

学習は Mac の `ml_stereo_e2e/`（LiDAR の `slam2d_route` を先生にした記録から模倣学習 → ONNX）。入出力の形は
`raspi/core/stereo_e2e_preproc.py` が正。走らせる側は `raspi/auto/stereo_e2e.py`（`e2e_fusion` と同じく経路点を
推測航法でずらして追う。既定は MPC）。モデルの選び方・節電（このモードが選ばれて ARM 中だけ推論）は
`e2e_fusion_node.py` と同じ。

## 推論のきっかけと時刻

**`stereo/disp` が届いたら推論する**（照合は ARM 中 15Hz。推論は `--rate-hz` 既定 10Hz に間引く。
平均が 10Hz になるよう 3 組に 2 組を通すので、間隔は 67ms と 133ms が交互になる）。
画像は**その視差を作った組そのもの**（同じ `t_capture` のコマを左右のリングから読む）なので、画像と視差は必ず
同じ瞬間。出力の基準時刻（`t_capture`）もその撮影時刻。組が読めない周は画像を伏せて（`avail`）視差と車両状態だけで
推論する。視差の設定（帯・探索幅・縮めた大きさ・版）がモデルの契約と違えば推論しない。
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
from raspi.core.cam_e2e_preproc import (  # noqa: E402
    PREPROC_VERSION_STEREO,
    STEREO_CAMS,
    STEREO_PAIR_TOL_NS,
    preprocess_stereo,
    to_rgb,
)
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.stereo_e2e_preproc import (  # noqa: E402
    DISP_VERSION,
    INPUT_NAMES,
    PREPROC_VERSION,
    FusionOutput,
    crop_disp,
    decode_output,
    disp_from_bytes,
    disp_input,
    output_size,
    state_input,
)
from raspi.msgs import E2EFusionPlan, StereoDisp, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_AUTO_CTRL,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_IMAGE_LEFT,
    TOPIC_STEREO_DISP,
    TOPIC_STEREO_E2E_MODEL,
    TOPIC_STEREO_E2E_PLAN,
    TOPIC_VEHICLE_STATE,
)
from raspi.nodes.e2e_fusion_node import pick_model_name  # noqa: E402

__all__ = ["StereoOnnx", "load_model", "StereoE2ENode", "MODES", "DEFAULT_MODELS_DIR", "MODEL_CONF"]

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "stereo_e2e"
MODEL_CONF = REPO_ROOT / "config" / "stereo_e2e_model.json"

NS = 1_000_000_000
HB_HZ = 10
#: 推論を必要とする自動運転モード
MODES = ("stereo_e2e",)
#: 推論の上限頻度の既定 [Hz]
DEFAULT_RATE_HZ = 10.0
#: 間引きで「予定より早い」組を通す幅（周期に対する割合）。照合の時刻の揺れを吸う
_RATE_JITTER = 0.1
#: 視差がこれより古ければ推論しない [s]
_DISP_STALE_S = 0.3
#: 推論中に視差が来ないとき、`ready=False` を出す間隔 [s]
_NO_DISP_REPORT_S = 0.5
#: 推論していない間、モデルの選び直しを見る間隔 [s]
_MODEL_CHECK_S = 2.0
#: 契約に要るキー
_CONTRACT_KEYS = ("input_size", "crop_x", "disp_hw", "disp_crop_cols", "disp_num_disp", "disp_rows", "disp_scale", "disp_version",
                  "k", "ds", "xy_scale", "max_steer", "speed_ref", "yaw_rate_ref")


class StereoOnnx:
    """ONNX 推論の薄いラッパ。契約（入出力の形と正規化）はモデル同梱の JSON から読む。"""

    def __init__(self, model_path: str, cfg: dict) -> None:
        import onnxruntime as ort

        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.inter_op_num_threads = 1
        opts.add_session_config_entry("session.intra_op.allow_spinning", "0")
        opts.add_session_config_entry("session.inter_op.allow_spinning", "0")
        self.path = model_path
        self.session = ort.InferenceSession(model_path, sess_options=opts,
                                            providers=["CPUExecutionProvider"])
        names = tuple(i.name for i in self.session.get_inputs())
        if names != INPUT_NAMES:
            raise ValueError(f"入力名が {list(INPUT_NAMES)} ではない: {list(names)}")
        w, h = cfg["input_size"]
        self.input_size = (int(w), int(h))
        self.crop_x = float(cfg["crop_x"])
        self.disp_hw = (int(cfg["disp_hw"][0]), int(cfg["disp_hw"][1]))
        self.disp_num_disp = int(cfg["disp_num_disp"])
        self.disp_rows = [int(x) for x in cfg["disp_rows"]]
        self.disp_scale = float(cfg["disp_scale"])
        self.disp_version = int(cfg["disp_version"])
        #: 帯の両端で落とす列（学習の向きの拡張の余白。モデルに入るのは中央の窓だけ）
        self.disp_crop_cols = int(cfg["disp_crop_cols"])
        #: 照合の方式・窓・一意性（契約の JSON に無い古いモデルは照合しない）
        self.disp_algo = str(cfg.get("disp_algo", ""))
        self.disp_block = int(cfg.get("disp_block", 0))
        self.disp_uniqueness = int(cfg.get("disp_uniqueness", 0))
        self.k = int(cfg["k"])
        self.ds = float(cfg["ds"])
        self.xy_scale = float(cfg["xy_scale"])
        self.max_steer = float(cfg["max_steer"])
        self.speed_ref = float(cfg["speed_ref"])
        self.yaw_rate_ref = float(cfg["yaw_rate_ref"])
        self._zero_image = np.zeros((1, 6, self.input_size[1], self.input_size[0]), dtype=np.float32)
        self._zero_disp = np.zeros((1, 2, self.disp_hw[0], self.disp_hw[1] - 2 * self.disp_crop_cols),
                                   dtype=np.float32)

    def disp_mismatch(self, msg: StereoDisp) -> str:
        """`stereo/disp` の設定が契約と違えば理由（同じなら空文字）。"""
        if msg.preproc_version != self.disp_version:
            return f"視差の版 {msg.preproc_version}（モデル {self.disp_version}）"
        if (msg.h, msg.w) != self.disp_hw:
            return f"視差の大きさ {msg.h}×{msg.w}（モデル {self.disp_hw[0]}×{self.disp_hw[1]}）"
        if msg.num_disp != self.disp_num_disp or abs(msg.scale - self.disp_scale) > 1e-6:
            return f"探索幅・縮尺 {msg.num_disp}/{msg.scale:g}（モデル {self.disp_num_disp}/{self.disp_scale:g}）"
        if list(msg.rows) != self.disp_rows:
            return f"行の帯 {list(msg.rows)}（モデル {self.disp_rows}）"
        # 方式・窓・一意性（`stereo_node --algo bm` や `MatchParams` の既定の変更で入力の分布が黙って変わるのを止める）
        got = (msg.algo, msg.block, msg.uniqueness)
        want = (self.disp_algo, self.disp_block, self.disp_uniqueness)
        if self.disp_algo and got != want:
            return f"照合の方式・窓・一意性 {got[0] or '?'}/{got[1]}/{got[2]}（モデル {want[0]}/{want[1]}/{want[2]}）"
        return ""

    def image_input(self, rgb: np.ndarray, rgb_left: np.ndarray) -> np.ndarray:
        return preprocess_stereo(rgb, rgb_left, self.input_size, 0.0, 255.0, self.crop_x)

    def state_input(self, speed: float, steer: float, yaw_rate: float) -> np.ndarray:
        return state_input(speed, steer, yaw_rate, speed_ref=self.speed_ref, max_steer=self.max_steer,
                           yaw_rate_ref=self.yaw_rate_ref)

    def infer(self, image: np.ndarray | None, disp: np.ndarray | None,
              state: np.ndarray | None) -> FusionOutput:
        """前処理済みの入力（無い入力は `None`）→ 物理量の出力。"""
        avail = np.array([[image is not None, disp is not None, state is not None]], dtype=np.float32)
        feed = {
            "image": image if image is not None else self._zero_image,
            "disp": disp if disp is not None else self._zero_disp,
            "state": state if state is not None else np.zeros((1, 3), dtype=np.float32),
            "avail": avail,
        }
        out = np.asarray(self.session.run(None, feed)[0]).reshape(-1)
        return decode_output(out, k=self.k, speed_ref=self.speed_ref, xy_scale=self.xy_scale)


def load_model(onnx_path: Path) -> StereoOnnx:
    """`<name>.onnx` と同梱の `<name>.json` から読む。**契約が合わないモデルは読まない**（例外）。"""
    if not onnx_path.exists():
        raise FileNotFoundError(f"モデルが見つかりません: {onnx_path}")
    cfg_path = onnx_path.with_suffix(".json")
    if not cfg_path.exists():
        raise FileNotFoundError(f"契約ファイルがありません: {cfg_path}")
    cfg = json.loads(cfg_path.read_text())
    if cfg.get("preproc_version") != PREPROC_VERSION:
        raise ValueError(f"入出力の版が違う（モデル {cfg.get('preproc_version')}・ノード {PREPROC_VERSION}）。"
                         "`ml_stereo_e2e.export_onnx` で書き出し直す")
    if cfg.get("image_preproc_version") != PREPROC_VERSION_STEREO:
        raise ValueError(f"画像の前処理の版が違う（モデル {cfg.get('image_preproc_version')}・"
                         f"ノード {PREPROC_VERSION_STEREO}）")
    if tuple(cfg.get("cams", ())) != STEREO_CAMS:
        raise ValueError(f"cams が {list(STEREO_CAMS)} ではない: {cfg.get('cams')}")
    for key in _CONTRACT_KEYS:
        if key not in cfg:
            raise ValueError(f"契約に {key} が無い")
    if cfg["disp_version"] != DISP_VERSION:
        raise ValueError(f"視差の版が違う（モデル {cfg['disp_version']}・ノード {DISP_VERSION}）")
    if not 0.0 < float(cfg["crop_x"]) < 0.3:
        raise ValueError(f"crop_x が不正: {cfg['crop_x']}")
    if not 0 <= int(cfg["disp_crop_cols"]) < int(cfg["disp_hw"][1]) // 2:
        raise ValueError(f"disp_crop_cols が不正: {cfg['disp_crop_cols']}")
    for key in ("max_steer", "speed_ref", "yaw_rate_ref", "ds", "xy_scale"):
        if not float(cfg[key]) > 0:
            raise ValueError(f"{key} が正でない: {cfg[key]}")
    model = StereoOnnx(str(onnx_path), cfg)
    out_dim = model.session.get_outputs()[0].shape[-1]
    if isinstance(out_dim, int) and out_dim != output_size(model.k):
        raise ValueError(f"出力の長さ {out_dim} が契約（K={model.k}）の {output_size(model.k)} と合わない")
    return model


class StereoE2ENode:
    """**`infer_once()` はバス・共有メモリを知らない純粋な計算。** `run()`（実バス配線）とテストの両方がここを通る。"""

    def __init__(self, *, model: StereoOnnx | None = None, models_dir: Path | None = None,
                 model_name: str = "", rate_hz: float = DEFAULT_RATE_HZ) -> None:
        self.model = model
        self.models_dir = models_dir or DEFAULT_MODELS_DIR
        self.model_name = model_name
        self.min_period_ns = int(NS / max(0.5, rate_hz))
        self._failed_key: tuple | None = None
        self._last_disp_seq = -1
        self._last_infer_t = -1
        self._running = False
        self._active = False
        self._reader = FrameReader()
        self._reader_left = FrameReader()

    def close(self) -> None:
        self._reader.close()
        self._reader_left.close()

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
            print(f"# モデル切替: {name}", flush=True)
            return True
        except Exception as e:                                # noqa: BLE001
            self._failed_key = key
            print(f"# モデル読み込み失敗（{name}）: {e}。前のモデルのまま続行", file=sys.stderr, flush=True)
            return False

    # ── 1回の推論（純粋な計算） ──

    def infer_once(self, *, disp: StereoDisp | None, rgb: np.ndarray | None, rgb_left: np.ndarray | None,
                   vs: VehicleState | None, now_ns: int, seq: int = 0) -> E2EFusionPlan:
        """視差・左右の画像（同じ撮影時刻 `disp.t_capture`）・車両状態から1回推論する。
        どこで失敗しても `ready=False` と理由を返す。左右どちらかが無ければ画像を伏せる。"""
        m = self.model
        if m is None:
            return self.failed("モデルが無い（models/stereo_e2e/ に .onnx と .json を置く）", seq=seq)
        if disp is None:
            return self.failed("視差（stereo/disp）が届いていない", seq=seq)
        if now_ns - int(disp.t_capture) > _DISP_STALE_S * NS:
            return self.failed("視差が古い", seq=seq)
        why = m.disp_mismatch(disp)
        if why:
            return self.failed(f"視差の設定がモデルと違う: {why}", seq=seq)
        t0 = time.perf_counter()
        try:
            d = disp_input(crop_disp(disp_from_bytes(disp.data, disp.h, disp.w), m.disp_crop_cols))
            image = m.image_input(rgb, rgb_left) if rgb is not None and rgb_left is not None else None
            state = m.state_input(vs.speed, vs.steer_actual, vs.yaw_rate) if vs is not None else None
            out = m.infer(image, d, state)
        except Exception as e:                                # noqa: BLE001
            print(f"# stereo_e2e の推論が例外: {e}", file=sys.stderr, flush=True)
            return self.failed(f"推論に失敗: {e}", seq=seq)
        return E2EFusionPlan(
            ready=True,
            path_x=out.path_xy[:, 0].tolist(), path_y=out.path_xy[:, 1].tolist(),
            path_v=out.path_v.tolist(), path_b=out.path_b.tolist(), ds=m.ds,
            has_image=image is not None, has_lidar=True,      # 2番目の入力（視差）を使った
            model=self.model_name,
            infer_ms=(time.perf_counter() - t0) * 1000.0, seq=seq, t_capture=int(disp.t_capture))

    def failed(self, reason: str, *, seq: int = 0) -> E2EFusionPlan:
        """`ready=False`。`t_capture` は「いま」（0 だと planning_node の鮮度判定が先に止め、理由が出ない）。"""
        return E2EFusionPlan(ready=False, reason=reason, model=self.model_name, seq=seq,
                             t_capture=time.monotonic_ns())

    def take_new_disp(self, disp: StereoDisp | None) -> StereoDisp | None:
        """推論のきっかけにする新しい視差（同じ組・上限頻度より早い組は None）。"""
        if disp is None or disp.seq == self._last_disp_seq:
            return None
        self._last_disp_seq = disp.seq
        t = int(disp.t_capture)
        # 撮影時刻で間引き、**平均が `rate_hz` になる**ように「次の予定」を周期ずつ進める（レビュー M3）。
        # 前回の推論から周期で測ると、15Hz（66.7ms 間隔）の視差を 10Hz にしたいとき 1 組おき（7.5Hz）に
        # なっていた。予定から大きく遅れた（組が途絶えた）ら今から数え直す。照合の時刻の揺れは
        # `_RATE_JITTER` の割合だけ早い組も通す
        period = self.min_period_ns
        if self._next_infer_t >= 0 and t < self._next_infer_t - int(period * _RATE_JITTER):
            return None
        due = self._next_infer_t + period
        self._next_infer_t = due if self._next_infer_t >= 0 and t - due < 0 else t + period
        return disp

    def read_pair(self, latest: dict, t_capture: int) -> tuple[np.ndarray | None, np.ndarray | None]:
        """視差を作った瞬間の左右のコマ（読めなければ None）。"""
        ref, left = latest.get(TOPIC_IMAGE_FRONT), latest.get(TOPIC_IMAGE_LEFT)
        if ref is None or left is None:
            return None, None
        got = self._reader.read_at(ref, t_capture, STEREO_PAIR_TOL_NS)
        got_l = self._reader_left.read_at(left, t_capture, STEREO_PAIR_TOL_NS) if got is not None else None
        if got is None or got_l is None:
            return None, None
        return to_rgb(got[0], ref.fmt), to_rgb(got_l[0], left.fmt)

    # ── ループ（実バス配線） ──

    def stop(self) -> None:
        self._running = False

    def run(self, *, sub, pub, duration_s: float | None = None, fixed_model: bool = False) -> None:
        self._running = True
        seq = 0
        t_end = time.monotonic() + duration_s if duration_s else None
        next_hb = time.monotonic_ns()
        next_model_check = 0.0
        last_pub = 0
        pacer = IdlePacer()
        while self._running:
            if t_end and time.monotonic() >= t_end:
                break
            for _ in sub.poll(pacer.poll_timeout_ms(self._active)):
                pass
            latest = sub.latest
            auto_ctrl = latest.get(TOPIC_AUTO_CTRL)
            vs = latest.get(TOPIC_VEHICLE_STATE)
            wanted = latest.get(TOPIC_STEREO_E2E_MODEL)
            chosen = wanted.name if wanted is not None and not fixed_model else ""
            if chosen:
                self.reload_if_changed(chosen)
            active = cam_infer_active(auto_ctrl, vs, MODES)
            if active != self._active:
                self._active = active
                print(f"# stereo_e2e {'推論開始' if active else '推論停止'}", flush=True)

            now = time.monotonic_ns()
            plan: E2EFusionPlan | None = None
            if not active:
                if not fixed_model and not chosen and time.monotonic() >= next_model_check:
                    next_model_check = time.monotonic() + _MODEL_CHECK_S
                    conf = self.models_dir / "__none__.json" if wanted is not None else MODEL_CONF
                    self.reload_if_changed(pick_model_name(self.models_dir, conf))
                if pacer.should_publish(active, now):
                    plan = self.failed("このモードが選ばれていない、または DISARM 中", seq=seq)
            else:
                disp = self.take_new_disp(latest.get(TOPIC_STEREO_DISP))
                if disp is not None:
                    rgb, rgb_left = self.read_pair(latest, int(disp.t_capture))
                    plan = self.infer_once(disp=disp, rgb=rgb, rgb_left=rgb_left, vs=vs, now_ns=now, seq=seq)
                elif now - last_pub > _NO_DISP_REPORT_S * NS:
                    cur = latest.get(TOPIC_STEREO_DISP)
                    plan = self.infer_once(disp=cur, rgb=None, rgb_left=None, vs=vs, now_ns=now, seq=seq)
                    if plan.ready:
                        plan = None                 # 届いている（間引いただけ）
            if plan is not None:
                pub.send(TOPIC_STEREO_E2E_PLAN, plan)
                seq += 1
                last_pub = now

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "stereo_e2e", HbMsg(node="stereo_e2e"))
            pacer.idle_sleep(active)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=None,
                    help="使うモデルを固定する（ONNX のパス。同名の .json が必要）。省略時は設定ファイルか最新")
    ap.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR))
    ap.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ, help="推論の上限頻度")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    node = StereoE2ENode(models_dir=Path(args.models_dir), rate_hz=args.rate_hz)
    if args.model:
        node.model = load_model(Path(args.model))
        node.model_name = Path(args.model).stem
    else:
        node.reload_if_changed(pick_model_name(node.models_dir, MODEL_CONF))

    pub = Publisher("stereo_e2e")
    sub = Subscriber({TOPIC_STEREO_DISP: LATEST, TOPIC_IMAGE_FRONT: LATEST, TOPIC_IMAGE_LEFT: LATEST,
                      TOPIC_VEHICLE_STATE: LATEST, TOPIC_AUTO_CTRL: LATEST, TOPIC_STEREO_E2E_MODEL: LATEST})
    print(f"# stereo_e2e_node  publish {pub.endpoint}  モデル: {node.model_name or '（なし）'}", flush=True)

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
