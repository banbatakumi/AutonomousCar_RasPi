"""E2E の推論ノード（`e2e_fusion_node`・`stereo_e2e_node`）の共通の骨格。

どちらも「GUI で選んだモデル（空なら設定ファイル → いちばん新しいもの）を読み直しながら、自分のモードが
engage されている間だけ推論し、`ready=False` の理由も含めて流す」という同じ形をしている。違うのは
入力（LiDAR の履歴か視差か）と前処理だけなので、それを各ノードの `step()` に書き、ここに次を置く:

- モデルの選択と読み直し（失敗しても今のモデルを保持し、同じファイルは再試行しない。最新が壊れていれば
  古い有効なモデルへ）。セッションを作る間ループを塞がないよう別スレッドでも読める（`BACKGROUND_LOAD`）
- 契約（モデル同梱の `.json`）の共通の検査（`check_contract`）
- ループ: 推論していない間の間引き（`IdlePacer`）・モデルの選び直し・hb
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Callable, Generic, TypeVar

from raspi.core.auto_gate import IdlePacer, cam_infer_active
from raspi.core.cam_e2e_preproc import PREPROC_VERSION_STEREO, STEREO_CAMS
from raspi.core.frame_reader import FrameReader
from raspi.msgs import Heartbeat as HbMsg
from raspi.msgs.types import TOPIC_AUTO_CTRL, TOPIC_HB_PREFIX, TOPIC_VEHICLE_STATE
from raspi.core.log import get_logger

_log = get_logger("e2e")

__all__ = ["E2ENodeBase", "check_contract", "pick_model_names", "pick_model_name"]

NS = 1_000_000_000
HB_HZ = 10
#: 推論していない間に、モデルを選び直す（新しいモデルが置かれたか見る）周期 [s]
MODEL_CHECK_S = 2.0

M = TypeVar("M")


def pick_model_names(models_dir: Path, conf_path: Path) -> list[str]:
    """使うモデルの候補を優先順に。設定ファイルの指定（あれば）、続いて契約のある `.onnx` を新しい順。

    先頭が壊れていても（書き出し途中・契約が合わない）次を試せるように全部返す（レビュー L20）。
    """
    out: list[str] = []
    try:
        name = str(json.loads(conf_path.read_text()).get("name", ""))
        if name and (models_dir / f"{name}.onnx").is_file():
            out.append(name)
    except (OSError, ValueError, AttributeError):
        pass                                   # 無い・壊れている → 新しい順だけ
    try:
        files = [p for p in models_dir.glob("*.onnx") if p.with_suffix(".json").is_file()]
    except OSError:
        files = []
    for p in sorted(files, key=lambda p: p.stat().st_mtime_ns, reverse=True):
        if p.stem not in out:
            out.append(p.stem)
    return out


def pick_model_name(models_dir: Path, conf_path: Path) -> str:
    """使うモデルの名前。設定ファイルの指定、無ければいちばん新しい `.onnx`。無ければ ""。"""
    names = pick_model_names(models_dir, conf_path)
    return names[0] if names else ""


def check_contract(onnx_path: Path, *, preproc_version: int, required: tuple[str, ...],
                   positive: tuple[str, ...], export_hint: str) -> dict:
    """`<name>.onnx` と同梱の `<name>.json`（契約）を確かめて契約の dict を返す。**合わなければ例外。**

    入出力の版・左右2台の画像の前処理の版・カメラの組・必須キー・`crop_x` の範囲・正であるべき値を見る。
    """
    if not onnx_path.exists():
        raise FileNotFoundError(f"モデルが見つかりません: {onnx_path}")
    cfg_path = onnx_path.with_suffix(".json")
    if not cfg_path.exists():
        raise FileNotFoundError(f"契約ファイルがありません: {cfg_path}")
    cfg = json.loads(cfg_path.read_text())
    if cfg.get("preproc_version") != preproc_version:
        raise ValueError(f"入出力の版が違う（モデル {cfg.get('preproc_version')}・ノード {preproc_version}）。"
                         f"`{export_hint}` で書き出し直す")
    if cfg.get("image_preproc_version") != PREPROC_VERSION_STEREO:
        # 版2は前後配置のころの右1台のモデル（視点が違うので使えない。左右の組で学習し直す）
        raise ValueError(f"画像の前処理の版が違う（モデル {cfg.get('image_preproc_version')}・"
                         f"ノード {PREPROC_VERSION_STEREO}）。左右2台で学習し直したモデルが必要")
    if tuple(cfg.get("cams", ())) != STEREO_CAMS:
        raise ValueError(f"cams が {list(STEREO_CAMS)} ではない: {cfg.get('cams')}")
    for key in required:
        if key not in cfg:
            raise ValueError(f"契約に {key} が無い")
    if not 0.0 < float(cfg["crop_x"]) < 0.3:
        raise ValueError(f"crop_x が不正: {cfg['crop_x']}")
    for key in positive:
        if not float(cfg[key]) > 0:
            raise ValueError(f"{key} が正でない: {cfg[key]}")
    return cfg


class E2ENodeBase(Generic[M]):
    """**`step()` 以下はバス・共有メモリを知らない純粋な計算に寄せる。** `run()`（実バス配線）とテストの両方が通る。

    派生クラスが決めるもの: `NAME`・`MODES`・`MODEL_TOPIC`・`PLAN_TOPIC`・`MODEL_CONF`・`load_model`、
    `step()`（推論中の1周期）・`failed()`（`ready=False` の出力）。
    """

    NAME = ""
    MODES: tuple[str, ...] = ()
    MODEL_TOPIC = ""
    PLAN_TOPIC = ""
    MODEL_CONF = Path()
    #: GUI で選んだモデルを別スレッドで読む（ONNX のセッションを作る間、推論のループを塞がない。レビュー L21）
    BACKGROUND_LOAD = False

    def __init__(self, *, load_model: Callable[[Path], M], models_dir: Path,
                 model: M | None = None, model_name: str = "") -> None:
        self._load_model = load_model
        self.model = model
        self.models_dir = models_dir
        self.model_name = model_name
        self._failed_key: tuple | None = None
        #: 別スレッドで読んでいるモデルの名前と、読み終えた結果 `(名前, キー, モデル, 例外)`
        self._loading: str | None = None
        self._loaded: tuple | None = None
        self._running = False
        self._active = False
        self._reader = FrameReader()
        #: 左カメラのリング用（`FrameReader` は1本のリングしか掴まない）
        self._reader_left = FrameReader()

    def close(self) -> None:
        self._reader.close()
        self._reader_left.close()

    def stop(self) -> None:
        self._running = False

    # ── モデル ──

    def on_model_changed(self) -> None:
        """モデルを差し替えた直後に呼ぶ（入力の履歴の消去など）。"""

    def _model_key(self, name: str) -> tuple:
        path = self.models_dir / f"{name}.onnx"

        def mtime(p: Path):
            try:
                return p.stat().st_mtime_ns
            except OSError:
                return None

        return (name, mtime(path), mtime(path.with_suffix(".json")))

    def _adopt(self, name: str, key: tuple, model: M | None, err: Exception | None) -> bool:
        if model is not None:
            self.model = model
            self.model_name = name
            self._failed_key = None
            self.on_model_changed()
            _log.info(f"モデル切替: {name}")
            return True
        self._failed_key = key
        _log.info(f"モデル読み込み失敗（{name}）: {err}。前のモデルのまま続行")
        return False

    def _try_load(self, name: str, key: tuple) -> tuple:
        try:
            return (name, key, self._load_model(self.models_dir / f"{name}.onnx"), None)
        except Exception as e:                                # noqa: BLE001  壊れ方は問わない（今のモデルを保持）
            return (name, key, None, e)

    def reload_if_changed(self, name: str) -> bool:
        """`name` のモデルへ切り替える。失敗しても今のモデルを保持し、同じファイルは再試行しない。"""
        if not name or name == self.model_name:
            return False
        key = self._model_key(name)
        if key == self._failed_key:
            return False
        return self._adopt(*self._try_load(name, key))

    def reload_first_usable(self, names: list[str]) -> bool:
        """`names`（優先順）のうち、読めるいちばん前のモデルへ切り替える。今のモデルに着いたらそこで止める
        （それより後ろ＝古いものへは移らない）。最新が壊れていても、古い有効なモデルで動けるように。"""
        for name in names:
            if name == self.model_name:
                return False
            if self.reload_if_changed(name):
                return True
        return False

    def reload_in_background(self, name: str) -> None:
        """`reload_if_changed` を別スレッドで。読み終えたら次の `take_loaded()` で差し替える。
        読んでいる間は今のモデルで推論を続ける。"""
        if not name or name == self.model_name or self._loading is not None:
            return
        key = self._model_key(name)
        if key == self._failed_key:
            return
        self._loading = name

        def work() -> None:
            self._loaded = self._try_load(name, key)

        threading.Thread(target=work, name=f"{self.NAME}-load", daemon=True).start()

    def take_loaded(self) -> bool:
        """別スレッドで読み終えたモデルがあれば差し替える。差し替えたら True。"""
        got, self._loaded = self._loaded, None
        if got is None:
            return False
        self._loading = None
        return self._adopt(*got)

    # ── 派生クラスが書くもの ──

    def failed(self, reason: str, *, seq: int = 0):
        raise NotImplementedError

    def on_poll(self, msgs: list) -> None:
        """`sub.poll()` で届いたメッセージ（`(topic, msg)`）。履歴を積むノードが使う。"""

    def on_active_changed(self, active: bool) -> None:
        """推論の開始・停止の直後に呼ぶ。"""

    def step(self, latest: dict, vs, now_ns: int, seq: int, last_pub_ns: int):
        """推論中の1周期。流す出力（無ければ None）。"""
        raise NotImplementedError

    # ── ループ（実バス配線） ──

    def run(self, *, sub, pub, duration_s: float | None = None, status_cb=None,
            fixed_model: bool = False) -> None:
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
            self.on_poll(list(sub.poll(pacer.poll_timeout_ms(self._active))))
            latest = sub.latest
            vs = latest.get(TOPIC_VEHICLE_STATE)
            # GUI で選んだモデル（空の名前は「自動」＝下の、推論していない間の選び直しに任せる）
            wanted = latest.get(self.MODEL_TOPIC)
            chosen = wanted.name if wanted is not None and not fixed_model else ""
            if self.BACKGROUND_LOAD:
                self.take_loaded()
                if chosen:
                    self.reload_in_background(chosen)
            elif chosen:
                self.reload_if_changed(chosen)
            active = cam_infer_active(latest.get(TOPIC_AUTO_CTRL), vs, self.MODES)
            if active != self._active:
                self._active = active
                _log.info(f"{self.NAME} {'推論開始' if active else '推論停止'}")
                self.on_active_changed(active)

            now = time.monotonic_ns()
            plan = None
            if not active:
                # 走っていない間だけモデルを選び直す
                if not fixed_model and not chosen and time.monotonic() >= next_model_check:
                    next_model_check = time.monotonic() + MODEL_CHECK_S
                    # GUI から「自動」と言われていれば最新、何も届いていなければ設定ファイル → 最新
                    conf = self.models_dir / "__none__.json" if wanted is not None else self.MODEL_CONF
                    self.reload_first_usable(pick_model_names(self.models_dir, conf))
                if pacer.should_publish(active, now):
                    plan = self.failed("このモードが選ばれていない、または DISARM 中", seq=seq)
            else:
                plan = self.step(latest, vs, now, seq, last_pub)
            if plan is not None:
                pub.send(self.PLAN_TOPIC, plan)
                seq += 1
                last_pub = now
                if status_cb:
                    status_cb(plan)

            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + self.NAME, HbMsg(node=self.NAME))
            pacer.idle_sleep(active)
