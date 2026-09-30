"""arrow_signal_node — 前方カメラの矢印信号（電光掲示板）を読み、`route/select` へ流す。

    .venv/bin/python -m raspi.nodes.arrow_signal_node

大会ルール（トヨタ技術会2026 ⑥矢印信号）の電光掲示板は、LEDドットマトリクスで
左/右/中央の矢印を「最も薄い色」（青・紫・緑のいずれか、低〜中彩度・高明度）で
表示する。濃い色での表示は別の意味を持ちうる（大会説明資料）ため、薄い色帯に
入らない点灯は「非該当」として無視する——安全側＝現在の経路グループのまま。

## なぜCNNではなく色/形状ベースの古典CVか

`sim/`はカメラ画像そのものを生成しない（`sim/README.md`）ため、CNN学習用の
合成データが作れず、学習データは実物の看板を実写するしかない。電光掲示板は
色数・形状パターンが少ない閉じた問題（3矢印×薄い色3系統）で、屋内・既知の
設置条件でもあるため、HSVしきい値＋モーメントという軽量な処理で十分と判断した
（`docs/competition_plan.md`）。ONNX推論より遥かに軽く、`cam_perception_node.py`
のようなスレッド数調整すら不要——`line_perception_node.py`（白線を古典的な
色しきい値で検出する）と同じ考え方。

## 受け口: `route/select`（ノード名`signal`を予約）

`raspi/bus/zbus.py`の`TOPIC_OWNER`で`route/select`は`signal`ノードの持ち物として
既に予約されている。`raspi/tools/route_select.py`（試験用CLI）と同じ endpoint に
bind するため、**このノードと`route_select.py`は同時に使えない**
（どちらも`signal`として publish しようとする）。

## ARM中は常時動く（特定の自動運転モードに紐づかない）

`cam_perception_node.py`等の`auto/ctrl`連動ゲート（特定モード選択中だけ推論）
とは違い、矢印信号はどの自動運転モードでも該当しうる横断的な入力なので、
`raspi.core.auto_gate.vehicle_armed()`だけで判定する（`cam_track_node.py`と
同じ「DISARM中は共有メモリすら読まない」節電。モード選択は見ない）。

## チャタリング対策: N連続フレーム一致でconfirmしてから流し続ける

単発フレームでの誤判定を避けるため、同じ方向が`_CONFIRM_FRAMES`回連続する
まで確定しない。確定後は`RouteSelect`を**毎周期そのまま流し続ける**
（`(value, event_id)`が変わらなければ何回届いても`slam2d_route`側は1回しか
切り替えない契約——`RouteSelect`のdocstring参照）。`event_id`は確定した値が
**変わったとき**だけ新しく発番する。左右どちらでもない（`straight`／非点灯）
の間は何も publish しない——現在の経路グループを維持する（安全側）。

## GUIからのライブ設定（`signal/config`）とON/OFFトグル

`SignalConfig`（`signal/config`）を購読し、`enabled=False`の間は
`vehicle_armed()`が真でも共有メモリを読まない（CPU節電のための明示トグル。
`raspi/nodes/cam_track_node.py`のIDLE節電と同じ考え方だが、こちらはモード
選択ではなくGUIの設定パネルから直接切り替える）。しきい値フィールド
（`roi_top`/`roi_bottom`/`sat_min`/`sat_max`/`val_min`/`min_lit_frac`）は
毎周期`self.detector`へそのまま反映する——会場で実物の看板を見ながらGUIの
スライダーを動かし、後述の`ArrowSignalStatus`（映像オーバーレイ）を見て
即座に追い込める（`raspi/tools/arrow_signal_preview.py`のヘッドレス版に
相当する体験をGUI側にも持たせる）。

## GUIへのライブフィードバック（`signal/status`）

`route/select`とは別に、直近フレームの生判定（`value`/`lit_frac`。confirm
前の値なので反応が速い）と確定値（`confirmed_value`）を`ArrowSignalStatus`
として毎周期publishする。GUIはこれを前方カメラ映像に重畳する（画素自体は
載せない——`ImageRef`と同じくバス帯域を節約する設計、`docs/architecture.md`
§6.2と同じ理由）。
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from raspi.core.auto_gate import vehicle_armed  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.msgs import ImageRef, SignalConfig, VehicleState  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_ROUTE_SELECT,
    TOPIC_SIGNAL_CONFIG,
    TOPIC_SIGNAL_STATUS,
    TOPIC_VEHICLE_STATE,
    ArrowSignalStatus,
    RouteSelect,
)

__all__ = ["ArrowSignalDetector", "ArrowSignalNode", "ArrowResult", "signal_mask"]

NS = 1_000_000_000
HB_HZ = 10

#: 画面高さに対する割合 `(top, bottom)`。看板は空間に固定されているので
#: `nav/ipm.py`の地面平面前提は使わない（地面上の物体ではないため相性が悪い、
#: モジュールdocstring参照）。単に画面上方の固定帯を切り出すだけにする。
#: 実機・実会場での取り付け高さに合わせてCLIから調整する前提の既定値
_DEFAULT_ROI_BAND = (0.05, 0.55)

#: 青・紫・緑の「薄い色」帯（OpenCVのHue 0-179）。彩度・明度は共通のレンジを使う。
#: 大会説明資料の「濃い色は別の意味」を除くため、彩度上限・明度下限を絞ってある。
#: **実機・実会場の照明でチューニングが要る**（`raspi/tools/arrow_signal_preview.py`）
_DEFAULT_HUE_BANDS = ((95, 130), (130, 160), (45, 85))   # 青 / 紫 / 緑
_DEFAULT_SAT_RANGE = (40, 180)
_DEFAULT_VAL_MIN = 150

#: ROI内の点灯画素がこの割合未満なら「非点灯・非該当」として`None`を返す
_MIN_LIT_FRAC = 0.02
#: 重心のROI中心からの水平偏差（ROI幅比）がこれ未満なら`straight`
_CENTER_DEADBAND_FRAC = 0.08
#: 同じ方向がこの回数連続するまで確定しない（単発フレームの誤判定対策）
_CONFIRM_FRAMES = 3


class ArrowResult:
    """1フレームの判定結果。`value`は`"left"`/`"right"`/`"straight"`/`None`（非該当）。"""

    __slots__ = ("value", "lit_frac")

    def __init__(self, value: str | None, lit_frac: float = 0.0) -> None:
        self.value = value
        self.lit_frac = lit_frac


def signal_mask(frame: np.ndarray, *,
                hue_bands: tuple[tuple[int, int], ...] = _DEFAULT_HUE_BANDS,
                sat_range: tuple[int, int] = _DEFAULT_SAT_RANGE,
                val_min: int = _DEFAULT_VAL_MIN) -> np.ndarray:
    """`(H, W, C)` uint8（`camera_node.py`既定の`RGB888`要求→メモリ上はBGR888）
    → 「薄い色」で点灯している画素の bool マスク。

    色相帯を複数OR合成する——矢印の色そのものに意味はなく、青/紫/緑いずれかの
    薄い帯で光っていれば点灯とみなす（モジュールdocstring参照）。
    """
    hsv = cv2.cvtColor(frame[..., :3], cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    sat_ok = (s >= sat_range[0]) & (s <= sat_range[1])
    val_ok = v >= val_min
    hue_ok = np.zeros(h.shape, dtype=bool)
    for lo, hi in hue_bands:
        hue_ok |= (h >= lo) & (h <= hi)
    return hue_ok & sat_ok & val_ok


class ArrowSignalDetector:
    """1フレーム → `ArrowResult`。バス・共有メモリを一切知らない純粋関数のラッパ。"""

    def __init__(self, *, roi_band: tuple[float, float] = _DEFAULT_ROI_BAND,
                hue_bands: tuple[tuple[int, int], ...] = _DEFAULT_HUE_BANDS,
                sat_range: tuple[int, int] = _DEFAULT_SAT_RANGE,
                val_min: int = _DEFAULT_VAL_MIN,
                min_lit_frac: float = _MIN_LIT_FRAC,
                center_deadband_frac: float = _CENTER_DEADBAND_FRAC) -> None:
        self.roi_band = roi_band
        self.hue_bands = hue_bands
        self.sat_range = sat_range
        self.val_min = val_min
        self.min_lit_frac = min_lit_frac
        self.center_deadband_frac = center_deadband_frac

    def process_frame(self, frame: np.ndarray) -> ArrowResult:
        h, w = frame.shape[:2]
        v0 = max(0, int(self.roi_band[0] * h))
        v1 = min(h, int(self.roi_band[1] * h))
        if v1 <= v0:
            return ArrowResult(None)
        roi = frame[v0:v1, :, :]

        mask = signal_mask(roi, hue_bands=self.hue_bands, sat_range=self.sat_range,
                           val_min=self.val_min)
        lit_frac = float(mask.mean())
        if lit_frac < self.min_lit_frac:
            return ArrowResult(None, lit_frac=lit_frac)

        # 重心のROI中心からの水平偏差で方向を判定する（矢印は指す側にドットが
        # 偏るドットマトリクス表示という前提。実物看板での最終確認が要る——
        # `line_perception_node.py`が白線の形状フィルタについて書いているのと
        # 同じ理由で、実コース以外での検証が原理的に当たらない領域）
        cols = np.arange(mask.shape[1])
        weights = mask.sum(axis=0).astype(np.float64)
        centroid_u = float((cols * weights).sum() / weights.sum())
        offset_frac = (centroid_u - mask.shape[1] / 2.0) / mask.shape[1]

        if offset_frac < -self.center_deadband_frac:
            value = "left"
        elif offset_frac > self.center_deadband_frac:
            value = "right"
        else:
            value = "straight"
        return ArrowResult(value, lit_frac=lit_frac)


class ArrowSignalNode:
    """前方カメラ → `route/select`。`process_cycle()`はバス・共有メモリを知らない純粋関数。"""

    def __init__(self, *, detector: ArrowSignalDetector | None = None,
                confirm_frames: int = _CONFIRM_FRAMES) -> None:
        self.detector = detector or ArrowSignalDetector()
        self.confirm_frames = confirm_frames

        self._pending_value: str | None = None
        self._pending_count = 0
        #: 確定した方向。**左右のみ**（`straight`/非点灯は経路切替の対象外なので
        #: 確定値として保持しない——大会ルールのボーナスは左右矢印にしかない）
        self._confirmed_value: str | None = None
        self._event_id = 0

        #: 直近フレームの生判定（GUIライブチューニング用。process_cycle()の
        #: confirmロジックとは独立に、毎周期の生値をそのまま持つ）
        self.last_result: ArrowResult | None = None

        #: ★省電力化・デバウンス修正: 直近に処理した `ImageRef.ring_seq`。
        #: カメラは30fps・このループは約10ms周期（`sub.poll(20)`）で回るので、
        #: 間を置かずに読むと同じフレームを約3回処理してしまう（issue #13）。
        #: **これは単なる無駄ではなく、`confirm_frames`（N連続一致で確定）が
        #: 同じフレームの繰り返しで満たされてしまい、デバウンスが実質
        #: `1/confirm_frames`ぶん弱まるという実害がある。** `ring_seq` が
        #: 変わっていなければ読み取り・`process_cycle()` の両方をスキップし、
        #: 確定状態は前回のまま据え置く
        self._last_ring_seq: int = -1

        self._reader = FrameReader()
        self._running = False

    def close(self) -> None:
        self._reader.close()

    # ── 1周期ぶんの処理（純粋関数。バスを知らない） ──

    def process_cycle(self, frame: np.ndarray | None) -> RouteSelect | None:
        """`frame`が`None`（DISARM中・読み取り失敗）ならconfirm状態は動かさず、
        既に確定済みの値があればそのまま流し続ける（`RouteSelect`は繰り返し
        publishしてよい契約——モジュールdocstring参照）。
        """
        if frame is not None:
            result = self.detector.process_frame(frame)
            self.last_result = result
            raw = result.value
            if raw in ("left", "right"):
                if raw == self._pending_value:
                    self._pending_count += 1
                else:
                    self._pending_value = raw
                    self._pending_count = 1
                if (self._pending_count >= self.confirm_frames
                        and raw != self._confirmed_value):
                    self._confirmed_value = raw
                    self._event_id = time.time_ns()
            else:
                self._pending_value = None
                self._pending_count = 0
        else:
            #: 読み取っていない周期はGUIにも「値なし」を出す（DISARM/enabled=False
            #: の間、映像オーバーレイが古い判定を出し続けないように）
            self.last_result = None

        return self._current_route_select()

    def reset(self) -> None:
        self._pending_value = None
        self._pending_count = 0
        self._confirmed_value = None
        self._event_id = 0

    def _current_route_select(self) -> RouteSelect | None:
        """`process_cycle()`を呼ばずに、今の確定状態から`RouteSelect`だけ組み立てる。

        同じフレームを再処理しないスキップ経路（`run()`参照）で、confirm状態は
        動かさずに直近の確定値をそのまま流し続けるために使う——
        `process_cycle()`の末尾と同じ組み立て方（**書き写さない**ために
        ここへ切り出した）。
        """
        if self._confirmed_value is None:
            return None
        return RouteSelect(value=self._confirmed_value, source="arrow_signal",
                           event_id=self._event_id)

    # ── 共有メモリの読み取り（`raspi/core/frame_reader.py` に委譲） ──

    def read_frame(self, ref: ImageRef) -> tuple[np.ndarray, int] | None:
        return self._reader.read(ref)

    # ── ループ（実バス配線） ──

    def stop(self) -> None:
        self._running = False

    def run(self, *, sub, pub, duration_s: float | None = None) -> None:
        self._running = True
        t_end = time.monotonic() + duration_s if duration_s else None
        next_hb = time.monotonic_ns()
        while self._running:
            if t_end and time.monotonic() >= t_end:
                break
            for _ in sub.poll(20):
                pass  # `latest` を見るだけなので中身の処理は不要

            cfg: SignalConfig | None = sub.latest.get(TOPIC_SIGNAL_CONFIG)
            enabled = cfg.enabled if cfg is not None else True
            if cfg is not None:
                #: 会場でGUIのスライダーを動かすたびに反映する（モジュールdocstring
                #: 「GUIからのライブ設定」参照）。値が同じでも軽量な代入なので、
                #: 変化検出は不要——`CamModelCtrl`拾い直しと同じ「毎回代入」でよい
                self.detector.roi_band = (cfg.roi_top, cfg.roi_bottom)
                self.detector.sat_range = (cfg.sat_min, cfg.sat_max)
                self.detector.val_min = cfg.val_min
                self.detector.min_lit_frac = cfg.min_lit_frac

            vs: VehicleState | None = sub.latest.get(TOPIC_VEHICLE_STATE)
            frame = None
            skip_same_frame = False
            if vehicle_armed(vs) and enabled:
                #: DISARM中・enabled=False中は共有メモリすら読まない
                #: （モジュールdocstring参照、`cam_track_node.py`と同じ節電）
                ref = sub.latest.get(TOPIC_IMAGE_FRONT)
                if ref is not None:
                    if ref.ring_seq == self._last_ring_seq:
                        # ★省電力化・デバウンス修正: 新しいカメラフレームが
                        # まだ来ていない。読み取り・process_cycle()の両方を
                        # スキップする（上の `_last_ring_seq` docstring参照）
                        skip_same_frame = True
                    else:
                        got = self.read_frame(ref)
                        if got is not None:
                            frame, _t_capture = got
                            self._last_ring_seq = ref.ring_seq

            if skip_same_frame:
                msg = self._current_route_select()
            else:
                try:
                    msg = self.process_cycle(frame)
                except Exception as e:
                    # process_cycle側のバグでノード全体を巻き込んで落とさない
                    # （`planning_node._replan()`と同じパターン）。confirm状態は
                    # 前周期のまま据え置かれる
                    print(f"# arrow_signal process_cycle() が例外: {e}",
                         file=sys.stderr, flush=True)
                    msg = None
            if msg is not None:
                pub.send(TOPIC_ROUTE_SELECT, msg)

            #: GUIライブチューニング・映像オーバーレイ向けの現在値（モジュール
            #: docstring「GUIへのライブフィードバック」参照）。`LineScan`と同じく
            #: 確定の有無に関わらず毎周期出す
            status = ArrowSignalStatus(
                enabled=enabled,
                value=(self.last_result.value or "") if self.last_result else "",
                lit_frac=self.last_result.lit_frac if self.last_result else 0.0,
                confirmed_value=self._confirmed_value or "",
                roi_top=self.detector.roi_band[0], roi_bottom=self.detector.roi_band[1])
            pub.send(TOPIC_SIGNAL_STATUS, status)

            now = time.monotonic_ns()
            if now >= next_hb:
                next_hb = now + NS // HB_HZ
                pub.send(TOPIC_HB_PREFIX + "signal", HbMsg(node="signal"))


def _parse_band(s: str) -> tuple[float, float]:
    a, b = (float(x) for x in s.split(","))
    return a, b


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--roi-band", default="%.2f,%.2f" % _DEFAULT_ROI_BAND,
                    help="看板を探す画面高さ割合 top,bottom（実機で要調整）")
    ap.add_argument("--sat-min", type=int, default=_DEFAULT_SAT_RANGE[0])
    ap.add_argument("--sat-max", type=int, default=_DEFAULT_SAT_RANGE[1])
    ap.add_argument("--val-min", type=int, default=_DEFAULT_VAL_MIN)
    ap.add_argument("--min-lit-frac", type=float, default=_MIN_LIT_FRAC)
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    detector = ArrowSignalDetector(roi_band=_parse_band(args.roi_band),
                                   sat_range=(args.sat_min, args.sat_max),
                                   val_min=args.val_min,
                                   min_lit_frac=args.min_lit_frac)
    node = ArrowSignalNode(detector=detector)

    pub = Publisher("signal")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_VEHICLE_STATE: LATEST,
                     TOPIC_SIGNAL_CONFIG: LATEST})

    print(f"# arrow_signal_node  publish {pub.endpoint}  route/select へ配信")

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
