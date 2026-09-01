"""cam_track_node — 前方カメラ＋LiDARを融合してドラッグ選択した対象を追跡する。

    .venv/bin/python -m raspi.nodes.cam_track_node

GUIが映像上でドラッグ選択した矩形（`track/roi`、`TargetRoiCtrl`）を受け、
NanoTrackで追跡したbboxからカメラの方位角(bearing)を算出し、LiDARの`scan`から
同じ方位の実測距離を読んで融合した結果を`track/target`（`TargetTrack`）へ
publishする。`raspi/auto/follow_object.py`（`FollowObject`）の`input_topic`。

## なぜカメラ単眼IPMではなくLiDAR融合か

`raspi/nav/ipm.py`の地面平面仮定は、人や車のような**高さのある物体**では
足元が地面に接していない・傾いているだけで距離誤差が大きくなる。ここでは
**カメラは対象の方位角(bearing)算出にのみ使い、実際の距離はLiDARの`Scan.dist`
配列から同じ方位を実測する**（bearingの算出はIPMのground投影を経由せず、
ピンホールカメラの内部パラメータ`f`/`cx`だけで決まる——`_bearing_for_pixel()`
参照。地面までの距離が不要な理由もそこにある）。

## NanoTrackモデルの置き場

`models/nano_track/nanotrack_backbone_sim.onnx` + `nanotrack_head_sim.onnx`
（OpenCVの`samples/dnn/models.yml`が参照する`HonglinChu/SiamTrackers`の
NanoTrack v2 ONNXエクスポートと同じもの。SHA1で検証済み）。**`.gitignore`
済み**（`models/`配下は他のONNXモデルと同様にコミットしない）。

モデルが無い間は`TargetTracker.available=False`のまま——`_try_select()`が
`select_seq`を消費せずに`tracking=False`を返し続けるので、あとからモデルを
配置してノードを再起動すれば選び直しから復帰できる（`cam_perception_node.py`
の「モデル未選択の間は壁扱いで出し続ける」と同じ、失敗を握りつぶして
安全側へ倒す設計）。

## `process_cycle()`はバスを知らない純粋関数

`run()`（実バス配線）とテスト（`raspi/tests/test_cam_track_node.py`）の
両方がここを通る（`cam_perception_node.py`と同じ構成）。
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from raspi.auto.base import sector_of_deg  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs import ImageRef, Scan, TargetRoiCtrl, TargetTrack  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_SCAN,
    TOPIC_TRACK_ROI,
    TOPIC_TRACK_TARGET,
)
from raspi.nav.ipm import camera_intrinsics  # noqa: E402

__all__ = ["CamTrackNode", "TargetTracker"]

NS = 1_000_000_000
HB_HZ = 10
#: `surge_mk2/`。既定モデルの場所を解決する基準（`cam_perception_node.py`と同じ深さ）
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODELS_DIR = REPO_ROOT / "models" / "nano_track"
BACKBONE_NAME = "nanotrack_backbone_sim.onnx"
NECKHEAD_NAME = "nanotrack_head_sim.onnx"

#: `scan`（LiDAR）がこれより古ければ融合に使わない。ギャップ探索系planner
#: 既定の`Planner.stale_ms`（300ms、LiDAR 10Hzを前提）と揃えた
_SCAN_STALE_NS = 300_000_000
#: LiDAR距離の妥当性ゲート。bboxが実際の見込み角より狭い/広いブレを吸収する下限・上限
_MIN_HALF_ANGLE_DEG = 2.0
_MAX_HALF_ANGLE_DEG = 15.0
#: 「遠のく方向へ大きく飛んだ」とみなす変化速度の上限[m/s]（Phase 2で実測調整）。
#: 実車測定（2026-09-01）でLiDAR距離融合の呼び出し間隔は中央値約20ms・p90約30ms
#: だった。**固定の距離差ではなく実際の経過時間`dt`に対する速度で見る**——
#: 呼び出し間隔が伸びたとき（見失いからの復帰直後等）に固定閾値だと過敏/鈍感の
#: どちらにも振れうるため。5m/sは走行速度（〜数m/s）+対象自身の動きを見込んだ余裕
_MAX_CLOSING_SPEED_MPS = 5.0
#: 上の速度×dtがこれを下回っても、**センサノイズぶんの変化は飛び扱いにしない**
#: 下限[m]。LiDARの分解能・NanoTrackのbbox揺らぎからの見込み角ブレを吸収する
_MIN_JUMP_M = 0.15
#: 遠のく方向の飛びを採用するまでに要する連続一致フレーム数
_CONFIRM_FRAMES = 3
#: ROI選択として受理する最小の矩形サイズ[px]（手ぶれ・誤クリックの排除）
_MIN_ROI_PX = 4.0
#: **ちらつき吸収。** NanoTrackの`update()`が単発で失敗しても、これだけ
#: 連続して失敗するまでは`lost`にしない（動きブレ・一瞬の重なりでの誤判定を
#: 吸収する。Phase 1では初回失敗で即`lost=True`にしていたが、`FollowObject`
#: 側の減衰処理まで毎回誘発してしまっていたため Phase 2 で追加）
_LOST_DEBOUNCE_FRAMES = 2
#: ★見失ってからこれだけ経ったら追跡を諦め、`IDLE`へ戻して選び直しを要求する。
#:
#: **NanoTrackは「検出」ではなく「追跡」アルゴリズム**——直前の位置を中心にした
#: 狭い探索範囲しか見ない（`update()`のOpenCVドキュメントも「対象が画角内に
#: いないだけかもしれない」と明言している）。対象が画角外に出て**別の場所から
#: 再入場した**場合や、長く隠れて位置がずれた場合は、探索範囲の外なので
#: 再認識できる保証が無い。
#:
#: 諦めずに`update()`を呼び続けると、探索範囲にたまたま入ってきた**別の物体を
#: 誤って対象と取り違える**リスクもある——見失う前より悪い。
#: `FollowObject.lost_timeout_s`（既定1.5s）で車は既に停止しているので、
#: この値はそれより長く取り、「戻ってくるかもしれない」猶予を残しつつ、
#: 尽きたら安全側（選び直し必須）に倒す
_GIVE_UP_MS = 5000.0


class TargetTracker:
    """`cv2.TrackerNano`の薄いラッパ。**モデルが無ければ`None`同然に振る舞う**
    （`cam_perception_node.SegmentationModel`と同じ「失敗を握りつぶして安全側
    に倒す」思想）。1つのインスタンスは1回の選択に対応する使い切りで、
    選び直すたびに`CamTrackNode`が新しいインスタンスを作る
    （前の対象の内部状態を持ち越さないため）。
    """

    def __init__(self, backbone_path: Path, neckhead_path: Path) -> None:
        import cv2

        params = cv2.TrackerNano_Params()
        params.backbone = str(backbone_path)
        params.neckhead = str(neckhead_path)
        self._tracker = cv2.TrackerNano_create(params)

    def init(self, frame: np.ndarray, box_px: tuple[int, int, int, int]) -> None:
        self._tracker.init(frame, box_px)

    def update(self, frame: np.ndarray) -> tuple[bool, tuple[float, float, float, float]]:
        return self._tracker.update(frame)

    def score(self) -> float:
        return float(self._tracker.getTrackingScore())


class TargetTrackerFactory:
    """モデルの読み込み状態を持ち、選択のたびに新しい`TargetTracker`を作る。"""

    def __init__(self, models_dir: Path) -> None:
        self.error: str | None = None
        self._backbone = models_dir / BACKBONE_NAME
        self._neckhead = models_dir / NECKHEAD_NAME
        if not self._backbone.is_file() or not self._neckhead.is_file():
            self.error = (f"NanoTrackモデルが見つかりません: "
                          f"{self._backbone} / {self._neckhead}")

    @property
    def available(self) -> bool:
        return self.error is None

    def new_instance(self) -> TargetTracker | None:
        """**失敗しても例外を投げない。** 選択のたびに呼ぶので、ここで例外が
        漏れると`process_cycle()`ごと落ちてノードが死ぬ（`_load_from_path()`系の
        「握りつぶさない」とは逆の場面——ロード失敗は`cam_perception_node`の
        推論失敗と同じ「1周期分の失敗」として扱いたいため、ここでは握りつぶす）。
        """
        if not self.available:
            return None
        try:
            return TargetTracker(self._backbone, self._neckhead)
        except Exception as e:                                        # noqa: BLE001
            self.error = f"NanoTrackの初期化に失敗: {e}"
            return None


def _wrap_rad(rad: float) -> float:
    """`base.wrap_deg()`のラジアン版。±πの符号付きに直す。"""
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


class CamTrackNode:
    """1本のROI選択 → 追跡結果。

    **`process_cycle()` はバス・共有メモリを一切知らない純粋関数。**
    `run()`（実バス配線）とテストの両方がここを通る。
    """

    def __init__(self, *, vehicle: Vehicle | None = None,
                models_dir: str | Path | None = None,
                factory: TargetTrackerFactory | None = None) -> None:
        self.vehicle = vehicle or Vehicle.load()
        self.models_dir = Path(models_dir) if models_dir else DEFAULT_MODELS_DIR
        self._factory = factory or TargetTrackerFactory(self.models_dir)

        #: ROI選択の状態機械。`IDLE`→(`select_seq`増加で)`TRACKING`。
        #: `clear_seq`増加で`IDLE`へ（`raspi/msgs/types.py`の`TargetRoiCtrl`参照）
        self._state = "IDLE"
        self._last_select_seq = 0
        self._last_clear_seq = 0
        self._tracker: TargetTracker | None = None

        #: `TRACKING`中に保持する直近の値（`lost=True`の間もこれを返し続ける）
        self._last_bearing = 0.0
        self._last_bbox = (0.0, 0.0, 0.0, 0.0)          # cx, cy, w, h（正規化）
        #: 直近フレームのピンホール内部パラメータ。`lost`でフレームが読めない
        #: 周期でも、LiDAR融合の見込み角計算に直近値を使えるよう保持する
        self._last_f = 0.0
        self._last_cx = 0.0

        self._lost_since_ns: int | None = None
        #: 直近の連続`update()`失敗回数。**`_LOST_DEBOUNCE_FRAMES`回続くまでは
        #: `lost`にしない**（ちらつき吸収）。成功したら即座に0へ戻す
        self._miss_streak = 0

        #: LiDAR距離の妥当性ゲート（「距離を近づける方向にしか誤らない」安全側）
        self._last_distance: float | None = None
        self._last_distance_ns: int | None = None
        self._pending_distance: float | None = None
        self._pending_count = 0

        self._running = False

    # ── ROI選択の状態遷移 ──

    def _enter_idle(self) -> None:
        self._state = "IDLE"
        self._tracker = None
        self._lost_since_ns = None
        self._miss_streak = 0
        self._last_bearing = 0.0
        self._last_bbox = (0.0, 0.0, 0.0, 0.0)
        self._last_distance = None
        self._last_distance_ns = None
        self._pending_distance = None
        self._pending_count = 0

    def _try_select(self, frame: np.ndarray, roi: TargetRoiCtrl) -> bool:
        """`roi`で指定された矩形からNanoTrackを初期化する。

        :returns: 選択seqを消費してよければ`True`。**モデル未配置・矩形が
            小さすぎる場合は`False`を返し、seqを消費しない**——次周期に
            同じ`roi`でもう一度試せるようにするため（モデルを後から配置した
            場合や、GUI側の判定漏れで極小矩形が飛んできた場合の保険）。
        """
        h, w = frame.shape[:2]
        x0 = max(0.0, min(1.0, roi.x0)) * w
        y0 = max(0.0, min(1.0, roi.y0)) * h
        x1 = max(0.0, min(1.0, roi.x1)) * w
        y1 = max(0.0, min(1.0, roi.y1)) * h
        bw, bh = x1 - x0, y1 - y0
        if bw < _MIN_ROI_PX or bh < _MIN_ROI_PX:
            return False

        tracker = self._factory.new_instance()
        if tracker is None:
            return False
        try:
            tracker.init(frame, (int(x0), int(y0), int(bw), int(bh)))
        except Exception:                                              # noqa: BLE001
            return False

        self._tracker = tracker
        self._state = "TRACKING"
        self._lost_since_ns = None
        self._miss_streak = 0
        self._last_distance = None
        self._last_distance_ns = None
        self._pending_distance = None
        self._pending_count = 0
        self._update_intrinsics(w, h)
        cx_px, cy_px = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        self._last_bbox = (cx_px / w, cy_px / h, bw / w, bh / h)
        self._last_bearing = self._bearing_for_pixel(cx_px)
        return True

    # ── カメラの内部パラメータ・方位角 ──

    def _update_intrinsics(self, frame_w: int, frame_h: int) -> None:
        intr = camera_intrinsics(self.vehicle.cam_front_hfov, frame_w, frame_h,
                                 self.vehicle.cam_front_bottom_crop)
        self._last_f = intr.f
        self._last_cx = intr.cx

    def _bearing_for_pixel(self, u_px: float) -> float:
        """画素→方位角。**IPMの地面投影は使わない**（ファイル冒頭の docstring 参照）。

        `pixel_to_ground()`の`lateral = (cx-u)*zc/f`と`depth = zc`（pitch分の
        投影は水平角には効かない）から`atan2(lateral, depth) = atan2(cx-u, f)`
        が導ける——地面までの距離や取付ピッチが要らない式になる。
        """
        raw = self.vehicle.cam_front_yaw + math.atan2(self._last_cx - u_px, self._last_f)
        return _wrap_rad(raw)

    # ── LiDAR融合 ──

    def _gate_distance(self, raw: float, now_ns: int) -> float:
        """距離の妥当性ゲート。**「距離を近づける方向にしか誤らない」安全側。**

        直前値より近づく・小さな変化は即座に採用する（早めに減速・停止する
        側の誤りは安全）。**大きく遠のく方向へ飛んだ値は、同じ値が
        `_CONFIRM_FRAMES`回連続するまで採用しない**——1フレームだけのノイズで
        「対象が遠い」と誤判定して速度を上げてしまう事故を防ぐ。

        「大きく」の基準は固定の距離差ではなく、**実際の経過時間`dt`に対する
        変化速度**（`_MAX_CLOSING_SPEED_MPS`）で決める。呼び出し間隔は
        待機/追跡中でほぼ変わらないが、見失いからの復帰直後など`dt`が伸びる
        場面で固定距離閾値だと過敏（dtが短いのに大きく動いたと誤検知）・
        鈍感（dtが長いのに小さな変化を見逃す）のどちらにも振れうるため
        （Phase 2、2026-09-01の実測で発見）。
        """
        if self._last_distance is None or self._last_distance_ns is None:
            self._last_distance = raw
            self._last_distance_ns = now_ns
            self._pending_distance = None
            self._pending_count = 0
            return raw

        dt_s = max(0.0, (now_ns - self._last_distance_ns) / 1e9)
        max_jump = max(_MIN_JUMP_M, _MAX_CLOSING_SPEED_MPS * dt_s)

        if raw <= self._last_distance + max_jump:
            self._last_distance = raw
            self._last_distance_ns = now_ns
            self._pending_distance = None
            self._pending_count = 0
            return raw
        if self._pending_distance is not None and abs(raw - self._pending_distance) <= max_jump:
            self._pending_count += 1
        else:
            self._pending_distance = raw
            self._pending_count = 1
        if self._pending_count >= _CONFIRM_FRAMES:
            self._last_distance = raw
            self._last_distance_ns = now_ns
            self._pending_distance = None
            self._pending_count = 0
            return raw
        return self._last_distance

    def _fuse_lidar(self, scan: Scan | None, now_ns: int) -> tuple[float, bool]:
        """`self._last_bearing`方向±見込み角の範囲で最小実測距離を読む。

        `sector_seen`/`saturated`/測距不能の読み方は`base.scan_window()`と
        同じ契約——**書き写さない**（欠測・飽和・測距不能をどう読むかは
        点群の読み方の契約そのもので、片方だけ直すともう片方が古い読み方の
        まま走る、という`follow_the_gap.py`と同じ理由）。ここでは
        「進んでよい距離」ではなく「対象までの実測距離」が欲しいので、
        欠測・飽和・測距不能はすべて**候補から除外する**（空き扱いにしない）。
        """
        if scan is None or self._last_f <= 0.0:
            return 0.0, False
        if now_ns - scan.t_pub > _SCAN_STALE_NS:
            return 0.0, False

        _, _, bbox_w_norm, _ = self._last_bbox
        frame_w_est = self._last_cx * 2.0
        half_w_px = max(1.0, bbox_w_norm * frame_w_est / 2.0)
        half_angle_deg = math.degrees(math.atan2(half_w_px, self._last_f))
        half_angle_deg = max(_MIN_HALF_ANGLE_DEG, min(_MAX_HALF_ANGLE_DEG, half_angle_deg))

        center_deg = math.degrees(self._last_bearing)
        half_i = int(math.ceil(half_angle_deg))
        candidates: list[float] = []
        for off in range(-half_i, half_i + 1):
            i = (int(round(center_deg)) + off) % 360
            if not scan.sector_seen[sector_of_deg(i)]:
                continue
            if scan.saturated is not None and scan.saturated[i]:
                continue
            raw = scan.dist[i]
            if not (raw > 0.0):
                continue
            candidates.append(raw)
        if not candidates:
            return 0.0, False
        return self._gate_distance(min(candidates), now_ns), True

    # ── 1周期ぶんの処理（純粋関数。バスを知らない） ──

    def process_cycle(self, frame: np.ndarray | None, *, roi: TargetRoiCtrl | None,
                      scan: Scan | None, now_ns: int) -> TargetTrack:
        if roi is not None and roi.clear_seq > self._last_clear_seq:
            self._last_clear_seq = roi.clear_seq
            self._enter_idle()

        just_selected = False
        if (roi is not None and roi.select_seq > self._last_select_seq
                and frame is not None):
            if self._try_select(frame, roi):
                self._last_select_seq = roi.select_seq
                just_selected = True

        if self._state == "IDLE":
            return TargetTrack(tracking=False)

        # ── TRACKING ──
        # 選択した直後は`_try_select()`が既にinit()先のbbox/bearingを
        # `_last_*`へ反映済みなので、**同じフレームに対してもう一度
        # `update()`は呼ばない**（二重推論・不要な探索によるブレを避ける）
        ok = False
        box = (0.0, 0.0, 0.0, 0.0)
        if not just_selected and frame is not None:
            self._update_intrinsics(frame.shape[1], frame.shape[0])
            if self._tracker is not None:
                try:
                    ok, box = self._tracker.update(frame)
                except Exception:                                      # noqa: BLE001
                    ok = False

        confidence = 0.0
        if just_selected:
            self._lost_since_ns = None
            self._miss_streak = 0
        elif ok:
            h, w = frame.shape[:2]                # `ok` は frame is not None の中でしか立たない
            x, y, bw, bh = box
            cx_px, cy_px = x + bw / 2.0, y + bh / 2.0
            self._last_bbox = (cx_px / w, cy_px / h, bw / w, bh / h)
            self._last_bearing = self._bearing_for_pixel(cx_px)
            self._lost_since_ns = None
            self._miss_streak = 0
            confidence = self._tracker.score() if self._tracker is not None else 0.0
        else:
            # ★ちらつき吸収。`_LOST_DEBOUNCE_FRAMES`回連続で失敗するまでは
            # `lost`にせず、直前のbearing/bboxを保持したまま様子を見る
            self._miss_streak += 1
            if self._miss_streak >= _LOST_DEBOUNCE_FRAMES and self._lost_since_ns is None:
                self._lost_since_ns = now_ns

        lost = self._lost_since_ns is not None
        lost_ms = (now_ns - self._lost_since_ns) / 1e6 if lost else 0.0

        if lost and lost_ms >= _GIVE_UP_MS:
            # ★見失いを諦める。`_enter_idle()`で`tracking=False`に戻すので、
            # GUIは「対象が選択されていません」表示に戻り、選び直しを促す
            # （`FollowObject.plan()`も`tracking=False`を見て自身の状態をresetする）
            self._enter_idle()
            return TargetTrack(tracking=False)

        distance, distance_valid = (0.0, False)
        if not lost:
            distance, distance_valid = self._fuse_lidar(scan, now_ns)

        cx, cy, bw_n, bh_n = self._last_bbox
        return TargetTrack(tracking=True, lost=lost, lost_ms=lost_ms,
                           bearing=self._last_bearing, distance=distance,
                           distance_valid=distance_valid,
                           bbox_cx=cx, bbox_cy=cy, bbox_w=bw_n, bbox_h=bh_n,
                           confidence=confidence)

    def reset(self) -> None:
        self._enter_idle()
        self._last_select_seq = 0
        self._last_clear_seq = 0

    # ── ループ（実バス配線） ──

    def run(self, *, sub, pub, duration_s: float | None = None) -> None:
        self._running = True
        t_end = time.monotonic() + duration_s if duration_s else None
        next_hb = time.monotonic_ns()
        reader = FrameReader()
        seq = 0
        try:
            while self._running:
                if t_end and time.monotonic() >= t_end:
                    break
                for _ in sub.poll(20):
                    pass  # `latest` を見るだけなので中身の処理は不要

                roi = sub.latest.get(TOPIC_TRACK_ROI)
                scan = sub.latest.get(TOPIC_SCAN)
                ref: ImageRef | None = sub.latest.get(TOPIC_IMAGE_FRONT)
                frame = None
                t_capture = 0
                if ref is not None:
                    got = reader.read(ref)
                    if got is not None:
                        frame, t_capture = got

                now = time.monotonic_ns()
                st = self.process_cycle(frame, roi=roi, scan=scan, now_ns=now)
                st.seq = seq
                st.t_capture = t_capture
                pub.send(TOPIC_TRACK_TARGET, st)
                seq += 1

                if now >= next_hb:
                    next_hb = now + NS // HB_HZ
                    pub.send(TOPIC_HB_PREFIX + "cam_track", HbMsg(node="cam_track"))
        finally:
            reader.close()

    def stop(self) -> None:
        self._running = False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models-dir", default=str(DEFAULT_MODELS_DIR),
                    help=f"NanoTrackのONNXを探すディレクトリ（既定 {DEFAULT_MODELS_DIR}）")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    from raspi.bus import LATEST, Publisher, Subscriber

    node = CamTrackNode(models_dir=args.models_dir)
    pub = Publisher("cam_track")
    sub = Subscriber({TOPIC_TRACK_ROI: LATEST, TOPIC_SCAN: LATEST, TOPIC_IMAGE_FRONT: LATEST})

    print(f"# cam_track_node  publish {pub.endpoint}  track/target へ配信")
    if node._factory.available:
        print(f"# NanoTrack: {node.models_dir}")
    else:
        print(f"# ★{node._factory.error}（選択しても追跡は開始しない）", file=sys.stderr)

    def _shutdown(*_):
        node.stop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _shutdown)

    try:
        node.run(sub=sub, pub=pub, duration_s=args.duration)
    finally:
        sub.close()
        pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
