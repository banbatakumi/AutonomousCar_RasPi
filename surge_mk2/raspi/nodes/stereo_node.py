"""stereo_node — 前方ステレオ（右 = `image/front`、左 = `image/left`）から障害物の擬似 `Scan` と深度映像を作る。

    .venv/bin/python -m raspi.nodes.stereo_node
    .venv/bin/python -m raspi.nodes.stereo_node --algo sgbm --scale 0.5 --rate-hz 10

計画は `docs/stereo_plan.md` 段階4。作りは `cam_perception_node.py` と同じ（共有メモリを読み、
IDLE/ACTIVE で省電力、`hb/stereo`）。

## 出力

- `scan/stereo`（型 `Scan`）: 床より上・高さ 20cm 以下の点を、base_link から見た 1° の方位ごとの距離にしたもの。
  約束（**0 は見つからない ≠ 空き**、原点、`sector_seen`）は `raspi/core/stereo_match.py` の docstring
- `stereo/depth`（`StereoDepth`）: 視差を色にした JPEG（照合した行の帯だけ）。telemetry_node が `/ws/camera/depth` へ中継する

## いつ照合するか（IDLE/ACTIVE）

`cam/config` の `CamConfig.depth` が True の間だけ。決めるのは telemetry_node（`_desired_depth`:
ステレオを使う自動運転モードが選ばれていて ARM 中、または深度映像の視聴者がいる間）。
camera_node が左カメラを回すか（`CamConfig.stereo`）も同じ判断に従うので、照合しない間は左カメラも止まる。

## 左右の組

両方の最新の `ImageRef` が `synced`（左右の開始が 0.5ms 以内）のときだけ照合する。左右は別々に届くので、
**2台の最新が同じ瞬間になるまで待ってから**、両方のリングからそのコマを読む（`FrameReader.read_at`。片方が止まったら
`PAIR_WAIT_NS` 後に揃っている古い方で処理する）。揃っていない間（起動直後・fps を変えた直後の数秒〜数十秒）は
全セクタ `sector_seen=False` の空の `Scan` を出す。

## 遅れを短くする作り（`docs/stereo_plan.md` 段階4 の経緯）

頻度の上限は撮影時刻の間隔で判定する。`scan/stereo` を先に出し、深度映像（色付け＋JPEG）は視聴者がいるときだけ
別スレッドで作る（`DepthWorker`）。照合は 2 スレッド、後処理は C 版（`raspi/core/stereo_native.c`）。Pi 5 で撮影から
`scan/stereo` まで中央値 47ms（SGBM・1.5GHz・表示なし）。

## 照合の設定

既定は 640 幅の StereoSGBM（ブロック 5・視差 96）、行 150〜260、10Hz。段階3 の物差しの壁の記録で決めた（`stereo_plan.md` §段階3）。
- SGBM: StereoBM は無地の面で取り違えて偽の近い障害物を出した
- 行の下端 260: 平行化画像の約 255 行より下は左右とも自分の車体（カメラが前車軸より 14cm 後ろ）で、照合が成り立たず
  車体の取り違えが 0.3m 先の偽の障害物になっていた。そのため床が見えるのは中央で約 1.1m より先
- 行の上端 150: 照合の時間を削るため（130 から 2.7ms）。0.3m 先で高さ約 18cm、最も近い 0.21m で約 15cm まで見える
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

from raspi.core.auto_gate import IdlePacer  # noqa: E402
from raspi.core.frame_reader import FrameReader  # noqa: E402
from raspi.core.jpeg import make_encoder  # noqa: E402
from raspi.core.stereo_match import (MatchParams, ObstacleParams,  # noqa: E402
                                     colorize_disparity, disparity, fov_degrees, make_matcher,
                                     obstacles_from_disparity, to_gray)
from raspi.core.stereo_model import StereoRectifier  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.msgs import Heartbeat as HbMsg  # noqa: E402
from raspi.msgs import Scan, StereoDepth, StereoInfo  # noqa: E402
from raspi.msgs.types import (  # noqa: E402
    TOPIC_CAM_CONFIG,
    TOPIC_HB_PREFIX,
    TOPIC_IMAGE_FRONT,
    TOPIC_IMAGE_LEFT,
    TOPIC_SCAN_STEREO,
    TOPIC_STEREO_DEPTH,
    TOPIC_STEREO_INFO,
)

__all__ = ["StereoProcessor", "StereoNode"]

NS = 1_000_000_000
HB_HZ = 10
#: 左右を組にする撮影時刻の差の上限（`camera_node.STEREO_SYNC_TOL_NS` と同じ値）
PAIR_TOL_NS = 500_000
#: 2台の最新が同じ瞬間になるのを待つ上限（前回の照合から数える）。過ぎたら揃っている古い方の組で処理する
PAIR_WAIT_NS = 200_000_000
#: 照合する平行化画像の行（640×480 の座標）。上のモジュールの docstring
DEFAULT_ROWS = (150, 260)
RECT_HFOV_RAD = math.radians(100)
#: 深度映像の JPEG の幅の上限 [px]（表示専用）
DEPTH_JPEG_W = 320
#: OpenCV（照合）のスレッド数の既定。Pi 5 の実測で 1 → 2 で処理 44 → 32ms、制御系電流は差なし（A-B-A-B）
DEFAULT_THREADS = 2


class StereoProcessor:
    """左右の画像 → `(Scan, 視差, 平行化した左)`。バスを知らない純ロジック（テストはここを叩く）。"""

    def __init__(self, vehicle: Vehicle, *, match: MatchParams = MatchParams(),
                 obst: ObstacleParams = ObstacleParams(), rows: tuple[int, int] = DEFAULT_ROWS,
                 hfov: float = RECT_HFOV_RAD, depth_quality: int = 70) -> None:
        if vehicle.stereo is None or vehicle.cam_front_fisheye is None or vehicle.cam_left_fisheye is None:
            raise ValueError("ステレオの校正値が無い（stereo 配置で、tools/cam_calib の「ステレオ」を書いたか）")
        self.match, self.obst = match, obst
        self.rect = StereoRectifier(vehicle.cam_front_fisheye, vehicle.cam_left_fisheye, vehicle.stereo,
                                    640, 480, hfov, rows=rows,
                                    crop_right=vehicle.cam_front_bottom_crop,
                                    crop_left=vehicle.cam_left_bottom_crop)
        self.cam_pose = (vehicle.cam_front_x, vehicle.cam_front_y, vehicle.cam_front_z,
                         vehicle.cam_front_pitch, vehicle.cam_front_yaw)
        self.fov_degs = fov_degrees(self.rect, self.cam_pose)
        self.matcher = make_matcher(match)
        self._encode, _ = make_encoder(depth_quality)
        self.last_valid_frac = 0.0
        self.last_n_points = 0

    def process(self, img_right: np.ndarray, img_left: np.ndarray, fmt: str = "BGR888", *,
                t_capture_ns: int = 0, seq: int = 0) -> tuple[Scan, np.ndarray]:
        gr, gl = to_gray(img_right, fmt), to_gray(img_left, fmt)
        rr, rl = self.rect(gr, gl)                 # 行の帯だけを平行化する
        d = disparity(self.match, self.matcher, rl, rr)
        res = obstacles_from_disparity(self.rect, d, self.cam_pose, self.fov_degs, self.obst,
                                       scale=self.match.scale, skip_cols=self.match.num_disp)
        self.last_n_points = res.n_points
        self.last_valid_frac = res.valid_frac
        return (Scan(dist=res.dist, sector_seen=res.sector_seen, seq=seq, t_capture=t_capture_ns,
                     rot_speed_dps=0.0), d)

    def depth_jpeg(self, d: np.ndarray) -> bytes | None:
        """深度映像の JPEG（視差を色にしただけ。**測れなかった所は黒のまま**）。

        幅 `DEPTH_JPEG_W` 以下に縮めてから焼く（Pi 5 で 640 幅は 8ms、320 幅は 2ms）。
        """
        if self._encode is None:
            return None
        import cv2

        img = colorize_disparity(d, self.match.num_disp)
        if img.shape[1] > DEPTH_JPEG_W:
            img = cv2.resize(img, (DEPTH_JPEG_W, max(1, img.shape[0] * DEPTH_JPEG_W // img.shape[1])),
                             interpolation=cv2.INTER_NEAREST)
        return self._encode(img, "BGR")

    @property
    def near_m(self) -> float:
        """深度映像の赤の端（視差 = 探索幅）の距離 [m]。照合できる最も近い距離でもある。"""
        return float(self.rect.f * self.match.scale * self.rect.baseline / self.match.num_disp)

    @staticmethod
    def failed(seq: int = 0, reason: str = "") -> Scan:
        """照合できない周期。全セクタ見えていない（`sector_seen=False`）・距離なし。"""
        return Scan(dist=[0.0] * 360, sector_seen=[False] * 12, seq=seq, reason=reason)


class StereoNode:
    def __init__(self, proc: StereoProcessor | None, *, rate_hz: float = 10.0,
                 depth_every: int = 1, init_error: str = "") -> None:
        self.proc = proc
        self.period_ns = int(NS / rate_hz)
        self.depth_every = max(1, int(depth_every))
        self.init_error = init_error
        self._readers = {"front": FrameReader(), "left": FrameReader()}
        self._running = False
        self._active = False
        self._status = "idle"
        self._last_target = -1
        self._last_run_ns = 0
        self.count = 0
        #: 深度映像を作る材料 `(視差, 平行化した左, t_capture)`。`scan/stereo` を出した**後で**別スレッドが焼く
        #: （`publish_depth`・`DepthWorker`）
        self._pending_depth: tuple | None = None
        #: 深度映像（色付け＋JPEG）を作るのにかかった時間 [ms]
        self.view_ms = 0.0
        #: 遅れの内訳 [ms]（`hb/stereo` に出す。合否は「撮影 → `scan/stereo` を出すまで」50ms 以内）
        #:   cam  = 撮影時刻 → 2台目の `ImageRef` が publish されるまで（露光・読み出し・ISP。camera_node 側）
        #:   wait = それ → 組を読み始めるまで（このノードが起きる）
        #:   read = 共有メモリから同じ瞬間の組を読む（640×480 カラー2枚のコピー）
        #:   proc = 照合〜擬似 Scan を作り終えるまで
        #:   lat  = 撮影時刻 → `scan/stereo` を出すまで
        self.cam_ms = self.wait_ms = self.read_ms = self.proc_ms = self.latency_ms = 0.0
        #: 直近に照合した組の 左 − 右 の撮影時刻の差 [ns]（`StereoInfo.dt_lr_ns`）
        self.dt_lr_ns = 0

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        for r in self._readers.values():
            r.close()

    def step(self, latest: dict, now_ns: int) -> Scan | None:
        """1周ぶん。publish すべき `Scan` を返す（照合しなかった周期は None）。"""
        cfg = latest.get(TOPIC_CAM_CONFIG)
        active = bool(cfg is not None and getattr(cfg, "depth", False))
        if active != self._active:
            self._active = active
            print(f"# stereo {'照合開始' if active else '照合停止'}", flush=True)
        if not active:
            self._status = "idle"
            return StereoProcessor.failed(reason="深度を使う者がいない（照合停止）")
        if self.proc is None:
            self._status = f"failed: {self.init_error}"
            return StereoProcessor.failed(reason=self.init_error)
        rf, rl = latest.get(TOPIC_IMAGE_FRONT), latest.get(TOPIC_IMAGE_LEFT)
        if rf is None or rl is None:
            self._status = "failed: 左右の映像が来ていない"
            return StereoProcessor.failed(reason="左右の映像が来ていない（image/front・image/left）")
        if not (rf.synced and rl.synced):
            self._status = "wait: 同期待ち"
            return StereoProcessor.failed(reason="左右の同期待ち")
        # **頻度の上限は撮影時刻の間隔で判定する**（周期の 1/4 だけ早い組も通す）。今の時刻で判定すると、
        # 30fps の3コマ（100.0ms）と 10Hz の周期が同じなので到着の揺らぎで見送って `poll` の時間切れ（20ms）まで
        # 待ったり、ループが塞がっていた間に届いた古い組を空いた瞬間に拾ったりした（実機で待ち 20ms・66.7ms 間隔）
        newest = max(rf.t_capture, rl.t_capture)
        if self._last_target >= 0 and newest - self._last_target < self.period_ns - self.period_ns // 4:
            return None
        if abs(rf.t_capture - rl.t_capture) > PAIR_TOL_NS and now_ns - self._last_run_ns < PAIR_WAIT_NS:
            # **2台の最新が同じ瞬間になるまで待つ。** 左右は同時に撮れているが別々に届くので、先に届いた
            # 片方で起きると、もう片方はまだ1つ前のコマ。ここで古い方に合わせると1コマ（33ms）古い組を
            # 処理してしまう（実機で遅れの 90% 点が 30ms 悪化していた）。もう片方は数 ms 以内に届き、
            # `poll` がそこで起こす。片方が止まったときに待ち続けないよう、`PAIR_WAIT_NS` を過ぎたら古い方で処理する
            return None
        target = min(rf.t_capture, rl.t_capture)
        if target == self._last_target:
            return None
        t_start = time.monotonic_ns()
        got_r = self._readers["front"].read_at(rf, target, PAIR_TOL_NS)
        got_l = self._readers["left"].read_at(rl, target, PAIR_TOL_NS) if got_r else None
        if got_r is None or got_l is None:
            self._status = "failed: 同じ瞬間の組を読めない"
            return StereoProcessor.failed(reason="同じ瞬間の左右を読めない")
        self._last_target = target
        self._last_run_ns = now_ns
        self.dt_lr_ns = got_l[1] - got_r[1]
        t0 = time.monotonic_ns()
        self.read_ms = (t0 - t_start) / 1e6
        try:
            st, d = self.proc.process(got_r[0], got_l[0], rf.fmt, t_capture_ns=got_r[1],
                                             seq=self.count)
        except Exception as e:                                 # noqa: BLE001
            print(f"# stereo process() が例外: {e}", file=sys.stderr, flush=True)
            self._status = f"failed: 照合が例外: {e}"
            return StereoProcessor.failed(reason=f"照合が例外: {e}")
        self.proc_ms = (time.monotonic_ns() - t0) / 1e6
        t_arrive = max(getattr(rf, "t_pub", 0), getattr(rl, "t_pub", 0))
        if t_arrive:
            self.cam_ms = (t_arrive - got_r[1]) / 1e6
            self.wait_ms = (t_start - t_arrive) / 1e6
        self.count += 1
        view = bool(getattr(cfg, "depth_view", True))
        if view and (self.count == 1 or self.count % self.depth_every == 0):
            self._pending_depth = (d, got_r[1])
        elif not view:
            self.view_ms = 0.0                       # 表示していない（`hb/stereo` に前の値を残さない）
        return st

    def take_depth(self) -> tuple | None:
        """溜めた深度映像の材料を取り出す（無ければ None）。"""
        item, self._pending_depth = self._pending_depth, None
        return item

    def render_depth(self, item) -> StereoDepth | None:
        """材料 → `StereoDepth`。照合のループを塞がないよう `DepthWorker` のスレッドで呼ぶ。"""
        if self.proc is None:
            return None
        d, t_cap = item
        t0 = time.monotonic_ns()
        jpg = self.proc.depth_jpeg(d)
        self.view_ms = (time.monotonic_ns() - t0) / 1e6
        return None if jpg is None else StereoDepth(jpeg=jpg, near_m=self.proc.near_m, t_capture=t_cap)

    def info(self, st: Scan) -> StereoInfo | None:
        """今回の組の診断（`StereoInfo`。開発用に `.mcap` へ残る）。照合できていなければ None。"""
        p = self.proc
        if p is None:
            return None
        m, q = p.match, p.obst
        return StereoInfo(
            t_capture=st.t_capture, seq=st.seq, dt_lr_ns=self.dt_lr_ns,
            cam_ms=self.cam_ms, wait_ms=self.wait_ms, read_ms=self.read_ms, proc_ms=self.proc_ms,
            latency_ms=self.latency_ms, n_points=p.last_n_points, valid_frac=p.last_valid_frac,
            algo=m.algo, scale=m.scale, num_disp=m.num_disp, block=m.block, rows=list(p.rect.rows),
            z_min=q.z_min, z_max=q.z_max, r_min=q.r_min, r_max=q.r_max, min_points=q.min_points,
            near_pct=q.near_pct, min_height_span=q.min_height_span, slice_m=q.slice_m,
            cam_pose=[float(x) for x in p.cam_pose])

    def publish_depth(self, pub) -> None:
        """溜めた深度映像を**その場で**焼いて出す（テスト・スレッドを使わない呼び出し用）。"""
        item = self.take_depth()
        if item is not None and (msg := self.render_depth(item)) is not None:
            pub.send(TOPIC_STEREO_DEPTH, msg)

    def _ok_status(self) -> str:
        p = self.proc
        return (f"ok 遅れ{self.latency_ms:.0f}ms(カメラ{self.cam_ms:.0f}+待ち{self.wait_ms:.0f}"
                f"+読み{self.read_ms:.0f}+処理{self.proc_ms:.0f}) 表示{self.view_ms:.0f}ms "
                f"視差{p.last_valid_frac * 100:.0f}% 点{p.last_n_points}")

    def run(self, *, sub, pub, duration_s: float | None = None) -> None:
        self._running = True
        t_end = time.monotonic() + duration_s if duration_s else None
        next_hb = time.monotonic_ns()
        pacer = IdlePacer()
        worker = DepthWorker(self)
        try:
            while self._running:
                if t_end and time.monotonic() >= t_end:
                    break
                for _ in sub.poll(pacer.poll_timeout_ms(self._active)):
                    pass
                now = time.monotonic_ns()
                n_before = self.count
                st = self.step(sub.latest, now)
                if st is not None and pacer.should_publish(self._active, now):
                    pub.send(TOPIC_SCAN_STEREO, st)
                    if self.count != n_before:              # 今回照合した
                        self.latency_ms = (time.monotonic_ns() - st.t_capture) / 1e6
                        self._status = self._ok_status()
                        if (info := self.info(st)) is not None:
                            pub.send(TOPIC_STEREO_INFO, info)
                item = self.take_depth()
                if item is not None:
                    worker.submit(item)
                # 別スレッドが焼き終えた深度映像を出す（`Publisher` は1つのスレッドからだけ使う）
                if (msg := worker.take_done()) is not None:
                    pub.send(TOPIC_STEREO_DEPTH, msg)
                if now >= next_hb:
                    next_hb = now + NS // HB_HZ
                    pub.send(TOPIC_HB_PREFIX + "stereo", HbMsg(node="stereo", detail=self._status))
                pacer.idle_sleep(self._active)
        finally:
            worker.stop()


class DepthWorker:
    """深度映像（色付け＋JPEG）を別スレッドで作る。**照合のループを塞がないため。**

    同じスレッドで作るとループが止まり、その間に届いた組の処理が遅れた（穴埋めがあった頃は約 20ms、実機で待ち 20ms）。
    cv2 と JPEG の符号化は処理中に GIL を手放すので並行して進む。材料は最新の1つだけ持ち（作っている間に来た
    古い材料は捨てる）、できあがりはメインのループが取り出して publish する。
    """

    def __init__(self, node: StereoNode) -> None:
        import threading

        self._node = node
        self._cv = threading.Condition()
        self._item = None
        self._done: StereoDepth | None = None
        self._stop = False
        self._th = threading.Thread(target=self._loop, name="stereo-depth", daemon=True)
        self._th.start()

    def submit(self, item) -> None:
        with self._cv:
            self._item = item
            self._cv.notify()

    def take_done(self) -> StereoDepth | None:
        with self._cv:
            msg, self._done = self._done, None
        return msg

    def stop(self) -> None:
        with self._cv:
            self._stop = True
            self._cv.notify()
        self._th.join(timeout=1.0)

    def _loop(self) -> None:
        while True:
            with self._cv:
                while self._item is None and not self._stop:
                    self._cv.wait()
                if self._stop:
                    return
                item, self._item = self._item, None
            try:
                msg = self._node.render_depth(item)
            except Exception as e:                             # noqa: BLE001
                print(f"# stereo 深度映像を作れない: {e}", file=sys.stderr, flush=True)
                continue
            with self._cv:
                self._done = msg


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--algo", choices=("bm", "sgbm"), default=MatchParams.algo)
    ap.add_argument("--scale", type=float, default=1.0, help="照合する画像の縮尺（0.5 で 320 幅）")
    ap.add_argument("--num-disp", type=int, default=96, help="視差の探索幅（縮めた画像の画素。16 の倍数）")
    ap.add_argument("--block", type=int, default=MatchParams.block, help="照合の窓（SGBM は 5、BM なら 15 前後）")
    ap.add_argument("--rows", default=f"{DEFAULT_ROWS[0]}:{DEFAULT_ROWS[1]}",
                    help="照合する平行化画像の行 V0:V1（640×480 の座標）")
    ap.add_argument("--rate-hz", type=float, default=10.0, help="照合の上限頻度")
    ap.add_argument("--depth-every", type=int, default=1, help="深度映像を照合 N 回に 1 回だけ配る")
    ap.add_argument("--z-min", type=float, default=ObstacleParams.z_min)
    ap.add_argument("--z-max", type=float, default=ObstacleParams.z_max)
    ap.add_argument("--r-max", type=float, default=ObstacleParams.r_max)
    ap.add_argument("--threads", type=int, default=DEFAULT_THREADS,
                    help=f"OpenCV（照合）のスレッド数（既定 {DEFAULT_THREADS}）")
    ap.add_argument("--duration", type=float, default=None)
    args = ap.parse_args()

    import cv2

    from raspi.bus import LATEST, Publisher, Subscriber

    cv2.setNumThreads(args.threads)
    v0, v1 = (int(x) for x in args.rows.split(":"))
    proc, err = None, ""
    try:
        proc = StereoProcessor(Vehicle.load(),
                               match=MatchParams(args.algo, args.scale, args.num_disp, args.block),
                               obst=ObstacleParams(z_min=args.z_min, z_max=args.z_max, r_max=args.r_max),
                               rows=(v0, v1))
    except ValueError as e:
        err = str(e)
        print(f"!! {err}。照合せずに待つ", file=sys.stderr, flush=True)
    node = StereoNode(proc, rate_hz=args.rate_hz, depth_every=args.depth_every, init_error=err)
    pub = Publisher("stereo")
    sub = Subscriber({TOPIC_IMAGE_FRONT: LATEST, TOPIC_IMAGE_LEFT: LATEST, TOPIC_CAM_CONFIG: LATEST})
    from raspi.core.stereo_native import load as load_native

    # 後処理の C 版は起動時にビルドしておく（最初のコマの処理中に 1 秒ほど止まらないように）
    native = load_native() is not None
    print(f"# stereo_node  publish {pub.endpoint}  scan/stereo・stereo/depth へ配信  "
          f"スレッド {args.threads}・後処理 {'C' if native else 'numpy'}")

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: node.stop())
    try:
        node.run(sub=sub, pub=pub, duration_s=args.duration)
    finally:
        node.close()
        sub.close()
        pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
