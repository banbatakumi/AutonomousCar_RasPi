"""camera_node — picamera2 で撮って共有メモリリングに書く。

    .venv/bin/python -m raspi.nodes.camera_node                      # 2台・640x480
    .venv/bin/python -m raspi.nodes.camera_node --cameras 0
    .venv/bin/python -m raspi.nodes.camera_node --size 1640x1232 --duration 30

やること:

- カメラごとに `FrameRing`（共有メモリ N枚）を作り、撮ったフレームを書き込む
- **カメラごとに独立したスレッドで取得する。** 1スレッドで順に取ると、
  2台目は常に1フレーム古いものを掴む（実測 5.6ms → 38.1ms）
- `t_capture` には picamera2 の `SensorTimestamp` をそのまま入れる。
  **これは Pi の CLOCK_MONOTONIC 基準**であることを実機で確認済みなので変換不要
- フレームの説明（`FrameDesc`）を `image/front` `image/rear` に publish する。
  **画素はバスに流さない**（下流は共有メモリからゼロコピーで読む）

カメラ番号とトピックの対応は `CAM_TOPIC`。**取り付けを入れ替えたらここだけ直す。**

共有メモリの名前は `surge_cam0` / `surge_cam1`。読み手は:

    python3 -m raspi.tools.shm_view surge_cam0
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import FrameRing  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.core.cleanup import quiet_close  # noqa: E402

SHM_PREFIX = "surge_cam"

#: `_grab()` が連続してこの回数失敗したら、そのカメラは諦めてスレッドを終了する
MAX_CONSECUTIVE_GRAB_FAILURES = 10


def _should_give_up(consecutive_failures: int, max_consecutive: int) -> bool:
    """`_grab()` の連続失敗回数が閾値に達したかどうか。picamera2非依存の純粋関数。"""
    return consecutive_failures >= max_consecutive
DEFAULT_SLOTS = 8

#: カメラ番号 → バスのトピックと役割名。**取り付けを入れ替えたらここだけ直す**
CAM_TOPIC = {0: ("image/front", "front"), 1: ("image/rear", "rear")}

#: カメラごとの下端カット率。前方はボンネット等が映り込む下1/4を、
#: 後方は下1/16だけ軽く除去する。**ISP の ScalerCrop で最初から読み出し範囲を
#: 絞るので、余分な画素は main ストリームに出てこない**（後段でスライスして
#: 捨てるより、共有メモリ書き込み・下流の処理・帯域がその分だけ軽くなる）
#: 値は `config/vehicle.toml` の `sensors.cam_front/cam_rear.bottom_crop` が正
#: （GUI の進路ガイド `CameraView.tsx` もここを参照して主点補正する。二重管理しない）。
_vehicle = Vehicle.load()
CAM_BOTTOM_CROP = {0: _vehicle.cam_front_bottom_crop, 1: _vehicle.cam_rear_bottom_crop}

#: **libcamera の形式名はメモリ上のバイト順ではない。**
#: 32bit ワードにパックしたときの並びを指すので、リトルエンディアンの
#: メモリ上では逆順になる。実測で確認済み: `format="RGB888"` で赤い物を撮ると
#: 配列の ch2 が最大になる（＝ ch0=B, ch1=G, ch2=R）。
#:
#: リングには**メモリ上の実際の並び**を記録する。下流が `fmt` を見て
#: そのまま解釈できることを優先する（名前を信じて色が入れ替わる事故を防ぐ）。
_MEMORY_ORDER = {
    "RGB888": "BGR888",
    "BGR888": "RGB888",
    "XRGB8888": "XBGR8888",
    "XBGR8888": "XRGB8888",
}


def memory_format(libcamera_name: str) -> str:
    """libcamera の形式名 → メモリ上のバイト順の名前。"""
    return _MEMORY_ORDER.get(libcamera_name, libcamera_name)


def full_fov_sensor_size(cam, main_size: tuple[int, int]) -> tuple[int, int] | None:
    """`main_size` を満たすフル画角のセンサーモードのサイズを選ぶ。

    **`main` の size だけ指定すると、picamera2 はそのサイズにちょうど一致する
    センサーネイティブモードを選ぶことがある。** IMX219 では 640x480 モードが
    センサー中央 1280x960 だけを切り出す望遠クロップで、フル画角 3280x2464 の
    約39%しか写らない（実機で `ScalerCrop=(1000,752,1280,960)` と確認済み）。
    `crop_limits` がセンサー全域と一致するモードだけを候補にし、要求解像度を
    満たす最小のものを選ぶことで、ISP 側の縮小でフル画角を保つ。
    """
    full = tuple(cam.camera_properties.get("PixelArraySize", (0, 0)))
    candidates = [m for m in cam.sensor_modes
                  if tuple(m["crop_limits"][2:]) == full]
    if not candidates:
        return None
    fits = [m for m in candidates
            if m["size"][0] >= main_size[0] and m["size"][1] >= main_size[1]]
    best = min(fits, key=lambda m: m["size"][0] * m["size"][1]) if fits \
        else max(candidates, key=lambda m: m["size"][0] * m["size"][1])
    return best["size"]


def bottom_cropped(cam, size: tuple[int, int],
                   fraction: float) -> tuple[tuple[int, int], tuple[int, int, int, int] | None]:
    """下端を `fraction` だけ切った main size と ScalerCrop を返す。

    **配列を受け取ってから下端をスライスするのではなく、ISP に最初から
    小さい画を作らせる。** main size をその分小さくして ScalerCrop で
    上側だけを選ぶので、共有メモリへの書き込み量・下流の処理・帯域が
    そのぶん減る（CSI からのセンサー読み出し自体はセンサーモード次第で
    変わらないので、そこは削れない）。
    """
    if not fraction:
        return size, None
    full = tuple(cam.camera_properties.get("PixelArraySize", (0, 0)))
    if not all(full):
        return size, None
    fw, fh = full
    keep_h = round(fh * (1 - fraction))
    out_h = round(size[1] * (1 - fraction))
    return (size[0], out_h), (0, 0, fw, keep_h)


#: `CamStats.gaps_ms` の保持上限。中央値・最大値の統計に十分な標本数を
#: 残しつつ、systemd 配下の長時間稼働でメモリが際限なく増えないようにする
_MAX_GAPS_SAMPLES = 1000


@dataclass(slots=True)
class CamStats:
    frames: int = 0
    dropped: int = 0            #: libcamera 側で落ちた（FrameCount の飛び）
    write_ns_max: int = 0       #: 共有メモリへの1回の書き込みにかかった最大時間
    write_ns_total: int = 0
    last_t_capture: int = 0
    gaps_ms: deque[float] = field(default_factory=lambda: deque(maxlen=_MAX_GAPS_SAMPLES))

    def summary(self, elapsed: float) -> str:
        fps = self.frames / elapsed if elapsed > 0 else 0
        avg_us = (self.write_ns_total / self.frames /
                  1000) if self.frames else 0
        jitter = ""
        if len(self.gaps_ms) > 2:
            g = sorted(self.gaps_ms)
            jitter = f" 間隔 中央値{g[len(g)//2]:.2f}ms 最大{g[-1]:.2f}ms"
        return (f"{self.frames}枚 {fps:.1f}fps 落ち{self.dropped} "
                f"書込 平均{avg_us:.0f}μs 最大{self.write_ns_max/1000:.0f}μs{jitter}")


class CameraWorker(threading.Thread):
    """カメラ1台ぶんの取得ループ。**1台につき1スレッド。**"""

    def __init__(self, idx: int, size: tuple[int, int], fmt: str, fps: float | None,
                 n_slots: int, on_frame=None) -> None:
        super().__init__(name=f"cam{idx}", daemon=True)
        self.idx = idx
        self.size = size
        self.fmt = fmt
        self.fps = fps
        self.on_frame = on_frame
        self.stats = CamStats()
        self.error: Exception | None = None
        self._running = False
        self._last_frame_id = -1
        #: **`run()` のスレッドだけが picamera2 を呼ぶ。** `request_fps`/
        #: `request_enabled` は他スレッド（`CameraNode._config_loop`）から呼ばれるが、
        #: そこでは pending 変数を書き換えるだけで picamera2 には触らない
        #: （`Picamera2` はマルチスレッドからの同時呼び出しを想定していないため）
        self._cfg_lock = threading.Lock()
        self._pending_fps: float | None = None
        self._pending_enabled: bool | None = None
        self._enabled = True

        from picamera2 import Picamera2

        self.cam = Picamera2(idx)
        ctrl = {}
        if fps:
            us = int(1e6 / fps)
            ctrl["FrameDurationLimits"] = (us, us)
        size, crop_rect = bottom_cropped(
            self.cam, size, CAM_BOTTOM_CROP.get(idx, 0.0))
        if crop_rect:
            ctrl["ScalerCrop"] = crop_rect
        sensor_size = full_fov_sensor_size(self.cam, size)
        cfg_kwargs = {"main": {"size": size, "format": fmt},
                      "buffer_count": 6, "controls": ctrl}
        if sensor_size:
            cfg_kwargs["sensor"] = {"output_size": sensor_size}
        cfg = self.cam.create_video_configuration(**cfg_kwargs)
        self.cam.configure(cfg)

        # 実際に確定した幾何をリングに使う。要求と食い違うことがあるため
        main = self.cam.camera_configuration()["main"]
        self.size = main["size"]
        self.libcamera_fmt = main["format"]
        self.fmt = memory_format(self.libcamera_fmt)      # メモリ上の並びで記録する
        self.ring = FrameRing.create(f"{SHM_PREFIX}{idx}", self.size[0], self.size[1],
                                     self.fmt, n_slots=n_slots)

    def start_camera(self) -> None:
        self.cam.start()

    def run(self) -> None:
        self._running = True
        consecutive_failures = 0
        try:
            while self._running:
                self._apply_pending()
                if not self._enabled:
                    # **止めている間は capture_request を呼ばない。** `Picamera2.stop()`
                    # 済みなので呼んでも取れない上、CPU を無駄に回さないため
                    time.sleep(0.2)
                    continue
                try:
                    self._grab()
                    consecutive_failures = 0
                except Exception as e:
                    # 一過性のキャプチャエラー（バッファ取得タイムアウト等）1回では
                    # 諦めない。閾値を超えて連続したときだけ本当に壊れたと判断する
                    consecutive_failures += 1
                    self.error = e
                    if _should_give_up(consecutive_failures, MAX_CONSECUTIVE_GRAB_FAILURES):
                        raise
                    time.sleep(0.05)
        except Exception as e:                       # 1台落ちても他は回す
            self.error = e
        finally:
            self._running = False

    def _apply_pending(self) -> None:
        """他スレッドから来た希望（FPS・ON/OFF）を、このスレッドの中で picamera2 に反映する。"""
        with self._cfg_lock:
            fps = self._pending_fps
            self._pending_fps = None
            enabled = self._pending_enabled
            self._pending_enabled = None
        if fps is not None and fps != self.fps:
            self.fps = fps
            us = int(1e6 / fps)
            self.cam.set_controls({"FrameDurationLimits": (us, us)})
        if enabled is not None and enabled != self._enabled:
            self._enabled = enabled
            if enabled:
                self.cam.start()
            else:
                self.cam.stop()
            # stop/start で picamera2 の FrameCount が 0 に戻ることがある。
            # 前回値と比較して「大量に落ちた」と誤集計しないようにリセットする
            self._last_frame_id = -1

    def request_fps(self, fps: float) -> None:
        """**picamera2 には触らない。** 次のループで `_apply_pending` が反映する。"""
        with self._cfg_lock:
            self._pending_fps = fps

    def request_enabled(self, enabled: bool) -> None:
        """**picamera2 には触らない。** 次のループで `_apply_pending` が反映する。"""
        with self._cfg_lock:
            self._pending_enabled = enabled

    def _grab(self) -> None:
        req = self.cam.capture_request()
        try:
            md = req.get_metadata()
            t_cap = md.get("SensorTimestamp", 0)
            fid = md.get("FrameCount", -1)
            # make_array は DMA バッファへの view。**release() すると無効になる**ので、
            # 共有メモリへの書き込みは release より前に済ませる。
            # ここで release を先にすると、解放済みメモリを読むことになる
            arr = req.make_array("main")
            if not arr.flags["C_CONTIGUOUS"]:
                arr = _contig(arr)
            t0 = time.monotonic_ns()
            desc = self.ring.write(
                arr, t_capture_ns=t_cap, frame_id=max(fid, 0))
            dt = time.monotonic_ns() - t0
        finally:
            req.release()

        s = self.stats
        s.frames += 1
        s.write_ns_total += dt
        s.write_ns_max = max(s.write_ns_max, dt)
        if s.last_t_capture and t_cap:
            s.gaps_ms.append((t_cap - s.last_t_capture) / 1e6)
        s.last_t_capture = t_cap
        if self._last_frame_id >= 0 and fid > self._last_frame_id + 1:
            s.dropped += fid - self._last_frame_id - 1
        self._last_frame_id = fid

        if self.on_frame:
            self.on_frame(desc)

    def stop(self) -> None:
        self._running = False

    def close(self) -> None:
        with quiet_close("picamera2 のカメラ"):
            self.cam.stop()
            self.cam.close()
        self.ring.unlink()


def _contig(arr):
    import numpy as np

    return np.ascontiguousarray(arr)


class CameraNode:
    def __init__(self, indices: list[int], size, fmt: str, fps, n_slots: int,
                 on_frame=None, cfg_sub=None) -> None:
        self.workers = [CameraWorker(i, size, fmt, fps, n_slots, on_frame)
                        for i in indices]
        self._t_start = 0.0
        #: GUI/自動運転からの capture 設定（`cam/config`）を拾うための Subscriber。
        #: `--no-bus` やバス未接続なら None（既定 fps・常時 ON のまま動く）
        self._cfg_sub = cfg_sub
        self._cfg_thread: threading.Thread | None = None
        self._cfg_running = False

    def run(self, duration_s: float | None = None, status_cb=None) -> None:
        for w in self.workers:
            w.start_camera()
        time.sleep(0.5)                     # AE/AWB が落ち着くまで
        for w in self.workers:
            w.stats = CamStats()
            w.start()

        if self._cfg_sub is not None:
            self._cfg_running = True
            self._cfg_thread = threading.Thread(
                target=self._config_loop, name="camcfg", daemon=True)
            self._cfg_thread.start()

        self._t_start = time.monotonic()
        next_status = 0.0
        try:
            while any(w.is_alive() for w in self.workers):
                el = time.monotonic() - self._t_start
                if duration_s is not None and el >= duration_s:
                    break
                if status_cb and el >= next_status:
                    status_cb(self)
                    next_status = el + 1.0
                time.sleep(0.05)
        finally:
            self.stop()

    def _config_loop(self) -> None:
        """`cam/config`（`CamConfig`）を低頻度で受け、該当ワーカーに希望を伝える。

        **ここでは picamera2 を直接呼ばない。** `CameraWorker.request_fps`/
        `request_enabled` は pending 変数を書き換えるだけで、実際の適用は
        そのワーカー自身のスレッドが次のループで行う（`_apply_pending`）。
        """
        from raspi.msgs.types import TOPIC_CAM_CONFIG

        role_of = {w.idx: CAM_TOPIC.get(w.idx, (None, f"cam{w.idx}"))[1]
                  for w in self.workers}
        while self._cfg_running:
            try:
                events = self._cfg_sub.poll(200)
            except Exception:
                break
            for topic, msg in events:
                if topic != TOPIC_CAM_CONFIG:
                    continue
                for w in self.workers:
                    role = role_of[w.idx]
                    if role == "front":
                        w.request_fps(msg.front_fps)
                    elif role == "rear":
                        w.request_fps(msg.rear_fps)
                        w.request_enabled(msg.rear_enabled)

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self._t_start if self._t_start else 0.0

    def stop(self) -> None:
        self._cfg_running = False
        if self._cfg_thread is not None:
            self._cfg_thread.join(timeout=1.0)
        for w in self.workers:
            w.stop()
        for w in self.workers:
            w.join(timeout=2.0)

    def close(self) -> None:
        for w in self.workers:
            w.close()
        if self._cfg_sub is not None:
            self._cfg_sub.close()


def _status(node: CameraNode) -> None:
    el = node.elapsed
    parts = [f"cam{w.idx}: {w.stats.frames}枚 "
             f"{w.stats.frames / el if el else 0:.1f}fps" for w in node.workers]
    sys.stdout.write("\r" + " | ".join(parts) + "    ")
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cameras", default="0,1", help="使うカメラ番号（例 0 / 0,1）")
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--fmt", default="RGB888")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--slots", type=int, default=DEFAULT_SLOTS,
                    help="リングの枚数（既定8。読み手が遅れても上書きされにくくなる）")
    ap.add_argument("--duration", type=float, default=None, help="秒で自動終了")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--no-bus", action="store_true", help="ZeroMQ に配信しない")
    args = ap.parse_args()

    w, h = (int(v) for v in args.size.lower().split("x"))
    indices = [int(v) for v in args.cameras.split(",") if v.strip()]

    # ── バス配信（画素は流さない。共有メモリへの参照だけ） ──
    pub = None
    on_frame = None
    cfg_sub = None
    if not args.no_bus:
        try:
            from raspi.bus import LATEST, Publisher, Subscriber
            from raspi.msgs import ImageRef, TOPIC_CAM_CONFIG

            # **カメラごとに別スレッドから publish するので thread_safe が必須。**
            # ZeroMQ のソケットはスレッドセーフではない
            pub = Publisher("camera", thread_safe=True)

            def on_frame(desc):                              # noqa: F811
                idx = int(desc.name[len(SHM_PREFIX):] or 0)
                topic, role = CAM_TOPIC.get(
                    idx, (f"image/cam{idx}", f"cam{idx}"))
                pub.send(topic, ImageRef(
                    t_capture=desc.t_capture_ns, cam=role,
                    shm_name=desc.name, slot=desc.slot, ring_seq=desc.seq,
                    frame_id=desc.frame_id, width=desc.width, height=desc.height,
                    fmt=desc.fmt, stride=desc.stride, nbytes=desc.nbytes))

            print(f"# バス配信 {pub.endpoint} "
                  + " ".join(CAM_TOPIC.get(i, (f'image/cam{i}',))[0] for i in indices))

            # telemetry_node からの capture 設定（FPS上限・後方カメラON/OFF）を拾う。
            # **失敗しても撮像自体は続ける**（既定 fps・常時 ON のまま動くだけ）
            cfg_sub = Subscriber({TOPIC_CAM_CONFIG: LATEST})
        except ImportError as e:
            print(f"!! pyzmq/msgspec が無いのでバス無しで動く: {e}")
        except Exception as e:
            print(f"!! バスを開けない（バス無しで続行）: {e}", file=sys.stderr)

    try:
        node = CameraNode(indices, (w, h), args.fmt, args.fps, args.slots,
                          on_frame=on_frame, cfg_sub=cfg_sub)
    except Exception as e:
        print(f"カメラを開けない: {e}", file=sys.stderr)
        return 2

    signal.signal(signal.SIGINT, lambda *_: node.stop())

    for wk in node.workers:
        print(f"# cam{wk.idx} -> /dev/shm/{wk.ring.name}  "
              f"{wk.ring.width}x{wk.ring.height} {wk.ring.fmt} "
              f"×{wk.ring.n_slots}枚 ({wk.ring._shm.size / 1e6:.1f}MB)")
    print("# 読み手: python3 -m raspi.tools.shm_view "
          f"{SHM_PREFIX}{indices[0]}\n")

    try:
        node.run(duration_s=args.duration,
                 status_cb=None if args.quiet else _status)
    finally:
        el = node.elapsed
        print("\n\n=== 終了時の統計 ===")
        for wk in node.workers:
            print(f"cam{wk.idx}: {wk.stats.summary(el)}")
            if wk.error:
                print(f"  !! 異常終了: {wk.error}")
        if pub is not None:
            print(f"bus: {pub.sent} 件 publish")
            pub.close()
        node.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
