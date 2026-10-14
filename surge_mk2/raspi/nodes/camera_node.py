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
指定した番号のカメラが一部見えないときは、**見えているカメラだけで起動を続ける**
（欠けた側の配信は止まり、`hb/camera` の detail に `<役割>=missing` と出る）。
どの番号をどの役割に使ったかは起動ログに出す。`--by-i2c` を付けると、番号ではなく
接続先の I2C アドレス（`I2C_ADDR_ROLE`）で前後を決める（libcamera の列挙順が入れ替わる
環境向け。既定は番号どおり）。

共有メモリの名前は `surge_cam0` / `surge_cam1`。読み手は:

    python3 -m raspi.tools.shm_view surge_cam0
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from raspi.bus import FrameRing  # noqa: E402
from raspi.core.cam_tuning import check_sensor, lens_tuning, pick_name  # noqa: E402
from raspi.core.vehicle import Vehicle  # noqa: E402
from raspi.core.cleanup import quiet_close  # noqa: E402

SHM_PREFIX = "surge_cam"

#: ログ（stdout/stderr）の行頭に日時を付ける。`logs/*.out` には日時が無く、再起動の
#: 時刻や間隔を後から追えなかったため
class _Stamped:
    def __init__(self, stream) -> None:
        self._stream = stream
        self._bol = True

    def write(self, text: str) -> int:
        out = []
        for piece in text.splitlines(keepends=True):
            if self._bol and piece.strip() and not piece.startswith("\r"):
                out.append(time.strftime("%Y-%m-%d %H:%M:%S "))
            out.append(piece)
            self._bol = piece.endswith("\n")
        self._stream.write("".join(out))
        return len(text)

    def __getattr__(self, name):
        return getattr(self._stream, name)


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
#: 値は `config/vehicle.toml` の選ばれたレンズの `bottom_crop`（`[sensors.cam_*.lenses.<レンズ>]`）が正
#: （GUI の進路ガイド `CameraView.tsx` もここを参照して主点補正する。二重管理しない）。
_vehicle = Vehicle.load()
CAM_BOTTOM_CROP = {0: _vehicle.cam_front_bottom_crop, 1: _vehicle.cam_rear_bottom_crop}

#: カメラごとの ISP チューニングの差分（選ばれたレンズの `isp_tuning`。"" ＝ センサー標準のまま）。
#: 標準の色むら補正は純正レンズ用で、広角レンズでは周辺がピンクに写る（`raspi/core/cam_tuning.py`）。
#: **プロセスに1つしか効かない**ので、`main` が最初のカメラを開く前に `prepare_isp_tuning` で1つ選ぶ
CAM_ISP_TUNING = {0: _vehicle.cam_front_isp_tuning, 1: _vehicle.cam_rear_isp_tuning}

#: このプロセスの全カメラに渡す (チューニング, そのセンサー名)。`(None, "")` ＝ センサー標準
_isp_tuning: tuple[dict | None, str] = (None, "")


def prepare_isp_tuning(indices: list[int]) -> None:
    """開くカメラのレンズから ISP チューニングを1つ選ぶ。**最初のカメラを開く前に呼ぶ。**

    無い・読めないときは標準のまま（`lens_tuning` が理由を stderr に出す）。
    """
    global _isp_tuning
    from picamera2 import Picamera2

    _isp_tuning = lens_tuning(pick_name([CAM_ISP_TUNING.get(i, "") for i in indices]),
                              Picamera2.load_tuning_file)

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

#: `capture_request()` がブロックしたまま戻らないハングを検出するまでの猶予[秒]。
#: **スレッド自体は`is_alive()`のままなので、生死監視だけでは検出できない**
#: （issue #4）。`CamStats.last_grab_ns`（`_grab()`が成功するたびに更新）を
#: 見て、これだけ進んでいなければハングとみなしてノード全体を終了させる
#: 実際の猶予は 1台ごとに `max(これ, 3×フレーム周期)`（1fps 設定でも誤検出しない）
_CAPTURE_HANG_S = 1.0


#: 取得間隔がフレーム周期のこの倍率を超えたら、その間にフレームが落ちたとみなす
DROP_GAP_FACTOR = 1.5


def count_dropped(gap_ns: float, period_ns: float) -> int:
    """連続する2フレームの `SensorTimestamp` の間隔から、その間に落ちたフレーム数を推定する。

    picamera2 のメタデータに `FrameCount` は無いので、`SensorTimestamp` の間隔が周期の
    `DROP_GAP_FACTOR` 倍を超えたとき、周期で割った分から1を引いた枚数（最低1）を落ちとみなす。
    周期が不明（0 以下）なら数えない。
    """
    if period_ns <= 0 or gap_ns <= period_ns * DROP_GAP_FACTOR:
        return 0
    return max(1, round(gap_ns / period_ns) - 1)


#: 終了時の後始末（picamera2 の stop/close 等）を待つ上限 [秒]。`capture_request()` が
#: 本当にハングしていると `Picamera2.close()` も戻らないことがあるため、これを超えたら
#: プロセスごと強制終了して systemd に再起動させる
CLOSE_TIMEOUT_S = 5.0


def run_bounded(fn, timeout_s: float) -> bool:
    """`fn()` を別スレッドで実行し、`timeout_s` 以内に終わったか（例外で終わった場合も True）を返す。"""
    def body() -> None:
        try:
            fn()
        except Exception as e:                       # noqa: BLE001
            print(f"!! 後始末中の例外: {type(e).__name__}: {e}", file=sys.stderr)

    t = threading.Thread(target=body, name="camclose", daemon=True)
    t.start()
    t.join(timeout_s)
    return not t.is_alive()


#: 接続先の I2C アドレス（`Picamera2.global_camera_info()` の `Id` に含まれる文字列）→ 役割。
#: Pi 5 の CAM0/CAM1 コネクタの I2C バスで、前がこの機体の実測（`--by-i2c` のときだけ使う）
I2C_ADDR_ROLE = {"i2c@88000": "front", "i2c@80000": "rear"}


def devices_by_i2c(infos: list[dict], wanted_roles: list[str]) -> dict[str, int]:
    """`global_camera_info()` の結果から 役割 → libcamera の番号 を作る（見つかった役割だけ）。"""
    out: dict[str, int] = {}
    for dev, info in enumerate(infos):
        ident = str(info.get("Id", ""))
        for addr, role in I2C_ADDR_ROLE.items():
            if addr in ident and role in wanted_roles and role not in out:
                out[role] = dev
    return out


#: `hb/camera`（生存申告。`detail` に実 fps と落ち数）を出す周期。撮像は別スレッドなので
#: 申告が遅くても困らず、待機中の消費を増やさないよう他ノード（10Hz）より低くしてある
HB_HZ = 2
#: `hb/camera` に載せる実 fps を測り直す間隔 [s]（短いと 10fps 時に ±1枚の粗さが出る）
HB_FPS_WINDOW_S = 2.0


def make_hb_detail():
    """`hb/camera` の `detail` を作る関数を返す（例 `front=29.9fps drop=0 rear=off`）。

    fps は `HB_FPS_WINDOW_S` ごとの `CamStats.frames` の増分から出す。"""
    last: dict[int, tuple[float, int]] = {}
    fps: dict[int, float] = {}

    def detail(node) -> str:
        now = time.monotonic()
        parts = []
        for i in getattr(node, "missing", []):
            parts.append(f"{CAM_TOPIC.get(i, ('', f'cam{i}'))[1]}=missing")
        for wk in node.workers:
            role = CAM_TOPIC.get(wk.idx, ("", f"cam{wk.idx}"))[1]
            frames = wk.stats.frames
            t0, f0 = last.get(wk.idx, (now, frames))
            if wk.idx not in last or frames < f0:
                last[wk.idx] = (now, frames)
            elif now - t0 >= HB_FPS_WINDOW_S:
                fps[wk.idx] = (frames - f0) / (now - t0)
                last[wk.idx] = (now, frames)
            if not wk._enabled:
                parts.append(f"{role}=off")
            elif wk.idx in fps:
                parts.append(f"{role}={fps[wk.idx]:.1f}fps drop={wk.stats.dropped}")
            else:
                parts.append(f"{role}=-")
        return " ".join(parts)

    return detail


@dataclass(slots=True)
class CamStats:
    frames: int = 0
    dropped: int = 0            #: 落ちたフレーム数の推定（SensorTimestamp の間隔が周期の1.5倍超）
    write_ns_max: int = 0       #: 共有メモリへの1回の書き込みにかかった最大時間
    write_ns_total: int = 0
    last_t_capture: int = 0
    #: `_grab()`が最後に成功した`time.monotonic_ns()`。ハング検出専用
    #: （`last_t_capture`はセンサ時刻で、フレームが来ない間は更新されない
    #: という点では同じだが、意味を混同しないよう別フィールドにする）
    last_grab_ns: int = 0
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
                 n_slots: int, on_frame=None, device: int | None = None) -> None:
        super().__init__(name=f"cam{idx}", daemon=True)
        self.idx = idx
        #: libcamera の番号（`Picamera2(device)`）。既定は `idx`（リング・トピックの番号）と同じ
        self.device = idx if device is None else device
        self.size = size
        self.fmt = fmt
        self.fps = fps
        self.on_frame = on_frame
        self.stats = CamStats()
        self.error: Exception | None = None
        self._running = False
        #: **`run()` のスレッドだけが picamera2 を呼ぶ。** `request_fps`/
        #: `request_enabled` は他スレッド（`CameraNode._config_loop`）から呼ばれるが、
        #: そこでは pending 変数を書き換えるだけで picamera2 には触らない
        #: （`Picamera2` はマルチスレッドからの同時呼び出しを想定していないため）
        self._cfg_lock = threading.Lock()
        self._pending_fps: float | None = None
        self._pending_enabled: bool | None = None
        self._enabled = True

        from picamera2 import Picamera2

        # チューニングは全カメラで同じもの（None ＝ 標準）。**ここより前に picamera2 でカメラを
        # 数えない**——`global_camera_info()` を呼ぶだけで標準に固定される（`cam_tuning.py`）
        tuning, tuning_sensor = _isp_tuning
        self.cam = Picamera2(self.device, tuning=tuning)
        props = self.cam.camera_properties
        print(f"# cam{idx}: libcamera 番号 {self.device}  {props.get('Model', '?')}  "
              f"{props.get('Location', '')}", flush=True)
        check_sensor(tuning_sensor, str(props.get("Model", "")), idx)
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
        # `run()` が始まる前に `stop()` が呼ばれていたら（起動直後の SIGTERM）そのまま終わる
        self._running = not getattr(self, "_stop_requested", False)
        consecutive_failures = 0
        try:
            while self._running:
                self._apply_pending()
                if not self._enabled:
                    # **止めている間は capture_request を呼ばない。** `Picamera2.stop()`
                    # 済みなので呼んでも取れない上、CPU を無駄に回さないため
                    #
                    # **ハング検出（`CameraNode.run`）に「止めているだけ」と伝える。** これを
                    # 更新しないと `last_grab_ns` が止まったまま `_CAPTURE_HANG_S` を過ぎ、
                    # 後カメラを OFF にしている間（disarm 中の既定）にノード全体が落ちる
                    self.stats.last_grab_ns = time.monotonic_ns()
                    time.sleep(0.2)
                    continue
                try:
                    self._grab()
                    if consecutive_failures:
                        self.error = None            # 一過性の失敗は回復したので残さない
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
                # 再開直後の最初の1枚が来るまでをハング扱いしない
                self.stats.last_grab_ns = time.monotonic_ns()
            else:
                self.cam.stop()
            # 停止していた間の間隔を「落ち」と誤集計しないよう、前回の時刻を捨てる
            self.stats.last_t_capture = 0

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
            # make_array は DMA バッファへの view。**release() すると無効になる**ので、
            # 共有メモリへの書き込みは release より前に済ませる。
            # ここで release を先にすると、解放済みメモリを読むことになる
            arr = req.make_array("main")
            if not arr.flags["C_CONTIGUOUS"]:
                arr = _contig(arr)
            t0 = time.monotonic_ns()
            # picamera2 のメタデータに FrameCount は無いので、取得枚数を frame_id にする
            desc = self.ring.write(
                arr, t_capture_ns=t_cap, frame_id=self.stats.frames)
            dt = time.monotonic_ns() - t0
        finally:
            req.release()

        s = self.stats
        s.frames += 1
        s.write_ns_total += dt
        s.write_ns_max = max(s.write_ns_max, dt)
        s.last_grab_ns = t0             # ハング検出用（上のCamStats参照）
        if s.last_t_capture and t_cap:
            gap_ns = t_cap - s.last_t_capture
            s.gaps_ms.append(gap_ns / 1e6)
            if self.fps:
                s.dropped += count_dropped(gap_ns, 1e9 / self.fps)
        s.last_t_capture = t_cap

        if self.on_frame:
            self.on_frame(desc)

    def stop(self) -> None:
        self._stop_requested = True
        self._running = False

    def close(self) -> None:
        # 共有メモリの unlink を先にする: `cam.stop()` がハングして強制終了になっても
        # 名前だけは残さない
        with quiet_close(f"FrameRing {self.ring.name} の unlink"):
            self.ring.unlink()
        with quiet_close("picamera2 のカメラ"):
            self.cam.stop()
            self.cam.close()


def _contig(arr):
    import numpy as np

    return np.ascontiguousarray(arr)


class CameraNode:
    def __init__(self, indices: list[int], size, fmt: str, fps, n_slots: int,
                 on_frame=None, cfg_sub=None,
                 devices: dict[int, int] | None = None) -> None:
        """`devices` は リング・トピックの番号 → libcamera の番号（省略時は同じ番号）。

        開けないカメラがあっても、1台でも開ければ続行する（開けなかった番号は `missing`）。
        1台も開けなければ最後の例外を投げる。
        """
        self.workers: list[CameraWorker] = []
        #: 開けなかったカメラの番号（`hb/camera` に `missing` と出す）
        self.missing: list[int] = []
        self._stop_requested = False
        last_err: Exception | None = None
        for i in indices:
            kwargs = {"device": devices[i]} if devices and i in devices else {}
            try:
                self.workers.append(
                    CameraWorker(i, size, fmt, fps, n_slots, on_frame, **kwargs))
            except Exception as e:                   # noqa: BLE001
                last_err = e
                self.missing.append(i)
                print(f"!! cam{i} を開けない（{type(e).__name__}: {e}）。"
                      f"開けたカメラだけで続行する", file=sys.stderr, flush=True)
        if not self.workers:
            assert last_err is not None
            raise last_err
        self._t_start = 0.0
        #: GUI/自動運転からの capture 設定（`cam/config`）を拾うための Subscriber。
        #: `--no-bus` やバス未接続なら None（既定 fps・常時 ON のまま動く）
        self._cfg_sub = cfg_sub
        self._cfg_thread: threading.Thread | None = None
        self._cfg_running = False

    def run(self, duration_s: float | None = None, status_cb=None,
           hang_s: float = _CAPTURE_HANG_S, hb_cb=None) -> None:
        for w in self.workers:
            w.start_camera()
        time.sleep(0.5)                     # AE/AWB が落ち着くまで
        if getattr(self, "_stop_requested", False):
            return                          # 起動中に SIGTERM が来た（下の finally は通らないので何も起動していない）
        start_ns = time.monotonic_ns()
        for w in self.workers:
            w.stats = CamStats()
            w.stats.last_grab_ns = start_ns  # 起動直後の「まだ1枚も来ていない」をハング扱いしない
            w.start()

        if self._cfg_sub is not None:
            self._cfg_running = True
            self._cfg_thread = threading.Thread(
                target=self._config_loop, name="camcfg", daemon=True)
            self._cfg_thread.start()

        self._t_start = time.monotonic()
        next_status = 0.0
        next_hb = 0.0
        try:
            while True:
                # **どちらか1台でも死んだら／ハングしたらノード全体を終了させる。**
                # `any(is_alive())`（全滅するまで回り続ける）だと、
                # 前後どちらか1台だけ死んでももう1台が生きている限り
                # プロセスが生き続け、誰も気づかない（issue #4）。
                # `capture_request()` がブロックしたまま戻らないハングは
                # スレッドが `is_alive()` のままなので、`last_grab_ns` の
                # 停滞で別途検出する
                if getattr(self, "_stop_requested", False):
                    break
                dead = [w for w in self.workers if not w.is_alive()]
                if dead:
                    break
                now_ns = time.monotonic_ns()
                # 1台ごとの猶予 = max(hang_s, 3×フレーム周期)。1fps 設定で1枚が1秒かかっても
                # ハング扱いしない
                limits = {}
                for w in self.workers:
                    fps = getattr(w, "fps", None)
                    limits[w.idx] = max(hang_s, 3.0 / fps) if fps else hang_s
                hung = [w for w in self.workers
                       if now_ns - w.stats.last_grab_ns > limits[w.idx] * 1e9]
                if hung:
                    for w in hung:
                        if w.error is None:
                            w.error = TimeoutError(
                                f"cam{w.idx}: capture_request() が{limits[w.idx]:.1f}s"
                                f"以上応答していない（ハング）")
                    break
                el = time.monotonic() - self._t_start
                if duration_s is not None and el >= duration_s:
                    break
                if status_cb and el >= next_status:
                    status_cb(self)
                    next_status = el + 1.0
                if hb_cb and el >= next_hb:
                    hb_cb(self)
                    next_hb = el + 1.0 / HB_HZ
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
            except Exception as e:
                print(f"!! cam/config 受信スレッドが異常終了"
                      f"（以後 GUI からの cam/config 変更が反映されない）: "
                      f"{type(e).__name__}: {e}",
                      file=sys.stderr)
                break
            for topic, msg in events:
                if topic != TOPIC_CAM_CONFIG:
                    continue
                for w in self.workers:
                    role = role_of[w.idx]
                    if role == "front":
                        w.request_fps(msg.front_fps)
                        w.request_enabled(msg.front_enabled)
                    elif role == "rear":
                        w.request_fps(msg.rear_fps)
                        w.request_enabled(msg.rear_enabled)

    @property
    def elapsed(self) -> float:
        return time.monotonic() - self._t_start if self._t_start else 0.0

    def stop(self) -> None:
        self._stop_requested = True
        self._cfg_running = False
        if self._cfg_thread is not None and self._cfg_thread.is_alive():
            self._cfg_thread.join(timeout=1.0)
        for w in self.workers:
            w.stop()
        for w in self.workers:
            # 起動前（start していない）スレッドの join は RuntimeError になる
            if w.is_alive():
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
    ap.add_argument("--by-i2c", action="store_true",
                    help="前後のカメラを番号でなく I2C アドレス（I2C_ADDR_ROLE）で決める。"
                         "ISP チューニングを使うレンズでは、先に列挙するとチューニングが"
                         "効かなくなるので無視して番号どおりにする")
    args = ap.parse_args()
    sys.stdout = _Stamped(sys.stdout)
    sys.stderr = _Stamped(sys.stderr)

    w, h = (int(v) for v in args.size.lower().split("x"))
    indices = [int(v) for v in args.cameras.split(",") if v.strip()]

    # ── バス配信（画素は流さない。共有メモリへの参照だけ） ──
    pub = None
    on_frame = None
    cfg_sub = None
    hb_cb = None
    if not args.no_bus:
        try:
            from raspi.bus import LATEST, Publisher, Subscriber
            from raspi.msgs import Heartbeat, ImageRef, TOPIC_CAM_CONFIG, TOPIC_HB_PREFIX

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

            hb_detail = make_hb_detail()

            def hb_cb(node):                                 # noqa: F811
                pub.send(TOPIC_HB_PREFIX + "camera",
                         Heartbeat(node="camera", pid=os.getpid(), detail=hb_detail(node)))

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
        prepare_isp_tuning(indices)
        devices = None
        if args.by_i2c:
            if _isp_tuning[0] is not None:
                print("!! --by-i2c は ISP チューニング使用中は使えない（カメラを数えると"
                      "チューニングが標準に固定される）。番号どおりに開く", file=sys.stderr)
            else:
                from picamera2 import Picamera2

                infos = Picamera2.global_camera_info()
                print(f"# 見えているカメラ: {[i.get('Id') for i in infos]}")
                roles = [CAM_TOPIC.get(i, ("", f"cam{i}"))[1] for i in indices]
                by_role = devices_by_i2c(infos, roles)
                devices = {i: by_role[CAM_TOPIC[i][1]] for i in indices
                           if i in CAM_TOPIC and CAM_TOPIC[i][1] in by_role}
                print(f"# I2C で決めた番号（リング番号→libcamera 番号）: {devices}")
        node = CameraNode(indices, (w, h), args.fmt, args.fps, args.slots,
                          on_frame=on_frame, cfg_sub=cfg_sub, devices=devices)
    except Exception as e:
        print(f"カメラを開けない: {e}", file=sys.stderr)
        return 2

    # SIGTERM も拾う（`systemctl stop`/`--restart` 用。拾わないと picamera2 の
    # stop・共有メモリの unlink が走らず finally が飛ぶ。`planning_node.py` と同じパターン）
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: node.stop())

    if node.missing:
        print(f"!! 開けなかったカメラ: {node.missing}（配信しない）", file=sys.stderr)
    for wk in node.workers:
        print(f"# cam{wk.idx} (libcamera {wk.device}) -> /dev/shm/{wk.ring.name}  "
              f"{wk.ring.width}x{wk.ring.height} {wk.ring.fmt} "
              f"×{wk.ring.n_slots}枚 ({wk.ring._shm.size / 1e6:.1f}MB)")
    print("# 読み手: python3 -m raspi.tools.shm_view "
          f"{SHM_PREFIX}{node.workers[0].idx}\n")

    try:
        node.run(duration_s=args.duration,
                 status_cb=None if args.quiet else _status, hb_cb=hb_cb)
    finally:
        el = node.elapsed
        print("\n\n=== 終了時の統計 ===")
        for wk in node.workers:
            print(f"cam{wk.idx}: {wk.stats.summary(el)}")
            if wk.error:
                print(f"  !! 異常終了: {wk.error}")
        def _cleanup() -> None:
            if pub is not None:
                print(f"bus: {pub.sent} 件 publish")
                pub.close()
            node.close()

        # `capture_request()` が本当にハングしていると後始末も戻らないことがある。
        # 時間を区切り、超えたらプロセスごと落として systemd に再起動させる
        if not run_bounded(_cleanup, CLOSE_TIMEOUT_S):
            print(f"!! 後始末が{CLOSE_TIMEOUT_S:.0f}s で終わらない。強制終了する",
                  file=sys.stderr, flush=True)
            sys.stdout.flush()
            os._exit(1)
    # **いずれかのワーカーにエラーが残っていれば exit 1。**
    # 常に `return 0` だと、カメラが全滅・ハングしても
    # systemd の `Restart=on-failure` が発火せず恒久停止する（issue #4）。
    # `--duration` を指定した正常終了（テスト・手動計測）ではどのワーカーにも
    # エラーが立たないので、ここは 0 のまま
    if any(wk.error for wk in node.workers):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
