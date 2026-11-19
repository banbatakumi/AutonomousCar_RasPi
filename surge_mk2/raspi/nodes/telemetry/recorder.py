"""記録の子プロセス（MCAP のライブ中継・🎥 録画）の起動と停止（`TelemetryServer` から切り出した）。

- **MCAP**: `logger_node -o -` の標準出力を `/ws/record` の視聴者へ生中継する。**Pi の SD には書かない**
- **録画**: `video_node` が Pi の `logs/` に mp4 を書く。止めたら GUI が `GET /logs/<name>` で取る

サーバとのつなぎは引数の関数だけ（カメラの希望を送り直す・状態を配る・視聴者へ送る）。
カメラは記録の間だけ起こし fps を上げるので、始める・終わるたびに `on_cameras_change` を呼ぶ。
"""

from __future__ import annotations

import asyncio
import sys
import time
from pathlib import Path
from typing import Awaitable, Callable

from raspi.core.cleanup import quiet_close

__all__ = ["Recorder"]

NS = 1_000_000_000
#: 🎥 録画が自動で止まるまでの時間 [s]（止め忘れでSDが埋まるのを防ぐ。`video_node` の `--duration`）
VIDEO_MAX_S = 600
#: 中継の読み取り単位
RECORD_CHUNK = 65536
#: 最初の視聴者（`/ws/record`）が繋がるまで取っておく量の上限 [B]（`_mcap_pump`）。
#: 接続は数十msで済むので、ふつうは先頭の数百バイトしか溜まらない
RECORD_BACKLOG_MAX = 8 * 1024 * 1024


class Recorder:
    """:param record_clients: `/ws/record` の視聴者の集合（サーバと共有。中継の宛先）
    :param send_or_drop: `(ws, payload, clients)` を送る。送れない相手は `clients` から外して切る
    :param on_cameras_change: 記録の開始・終了でカメラの希望が変わったときに呼ぶ
    :param on_status: 状態が変わったときに呼ぶ（待たない配信）
    :param broadcast_status: 状態を配り終わるまで待つ配信（録画の終了時。GUI がすぐダウンロードする）
    """

    def __init__(self, *, repo_root: Path, video_fps: float,
                 record_clients: set,
                 send_or_drop: Callable[[object, bytes, set], Awaitable[None]],
                 on_cameras_change: Callable[[], None],
                 on_status: Callable[[], None],
                 broadcast_status: Callable[[], Awaitable[None]]) -> None:
        self.repo_root = repo_root
        self.video_fps = video_fps
        self.record_clients = record_clients
        self._send_or_drop = send_or_drop
        self._on_cameras_change = on_cameras_change
        self._on_status = on_status
        self._broadcast_status = broadcast_status

        #: mcap のライブ中継（`logger_node -o -` のサブプロセス）
        self.mcap_proc: asyncio.subprocess.Process | None = None
        #: `mcap_start` の `await create_subprocess_exec` 中の再入防止フラグ
        self.mcap_starting = False
        self.mcap_started_ns = 0
        self.mcap_error: str | None = None
        self.mcap_pump_task: asyncio.Task | None = None

        #: 🎥 録画（`video_node` のサブプロセス）
        self.video_proc: asyncio.subprocess.Process | None = None
        self.video_starting = False
        self.video_started_ns = 0
        self.video_error: str | None = None
        #: 今回（または直近）の出力の接頭辞と、書き終えた mp4 のファイル名。
        #: `files` が空でなく `active` が false になったら GUI がダウンロードする
        self.video_prefix = ""
        self.video_files: list[str] = []
        self.video_stopping = False
        self.video_task: asyncio.Task | None = None

    # ── 状態 ──

    @property
    def mcap_busy(self) -> bool:
        """MCAP を記録中か、起動の途中（カメラを起こしておく条件）。"""
        return self.mcap_proc is not None or self.mcap_starting

    @property
    def video_recording(self) -> bool:
        """録画中か、起動の途中（fps を上げておく条件）。"""
        return self.video_proc is not None or self.video_starting

    @property
    def active(self) -> bool:
        """どちらかが動いている（経過時間の表示を進めるため状態を毎秒配る）。"""
        return self.mcap_proc is not None or self.video_proc is not None

    def mcap_status(self) -> dict:
        active = self.mcap_proc is not None
        elapsed = (time.monotonic_ns() - self.mcap_started_ns) / NS if active else 0.0
        return {"active": active, "elapsed_s": elapsed, "error": self.mcap_error}

    def video_status(self) -> dict:
        active = self.video_proc is not None
        elapsed = (time.monotonic_ns() - self.video_started_ns) / NS if active else 0.0
        return {"active": active, "elapsed_s": elapsed, "error": self.video_error,
                "files": [] if active else list(self.video_files)}

    # ── 🎥 録画 ──

    async def video_start(self, logs_dir: Path) -> None:
        """`logs_dir` に mp4 を書く `video_node` を起こす。"""
        if self.video_recording:
            return
        self.video_starting = True
        self.video_error = None
        self.video_files = []
        # DISARM 中に止めているカメラを起こし、fps を上げる
        self._on_cameras_change()
        logs_dir.mkdir(parents=True, exist_ok=True)
        prefix = logs_dir / f"surge_{time.strftime('%Y%m%d_%H%M%S')}"
        cmd = [sys.executable, "-u", "-m", "raspi.nodes.video_node",
               "--prefix", str(prefix), "--fps", str(self.video_fps),
               "--duration", str(VIDEO_MAX_S)]
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd, cwd=str(self.repo_root),
                stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.PIPE)
        except OSError as e:
            self.video_error = f"video_node を起動できない: {e}"
            self.video_starting = False
            self._on_cameras_change()          # 起動に失敗したら fps を戻す
            return
        # `video_proc` を先に立ててから `video_starting` を下ろす（間で fps が戻らないように）
        self.video_prefix = str(prefix)
        self.video_proc = proc
        self.video_starting = False
        self.video_started_ns = time.monotonic_ns()
        self.video_task = asyncio.create_task(self._video_wait(proc))

    async def _video_wait(self, proc: asyncio.subprocess.Process) -> None:
        """`video_node` の終了を待ち、書き終えた mp4 の一覧を status に載せる。"""
        assert proc.stderr is not None
        err = (await proc.stderr.read()).decode(errors="replace").strip()
        rc = await proc.wait()
        if self.video_proc is proc:
            self.video_proc = None
        prefix = Path(self.video_prefix)
        self.video_files = sorted(p.name for p in prefix.parent.glob(prefix.name + "_*.mp4"))
        if rc != 0 and not self.video_stopping:
            self.video_error = f"video_node が異常終了しました（rc={rc}）: {err[-200:]}"
        elif not self.video_files:
            self.video_error = "映像が来なかったので mp4 ができていません（カメラが止まっていないか確認）"
        self.video_stopping = False
        self._on_cameras_change()              # 録画のために上げた fps・起こしたカメラを元に戻す
        await self._broadcast_status()

    async def video_stop(self) -> None:
        proc = self.video_proc
        if proc is None:
            return
        self.video_stopping = True
        try:
            proc.terminate()                   # SIGTERM → video_node が mp4 を閉じて終わる
        except ProcessLookupError:
            pass

    # ── mcap のライブ中継（Pi のディスクには書かない） ──

    async def mcap_start(self, *, image_hz: float) -> None:
        if self.mcap_busy:
            return                             # 既に録画中、または起動処理の途中
        prev = self.mcap_pump_task
        if prev is not None and not prev.done():
            # 止めた直後の記録は、末尾（索引）の中継がまだ続いている。ここで次を始めると、
            # 前の記録の末尾が新しい視聴者へ混ざり、前の中継の終了が新しい `/ws/record` を切る
            self.mcap_error = "前の記録の終了処理中です。少し待ってからもう一度"
            return
        # **`await create_subprocess_exec` の間だけこのフラグで再入を防ぐ。**
        # `mcap_proc is not None` のチェックと代入の間には中断点（await）があり、素通しだと
        # 短時間の二重呼び出しで logger_node を二重起動してしまう（録画データの混線・先発プロセスのリーク）
        self.mcap_starting = True
        self.mcap_error = None
        # DISARM 中に止めているカメラを記録のために起こす（`_cam_in_use_while_disarmed`）
        self._on_cameras_change()
        cmd = [sys.executable, "-u", "-m", "raspi.nodes.logger_node",
               "--quiet", "-o", "-", "--image-hz", str(image_hz)]
        try:
            proc = await asyncio.create_subprocess_exec(
                *cmd, cwd=str(self.repo_root),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
        except OSError as e:
            self.mcap_error = f"logger_node を起動できない: {e}"
            return
        finally:
            self.mcap_starting = False
        self.mcap_proc = proc
        self.mcap_started_ns = time.monotonic_ns()
        self.mcap_pump_task = asyncio.create_task(self._mcap_pump(proc))

    async def _mcap_pump(self, proc: asyncio.subprocess.Process) -> None:
        """`logger_node -o -` の標準出力を `/ws/record` へ生中継する。

        **視聴者が居なくても読み続ける。** 読まないと子プロセスのパイプが詰まって
        書き込みがブロックしてしまう（Piには保存しないので読んだ分は捨てるだけ）。
        """
        assert proc.stdout is not None
        # **最初の視聴者が繋がるまでの分は捨てずに取っておく**。ブラウザは `/ws/record` を
        # 開くのと同時に記録開始を送るので、logger_node の最初の出力（MCAP のマジック・ヘッダ）が
        # `/ws/record` の接続より先に届くことがある。捨てると先頭の欠けた mcap になり、普通の
        # リーダでは開けない（実機で `sysid_wheel` の記録がこうなった: "invalid magic"）。
        # 一度でも視聴者が付いた後は捨てる（視聴者が去った＝もう要らない）
        backlog: list[bytes] = []
        backlog_bytes = 0
        had_client = False
        try:
            while True:
                chunk = await proc.stdout.read(RECORD_CHUNK)
                if not chunk:
                    break
                if not self.record_clients:
                    if not had_client and backlog_bytes < RECORD_BACKLOG_MAX:
                        backlog.append(chunk)
                        backlog_bytes += len(chunk)
                    continue
                had_client = True
                if backlog:
                    chunk = b"".join(backlog) + chunk
                    backlog.clear()
                await asyncio.gather(
                    *(self._send_or_drop(ws, chunk, self.record_clients)
                      for ws in list(self.record_clients)),
                    return_exceptions=True)
        finally:
            rc = await proc.wait()
            # `mcap_proc is proc` が False なら `mcap_stop` が既に片付け済み
            # （＝人間が止めた）。ここで上書きすると正常終了がエラー扱いになる
            if self.mcap_proc is proc:
                if rc != 0:
                    self.mcap_error = f"logger_node が異常終了しました（rc={rc}）"
                self.mcap_proc = None
                self._on_cameras_change()
                self._on_status()
            # **中継が完全に終わった合図として `/ws/record` を切る。** MCAP の索引は
            # 最後に書かれるので、GUI 側はこの切断イベントを「もう来ない」の目印にして
            # ダウンロードを発火する（`record.ts` の `onClose`）
            for ws in list(self.record_clients):
                # 既に相手が切っていれば例外になる。**それはむしろ望んだ状態**
                with quiet_close("/ws/record の切断（mcap 中継の終了合図）"):
                    await ws.close()

    async def mcap_stop(self) -> None:
        proc = self.mcap_proc
        if proc is None:
            return
        self.mcap_proc = None
        try:
            proc.terminate()
        except ProcessLookupError:
            pass
        self._on_cameras_change()              # 記録のために上げた fps / 起こしたカメラを元に戻す
        # 実際の終了待ち・末尾チャンク（MCAPの索引）の中継は `_mcap_pump` が続ける
