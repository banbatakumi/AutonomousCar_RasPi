"""テレメトリ配信サーバ（WebSocket）。

確定事項に基づく実機/SIM共通のサーバ。Controller（実機/SIM共通）と
シーン提供関数を受け取り、ブラウザ等のUIへ:

  - ダウンリンク: テレメトリ封筒(JSON text) ＋ LiDAR(binary, 先頭タグ0x01)
                  ＋ 占有格子(binary, タグ0x02。Phase3で有効化)
  - アップリンク: CommandFrame(JSON text)

接続管理（決定④）:
  - 認証なし（private AP前提）
  - 操作クライアントは1つ＋閲覧は複数。最初の接続者が操作権を得る
  - `claim_control` で操作権を移譲（last-writer-wins）
  - 操作者が切断したら **安全停止(estop)** し操作権を解放
  - 接続/再接続時に Scene（＋将来は現在地図）を即送信

バイナリフレーム先頭タグ: 0x01=LiDAR, 0x02=占有格子。
"""

import asyncio
import json

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

from core.telemetry import (
    CommandFrame,
    apply_command,
    build_view,
    encode_grid,
    encode_lidar,
    grid_meta,
)

TAG_LIDAR = 0x01
TAG_GRID = 0x02


class _Client:
    def __init__(self, ws):
        self.ws = ws
        self.is_control = False


class TelemetryServer:
    """Controller を WebSocket でUIへ橋渡しするサーバ。"""

    def __init__(self, controller, get_scene, *, source: str = "sim",
                 on_course_change=None, get_courses=None, telemetry_hz: float = 20.0,
                 map_hz: float = 2.0, get_grid=None,
                 host: str = "0.0.0.0", port: int = 8000,
                 static_dir: str | None = None):
        self.controller = controller
        self.get_scene = get_scene
        self.source = source
        self.on_course_change = on_course_change
        self.get_courses = get_courses    # () -> list[str]
        self.telemetry_dt = 1.0 / telemetry_hz
        self.map_dt = 1.0 / map_hz if map_hz > 0 else 0.0
        self.get_grid = get_grid          # () -> OccupancyGrid | None
        self.host = host
        self.port = port
        self.static_dir = static_dir

        self._clients: list[_Client] = []
        self._control: _Client | None = None

        self.app = self._build_app()

    # ==================================================================
    def _build_app(self):
        app = FastAPI()

        @app.websocket("/ws")
        async def ws_endpoint(ws: WebSocket):
            await self._on_connect(ws)
            client = self._find(ws)
            send_task = asyncio.create_task(self._send_loop(client))
            try:
                while True:
                    text = await ws.receive_text()
                    await self._on_message(client, text)
            except WebSocketDisconnect:
                pass
            except Exception:
                pass
            finally:
                send_task.cancel()
                self._on_disconnect(client)

        @app.get("/courses.json")
        async def courses_json():
            names = self.get_courses() if self.get_courses else []
            return {"courses": list(names)}

        # 静的UI（surge_sim/ui_web を配信）。マウントは最後（他ルートを優先）
        if self.static_dir:
            from fastapi.staticfiles import StaticFiles
            app.mount("/", StaticFiles(directory=self.static_dir, html=True), name="ui")

        return app

    # ==================================================================
    # 接続管理
    # ==================================================================
    def _find(self, ws) -> "_Client | None":
        for c in self._clients:
            if c.ws is ws:
                return c
        return None

    async def _on_connect(self, ws) -> None:
        await ws.accept()
        client = _Client(ws)
        self._clients.append(client)
        # 操作者が不在なら、この接続が操作権を得る
        if self._control is None:
            self._control = client
            client.is_control = True
        await self._send_role(client)
        await self._send_scene(client)

    def _on_disconnect(self, client) -> None:
        if client in self._clients:
            self._clients.remove(client)
        if client is self._control:
            # 操作者切断 → 安全停止し操作権を解放
            self._safe_stop()
            self._control = None

    async def _on_message(self, client, text: str) -> None:
        try:
            msg = json.loads(text)
        except (ValueError, TypeError):
            return
        if msg.get("type") != "cmd":
            return
        name = msg.get("name")

        if name == "claim_control":
            await self._transfer_control(client)
            return

        # 操作権を持つクライアントの指令のみ適用
        if client is not self._control:
            return

        cmd = CommandFrame.from_dict(msg)
        if cmd.name == "set_course" and self.on_course_change is not None:
            self.on_course_change(cmd.payload.get("course"))
            await self._broadcast_scene()   # 新シーンを全クライアントへ再送
        else:
            apply_command(self.controller, cmd)
            # レーシングライン生成時は経路を含む新シーンを再送
            if cmd.name == "build_racing_line":
                await self._broadcast_scene()

    async def _transfer_control(self, client) -> None:
        prev = self._control
        self._control = client
        client.is_control = True
        if prev is not None and prev is not client:
            prev.is_control = False
            await self._send_role(prev)
        await self._send_role(client)

    def _safe_stop(self) -> None:
        apply_command(self.controller, CommandFrame("estop"))

    # ==================================================================
    # 送信
    # ==================================================================
    async def _send_role(self, client) -> None:
        await self._safe_send_text(client, json.dumps({
            "type": "role", "control": client.is_control, "source": self.source,
        }))

    async def _send_scene(self, client) -> None:
        scene = self.get_scene()
        await self._safe_send_text(client, json.dumps(scene.to_dict()))
        # 現在地図があれば即送信（Phase3）
        grid = self.get_grid() if self.get_grid else None
        if grid is not None:
            await self._safe_send_bytes(client, bytes([TAG_GRID]) + encode_grid(grid))

    async def _broadcast_scene(self) -> None:
        """全クライアントへ現在シーンを再送（コース切替時）。"""
        for client in list(self._clients):
            await self._send_scene(client)

    async def _send_loop(self, client) -> None:
        """このクライアント向けにテレメトリを定期送信する。"""
        last_map = 0.0
        try:
            while True:
                snap = self.controller.get_snapshot()
                # 運用UIには真値を載せない（実機相当）
                view = build_view(snap, self.get_scene(), source=self.source,
                                  include_truth=False)
                env = view.frame.to_envelope()

                # 地図メタを封筒へ（実データはbinaryで低レート送信）
                grid = self.get_grid() if self.get_grid else None
                if grid is not None:
                    env["map"] = grid_meta(grid)

                ok = await self._safe_send_text(client, json.dumps(env))
                if not ok:
                    return
                if view.lidar is not None:
                    await self._safe_send_bytes(
                        client, bytes([TAG_LIDAR]) + encode_lidar(view.lidar))

                # 占有格子は低レートで全体送信（決定②）
                if grid is not None and self.map_dt > 0:
                    last_map += self.telemetry_dt
                    if last_map >= self.map_dt:
                        last_map = 0.0
                        await self._safe_send_bytes(
                            client, bytes([TAG_GRID]) + encode_grid(grid))

                await asyncio.sleep(self.telemetry_dt)
        except asyncio.CancelledError:
            pass

    async def _safe_send_text(self, client, text: str) -> bool:
        try:
            await client.ws.send_text(text)
            return True
        except Exception:
            return False

    async def _safe_send_bytes(self, client, data: bytes) -> bool:
        try:
            await client.ws.send_bytes(data)
            return True
        except Exception:
            return False

    # ==================================================================
    def run(self) -> None:
        """ブロッキングでサーバを起動する。"""
        import uvicorn
        uvicorn.run(self.app, host=self.host, port=self.port, log_level="warning")
