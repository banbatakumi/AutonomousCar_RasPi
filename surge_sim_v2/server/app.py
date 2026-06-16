"""FastAPI アプリ・WebSocket 配信。

起動方法（通常は main.py から uvicorn を別スレッドで起動）：
    uvicorn server.app:app --host 0.0.0.0 --port 8000

WebSocket エンドポイント ws://host:8000/ws
  - SystemState を JSON シリアライズして 20Hz で配信
  - クライアントからの受信メッセージ：
      {"type":"command","target_speed":1.0,"target_steer":10.0}
      {"type":"ping","timestamp":1234567890.0}            ← レイテンシ計測
      {"type":"emergency_stop"}
"""
from __future__ import annotations

import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from core.interfaces import ControlCommand

from .broadcaster import Broadcaster, ConnectionManager
from .context import get_context
from .routes import router

app = FastAPI(title="SURGE Mark.2 Server")
app.include_router(router)

manager = ConnectionManager()
_broadcaster: Broadcaster | None = None

WEB_DIST = Path(__file__).resolve().parent.parent / "web" / "dist"


@app.on_event("startup")
async def _on_startup() -> None:
    global _broadcaster
    ctx = get_context()
    hz = 20.0
    _broadcaster = Broadcaster(ctx.shared_state, manager, broadcast_hz=hz)
    _broadcaster.start()


@app.on_event("shutdown")
async def _on_shutdown() -> None:
    if _broadcaster is not None:
        _broadcaster.stop()


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    ctx = get_context()
    await manager.connect(ws)
    try:
        while True:
            msg = await ws.receive_json()
            mtype = msg.get("type")
            if mtype == "command":
                cmd = ControlCommand(
                    target_speed=float(msg.get("target_speed", 0.0)),
                    target_steer=float(msg.get("target_steer", 0.0)),
                    timestamp=time.time(),
                )
                ctx.controller.set_command(cmd)
            elif mtype == "ping":
                await ws.send_json({"type": "pong", "timestamp": msg.get("timestamp")})
            elif mtype == "emergency_stop":
                ctx.controller.emergency_stop()
    except WebSocketDisconnect:
        await manager.disconnect(ws)
    except Exception:  # noqa: BLE001
        await manager.disconnect(ws)


# --- 静的ファイル（React ビルド）の配信 -----------------------------------
if WEB_DIST.exists():
    app.mount("/", StaticFiles(directory=str(WEB_DIST), html=True), name="web")
else:
    @app.get("/", response_class=HTMLResponse)
    def _dev_index() -> str:
        return (
            "<html><body style='font-family:sans-serif;padding:2rem'>"
            "<h2>SURGE Mark.2 Server 稼働中</h2>"
            "<p>WebUI のビルドが見つかりません（web/dist）。</p>"
            "<p>開発時は <code>cd web &amp;&amp; npm install &amp;&amp; npm run dev</code> "
            "で Vite 開発サーバーを起動し、そちらにアクセスしてください。</p>"
            "<p>本番用は <code>cd web &amp;&amp; npm run build</code> でビルドすると "
            "このポートで配信されます。</p>"
            "<p>WebSocket: <code>ws://localhost:8000/ws</code></p>"
            "</body></html>"
        )
