import asyncio
import json
import logging
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from uart_handler import UARTHandler
from camera_stream import CameraStream
from gpio_handler import GPIOHandler

logging.basicConfig(level=logging.INFO)

APP_DIR = Path(__file__).parent.parent / "app"

uart   = UARTHandler()
camera = CameraStream()
gpio   = GPIOHandler()

connected: set[WebSocket] = set()


async def _broadcast_sensor():
    while True:
        data = uart.get_sensor_data()
        msg = json.dumps(data)
        dead = set()
        for ws in list(connected):
            try:
                await ws.send_text(msg)
            except Exception:
                dead.add(ws)
        connected.difference_update(dead)
        await asyncio.sleep(0.05)  # 20 Hz


@asynccontextmanager
async def lifespan(app: FastAPI):
    uart.start()
    camera.start()
    task = asyncio.create_task(_broadcast_sensor())
    yield
    task.cancel()
    uart.stop()
    camera.stop()
    gpio.cleanup()


app = FastAPI(lifespan=lifespan)


@app.get("/stream")
async def stream():
    return StreamingResponse(
        camera.generate(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Connection": "keep-alive",
        },
    )


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    first = len(connected) == 0
    connected.add(websocket)
    if first:
        gpio.on_connect()

    try:
        while True:
            text = await websocket.receive_text()
            cmd = json.loads(text)
            uart.set_command(cmd)
            gpio.set_remote_led(cmd.get("do_remote_control", False))
    except WebSocketDisconnect:
        pass
    finally:
        connected.discard(websocket)
        if not connected:
            uart.set_command({"do_stop": True, "do_remote_control": False})
            gpio.on_disconnect()


app.mount("/", StaticFiles(directory=str(APP_DIR), html=True), name="static")
