import asyncio
import json
import logging
import re
import subprocess
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles

from uart_handler import UARTHandler
from camera_stream import CameraStream
from gpio_handler import GPIOHandler

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

APP_DIR = Path(__file__).parent.parent / "app"

uart   = UARTHandler()
camera = CameraStream()
gpio   = GPIOHandler()

connected: set[WebSocket] = set()

_CPU_TEMP_PATH = Path("/sys/class/thermal/thermal_zone0/temp")
_sys: dict = {"cpu_temp": None, "cpu_load": None, "mem_usage": None, "wifi_tx_mbps": None}
_POLL_INTERVAL = 3.0
_prev_cpu: tuple[int, int] | None = None  # (total, idle)


async def _poll_wifi():
    global _prev_cpu
    loop = asyncio.get_event_loop()
    while True:
        # CPU 温度
        try:
            _sys["cpu_temp"] = round(int(_CPU_TEMP_PATH.read_text().strip()) / 1000.0, 1)
        except Exception:
            _sys["cpu_temp"] = None

        # CPU 使用率（/proc/stat のデルタ）
        try:
            vals = list(map(int, Path("/proc/stat").read_text().splitlines()[0].split()[1:]))
            total, idle = sum(vals), vals[3]
            if _prev_cpu:
                dt = total - _prev_cpu[0]
                di = idle  - _prev_cpu[1]
                _sys["cpu_load"] = round((1 - di / dt) * 100, 1) if dt > 0 else None
            _prev_cpu = (total, idle)
        except Exception:
            _sys["cpu_load"] = None

        # メモリ使用率（/proc/meminfo）
        try:
            info = {}
            for line in Path("/proc/meminfo").read_text().splitlines():
                k, v = line.split(":")
                info[k.strip()] = int(v.strip().split()[0])
            _sys["mem_usage"] = round((info["MemTotal"] - info["MemAvailable"]) / info["MemTotal"] * 100, 1)
        except Exception:
            _sys["mem_usage"] = None

        # Wi-Fi 接続速度
        try:
            out = await loop.run_in_executor(
                None,
                lambda: subprocess.check_output(
                    ["sudo", "iw", "dev", "wlan1", "station", "dump"],
                    stderr=subprocess.DEVNULL, text=True,
                ),
            )
            best_rssi  = None
            best_inact = float("inf")
            for block in re.split(r"(?=^Station )", out, flags=re.MULTILINE):
                if not block.strip():
                    continue
                inact_m = re.search(r"inactive time:\s+(\d+)", block)
                rssi_m  = re.search(r"signal:\s+([-\d]+)", block)
                inact   = int(inact_m.group(1)) if inact_m else float("inf")
                if inact < best_inact:
                    best_inact = inact
                    best_rssi  = int(rssi_m.group(1)) if rssi_m else None
            _sys["wifi_tx_mbps"] = best_rssi
        except Exception as e:
            logger.error("wifi poll error: %s", e)
            _sys["wifi_tx_mbps"] = None

        await asyncio.sleep(_POLL_INTERVAL)


async def _broadcast_sensor():
    while True:
        data = uart.get_sensor_data()
        data.update(_sys)
        msg = json.dumps(data)
        dead = set()
        for ws in list(connected):
            try:
                await ws.send_text(msg)
            except Exception:
                dead.add(ws)
        connected.difference_update(dead)
        await asyncio.sleep(0.1)  # 10 Hz


@asynccontextmanager
async def lifespan(app: FastAPI):
    uart.start()
    camera.start()
    task      = asyncio.create_task(_broadcast_sensor())
    wifi_task = asyncio.create_task(_poll_wifi())
    yield
    task.cancel()
    wifi_task.cancel()
    uart.stop()
    camera.stop()
    gpio.cleanup()


app = FastAPI(lifespan=lifespan)


@app.websocket("/ws/camera")
async def camera_ws(websocket: WebSocket):
    await websocket.accept()
    loop = asyncio.get_running_loop()
    last_frame = None
    try:
        while True:
            output = camera._output
            if output is None:
                await asyncio.sleep(0.1)
                continue
            frame = await loop.run_in_executor(None, output.wait_new, last_frame)
            if frame is None:
                if not camera._running:
                    break
                continue
            last_frame = frame
            await websocket.send_bytes(frame)
    except WebSocketDisconnect:
        pass
    except Exception as e:
        logger.error("Camera WS error: %s", e, exc_info=True)


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
