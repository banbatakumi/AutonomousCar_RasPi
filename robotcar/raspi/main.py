import asyncio
import json
import logging
import re
import subprocess
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
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
_sys: dict = {"cpu_temp": None, "cpu_load": None, "mem_usage": None, "wifi_tx_mbps": None, "throttle": None, "uptime": None}
_POLL_INTERVAL = 3.0
_prev_cpu: tuple[int, int] | None = None  # (total, idle)

_standby = True  # mode=0 のとき True


def _apply_standby(standby: bool):
    global _standby
    if _standby == standby:
        return
    _standby = standby
    gov = "powersave" if standby else "ondemand"
    for path in Path("/sys/devices/system/cpu").glob("cpu[0-9]/cpufreq/scaling_governor"):
        try:
            path.write_text(gov)
        except Exception:
            pass
    logger.info("Standby: %s (CPU governor: %s)", standby, gov)


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

        # スロットリング状態
        try:
            out = await loop.run_in_executor(
                None,
                lambda: subprocess.check_output(
                    ["vcgencmd", "get_throttled"],
                    stderr=subprocess.DEVNULL, text=True,
                ),
            )
            m = re.search(r"0x([0-9a-fA-F]+)", out)
            _sys["throttle"] = int(m.group(1), 16) if m else None
        except Exception:
            _sys["throttle"] = None

        # 稼働時間
        try:
            secs = int(float(Path("/proc/uptime").read_text().split()[0]))
            d, rem = divmod(secs, 86400)
            h, rem = divmod(rem, 3600)
            m = rem // 60
            if d > 0:
                _sys["uptime"] = f"{d}d{h}h"
            elif h > 0:
                _sys["uptime"] = f"{h}h{m:02d}m"
            else:
                _sys["uptime"] = f"{m}m"
        except Exception:
            _sys["uptime"] = None

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
        if dead and not connected:
            uart.set_safe_state()
            gpio.on_disconnect()
        await asyncio.sleep(0.2)  # 5 Hz


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


@app.middleware("http")
async def no_cache(request: Request, call_next):
    response = await call_next(request)
    response.headers["Cache-Control"] = "no-store"
    return response


@app.websocket("/ws/camera")
async def camera_ws(websocket: WebSocket):
    await websocket.accept()
    loop = asyncio.get_running_loop()
    last_frame = None

    # Adaptive FPS: skip_target=0→30fps, 1→15fps, 2→10fps
    skip_target = 0
    skip_count  = 0
    slow_streak = 0
    fast_streak = 0

    try:
        while True:
            output = camera._output
            if output is None:
                await asyncio.sleep(0.1)
                continue

            t0 = time.monotonic()
            frame = await loop.run_in_executor(None, output.wait_new, last_frame)
            wait_ms = (time.monotonic() - t0) * 1000

            if frame is None:
                if not camera._running:
                    break
                continue
            last_frame = frame

            # Frame skipping
            if skip_count < skip_target:
                skip_count += 1
                continue
            skip_count = 0

            await websocket.send_bytes(frame)

            if _standby:
                await asyncio.sleep(0.5)  # standby: ~2fps
                continue

            # wait_new が 5ms 未満で返った = フレームが溜まっている = 送信遅延中
            if wait_ms < 5:
                slow_streak += 1
                fast_streak  = 0
                if slow_streak >= 3 and skip_target < 2:
                    skip_target += 1
                    slow_streak  = 0
                    logger.info("Camera adaptive: tier→%d (%dfps)", skip_target, 30 // (skip_target + 1))
            else:
                fast_streak += 1
                slow_streak  = 0
                if fast_streak >= 60 and skip_target > 0:
                    skip_target -= 1
                    fast_streak  = 0
                    logger.info("Camera adaptive: tier→%d (%dfps)", skip_target, 30 // (skip_target + 1))

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
            try:
                text = await asyncio.wait_for(websocket.receive_text(), timeout=1.0)
            except asyncio.TimeoutError:
                logger.warning("WebSocket heartbeat timeout — treating as disconnected")
                break
            cmd = json.loads(text)
            uart.set_command(cmd)
            gpio.set_remote_led(cmd.get("do_remote_control", False))
            _apply_standby(cmd.get("mode", 0) == 0)
    except WebSocketDisconnect:
        pass
    finally:
        connected.discard(websocket)
        if not connected:
            uart.set_safe_state()
            gpio.on_disconnect()


app.mount("/", StaticFiles(directory=str(APP_DIR), html=True), name="static")
