import asyncio
import io
import threading
import time
import logging

logger = logging.getLogger(__name__)

CAPTURE_FPS = 30


class CameraStream:
    def __init__(self):
        self._frame = None
        self._lock = threading.Lock()
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _capture_loop(self):
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            cam.configure(cam.create_video_configuration(main={"size": (640, 480)}))
            cam.start()
            try:
                while self._running:
                    buf = io.BytesIO()
                    cam.capture_file(buf, format="jpeg")
                    with self._lock:
                        self._frame = buf.getvalue()
                    time.sleep(1 / CAPTURE_FPS)
            finally:
                cam.stop()
        except Exception as e:
            logger.error("Camera error: %s", e)

    async def generate(self):
        while True:
            with self._lock:
                frame = self._frame
            if frame:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                )
            await asyncio.sleep(1 / CAPTURE_FPS)
