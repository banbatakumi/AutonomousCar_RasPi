import asyncio
import io
import threading
import time
import logging

logger = logging.getLogger(__name__)

CAPTURE_FPS = 20
CAPTURE_WIDTH = 240
CAPTURE_HEIGHT = 144   # 240 * 3/5: 下2/5（車体）を除外
JPEG_QUALITY = 55

# IMX219 フルセンサー座標 (3280x2464) の上位 4/5 を使用
_SENSOR_CROP = (0, 0, 3280, 1971)   # 2464 * 0.8 ≈ 1971


class _Output(io.BufferedIOBase):
    """picamera2 JpegEncoder から呼ばれる: 完結した JPEG 1枚ごとに write() が来る。"""

    def __init__(self):
        self.frame: bytes | None = None
        self._condition = threading.Condition()

    def write(self, buf) -> int:
        with self._condition:
            self.frame = bytes(buf)
            self._condition.notify_all()
        return len(buf)

    def wait_new(self, prev_frame, timeout: float = 1.0):
        with self._condition:
            while self.frame is prev_frame:
                if not self._condition.wait(timeout):
                    return None
            return self.frame


class CameraStream:
    def __init__(self):
        self._output: _Output | None = None
        self._running = False

    def start(self):
        self._running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def stop(self):
        self._running = False

    def _capture_loop(self):
        try:
            from picamera2 import Picamera2
            from picamera2.encoders import JpegEncoder
            from picamera2.outputs import FileOutput

            cam = Picamera2()
            frame_us = int(1_000_000 / CAPTURE_FPS)
            # main を 1640x1232（フルFOV 2x2ビニングモード）にすることで
            # センサーが全画角を使用する。エンコードは lores から行う。
            config = cam.create_video_configuration(
                main={"size": (1640, 1232)},
                lores={"size": (CAPTURE_WIDTH, CAPTURE_HEIGHT), "format": "YUV420"},
                buffer_count=2,
            )
            cam.configure(config)

            output = _Output()
            self._output = output
            cam.start_recording(JpegEncoder(q=JPEG_QUALITY), FileOutput(output), name="lores")
            cam.set_controls({
                "FrameDurationLimits": (frame_us, frame_us),
                "ScalerCrop": _SENSOR_CROP,
            })

            try:
                while self._running:
                    time.sleep(0.5)
            finally:
                cam.stop_recording()
                self._output = None
        except Exception as e:
            logger.error("Camera error: %s", e)

    async def generate(self):
        loop = asyncio.get_event_loop()
        last_frame = None
        while True:
            output = self._output
            if output is None:
                await asyncio.sleep(0.1)
                continue
            frame = await loop.run_in_executor(None, output.wait_new, last_frame)
            if frame is None:
                if not self._running:
                    break
                continue
            last_frame = frame
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                + frame + b"\r\n"
            )
