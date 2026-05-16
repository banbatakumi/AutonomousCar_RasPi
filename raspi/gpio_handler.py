import threading
import time
import logging

logger = logging.getLogger(__name__)

BUZZER_PIN = 18
LED1_PIN   = 19   # remote control indicator
LED2_PIN   = 13   # PC connection indicator

BUZZ_DUTY = 50

# 接続メロディー: (周波数Hz, 音長ms, 次の音までの無音ms)
# C5 → E5 → G5 → C6 の上昇アルペジオ
CONNECT_MELODY = [
    (523,  100, 20),   # C5
    (659,  100, 20),   # E5
    (784,  100, 20),   # G5
    (1047, 350,  0),   # C6
]


class GPIOHandler:
    def __init__(self):
        self._available = False
        self._GPIO = None
        self._buzzer_pwm = None
        try:
            import RPi.GPIO as GPIO
            self._GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            for pin in (BUZZER_PIN, LED1_PIN, LED2_PIN):
                GPIO.setup(pin, GPIO.OUT)
            GPIO.output(LED1_PIN, GPIO.LOW)
            GPIO.output(LED2_PIN, GPIO.LOW)
            self._buzzer_pwm = GPIO.PWM(BUZZER_PIN, 440)
            self._available = True
        except Exception as e:
            logger.warning("GPIO not available: %s", e)

    def on_connect(self):
        if not self._available:
            return
        self._GPIO.output(LED2_PIN, self._GPIO.HIGH)
        self._play_melody(CONNECT_MELODY)

    def on_disconnect(self):
        if not self._available:
            return
        self._GPIO.output(LED2_PIN, self._GPIO.LOW)

    def set_remote_led(self, active: bool):
        if not self._available:
            return
        self._GPIO.output(LED1_PIN, self._GPIO.HIGH if active else self._GPIO.LOW)

    def _play_melody(self, melody):
        def _run():
            pwm = self._buzzer_pwm
            for freq, dur_ms, gap_ms in melody:
                pwm.ChangeFrequency(freq)
                pwm.start(BUZZ_DUTY)
                time.sleep(dur_ms / 1000)
                pwm.stop()
                if gap_ms > 0:
                    time.sleep(gap_ms / 1000)
        threading.Thread(target=_run, daemon=True).start()

    def cleanup(self):
        if not self._available:
            return
        if self._buzzer_pwm:
            self._buzzer_pwm.stop()
        self._GPIO.cleanup()
