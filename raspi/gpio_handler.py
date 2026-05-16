import threading
import logging

logger = logging.getLogger(__name__)

BUZZER_PIN  = 18
LED1_PIN    = 19  # remote control indicator
LED2_PIN    = 13  # PC connection indicator

BUZZ_FREQ_HZ   = 2000
BUZZ_DUTY      = 50
BUZZ_DURATION  = 0.1  # seconds


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
            self._buzzer_pwm = GPIO.PWM(BUZZER_PIN, BUZZ_FREQ_HZ)
            self._available = True
        except Exception as e:
            logger.warning("GPIO not available: %s", e)

    def on_connect(self):
        if not self._available:
            return
        self._GPIO.output(LED2_PIN, self._GPIO.HIGH)
        self._buzz()

    def on_disconnect(self):
        if not self._available:
            return
        self._GPIO.output(LED2_PIN, self._GPIO.LOW)

    def set_remote_led(self, active: bool):
        if not self._available:
            return
        self._GPIO.output(LED1_PIN, self._GPIO.HIGH if active else self._GPIO.LOW)

    def _buzz(self):
        pwm = self._buzzer_pwm
        pwm.start(BUZZ_DUTY)
        t = threading.Timer(BUZZ_DURATION, pwm.stop)
        t.daemon = True
        t.start()

    def cleanup(self):
        if not self._available:
            return
        if self._buzzer_pwm:
            self._buzzer_pwm.stop()
        self._GPIO.cleanup()
