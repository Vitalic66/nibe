import time
import subprocess
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)  # Beispiel: GPIO18 verwenden

def is_program_running(name):
    try:
        subprocess.check_output(["pgrep", "-f", name])
        return True
    except subprocess.CalledProcessError:
        return False

try:
    while True:
        if is_program_running("acvm270_keypress"):
            GPIO.output(18, GPIO.HIGH)
        else:
            GPIO.output(18, GPIO.LOW)
        time.sleep(5)  # alle 5 Sekunden prüfen
except KeyboardInterrupt:
    GPIO.cleanup()
