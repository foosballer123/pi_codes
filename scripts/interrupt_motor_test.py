#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

ENCODER_6_PIN = 2  # GPIO 2 (BCM)

# State flag: True when sensor is triggered
triggered = False

def encoder_callback(channel):
    global triggered
    triggered = True

def setup():
    GPIO.setwarnings(False)
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENCODER_6_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.add_event_detect(ENCODER_6_PIN, GPIO.RISING, callback=encoder_callback, bouncetime=5)

if __name__ == '__main__':
    try:
        setup()
        while True:
            if triggered:
                print(1)
                triggered = False
            else:
                print(0)
            time.sleep(0.1)  # adjust polling rate if needed
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
