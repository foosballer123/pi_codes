#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

# GPIO setup
PUL_PIN = 24  # Pulse pin for stepper motor
DIR_PIN = 23  # Direction pin for stepper motor

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

GPIO.output(DIR_PIN, GPIO.LOW)

def step():
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.001)

try:
    for _ in range(400):  # One full revolution
        step()
finally:
    GPIO.cleanup()

