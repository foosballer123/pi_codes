#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import math

# GPIO setup
PUL_PIN = 24  # Pulse pin for stepper motor
DIR_PIN = 23  # Direction pin for stepper motor

# motor 1 pins
# PUL_PIN = 27  # Pulse pin for stepper motor
# DIR_PIN = 17  # Direction pin for stepper motor

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

GPIO.output(DIR_PIN, GPIO.LOW)

# maximum velocity is around 40 radians per second
def step(omega_d):

    tick_val = 0.015708
    pw = float(tick_val/abs(omega_d))
    pt = pw/2

    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(pt)
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(pt)

try:
    t_s = time.time()
    for _ in range(10):  # One full revolution is 400 steps
        step(0.5)
    t_f = time.time()

finally:
    print(t_f - t_s)
    GPIO.cleanup()

