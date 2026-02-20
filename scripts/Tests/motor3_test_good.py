#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import math

# GPIO setup
PUL_PIN = 21  # Pulse pin for stepper motor
DIR_PIN = 20  # Direction pin for stepper motor

# GPIO pins for encoders
SENSOR_4 = 7
SENSOR_5 = 8

GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

GPIO.output(DIR_PIN, GPIO.LOW)

# GPIO for encoder states
GPIO.setup(SENSOR_4, GPIO.IN)
GPIO.setup(SENSOR_5, GPIO.IN)

# Sensor variable
right = GPIO.input(SENSOR_4) # encoder 4 (based on player facing)
left = GPIO.input(SENSOR_5) # encoder 5 (based on player facing)

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

    GPIO.output(DIR_PIN, GPIO.HIGH)  # Set initial motor direction (towards hardware)
    sensor_flag = 0

    while sensor_flag != 1:  

        print("Callibrating...")
        right = GPIO.input(SENSOR_4)
        if (right == 1) and (sensor_flag == 0):
            sensor_flag = 1
            print("Finished callibration.")
            break

        step()

    pos = 0

    GPIO.output(DIR_PIN, GPIO.HIGH)
    
    for _ in range(400):  # One full revolution is 400 steps
        pos += 1
        step(31)
   

finally:
    print(pos)
    GPIO.cleanup()

