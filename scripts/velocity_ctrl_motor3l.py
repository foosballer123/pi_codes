#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import numpy as np

# Global variable to store ball position
omega_d = None
bound = 0.3

pt = 0.00025 # fastest speed (31 rad/s)
up = True # ignore this for now
# GPIO pins for motor control
PUL_PIN = 21  # Pulse pin for stepper motor
DIR_PIN = 20  # Direction pin for stepper motor
# GPIO pins for encoders
SENSOR_4 = 7
SENSOR_5 = 8

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# GPIO for encoder states
GPIO.setup(SENSOR_4, GPIO.IN)
GPIO.setup(SENSOR_5, GPIO.IN)

# Sensor variable
right = GPIO.input(SENSOR_4) # encoder 4 (based on player facing)
left = GPIO.input(SENSOR_5) # encoder 5 (based on player facing)

def pos_callback(data):
    """
    Callback function for receiving ball position data.
    Updates the global variable ball_pos.
    """
    global omega_d
    global pt
    omega_d = data
    tick_val = 0.015708

    if omega_d.linear.x > bound:
        pw = float(tick_val/omega_d.linear.x)
        pt = pw/2
        #pt = 1.0
    if omega_d.linear.x < -bound:
        pw = float(tick_val/-omega_d.linear.x)
        pt = pw/2
        

    #print('Got this pt value:',pt)   

def step():
    """
    Function to generate a single step pulse for the motor.
    """
    global pt
    #print(pt,'inside the step')
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(pt)  # Smallest delay possible for step pulse
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(pt)  # Smallest delay possible for step pulse

if __name__ == '__main__':

    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor3', anonymous=True)
    rate = rospy.Rate(2000) # loop rate

    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)
    
    time.sleep(2) 

    while not rospy.is_shutdown():

        right = GPIO.input(SENSOR_4)
        left = GPIO.input(SENSOR_5)

        if not (right or left):
            
            if omega_d.linear.x > bound:
                GPIO.output(DIR_PIN, GPIO.HIGH)
                step()
            
            if omega_d.linear.x < -bound:
                GPIO.output(DIR_PIN, GPIO.LOW)
                step()

        if right and not left:
            
            if omega_d.linear.x < -bound:
                GPIO.output(DIR_PIN, GPIO.LOW)
                step()

        if left and not right:
            
            if omega_d.linear.x > bound:
                GPIO.output(DIR_PIN, GPIO.HIGH)
                step()
            
            
        rate.sleep()
        

# Clean up GPIO settings
GPIO.cleanup()