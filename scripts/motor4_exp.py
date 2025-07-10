#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import enc_states 
import time
import numpy as np

# Global variable to store ball position
omega_d = None

pt = 0.00025 # fastest speed (31 rad/s)
up = True # ignore this for now
# GPIO pins for motor control
PUL_PIN = 24  # Pulse pin for stepper motor
DIR_PIN = 23  # Direction pin for stepper motor

# Sensor variable
sensor = 0

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
#GPIO.setup(2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
current_dir = GPIO.input(DIR_PIN)

"""
#optical encoder detection
def sensor_callback(channel):
    current_dir = GPIO.input(DIR_PIN)
    GPIO.output(DIR_PIN, not current_dir)  # Flip motor direction


while not rospy.is_shutdown():
    sensor_state = GPIO.input(2)
    rospy.loginfo(f"Sensor state: {sensor_state}")
    time.sleep(0.1)

GPIO.add_event_detect(2, GPIO.RISING, callback=sensor_callback, bouncetime=5)
"""
def pos_callback(data):
    """
    Callback function for receiving ball position data.
    Updates the global variable ball_pos.
    """
    global omega_d
    global pt
    omega_d = data
    tick_val = 0.015708

    if abs(omega_d.linear.x) > 0.3:
        pw = float(tick_val/abs(omega_d.linear.x))
        pt = pw/2
        #pt = 1.0
    if omega_d.linear.x >= 0:
        GPIO.output(DIR_PIN, GPIO.HIGH)
    else:
        GPIO.output(DIR_PIN, GPIO.LOW)  # Flip motor direction

    print('Got this pt value:',pt, 'for ',omega_d.linear.x, 'and ',abs(omega_d.linear.x))   

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
    rospy.init_node('motor4', anonymous=True)
    
    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)

    
    # Move motor until sensor is triggered
    while sensor != 1:
        step()
        sensor = enc_states.enc_status_6()
    
    GPIO.output(DIR_PIN, GPIO.HIGH)  # Change motor direction
    
    # Perform initial stepping motion
    for i in range(100):
        step()
    while not rospy.is_shutdown():
        
        if abs(omega_d.linear.x) > 0.3:
            step()
        #time.sleep(0.005)

    
                        
# Clean up GPIO settings
GPIO.cleanup()
