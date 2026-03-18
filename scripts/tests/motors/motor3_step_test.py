#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import enc_states 
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

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Sensor variable
right = 0 # encoder 4 (based on player facing)
left = 0 # encoder 5 (based on player facing)

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
    
    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)
    
    time.sleep(2)

    GPIO.output(DIR_PIN, GPIO.HIGH)  # Change motor direction
    key = 0

    while not rospy.is_shutdown():

        right = enc_states.enc_status_4()
        left = enc_states.enc_status_5()

        if key == 0:
            GPIO.output(DIR_PIN, GPIO.HIGH)
            step()

        if key == 1:
            GPIO.output(DIR_PIN, GPIO.LOW)
            step()

   
 
            

      
                
                        
# Clean up GPIO settings
GPIO.cleanup()