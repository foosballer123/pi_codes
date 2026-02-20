#!/usr/bin/env python3

# Velocity Control Code for Motor 4 - 10/23/2025
# Written by Dr. Ahmed Saeidi, Joe Scott, and Benjamin Simpson

# The loop rate here is important: 
# A low loop rate is a bottleneck for actuation
# A high loop rate can lead to missed encoder readings
# A good loop rate should account for the sampling rate of the encoder as well as the maximum velocity of the motor

# It is also worth noting that the "sent velocity command" does not equate to what actually actuates
# Remember this when designing a world model - it cannot rely too heavily on precise actuation

# Equations worth remembering:
# omega = theta / time (in radians)
# time / revolution = 2pi / omega (in radians)

# after testing, the default loop rate of a python program seems to be ~24khz
# this is 21khz over the maximum sampling rate of our optical encoders (which is 3khz)

# the maximum omega for our motors seems to be ~40 radians/second
# however Dr. Saeidi wrote this code initially with 31 rad/s in mind

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import enc_states 
import time
import numpy as np
from Tests import motor_dict as md
import math

# GPIO pins for motor control
PUL_PIN = 24  # Pulse pin for stepper motor
DIR_PIN = 23  # Direction pin for stepper motor

step = None

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

def step_callback(data):
    """
    Callback function for receiving step flag.
    Updates the global variable step.
    """
    global step
    step = data

if __name__ == '__main__':
    
    i = 0
    
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor4_step', anonymous=True)
    step_sub = rospy.Subscriber("/step", Twist, step_callback, queue_size=10)
    rate = rospy.Rate(2000) # 2000 hz loop rate 

    # sleep to wait for omega_d to update (TO-DO: replace this with a 'command recieved' flag)
    time.sleep(1)

    t_s = time.time()
    while not rospy.is_shutdown():
        """
        Function to generate a single step pulse for the motor.
        """

        if step.linear.x == 1.0:
            GPIO.output(DIR_PIN, GPIO.HIGH)
               
        elif step.linear.x == -1.0:
            GPIO.output(DIR_PIN, GPIO.LOW)

        GPIO.output(PUL_PIN, GPIO.HIGH)
        rospy.sleep(0.000005)
        GPIO.output(PUL_PIN, GPIO.LOW)
        rospy.sleep(0.000005)  
     
        #i += 1
        rate.sleep() # if our loop is too fast we risk adding error to our calculation 
            

# Clean up GPIO settings and displaying loop rate
delta_t = (time.time()-t_s)
print("Iterations", i)
print("Real HZ =", i/delta_t)
GPIO.cleanup()




# Notes:
# add ros spin once (or spin) and ros sleep (not time sleep) so that different timing mechanisms stop conflicting
# ros spin helps with thread handling  

# to convert to linear velocity figure out the correlation between one tick and horizontal distance
# put a ruler on the table and send a known number of ticks to the system to convert from ticks to meters

# clean up this code so that we can easily spawn instances of it for each motor

# send 200 ticks in one direction and 200 ticks in the opposite direction to test if the system can return to an initial configuraiton
# do this 30 times to test how reliable the system is