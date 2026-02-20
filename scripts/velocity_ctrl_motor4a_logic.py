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
from geometry_msgs.msg import Twist

# Global variable to store ball position
omega_d = None
bound = 0.3

pt = 0.00025 # fastest speed (31 rad/s)
up = True # ignore this for now
# GPIO pins for motor control

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)

GPIO.setup(md.motor[4]["SENSOR"], GPIO.IN)

# TO-DO: Define global command receieved flag as false
def pos_callback(data):
    """
    Callback function for receiving ball position data.
    Updates the global variable ball_pos.
    """
    global omega_d
    global pt
    omega_d = data
    tick_val = 0.015708 # in radians 2pi/400

    if abs(omega_d.linear.x) > bound:
        pw = float(tick_val/abs(omega_d.linear.x)) # omega = delta_theta / delta_t
        pt = pw/2
        #pt = 1.0
        # update the global flag to true
    
    #print('Got this pt value:', pt)   
    #print('From omega', omega_d.linear.x)

if __name__ == '__main__':
    
    i = 0
    j = 0 # number of complete rotations (2pi radians)

    rospy.init_node('motor4_logic', anonymous=True)
    rate = rospy.Rate(2000) # 2000 hz loop rate 

    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)
    step_pub = rospy.Publisher("/step", Twist, queue_size=10)
    step = Twist()

    # sleep to wait for omega_d to update (TO-DO: replace this with a 'command recieved' flag)
    time.sleep(1)

    state_flag = 0
    move_flag = 0
    t_s = time.time()
    t_k = 0
    while not rospy.is_shutdown():
        
        state = GPIO.input(md.motor[4]["SENSOR"])

        if (state == 1) and (move_flag == 0):
            move_flag = 1
            t_k = time.time()
            
        if (state == 1) and (state_flag == 0):
            state_flag = 1
            j += 1
        elif (state == 0) and (state_flag == 1):
            state_flag = 0

        if omega_d.linear.x > bound:
            step.linear.x = 1.0
        elif omega_d.linear.x < bound:
            step.linear.x = -1.0
        else:
            step.linear.x = 0.0

        step.linear.y = pt
        step_pub.publish(step)

        if t_k != 0:
            delta_t = (time.time()-t_k)
            print("Triggers", j)
            print("Real omega=", (j*2*math.pi)/delta_t)

        i += 1
        rate.sleep() # if our loop is too fast we risk adding error to our calculation 
            

# Clean up GPIO settings and displaying loop rate
delta_t = (time.time()-t_s)
print("Iterations", i)
print("Real HZ =", i/delta_t)
print("Triggers", j)
print("Real omega=", (j*2*math.pi)/delta_t)
GPIO.cleanup()




# Notes:
# add ros spin once (or spin) and ros sleep (not time sleep) so that different timing mechanisms stop conflicting
# ros spin helps with thread handling  

# to convert to linear velocity figure out the correlation between one tick and horizontal distance
# put a ruler on the table and send a known number of ticks to the system to convert from ticks to meters

# clean up this code so that we can easily spawn instances of it for each motor

# send 200 ticks in one direction and 200 ticks in the opposite direction to test if the system can return to an initial configuraiton
# do this 30 times to test how reliable the system is