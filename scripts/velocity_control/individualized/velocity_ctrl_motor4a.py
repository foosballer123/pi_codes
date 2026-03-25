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
from std_msgs.msg import Float32

# Global variable to store ball position
omega_d = None
cmd_recieved = False

#pt = 0.00025 # fastest speed (31 rad/s)
pt = 0.001
up = True # ignore this for now
i = 0 # direction vector

# GPIO pins for motor control
PUL_PIN = 24  # Pulse pin for stepper motor
DIR_PIN = 23  # Direction pin for stepper motor

# GPIO pins for encoders
SENSOR = 2

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# GPIO for encoder states
GPIO.setup(SENSOR, GPIO.IN)

# Sensor variable
sensor = GPIO.input(SENSOR) 

# TO-DO: Define global command receieved flag as false
def cmd_callback(data):
    """
    Callback function for receiving velocity commands.
    Updates the global variable oemga_d.
    """
    global omega_d
    global pt
    global cmd_recieved
    global i

    if cmd_recieved == False:
        cmd_recieved = True
    omega_d = data.linear.x

    tick_val = 0.015708 # in radians 2pi/400
    timing_constant = 0.000178 # seconds (10^-6 is microseconds)

    if omega_d != 0.0:

        pw = float(tick_val/abs(omega_d)) - timing_constant # omega = delta_theta / delta_t 
        pt = pw/2

        if omega_d > 0.0:
            GPIO.output(DIR_PIN, GPIO.LOW)
            i = 1
        elif omega_d < 0.0:
            GPIO.output(DIR_PIN, GPIO.HIGH)
            i = -1

        # update the global flag to true (command recieved)
        #print('Got this pt value:', pt)   
        #print('From omega', omega_d)
    else:
        cmd_recieved = False

def step():
    """
    Function to generate a single step pulse for the motor.
    """
    global pt
 
    GPIO.output(PUL_PIN, GPIO.HIGH)
    rospy.sleep(pt)
    GPIO.output(PUL_PIN, GPIO.LOW)
    rospy.sleep(pt)  # Smallest delay possible for step pulse
   

if __name__ == '__main__':
    
    sensor_flag = 0
    pos = 0

    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor4', anonymous=True)
    #rate = rospy.Rate(2000) # 2000 hz loop rate 

    # Subscribe to the ball position topic
    cmd_sub = rospy.Subscriber("/omega_d_4", Twist, cmd_callback, queue_size=10)
    pos_pub = rospy.Publisher("/motor4_rad", Float32, queue_size=10)

    print("Initializing...")

    sensor = GPIO.input(SENSOR)
    while sensor != 1:
        step()
        sensor = GPIO.input(SENSOR) 
    print("Initialized")

    # sleep to wait for omega_d to update (TO-DO: replace this with a 'command recieved' flag)
    time.sleep(1)

    #t_s = time.time()
    while not rospy.is_shutdown():

        sensor = GPIO.input(SENSOR) 

        # counts a complete rotation on encoder HIGH
        if (sensor == 1) and (sensor_flag == 0):
            #t_n = time.time()
            sensor_flag = 1
            #omega_real = (2*math.pi) / (t_n - time_trigger)
            #print("Real omega:", omega_real )
            #or_pub.publish(omega_real)
            #time_trigger = t_n
            pos = 0

        elif (sensor == 0) and (sensor_flag == 1):
            sensor_flag = 0

        if cmd_recieved:
            # TO-DO: if 'command recieved' is true
            step()
            pos += i
            #print(pos)
        
        rad = pos*((2*math.pi)/400)
        pos_pub.publish(rad) #(math.cos(rad)) 
        #rate.sleep()
            

# Clean up GPIO settings
GPIO.cleanup()




# Notes:
# add ros spin once (or spin) and ros sleep (not time sleep) so that different timing mechanisms stop conflicting
# ros spin helps with thread handling  

# to convert to linear velocity figure out the correlation between one tick and horizontal distance
# put a ruler on the table and send a known number of ticks to the system to convert from ticks to meters

# clean up this code so that we can easily spawn instances of it for each motor

# send 200 ticks in one direction and 200 ticks in the opposite direction to test if the system can return to an initial configuraiton
# do this 30 times to test how reliable the system is