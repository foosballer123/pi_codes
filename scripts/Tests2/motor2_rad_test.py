#!/usr/bin/env python3

# Velocity Control Code for Motor 2 - 10/23/2025                     (Copy of Motor 4)
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
import time
import math

# Global variable to store ball position

#pt = 0.00025 # fastest speed (31 rad/s)
pt = 0.001 # initial pt
up = True # ignore this for now
i = 0 # direction vector
pos = 0

# GPIO pins for motor control
PUL_PIN = 26  # Pulse pin for stepper motor
DIR_PIN = 19  # Direction pin for stepper motor

# GPIO pins for encoders
SENSOR = 4

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# GPIO for encoder states
GPIO.setup(SENSOR, GPIO.IN)

# Sensor variable
sensor = GPIO.input(SENSOR) 


def step():
    """
    Function to generate a single step pulse for the motor.
    """
    global pt
    global i
    global pos

    if pt < 0.1:
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(pt)  
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(pt)
   
        pos += i

if __name__ == '__main__':
    
    while not rospy.is_shutdown():
        sensor_flag = 0
        pos = 0

        dir = int(input("Dir (0 = CCW and 1 = CW): "))
        if dir == 0:
            GPIO.output(DIR_PIN, GPIO.LOW)  # Set motor direction
            i = 1
        elif dir == 1:
            GPIO.output(DIR_PIN, GPIO.HIGH) 
            i = -1

        rospy.init_node('motor2', anonymous=True)
        #rate = rospy.Rate(3000) # hz loop rate 

        print("Initializing...")
    
        # WARNING: Encoder for motor 2a is malfunctioning!! 
        sensor = GPIO.input(SENSOR)
        while sensor != 1:
            step()
            sensor = GPIO.input(SENSOR) 
        print("Initialized to 0")
        pos = 0

        x = float(input("Move "+str(i)+"*X*Pi... X = "))
        while abs((2*math.pi)/400 * pos) <= abs(x*math.pi):
            step()

        print("Moved to", (2*math.pi)/400 * pos)
        input("Press enter to continue...")
    
            
# Clean up GPIO settings and displaying loop rate
GPIO.cleanup()


