#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import numpy as np
from std_msgs.msg import Float32

# Global variable to store ball position
i = 1
pos = 0 
init = False


pt = 0.0005 # initial speed

# GPIO pins for motor control
PUL_PIN = 27  # Pulse pin for stepper motor
DIR_PIN = 17  # Direction pin for stepper motor
# GPIO pins for encoders 1 and 2
SENSOR_1 = 6
SENSOR_2 = 5

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# GPIO for encoder states
GPIO.setup(SENSOR_1, GPIO.IN)
GPIO.setup(SENSOR_2, GPIO.IN)

# Sensor variable
right = GPIO.input(SENSOR_1) # encoder 1 (based on player facing)
left = GPIO.input(SENSOR_2) # encoder 2 (based on player facing)

def step():
    """
    Function to generate a single step pulse for the motor.
    """
    global pt
    global pos
    global i

    if pt < 0.1:

        #print(pt,'inside the step')
        GPIO.output(PUL_PIN, GPIO.HIGH)
        time.sleep(pt)  # Smallest delay possible for step pulse
        GPIO.output(PUL_PIN, GPIO.LOW)
        time.sleep(pt)  # Smallest delay possible for step pulse

        pos += i

# HIGH = step towards hardware
# LOW = step away from hardware
# pos = 525 closest to hardware    
if __name__ == '__main__':

    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor1', anonymous=True)
    rate = rospy.Rate(10) # loop rate

    pos_pub = rospy.Publisher("/motor1_pos", Float32, queue_size=10)

    left = GPIO.input(SENSOR_2) # left is 0 right is 525
    while not left:
        step()
        left = GPIO.input(SENSOR_2)
    init = True 
    pos = 0

    print("Successfully initialized to", pos)

    x = int(input("Move X ???... X ="))
    GPIO.output(DIR_PIN, GPIO.HIGH)  # Set opposite motor direction
    for k in range(x):
        step()

    # TESTING
    while not rospy.is_shutdown():
        pos_pub.publish(pos)
        rate.sleep()

# Clean up GPIO settings
GPIO.cleanup()