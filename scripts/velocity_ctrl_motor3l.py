#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import numpy as np
from std_msgs.msg import Float32

# Global variable to store ball position
omega_d = None
bound = 0.3

#pt = 0.00025 # fastest speed (31 rad/s)
pt = 0.001 
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
        
    if abs(omega_d.linear.x) < 0.5:
        print('Got this pt value:',pt)
        print('From omega', omega_d.linear.x) 

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

# HIGH = step towards hardware
# LOW = step away from hardware
# pos = 525 closest to hardware 
if __name__ == '__main__':

    rospy.init_node('motor3', anonymous=True)

     # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)
    pub = rospy.Publisher("/motor3_pos", Float32, queue_size=10)
    right_pub = rospy.Publisher("/right", Float32, queue_size=10)
    left_pub = rospy.Publisher("/left", Float32, queue_size=10)
    #rate = rospy.Rate(2000) # loop rate
    
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

    pos = 525
    print("Position reset and published:", pos)
    pub.publish(pos)

    GPIO.output(DIR_PIN, GPIO.LOW)
    for i in range(200):
        step()
        pos += -1

    print("Stepped 200 and published:", pos)

    # # TESTING LOOP
    # while True:
    #     pub.publish(pos)

    time.sleep(2) 

    while not rospy.is_shutdown():

        right = GPIO.input(SENSOR_4)
        left = GPIO.input(SENSOR_5)
        right_pub.publish(right)
        left_pub.publish(left)

        if omega_d != None:
            if not (right or left):
                
                if (omega_d.linear.x > bound) and (pos != 0):
                    GPIO.output(DIR_PIN, GPIO.HIGH)
                    dir = 1
                    step()
                    pos += dir
                    pub.publish(pos)
                    print(pos)

                elif (omega_d.linear.x < -bound) and (pos != 525):
                    GPIO.output(DIR_PIN, GPIO.LOW)
                    dir = -1
                    step()
                    pos += dir
                    pub.publish(pos)
                    print(pos)

            elif right and not left:
                print("Ok buddy")
                pos = 525
                if omega_d.linear.x < -bound:
                    GPIO.output(DIR_PIN, GPIO.LOW)
                    dir = -1
                    step()
                    pos += dir
                    pub.publish(pos)
                    print(pos)

            elif left and not right:
                pos = 0
                if omega_d.linear.x > bound:
                    GPIO.output(DIR_PIN, GPIO.HIGH)
                    dir = 1
                    step()
                    pos += dir
                    pub.publish(pos)
                    print(pos)
            else:
                pub.publish(pos)
                print(pos)

        #rate.sleep()
        

# Clean up GPIO settings
GPIO.cleanup()