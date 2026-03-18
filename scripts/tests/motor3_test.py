#!/usr/bin/env python3

# test code for motor 3 - 10/24/2025
# 
# includes an updated step function that takes both a desired number of steps and a desired velocity as input
# while also checking to make sure that commands exist within the bounds of the table

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import motor_dict as md

# Global variables to store users command and position readings from encoder
user_cmd = None
cmd_received = False
pos = 0

pt = 0.00025 # fastest speed (31 rad/s) 

# GPIO pins for motor control
PUL_PIN = md.motor[3]["PUL"]  # Pulse pin for stepper motor
DIR_PIN = md.motor[3]["DIR"]  # Direction pin for stepper motor

# GPIO pins for encoders
SENSOR_R = md.motor[3]["SENSOR R"]
SENSOR_L = md.motor[3]["SENSOR L"]

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# GPIO for encoder states
GPIO.setup(SENSOR_R, GPIO.IN)
GPIO.setup(SENSOR_L, GPIO.IN)

# Sensor variable
right = GPIO.input(SENSOR_R) # encoder 4 (based on player facing)
left = GPIO.input(SENSOR_L) # encoder 5 (based on player facing)

def pos_callback(data):
    """
    Callback function for receiving ball position data.
    Updates the global variable ball_pos.
    """
    global user_cmd
    global cmd_received 
    user_cmd = data
    cmd_received = True

    #rospy.loginfo(f"Received input: {user_cmd}")

def step(steps, omega_d):
    """
    Function to step a desired number of pulses with a given omega.
    """
    global pos
    min_signal = 0.3   # ignoring tiny inputs for a cleaner signal 
    s = False
    dir = -1   # negative direction is away from camera 

    # bounding the input while retaining its sign
    omega_d = omega_d/abs(omega_d) * min(31, abs(omega_d))

    tick_val = 0.015708
    pw = float(tick_val/abs(omega_d))
    pt = pw/2

    for _ in range(int(steps)):

        right = GPIO.input(SENSOR_R)
        left = GPIO.input(SENSOR_L)
        
        if abs(omega_d) > min_signal:
            
            if not (right or left):
                print(omega_d)
                if omega_d > 0:
                    GPIO.output(DIR_PIN, GPIO.HIGH)
                    dir = -1
                    s = True
                if omega_d < 0:
                    GPIO.output(DIR_PIN, GPIO.LOW)
                    dir = 1
                    s = True
            elif right and not left:
                pos = 0
                if omega_d < 0:
                    GPIO.output(DIR_PIN, GPIO.LOW)
                    dir = 1
                    s = True
                else:
                    s = False
            elif left and not right:
                pos = 525 # approximate max y range of table
                if omega_d > 0:
                    GPIO.output(DIR_PIN, GPIO.HIGH)
                    dir = -1
                    s = True
                else:
                    s = False
            else:
                s = False
        else:
            s = False
            
        if s == True:

            GPIO.output(PUL_PIN, GPIO.HIGH)
            time.sleep(pt)  # Smallest delay possible for step pulse
            GPIO.output(PUL_PIN, GPIO.LOW)
            time.sleep(pt)  # Smallest delay possible for step pulse
            
            # tracking changes in the position of the rod
            pos += dir
            print("Position:", pos)

        else:
            print("Wall detected! Reset Position to", pos)

if __name__ == '__main__':

    rospy.init_node('motor3', anonymous=True)
    cmd_sub = rospy.Subscriber("/motor_cmd", Twist, pos_callback, queue_size=10)
    rate = rospy.Rate(2000)

    # return to initial position to initialze pos reading
    print("Initializing...")
    step(525, 15)
    step(5, 5)  # extra step to make sure sensor is triggered

    time.sleep(2) 
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction

    while not rospy.is_shutdown():

        if cmd_received == True:
            
            # move to a desired position (y==1)? or make a regular step...
            if user_cmd.angular.y == 1.0:  # move to desired pos
                dir_d = ((pos - user_cmd.linear.y) + 0.01) / (abs(pos - user_cmd.linear.y) + 0.01)
                steps_d = abs(pos - user_cmd.linear.y)

                step(steps_d, dir_d * abs(user_cmd.angular.x))
            else:    # regular step
                step(user_cmd.linear.x, user_cmd.angular.x)

            # stop and wait command
            if user_cmd.angular.z == 1.0:
                input("Press ENTER to step.")
        else:
            print("Waiting for command...")

 
        
# Clean up GPIO settings
GPIO.cleanup()
