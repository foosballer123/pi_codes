#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import enc_states 
import time

# Global variable to store ball position
ball_pos = None

# GPIO pins for motor control
PUL_PIN = 21  # Pulse pin for stepper motor
DIR_PIN = 20  # Direction pin for stepper motor

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Sensor variables
sensor = 0
temp_sensor4 = 0
temp_sensor5 = 0
THRESHOLD = 1  # Movement threshold

def pos_callback(data):
    """
    Callback function for receiving ball position data.
    Updates the global variable ball_pos.
    """
    global ball_pos
    ball_pos = data

def step():
    """
    Function to generate a single step pulse for the motor.
    """
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(0.005)  # Smallest delay possible for step pulse
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.005)  # Smallest delay possible for step pulse

if __name__ == '__main__':
    counter = 0  # Counter to track motor steps
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor3', anonymous=True)
    
    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/ball_pos", Twist, pos_callback, queue_size=10)
    
    # Move motor until sensor is triggered
    while sensor != 1:
        step()
        sensor = enc_states.enc_status_5()
    
    GPIO.output(DIR_PIN, GPIO.HIGH)  # Change motor direction
    
    while not rospy.is_shutdown():
        if ball_pos is not None:
            # Compute the difference between the ball position and motor position
            ball_to_player = ball_pos.linear.z - counter
            
            # If the difference is significant, adjust motor position
            if abs(ball_to_player) > THRESHOLD:
                if ball_to_player > 0:  # Move towards ball
                    GPIO.output(DIR_PIN, GPIO.HIGH)
                    counter += 1
                elif ball_to_player < 0:  # Move away from ball
                    GPIO.output(DIR_PIN, GPIO.LOW)
                    counter -= 1
                
                step()  # Perform a step movement
                THRESHOLD = 1  # Reset threshold
            else:
                THRESHOLD = 5  # Adjust threshold for filtering
            
            # Reset counter based on encoder status
            if enc_states.enc_status_5() == 1 and temp_sensor5 == 1:
                counter = 0
            elif enc_states.enc_status_4() == 1 and temp_sensor4 == 1:
                counter = 550
            
            # Update temporary sensor states
            temp_sensor5 = enc_states.enc_status_5()
            temp_sensor4 = enc_states.enc_status_4()

# Clean up GPIO settings
GPIO.cleanup()

