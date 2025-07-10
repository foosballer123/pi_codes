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
PUL_PIN = 27  # Pulse pin for stepper motor
DIR_PIN = 17  # Direction pin for stepper motor

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Sensor variables
sensor = 0
temp_sensor1 = 0
temp_sensor2 = 0
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
    time.sleep(0.0005)  # Small delay for step pulse
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.0005)  # Small delay for step pulse

if __name__ == '__main__':
    counter = 0  # Counter to track motor steps
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor1', anonymous=True)
    
    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/ball_pos", Twist, pos_callback, queue_size=10)
    
    # Move motor until sensor is triggered
    while sensor != 1:
        step()
        sensor = enc_states.enc_status_2()
    
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
                THRESHOLD = 10  # Adjust threshold for filtering
            
            # Debugging output
            #print(str(ball_to_player) + "---" + str(ball_pos.linear.z) + "---" + str(counter))
            
            # Reset counter based on encoder status
            if enc_states.enc_status_2() == 1 and temp_sensor2 == 1:
                counter = 0
            elif enc_states.enc_status_1() == 1 and temp_sensor1 == 1:
                counter = 550
            
            # Update temporary sensor states
            temp_sensor1 = enc_states.enc_status_1()
            temp_sensor2 = enc_states.enc_status_2()

# Clean up GPIO settings
GPIO.cleanup()

