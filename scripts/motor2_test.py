#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import enc_states_test 
import time
import numpy as np

# Global variable to store ball position
ball_pos = None
temp_sensor=False

# GPIO pins for motor control
PUL_PIN = 26  # Pulse pin for stepper motor
DIR_PIN = 19  # Direction pin for stepper motor
SENSOR_PIN = 4  # Sensor pin for interrupt (ENCODER_3)

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Interrupt flag
sensor_triggered = False

def sensor_callback(channel):
    """
    Interrupt callback for sensor pin.
    Sets the global sensor_triggered flag to True when triggered.
    """
    global sensor_triggered
    sensor_triggered = True

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
    time.sleep(0.025)  # Smallest delay possible for step pulse
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.025)  # Smallest delay possible for step pulse

if __name__ == '__main__':
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor2', anonymous=True)

    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/ball_pos", Twist, pos_callback, queue_size=10)

    # Set up interrupt for the sensor pin
    GPIO.add_event_detect(SENSOR_PIN, GPIO.RISING, callback=sensor_callback, bouncetime=10)

    # Move motor until sensor is triggered
    while ((not sensor_triggered and not temp_sensor) or (not sensor_triggered and temp_sensor)):
        step()

    GPIO.output(DIR_PIN, GPIO.HIGH)  # Change motor direction

    # Perform initial stepping motion
    for i in range(100):
        step()

    defense = np.arange(80, 130)  # Define defense zone range
    while not rospy.is_shutdown():
        if ball_pos is not None:
            # Read x position from ball position data
            position = ball_pos.linear.x
            sensor_triggered = False  # Reset interrupt flag

            # Check if ball is within the defense zone
            if int(position) in defense:
                GPIO.output(DIR_PIN, GPIO.LOW)  # Set direction for defensive movement

                # Move back until sensor is triggered
                while ((not sensor_triggered and not temp_sensor) or (not sensor_triggered and temp_sensor)):
                    step()

                # Rotate 90 deg to reset position
                GPIO.output(DIR_PIN, GPIO.HIGH)  # Reverse direction
                for i in range(100):
                    step()

    # Clean up GPIO settings
    GPIO.remove_event_detect(SENSOR_PIN)
    GPIO.cleanup()

