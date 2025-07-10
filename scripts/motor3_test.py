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

class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()

    def compute(self, measurement):
        error = self.setpoint - measurement
        current_time = time.time()
        delta_time = current_time - self.last_time

        if delta_time == 0:
            return 0

        self.integral += error * delta_time
        derivative = (error - self.last_error) / delta_time

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.last_error = error
        self.last_time = current_time

        return output

def step(delay):
    """
    Function to generate a single step pulse for the motor, using variable delay.
    """
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(delay)
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(delay)

if __name__ == '__main__':
    counter = 0  # Counter to track motor steps
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor3', anonymous=True)
    
    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/ball_pos", Twist, pos_callback, queue_size=10)
    
    # PID Controller Initialization (Tune these values)
    pid = PID(kp=0.75, ki=0.01, kd=0.05)

    # Move motor until sensor is triggered
    while sensor != 1:
        step(0.0005)
        sensor = enc_states.enc_status_5()
    
    GPIO.output(DIR_PIN, GPIO.HIGH)  # Change motor direction
    
    while not rospy.is_shutdown():
        if ball_pos is not None:
            ball_to_player = ball_pos.linear.z - counter
            
            if abs(ball_to_player) > THRESHOLD:
                if ball_to_player > 0:
                    GPIO.output(DIR_PIN, GPIO.HIGH)
                    counter += 1
                elif ball_to_player < 0:
                    GPIO.output(DIR_PIN, GPIO.LOW)
                    counter -= 1

                # Compute dynamic delay from PID controller
                speed = abs(pid.compute(ball_to_player))*10
                print (speed)
                speed = max(min(abs(speed), 2000), 100)
                delay = 1.0/speed
                print (delay)
                step(delay)
                THRESHOLD = 1
            else:
                THRESHOLD = 10  # Increase threshold to reduce jitter

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

