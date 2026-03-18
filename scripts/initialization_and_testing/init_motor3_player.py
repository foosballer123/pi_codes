#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Float32

pt = 0.001 

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

    pub = rospy.Publisher("/motor3_pos", Float32, queue_size=10)
    right_pub = rospy.Publisher("/right", Float32, queue_size=10)
    left_pub = rospy.Publisher("/left", Float32, queue_size=10)
    
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction (towards hardware)
    sensor_flag = 0

    while sensor_flag != 1:

        print("Callibrating...")
        left = GPIO.input(SENSOR_5)
        if (left == 1) and (sensor_flag == 0):
            sensor_flag = 1
            print("Finished callibration.")
            break
        step()

    pos = 0
    print("Position reset and published:", pos)
    pub.publish(pos)

    # TESTING LOOP
    while not rospy.is_shutdown():
        pub.publish(pos)
        
# Clean up GPIO settings
GPIO.cleanup()