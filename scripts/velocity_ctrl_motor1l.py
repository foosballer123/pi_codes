#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import numpy as np
from std_msgs.msg import Float32

# Global variable to store ball position
omega_d = None
cmd_recieved = False
i = 1
pos = 0 
init = False

#pt = 0.00025 # fastest speed (31 rad/s)
pt = 0.001 # initial speed
up = True # ignore this for now
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

def pos_callback(data):
    """
    Callback function for receiving ball position data.
    Updates the global variable ball_pos.
    """
    global omega_d
    global pt
    global i
    global cmd_recieved
    global init 
    
    if cmd_recieved == False:
        cmd_recieved = True
    omega_d = data.linear.x
    tick_val = 0.015708
    timing_constant = 0.000178 # seconds (10^-6 is microseconds)

    if init == True: # wait for init confirmation so that callback doesnt interfere with init process
        print("Recieved", omega_d)
        if ((abs(omega_d) < 35) and (abs(omega_d) > 0.1)):
            if omega_d > 0.0:
                GPIO.output(DIR_PIN, GPIO.HIGH)
                i = 1
                pw = float(tick_val/omega_d) - timing_constant
                pt = pw/2
                
            elif omega_d < 0.0:
                GPIO.output(DIR_PIN, GPIO.LOW)
                i = -1
                pw = float(tick_val/-omega_d) - timing_constant
                pt = pw/2
        else:
            cmd_recieved = False

    #print('Got this pt value:',pt)   

def step():
    """
    Function to generate a single step pulse for the motor.
    """
    global pt
    global pos
    global i
    #print("Absurdity: ", pt)

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
    #rate = rospy.Rate(2000) # loop rate

    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)
    pos_pub = rospy.Publisher("/motor1_pos", Float32, queue_size=10)
    vel_pub = rospy.Publisher("/motor1_vel", Float32, queue_size=10)
    right_enc_pub = rospy.Publisher("motor1_right_encoder", Float32, queue_size=10)
    left_enc_pub = rospy.Publisher("motor1_left_encoder", Float32, queue_size=10)

    left = GPIO.input(SENSOR_2) # left is 0 right is 525
    while not left:
        step()
        left = GPIO.input(SENSOR_2)
    init = True 
    pos = 0

    print("Successfully initialized to 0!")
    #print("Starting control loop...")    
    print("Stepping to ~center...")

    GPIO.output(DIR_PIN, GPIO.HIGH)  # Set initial motor direction
    for k in range(400):
        step()

    print("Stepped to ~center.","(",pos,")")
    print("Starting control loop...") 
    
    time.sleep(2) 
    t_s = 0
    while not rospy.is_shutdown():

        right = GPIO.input(SENSOR_1)
        left = GPIO.input(SENSOR_2)
        
        #if ~cmd_recieved:
        #    print("Waiting for command...")
        if cmd_recieved:
            
            t_n = time.time()
            if not (right or left):
                step()
                t_s = time.time() - t_n
            elif right and not left:
                pos = 525
                if i == -1:
                    step()
                    t_s = time.time() - t_n
            elif left and not right:
                pos = 0
                if i == 1:
                    step()
                    t_s = time.time() - t_n

        pos_pub.publish(pos)
        left_enc_pub.publish(left)
        right_enc_pub.publish(right)
        
        #vel_pub.publish(1/t_s)
        #rate.sleep()
        

# Clean up GPIO settings
GPIO.cleanup()