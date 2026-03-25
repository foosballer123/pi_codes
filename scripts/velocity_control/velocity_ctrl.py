#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float32
import math
from libraries import tools # import libraries for step and direction/pw calculations
import argparse

parser = argparse.ArgumentParser(
        description="Velocity control scripts that actuates motors based on recieved commands."
)
parser.add_argument("--motor", required=True, type=int)
args = parser.parse_args()

motor = args.motor

# Global variable to store ball position
omega_d = None # temporary variable for velocity commands
cmd_recieved = False # command recieved flag
init = False # initialization flag
i = 1 # initial direction
pos = 0 # initial position
pt = 0.001 # initial speed

# Get and set pins for motors and encoders
PUL_PIN, DIR_PIN, SENSOR_L, SENSOR_R, SENSOR = tools.get_pins(motor)
tools.setup_pins(motor, PUL_PIN, DIR_PIN, SENSOR_R, SENSOR_L, SENSOR)

def pos_callback(data):
    """
    Callback function for receiving motor commands.
    Updates the global variable 'pt' for time between steps.
    """
    global omega_d
    global pt
    global i
    global cmd_recieved
    global init 
    
    if cmd_recieved == False:
        cmd_recieved = True

    # Read command signal based on motor number  
    if motor == 1:
        omega_d = data.linear.x
    elif motor == 2:
        omega_d = data.angular.x
    elif motor == 3:
        omega_d = data.linear.y
    elif motor == 4:
        omega_d = data.angular.y
    elif motor == 5:
        omega_d = data.linear.z
    elif motor == 6:
        omega_d = data.angular.z

    # calculating step commands based on incoming omega_d and setting global variables
    i, pt, cmd_recieved = tools.get_step(init, i, omega_d, DIR_PIN)

  
if __name__ == '__main__':

    '''
    HIGH -> Step Towards Hardware and XX (Angular Motion)
    LOW -> Step Away From Hardware (Linear Motion) and XX (Angular Motion)
    pos = 525 is closest to hardware
    '''

    encoder_flag = 0

    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor'+str(motor), anonymous=True)

    # Subscribe to the command topic and create a position publisher
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)
    pos_pub = rospy.Publisher("/motor"+str(motor)+"_pos", Float32, queue_size=10)

    # Create encoder publishers based on motor (Linear vs. Angular)
    if (motor % 2) == 1:
        right_enc_pub = rospy.Publisher("motor"+str(motor)+"_right_encoder", Float32, queue_size=10)
        left_enc_pub = rospy.Publisher("motor"+str(motor)+"_left_encoder", Float32, queue_size=10)
    if (motor % 2) == 0:
        enc_pub = rospy.Publisher("motor"+str(motor)+"_encoder", Float32, queue_size=10)

    # Initialization for odd motors (1 or 3 or 2n-1: linear motor)
    if (motor % 2) == 1:
        left_encoder = GPIO.input(SENSOR_L) # left is 0 right is 525
        while not left_encoder:
            _ = tools.step(PUL_PIN, pt, pos, i)
            left_encoder = GPIO.input(SENSOR_L)
        init = True 
        pos = 0

        print('Successfully initialized motor '+str(motor)+' to 0!')

    # Initialization for even motors (2 or 4 or 2n : angular motors)
    if (motor % 2) == 0:
        encoder = GPIO.input(SENSOR)     
        while not encoder:
            _ = tools.step(PUL_PIN, pt, pos, i)
            encoder = GPIO.input(SENSOR) 
        init = True
        pos = 0

        print('Successfully initialized motor '+str(motor)+' to 0!')

    print('Starting control loop for motor '+str(motor)+'.')

    time.sleep(2) 

    # Start control loop!
    while not rospy.is_shutdown():

        if (motor % 2) == 1:

            # reading encoder values
            right_encoder = GPIO.input(SENSOR_R) # 525 steps
            left_encoder = GPIO.input(SENSOR_L) # 0 steps
   
            if cmd_recieved:
                
                # Checking encoder values before sending step commands to the motors to prevent wall collisions
                if not (right_encoder or left_encoder):
                    pos = tools.step(PUL_PIN, pt, pos, i) # stepping and updating position
                    
                elif right_encoder and not left_encoder:
                    pos = 525
                    if i == -1:
                        pos = tools.step(PUL_PIN, pt, pos, i) # stepping and updating position
                        
                elif left_encoder and not right_encoder:
                    pos = 0
                    if i == 1:
                        pos = tools.step(PUL_PIN, pt, pos, i) # stepping and updating position

            meters = round(pos * ((0.340/3)/525), 7)
            pos_pub.publish(meters) # replace pix / step with meters / step
            left_enc_pub.publish(left_encoder)
            right_enc_pub.publish(right_encoder)
        
        if (motor % 2) == 0:

            # reading encoder value
            encoder = GPIO.input(SENSOR) 

            if (encoder == 1) and (encoder_flag == 0):
                encoder_flag = 1
                pos = 0
            elif (encoder == 0) and (encoder_flag == 1):
                encoder_flag = 0

            if cmd_recieved:
                pos = tools.step(PUL_PIN, pt, pos, i) # stepping and updating position
            
            rad = pos*((2*math.pi)/400) 
            pos_pub.publish(rad)
            enc_pub.publish(encoder)

print('Shutting down motor '+str(motor)+'.')

# Clean up GPIO settings
GPIO.cleanup()