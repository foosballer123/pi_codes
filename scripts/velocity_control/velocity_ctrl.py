#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float64MultiArray
import math
from libraries import tools # import libraries for step and direction/pw calculations
import argparse

parser = argparse.ArgumentParser(
    description="Velocity control scripts that actuates motors based on recieved commands."
)
parser.add_argument("--motor", required=True, type=int)
args = parser.parse_args(rospy.myargv()[1:])

motor = args.motor

# Global variable to store ball position
omega_d = None # temporary variable for velocity commands
cmd_recieved = False # command recieved flag
init = False # initialization flag
i = 1 # initial direction
pos = 0 # initial position
pt = 0.001 # initial speed

# initial measurements
rad, meters = 0, 0
player_positions = Float64MultiArray()

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
    # Note: Direction of angular controllers are opposite of linear controllers for consistency with solver  
    if motor == 1:
        omega_d = data.linear.x
    elif motor == 2:
        omega_d = -data.angular.x
    elif motor == 3:
        omega_d = data.linear.y
    elif motor == 4:
        omega_d = -data.angular.y
    elif motor == 5:
        omega_d = data.linear.z
    elif motor == 6:
        omega_d = -data.angular.z

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
    player_position_pub = rospy.Publisher("/rod"+str(motor)+"_player_positions", Float64MultiArray, queue_size=10)

    # Get system parameters from ros config on master
    field_height = rospy.get_param('/table_measurements/field_height')
    steps_across_field = rospy.get_param('/table_measurements/steps_across_field')
    steps_per_revolution = rospy.get_param('/table_measurements/steps_per_revolution')
    distance_between_players = rospy.get_param('/table_measurements/distance_between_players')
    player_distance_from_wall = rospy.get_param('/table_measurements/player_distance_from_wall')
    distance_to_clear_zone = rospy.get_param('/table_measurements/distance_to_clear_zone')

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
        pos = tools.OFFSET[0]
        while pos != 0:
            pos = tools.step(PUL_PIN, pt, pos, i)
        init = True
        #print("Error: ", type(pos))
        print('Successfully initialized motor '+str(motor)+' to 0!')

    print('Starting control loop for motor '+str(motor)+'.')

    time.sleep(2) 

    # Start control loop!
    while not rospy.is_shutdown():
        
        # Updating variables with each iteration of the control loop with tools.control()
        #   measurements: rad, meters, pos (calculated rad and meters tools.calculate())
        #   encoder values: left_encoder, encoder, right_encoder, encoder_flag
        #   motor type: linear, angular

        # print("Motor Paramters:")
        # print(
        #     motor, # motor id
        #     pos, i, pt, # step parameters
        #     SENSOR_R, SENSOR_L, SENSOR, encoder_flag, # encoder parameters
        #     cmd_recieved, PUL_PIN, # actuation parameters
        # )

        pos, \
        left_encoder, encoder, right_encoder, encoder_flag, \
        angular, linear \
        = tools.control(
            motor, # motor id
            pos, i, pt, # step parameters
            SENSOR_R, SENSOR_L, SENSOR, encoder_flag, # encoder parameters
            cmd_recieved, PUL_PIN # actuation parameters
        )

        #print("Encoders:", left_encoder, right_encoder)
        # if angular:
        #     print("Error 2: ", pos, type(pos))
        rad, meters = tools.calculate(motor, pos, steps_across_field, steps_per_revolution, distance_between_players, player_distance_from_wall, distance_to_clear_zone)

        # publishing measurements based on motor type
        if linear: 
            player_positions.data = meters
            player_position_pub.publish(player_positions); left_enc_pub.publish(left_encoder); right_enc_pub.publish(right_encoder)
        if angular: 
            player_positions.data = rad
            player_position_pub.publish(player_positions); enc_pub.publish(encoder)


print('Shutting down motor '+str(motor)+'.')

# Clean up GPIO settings
GPIO.cleanup()