import time
import RPi.GPIO as GPIO
import math

OFFSET = (0, 0)

def step(pin, pt, pos, i):

    """
    Function to generate a single step pulse for the motor.
    """

    if pt < 0.1:

        GPIO.output(pin, GPIO.HIGH)
        time.sleep(pt)  # Smallest delay possible for step pulse
        GPIO.output(pin, GPIO.LOW)
        time.sleep(pt)  # Smallest delay possible for step pulse

        pos += i
    
    return pos

def get_step(init, i, omega_d, pin):

    """
    Function calculates step variables based on direction and omega_d.
    """

    pt = 0
    cmd_recieved = True
    tick_val = 0.015708
    timing_constant = 0.000178 # seconds (10^-6 is microseconds)

    if init == True: # wait for init confirmation so that callback doesnt interfere with init process
       
        if ((abs(omega_d) < 40) and (abs(omega_d) > 0.1)): # make omega_d.max a config. variable

            if omega_d > 0.0:
                GPIO.output(pin, GPIO.HIGH)
                i = 1
                pw = float(tick_val/omega_d) - timing_constant
                pt = pw/2
                
            elif omega_d < 0.0:
                GPIO.output(pin, GPIO.LOW)
                i = -1
                pw = float(tick_val/-omega_d) - timing_constant
                pt = pw/2
        else:
            cmd_recieved = False
    
    return i, pt, cmd_recieved

# Odd motors are linear and even motors are angular [2n = angular and (2n-1) = linear]
def get_pins(motor):

    """
    Function takes a motor # and returns the associated PUL, DIR, and ENC pins.
    To add a new motor follow the convention that the two motors along a rod are in sequence (i.e. rod 1 = motors 1 and 2, rod 2 = motors 3 and 4) 
    and that the linear motors are odd (motors 1 and 3) and the angular motors are even (motors 2 and 4).
    Note: The right encoders are closest to the hardware and the left encoders are closest to the human player.
    """

    if motor == 1:

        PUL_PIN = 27  # Pulse pin for stepper motor
        DIR_PIN = 17  # Direction pin for stepper motor

        SENSOR_R = 6 # [Left (Player), Right (Hardware)]
        SENSOR_L = 5
        SENSOR = None

    elif motor == 2:

        PUL_PIN = 26  # Pulse pin for stepper motor
        DIR_PIN = 19  # Direction pin for stepper motor

        SENSOR = 4
        SENSOR_R = None
        SENSOR_L = None

    elif motor == 3:

        PUL_PIN = 21  # Pulse pin for stepper motor
        DIR_PIN = 20  # Direction pin for stepper motor

        SENSOR_R = 7 # [Left (Player), Right (Hardware)]
        SENSOR_L = 8
        SENSOR = None

    elif motor == 4:

        PUL_PIN = 24  # Pulse pin for stepper motor
        DIR_PIN = 23  # Direction pin for stepper motor

        SENSOR = 2
        SENSOR_R = None
        SENSOR_L = None

    return PUL_PIN, DIR_PIN, SENSOR_L, SENSOR_R, SENSOR

def setup_pins(motor, pul_pin, dir_pin, pin1, pin2, pin3):

    """
    Function sets up pins from arguments using the GPIO library.
    """

    # Set up GPIO for motor pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pul_pin, GPIO.OUT)
    GPIO.setup(dir_pin, GPIO.OUT)

    # Set up GPIO for encoder states values
    if motor == 1 or motor == 3:
        GPIO.setup(pin1, GPIO.IN)
        GPIO.setup(pin2, GPIO.IN)
    elif motor == 2 or motor == 4:
        GPIO.setup(pin3, GPIO.IN)

def if_linear_step(motor, pos, i, pt, right_encoder_pin, left_encoder_pin, cmd_recieved, pul_pin):
    
    # Checking if motor is linear (odd numbers are linear)
    if (motor % 2) == 1:

        linear = True

        # reading encoder values
        right_encoder = GPIO.input(right_encoder_pin) # 525 steps
        left_encoder = GPIO.input(left_encoder_pin) # 0 steps

        if cmd_recieved:
            
            # Checking encoder values before sending step commands to the motors to prevent wall collisions
            if not (right_encoder or left_encoder):
                pos = step(pul_pin, pt, pos, i) # stepping and updating position
                
            elif right_encoder and not left_encoder:
                pos = 525
                if i == -1:
                    pos = step(pul_pin, pt, pos, i) # stepping and updating position
                    
            elif left_encoder and not right_encoder:
                pos = 0
                if i == 1:
                    pos = step(pul_pin, pt, pos, i) # stepping and updating position

    else:
        linear = False; right_encoder = None; left_encoder = None

    return pos, right_encoder, left_encoder, linear

def if_angular_step(motor, pos, i, pt, encoder_pin, encoder_flag, cmd_recieved, pul_pin):

    # Checking if motor is angular (even numbers are angular)
    if (motor % 2) == 0:

        angular = True

        # reading encoder value
        encoder = GPIO.input(encoder_pin) 

        if (encoder == 1) and (encoder_flag == 0):
            encoder_flag = 1
            if OFFSET[0] != 0:
                if pos > 0:
                    pos = OFFSET[1]
                elif pos < 0:
                    pos = OFFSET[0] 
            else:
                pos = 0
        elif (encoder == 0) and (encoder_flag == 1):
            encoder_flag = 0

        if cmd_recieved:
            pos = step(pul_pin, pt, pos, i) # stepping and updating position
        
    else:
        angular = False; encoder = None

    return pos, encoder, encoder_flag, angular

def control(motor, pos, i, pt, right_encoder_pin, left_encoder_pin, encoder_pin, encoder_flag, cmd_recieved, pul_pin):

    pos, \
    right_encoder, left_encoder, \
    linear \
    = if_linear_step(
        motor, 
        pos, i, pt, 
        right_encoder_pin, left_encoder_pin, 
        cmd_recieved, pul_pin
    )
    pos, \
    encoder, encoder_flag, \
    angular \
    = if_angular_step(
        motor, 
        pos, -i, pt, # Direction of angular controller is opposite linear controller for consitency with solver
        encoder_pin, encoder_flag, 
        cmd_recieved, pul_pin
    )

    return pos, \
    left_encoder, encoder, right_encoder, encoder_flag, \
    angular, linear

def calculate(motor, pos, steps_across_field, steps_per_revolution, distance_between_players, player_distance_from_wall, distance_to_clear_zone):

    rad, meters = [0], [0.0, 0.0, 0.0]
    
    if (motor % 2) == 0: # angular position 
        if OFFSET[0] != 0:
            if pos > 400:
                pos = pos % 400
            elif pos < -400:
                pos = pos % -400 
        
        rad = [pos*((2*math.pi)/steps_per_revolution)]
        #print(math.sin(rad[0]))

    elif (motor % 2) == 1: # linear positions
        base_player_position = round(pos * (distance_to_clear_zone / steps_across_field), 7)
        meters = [player_distance_from_wall + base_player_position, 
                  player_distance_from_wall + base_player_position + distance_between_players, 
                  player_distance_from_wall + base_player_position + 2*distance_between_players]

    #print(rad, meters)
    return rad, meters
