import time
import RPi.GPIO as GPIO

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

    # Set up GPIO mode and configure pins
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(pul_pin, GPIO.OUT)
    GPIO.setup(dir_pin, GPIO.OUT)

    # Set up GPIO for encoder states and read initial values
    if motor == 1 or motor == 3:
        GPIO.setup(pin1, GPIO.IN)
        GPIO.setup(pin2, GPIO.IN)

    elif motor == 2 or motor == 4:
        GPIO.setup(pin3, GPIO.IN)
