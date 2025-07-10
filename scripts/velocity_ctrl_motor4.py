import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import enc_states 
import time
import numpy as np

# Global variable to store ball position
omega_d = None

pt = 0.00025 # fastest speed (31 rad/s)
up = True # ignore this for now
# GPIO pins for motor control
PUL_PIN = 24  # Pulse pin for stepper motor
DIR_PIN = 23  # Direction pin for stepper motor

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

# Sensor variable
sensor = 0

def pos_callback(data):
    """
    Callback function for receiving ball position data.
    Updates the global variable ball_pos.
    """
    global omega_d
    global pt
    omega_d = data
    tick_val = 0.015708

    if omega_d.linear.x > 0.3:
        pw = float(tick_val/omega_d.linear.x)
        pt = pw/2
        #pt = 1.0
        

    print('Got this pt value:',pt)   

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

if __name__ == '__main__':
    GPIO.output(DIR_PIN, GPIO.LOW)  # Set initial motor direction
    rospy.init_node('motor4', anonymous=True)
    
    # Subscribe to the ball position topic
    ballpos_sub = rospy.Subscriber("/omega_d", Twist, pos_callback, queue_size=10)

    # Move motor until sensor is triggered
    while sensor != 1:
        step()
        sensor = enc_states.enc_status_6()
    
    GPIO.output(DIR_PIN, GPIO.HIGH)  # Change motor direction
    
    # Perform initial stepping motion
    for i in range(100):
        step()
    while not rospy.is_shutdown():
        if omega_d.linear.x > 0.3:
            step()
        #time.sleep(0.005)

    """
    offense = np.arange(460, 520)  # Define offense zone range
    
    while not rospy.is_shutdown():
        if omega_d is not None:
            # Read x position from ball position data
            position = ball_pos.linear.x
            sensor = 0  # Reset sensor state
            if (position < 440 and not up):
                for i in range(50):
                    step()
                    up = True
            
            # Check if ball is in offense zone or near the boundary with specific conditions
            if ((int(position) in offense and (ball_pos.angular.x == 0))):
                GPIO.output(DIR_PIN, GPIO.LOW)  # Set direction for defensive movement
                if up:
                    for i in range(130):
                        step()
                        up = False
                else:
                    for i in range(90):
                        step()
                        up = False
                GPIO.output(DIR_PIN, GPIO.HIGH)  # Set direction for defensive movement
                # Move back until sensor is triggered
                while sensor != 1:
                    step()
                    sensor = enc_states.enc_status_6()
                
                # Rotate 90 deg slightly to reset position
                GPIO.output(DIR_PIN, GPIO.HIGH)  # Reverse direction
                for i in range(40):
                    step()
                    up = False
    """                
                
                        
# Clean up GPIO settings
GPIO.cleanup()