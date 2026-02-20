#!/usr/bin/env python3

import rospy
import RPi.GPIO as GPIO
import time
import motor_dict as md
import math

# Set up GPIO mode and configure pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(md.motor[4]["SENSOR"], GPIO.IN)

if __name__ == '__main__':
    
    i = 0
    j = 0 # number of complete rotations (2pi radians)

    rospy.init_node('omega_monitor', anonymous=True)
    rate = rospy.Rate(2000) # 2000 hz loop rate 

    # sleep to wait for omega_d to update (TO-DO: replace this with a 'command recieved' flag)
    time.sleep(1)

    state_flag = 0
    move_flag = 0
    t_s = time.time()
    t_k = 0
    while not rospy.is_shutdown():
        
        state = GPIO.input(md.motor[4]["SENSOR"])

        if (state == 1) and (move_flag == 0):
            move_flag = 1
            t_k = time.time()
            
        if (state == 1) and (state_flag == 0):
            state_flag = 1
            j += 1
        elif (state == 0) and (state_flag == 1):
            state_flag = 0

        if t_k != 0:
            delta_t = (time.time()-t_k)
            print("Triggers", j)
            print("Real omega=", (j*2*math.pi)/delta_t)

        i += 1
        rate.sleep() # if our loop is too fast we risk adding error to our calculation 
            

# Clean up GPIO settings and displaying loop rate
delta_t = (time.time()-t_s)
print("Iterations", i)
print("Real HZ =", i/delta_t)
print("Triggers", j)
print("Real omega=", (j*2*math.pi)/delta_t)
GPIO.cleanup()

