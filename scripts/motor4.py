#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import enc_states 
import time
import numpy as np

ball_pos = None

# GPIO pins for PUL, DIR, and SENSOR
PUL_PIN = 24
DIR_PIN = 23

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

sensor = 0

def pos_callback(data):
	global ball_pos
	ball_pos = data

def step():
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(0.00025)  # smallest delay possible
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.00025)  # smallest delay possible

if __name__ == '__main__':
	GPIO.output(DIR_PIN, GPIO.LOW) 
	rospy.init_node('motor4', anonymous = True)
	ballpos_sub = rospy.Subscriber("/ball_pos", Twist, pos_callback, queue_size = 10)

	while (sensor != 1):
		step()
		sensor = enc_states.enc_status_6()
		
	GPIO.output(DIR_PIN, GPIO.HIGH) 
	
	for i in range(100):
		step()
	
	#temp = int(ball_pos.linear.z)
	while not rospy.is_shutdown():

		offense = np.arange(486, 520)
		

		
		if ball_pos is not None:
		
			# read x position from echo
			position = ball_pos.linear.x
			sensor = 0
			
			if ( (int(position) in offense) or ( (380 < position < 486) and (ball_pos.angular.x == 0) ) ):
			
				GPIO.output(DIR_PIN, GPIO.LOW) 
				
				for i in range(150):
					step()
					
				GPIO.output(DIR_PIN, GPIO.HIGH) 
				
				while (sensor != 1):
					step()
					sensor = enc_states.enc_status_6()
					
				for i in range(100):
					step()	
			


GPIO.cleanup()
