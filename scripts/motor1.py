#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import enc_states 
import time

ball_pos = None

# GPIO pins for PUL, DIR, and SENSOR
PUL_PIN = 27
DIR_PIN = 17

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
    time.sleep(0.0005)  # smallest delay possible
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.0005)  # smallest delay possible

if __name__ == '__main__':
	counter = 0
	GPIO.output(DIR_PIN, GPIO.LOW) 
	rospy.init_node('motor1', anonymous = True)
	ballpos_sub = rospy.Subscriber("/ball_pos", Twist, pos_callback, queue_size = 10)
	
	while (sensor != 1):
		step()
		sensor = enc_states.enc_status_2()
		
	GPIO.output(DIR_PIN, GPIO.HIGH) 
	#temp = int(ball_pos.linear.z)
	while not rospy.is_shutdown():
		
		if ball_pos is not None:
			# vaiable to encroach on ball y pos
			ball_to_player = ball_pos.linear.z - counter
			# check direction, positive, towards brain
			if (abs(ball_to_player) > 2.5):
				if (ball_to_player > 0): 
					#set direction
					GPIO.output(DIR_PIN, GPIO.HIGH)
					counter += 1
					step()
					
				# check direction, negative, away from brain
				elif(ball_to_player < 0):
					GPIO.output(DIR_PIN, GPIO.LOW) 
					counter -= 1
					step()
				
			
			if enc_states.enc_status_2() == 1:
				counter = 0
			
			elif enc_states.enc_status_1() == 1:
				counter = 550
				
GPIO.cleanup()
