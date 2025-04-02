#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import enc_states 
import time

ball_pos = None

# GPIO pins for PUL, DIR, and SENSOR
PUL_PIN = 21
DIR_PIN = 20

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

sensor = 0
temp_sensor4 = 0
temp_sensor5 = 0
THRESHOLD = 1

def pos_callback(data):
	global ball_pos
	ball_pos = data

def step():
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(0.005)  # smallest delay possible
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.005)  # smallest delay possible

if __name__ == '__main__':
	counter = 0
	GPIO.output(DIR_PIN, GPIO.LOW) 
	rospy.init_node('motor3', anonymous = True)
	ballpos_sub = rospy.Subscriber("/ball_pos", Twist, pos_callback, queue_size = 10)
	
	while (sensor != 1):
		step()
		sensor = enc_states.enc_status_5()
		
	GPIO.output(DIR_PIN, GPIO.HIGH) 
	#temp = int(ball_pos.linear.z)
	while not rospy.is_shutdown():

		if ball_pos is not None:
			# vaiable to encroach on ball y pos
			ball_to_player = ball_pos.linear.z - counter
			# check direction, positive, towards brain
			if (abs(ball_to_player) > THRESHOLD):
				if (ball_to_player > 0): 
					#set direction
					GPIO.output(DIR_PIN, GPIO.HIGH)
					counter += 1
					
				# check direction, negative, away from brain
				elif(ball_to_player < 0):
					GPIO.output(DIR_PIN, GPIO.LOW) 
					counter -= 1

				step()
				THRESHOLD = 1
				
			else:
				THRESHOLD = 5

			if (enc_states.enc_status_5() == 1 and temp_sensor5 == 1):
				counter = 0
			
			elif (enc_states.enc_status_4() == 1 and temp_sensor4 == 1):
				counter = 550
				
			temp_sensor5 = enc_states.enc_status_5()
			temp_sensor4 = enc_states.enc_status_4()	
GPIO.cleanup()
