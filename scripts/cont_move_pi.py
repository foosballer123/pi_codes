#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time

# GPIO setup
PUL_PIN = 18
DIR_PIN = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PUL_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)

GPIO.output(DIR_PIN, GPIO.LOW)

def step():
    GPIO.output(PUL_PIN, GPIO.HIGH)
    time.sleep(0.0005)  # smallest delay possible
    GPIO.output(PUL_PIN, GPIO.LOW)
    time.sleep(0.0005)  # smallest delay possible

def continuous_rotation(rev_per_minute):
    delay = 60.0 / (400 * rev_per_minute)  # 400 steps per revolution
    for _ in range(400):  # One full revolution
        step()

def command_callback(msg):
    command = msg.data.lower()
    rospy.loginfo(f"Received command: {command}")
    
    if command == "step":
        step()
        rospy.loginfo("Executed one step")
    elif command == "rotate":
        continuous_rotation(100)  # Rotate at 100 RPM
        rospy.loginfo("Executed continuous rotation")
    elif command == "stop":
        rospy.loginfo("Stopping...")
        rospy.signal_shutdown("Stop command received")

def motor_control_listener():
    rospy.init_node('motor_command_subscriber', anonymous=True)
    rospy.Subscriber('motor_commands', String, command_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        motor_control_listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()

