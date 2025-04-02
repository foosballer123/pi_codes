#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist
import time

# Define GPIO pins for sensors
SENSOR_1 = 6  
SENSOR_2 = 5
SENSOR_3 = 4  
SENSOR_4 = 7
SENSOR_5 = 8
SENSOR_6 = 2

# Initialize ROS node
rospy.init_node('sensor_publisher', anonymous=True)
pub = rospy.Publisher('sensor_data', Twist, queue_size=10)
rate = rospy.Rate(10)  # Publish at 10 Hz

def read_sensors():
    # Create a Twist message to hold sensor data
    sensor_data = Twist()
    
    # Read the sensor values
    sensor_data.linear.x = GPIO.input(SENSOR_1)
    sensor_data.linear.y = GPIO.input(SENSOR_2)
    sensor_data.linear.z = GPIO.input(SENSOR_3)
    sensor_data.angular.x = GPIO.input(SENSOR_4)
    sensor_data.angular.y = GPIO.input(SENSOR_5)
    sensor_data.angular.z = GPIO.input(SENSOR_6)
    
    return sensor_data

def main():
    # Set up GPIO mode
    GPIO.setmode(GPIO.BCM)
    
    # Set up GPIO pins as inputs
    GPIO.setup(SENSOR_1, GPIO.IN)
    GPIO.setup(SENSOR_2, GPIO.IN)
    GPIO.setup(SENSOR_3, GPIO.IN)
    GPIO.setup(SENSOR_4, GPIO.IN)
    GPIO.setup(SENSOR_5, GPIO.IN)
    GPIO.setup(SENSOR_6, GPIO.IN)
    
    
    rospy.loginfo("Starting sensor data stream...")

    try:
        while not rospy.is_shutdown():
            # Read sensor data
            sensor_data = read_sensors()
            
            # Publish sensor data
            pub.publish(sensor_data)
            
            # Sleep to maintain loop rate
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass
    
    finally:
        # Clean up GPIO before exiting
        GPIO.cleanup()

if __name__ == '__main__':
    main()

