#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time

"""
physical pin
works: 3,5,7,11,13,26,29,31 
no work: 15,19,21,23,32,33,35,36,37,38 39, 40

"""

SENSOR_1 = 5
SENSOR_2 = 6
SENSOR_3 = 26
SENSOR_4 = 7
SENSOR_5 = 12
SENSOR_6 = 4

GPIO.setmode(GPIO.BCM)

GPIO.setup(SENSOR_1, GPIO.IN)
GPIO.setup(SENSOR_2, GPIO.IN)
GPIO.setup(SENSOR_3, GPIO.IN)
GPIO.setup(SENSOR_4, GPIO.IN)
GPIO.setup(SENSOR_5, GPIO.IN)
GPIO.setup(SENSOR_6, GPIO.IN)


def enc_status_1():
    try:
        while True:
            # Read the sensor value (HIGH or LOW)
            sensor1 = GPIO.input(SENSOR_1)

            if sensor1 == GPIO.HIGH:
                print("Object detected! [1]")
                # Adjust to desired revolutions per minute

            else:
                print("No object detected. [1]")
                # Adjust to desired revolutions per minute
                

            # Add a delay to avoid excessive readings
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting due to keyboard interrupt.")
        return "Keyboard interrupt"
        
        
def enc_status_2():
    try:
        while True:
            # Read the sensor value (HIGH or LOW)
            sensor2 = GPIO.input(SENSOR_2)

            if sensor2 == GPIO.HIGH:
                print("Object detected! [2]")
                # Adjust to desired revolutions per minute

            else:
                print("No object detected. [2]")
                # Adjust to desired revolutions per minute

            
            time.sleep(0.05)
    except:
        print("\nEncoder not read properly.")
        
def enc_status_3(): ###########RETEST, NOT WORKING ########
    try:
        while True:
            # Read the sensor value (HIGH or LOW)
            sensor3 = GPIO.input(SENSOR_3)

            if sensor3 == GPIO.HIGH:
                print("Object detected![3]")
                # Adjust to desired revolutions per minute

            else:
                print("No object detected.[3]")
                # Adjust to desired revolutions per minute

            
            time.sleep(0.05)
    except:
        print("\nEncoder not read properly.")
        
        
def enc_status_4():
    try:
        while True:
            # Read the sensor value (HIGH or LOW)
            sensor4 = GPIO.input(SENSOR_4)

            if sensor4 == GPIO.HIGH:
                print("Object detected![4]")
                # Adjust to desired revolutions per minute

            else:
                print("No object detected.[4]")
                # Adjust to desired revolutions per minute

            
            time.sleep(0.05)
    except:
        print("\nEncoder not read properly.")
        
def enc_status_5(): ###########RETEST, NOT WORKING ########
    try:
        while True:
            # Read the sensor value (HIGH or LOW)
            sensor5 = GPIO.input(SENSOR_5)

            if sensor5 == GPIO.HIGH:
                print("Object detected![5]")
                # Adjust to desired revolutions per minute

            else:
                print("No object detected.[5]")
                # Adjust to desired revolutions per minute

            
            time.sleep(0.05)
    except:
        print("\nEncoder not read properly.")
        
def enc_status_6(): ###########RETEST, NOT WORKING ########
    try:
        while True:
            # Read the sensor value (HIGH or LOW)
            sensor6 = GPIO.input(SENSOR_6)

            if sensor6 == GPIO.HIGH:
                print("Object detected![6]")
                # Adjust to desired revolutions per minute

            else:
                print("No object detected.[6]")
                # Adjust to desired revolutions per minute

            
            time.sleep(0.05)            
    except:
        print("\nEncoder not read properly.")
        


# Example usage
if __name__ == "__main__":
    enc_status_5()
