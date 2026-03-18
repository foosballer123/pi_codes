import RPi.GPIO as GPIO
import time

# Sensor GPIO pin definitions
SENSOR_1 = 6  
SENSOR_2 = 5
SENSOR_3 = 4  
SENSOR_4 = 7
SENSOR_5 = 8
SENSOR_6 = 2

# Dictionary to track interrupt flags
sensor_flags = {
    1: False,
    2: False,
    3: False,
    4: False,
    5: False,
    6: False
}

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_1, GPIO.IN)  
GPIO.setup(SENSOR_2, GPIO.IN)
GPIO.setup(SENSOR_3, GPIO.IN)
GPIO.setup(SENSOR_4, GPIO.IN)
GPIO.setup(SENSOR_5, GPIO.IN)
GPIO.setup(SENSOR_6, GPIO.IN)

def make_callback(sensor_id):
    def callback(channel):
        global sensor_flags
        sensor_flags[sensor_id] = True
        print(f"Interrupt: Sensor {sensor_id} triggered!")
    return callback

def init_interrupts():
    """Initialize GPIO interrupts for each sensor."""
    GPIO.add_event_detect(SENSOR_1, GPIO.RISING, callback=make_callback(1), bouncetime=200)
    GPIO.add_event_detect(SENSOR_2, GPIO.RISING, callback=make_callback(2), bouncetime=200)
    GPIO.add_event_detect(SENSOR_3, GPIO.RISING, callback=make_callback(3), bouncetime=200)
    GPIO.add_event_detect(SENSOR_4, GPIO.RISING, callback=make_callback(4), bouncetime=200)
    GPIO.add_event_detect(SENSOR_5, GPIO.RISING, callback=make_callback(5), bouncetime=200)
    GPIO.add_event_detect(SENSOR_6, GPIO.RISING, callback=make_callback(6), bouncetime=200)

def clear_flags():
    """Reset all sensor flags to False."""
    global sensor_flags
    for key in sensor_flags:
        sensor_flags[key] = False

def remove_interrupts():
    """Remove all GPIO event detections."""
    GPIO.remove_event_detect(SENSOR_1)
    GPIO.remove_event_detect(SENSOR_2)
    GPIO.remove_event_detect(SENSOR_3)
    GPIO.remove_event_detect(SENSOR_4)
    GPIO.remove_event_detect(SENSOR_5)
    GPIO.remove_event_detect(SENSOR_6)

# Optional: test interrupt system in standalone mode
if __name__ == "__main__":
    print("Initializing interrupts...")
    init_interrupts()
    try:
        while True:
            for i in range(1, 7):
                if sensor_flags[i]:
                    print(f"Sensor {i} triggered!")
                    sensor_flags[i] = False
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        remove_interrupts()
        GPIO.cleanup()

