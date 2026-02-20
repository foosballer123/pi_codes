from Tests import motor_dict as md
import rospy
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist
import time
import numpy as np
from std_msgs.msg import Float32

class Motor:

    def __init__(self, num=1):

        # motor number
        self.num = num

        # Global variables to store users command and position readings from encoder
        self.user_cmd = None
        self.cmd_received = False
        self.pos = 0
        self.min_signal = 0.3 # minimum signal for actuation
    
        self.pt = 0.00025 # fastest speed (31 rad/s) 

        GPIO.setmode(GPIO.BCM)

        if (self.num == 1) or (self.num == 3):

            # Set up GPIO mode and configure pins
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(md.motor[num]["PUL"], GPIO.OUT)
            GPIO.setup(md.motor[num]["DIR"], GPIO.OUT)

            # GPIO for encoder states
            GPIO.setup(md.motor[num]["SENSOR L"], GPIO.IN)
            GPIO.setup(md.motor[num]["SENSOR R"], GPIO.IN)

        if (self.num == 2) or (self.num == 4):

            # Set up GPIO mode and configure pins
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(md.motor[num]["PUL"], GPIO.OUT)
            GPIO.setup(md.motor[num]["DIR"], GPIO.OUT)

            # GPIO for encoder states
            GPIO.setup(md.motor[num]["SENSOR"], GPIO.IN)


    def step(self, omega_d, steps = 1):
        """
        Function to step a desired number of pulses with a given omega.
        """

        # bounding the input while retaining its sign (max omega_d of 31 rad/s)
        omega_d = (omega_d + 0.01)/(abs(omega_d) + 0.01) * min(31, abs(omega_d))

        tick_val = 0.015708
        pw = float((tick_val+0.01)/(abs(omega_d)+0.01))
        pt = pw/2

        if (self.num == 1) or (self.num == 3):

            r = GPIO.input(md.motor[self.num]["SENSOR R"])
            l = GPIO.input(md.motor[self.num]["SENSOR L"])
           
        
        if (self.num == 2) or (self.num == 4):

            state = GPIO.input(md.motor[self.num]["SENSOR"])
            
    
        # make sure that motors 1 and 3 do not ram through the walls of the table
        if (self.num == 1) or (self.num == 3):
            
            print("In bounds 2")
            if abs(omega_d) > self.min_signal:
                    
                if not (r or l):
                    print(omega_d)
                    if omega_d > 0:
                        dir = -1
                        s = True
                    elif omega_d < 0:
                        dir = 1
                        s = True
                    else:
                        dir = 0
                        s = False

                elif r and not l:
                    self.pos = 0
                    if omega_d < 0:
                        dir = 1
                        s = True
                    else:
                        dir = 0
                        s = False
                elif l and not r:
                    self.pos = 525 # approximate max y range of table
                    if omega_d > 0:
                        dir = -1
                        s = True
                    else:
                        dir = 0
                        s = False
                else:
                    dir = 0
                    s = False
            else:
                dir = 0
                s = False
        
        # motors 2 and 4 can step whenever they please given that they have no objects they can collide with
        if (self.num == 2) or (self.num == 4):

            # remember to reset the position here when the encoder is triggered
            if omega_d > 0:
                dir = -1
            elif omega_d < 0:
                dir = 1
            else:
                dir = 0
            s = True
        
        if steps > 1:
            for _ in range(steps):

                if (dir != 0) and s:

                    # set motor direction
                    if dir == -1: # dir of -1 is towards the actuation table
                        GPIO.output(md.motor[self.num]["DIR"], GPIO.HIGH)
                    elif dir == 1: # dir of 1 is away from the actuation table
                        GPIO.output(md.motor[self.num]["DIR"], GPIO.LOW)
                    
                    # actuation with pulse time calculated in step loop
                    GPIO.output(md.motor[self.num]["PUL"], GPIO.HIGH)
                    time.sleep(pt)  # Smallest delay possible for step pulse
                    GPIO.output(md.motor[self.num]["PUL"], GPIO.LOW)
                    time.sleep(pt)  # Smallest delay possible for step pulse
                            
                    # tracking changes in the position of the rod
                    self.pos += dir 

        elif steps == 1:
            # set motor direction
            if dir == -1: # dir of -1 is towards the actuation table
                GPIO.output(md.motor[self.num]["DIR"], GPIO.HIGH)
            elif dir == 1: # dir of 1 is away from the actuation table
                GPIO.output(md.motor[self.num]["DIR"], GPIO.LOW)

            # actuation with pulse time calculated in step loop
            GPIO.output(md.motor[self.num]["PUL"], GPIO.HIGH)
            time.sleep(pt)  # Smallest delay possible for step pulse
            GPIO.output(md.motor[self.num]["PUL"], GPIO.LOW)
            time.sleep(pt)  # Smallest delay possible for step pulse

            self.pos += dir
            #self.actuate(s, pt, dir) 
        else:
            time.sleep(0.1)
    

    def pos_callback(self, data):
        """
        Callback function for receiving ball position data.
        Updates the global variable ball_pos.
        """
        self.user_cmd = data
        self.cmd_received = True

    def start_node(self, topic = '/omega_d'):

        rospy.init_node('motor' + str(self.num), anonymous=True)
        self.cmd_sub = rospy.Subscriber(topic, Twist, self.pos_callback, queue_size=10)
        pub = rospy.Publisher("/looprate_test", Float32, queue_size=10)
        # return to initial position to initialze pos reading
        print("Initializing...")
        self.step(15, 525)
        self.step(5, 5)  # extra step to make sure sensor is triggered

        time.sleep(2) 

        rate = rospy.Rate(2000)
        #t_s = time.time()
        #i = 0
        while not rospy.is_shutdown():
            #i += 1
            pub.publish(time.time())
            if self.cmd_received == True:
                
                #print(self.user_cmd.linear.x)
                self.step(self.user_cmd.linear.x)
            else:
                print("Waiting for command...")

            #print("Loop rate:", i/(time.time()-t_s), "Hz")
            rate.sleep()

    def close(self):
        GPIO.cleanup()

if __name__ == '__main__':

    motor = Motor(2)

    #motor.init_node()
    motor.start_node()

motor.close()

    

### Technically this class is working properly but it doesn't seem as quick as using functions
### Next time:
### Double check that you are using rosrate correctly within the class
### Test the 'real' loop rate
### Theory: Because I split a bunch of the calculation across different methods in the class the program slowed down
### Solution: Try to congregate as many of the calculations as possible under a single method in the class
###           Split the class into 'variables' and 'processes'
###
### Note: It might not even be worth it to create a motor class! These objects are meant to exist in real time! Which a class might not be well suited for.
###       Why not break the class apart into functions? It seemed to work just fine for the other codes! 
###       If statements and global variables should be all you need....
###       You can just import the motor library at the beginning of other codes
###
###
### Bug Found: The subscriber is lagging and does not recieve commands fast enough
### Attempted Fix: Combined init_node with start_node but this did nothing.
    