#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pigpio.h>
#include <iostream>
#include <vector>
#include <std_msgs/String.h>

#define PUL_PIN 3
#define DIR_PIN 2

int sensor; 
double ball_pos_x = -1;
double sensor_z; 

void step() {
    gpioWrite(PUL_PIN, 1);
    ros::Duration(1.0).sleep();
    gpioWrite(PUL_PIN, 0);
    ros::Duration(1.0).sleep();
}

void pos_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    ball_pos_x = msg->linear.x;
}

void sensor_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    sensor_z = msg->linear.z;
}

int main(int argc, char** argv) {
	

    gpioSetMode(PUL_PIN, PI_OUTPUT);
    gpioSetMode(DIR_PIN, PI_OUTPUT);
    gpioWrite(DIR_PIN, 0);
    gpioWrite(PUL_PIN, 0);
    
    ros::init(argc, argv, "motor2_pi");
    ros::NodeHandle nh;
    ros::Subscriber ballpos_sub = nh.subscribe("/ball_pos", 10, pos_callback);
    ros::Subscriber sensor_sub = nh.subscribe("/sensor_data", 10, sensor_callback);

    while (sensor_z != 1) {
        step();
    }

    gpioWrite(DIR_PIN, 1);
    for (int i = 0; i < 100; ++i) {
        step();
    }
    
    std::vector<int> defense_zone = {80, 130};
    while (ros::ok()) {
        if (ball_pos_x >= defense_zone[0] && ball_pos_x <= defense_zone[1]) {
            gpioWrite(DIR_PIN, 0);
            for (int i = 0; i < 150; ++i) {
                step();
            }
            
            gpioWrite(DIR_PIN, 1);
            sensor_z = 0;
            while (sensor_z != 1) {
                step();
                ros::spinOnce();
            }
            
            for (int i = 0; i < 100; ++i) {
                step();
            }
        }
        ros::spinOnce();
    }
    
    gpioTerminate();
    return 0;
}

