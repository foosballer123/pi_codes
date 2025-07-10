#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pigpio.h>
#include <string>
#include <iostream>
#include <chrono>

#define PUL_PIN 24
#define DIR_PIN 23
#define STEPS_PER_REV 400

void step() {
    gpioWrite(PUL_PIN, 1);
    ros::Duration(0.00025).sleep(); // 100ms delay
    gpioWrite(PUL_PIN, 0);
    ros::Duration(0.00025).sleep(); // 100ms delay
}

void continuous_rotation(int rev_per_minute) {
    double delay = 0.00001; // 0.001 seconds delay
    ROS_INFO("Rotating motor at %d RPM with %f seconds delay per step", rev_per_minute, delay);
    for (int i = 0; i < STEPS_PER_REV; ++i) { // One full revolution
        step();
    }
}

void command_callback(const std_msgs::String::ConstPtr& msg) {
    std::string command = msg->data;
    std::transform(command.begin(), command.end(), command.begin(), ::tolower);
    ROS_INFO("Received command: %s", command.c_str());
    
    if (command == "step") {
        step();
        ROS_INFO("Executed one step");
    } else if (command == "rotate") {
        gpioWrite(DIR_PIN, 1); // Ensure direction is set before rotating
        continuous_rotation(100); // Rotate at 100 RPM
        ROS_INFO("Executed continuous rotation");
    } else if (command == "stop") {
        ROS_INFO("Stopping...");
        ros::shutdown();
    }
}

int main(int argc, char** argv) {
//    if (gpioInitialise() < 0) {
//        std::cerr << "Failed to initialize GPIO." << std::endl;
//        return 1;
//    }
    
    gpioSetMode(PUL_PIN, PI_OUTPUT);
    gpioSetMode(DIR_PIN, PI_OUTPUT);
    gpioWrite(DIR_PIN, 0);
    gpioWrite(PUL_PIN, 0); // Ensure PUL_PIN starts LOW
    
    ros::init(argc, argv, "motor_command_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("motor_commands", 10, command_callback);
    
    ROS_INFO("Motor controller node started.");
    ros::spin();
    
    gpioTerminate();
    return 0;
}

