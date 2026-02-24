// Velocity Control Code for Motor 4 - 10/23/2025
// Converted from Python by ChatGPT
// Written by Dr. Ahmed Saeidi, Joe Scott, and Benjamin Simpson
//
// Notes:
// - ROS 1 C++ node equivalent of the Python motor4_step node
// - Uses wiringPi for GPIO (you can use pigpio or sysfs as an alternative)
// - Timing handled with ros::Rate and high-resolution nanosleep
// - Loop rate: 2000 Hz

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <wiringPi.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <signal.h>

// GPIO pins for motor control
#define PUL_PIN 26  // Pulse pin (BCM numbering)
#define DIR_PIN 19  // Direction pin (BCM numbering)

bool stop_node = false;


// ------------------------------------------------------
// Helper function for microsecond-level sleep
// ------------------------------------------------------
void sleepMicros(int microseconds)
{
    std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
}

void sigintHandler(int sig)
{
    ROS_WARN("Caught Ctrl+C — shutting down cleanly...");
    stop_node = true;
}

// ------------------------------------------------------
// Main
// ------------------------------------------------------
int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor2_step");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);  // register custom handler

    // Initialize GPIO using wiringPi
    if (wiringPiSetupGpio() == -1) {
        ROS_ERROR("Failed to initialize wiringPi!");
        return 1;
    }

    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, HIGH); // Initial motor direction

    while (ros::ok() && !stop_node) {
        ros::spinOnce();

        digitalWrite(PUL_PIN, HIGH);
        sleepMicros(0.00025 * 1000000);  // seconds to microseconds high
        digitalWrite(PUL_PIN, LOW);
        sleepMicros(0.00025 * 1000000);  // seconds to microseconds low

    }

    // Clean up GPIO
    digitalWrite(PUL_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    ros::shutdown();
    return 0;
}


