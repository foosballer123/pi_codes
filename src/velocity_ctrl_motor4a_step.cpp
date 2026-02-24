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
#define PUL_PIN 24  // Pulse pin (BCM numbering)
#define DIR_PIN 23  // Direction pin (BCM numbering)

geometry_msgs::Twist step_msg;
bool step_received = false;
bool stop_node = false;

// ------------------------------------------------------
// Callback function for receiving step flag
// ------------------------------------------------------
void stepCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    step_msg = *msg;
    step_received = true;
}

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
    ros::init(argc, argv, "motor4_step");
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

    //ros::Subscriber step_sub = nh.subscribe("/step", 10, stepCallback);

    //ros::Rate loop_rate(2000); // 2500 Hz

    ROS_INFO("motor4_step node started. Waiting for commands...");
    //ros::Duration(1.0).sleep(); // Wait for initial data

    int iterations = 0;
    auto start_time = ros::Time::now();

    while (ros::ok() && !stop_node) {
        ros::spinOnce();

        //if (step_received) {
            // Set motor direction based on command
            // if (step_msg.linear.x == 1.0) {
            //     digitalWrite(DIR_PIN, HIGH);

            //     // Generate one pulse
            //     digitalWrite(PUL_PIN, HIGH);
            //     sleepMicros(step_msg.linear.y * 1000000);  // seconds to microseconds high
            //     digitalWrite(PUL_PIN, LOW);
            //     sleepMicros(step_msg.linear.y * 1000000);  // seconds to microseconds low
            // }
            // else if (step_msg.linear.x == -1.0) {
            //     digitalWrite(DIR_PIN, LOW);

            //     // Generate one pulse
            //     digitalWrite(PUL_PIN, HIGH);
            //     sleepMicros(step_msg.linear.y * 1000000);  // seconds to microseconds high
            //     digitalWrite(PUL_PIN, LOW);
            //     sleepMicros(step_msg.linear.y * 1000000);  // seconds to microseconds low
        
            // }
                    //     digitalWrite(DIR_PIN, HIGH);
                    
            //digitalWrite(DIR_PIN, HIGH);
            //digitalWrite(DIR_PIN, HIGH * (int)(step_msg.linear.x == 1) + LOW * (int)(step_msg.linear.x == -1));
            // Generate one pulse
            // digitalWrite(PUL_PIN, HIGH);
            // sleepMicros(step_msg.linear.y * 1000000);  // seconds to microseconds high
            // digitalWrite(PUL_PIN, LOW);
            // sleepMicros(step_msg.linear.y * 1000000);  // seconds to microseconds low
            // digitalWrite(DIR_PIN, HIGH * (int)(step_msg.linear.x == 1) + LOW * (int)(step_msg.linear.x == -1));
            // Generate one pulse
        digitalWrite(PUL_PIN, HIGH);
        sleepMicros(0.00025 * 1000000);  // seconds to microseconds high
        digitalWrite(PUL_PIN, LOW);
        sleepMicros(0.00025 * 1000000);  // seconds to microseconds low

        //}

        //iterations++;
        //loop_rate.sleep();
    }

    // Calculate real loop frequency
    double elapsed = (ros::Time::now() - start_time).toSec();
    ROS_INFO("Iterations: %d", iterations);
    ROS_INFO("Real HZ: %.2f", iterations / elapsed);

    // Clean up GPIO
    digitalWrite(PUL_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    ros::shutdown();
    return 0;
}

// Dr. Saeidi is correct in that if the main loop rate is higher than the velocity commands we send 
// Then we will trip over ourselves and send to many steps in quick succession
// This is why we have to remove the loop rate entirely

// Take the time to sit down with a piece of paper and trace out the timing hierarchy involves in sending step commands and actuation
// Make sure that you are sending approximately smooth and continuous motion profiles to the motor and not a bunch of jerky steps

// It might help to take a single velocity command and not actuate it in a single step but across many
// Break down the minimum motion profile 5x times so that you can gaurentee smooth motion