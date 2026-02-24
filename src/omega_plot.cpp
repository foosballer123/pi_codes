// Combined Stepper + Encoder Monitor (ROS-native plotting)
// Uses wiringPi (BCM numbering)
// Author: Benjamin Simpson, ChatGPT (2025-11)

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <wiringPi.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include <signal.h>

// GPIO pin definitions (BCM numbering)
#define PUL_PIN 26
#define DIR_PIN 19
#define ENC_PIN 4  // encoder input pin

bool stop_node = false;

// ------------------------------------------------------
// Microsecond sleep
// ------------------------------------------------------
inline void sleepMicros(int micros)
{
    std::this_thread::sleep_for(std::chrono::microseconds(micros));
}

// ------------------------------------------------------
// Signal handler for clean shutdown
// ------------------------------------------------------
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
    ros::init(argc, argv, "motor4_step_monitor");
    ros::NodeHandle nh;
    signal(SIGINT, sigintHandler);

    // Publishers for real and desired omega
    ros::Publisher omega_pub = nh.advertise<std_msgs::Float64>("omega_real", 10);
    ros::Publisher omega_d_pub = nh.advertise<std_msgs::Float64>("omega_desired", 10);

    if (wiringPiSetupGpio() == -1)
    {
        ROS_ERROR("Failed to initialize wiringPi!");
        return 1;
    }

    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENC_PIN, INPUT);

    digitalWrite(DIR_PIN, HIGH);  // forward direction

    ROS_INFO("Stepper + Encoder node started.");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Timing setup
    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_trigger_time = start_time;

    int encoder_state = 0;
    int prev_state = 0;
    int trigger_count = 0;
    int iterations = 0;

    double omega_d;
    std::cout << "Enter a desired omega_d (rad/s): ";
    std::cin >> omega_d;
    std::cout << "You entered: " << omega_d << std::endl;

    int ramp_up;
    std::cout << "Ramp up (1 or 0): ";
    std::cin >> ramp_up;
    std::cout << "You entered: " << ramp_up << std::endl;

    int time_increase = 0;
    double speed_increase = 0.0;

    if (ramp_up == 1) {
        std::cout << "Enter a desired time to increase omega: ";
        std::cin >> time_increase;
        std::cout << "You entered: " << time_increase << std::endl;

        std::cout << "Enter a desired speed to increase omega: ";
        std::cin >> speed_increase;
        std::cout << "You entered: " << speed_increase << std::endl;
    }

    int time_constant = 154 / 2; // intrinsic processing delay in microseconds
    int pulse_us = int( ((0.015708 / std::abs(omega_d)) * 1000000) / 2); // half period in microseconds
    
    while (ros::ok() && !stop_node)
    {
        // Generate one step pulse
        digitalWrite(PUL_PIN, HIGH);
        sleepMicros(pulse_us - time_constant);
        digitalWrite(PUL_PIN, LOW);
        sleepMicros(pulse_us - time_constant);

        // Read encoder
        encoder_state = digitalRead(ENC_PIN);
        auto now = std::chrono::high_resolution_clock::now();

        if (encoder_state == HIGH && prev_state == LOW)
        {
            trigger_count++;
            double delta_t = std::chrono::duration<double>(now - last_trigger_time).count();

            if (delta_t > 0.0)
            {
                double omega = (2 * M_PI) / delta_t;  // rad/s per full rotation (assuming 1 trigger = 1 rotation)

                // Publish real omega
                std_msgs::Float64 omega_msg;
                omega_msg.data = omega;
                omega_pub.publish(omega_msg);

                // Publish desired omega
                std_msgs::Float64 omega_d_msg;
                omega_d_msg.data = omega_d;
                omega_d_pub.publish(omega_d_msg);

                // std::cout << "Desired omega: " << omega_d << std::endl;
                // std::cout << "Real omega: " << omega << std::endl;

                ROS_INFO_STREAM("Triggers: " << trigger_count << " | Omega = " << omega << " rad/s | Omega_d = " << omega_d);
                
                // Change the desired omega if ramp_up flag is set to one
                if (ramp_up == 1) {
                    if ((trigger_count % time_increase) == 0)
                    {
                        // Optionally increase desired omega every 5 triggers
                        omega_d = omega_d + speed_increase;
                        pulse_us = int( ((0.015708 / std::abs(omega_d)) * 1000000 ) / 2);
                        ROS_INFO_STREAM("Increased omega to: " << omega_d);
                    }
                }
            }

            last_trigger_time = now;
        }

        prev_state = encoder_state;
        iterations++;
        // ROS_INFO_STREAM("Triggers % 5: " << trigger_count % 5);
        // if ((trigger_count % 5) == 0)
        // {
        //     // Optionally increase desired omega every 5 triggers
        //     omega_d++;
        //     pulse_us = int( ((0.015708 / std::abs(omega_d)) * 1000000 ) / 2);
        // }

        ros::spinOnce();
    }

    // Cleanup
    digitalWrite(PUL_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    auto total_time = std::chrono::high_resolution_clock::now() - start_time;
    double elapsed = std::chrono::duration<double>(total_time).count();
    ROS_INFO("Iterations: %d", iterations);
    ROS_INFO("Total triggers: %d", trigger_count);
    ROS_INFO("Elapsed time: %.3f s", elapsed);
    ROS_INFO("Average omega = %.3f rad/s", (trigger_count * 2 * M_PI) / elapsed);

    ros::shutdown();
    return 0;
}
