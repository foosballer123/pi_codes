// Combined Stepper + Encoder Monitor
// Uses wiringPi (BCM numbering)
// Author: Benjamin Simpson, ChatGPT (2025-11)

#include <ros/ros.h>
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

    // int pulse_high_us = 250;  // 0.00025 seconds high
    // int pulse_low_us = 250;   // 0.00025 seconds low
    // int pulse_us = 300;
    int encoder_state = 0;
    int prev_state = 0;
    int trigger_count = 0;
    int iterations = 0;

    int k = 10;
    int omega_d;
    std::cout << "Enter a omega_d: ";
    std::cin >> omega_d; // user types input and presses Enter
    std::cout << "You entered: " << omega_d << std::endl;

    int pulse_w = int( (0.015708 / std::abs(omega_d)) * 1000000 );
    std::cout << "Theoretical pulse time: " << int(pulse_w/2) << std::endl;
    int pulse_us = int(pulse_w/2);

    while (ros::ok() && !stop_node)
    {
        // Generate one step pulse
        digitalWrite(PUL_PIN, HIGH);
        sleepMicros(pulse_us);
        digitalWrite(PUL_PIN, LOW);
        sleepMicros(pulse_us);

        // Read encoder
        encoder_state = digitalRead(ENC_PIN);
        if (encoder_state == HIGH && prev_state == LOW)
        {
            trigger_count++;
            auto now = std::chrono::high_resolution_clock::now();
            double delta_t = std::chrono::duration<double>(now - last_trigger_time).count();

            if (delta_t > 0.0)
            {
                double omega = (2 * M_PI) / delta_t;  // rad/s per full rotation (if 1 trigger = 1 rotation)
                ROS_INFO_STREAM("Triggers: " << trigger_count << " | Omega = " << omega << " rad/s");
                if (std::abs(omega - omega_d) > 0.5) {
                    pulse_us = int(pulse_us + k*((omega - omega_d)/std::abs(omega - omega_d)));
                }
            }
            last_trigger_time = now;
        }
        prev_state = encoder_state;
        iterations++;
    }

    // Cleanup
    digitalWrite(PUL_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);

    auto total_time = std::chrono::high_resolution_clock::now() - start_time;
    double elapsed = std::chrono::duration<double>(total_time).count();
    ROS_INFO("Theoretical pulse time: %d", int(pulse_w/2));
    ROS_INFO("Real pulse time: %d", pulse_us);
    ROS_INFO("Iterations: %d", iterations);
    ROS_INFO("Total triggers: %d", trigger_count);
    ROS_INFO("Elapsed time: %.3f s", elapsed);
    ROS_INFO("Hz: %.3f s", iterations/elapsed);
    ROS_INFO("Average omega = %.3f rad/s", (trigger_count * 2 * M_PI) / elapsed);

    ros::shutdown();
    return 0;
}
