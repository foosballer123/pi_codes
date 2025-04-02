#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pigpio.h>

// Define GPIO pins for sensors
#define SENSOR_1 6
#define SENSOR_2 5
#define SENSOR_3 13
#define SENSOR_4 19
#define SENSOR_5 26
#define SENSOR_6 4

// Function to read sensor data
geometry_msgs::Twist read_sensors() {
    geometry_msgs::Twist sensor_data;
    
    sensor_data.linear.x = gpioRead(SENSOR_1);
    sensor_data.linear.y = gpioRead(SENSOR_2);
    sensor_data.linear.z = gpioRead(SENSOR_3);
    sensor_data.angular.x = gpioRead(SENSOR_4);
    sensor_data.angular.y = gpioRead(SENSOR_5);
    sensor_data.angular.z = gpioRead(SENSOR_6);
    
    return sensor_data;
}

int main(int argc, char** argv) {
    // Initialize pigpio ********ONLY DO THIS IN ONE FILE OR THINGS WONT WORK***********
    if (gpioInitialise() < 0) {
        ROS_ERROR("Failed to initialize pigpio");
        gpioTerminate();
        return 1;
        
    }
    
    // Set up GPIO as inputs
    gpioSetMode(SENSOR_1, PI_INPUT);
    gpioSetMode(SENSOR_2, PI_INPUT);
    gpioSetMode(SENSOR_3, PI_INPUT);
    gpioSetMode(SENSOR_4, PI_INPUT);
    gpioSetMode(SENSOR_5, PI_INPUT);
    gpioSetMode(SENSOR_6, PI_INPUT);
    
    // Initialize ROS node
    ros::init(argc, argv, "sensor_publisher");
    ros::NodeHandle nh;
    // Initialize ROS publisher
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("sensor_data", 10);
    ros::Rate rate(10); // Publish at 10 Hz
    
    ROS_INFO("Starting sensor data stream...");
    
    try {
        while (ros::ok()) {
            // Read sensor data
            geometry_msgs::Twist sensor_data = read_sensors();
            
            // Publish sensor data
            pub.publish(sensor_data);
            rate.sleep();
        }
    } catch (...) {
        ROS_WARN("Shutting down sensor data publisher...");
    }
    
    // Cleanup GPIO before exiting
    gpioTerminate();
    return 0;
}
