#!/bin/bash

echo "Setting up environment variables..."
export ROS_MASTER_URI=http://10.42.0.1:11311
export ROS_HOSTNAME=10.42.0.194

echo "Sourcing ROS setup..."
source /opt/ros/noetic/setup.bash
source /home/FooBot/catkin_ws/devel/setup.bash

echo "Starting Launch File..."
roslaunch pi_codes foobot.launch
#rosrun pi_codes cont_move_pi.py
