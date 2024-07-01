#!/bin/sh

# open Lidar SLAM #  
gnome-terminal -- roslaunch turtlebot3_slam turtlebot3_slam.launch

# open telelop #  
sleep 1 && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

