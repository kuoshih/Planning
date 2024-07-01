
# Planning


## Abstract
This is a sample code for TB3 with Rtabmap + 2D map.
It incudes:
1. build and save Rtabmap (RtabMapping.sh and RtabSave.sh)
2. build and save Lidar map (LidarMapping.sh and LidarSave.sh)
3. planning.cpp:  
3.1 rotates 3D map to matching 2D map.  
3.2 moves to (0.3,0,0).  

The code runs on Jetson Orin NX and a Turtlebot3 Waffle/Burger.  
This code activate a node "planning".   
You can access data from Callback functions (e.g., batteryStateCallback and timercallback) in src/planning.cpp.  
There are makers for landmarks or targets.   
There are 3D occupied map (octomap).  
You can see them in rviz.  

## About us

Developer:   
* Kuo-Shih Tseng   
Contact: kuoshih@math.ncu.edu.tw   
Date: 2024/07/01  

## About us
Before running the code,  
you should install  
1. Rtabmap  (https://github.com/introlab/rtabmap_ros)  
2. TB3 packages (https://github.com/ROBOTIS-GIT/turtlebot3)  

## 3D and 2D mapping
Rtabmap:  
$ ./RtabMapping.sh  
The script will run Rtabmap and remote control code.  
You can control your robot for 3D mapping.  
$ ./RtabSave.sh  
The script will save the rtabmap to ~/rtabmap.db  

Lidar SLAM:  
$ ./LidarMapping.sh  
The script will run turtlebot3_slam.launch and turtlebot3_teleop_key.launch.  
You can control your robot for 2D mapping.  
$ ./RtabSave.sh  
The script will save ~/map.pgm ~/map.yaml and copy them to ~/subgoal_map  

## Compile the code
$cd catkin_ws    
$catkin_make    

## Planning
Start the node of TB3:  
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch  
Start Navigation launch:  
After running it, you will see the rviz.  
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml  
Start planning launch:  
$ roslaunch planning planning.launch  


## rviz
Once the rviz is actived by turtlebot3_navigation.launch,  
you can load the rviz config file from ~catkin_ws\src\planning\rviz\coverage_rviz.rviz  
You should see the following rviz HMI.   
![alt text](https://github.com/kuoshih/Planning/blob/main/Screenshot.png)  

