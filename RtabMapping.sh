#!/bin/sh

# open realsense camera
gnome-terminal -- roslaunch realsense2_camera rs_camera.launch 	align_depth:=true 	unite_imu_method:="linear_interpolation" 	enable_gyro:=true  	enable_accel:=true

# open IMU filter
sleep 5 && gnome-terminal -- rosrun imu_filter_madgwick imu_filter_node 	_use_mag:=false 	_publish_tf:=false 	_world_frame:="enu" 	/imu/data_raw:=/camera/imu 	/imu/data:=/rtabmap/imu

# open RTABMAP #  
sleep 5 && gnome-terminal -- roslaunch rtabmap_launch rtabmap.launch     args:="--Optimizer/GravitySigma 0.3  --delete_db_on_start"	depth_topic:=/camera/aligned_depth_to_color/image_raw 	rgb_topic:=/camera/color/image_raw 	camera_info_topic:=/camera/color/camera_info 	approx_sync:=false 	wait_imu_to_init:=true 	imu_topic:=/rtabmap/imu

# open telelop #  
sleep 1 && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

