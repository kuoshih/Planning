/*----------------------------------------------------------------------------
--  Name: planning\planning.cpp
--  Description:  planning code.
--  This code loads 2D map (Lidar) and 3D map (Rtabmap). 
--  It:
--  1. rotates 3D map to matching 2D map.
--  2. moves to (0.3,0,0).
--    
--  $Revision: 1.0 $            
--  coding: Kuo-Shih Tseng, NCU.  2024.07.201~  
--  known bugs: 
--  TODO:
--

----------------------------------------------------------------------------*/

#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/BatteryState.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <vector>
#include <cmath>
#include <iostream>



#define FILE_PATH "/home/user/subgoal_map/subgoal.dat"
#define Pi 3.14159265358979323846
#define RAD2DEG(x) ((x)*180./M_PI)
#define UVC_Y 0.3

#define costMap_min 0 // CostMap threshold for ground set
#define costMap_max 30 //30

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include "nav_msgs/OccupancyGrid.h"


#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <iostream>
#include <unistd.h>
#include <JetsonGPIO.h>

#include <fstream>
#include <streambuf>

struct pos2D{   
  float x;           
  float y;        
  float theta;
};


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace octomap;
using namespace std;

// prameters from /param/planning.yaml
int K=3; // subgoal numberfloat Subgoal_time=20;
float Subgoal_time=20;


void timercallback(const ros::TimerEvent&);
void init_marker(void);

visualization_msgs::Marker marker;
visualization_msgs::MarkerArray marker_array_msg;
visualization_msgs::MarkerArray marker_array_msg2;
visualization_msgs::MarkerArray marker_array_msg3;
uint32_t shape = visualization_msgs::Marker::CYLINDER;
uint32_t cube_shape = visualization_msgs::Marker::CUBE;
ros::Publisher marker_pub;
ros::Publisher marker_arrary_pub;
ros::Publisher marker_arrary_pub2;
ros::Publisher marker_arrary_pub3;
ros::Publisher cmd_vel_pub_;

octomap_msgs::Octomap octomap1;
octomap_msgs::Octomap octomap2;
octomap_msgs::Octomap octomap3;
ros::Publisher octomap_pub ;
ros::Publisher octomap_pub2 ;
ros::Publisher octomap_pub3 ;
octomap::ColorOcTree*  PtrOctomap_loaded;
octomap::ColorOcTree*  PtrOctomap_global;
octomap::ColorOcTree*  PtrOctomap_surface;
octomap::ColorOcTree*  PtrOctomap_temp;
octomap::ColorOcTree*  PtrOctomap_current;
octomap::ColorOcTree Octomap_temp(0.02);

float Battery_volt_t=0;
int grid_N=0;
int counter=0;

void timercallback(const ros::TimerEvent&)
{
   // update octomap and publish it. 

   ROS_INFO("Counter=%d", counter);

   octomap1.header.frame_id = "map";
   octomap1.header.stamp = ros::Time::now();
   octomap_msgs::fullMapToMsg(*PtrOctomap_global, octomap1);	

    octomap2.header.frame_id = "map";
    octomap2.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*PtrOctomap_surface, octomap2);

    octomap3.header.frame_id = "map";
    octomap3.header.stamp = ros::Time::now();
    octomap_msgs::fullMapToMsg(*PtrOctomap_current, octomap3);
    
    octomap_pub.publish(octomap1);		
    octomap_pub2.publish(octomap2);	
    octomap_pub3.publish(octomap3);	
    	
    // update maker location and publish it. 
    marker_arrary_pub.publish(marker_array_msg);
    marker_arrary_pub2.publish(marker_array_msg2);
    marker_arrary_pub3.publish(marker_array_msg3);

    counter++;	
     
}



void rotateOctomap(octomap::ColorOcTree* tree, octomap::ColorOcTree* rot_tree,double angle_rad) {

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        double x = it.getX()+0; // + RGBD Camera X location
        double y = it.getY()-0.3; // + RGBD Camera Y location
        double z = it.getZ()+0.25; //  + RGBD Camera Z location

        double new_x = x * cos(angle_rad) - y * sin(angle_rad);
        double new_y = x * sin(angle_rad) + y * cos(angle_rad);
        
        const octomap::point3d d(new_x, new_y, z); 	    
	ColorOcTreeNode* n = rot_tree->updateNode(d, true); // add this node to the tree
	n->setColor(it->getColor()); // set color
    }
}


void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
    Battery_volt_t = msg->voltage;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Planning_node");
  ros::NodeHandle n;

  // create a timer callback
  ros::Timer timer1 = n.createTimer(ros::Duration(0.5), timercallback);

  ros::Subscriber battery_sub = n.subscribe<sensor_msgs::BatteryState>("/battery_state", 10, batteryStateCallback);

  // create topic "visualization_marker"
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_arrary_pub= n.advertise<visualization_msgs::MarkerArray>("scan_area", 100);
  marker_arrary_pub2= n.advertise<visualization_msgs::MarkerArray>("ground_set", 100);
  marker_arrary_pub3= n.advertise<visualization_msgs::MarkerArray>("laser_center", 100);
  marker_array_msg.markers.resize(100);
  marker_array_msg2.markers.resize(100); // need to be upgraded by costmap2D
  marker_array_msg3.markers.resize(100);
  cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  n.param<int>("K", K, 3);
  ROS_INFO("The K from param/planning.yaml is %d",K);
  n.param<float>("Subgoal_Time", Subgoal_time, 20.0);
  

  ROS_INFO("START to run main code!");

  ROS_INFO("Load 3D map (ot format)");
  octomap_pub = n.advertise<octomap_msgs::Octomap>("octomap_loaded", 1);
  octomap_pub2 = n.advertise<octomap_msgs::Octomap>("octomap_surface", 1);
  octomap_pub3 = n.advertise<octomap_msgs::Octomap>("octomap_current", 1);
  octomap::AbstractOcTree* tree1 = AbstractOcTree::read("/home/user/temp_otfile.ot");
  PtrOctomap_loaded = dynamic_cast<octomap::ColorOcTree*>(tree1);
  
  octomap::AbstractOcTree* rot_tree= new octomap::ColorOcTree(0.1);
  rot_tree->clear(); 
  rot_tree->setResolution(0.05); //
  PtrOctomap_global = dynamic_cast<octomap::ColorOcTree*>(rot_tree);
  rotateOctomap(PtrOctomap_loaded, PtrOctomap_global,1.57); // roate the Octomap 90 degrees to matching 2D grid map.s

  octomap::AbstractOcTree* tree2= new octomap::ColorOcTree(0.1); // initial a tree
  tree2->clear();
  tree2->setResolution(0.05); //0.1
  PtrOctomap_surface = dynamic_cast<octomap::ColorOcTree*>(tree2);
  ROS_INFO("tree1_size=%i,tree2_size=%i",PtrOctomap_global->size(),PtrOctomap_surface->size());
  
  octomap::AbstractOcTree* tree3= new octomap::ColorOcTree(0.1); // initial a tree
  tree3->clear();
  tree3->setResolution(0.05); //0.1  
  PtrOctomap_current = dynamic_cast<octomap::ColorOcTree*>(tree3);
  /**/


 //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
	
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(0.5))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //ROS_INFO("Move_base action server starts!");


  ROS_INFO("===========Charging_path================");
  pos2D subgoal;
  subgoal.x=0.30;
  subgoal.y=0;//-0.3+Y_shift;
  subgoal.theta=0;//-1.57;
  ROS_INFO("Sending charging goal #1, x=%.2f,y=%.2f,theta=%.2f",subgoal.x,subgoal.y,subgoal.theta);
    
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map"; 
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = subgoal.x;
  goal.target_pose.pose.position.y = subgoal.y;	
  geometry_msgs::Quaternion quat_msg;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, subgoal.theta); //Euler to Quaternion
  tf2::convert(quat, quat_msg); 
  goal.target_pose.pose.orientation = quat_msg;

  ac.sendGoalAndWait(goal, ros::Duration(Subgoal_time,0), ros::Duration(Subgoal_time,0));
		
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {  
  	ROS_INFO("Goal arrived!");
  }
  else
  {  
  	ROS_INFO("The base failed to move to goal for some reasons");
  }


  ros::spin();


  return 0;
}


void init_marker(void)
{

    // Initialize maker's setting.
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // Tag(ACTION)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //Tag(POSE)
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.3;

    //Tag(LIFETIME)
    marker.lifetime = ros::Duration();
	
	for ( int i = 0; i < K; i++)
	{	// subgoal_scan (purple)
		marker_array_msg.markers[i].header.frame_id = "map";
		marker_array_msg.markers[i].header.stamp = ros::Time();
		marker_array_msg.markers[i].ns = "my_namespace";
		marker_array_msg.markers[i].id = i;
		marker_array_msg.markers[i].type = cube_shape;
		marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
		marker_array_msg.markers[i].pose.position.x = 0.0;
		marker_array_msg.markers[i].pose.position.y = 0.0;
		marker_array_msg.markers[i].pose.position.z = -0.05;
		marker_array_msg.markers[i].pose.orientation.x = 0.0;
		marker_array_msg.markers[i].pose.orientation.y = 0.0;
		marker_array_msg.markers[i].pose.orientation.z = 0.0;
		marker_array_msg.markers[i].pose.orientation.w = 1.0;
		marker_array_msg.markers[i].scale.x = 0.3;
		marker_array_msg.markers[i].scale.y = 0.2;
		marker_array_msg.markers[i].scale.z = 0.1;
		marker_array_msg.markers[i].color.a = 0.3;
		marker_array_msg.markers[i].color.r = 0.55; //purple
		marker_array_msg.markers[i].color.g = 0;
		marker_array_msg.markers[i].color.b = 0.55; //purple
	   	marker_array_msg.markers[i].lifetime = ros::Duration();
	}
	
	for ( int i = 0; i < grid_N; i++)
	{	// ground_set (blue)
		marker_array_msg2.markers[i].header.frame_id = "map";
		marker_array_msg2.markers[i].header.stamp = ros::Time();
		marker_array_msg2.markers[i].ns = "my_namespace";
		marker_array_msg2.markers[i].id = i;
		marker_array_msg2.markers[i].type = shape;
		marker_array_msg2.markers[i].action = visualization_msgs::Marker::ADD;
		marker_array_msg2.markers[i].pose.position.x = 0.0;
		marker_array_msg2.markers[i].pose.position.y = 0.0;
		marker_array_msg2.markers[i].pose.position.z = 0.0;
		marker_array_msg2.markers[i].pose.orientation.x = 0.0;
		marker_array_msg2.markers[i].pose.orientation.y = 0.0;
		marker_array_msg2.markers[i].pose.orientation.z = 0.0;
		marker_array_msg2.markers[i].pose.orientation.w = 1.0;
		marker_array_msg2.markers[i].scale.x = 0.1;
		marker_array_msg2.markers[i].scale.y = 0.1;
		marker_array_msg2.markers[i].scale.z = 0.25;
		marker_array_msg2.markers[i].color.a = 0.5;
		marker_array_msg2.markers[i].color.r = 0.0;
		marker_array_msg2.markers[i].color.g = 0.0;
		marker_array_msg2.markers[i].color.b = 1.0; // blue
	    	marker_array_msg2.markers[i].lifetime = ros::Duration();
	}
	
		
			
	for ( int i = 0; i < 100; i++)
	{	// laser center (red)
		marker_array_msg3.markers[i].header.frame_id = "map";
		marker_array_msg3.markers[i].header.stamp = ros::Time();
		marker_array_msg3.markers[i].ns = "my_namespace";
		marker_array_msg3.markers[i].id = i;
		marker_array_msg3.markers[i].type = cube_shape;
		marker_array_msg3.markers[i].action = visualization_msgs::Marker::ADD; //DELETE if no show.
		//marker_array_msg3.markers[i].action = visualization_msgs::Marker::DELETE;
		marker_array_msg3.markers[i].pose.position.x = 0;
		marker_array_msg3.markers[i].pose.position.y = 0;
		marker_array_msg3.markers[i].pose.position.z = 0;
		marker_array_msg3.markers[i].pose.orientation.x = 0.0;
		marker_array_msg3.markers[i].pose.orientation.y = 0.0;
		marker_array_msg3.markers[i].pose.orientation.z = 0.0;
		marker_array_msg3.markers[i].pose.orientation.w = 1.0;
		marker_array_msg3.markers[i].scale.x = 0.1;
		marker_array_msg3.markers[i].scale.y = 0.1;
		marker_array_msg3.markers[i].scale.z = 0.2;
		marker_array_msg3.markers[i].color.a = 0.5;
		marker_array_msg3.markers[i].color.r = 1.0; // red
		marker_array_msg3.markers[i].color.g = 0.0;
		marker_array_msg3.markers[i].color.b = 0.0;
	    	marker_array_msg3.markers[i].lifetime = ros::Duration();
	}
	


}





