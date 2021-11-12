//============================================================================
// Name        : Seekur.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : ROS node running the controller for the robot
//============================================================================

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Geometry>

//GTSAM
	//geometry
	#include <gtsam/geometry/Pose3.h>
	#include <gtsam/geometry/Point3.h>
	#include <gtsam/geometry/Rot3.h>

//Custom made
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

#define DEFINE_GLOBALS // Prevents the variables getting redefined
#include "variables.h" // Contains all the variables related to visualization
#include "Visualizer.h"

//Namespaces
using namespace gtsam;

tf::Transform rob_pose = tf::Transform(tf::createQuaternionFromRPY(0,0,3.14/2) , tf::Vector3(0,-8.6,0));;

void trajectory_callback(const nav_msgs::Odometry::ConstPtr& msg){

	tf::Quaternion rob_orientation = tf::Quaternion(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z); //Rotation matrix 
	tf::Vector3 rob_position = tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z); //Translation vector	

	rob_pose = tf::Transform(rob_orientation, rob_position);
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "seekur");
  ros::NodeHandle seekur;

  ros::Rate rate(100);

  tf::TransformBroadcaster robot_pose_broadcaster;

  while(seekur.ok()){

    ros::Subscriber trajectory_sub = seekur.subscribe("pioneer1/odom", 1, trajectory_callback);
	robot_pose_broadcaster.sendTransform(tf::StampedTransform(rob_pose,ros::Time::now(),"pioneer1/odom", "pioneer1/base_link"));

    ros::spinOnce();
    rate.sleep();
  }
}