//=================================================================================================
// Name        : Ground_truth.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : ROS node determining the ground truth of the robot using Laser scan data and amcl
//=================================================================================================

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Geometry>

//Custom made
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

const int loop_rate = 100; //ROS loop rate (Hertz)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log GT_log; //Log object

ros::Subscriber amcl_sub;
ros::Publisher ground_truth_pub;
geometry_msgs::PoseStamped ground_truth_pose;
nav_msgs::Path robot_path;

int node_id = 0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void odometer_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
	
  	ground_truth_pose.pose.position.x = msg ->pose.pose.position.x;
	ground_truth_pose.pose.position.y = msg ->pose.pose.position.y;
	ground_truth_pose.pose.position.z = msg ->pose.pose.position.z;
	ground_truth_pose.pose.orientation.w = msg ->pose.pose.orientation.w;
	ground_truth_pose.pose.orientation.x = msg ->pose.pose.orientation.x;
	ground_truth_pose.pose.orientation.y = msg ->pose.pose.orientation.y;
	ground_truth_pose.pose.orientation.z = msg ->pose.pose.orientation.z;

	robot_path.header.stamp = ros::Time::now();
	robot_path.header.frame_id = "map";
	robot_path.poses.push_back(ground_truth_pose);

	ground_truth_pub.publish(robot_path);

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char** argv){

    ros::init(argc, argv,"ground_truth");
    ros::NodeHandle Trajectory;

    ground_truth_pub = Trajectory.advertise<nav_msgs::Path>("ground_truth",1000);

    ros::Rate loop_rate(loop_rate);

    GT_log.info("Ground Truth launched!");

  while(Trajectory.ok()){

    amcl_sub = Trajectory.subscribe("amcl_pose", 1000, odometer_callback);
    ros::spin();

  }
}