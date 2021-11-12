//============================================================================
// Name        : Controller.cpp
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


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

const int loop_rate = 100; //ROS loop rate (Hertz)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// Creating object instances to store data
Log::log Con_log; //Log object	

// geometry_msgs::PoseStamped robot_pose;
ros::Publisher controller_pub;
ros::Subscriber controller_sub;

geometry_msgs::PoseStamped trajec_pose;
nav_msgs::Path robot_path;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void controller_callback(const nav_msgs::Odometry::ConstPtr& msg){

	trajec_pose.pose.position.x = msg ->pose.pose.position.x;
	trajec_pose.pose.position.y = msg ->pose.pose.position.y;
	trajec_pose.pose.position.z = msg ->pose.pose.position.z;
	trajec_pose.pose.orientation.w = msg ->pose.pose.orientation.w;
	trajec_pose.pose.orientation.x = msg ->pose.pose.orientation.x;
	trajec_pose.pose.orientation.y = msg ->pose.pose.orientation.y;
	trajec_pose.pose.orientation.z = msg ->pose.pose.orientation.z;

	robot_path.header.stamp = ros::Time::now();
	robot_path.header.frame_id = "fixed_reference";
	robot_path.poses.push_back(trajec_pose);

	controller_pub.publish(robot_path);

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"controller");
	ros::NodeHandle controller;

	controller_pub = controller.advertise<nav_msgs::Path>("path",1000);
	ros::Rate loop_rate(loop_rate);

	Con_log.info("Controller launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

	// Initializing the visualizer
	Visualizer::Initialize(controller);


	while(ros::ok()){

		// Subscribe to the odometer

        ros::Subscriber controller_sub = controller.subscribe("pioneer1/odom", 1000, controller_callback);
		ros::spin();
    };

		return 0;
};