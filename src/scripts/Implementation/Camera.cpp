//============================================================================
// Name        : Camera.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : ROS node simulating a camera
//============================================================================

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

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

#define DEFINE_GLOBALS // Prevents the variables getting redefined
#include "variables.h" // Contains all the variables related to visualization
#include "Visualizer.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

const int origin = 1; //ID of the origin . Actual ID of the node -1.
const int no_of_nodes = 48; //No. of nodes
const int no_of_way_points = 48; //No. of nodes
const int feedback_interval = 5; //Interval at which visual feedback is taken
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 100; //ROS loop rate (Hertz)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log cam_log; //Log object

int node_id = 1;
std::string filePath = "";
std::string capture_img = "";

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

bool cam_server_trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	// Publishing the node feedback in the visualizer
	capture_img = filePath + "/src/munans_localization/src/ISLAB/node(" + std::to_string(node_id) + ").jpg";
	
	cam_log.info("Feedback image captured");

	Visualizer::publishCameraFeed(capture_img);

	node_id++;

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"camera");
	ros::NodeHandle Cam;

	ros::Rate loop_rate(loop_rate);

	// cam_pose.header.frame_id = "map";

	Visualizer::Initialize(Cam);

	ros::Publisher map_img_pub = Cam.advertise<sensor_msgs::Image>("pano_img", 1000);
	ros::Publisher query_img_pub = Cam.advertise<sensor_msgs::Image>("query_img", 1000);

	cam_log.info("Camera launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");


// 2) Importing Data
	 ///////////////

	while(ros::ok()){
		if(node_id <= no_of_way_points){

			ros::ServiceServer cam_server = Cam.advertiseService("cam_server/trigger", cam_server_trigger);
			// 	// Publishing the node feedback in the visualizer
			// capture_img = filePath + "/src/munans_localization/src/ISLAB_cropped/node(" + std::to_string(node_id) + ").jpg";
			
			// cam_log.info("Feedback image captured");

			// Visualizer::publishCameraFeed(capture_img);

			// node_id++;
			// ros::spinOnce();
			// ros::Duration(1).sleep();
			ros::spin();
		};
	};			

	return 0;
};		