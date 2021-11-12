//============================================================================
// Name        : autopilot.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Main ROS node controlling the entire system
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
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "munans_localization/Detect.h"
#include "munans_localization/DetectRequest.h"
#include "munans_localization/DetectResponse.h"

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


const int origin = 1; //ID of the origin . Actual ID of the node -1.
const int no_of_nodes = 50; //No. of nodes
const int no_of_way_points = 50; //No. of nodes
const int feedback_interval = 5; //Interval at which visual feedback is taken
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 0.2; //ROS loop rate (Hertz)
const int initial_noise_factor = 0.5;

std::string param_name;
std::vector<int> FBnodes = {3,18,43};

bool VFB_Display = false;

int node_id = 1;
int map_node_id = 1;


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// Creating object instances to store data
Log::log P_log; //Log object	

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"Autopilot");
	ros::NodeHandle autopilot;

	ros::Rate loop_rate(loop_rate);

	ros::ServiceClient solver = autopilot.serviceClient<std_srvs::Trigger>("solve_graph");
	ros::ServiceClient map_server = autopilot.serviceClient<std_srvs::Trigger>("map_server/trigger");
	ros::ServiceClient accel_server = autopilot.serviceClient<std_srvs::Trigger>("accel_server/trigger");
	ros::ServiceClient odom_server = autopilot.serviceClient<std_srvs::Trigger>("odom_server/trigger");
	ros::ServiceClient cam_server = autopilot.serviceClient<std_srvs::Trigger>("cam_server/trigger");
	ros::ServiceClient controller_server = autopilot.serviceClient<std_srvs::Trigger>("controller_server/trigger");
	ros::ServiceClient place_recog_server = autopilot.serviceClient<munans_localization::Detect>("place_recog_server/request");

	std_srvs::Trigger trigger;
	munans_localization::Detect PR_request;

	P_log.info("Autopilot launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

	P_log.info("Loading...\n");

	// Waiting for servers to load
	//solver.waitForExistence();
	// map_server.waitForExistence();
	//accel_server.waitForExistence();
	// odom_server.waitForExistence();
	//cam_server.waitForExistence();
	// controller_server.waitForExistence();
	//place_recog_server.waitForExistence();

	//Setting parameters

	ros::param::set("/visual_feedback_toggle", "off");
	ros::param::set("/covariance_ellipse_scale", 1);

	ros::Duration(5).sleep();
	P_log.info("Loading completed....!\n");
	ros::Duration(5).sleep();

	P_log.info("Building map...!\n");
	ros::Duration(1).sleep();

	while(ros::ok()){

		while(map_node_id<=no_of_nodes){

			bool map_success = map_server.call(trigger);
			map_node_id++;

			if (map_node_id > no_of_nodes){
				P_log.info("Build completed...!\n");
				ros::Duration(3).sleep();
			}
		};

		P_log.println("Node " + std::to_string(node_id));
		P_log.println("-------\n");

		// P_log.info("Map server triggered!");
		// bool map_success = map_server.call(trigger);

		// P_log.info("Controller triggered!");
		// bool controller_success = controller_server.call(trigger);

		// P_log.info("Accel server triggered!");
		// bool accel_success = accel_server.call(trigger);

		// P_log.info("Odom server triggered!");
		// bool odom_success = odom_server.call(trigger);

		// P_log.info("Cam server triggered!");
		// bool cam_success = cam_server.call(trigger);

        // if (((node_id-3) % 5 == 0)){
		// if (std::binary_search(FBnodes.begin(),FBnodes.end(),node_id)){

		// 	PR_request.request.RobotPoseID = node_id;
		// 	P_log.info("Place recognizer triggered!");
		// 	bool place_recog_success = place_recog_server.call(PR_request);

		// 	P_log.info("Solver triggered!");
        // 	bool solver_success = solver.call(trigger);

		// 	if (trigger.response.success == 1){
		// 		P_log.info(trigger.response.message);
		// 	}else{
		// 		P_log.error("Solution failed !");
		// 	};
        // };

	// 	node_id++;

	// 	if (node_id > no_of_way_points){

	// 		P_log.info("Solver triggered!");
    //     	bool solver_success = solver.call(trigger);

	// 		if (trigger.response.success == 1){
	// 			P_log.info(trigger.response.message);
	// 		}else{
	// 			P_log.error("Solution failed !");
	// 		};

	// 		// Visualizer::keepVisible();
	// 		pause();

	// 	}else{
	// 		ros::spinOnce();
	// 		ros::Duration(1).sleep();
	// 	};

		ros::spin();

     };

	return 0;
};