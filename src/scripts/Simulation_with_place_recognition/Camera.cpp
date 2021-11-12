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
#include <sensor_msgs/image_encodings.h>
// #include <rviz_visual_tools/rviz_visual_tools.h>

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

const int origin = 3-1; //ID of the origin . Actual ID of the node -1.
const int no_of_nodes = 48; //No. of nodes
const int no_of_way_points = 48; //No. of nodes
const int feedback_interval = 5; //Interval at which visual feedback is taken
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 100; //ROS loop rate (Hertz)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

munans::Robot robot[no_of_way_points]; //Robot object
Log::log cam_log; //Log object

geometry_msgs::PoseStamped cam_pose;
ros::Publisher camera_pub;
int node_id = 0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

bool cam_server_trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	cam_pose.header.stamp = ros::Time::now();
	cam_pose.pose.position.x = robot[node_id].pose.x;
	cam_pose.pose.position.y = robot[node_id].pose.y;
	cam_pose.pose.position.z = robot[node_id].pose.z;
	cam_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(robot[node_id].pose.thx,robot[node_id].pose.thy,robot[node_id].pose.thz);

	camera_pub.publish(cam_pose);
	node_id++;

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"camera");
	ros::NodeHandle camera;
	camera_pub = camera.advertise<geometry_msgs::PoseStamped>("camera/image",1000);

	ros::Rate loop_rate(loop_rate);

	cam_pose.header.frame_id = "fixed_reference";

	cam_log.info("Camera launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

// 2) Importing Data
	 ///////////////

    // 2.1) Accessing data files

		std::ifstream VisualData;
		VisualData.open(filePath + "/src/Data/visual_3D.txt");

	// Temporary data holders
		double visualdata[7]={};

	// 2.2.) Acquiring data

	//Visual feedback

		if (VisualData.is_open()){

			cam_log.info("Visual data file opened successfully!\n");

			int j = 0;

			while(j<no_of_way_points){

				if ((j != origin) & ((j-origin)%feedback_interval ==0)){

					int feedback_node = (j-origin)/feedback_interval;

					VisualData >> visualdata[0] >> visualdata[1] >> visualdata[2] >> visualdata[3] >> visualdata[4] >> visualdata[5] >> visualdata[6];

					robot[j].visual.get(visualdata);

				};

				j++;
			};
		}
		else{

			cam_log.error("Unable to read file!\n");
		};

// 3) Importing Data
	 ///////////////

	while(ros::ok()){
		while(node_id <= no_of_way_points){

			ros::ServiceServer cam_server = camera.advertiseService("cam_server/trigger", cam_server_trigger);
			ros::spin();
		};
	};			

	return 0;
};		