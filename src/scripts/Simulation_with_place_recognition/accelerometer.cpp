/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

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
Log::log Acc_log; //Log object

geometry_msgs::PoseStamped accelerometer_pose;
ros::Publisher accelerometer_pub;
int node_id = 0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

bool accel_server_trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	accelerometer_pose.header.stamp = ros::Time::now();
	accelerometer_pose.pose.position.x = robot[node_id].pose.x;
	accelerometer_pose.pose.position.y = robot[node_id].pose.y;
	accelerometer_pose.pose.position.z = robot[node_id].pose.z;
	accelerometer_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(robot[node_id].pose.thx,robot[node_id].pose.thy,robot[node_id].pose.thz);

	accelerometer_pub.publish(accelerometer_pose);
	node_id++;

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"accelerometer");
	ros::NodeHandle accelerometer;
	accelerometer_pub = accelerometer.advertise<geometry_msgs::PoseStamped>("accelerometer",1000);

	ros::Rate loop_rate(loop_rate);
	
	accelerometer_pose.header.frame_id = "fixed_reference";

	Acc_log.info("Accelerometer launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

// 2) Importing Data
	 ///////////////

    // 2.1) Accessing data files
		std::ifstream PathData;
		PathData.open(filePath + "/src/Data/Odometer_3D.txt");

	// Temporary data holders
		double pathdata[7]={};


	// 2.2.) Acquiring data

	//Robot path

		if (PathData.is_open()){

			Acc_log.info("Accelerometer data file opened successfully!\n");

			int i = 0;

			while(PathData >> pathdata[0] >> pathdata[1] >> pathdata[2] >> pathdata[3] >> pathdata[4] >> pathdata[5] >> pathdata[6]){

				robot[i].pose.get(pathdata);
				i++;
			};
		}
		else{

			Acc_log.error("Unable to read file!\n");
		};

// 3) Importing Data
	 ///////////////

	while(ros::ok()){
		while(node_id <= no_of_way_points){

			ros::ServiceServer accel_server = accelerometer.advertiseService("accel_server/trigger", accel_server_trigger);
			ros::spin();
		};
	};		

	return 0;
};		