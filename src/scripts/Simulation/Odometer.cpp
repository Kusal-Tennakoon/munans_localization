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

const int origin = 1; //ID of the origin . Actual ID of the node -1.
const int no_of_nodes = 50; //No. of nodes
const int no_of_way_points = 45; //No. of nodes
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 100; //ROS loop rate (Hertz)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

munans::Robot robot[no_of_way_points]; //Robot object
Log::log Odom_log; //Log object

geometry_msgs::PoseStamped odom_pose;
ros::Publisher odometer_pub;
int node_id = 0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

bool odom_server_trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	
	odom_pose.header.stamp = ros::Time::now();
	odom_pose.pose.position.x = robot[node_id].odometry.x;
	odom_pose.pose.position.y = robot[node_id].odometry.y;
	odom_pose.pose.position.z = robot[node_id].odometry.z;
	odom_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(robot[node_id].odometry.thx,robot[node_id].odometry.thy,robot[node_id].odometry.thz);

	odometer_pub.publish(odom_pose);
	node_id++;

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"odometer");
	ros::NodeHandle Odometer;
	odometer_pub = Odometer.advertise<geometry_msgs::PoseStamped>("odometer",1000);

	ros::Rate loop_rate(loop_rate);
	
	odom_pose.header.frame_id = "fixed_reference";

	Odom_log.info("Odometer launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

// 2) Importing Data
	 ///////////////

    // 2.1) Accessing data files
		std::ifstream OdometryData;
		OdometryData.open(filePath + "/src/Data/odometry_3D.txt");

	// Temporary data holders
		double odometrydata[8]={};


	// 2.2.) Acquiring data

	//Robot odometry

		if (OdometryData.is_open()){

			Odom_log.info("Odometry data file opened successfully!\n");

			int i = 0;

			while(OdometryData >> odometrydata[0] >> odometrydata[1] >> odometrydata[2] >> odometrydata[3] >> odometrydata[4] >> odometrydata[5] >> odometrydata[6] >> odometrydata[7]){

				robot[i].odometry.get(odometrydata);
				i++;
			};
		}
		else{

			Odom_log.error("Unable to read file!\n");
		};

// 3) Importing Data
	 ///////////////

	while(ros::ok()){
		while(node_id <= no_of_way_points){

			ros::ServiceServer odom_server = Odometer.advertiseService("odom_server/trigger", odom_server_trigger);
			ros::spin();
		};
	};		

	return 0;
};		