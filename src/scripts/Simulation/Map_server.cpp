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
const int no_of_way_points = 50; //No. of nodes
const int feedback_interval = 5; //Interval at which visual feedback is taken
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 100; //ROS loop rate (Hertz)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

munans::Map map[no_of_nodes]; // Map object
Log::log MS_log; //Log object

geometry_msgs::PoseStamped map_pose;
ros::Publisher map_server_pub;
int node_id = 0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

bool map_server_trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	for (node_id = 0; node_id < 50 ; node_id++){
		map_pose.header.stamp = ros::Time::now();
		map_pose.pose.position.x = map[node_id].pose.x;
		map_pose.pose.position.y = map[node_id].pose.y;
		map_pose.pose.position.z = map[node_id].pose.z;
		map_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(map[node_id].pose.thx,map[node_id].pose.thy,map[node_id].pose.thz);

		map_server_pub.publish(map_pose);
		// node_id++;
	}

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"map_server");
	ros::NodeHandle mapServer;
	map_server_pub = mapServer.advertise<geometry_msgs::PoseStamped>("map_server",1000);

	ros::Rate loop_rate(loop_rate);

	map_pose.header.frame_id = "fixed_reference";

	MS_log.info("Map server launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

// 2) Importing Data
	 ///////////////

    // 2.1) Accessing data files

		std::ifstream MapData;
		MapData.open(filePath + "/src/Data/map_3D.txt");

	// Temporary data holders
		double mapdata[7]={};

	// 2.2.) Acquiring data

	//Map

		if (MapData.is_open()){

			MS_log.info("Map data file opened successfully!\n");

			int i = 0;

			while(MapData >> mapdata[0] >> mapdata[1] >> mapdata[2] >> mapdata[3] >> mapdata[4] >> mapdata[5] >> mapdata[6]){

				map[i].pose.get(mapdata);
				i++;
			};
		}
		else{

			MS_log.error("Unable to read file!\n");
		};

// 3) Importing Data
	 ///////////////

	while(ros::ok()){
		while(node_id <= no_of_nodes){

		map_pose.header.stamp = ros::Time::now();
		map_pose.pose.position.x = map[node_id].pose.x;
		map_pose.pose.position.y = map[node_id].pose.y;
		map_pose.pose.position.z = map[node_id].pose.z;
		map_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(map[node_id].pose.thx,map[node_id].pose.thy,map[node_id].pose.thz);

		map_server_pub.publish(map_pose);
		node_id++;

			ros::ServiceServer map_server = mapServer.advertiseService("map_server/trigger", map_server_trigger);
			ros::spin();
		};
	};


	return 0;
};		