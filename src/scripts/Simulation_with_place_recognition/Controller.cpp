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
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Trigger.h>
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

const int origin = 3-1; //ID of the origin . Actual ID of the node -1.
const int no_of_nodes = 48; //No. of nodes
const int no_of_way_points = 48; //No. of nodes
const int feedback_interval = 5; //Interval at which visual feedback is taken
const int loop_rate = 100; //ROS loop rate (Hertz)

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// Creating object instances to store data
Log::log Con_log; //Log object	
munans::Robot robot[no_of_way_points]; //Robot object

// geometry_msgs::PoseStamped robot_pose;
ros::Publisher controller_pub;
int node_id = 0;

std::vector<Pose3> traj_poses;
geometry_msgs::PoseStamped trajec_pose;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// void controller_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

// 	Rot3 Ri = Rot3::quaternion(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z); //Rotation matrix 
// 	Point3 Ti = Point3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z); //Translation vector

// 	traj_poses.push_back(Pose3(Ri,Ti));	
// 	Visualizer::collectTrajectoryPose(traj_poses.at(node_id),node_id+1);	
// 	Visualizer::publishTrajectory();
// 	Visualizer::publishRobotPose(traj_poses.at(node_id),ros::Time::now());
// 	Con_log.info("Robot way point " + std::to_string(node_id+1)+ " passed!");
// 	node_id++;
// }

bool controller_server_trigger(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){

	// Rot3 Ri = Rot3::RzRyRx (robot[node_id].pose.thx,robot[node_id].pose.thy,robot[node_id].pose.thz); //Rotation matrix 
	// Point3 Ti = Point3(robot[node_id].pose.x,robot[node_id].pose.y,robot[node_id].pose.z); //Translation vector

	// traj_poses.push_back(Pose3(Ri,Ti));	
	// Visualizer::collectTrajectoryPose(traj_poses.at(node_id),node_id+1);	
	// Visualizer::publishTrajectory();
	// Visualizer::publishRobotPose(traj_poses.at(node_id),ros::Time::now());
	// Con_log.info("Robot way point " + std::to_string(node_id+1)+ " passed!");

	trajec_pose.header.stamp = ros::Time::now();
	trajec_pose.pose.position.x = robot[node_id].pose.x;
	trajec_pose.pose.position.y = robot[node_id].pose.y;
	trajec_pose.pose.position.z = robot[node_id].pose.z;
	trajec_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(robot[node_id].pose.thx,robot[node_id].pose.thy,robot[node_id].pose.thz);

	controller_pub.publish(trajec_pose);
	node_id++;

	// // Panorama image
    // std::string camera_feed_img ="./src/ISLAB/node(" + std::to_string(node_id+1) + ").jpg";

	// ros::Time current_time = ros::Time::now();
	
	// // Transform
    // robot_odometry_trans.transform.translation.x = msg->pose.position.x;
    // robot_odometry_trans.transform.translation.y = msg->pose.position.y;
    // robot_odometry_trans.transform.translation.z = msg->pose.position.z;
    // robot_odometry_trans.transform.rotation.x = msg->pose.orientation.x;
    // robot_odometry_trans.transform.rotation.y = msg->pose.orientation.y;
    // robot_odometry_trans.transform.rotation.z = msg->pose.orientation.z;
    // robot_odometry_trans.transform.rotation.w = msg->pose.orientation.w;

    // Publish data 

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"controller");
	ros::NodeHandle controller;
	// tf::TransformBroadcaster robot_pose_broadcaster;

	controller_pub = controller.advertise<geometry_msgs::PoseStamped>("controller",1000);
	
	trajec_pose.header.frame_id = "fixed_reference";

	ros::Rate loop_rate(loop_rate);

	Con_log.info("Controller launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

	// Initializing the visualizer
	Visualizer::Initialize(controller);

// 2) Importing Data
	 ///////////////

    // 2.1) Accessing data files
		std::ifstream PathData;
		PathData.open(filePath + "/src/Data/path_3D.txt");

	// Temporary data holders
		double pathdata[7]={};


	// 2.2.) Acquiring data

	//Robot path

		if (PathData.is_open()){

			Con_log.info("Path data file opened successfully!\n");

			int i = 0;

			while(PathData >> pathdata[0] >> pathdata[1] >> pathdata[2] >> pathdata[3] >> pathdata[4] >> pathdata[5] >> pathdata[6]){

				robot[i].pose.get(pathdata);
				i++;
			};
		}
		else{

			Con_log.error("Unable to read file!\n");
		};

	// //Robot model starting point
	// robot_odometry_trans.header.stamp = ros::Time::now();
	
	// // Transform
    // robot_odometry_trans.transform.translation.x = -0.2846;
    // robot_odometry_trans.transform.translation.y = -8.3164;
    // robot_odometry_trans.transform.translation.z = 10;
    // robot_odometry_trans.transform.rotation.x = 0;
    // robot_odometry_trans.transform.rotation.y = 0;
    // robot_odometry_trans.transform.rotation.z = 0;
    // robot_odometry_trans.transform.rotation.w = 1;

	while(ros::ok()){

		// Publish data
		// robot_pose_broadcaster.sendTransform(robot_odometry_trans);

        // ros::Subscriber controller_sub = controller.subscribe("accelerometer", 1000, controller_callback);

		ros::ServiceServer controller_server = controller.advertiseService("controller_server/trigger", controller_server_trigger);
		
		ros::spin();
    };

		return 0;
};