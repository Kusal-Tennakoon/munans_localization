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
#include <std_srvs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

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
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 100; //ROS loop rate (Hertz)
const int initial_noise_factor = 0.5;
const int covariance_ellipse_scale = 1000;
std::string correction = "on";

bool VFB_Display = false;

int node_id = 1;


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// Creating object instances to store data
Log::log Con_log; //Log object	

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void controller_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

	ros::Time current_time = ros::Time::now();

	Rot3 Ri = Rot3::quaternion(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z); //Rotation matrix 
	Point3 Ti = Point3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z); //Translation vector		

	Visualizer::collectTrajectoryPose(Pose3(Ri,Ti),node_id);

	// Panorama image
    std::string camera_feed_img ="./src/ISLAB/node(" + std::to_string(node_id) + ").jpg";

	robot_odometry_trans.header.stamp = current_time;
	
	// Transform
    robot_odometry_trans.transform.translation.x = msg->pose.position.x;
    robot_odometry_trans.transform.translation.y = msg->pose.position.y;
    robot_odometry_trans.transform.translation.z = msg->pose.position.z;
    robot_odometry_trans.transform.rotation.x = msg->pose.orientation.x;
    robot_odometry_trans.transform.rotation.y = msg->pose.orientation.y;
    robot_odometry_trans.transform.rotation.z = msg->pose.orientation.z;
    robot_odometry_trans.transform.rotation.w = msg->pose.orientation.w;

    // Publish data
    Visualizer::publishRobotPose(Pose3(Ri,Ti),current_time);
    Visualizer::publishCameraFeed(camera_feed_img);
	Visualizer::publishTrajectory();
	Con_log.info("Robot way point " + std::to_string(node_id)+ " passed!");
	node_id++;

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"controller");
	ros::NodeHandle controller;
	tf::TransformBroadcaster robot_pose_broadcaster;

	ros::Rate loop_rate(loop_rate);

	ros::ServiceClient solver = controller.serviceClient<std_srvs::Empty>("solve_graph");

	std_srvs::Empty::Request request;
	std_srvs::Empty::Response response;

	Con_log.info("Controller launched!");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

	// Initializing the visualizer
	Visualizer::Initialize(controller);

	//Robot model starting point
	robot_odometry_trans.header.stamp = ros::Time::now();
	
	// Transform
    robot_odometry_trans.transform.translation.x = -0.2846;
    robot_odometry_trans.transform.translation.y = -8.3164;
    robot_odometry_trans.transform.translation.z = 10;
    robot_odometry_trans.transform.rotation.x = 0;
    robot_odometry_trans.transform.rotation.y = 0;
    robot_odometry_trans.transform.rotation.z = 0;
    robot_odometry_trans.transform.rotation.w = 1;

    // robot_pose_broadcaster.sendTransform(robot_odometry_trans);

    // Publish data
    robot_pose_broadcaster.sendTransform(robot_odometry_trans);
    Con_log.info("Out!");

	while(ros::ok()){

        ros::Subscriber controller_sub = controller.subscribe("accelerometer", 1000, controller_callback);
        // Con_log.info("In loop!");

        if ((node_id-3) % 5 == 0){
        	Con_log.info("In if!");
        	bool success = solver.call(request,response);
        	Con_log.info("Request sent!");

        };
        // sleep(5);
        // node_id++;
		// loop_rate.sleep();
		ros::spin();
    };

		return 0;
};