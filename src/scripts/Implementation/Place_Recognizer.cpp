//============================================================================
// Name        : Place_Recognizer.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : ROS node handling the place recognition task
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
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "munans_localization/EssentialMatrixConstraint.h"
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

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "ASiftDetector.h"

//Custom made
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

const double kNewWidth = 480;
const double kNewHeight = 360;
const double kRescaleFactor_trial = 1;
const double kRescaleFactor_test = 1;

int no_of_query_images = 10;
int starting_image = 11;
int no_of_putatives = 10;
int max_features = 500;
int resize_factor = 1;
int hes_thresh = 12000;
int predicted_node = 0;
double verification_thresh = 0.1000;

const int origin = 3-1; //ID of the origin . Actual ID of the node -1.
const int no_of_nodes = 48; //No. of nodes
const int no_of_way_points = 48; //No. of nodes
const int Simulation_delay = 1; //Simulation delay (seconds)
const int loop_rate = 100; //ROS loop rate (Hertz)

int node_id = 0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

std::string image_type = "png";
std::string debug_mode = "off";
std::string second_stage_verification = "on";
std::string present_absent = "Absent";

std::vector<int> presense_array(no_of_putatives, 0);
std::vector<int> putatives(no_of_putatives, 0);
std::vector<Qmath::Max<int>> max_node;
std::vector<double> verif_array;
std::vector<Qmath::Max<double>> node_closest;

munans::features brief_test; 
munans::closestNode node_cl;

Qmath::Vector<int> voting_array(no_of_nodes);

cv::Mat K; // Camera matrix
Log::log PR_log; //Log object

geometry_msgs::Pose EssentialMat;
munans::EssentialMatrix Essential;
munans_localization::EssentialMatrixConstraint EM_Constraint;
ros::Publisher visualFB_pub;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

bool place_recognition(munans_localization::Detect::Request &req, munans_localization::Detect::Response &res){

	// Node id of the robot's pose and the time stamp at the request
	EM_Constraint.RobotPoseID = req.RobotPoseID;
	EM_Constraint.header.stamp = ros::Time::now();
	EM_Constraint.header.frame_id = "map";

    // Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

    // std::vector<int> FBnodes = {3,8,13,18,23,28,33,38,43,48};
	std::vector<int> FBnodes = {3,18,43};

    // for (int i = 0 ; i < FBnodes.size() ; i++){
    
    PR_log.println("Feedback image at node " + std::to_string(FBnodes.at(node_id)));
    PR_log.println("-------------------------");

	std::string test_image = filePath + "/src/Image_Data/ISLAB_cropped/node(" + std::to_string(FBnodes.at(node_id)) + ").jpg";
    // std::string test_image = filePath + "/src/node17(1).JPG";
    // std::string test_image = filePath + "/src/image10.png";

	cv::Mat img_test = cv::imread(test_image, cv::IMREAD_COLOR);

	cv::resize(img_test, img_test, cv::Size(0,0), kRescaleFactor_test, kRescaleFactor_test,2);

	// Place recognition
    node_cl = munans::PlaceRecognition::Detect(img_test,filePath);

	if (node_cl.success == true){

    	PR_log.info("Place recognition successful...!");

		// Decomposing the Essential Matrix into the Rotation Matrix and Translation Vector
		Essential = munans::PlaceRecognition::computeRotationTranslation(node_cl.FundamentalMat);
		
		// Closest map node to the robots position
		EM_Constraint.MapPoseID = node_cl.ID;

		// Constructing the Essential matrix between map and robot poses
		EM_Constraint.EssentialMatrix.position.x = Essential.t_x;
		EM_Constraint.EssentialMatrix.position.y = Essential.t_y;
		EM_Constraint.EssentialMatrix.position.z = Essential.t_z;
		EM_Constraint.EssentialMatrix.orientation = tf::createQuaternionMsgFromRollPitchYaw(Essential.thx,Essential.thy,Essential.thz);

		// Printing the result
		PR_log.println("Visual feedback result");
		PR_log.println("----------------------\n");

		PR_log.print("Robot node = ");PR_log.println(std::to_string(EM_Constraint.RobotPoseID));
		PR_log.print("Map node = ");PR_log.println(std::to_string(EM_Constraint.MapPoseID));
		PR_log.print("x = ");PR_log.println(std::to_string(Essential.t_x));
		PR_log.print("y = ");PR_log.println(std::to_string(Essential.t_y));
		PR_log.print("z = ");PR_log.println(std::to_string(Essential.t_z));
		PR_log.print("thx = ");PR_log.println(std::to_string(Essential.thx));
		PR_log.print("thy = ");PR_log.println(std::to_string(Essential.thy));
		PR_log.print("thz = ");PR_log.println(std::to_string(Essential.thz));

		// Publishing the Essential Matrix Constraint messsage
		visualFB_pub.publish(EM_Constraint);
		node_id++;
	}else{
		PR_log.error("Place recognition failed..! Visual feedback rejected..!");
	};

}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"Place Recognizer");
	ros::NodeHandle placeRecog;

    ros::Rate loop_rate(loop_rate);

	visualFB_pub = placeRecog.advertise<munans_localization::EssentialMatrixConstraint>("visual_feedback", 1000);

	PR_log.info("Place Recognizer launched !");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

// 2) Importing Data
	 ///////////////

	while(ros::ok()){

		ros::ServiceServer place_recog_server = placeRecog.advertiseService("place_recog_server/request", place_recognition);
		ros::spin();
	};

	return 0;
};


