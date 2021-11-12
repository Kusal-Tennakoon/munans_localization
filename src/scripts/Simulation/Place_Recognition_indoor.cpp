/*
 * Place_Recognition.cpp
 *
 *  Created on: 2019-07-24
 *      Author: umbra
 */

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
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

// GTSAM

	//non-linear
	#include <gtsam/nonlinear/NonlinearFactorGraph.h>
	#include <gtsam/nonlinear/Values.h>
	#include <gtsam/nonlinear/Marginals.h>
	#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

	//inference
	#include <gtsam/inference/Symbol.h>
	#include <gtsam/inference/Key.h>

	//geometry
	#include <gtsam/geometry/Pose3.h>
	#include <gtsam/geometry/Point3.h>
	#include <gtsam/geometry/Rot3.h>
	#include <gtsam/geometry/EssentialMatrix.h>
	#include <gtsam/geometry/Unit3.h>

	//slam
	#include <gtsam/slam/PriorFactor.h>
	#include <gtsam/slam/BetweenFactor.h>
	#include <gtsam/slam/EssentialMatrixConstraint.h>

	//sam
	#include <gtsam/sam/BearingRangeFactor.h>

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

//Parameters
//-----------

const double kNewWidth = 480;
const double kNewHeight = 360;
const double kRescaleFactor_trial = 1;
const double kRescaleFactor_test = 1;

int no_of_nodes = 47;
int no_of_query_images = 10;
int starting_image = 11;
int no_of_putatives = 10;
int max_features = 500;
int resize_factor = 1;
int hes_thresh = 12000;
int predicted_node = 0;
double verification_thresh = 0.1000;

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
munans::verificationParams verif_params;

Qmath::Vector<int> voting_array(no_of_nodes);

cv::Mat K; // Camera matrix
Log::log PR_log; //Log object

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"Indoor_place_recognition");
	ros::NodeHandle placeRecog;

	PR_log.info("Place Recognizer launched !");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

    // std::vector<int> FBnodes = {3,8,13,18,23,28,33,38,43,48};
	std::vector<int> FBnodes = {3,18,33,48};

    for (int i = 0 ; i < FBnodes.size() ; i++){
    
    PR_log.println("Feedback image at node " + std::to_string(FBnodes.at(i)));
    PR_log.println("-------------------------");

	std::string test_image = filePath + "/src/ISLAB_cropped/node(" + std::to_string(FBnodes.at(i)) + ").jpg";
    // std::string test_image = filePath + "/src/node17(1).JPG";
    // std::string test_image = filePath + "/src/image10.png";

	cv::Mat img_test = cv::imread(test_image, cv::IMREAD_COLOR);

	cv::resize(img_test, img_test, cv::Size(0,0), kRescaleFactor_test, kRescaleFactor_test,2);

    int node_cl = munans::PlaceRecognition::Detect(img_test,filePath);

    PR_log.info("Closest node = " + std::to_string(node_cl));
    }
	return 0;
};


