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

const double kNewWidth = 480;
const double kNewHeight = 360;
const double kRescaleFactor_trial = 0.2;
const double kRescaleFactor_test = 0.1;

Log::log PR_log; //Log object

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"place_recognition");
	ros::NodeHandle placeRecog;

	PR_log.info("Place Recognizer launched !");

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");
	//Parameters
	//-----------

	int no_of_nodes = 30;
	int no_of_query_images = 10;
	int starting_image = 11;
	int no_of_putatives = 5;
	int max_features = 500;
	double verification_thresh = 0.1000;
	int resize_factor = 1;
	std::string image_type = "png";
	int hes_thresh = 12000;
	std::string debug_mode = "on";
	std::string second_stage_verification = "on";

	std::string present_absent = "Absent";
	int predicted_node = 0;
	std::vector<int> presense_array(no_of_putatives, 0);
	std::vector<int> putatives(no_of_putatives, 0);
	std::vector<Qmath::Max<int>> max_node;
	std::vector<double> verif_array;
	munans::verificationParams verif_params;
	std::vector<Qmath::Max<double>> node_closest;
	cv::Mat K; // Camera matrix
	Log::log Log;

	ASiftDetector Asift ;


	Qmath::Vector<int> voting_array(no_of_nodes);


	std::string test_image = filePath + "/src/Test_images_Prince_Philip_cropped/node(17)/image4.png";
	// std::string test_image = filePath + "/src/node17(1).JPG";

	cv::Mat img_test = cv::imread(test_image, cv::IMREAD_COLOR);

	cv::resize(img_test, img_test, cv::Size(0, 0), kRescaleFactor_test, kRescaleFactor_test,2);

	munans::features brief_test; //= munans::PlaceRecognition::getSIFTdes(img_test,max_features);

	Asift.detectAndCompute(img_test,brief_test.kp,brief_test.des);

	//		print(str(len(kp_test)) + ' features detected!\n')
	//		print('Checking for matches...\n')

	// Checking for similarities

	for (int k = 0; k <= no_of_nodes-1; k++) {

		std::string trial_image = filePath + "/src/Panoramas/node_" + std::to_string(k + 1) + ".png";

		cv::Mat img_trial = cv::imread(trial_image);
		cv::resize(img_trial, img_trial, cv::Size(0, 0), kRescaleFactor_trial, kRescaleFactor_trial,2);

		munans::features brief_trial;// = munans::PlaceRecognition::getSIFTdes(img_trial, max_features);

	Asift.detectAndCompute(img_trial,brief_trial.kp,brief_trial.des);

		int no_of_matches = munans::PlaceRecognition::countMatches(brief_test.des, brief_trial.des);

		voting_array.Set(k, no_of_matches);

		Log.info("node " + std::to_string(k+1) + " checked!..." + std::to_string(no_of_matches) + " matches found!");

		//---------------------------------------------------------------------------------------------
		// Debugging

		if (debug_mode == "off") {
			continue;
		} else {
			cv::Mat debug_result = munans::Debug::debugImage(img_test,brief_test.kp, brief_test.des, img_trial, brief_trial.kp,brief_trial.des);
			munans::Debug::showImage("Debug", debug_result);
			sleep(5000);
			//				cv2.imwrite(Directory_test + Directory_test_result +"/Debug_images"+ "/node"+ str(i+1) +"_image" + str(j) + "--> node" + str(k+1) + ".png",debug_result)
		};
	};

	// ----------------------------------------------------------------------------------------------
	if (voting_array.sum() != (voting_array.Get(0) * voting_array.size())) { // If all the elements of the array are not identical
		munans::PlaceRecognition::getPutative(voting_array, no_of_putatives,putatives);

		// Qmath::Vector<int> voting_array(voting_array);

		voting_array.max(1, max_node);

		predicted_node = max_node.at(0).ID;

		Log.info("\nPutative nodes : ");

		Log::printVec(putatives);

		Log.info("\nBest matching node from voting array : " + std::to_string(predicted_node + 1));

		Log.print("");

		// Second stage verification
		// -------------------------
		if (second_stage_verification == "off") {
			//confusion_matrix[i][predicted_node] = confusion_matrix[i][predicted_node] + 1;
		} else {
			Log.info("\nSecond stage verification...\n");
			verif_array.clear();

			for (int l = 0; l <= putatives.size()-1; l++) {

				std::string putative_image = filePath + "/src/Panoramas/node_" + std::to_string(putatives.at(l)) + ".png";
				cv::Mat img_put = cv::imread(putative_image);
				cv::resize(img_put, img_put, cv::Size(0, 0), kRescaleFactor_trial, kRescaleFactor_trial,2);
				verif_params = munans::PlaceRecognition::verifyGeometric(img_test, img_put);

				Log.info("node " + std::to_string(putatives.at(l)) + " checked!... " + std::to_string(verif_params.no_of_inliers) + " inliers detected!... Similarity score = " + std::to_string(verif_params.P_match));

				if (verif_params.P_match >= verification_thresh) {

					verif_array.push_back(verif_params.P_match);
				} else {
					continue;
				};
			};
			

			if (verif_array.size() != 0) {
				Log.info("\nVerification completed!\n");
				Qmath::Vector<double> verification_array(verif_array); //Converting the list into array in order to extract the first column
				verification_array.max(1, node_closest);

				Log.info("\nClosest node : " + std::to_string(node_closest.at(0).ID));
				Log.print(" ");

				Log.print("\nVerification array\n-------------------");
				verification_array.print();

			} else {
				Log.error("\nVerification failed !");
				//confusion_matrix[i][-1] = confusion_matrix[i][-1] + 1.0
			};

			// Determining orientation
			// -----------------------

		};
	} else {

		Log.error("\nUnable to predict a matching node and putative nodes!\n");

	};

	return 0;
};


