//============================================================================
// Name        : affine_test.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Testing and tuning feature extraction using affineSIFT
//============================================================================

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

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
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>

#include <opencv2/objdetect.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/calib3d.hpp>
#include "ASiftDetector.h"
//Custom made
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

const double kNewWidth = 480;
const double kNewHeight = 360;
const double kRescaleFactor_trial = 0.3;
const double kRescaleFactor_test = 1;

int main(int argc, char **argv){

// 1)Initializing the ROS node
    //////////////////////////

	ros::init(argc, argv,"affine_test");
	// ros::NodeHandle nh;
	// ros::Rate loop_rate(10);
	// ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image", 10);

	// Obtaining file path to package
	std::string filePath = ros::package::getPath("munans_localization");

	// std::string trial_image = filePath + "/src/node17(1).JPG";
	std::string test_image = filePath + "/src/Image_Data/ISLAB_cropped/node(3).jpg";

	std::string trial_image = filePath + "/src/Image_Data/ISLAB/node(4).jpg";

	cv::Mat trial = cv::imread(trial_image, cv::IMREAD_COLOR);
	cv::Mat test = cv::imread(test_image, cv::IMREAD_COLOR);

	cv::resize(trial, trial, cv::Size(0, 0), kRescaleFactor_trial, kRescaleFactor_trial,2);
	cv::resize(test, test, cv::Size(0, 0), kRescaleFactor_test, kRescaleFactor_test,2);

	std::vector<cv::KeyPoint> kp_trial , kp_test;
	cv::Mat des_trial, des_test;

  	// cv::Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create(500,3,0.09,5,2);

  	// sift->detectAndCompute(trial, cv::Mat(), kp_trial, des_trial);

	ASiftDetector Asift ;
	Asift.detectAndCompute(test,kp_test,des_test);
	Asift.detectAndCompute(trial,kp_trial,des_trial);
	std::vector<cv::DMatch> matches;

	matches.clear();

	//Brute-force match descriptors
	cv::BFMatcher desc_matcher(cv::NORM_L2, true);
	desc_matcher.match(des_trial, des_test, matches, cv::Mat());

	int k =1;
	cv::Mat imgDebug;
	std::vector<cv::DMatch> matches_one_by_one;

	// for (int i=0 ; i<= k ; i++){

	// matches_one_by_one.clear();		
	// matches_one_by_one.push_back(matches.at(i));

	// cv::drawMatches(trial, kp_trial, test, kp_test,matches_one_by_one, imgDebug,cv::Scalar(0, 255, 255), cv::Scalar(0, 0, 255),std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);
	// cv::Mat imgDebug = munans::Debug::plotColouredFeatures(trial,kp_trial, 'g');

	// munans::Debug::showImage("Affine Test",imgDebug);
	// cv::waitKey();
	// if (k <= matches.size()-1){
	// 	k++;
	// }else{
	// 	break;
	// };
// };
	// sensor_msgs::Image ImgMap;
 //    cv_bridge::CvImage cv_image;
 //    cv_image.image = imgDebug;
 //    cv_image.encoding = "bgr8";
 //    cv_image.toImageMsg(ImgMap);

 //    while(ros::ok()){

 //  	pub.publish(ImgMap);
 //  	loop_rate.sleep();
	// };

	cv::drawMatches(trial, kp_trial, test, kp_test,matches, imgDebug,cv::Scalar(0, 255, 255), cv::Scalar(0, 0, 255),std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);
	// cv::Mat imgDebug = munans::Debug::plotColouredFeatures(trial,kp_trial, 'g');

	munans::Debug::showImage("Affine Test",imgDebug);
	cv::waitKey();

	return 0;
}