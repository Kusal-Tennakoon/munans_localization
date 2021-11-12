//============================================================================
// Name        : verify_Essential.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : A code for the experimental verification of the Essential 
//               cv::Matrix calculation using the virtual views
//============================================================================

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>

//ROS
#include <ros/ros.h>
#include <ros/package.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <sys/stat.h>

//Custom made
#include "munans.h"
#include "Log.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int board_width = 6;
int board_height = 7;
int num_imgs = 61;
float square_size = 0.05; //0.05 m
std::string img_file; 
std::string imgs_directory = "/home/umbra/catkin_ws/src/munans_localization/src/Image_Data/checkerboard";
std::string imgs_filename = "image";
std::string out_file;
std::string extension = "JPG";
int board_n = board_width * board_height;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

std::vector<std::vector<cv::Vec3f>> object_points;
std::vector<std::vector<cv::Vec2f>> image_points;
std::vector<cv::Vec2f> corners;
std::vector<std::vector<cv::Vec2f>> left_img_points;

cv::Mat img, gray;
cv::Size im_size;
cv::Mat K;
cv::Mat D;
std::vector< cv::Mat > rvecs, tvecs;
std::vector< cv::Vec3f > obj;
cv::Size board_size = cv::Size(board_width, board_height);

Log::log CC_log;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

bool doesExist (const std::string& name) {
    struct stat buffer;   
        return (stat (name.c_str(), &buffer) == 0); 
}

void setup_calibration(int board_width, int board_height, int num_imgs, 
                       float square_size, std::string imgs_directory, std::string imgs_filename,
                       std::string extension) {

    for (int k = 1; k <= num_imgs; k++) {

        // sprintf(img_file, "%s%s%d.%s", imgs_directory, imgs_filename, k, extension);

        img_file = imgs_directory + "/" + imgs_filename + std::to_string(k) + "." + extension;

        CC_log.info("Loading " + imgs_filename + std::to_string(k) + "." + extension);

        if(!doesExist(img_file)){
            continue;
        };

        img = cv::imread(img_file, CV_LOAD_IMAGE_COLOR);
        cv::cvtColor(img, gray, CV_BGR2GRAY);

        bool found = false;
        found = cv::findChessboardCorners(img, board_size, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if (found){
            cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                        cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            cv::drawChessboardCorners(gray, board_size, corners, found);
        };

        for (int i = 0; i < board_height; i++){
            for (int j = 0; j < board_width; j++){
                obj.push_back(cv::Vec3f((float)j * square_size, (float)i * square_size, 0));
            };
        };

        if (found){
            CC_log.info(imgs_filename + std::to_string(k) + " found corners!");
            image_points.push_back(corners);
            object_points.push_back(obj);
        };
    }
}

double computeReprojectionErrors(const std::vector< std::vector< cv::Point3f > >& objectPoints,
                                 const std::vector< std::vector< cv::Point2f > >& imagePoints,
                                 const std::vector< cv::Mat >& rvecs, const std::vector< cv::Mat >& tvecs,
                                 const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs) {
                                        
    std::vector< cv::Point2f > imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    std::vector< float > perViewErrors;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); ++i) {
        projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    };

    return std::sqrt(totalErr/totalPoints);
}

int main(int argc, char const **argv){

    setup_calibration(board_width, board_height, num_imgs, square_size, imgs_directory, imgs_filename, extension);

    CC_log.info("Starting Calibration!");

    int flag = 0;
    flag |= CV_CALIB_FIX_K4;
    flag |= CV_CALIB_FIX_K5;

    double cc = cv::calibrateCamera(object_points, image_points, img.size(), K, D, rvecs, tvecs, flag);
    CC_log.info("here");
    // double error = computeReprojectionErrors(object_points, image_points, rvecs, tvecs, K, D);
    
    // CC_log.info("Calibration error = " + std::to_string(error));
    CC_log.info("Calibration completed!");

    CC_log.println("Camera matrix");
    CC_log.println("-------------");

    std::cout << K << std::endl;

    CC_log.println("Distortion parameters");
    CC_log.println("---------------------");

    std::cout << D << std::endl;

    return 0;
}
