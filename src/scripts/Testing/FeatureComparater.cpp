//=============================================================================
// Name        : EMRob2Map.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : A code for calculating the relative pose between the robot and
//               the map from two images from the robot and map cameras
//=============================================================================

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
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Scalar.h>

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
#include "Log.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

// 1.Inputs
    const int trial_no = 1;
    const int step_size = 5;

    const float ratio_thresh = 0.7;
    int no_of_features = 1500;
    std::string detector = "FAST";
    std::string descriptor = "BRIEF";
    const float scale_factor_1 = 2.0;
    const float scale_factor_2 = 0.5;
    const float visualization_scale_factor = 1;
    const double RANSAC_thresh = 3;
    double inliers = 0;
    float FOV_x = 90.00;
    float FOV_y = 90.00;

    std::string destination_dir  = "/home/lupus/MUN Drive/1-P.hD/1-Project/Results/Test2/";

    std::string fileName_1 = "/home/lupus/MUN Drive/1-P.hD/1-Project/MATLAB/Feature comparison/Node26_image_1.txt";
    std::string fileName_2 = "/home/lupus/MUN Drive/1-P.hD/1-Project/MATLAB/Feature comparison/Node26_image_2.txt";

// 2.Images

    // std::string image1 = "seekur/pose 4/rob_pose4_120.jpg";
    // std::string image2 = "seekur/pose 4/map_pose4_110.jpg";
    // std::string image1 = "seekur/pose 7/rob_pose7_neg_60.png";
    // std::string image2 = "seekur/pose 7/map_pose7_neg_60.jpg";
    // std::string image1 = "seekur/pose 11/rob_pose11_60.png";
    // std::string image2 = "seekur/pose 11/map_pose11_60.jpg";
    // std::string image1 = "seekur/pose 15/rob_pose15_60.png";
    // std::string image2 = "seekur/pose 15/map_pose15_40.jpg";
    // std::string image1 = "seekur/pose 26/rob_pose26_120.png";
    // std::string image2 = "seekur/pose 26/map_pose26_130.jpg";
    // std::string image1 = "seekur/pose 39/rob_pose39_neg_60.png";
    // std::string image2 = "seekur/pose 39/map_pose39_neg_40.jpg";
    std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/precise/iphone5.JPG";
    std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/precise/iphone6.JPG";

    //Camera2
    float pixel_size = 1.22; //micrometres
    float fx2 = 3431.7; // pixels
    float fy2 = 3429.3; //
    float cx2 = 2009.8;
    float cy2 = 1517.9;

    //Distortion coefficients
    // cv::Mat distCoeffs(0.0,0.0,0.0869,-0.0761);
    cv::Mat distCoeffs1(0.0,0.0,0.0,0.0);
    cv::Mat distCoeffs2(0.0,0.0,0.0,0.0);

    // Image size (Derived)
    float height = 0.0;
    float width = 0.0;

    float tx_cal,ty_cal,tz_cal,tx_cal2,ty_cal2,tz_cal2;
    tfScalar thx_cal,thy_cal,thz_cal,thx_cal2,thy_cal2,thz_cal2;

    std::vector<cv::KeyPoint> f_known1,f_known2;

    // std::vector<cv::KeyPoint> f_known1 = {cv::KeyPoint(74.75,114.75,float(step_size)),
    //                                     cv::KeyPoint(113.25,102.25,float(step_size)),
    //                                     cv::KeyPoint(76.75,190.75,float(step_size)),
    //                                     cv::KeyPoint(115.75,195.75,float(step_size)),
    //                                     cv::KeyPoint(36.75,117.75,float(step_size)),
    //                                     cv::KeyPoint(60.25,119.25,float(step_size)),
    //                                     cv::KeyPoint(36.25,151.25,float(step_size)),
    //                                     cv::KeyPoint(60.75,151.25,float(step_size)),
    //                                     cv::KeyPoint(134.25,178.25,float(step_size)),
    //                                     cv::KeyPoint(152.75,179.25,float(step_size)),
    //                                     cv::KeyPoint(122.75,145.75,float(step_size)),
    //                                     cv::KeyPoint(166.75,110.25,float(step_size)),
    //                                     cv::KeyPoint(165.75,148.75,float(step_size)),
    //                                     cv::KeyPoint(16.75,102.25,float(step_size)),
    //                                     cv::KeyPoint(18.25,151.75,float(step_size))
    //                                     };

    // std::vector<cv::KeyPoint> f_known2 = {cv::KeyPoint(105.25,109.25,float(step_size)),
    //                                     cv::KeyPoint(186.75,99.25,float(step_size)),
    //                                     cv::KeyPoint(103.75,217.75,float(step_size)),
    //                                     cv::KeyPoint(184.25,234.25,float(step_size)),
    //                                     cv::KeyPoint(48.75,111.75,float(step_size)),
    //                                     cv::KeyPoint(85.25,113.75,float(step_size)),
    //                                     cv::KeyPoint(46.75,163.75,float(step_size)),
    //                                     cv::KeyPoint(83.25,163.25,float(step_size)),
    //                                     cv::KeyPoint(222.25,212.25,float(step_size)),
    //                                     cv::KeyPoint(258.25,217.75,float(step_size)),
    //                                     cv::KeyPoint(202.75,161.75,float(step_size)),
    //                                     cv::KeyPoint(292.75,122.75,float(step_size)),
    //                                     cv::KeyPoint(290.75,176.75,float(step_size)),
    //                                     cv::KeyPoint(16.75,84.75,float(step_size)),
    //                                     cv::KeyPoint(17.25,160.75,float(step_size))
    //                                     };

    // std::vector<cv::KeyPoint> f_known1 = {cv::KeyPoint(75,115,float(step_size)),
    //                                     cv::KeyPoint(113,102,float(step_size)),
    //                                     cv::KeyPoint(77,191,float(step_size)),
    //                                     cv::KeyPoint(116,196,float(step_size)),
    //                                     cv::KeyPoint(37,118,float(step_size)),
    //                                     cv::KeyPoint(60,119,float(step_size)),
    //                                     cv::KeyPoint(36,151,float(step_size)),
    //                                     cv::KeyPoint(61,151,float(step_size)),
    //                                     cv::KeyPoint(134,178,float(step_size)),
    //                                     cv::KeyPoint(153,179,float(step_size)),
    //                                     cv::KeyPoint(123,146,float(step_size)),
    //                                     cv::KeyPoint(167,110,float(step_size)),
    //                                     cv::KeyPoint(166,149,float(step_size)),
    //                                     cv::KeyPoint(17,103,float(step_size)),
    //                                     cv::KeyPoint(18,152,float(step_size))
    //                                     };

    // std::vector<cv::KeyPoint> f_known2 = {cv::KeyPoint(105,109,float(step_size)),
    //                                     cv::KeyPoint(187,99,float(step_size)),
    //                                     cv::KeyPoint(104,218,float(step_size)),
    //                                     cv::KeyPoint(184,234,float(step_size)),
    //                                     cv::KeyPoint(49,112,float(step_size)),
    //                                     cv::KeyPoint(85,114,float(step_size)),
    //                                     cv::KeyPoint(47,164,float(step_size)),
    //                                     cv::KeyPoint(83,163,float(step_size)),
    //                                     cv::KeyPoint(222,212,float(step_size)),
    //                                     cv::KeyPoint(258,218,float(step_size)),
    //                                     cv::KeyPoint(203,162,float(step_size)),
    //                                     cv::KeyPoint(293,123,float(step_size)),
    //                                     cv::KeyPoint(291,177,float(step_size)),
    //                                     cv::KeyPoint(17,85,float(step_size)),
    //                                     cv::KeyPoint(17,161,float(step_size))
    //                                     };

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log VE_log; //Log object
tf::Vector3 t_exp, t_cal;
tf::Matrix3x3 t_exp_cross, t_cal_cross;
tf::Matrix3x3 R_exp, R_cal, R_cal2;
cv::Matx33d K_cam1, K_cam2, K_cam2_norm;
cv::Mat K_1, K_2;
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_32F);
cv::Mat E_cal = cv::Mat::zeros(3,3,CV_64F);
cv::Mat F_cal = cv::Mat::zeros(3,3,CV_64F);
cv::Mat E_cal2 = cv::Mat::zeros(3,3,CV_64F);
cv::Mat F_cal2 = cv::Mat::zeros(3,3,CV_64F);
cv::Mat E_cal3 = cv::Mat::zeros(3,3,CV_64F);
cv::Mat E_cal4 = cv::Mat::zeros(3,3,CV_64F);
cv::Mat match_mask;
cv::Mat F,F2;
cv::Mat R,R2, trans_vec,trans_vec2, rot_vec1;
cv::FlannBasedMatcher matcher;
tf::Matrix3x3 E_exp, F_exp;
std::vector<cv::Point2f> src_pts;
std::vector<cv::Point2f> dst_pts;
std::vector<cv::Point2f> src_norm, dst_norm;
std::vector<cv::Point2f> src_norm_new, dst_norm_new;
std::vector<std::vector<cv::DMatch>> knn_matches; // k-nearest neighbour matches
std::vector<cv::DMatch> good_matches; // features that passes the ratio test
munans::verificationParams verif_params;
munans::EssentialMatrix EssentialMat_cal;
munans::features features1;
munans::features features2;
cv::Mat E_Mat_exp;
tf::Vector3 t_12, t_23, t_34, t_14, t_41;
tf::Matrix3x3 R_12, R_23, R_34, R_14, R_41;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

float to_Degrees(float angle){
    angle = tfDegrees(angle);
    if (abs(angle) >= 360){
        angle = ((int)angle%360);
    };

    return angle;
};

double computeDeviation(tfScalar thx1,tfScalar thy1,tfScalar thz1,tfScalar thx2,tfScalar thy2,tfScalar thz2){

	double delta_x = 0,delta_y = 0,delta_z = 0,delta_th = 0;

	tf::Quaternion q_test, q_base;

    q_test.setEulerZYX(thz1-thz2,thy1-thy2,thx1-thx2);
    // q_base.setValue(thx2,thy2,thz2);

    q_test.normalize();
    // q_base.normalize();

    // delta_th = std::abs(tfDegrees(q_test.angle(q_base)));
    delta_th = std::abs(tfDegrees(q_test.getAngleShortestPath()));

	return delta_th;
};

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

	VE_log.info("Feature comparater launched!");

    std::string filePath = ros::package::getPath("munans_localization");

// 1. Forming the camera matrix
// ////////////////////////////

    // cx2 = cx2 * scale_factor_2;
    // cy2 = cy2 * scale_factor_2;
    // fx2 = fx2 * scale_factor_2;
    // fy2 = fy2 * scale_factor_2;
    // K_cam2 << fx2,  0, cx2,
    //           0, fy2, cy2,
    //           0,  0,  1;

// 2. Importing images
// ///////////////////

    // Image 1
    cv::Mat img1 = cv::imread(filePath + "/src/Image_Data/360/" + image1);
    cv::resize(img1,img1,cv::Size(),scale_factor_1,scale_factor_1);

    // Image 2
    cv::Mat img2 = cv::imread(filePath + "/src/Image_Data/360/" + image2);
    cv::resize(img2,img2,cv::Size(),scale_factor_2,scale_factor_2);

    VE_log.info("Image loading complete...!");

   std::ifstream file1,file2;

    file1.open(fileName_1);

	// Temporary data holders
		double feat[2]={};

	// 2.2.) Acquiring data

	//Map

		if (file1.is_open()){

			VE_log.info("Features data file opened successfully!\n");

			int i = 0;

			while(file1 >> feat[0] >> feat[1]){

                f_known1.push_back(cv::KeyPoint(feat[0],feat[1],float(step_size)));
				i++;
			};
		}		else{

			VE_log.error("Unable to read file1!\n");
		};

    file1.close();

    file2.open(fileName_2);

	// 2.2.) Acquiring data

	//Map

		if (file2.is_open()){

			VE_log.info("Features data file opened successfully!\n");

			int i = 0;

			while(file2 >> feat[0] >> feat[1]){

                f_known2.push_back(cv::KeyPoint(feat[0],feat[1],float(step_size)));
				i++;
			};
		}
		else{

			VE_log.error("Unable to read file2!\n");
		};

// 3.Calculating parameters
// ////////////////////////

    // 3.1 Image 1
        // 3.1.1 Obtaining image size
            cv::Size img_size = img1.size();
            width = img_size.width;
            height = img_size.height;

        // 3.1.2 Calculating focal lengthscv::ORB
            double fx1 = width/(2*tan(tfRadians(FOV_x/2))); //pixels
            double fy1 = height/(2*tan(tfRadians(FOV_y/2))); //pixels

        // 3.1.3 Calculating the image center
            double cx1 = width/2;
            double cy1 = height/2;

        // 3.1.4 Composing the Camera matrix
            K_cam1 << fx1,  0, cx1,
                    0, fy1, cy1,
                    0,  0,  1;

            K_1 = cv::Mat(K_cam1);

            VE_log.info("Intrinsic Matrix 1 calculation complete...!");

    // 3.2 Image 2
        // 3.2.1 Obtaining image size
            img_size = img2.size();
            width = img_size.width;
            height = img_size.height;

        // 3.2.2 Calculating focal lengths
            fx2 = width/(2*tan(tfRadians(FOV_x/2))); //pixels
            fy2 = height/(2*tan(tfRadians(FOV_y/2))); //pixels

        // 3.2.3 Calculating the image center
            cx2 = width/2;
            cy2 = height/2;

        // 3.2.4 Composing the Camera matrix
            K_cam2 << fx2,  0, cx2,
                    0, fy2, cy2,
                    0,  0,  1;

            K_2 = cv::Mat(K_cam2);

            VE_log.info("Intrinsic Matrix 2 calculation complete...!");

// 4. Undistorting images
// //////////////////////

    // Image 1
        cv::Mat imgUn1;
        cv::undistort(img1, imgUn1, K_1, distCoeffs1);

    // Image 2
        cv::Mat imgUn2;
        cv::undistort(img2, imgUn2, K_2, distCoeffs2);

        VE_log.info("Image undistortion complete...!");

// 5.Calculating using images
// //////////////////////////

	// 5.1 Extracting feature points

    if (detector == "Dense"){

        for (int i=step_size; i<imgUn1.rows-step_size; i+=step_size)
        {
            for (int j=step_size; j<imgUn1.cols-step_size; j+=step_size)
            {
                // x,y,radius
                features1.kp.push_back(cv::KeyPoint(float(j), float(i), float(step_size)));
            }
        }

        for (int i=step_size; i<imgUn2.rows-step_size; i+=step_size)
        {
            for (int j=step_size; j<imgUn2.cols-step_size; j+=step_size)
            {
                // x,y,radius
                features2.kp.push_back(cv::KeyPoint(float(j), float(i), float(step_size)));
            }
        }

    }else if (detector == "FAST") {
        cv::FAST(imgUn1, features1.kp, 20, true);
        cv::FAST(imgUn2, features2.kp, 20, true);

    // }else if (detector == "BLOB") {
    //     Ptr<SimpleBlobDetector> blob = SimpleBlobDetector::create();
    //     blob->detect(img, kpts);
    };

    VE_log.info("Feature detection complete...!");

	// 5.1 Extracting feature descriptors

    if (descriptor == "SIFT"){

        Ptr<cv::Feature2D> sift = cv::xfeatures2d::SIFT::create();
        sift->compute(imgUn1,features1.kp,features1.des);
        sift->compute(imgUn2,features2.kp,features2.des);

    }else if(descriptor == "DenseSIFT"){

        features1 = munans::PlaceRecognition::getDenseSIFTdes(imgUn1,step_size);
        features2 = munans::PlaceRecognition::getDenseSIFTdes(imgUn2,step_size);

    }else if (descriptor == "SURF") {
        Ptr<cv::Feature2D> surf = cv::xfeatures2d::SURF::create(no_of_features);
        surf->compute(imgUn1,features1.kp,features1.des);
        surf->compute(imgUn2,features2.kp,features2.des);

    }else if (descriptor == "ORB") {
        Ptr<cv::ORB> orb = cv::ORB::create(no_of_features);
        orb->compute(imgUn1,features1.kp,features1.des);
        orb->compute(imgUn2,features2.kp,features2.des);

    }else if (descriptor == "BRISK") {
        Ptr<cv::BRISK> brisk = cv::BRISK::create(no_of_features);
        brisk->compute(imgUn1,features1.kp,features1.des);
        brisk->compute(imgUn2,features2.kp,features2.des);

    }else if (descriptor == "KAZE") {
        Ptr<cv::KAZE> kaze = cv::KAZE::create(no_of_features);
        features1.kp.clear();
        features2.kp.clear();
        kaze->detectAndCompute(imgUn1,cv::Mat(),features1.kp,features1.des);
        kaze->detectAndCompute(imgUn2,cv::Mat(),features2.kp,features2.des);

    }else if (descriptor == "AKAZE") {
        features1.kp.clear();
        features2.kp.clear();
        Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
        akaze->detectAndCompute(imgUn1,cv::noArray(),features1.kp,features1.des);
        akaze->detectAndCompute(imgUn2,cv::noArray(),features2.kp,features2.des);

    }else if (descriptor == "FREAK") {
        Ptr<cv::xfeatures2d::FREAK> freak = cv::xfeatures2d::FREAK::create(no_of_features);
        freak->compute(imgUn1,features1.kp,features1.des);
        freak->compute(imgUn2,features2.kp,features2.des);

    }else if (descriptor == "DAISY") {
        Ptr<cv::xfeatures2d::DAISY> daisy = cv::xfeatures2d::DAISY::create();
        daisy->compute(imgUn1,features1.kp,features1.des);
        daisy->compute(imgUn2,features2.kp,features2.des);

    }else if (descriptor == "BRIEF") {
        Ptr<cv::xfeatures2d::BriefDescriptorExtractor> brief = cv::xfeatures2d::BriefDescriptorExtractor::create(64);
        brief->compute(imgUn1,features1.kp,features1.des);
        brief->compute(imgUn2,features2.kp,features2.des);

    }else if (descriptor == "LUCID") {
        Ptr<cv::xfeatures2d::LUCID> lucid = cv::xfeatures2d::LUCID::create();
        lucid->compute(imgUn1,features1.kp,features1.des);
        lucid->compute(imgUn2,features2.kp,features2.des);

    }else if (descriptor == "LATCH") {
        Ptr<cv::xfeatures2d::LATCH> latch = cv::xfeatures2d::LATCH::create();
        latch->compute(imgUn1,features1.kp,features1.des);
        latch->compute(imgUn2,features2.kp,features2.des);

    }else if(descriptor == "ASIFT"){

        std::vector<cv::KeyPoint> kp_1 , kp_2;
        cv::Mat des_1, des_2;

        ASiftDetector Asift ;
        Asift.detectAndCompute(img1,kp_1,des_1);
        Asift.detectAndCompute(img2,kp_2,des_2);

        features1.kp = kp_1;
        features1.des = des_1;
        features2.kp = kp_2;
        features2.des = des_2;
    };


    VE_log.info("Feature descriptor extraction complete...!");

    if (detector == "BRIEF") {
      matcher = cv::BFMatcher::create(NORM_HAMMING,true);
    }else{
      matcher = cv::BFMatcher::create(NORM_L2,true);
    };

    // features1.des.convertTo(features1.des, CV_32F);
    // features2.des.convertTo(features2.des, CV_32F);

	// 5.2 k-NN match descriptors
        matcher->match(features1.des, features2.des, good_matches);
        VE_log.info("Feature matching complete...!");

	// 5.3 Filter matches using the Lowe's ratio test
        // for (int i = 0; i <= knn_matches.size()-1; i++) {
        //     if (knn_matches.at(i).at(0).distance < ratio_thresh * knn_matches.at(i).at(1).distance) {
        //         good_matches.push_back(knn_matches.at(i).at(0));
        //         src_pts.push_back(features1.kp.at(knn_matches.at(i).at(0).queryIdx).pt);
        //         dst_pts.push_back(features2.kp.at(knn_matches.at(i).at(0).trainIdx).pt);
        //     };
        // };

  // 5.3 Filter matches using the Lowe's ratio test
        for (int i = 0; i <= good_matches.size()-1; i++) {
            src_pts.push_back(features1.kp.at(good_matches.at(i).queryIdx).pt);
            dst_pts.push_back(features2.kp.at(good_matches.at(i).trainIdx).pt);
        };
        VE_log.info("Ratio test complete...!");

    // 5.4 Calculating the Fundamental Matrix

        cv::Mat imgUn1_gray, imgUn2_gray;
        cv::cvtColor(imgUn1,imgUn1_gray,cv::COLOR_RGB2GRAY,0); // converting to grayscale
        cv::cvtColor(imgUn2,imgUn2_gray,cv::COLOR_RGB2GRAY,0); // converting to grayscale

        // Normalizing image points (K1 != K2)
        std::vector<cv::Point2f> src_pts2,dst_pts2,src_norm2,dst_norm2;
        for (int i=0;i<f_known1.size();i++){
            src_pts2.push_back(f_known1.at(i).pt);
            dst_pts2.push_back(f_known2.at(i).pt);
        }
        cv::undistortPoints(src_pts,src_norm,K_1,distCoeffs1,noArray(),noArray());
        cv::undistortPoints(dst_pts,dst_norm,K_2,distCoeffs2,noArray(),noArray());
        cv::undistortPoints(src_pts2,src_norm2,K_1,distCoeffs1,noArray(),noArray());
        cv::undistortPoints(dst_pts2,dst_norm2,K_2,distCoeffs2,noArray(),noArray());

        F = cv::findFundamentalMat(src_norm, dst_norm, FM_RANSAC, RANSAC_thresh, 0.99, match_mask);
        F2 = cv::findFundamentalMat(src_pts2, dst_pts2,FM_8POINT);

        if (F.empty() == false){
            F_cal = F;
        };

        if (F2.empty() == false){
            F_cal2 = F2;
        };

    // 5.5 Calculating the Essential Matrix

        E_cal = (K_2.t() * F_cal * K_1);
        E_cal2 = (K_2.t() * F_cal2 * K_1);
        double beta =E_cal.at<double>(2,2);
        E_cal3.at<double>(0,0) = E_cal.at<double>(0,0) / beta;
        E_cal3.at<double>(0,1) = E_cal.at<double>(0,1) / beta;
        E_cal3.at<double>(0,2) = E_cal.at<double>(0,2) / beta;
        E_cal3.at<double>(1,0) = E_cal.at<double>(1,0) / beta;
        E_cal3.at<double>(1,1) = E_cal.at<double>(1,1) / beta;
        E_cal3.at<double>(1,2) = E_cal.at<double>(1,2) / beta;
        E_cal3.at<double>(2,0) = E_cal.at<double>(2,0) / beta;
        E_cal3.at<double>(2,1) = E_cal.at<double>(2,1) / beta;
        E_cal3.at<double>(2,2) = E_cal.at<double>(2,2) / beta;

        double beta2 =E_cal2.at<double>(2,2);
        E_cal4.at<double>(0,0) = E_cal2.at<double>(0,0) / beta2;
        E_cal4.at<double>(0,1) = E_cal2.at<double>(0,1) / beta2;
        E_cal4.at<double>(0,2) = E_cal2.at<double>(0,2) / beta2;
        E_cal4.at<double>(1,0) = E_cal2.at<double>(1,0) / beta2;
        E_cal4.at<double>(1,1) = E_cal2.at<double>(1,1) / beta2;
        E_cal4.at<double>(1,2) = E_cal2.at<double>(1,2) / beta2;
        E_cal4.at<double>(2,0) = E_cal2.at<double>(2,0) / beta2;
        E_cal4.at<double>(2,1) = E_cal2.at<double>(2,1) / beta2;
        E_cal4.at<double>(2,2) = E_cal2.at<double>(2,2) / beta2;

        VE_log.info("Essential Matrix calculation complete...!");

        // std::cout << E_cal << std::endl;
        // std::cout << E_cal2 << std::endl;
        // std::cout << E_cal3 << std::endl;
        // std::cout << beta << std::endl;

        std::vector<cv::KeyPoint> kp1,kp2;

    // 5.6 Calculating the no. of inliers
        for(int i = 0; i < match_mask.total(); i++){
            if ((match_mask.at<double>(i) > 0)){
                inliers +=1;
            };
        };

        double inlier_ratio = (inliers/good_matches.size());
        double P_match = 2*inliers/(features1.kp.size()/2+features2.kp.size()/2);

        VE_log.info("Inlier calculation complete...!");

    // 5.7 Decomposing the Essential Matrix
        cv::recoverPose(E_cal3,src_norm,dst_norm,K_1,R,trans_vec,match_mask);
        cv::recoverPose(E_cal4,src_pts2,dst_pts2,K_1,R2,trans_vec2);

        // 5.7.1 Translation vector

            tx_cal = trans_vec.at<float>(0);
            ty_cal = trans_vec.at<float>(1);
            tz_cal = trans_vec.at<float>(2);

            tx_cal2 = trans_vec2.at<float>(0);
            ty_cal2 = trans_vec2.at<float>(1);
            tz_cal2 = trans_vec2.at<float>(2);

        // 5.7.2 Converting the Rotation matrix into Euler angles
            Matx33f RM(R);
            Matx33f RM2(R2);
            R_cal.setValue(RM.val[0],RM.val[1],RM.val[2],
                           RM.val[3],RM.val[4],RM.val[5],
                           RM.val[6],RM.val[7],RM.val[8]);

            R_cal.getEulerYPR(thz_cal,thy_cal,thx_cal);

            R_cal2.setValue(RM2.val[0],RM2.val[1],RM2.val[2],
                           RM2.val[3],RM2.val[4],RM2.val[5],
                           RM2.val[6],RM2.val[7],RM2.val[8]);

            R_cal2.getEulerYPR(thz_cal2,thy_cal2,thx_cal2);

            VE_log.info("Pose recovery complete...!");


    VE_log.info("Printing results...\n");

    std::vector<cv::KeyPoint> kp_src, kp_dst;
    std::vector<cv::DMatch> inlier_matches;

    cv::correctMatches(F_cal2,src_pts,dst_pts,src_norm_new,dst_norm_new);

        // 5.6 Calculating the no. of inliers
        for(int i = 0; i < match_mask.total(); i++){
            if ((match_mask.at<double>(i)) > 0){
                kp1.push_back(cv::KeyPoint(src_pts.at(i).x,src_pts.at(i).y,step_size));
                kp2.push_back(cv::KeyPoint(dst_pts.at(i).x,dst_pts.at(i).y,step_size));
                kp_src.push_back(cv::KeyPoint(src_norm_new.at(i).x,src_norm_new.at(i).y, step_size));
                kp_dst.push_back(cv::KeyPoint(dst_norm_new.at(i).x,dst_norm_new.at(i).y, step_size));
                inlier_matches.push_back(good_matches.at(i));

            };
        };

// 7.Printing the results
// //////////////////////

    // // 7.1 Input images
    //     cv::Mat img1_viz;
    //     cv::resize(img1,img1_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
    //     cv::namedWindow("Image 1");
    //     cv::imshow("Image 1",img1_viz);

    //     cv::Mat img2_viz;
    //     cv::resize(img2,img2_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
    //     cv::namedWindow("Image 2");
    //     cv::imshow("Image 2",img2_viz);

    // // 7.2 Undistorted images
    //     cv::Mat imgUn1_viz;
    //     cv::resize(imgUn1,imgUn1_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
    //     cv::namedWindow("Image 1 undistorted");
    //     cv::imshow("Image 1 undistorted",imgUn1_viz);

    //     cv::Mat imgUn2_viz;
    //     cv::resize(imgUn2,imgUn2_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
    //     cv::namedWindow("Image 2 undistorted");
    //     cv::imshow("Image 2 undistorted",imgUn2_viz);

    // 7.3 Matching features
        cv::Mat debug_result;

        cv::drawMatches(imgUn1,features1.kp, imgUn2,features2.kp, good_matches,
        debug_result, cv::Scalar::all(-1) , cv::Scalar(0, 0, 255),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        cv::resize(debug_result,debug_result,cv::Size(),visualization_scale_factor,visualization_scale_factor);

        cv::namedWindow("Feature matches");
        cv::imshow("Feature matches",debug_result);

        cv::imwrite(destination_dir + "GoodMatches_" + detector + "_" + descriptor + ".png",debug_result);

    // 7.4 Inliers and Outliers
        cv::Mat inliersOutliers,inliersOutliers2,imgUn12,imgUn22,img_out;

        imgUn1.copyTo(imgUn12);
        imgUn2.copyTo(imgUn22);

        for (int i = 0 ; i < src_pts.size(); i++){
            cv::circle(imgUn12,cv::Point(src_pts.at(i).x,src_pts.at(i).y),2,cv::Scalar(0,255,255));
            cv::circle(imgUn12,cv::Point(src_norm_new.at(i).x,src_norm_new.at(i).y),2,cv::Scalar(0,0,255));
            cv::line(imgUn12,cv::Point(src_pts.at(i).x,src_pts.at(i).y),cv::Point(src_norm_new.at(i).x,src_norm_new.at(i).y),cv::Scalar(0,255,255));
        }

        for (int i = 0 ; i < dst_pts.size(); i++){
            cv::circle(imgUn22,cv::Point(dst_pts.at(i).x,dst_pts.at(i).y),2,cv::Scalar(0,255,255));
            cv::circle(imgUn22,cv::Point(dst_norm_new.at(i).x,dst_norm_new.at(i).y),2,cv::Scalar(0,0,255));
            cv::line(imgUn22,cv::Point(dst_pts.at(i).x,dst_pts.at(i).y),cv::Point(dst_norm_new.at(i).x,dst_norm_new.at(i).y),cv::Scalar(0,255,255));
        }
        // cv::line(imgUn22,dst_pts,dst_norm_new,cv::Scalar(0,255,0));

        cv::hconcat(imgUn12,imgUn22,img_out);
        cv::namedWindow("Point shift");
        cv::resize(img_out,img_out,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::imshow("Point shift",img_out);

        cv::imwrite(destination_dir + "PointShift_" + detector + "_"  + descriptor + ".png",img_out);

        cv::Mat imgUn13,imgUn23,imgInlier;
        imgUn1.copyTo(imgUn13);
        imgUn2.copyTo(imgUn23);

        for (int i = 0 ; i < kp1.size(); i++){
            cv::circle(imgUn13,cv::Point(kp1.at(i).pt.x,kp1.at(i).pt.y),3,cv::Scalar(0,255,0),1.5);
        }

        for (int i = 0 ; i < kp2.size(); i++){
            cv::circle(imgUn23,cv::Point(kp2.at(i).pt.x,kp2.at(i).pt.y),3,cv::Scalar(0,255,0),1.5);
        }

        cv::hconcat(imgUn13,imgUn23,imgInlier);

        for (int i = 0 ; i < kp1.size(); i++){
            cv::line(imgInlier,cv::Point(kp1.at(i).pt.x,kp1.at(i).pt.y),cv::Point(kp2.at(i).pt.x+imgUn13.size().width,kp2.at(i).pt.y),cv::Scalar(0,255,0),1.5);
        }
        // cv::line(imgUn22,dst_pts,dst_norm_new,cv::Scalar(0,255,0));

        cv::namedWindow("Inliers");
        cv::resize(imgInlier,imgInlier,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::imshow("Inliers",imgInlier);

        cv::imwrite(destination_dir + "Inliers_" + detector + "_"  + descriptor + ".png",imgInlier);

        std::vector<cv::KeyPoint> in_actual_1,in_actual_2;

        for (int i = 0 ; i < kp1.size(); i++){
            double dist_1 = std::sqrt(std::pow(kp1.at(i).pt.x - kp_src.at(i).pt.x,2) + std::pow(kp1.at(i).pt.y - kp_src.at(i).pt.y,2));
            double dist_2 = std::sqrt(std::pow(kp2.at(i).pt.x - kp_dst.at(i).pt.x,2) + std::pow(kp2.at(i).pt.y - kp_dst.at(i).pt.y,2));
            if ((dist_1 <= 2.5) && (dist_2 <= 2.5)){
                in_actual_1.push_back(kp1.at(i));
                in_actual_2.push_back(kp2.at(i));
            }
        }

        double inliers_actual = in_actual_1.size();
        double inlier_ratio_actual = inliers_actual/good_matches.size();
        double P_match_actual = 2*inliers_actual/(features1.kp.size()/2+features2.kp.size()/2);

        cv::Mat imgUn14,imgUn24,imgInlierActual;
        imgUn1.copyTo(imgUn14);
        imgUn2.copyTo(imgUn24);

        for (int i = 0 ; i < in_actual_1.size(); i++){
            cv::circle(imgUn14,cv::Point(in_actual_1.at(i).pt.x,in_actual_1.at(i).pt.y),3,cv::Scalar(0,255,0),1.5);
        }

        for (int i = 0 ; i < in_actual_2.size(); i++){
            cv::circle(imgUn24,cv::Point(in_actual_2.at(i).pt.x,in_actual_2.at(i).pt.y),3,cv::Scalar(0,255,0),1.5);
        }

        cv::hconcat(imgUn14,imgUn24,imgInlierActual);

        for (int i = 0 ; i < in_actual_1.size(); i++){
            cv::line(imgInlierActual,cv::Point(in_actual_1.at(i).pt.x,in_actual_1.at(i).pt.y),cv::Point(in_actual_2.at(i).pt.x+imgUn14.size().width,in_actual_2.at(i).pt.y),cv::Scalar(0,255,0),1.5);
        }
        // cv::line(imgUn22,dst_pts,dst_norm_new,cv::Scalar(0,255,0));

        cv::namedWindow("Inliers actual");
        cv::resize(imgInlierActual,imgInlierActual,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::imshow("Inliers actual",imgInlierActual);
        cv::imwrite(destination_dir + "Inliers_actual_" + detector + "_"  + descriptor + ".png",imgInlierActual);

        // cv::drawMatches(imgUn12,features1.kp, imgUn22,features2.kp, good_matches,
        // inliersOutliers, cv::Scalar(0,255,0), cv::Scalar(0,0,255),match_mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        // cv::namedWindow("Inliers and Outliers");
        // cv::imshow("Inliers and Outliers",inliersOutliers);
        // cv::imwrite("InliersOutliers" + descriptor + ".png",inliersOutliers);

        // cv::namedWindow("Image 1");
        // cv::imshow("Image 1",imgUn12);

        // cv::namedWindow("Image 2");
        // cv::imshow("Image 2",imgUn22);

        // cv::drawKeypoints(imgUn1,kp1,inliersOutliers2,cv::Scalar(0,255,0),cv::DrawMatchesFlags::DEFAULT);
        // cv::drawKeypoints(imgUn1,kp_src,inliersOutliers2,cv::Scalar(0,0,255),cv::DrawMatchesFlags::DEFAULT);

        // cv::drawMatches(imgUn12, kp_src, imgUn22,kp_dst,inlier_matches,inliersOutliers2);

        // cv::namedWindow("Inliers and Outliers2");
        // cv::imshow("Inliers and Outliers2",inliersOutliers2);

        // cv::resize(inliersOutliers,inliersOutliers,cv::Size(),visualization_scale_factor,visualization_scale_factor);

    // 7.5 Image size
        // VE_log.println("\nImage size");
        // VE_log.println("----------\n");

        // VE_log.info("Height = " + std::to_string(height));
        // VE_log.info("Width = " + std::to_string(width));

    // // 7.6 Field of Views
    //     VE_log.println("\nField of Views");
    //     VE_log.println("--------------\n");

    //     VE_log.info("FOV_x  = " + std::to_string(FOV_x) + " degrees");
    //     VE_log.info("FOV_y = " + std::to_string(FOV_y) + " degrees");

    // // 7.7 Focal lengths
    //     VE_log.println("\nFocal Lengths");
    //     VE_log.println("-------------\n");

    //     VE_log.info("fx1 = " + std::to_string(fx1) + " px");
    //     VE_log.info("fy1 = " + std::to_string(fy1) + " px");
    //     VE_log.info("fx2 = " + std::to_string(fx2) + " px");
    //     VE_log.info("fy2 = " + std::to_string(fy2) + " px");

    // // 7.8 Camera matrices
    //     VE_log.println("\nCamera 1 camera Matrix");
    //     VE_log.println("--------------------------\n");

    //     std::cout<< K_1 << std::endl;

    //     VE_log.println("\nCamera 2 camera Matrix");
    //     VE_log.println("----------------------\n");

    //     std::cout << K_2 << std::endl;

    // 7.9 Image features
        // VE_log.println("\nFeature matching");
        // VE_log.println("------------------");
        VE_log.info("Descriptor type = " + descriptor);
        VE_log.info("No. of raw feature matches = " + std::to_string(knn_matches.size()));
        VE_log.info("No. of strong feature matches = " + std::to_string(good_matches.size()));

    // 7.10 No. of inliers and inlier ratio
        // VE_log.println("\nNo. of inliers");
        // VE_log.println("----------------");

        VE_log.info("RANSAC threshold = " + std::to_string(RANSAC_thresh));
        VE_log.info("Calculated o. of inliers = " + std::to_string(int(inliers)));
        VE_log.info("Calculated inlier ratio = " + std::to_string(int(std::roundf(inlier_ratio*100))) + " %");
        VE_log.info("Calculated match threshold = " + std::to_string(P_match));
        VE_log.info("Actual no. of inliers = " + std::to_string(int(inliers_actual)));
        VE_log.info("Actual inlier ratio = " + std::to_string(int(std::roundf(inlier_ratio_actual*100))) + " %");
        VE_log.info("Actual match threshold = " + std::to_string(P_match_actual));

    // // 7.11 Fundamental Matrix
    //     VE_log.println("\nFundamental Matrix from the images");
    //     VE_log.println("-------------------------------------\n");

    //     std::cout<< F_cal << std::endl;

    // // 7.12 Derived Essential Matrix
    //     VE_log.println("\nEssential Matrix form the images");
    //     VE_log.println("-----------------------------------\n");

    //     std::cout<< E_cal << std::endl << std::endl;

    // // 7.13 Relative rotation matrix
    //     VE_log.println("\nRelative rotation(matrix) robot cam to map cam");
    //     VE_log.println("----------------------------------------------\n");

    //     std::cout << R << std::endl;

    // // 7.14 Relative translation unit vector
    //     VE_log.println("\nTranslation vector(unit vector) robot cam to map cam");
    //     VE_log.println("----------------------------------------------------\n");

    //     std::cout << trans_vec << std::endl;

    // 7.16 Result comparison (Raw)
        // VE_log.println("\nRelative pose of the map camera w.r.t the robot camera");
        // VE_log.println("----------------------------------------\n");

        // double norm1 = std::sqrt(std::pow(tx_cal,2) + std::pow(ty_cal,2) + std::pow(tz_cal,2));
        // double norm2 = std::sqrt(std::pow(tx_cal2,2) + std::pow(ty_cal2,2) + std::pow(tz_cal2,2));

        // VE_log.info("t_x = " + std::to_string(tx_cal/norm1) + "/t_x = " + std::to_string(tx_cal2/norm2));
        // VE_log.info("t_y = " + std::to_string(ty_cal/norm1) + "/t_y = " + std::to_string(ty_cal2/norm2));
        // VE_log.info("t_z = " + std::to_string(tz_cal/norm1) + "/t_z = " + std::to_string(tz_cal2/norm2));
        // VE_log.info("th_x = " + std::to_string(tfDegrees(thx_cal)) + "/th_x = " + std::to_string(tfDegrees(thx_cal2)));
        // VE_log.info("th_y = " + std::to_string(tfDegrees(thy_cal)) + "/th_y = " + std::to_string(tfDegrees(thy_cal2)));
        // VE_log.info("th_z = " + std::to_string(tfDegrees(thz_cal)) + "/th_z = " + std::to_string(tfDegrees(thz_cal2)));

        double deviation = computeDeviation(thx_cal,thy_cal,thz_cal,thx_cal2,thy_cal2,thz_cal2);
        VE_log.info("Deviation from benchmark = " + std::to_string(deviation));

        std::vector <std::pair <int, int> > aMatches;

        for (int i = 0 ; i < f_known1.size() ; i++){

            aMatches.push_back(std::make_pair(i,i));

        } ;

       std::vector <cv::DMatch> matches;
        matches.reserve((int)aMatches.size());

        cv::Mat output;

        for (int i=0; i < (int)aMatches.size(); ++i){
            matches.push_back(cv::DMatch(aMatches[i].first, aMatches[i].second, std::numeric_limits<float>::max()));
        };

        cv::drawMatches(imgUn1,f_known1,imgUn2,f_known2,matches,output);

        cv::namedWindow("Match", cv::WINDOW_FULLSCREEN);
        // cv::setWindowProperty("Match", CV_WINDOW_FULLSCREEN, 1);
        cv::imshow("Match", output);

        cv::imwrite(destination_dir + "Benchmark.png",output);

    cv::waitKey();

    // std::cout << src_pts << std::endl;
    // std::cout << dst_pts << std::endl;
    // // std::cout << knn_matches << std::endl;
    // // std::cout << good_matches << std::endl;
    // std::cout << match_mask << std::endl;

}
