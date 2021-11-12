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
    const float scale_factor_1 = 0.25;
    const float scale_factor_2 = 0.25;
    const float visualization_scale_factor = 1;
    const double RANSAC_thresh = 3;
    double inliers = 0;
    float FOV_x = 90.00;
    float FOV_y = 90.00;



    std::string destination_dir  = "/home/lupus/MUN Drive/LUPUS/1-P.hD/1-Project/Results/Test2/";

    std::string fileName_1 = "/home/lupus/MUN Drive/LUPUS/1-P.hD/1-Project/MATLAB/Feature comparison/Node26_image_1.txt";
    std::string fileName_2 = "/home/lupus/MUN Drive/LUPUS/1-P.hD/1-Project/MATLAB/Feature comparison/Node26_image_2.txt";

// 2.Images

    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 4/rob_pose4_120.jpg";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 4/map_pose4_110.jpg";
    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 7/rob_pose7_neg_60.png";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 7/map_pose7_neg_60.jpg";
    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 11/rob_pose11_60.png";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 11/map_pose11_60.jpg";
    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 15/rob_pose15_60.png";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 15/map_pose15_40.jpg";
    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 26/rob_pose26_120.png";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 26/map_pose26_130.jpg";
    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 39/rob_pose39_neg_60.png";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/seekur/pose 39/map_pose39_neg_40.jpg";
    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/precise/iphone5.JPG";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/precise/iphone6.JPG";
    std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/iphone1.JPG";
    std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/iphone2.JPG";
    // std::string image1 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/image1_120.jpg";
    // std::string image2 = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/360/image2_60.jpg";

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
// cv::FlannBasedMatcher matcher;
cv::Ptr<cv::DescriptorMatcher> matcher;
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

// 2. Importing images
// ///////////////////

    // Image 1
    cv::Mat img1 = cv::imread(image1);
    cv::resize(img1,img1,cv::Size(),scale_factor_1,scale_factor_1);

    // Image 2
    cv::Mat img2 = cv::imread(image2);
    cv::resize(img2,img2,cv::Size(),scale_factor_2,scale_factor_2);

    VE_log.info("Image loading complete...!");

   std::ifstream file1,file2;

    file1.open(fileName_1);

	// Temporary data holders
		double feat[2]={};

	// 2.2.) Acquiring data

// 3.Calculating parameters
// ////////////////////////

// 3.2.4 Composing the Camera matrix
    K_cam2 << fx2,  0, cx2,
            0, fy2, cy2,
            0,  0,  1;

    K_2 = cv::Mat(K_cam2);

    K_1 = K_2;

    VE_log.info("Intrinsic Matrix 2 calculation complete...!");

// 4. Undistorting images
// //////////////////////

    // // Image 1
        cv::Mat imgUn1;
        cv::undistort(img1, imgUn1, K_1, distCoeffs1);

    // // Image 2
        cv::Mat imgUn2;
        cv::undistort(img2, imgUn2, K_2, distCoeffs2);

    //     VE_log.info("Image undistortion complete...!");

    // img1.copyTo(imgUn1);
    // img2.copyTo(imgUn2);

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

        cv::undistortPoints(src_pts,src_norm,K_1,distCoeffs1,noArray(),noArray());
        cv::undistortPoints(dst_pts,dst_norm,K_2,distCoeffs2,noArray(),noArray());

       F = cv::findFundamentalMat(src_pts, dst_pts, FM_RANSAC, RANSAC_thresh, 0.99, match_mask);

        if (F.empty() == false){
            F_cal = F;
        };

    // 5.5 Calculating the Essential Matrix

        E_cal = (K_2.t() * F_cal * K_1);
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

        VE_log.info("Essential Matrix calculation complete...!");

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

    VE_log.info("Printing results...\n");

        // 7.9 Image features
        // VE_log.println("\nFeature matching");
        // VE_log.println("------------------");
        VE_log.info("Detector = " + detector);
        VE_log.info("Descriptor = " + descriptor);
        VE_log.info("No. of raw feature matches = " + std::to_string(knn_matches.size()));
        VE_log.info("No. of strong feature matches = " + std::to_string(good_matches.size()));

    // 7.10 No. of inliers and inlier ratio
        // VE_log.println("\nNo. of inliers");
        // VE_log.println("----------------");

        VE_log.info("RANSAC threshold = " + std::to_string(RANSAC_thresh));
        VE_log.info("Calculated o. of inliers = " + std::to_string(int(inliers)));
        VE_log.info("Calculated inlier ratio = " + std::to_string(int(std::roundf(inlier_ratio*100))) + " %");
        VE_log.info("Calculated match threshold = " + std::to_string(P_match));

    std::vector<cv::KeyPoint> kp_src, kp_dst;
    std::vector<cv::DMatch> inlier_matches;

        // 5.6 Calculating the no. of inliers
        for(int i = 0; i < match_mask.total(); i++){
            if ((match_mask.at<double>(i)) > 0){
                kp1.push_back(cv::KeyPoint(src_pts.at(i).x,src_pts.at(i).y,step_size));
                kp2.push_back(cv::KeyPoint(dst_pts.at(i).x,dst_pts.at(i).y,step_size));
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

        cv::Mat imgUn13,imgUn23,imgInlier;
        imgUn1.copyTo(imgUn13);
        imgUn2.copyTo(imgUn23);

        // for (int i = 0 ; i < kp1.size(); i++){
        //     cv::circle(imgUn13,cv::Point(kp1.at(i).pt.x,kp1.at(i).pt.y),3,cv::Scalar(0,255,0),1.5);
        // }
        //
        // for (int i = 0 ; i < kp2.size(); i++){
        //     cv::circle(imgUn23,cv::Point(kp2.at(i).pt.x,kp2.at(i).pt.y),3,cv::Scalar(0,255,0),1.5);
        // }
        //
        // cv::hconcat(imgUn13,imgUn23,imgInlier);
        //
        // for (int i = 0 ; i < kp1.size(); i++){
        //     cv::line(imgInlier,cv::Point(kp1.at(i).pt.x,kp1.at(i).pt.y),cv::Point(kp2.at(i).pt.x+imgUn13.size().width,kp2.at(i).pt.y),cv::Scalar(0,255,0),1.75);
        // }
        // cv::line(imgUn22,dst_pts,dst_norm_new,cv::Scalar(0,255,0));

        cv::drawMatches(imgUn1,features1.kp, imgUn2,features2.kp, good_matches,
        imgInlier, cv::Scalar(0,255,0) , cv::Scalar(0, 0, 255),match_mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        cv::namedWindow("Inliers");
        cv::resize(imgInlier,imgInlier,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::imshow("Inliers",imgInlier);

        cv::imwrite(destination_dir + "Inliers_" + detector + "_"  + descriptor + ".png",imgInlier);

    cv::waitKey();

}
