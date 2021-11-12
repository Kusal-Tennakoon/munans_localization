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

    std::vector<double> scale_factor = {1.929246, 1.929246,  1.005301,  0.525500,  0.468480,  1.197049};

    // Yaw of the robots camera
    std::vector<double> robot_yaw = {120, -60, 60, 60, 120, -60};
    
    // Yaw of the map camera
    std::vector<double> map_yaw = {110, -60, 60, 40, 130, -40};

    const float ratio_thresh = 0.7;
    int no_of_features = 1500;
    std::string detector = "SIFT";
    const float scale_factor_1 = 2.0;
    const float scale_factor_2 = 0.5;
    const float visualization_scale_factor = 1;
    const double RANSAC_thresh = 0.0001;
    double inliers = 0.0;
    float FOV_x = 90.00;
    float FOV_y = 90.00;
    const float h_tripod = 1.2;

// 2.Images

    std::string image1 = "seekur/pose 4/rob_pose4_120.jpg";
    std::string image2 = "seekur/pose 4/map_pose4_110.jpg";
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

// 4. Relative camera poses

    const float PI = 3.141593;

    // Camera frame w.r.t robot's body frame
    float t12_x = 0.5;
    float t12_y = 0.0;
    float t12_z = 0.458 - 0.250;
    float th12_x = -90.0;
    float th12_y = 0.0;
    float th12_z = robot_yaw.at(trial_no-1) - 90.0;

    // map camera frame w.r.t robot's camera frame
    float t23_x = 0.0;
    float t23_y = 0.0;
    float t23_z = 0.0;
    float th23_x = 0.0;
    float th23_y = 0.0;
    float th23_z = 0.0;

    // Map camera frame w.r.t map frame
    float t34_x = 0.0;
    float t34_y = 0.0;
    float t34_z = h_tripod;
    float th34_x = -90.0;
    float th34_y = 0.0;
    float th34_z = map_yaw.at(trial_no-1) - 90.0;

    // Robot to Map transformation
    float t14_x = 0.0;
    float t14_y = 0.0;
    float t14_z = 0.0;
    tfScalar th14_x = 0.0;
    tfScalar th14_y = 0.0;
    tfScalar th14_z = 0.0;

    // Cam1 to Cam2 (Derived)
    float tx_cal = 0.0;
    float ty_cal = 0.0;
    float tz_cal = 0.0;
    tfScalar thx_cal = 0.0;
    tfScalar thy_cal = 0.0;
    tfScalar thz_cal = 0.0;

    // Image size (Derived)
    float height = 0.0;
    float width = 0.0;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log VE_log; //Log object
tf::Vector3 t_exp, t_cal;
tf::Matrix3x3 t_exp_cross, t_cal_cross;
tf::Matrix3x3 R_exp, R_cal;
cv::Matx33d K_cam1, K_cam2, K_cam2_norm;
cv::Mat K_1, K_2;
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_32F);
cv::Mat E_cal = cv::Mat::zeros(3,3,CV_64F);
cv::Mat F_cal = cv::Mat::zeros(3,3,CV_64F);
cv::Mat match_mask;
cv::Mat F;
cv::Mat R, trans_vec, rot_vec1;
cv::FlannBasedMatcher matcher;
tf::Matrix3x3 E_exp, F_exp;
std::vector<cv::Point2f> src_pts;
std::vector<cv::Point2f> dst_pts;
std::vector<cv::Point2f> src_norm, dst_norm;
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

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

	VE_log.info("Robot to map relative pose calculation launched!");

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

// 3.Calculating parameters
// ////////////////////////

    // 3.1 Image 1
        // 3.1.1 Obtaining image size
            cv::Size img_size = img1.size();
            width = img_size.width;
            height = img_size.height;

        // 3.1.2 Calculating focal lengths
            double fx1 = width/(2*tan(tfRadians(FOV_x/2))); //pixels
            double fy1 = height/(2*tan(tfRadians(FOV_y/2))); //pixels    

        // 3.1.3 Calculating the image center
            double cx1 = width/2; 
            double cy1 = height/2;

        // 3.1.4 Composing the Camera matrix
            K_cam1 << fx1,  0, cx1,
                    0, fy1, cy1,
                    0,  0,  1;

            K_1 = cv::Mat (K_cam1);

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

	// 5.1 Extracting features

    if (detector == "SIFT"){

        features1 = munans::PlaceRecognition::getSIFTdes(imgUn1,no_of_features);
        features2 = munans::PlaceRecognition::getSIFTdes(imgUn2,no_of_features);

    }else if(detector == "DenseSIFT"){

        features1 = munans::PlaceRecognition::getDenseSIFTdes(imgUn1,step_size);
        features2 = munans::PlaceRecognition::getDenseSIFTdes(imgUn2,step_size);

    }else if(detector == "ASIFT"){

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

    VE_log.info("Feature detection complete...!");

	// 5.2 k-NN match descriptors
        matcher.knnMatch(features1.des, features2.des, knn_matches, 2);
        VE_log.info("Feature matching complete...!");

	// 5.3 Filter matches using the Lowe's ratio test
        for (int i = 0; i <= knn_matches.size()-1; i++) {
            if (knn_matches.at(i).at(0).distance < ratio_thresh * knn_matches.at(i).at(1).distance) {
                good_matches.push_back(knn_matches.at(i).at(0));
                src_pts.push_back(features1.kp.at(knn_matches.at(i).at(0).queryIdx).pt);
                dst_pts.push_back(features2.kp.at(knn_matches.at(i).at(0).trainIdx).pt);
            };
        };
        VE_log.info("Ratio test complete...!");
    
    // 5.4 Calculating the Fundamental Matrix

        cv::Mat imgUn1_gray, imgUn2_gray;
        cv::cvtColor(imgUn1,imgUn1_gray,cv::COLOR_RGB2GRAY,0); // converting to grayscale
        cv::cvtColor(imgUn2,imgUn2_gray,cv::COLOR_RGB2GRAY,0); // converting to grayscale

        // Normalizing image points (K1 != K2)
        cv::undistortPoints(src_pts,src_norm,K_1,distCoeffs1,noArray(),noArray());        
        cv::undistortPoints(dst_pts,dst_norm,K_2,distCoeffs2,noArray(),noArray());        
       
        F = cv::findFundamentalMat(src_norm, dst_norm, FM_RANSAC, RANSAC_thresh, 0.99, match_mask); 

        if (F.empty() == false){
            F_cal = F;
        };

    // 5.5 Calculating the Essential Matrix

        E_cal = (K_2.t() * F_cal * K_1);

        VE_log.info("Essential Matrix calculation complete...!");

    // 5.6 Calculating the no. of inliers
        for(int i = 0; i < match_mask.total(); i++){
            if ((match_mask.at<int>(i) > 0)){
                inliers +=1;
            };
        };

        double inlier_ratio = (inliers/match_mask.rows);

        VE_log.info("Inlier calculation complete...!");

    // 5.7 Decomposing the Essential Matrix
        cv::recoverPose(E_cal,src_norm,dst_norm,K_1,R,trans_vec,match_mask);

        // 5.7.1 Translation vector
        
            tx_cal = trans_vec.at<float>(0);
            ty_cal = trans_vec.at<float>(1);
            tz_cal = trans_vec.at<float>(2);

        // 5.7.2 Converting the Rotation matrix into Euler angles
            Matx33f RM(R);
            R_cal.setValue(RM.val[0],RM.val[1],RM.val[2],
                           RM.val[3],RM.val[4],RM.val[5],
                           RM.val[6],RM.val[7],RM.val[8]);

            R_cal.getEulerYPR(thz_cal,thy_cal,thx_cal);   

            VE_log.info("Pose recovery complete...!");
    
// 6.Computing relative pose between robot and map

    // 6.1.Camera 1 pose w.r.t to reference frame 1

        // 6.1.1.Translation vector
            t_12.setX(t12_x);
            t_12.setY(t12_y);
            t_12.setZ(t12_z);

        // 6.1.2.Rotation matrix
            R_12.setEulerYPR(tfRadians(th12_z),tfRadians(th12_y),tfRadians(th12_x));

    // 6.2.Camera 2 pose w.r.t camera 1

        // 6.2.1.Translation vector
            t_23.setX(tx_cal);
            t_23.setY(ty_cal);
            t_23.setZ(tz_cal);

            t_23.normalized();

            t_23 = t_23 * scale_factor.at(trial_no-1);;

        // 6.2.2.Rotation matrix
            R_23.setEulerYPR(thz_cal,thy_cal,thx_cal);

    // 6.3.Camera 2 pose w.r.t to reference frame 2

        // 6.3.1.Translation vector
            t_34.setX(t34_x);
            t_34.setY(t34_y);
            t_34.setZ(t34_z);

        // 6.3.2.Rotation matrix
            R_34.setEulerYPR(tfRadians(th34_z),tfRadians(th34_y),tfRadians(th34_x));

    // 6.4.Calulation

        // 6.4.1.Translation vector
            t_14 = t_12 + R_12 * t_23 - R_12 * R_23 *R_34.transpose() * t_34;

        // 6.4.2.Rotation matrix
            R_14 = R_12 * R_23 * R_34.transpose();

        // 6.4.3 Decomposing the translation vector rotation matrix

            t14_x = t_14.getX();
            t14_y = t_14.getY();
            t14_z = t_14.getZ();

            R_14.getEulerYPR(th14_z,th14_y,th14_x); 

            VE_log.info("Pose transformation complete...!");   

    
    VE_log.info("Printing results...");                 
    
// 7.Printing the results
// //////////////////////

    // 7.1 Input images
        cv::Mat img1_viz;
        cv::resize(img1,img1_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::namedWindow("Image 1");
        cv::imshow("Image 1",img1_viz);

        cv::Mat img2_viz;
        cv::resize(img2,img2_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::namedWindow("Image 2");
        cv::imshow("Image 2",img2_viz);

    // 7.2 Undistorted images
        cv::Mat imgUn1_viz;
        cv::resize(imgUn1,imgUn1_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::namedWindow("Image 1 undistorted");
        cv::imshow("Image 1 undistorted",imgUn1_viz);

        cv::Mat imgUn2_viz;
        cv::resize(imgUn2,imgUn2_viz,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        cv::namedWindow("Image 2 undistorted");
        cv::imshow("Image 2 undistorted",imgUn2_viz);

    // 7.3 Matching features
        cv::Mat debug_result;
        
        cv::drawMatches(imgUn1,features1.kp, imgUn2,features2.kp, good_matches,
        debug_result, cv::Scalar::all(-1) , cv::Scalar(0, 0, 255),std::vector<char>(), cv::DrawMatchesFlags::DEFAULT);

        cv::resize(debug_result,debug_result,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        
        cv::namedWindow("Feature matches");
        cv::imshow("Feature matches",debug_result);

    // 7.4 Inliers and Outliers
        cv::Mat inliersOutliers;

        cv::drawMatches(imgUn1,features1.kp, imgUn2,features2.kp, good_matches,
        inliersOutliers, cv::Scalar(0,255,0), cv::Scalar(0, 0, 255),match_mask, cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        
        cv::resize(inliersOutliers,inliersOutliers,cv::Size(),visualization_scale_factor,visualization_scale_factor);
        
        cv::namedWindow("Inliers and Outliers");
        cv::imshow("Inliers and Outliers",inliersOutliers);

    // 7.5 Image size
        VE_log.println("\nImage size");
        VE_log.println("----------\n");

        VE_log.info("Height = " + std::to_string(height));
        VE_log.info("Width = " + std::to_string(width));

    // 7.6 Field of Views
        VE_log.println("\nField of Views");
        VE_log.println("--------------\n");

        VE_log.info("FOV_x  = " + std::to_string(FOV_x) + " degrees");
        VE_log.info("FOV_y = " + std::to_string(FOV_y) + " degrees");

    // 7.7 Focal lengths
        VE_log.println("\nFocal Lengths");
        VE_log.println("-------------\n");

        VE_log.info("fx1 = " + std::to_string(fx1) + " px");
        VE_log.info("fy1 = " + std::to_string(fy1) + " px");
        VE_log.info("fx2 = " + std::to_string(fx2) + " px");
        VE_log.info("fy2 = " + std::to_string(fy2) + " px");

    // 7.8 Camera matrices
        VE_log.println("\nCamera 1 camera Matrix");
        VE_log.println("--------------------------\n");

        std::cout<< K_1 << std::endl;

        VE_log.println("\nCamera 2 camera Matrix");
        VE_log.println("----------------------\n");

        std::cout << K_2 << std::endl; 

    // 7.9 Image features
        VE_log.println("\nFeature matching");
        VE_log.println("------------------");

        VE_log.info("No. of raw feature matches = " + std::to_string(knn_matches.size()));    
        VE_log.info("No. of strong feature matches = " + std::to_string(good_matches.size()));

    // 7.10 No. of inliers and inlier ratio
        VE_log.println("\nNo. of inliers");
        VE_log.println("----------------");

        VE_log.info("No. of inliers = " + std::to_string(inliers));    
        VE_log.info("Inlier ratio = " + std::to_string(inlier_ratio));

    // 7.11 Fundamental Matrix
        VE_log.println("\nFundamental Matrix from the images");
        VE_log.println("-------------------------------------\n");

        std::cout<< F_cal << std::endl; 

    // 7.12 Derived Essential Matrix
        VE_log.println("\nEssential Matrix form the images");
        VE_log.println("-----------------------------------\n");

        std::cout<< E_cal << std::endl << std::endl;

    // 7.13 Relative rotation matrix
        VE_log.println("\nRelative rotation(matrix) robot cam to map cam");
        VE_log.println("----------------------------------------------\n");

        std::cout << R << std::endl;

    // 7.14 Relative translation unit vector
        VE_log.println("\nTranslation vector(unit vector) robot cam to map cam");
        VE_log.println("----------------------------------------------------\n");

        std::cout << trans_vec << std::endl;

    // 7.15 Scale factor
        VE_log.println("\nScale factor");
        VE_log.println("--------------");

        VE_log.info("Scale factor = " + std::to_string(scale_factor.at(trial_no-1)));

    // 7.16 Result comparison (Raw)
        VE_log.println("\nRelative pose of the map camera w.r.t the robot camera");
        VE_log.println("----------------------------------------\n");

        VE_log.info("t_x = " + std::to_string(tx_cal));  
        VE_log.info("t_y = " + std::to_string(ty_cal));  
        VE_log.info("t_z = " + std::to_string(tz_cal));  
        VE_log.info("th_x = " + std::to_string(tfDegrees(thx_cal)));  
        VE_log.info("th_y = " + std::to_string(tfDegrees(thy_cal)));  
        VE_log.info("th_z = " + std::to_string(tfDegrees(thz_cal)));  

    // 7.17 Result comparison (Refined)
        VE_log.println("\nRelative pose of the map w.r.t the robot(Translation upto scale)");
        VE_log.println("----------------------------------------------------------------\n");

        float norm = sqrt(pow(t14_x,2) + pow(t14_y,2) + pow(t14_z,2));

        VE_log.info("t_x = " + std::to_string(t14_x/norm));  
        VE_log.info("t_y = " + std::to_string(t14_y/norm));  
        VE_log.info("t_z = " + std::to_string(t14_z/norm));  
        VE_log.info("th_x = " + std::to_string(tfDegrees(th14_x)));  
        VE_log.info("th_y = " + std::to_string(tfDegrees(th14_y)));  
        VE_log.info("th_z = " + std::to_string(tfDegrees(th14_z)));   

        VE_log.info("Robot to map pose calculation complete...!");

    cv::waitKey();

    // std::cout << src_pts << std::endl;
    // std::cout << dst_pts << std::endl;
    // // std::cout << knn_matches << std::endl;
    // // std::cout << good_matches << std::endl;
    // std::cout << match_mask << std::endl;

};