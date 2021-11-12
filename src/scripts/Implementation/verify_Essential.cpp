//============================================================================
// Name        : verify_Essential.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : A code for the experimental verification of the Essential 
//               matrix calculation using the virtual views
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
    const float ratio_thresh = 0.7;
    int no_of_features = 1500;
    bool same_camera = true;
    bool change_camera = false; //default Cam1 - Camera on the left Cam2 - Camera on the right
    std::string detector = "SIFT";
    const float scale_factor_1 = 2.0;
    const float scale_factor_2 = 0.5;
    const float visualization_scale_factor = 1;
    const double RANSAC_thresh = 0.001;
    double inliers = 0.0;
    float FOV_x = 90.00;
    float FOV_y = 90.00;

// 2.Images
    // std::string image1 = "precise/iphone1.JPG";
    // std::string image2 = "precise/iphone2.JPG";
    // std::string image1 = "precise/iphone5.JPG";
    // std::string image2 = "precise/iphone6.JPG";
    // std::string image1 = "point2/image1_120.jpg";
    // std::string image2 = "point2/image2_60.jpg";
    std::string image1 = "seekur/pose 39/rob_pose39_neg_60.png";
    std::string image2 = "seekur/pose 39/map_pose39_neg_40.jpg";    

    // std::string image2 = "image1_minusz90_fov80.jpg";
    // std::string image2 = "image2_minusz120_fov80.jpg";
    
    //Camera2
    float pixel_size = 1.22; //micrometres
    float fx2 = 3431.7; // pixels
    float fy2 = 3429.3; // 
    float cx2 = 2009.8;
    float cy2 = 1517.9;

    //Distortion coefficients
    // cv::Mat distCoeffs(0.0,0.0,0.0869,-0.0761);
    cv::Mat distCoeffs(0.0,0.0,0.0,0.0);

// 4. Relative camera poses

    // Cam1 to Cam2 (Measured)
    float tx_exp = 2.463269;  // Cam1 -> Cam2
    float ty_exp = 0.0;       // Cam1 -> Cam2
    float tz_exp = -0.022811; // Cam1 -> Cam2
    float thx_exp = 0.0;      // Cam2 -> Cam1
    float thy_exp = 30.0;     // Cam2 -> Cam1
    float thz_exp = 0.0;      // Cam2 -> Cam1

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
cv::Mat E;
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

	VE_log.info("Essential Matrix verification launched!");

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

    // 2.1 Obtaining image size
        cv::Size img_size = img1.size();
        width = img_size.width;
        height = img_size.height;

    // 2.2 Calculating focal lengths
       double fx1 = width/(2*tan(tfRadians(FOV_x/2))) * scale_factor_1; //pixels
       double fy1 = height/(2*tan(tfRadians(FOV_y/2))) * scale_factor_1; //pixels    

    // 2.3 Calculating the image center
       double cx1 = width/2 * scale_factor_1; 
       double cy1 = height/2 * scale_factor_1;

    // 2.4 Composing the Camera matrix
        K_cam1 << fx1,  0, cx1,
                  0, fy1, cy1,
                  0,  0,  1;

        VE_log.info("Intrinsic Matrix calculation complete...!");     

    // 2.1 Obtaining image size
        img_size = img2.size();
        width = img_size.width;
        height = img_size.height;

    // 2.2 Calculating focal lengths
       fx2 = width/(2*tan(tfRadians(FOV_x/2))) * scale_factor_2; //pixels
       fy2 = height/(2*tan(tfRadians(FOV_y/2))) * scale_factor_2; //pixels    

    // 2.3 Calculating the image center
       cx2 = width/2 * scale_factor_2; 
       cy2 = height/2 * scale_factor_2;

    // 2.4 Composing the Camera matrix
        K_cam2 << fx2,  0, cx2,
                  0, fy2, cy2,
                  0,  0,  1;

        VE_log.info("Intrinsic Matrix calculation complete...!");                      

    // 2.5 Choosing the proper camera matrix             

        if (same_camera == true){
            if (change_camera == false){
                K_1 = cv::Mat(K_cam1);
                K_2 = cv::Mat(K_cam1);
            }else if(change_camera == true){
                K_1 = cv::Mat(K_cam2);
                K_2 = cv::Mat(K_cam2);
            };
        }else if(same_camera == false){
            if (change_camera == false){
                K_1 = cv::Mat (K_cam1);
                K_2 = cv::Mat(K_cam2);
            }else if(change_camera == true){
                K_1 = cv::Mat(K_cam2);
                K_2 = cv::Mat(K_cam1);
            }; 
        };

// 4.Experimental calculation
// //////////////////////////

    // 3.1 Translation vector

        t_exp.setX(tx_exp);
        t_exp.setY(ty_exp);
        t_exp.setZ(tz_exp);

        t_exp.normalize(); 

        // Normalized values
        tx_exp = t_exp.getX();
        ty_exp = t_exp.getY();
        tz_exp = t_exp.getZ();

    // 3.2 Rotation matrix
        R_exp.setRPY(tfRadians(thx_exp),tfRadians(thy_exp),tfRadians(thz_exp));
        t_exp_cross.setValue(0,-t_exp.getZ(),t_exp.getY(),t_exp.getZ(),0,-t_exp.getX(),-t_exp.getY(),t_exp.getX(),0);

    // 3.3 Calculating the Essential Matrix
        E_exp = R_exp.operator*= (t_exp_cross);
        cv::Matx33d E_temp(E_exp.operator[](0).getX(),E_exp.operator[](0).getY(),E_exp.operator[](0).getZ(),
                           E_exp.operator[](1).getX(),E_exp.operator[](1).getY(),E_exp.operator[](1).getZ(),
                           E_exp.operator[](2).getX(),E_exp.operator[](2).getY(),E_exp.operator[](2).getZ());

        cv::Matx33d E_Mat_exp(E_temp);

        VE_log.info("Experimental Essential Matrix calculation complete...!");
        
// 5. Undistorting images
// //////////////////////

    // Image 1
    cv::Mat imgUn1;
    cv::undistort(img1, imgUn1, K_1, distCoeffs);

    // Image 2
    cv::Mat imgUn2;
    cv::undistort(img2, imgUn2, K_2, distCoeffs);

    VE_log.info("Image undistortion complete...!");

// 6.Calculating using images
// //////////////////////////

	// 6.1 Extracting features

    if (detector == "SIFT"){

        features1 = munans::PlaceRecognition::getSIFTdes(imgUn1,no_of_features);
        features2 = munans::PlaceRecognition::getSIFTdes(imgUn2,no_of_features);

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


	// 6.2 k-NN match descriptors
        matcher.knnMatch(features1.des, features2.des, knn_matches, 2);
        VE_log.info("Feature matching complete...!");

	// 6.3 Filter matches using the Lowe's ratio test
        for (int i = 0; i <= knn_matches.size()-1; i++) {
            if (knn_matches.at(i).at(0).distance < ratio_thresh * knn_matches.at(i).at(1).distance) {
                good_matches.push_back(knn_matches.at(i).at(0));
                src_pts.push_back(features1.kp.at(knn_matches.at(i).at(0).queryIdx).pt);
                dst_pts.push_back(features2.kp.at(knn_matches.at(i).at(0).trainIdx).pt);
            };
        };
        VE_log.info("Ratio test complete...!");
    
    // 6.4 Calculating the Fundamental Matrix
        if (same_camera == true){

            // Normalizing image points (K1 = K2)
            cv::undistortPoints(src_pts,src_norm,K_1,distCoeffs,noArray(),noArray());
            cv::undistortPoints(dst_pts,dst_norm,K_2,distCoeffs,noArray(),noArray());
   
            E = cv::findEssentialMat(src_norm, dst_norm,cameraMatrix, RANSAC, 0.99, RANSAC_thresh, match_mask);

        }else if (same_camera == false){

            // Normalizing image points (K1 != K2)
            cv::undistortPoints(imgUn1,src_norm,K_1,distCoeffs,noArray(),noArray());        
            cv::undistortPoints(imgUn2,dst_norm,K_2,distCoeffs,noArray(),noArray());        
            
            E = cv::findFundamentalMat(src_norm, dst_norm, RANSAC, RANSAC_thresh, 0.99, match_mask); 
        };

        if (E.empty() == false){
            F_cal = E;
        };

    // 6.5 Calculating the Essential Matrix
        if (same_camera == true){
            E_cal = F_cal  ;
        }else if (same_camera == false){
            E_cal = (K_2.t() * F_cal * K_1);
        };

        VE_log.info("Essential Matrix calculation complete...!");

    // 6.6 Calculating the no. of inliers
        for(int i = 0; i < match_mask.total(); i++){
            if ((match_mask.at<int>(i) > 0)){
                inliers +=1;
            };
        };

        double inlier_ratio = (inliers/match_mask.rows);

        VE_log.info("Inlier calculation complete...!");

    // 6.7 Decomposing the Essential Matrix
        cv::recoverPose(E_cal,src_norm,dst_norm,cameraMatrix,R,trans_vec,match_mask);

        // 6.7.1 Translation vector
            tx_cal = trans_vec.at<float>(0);
            ty_cal = trans_vec.at<float>(1);
            tz_cal = trans_vec.at<float>(2);

        // 6.7.2 Converting the Rotation matrix into Euler angles
            Matx33f RM(R);
            R_cal.setValue(RM.val[0],RM.val[1],RM.val[2],
                           RM.val[3],RM.val[4],RM.val[5],
                           RM.val[6],RM.val[7],RM.val[8]);

            R_cal.getRPY(thx_cal,thy_cal,thz_cal);   

            VE_log.info("Pose recovery complete...!");
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
        inliersOutliers, cv::Scalar(0,255,0), cv::Scalar(0, 0, 255),match_mask, cv::DrawMatchesFlags::DEFAULT);
        
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

    // 7.12 calculated Essential Matrix
        VE_log.println("\nEssential Matrix form calculation");
        VE_log.println("---------------------------------\n");
        
        std::cout<< E_Mat_exp << std::endl;       

    // 7.13 Derived Essential Matrix
        VE_log.println("\nEssential Matrix form the images");
        VE_log.println("-----------------------------------\n");

        std::cout<< E_cal << std::endl << std::endl;

    // 7.14 Relative rotation matrix
        VE_log.println("\nRelative rotation(matrix)");
        VE_log.println("-------------------------\n");

        std::cout << R << std::endl;

    // 7.15 Relative translation unit vector
        VE_log.println("\nTranslation vector(unit vector)");
        VE_log.println("----------------------------  -\n");

        std::cout << trans_vec << std::endl;

    // 7.16 Result comparison (Raw)
        VE_log.println("\nComparison");
        VE_log.println("----------\n");

        VE_log.info("t_x(measured) = " + std::to_string(tx_exp) + "\tt_x(calculated) = " + std::to_string(tx_cal));  
        VE_log.info("t_y(measured) = " + std::to_string(ty_exp) + "\tt_y(calculated) = " + std::to_string(ty_cal));  
        VE_log.info("t_z(measured) = " + std::to_string(tz_exp) + "\tt_z(calculated) = " + std::to_string(tz_cal));  
        VE_log.info("th_x(measured) = " + std::to_string(thx_exp) + "\tth_x(calculated) = " + std::to_string(tfDegrees(thx_cal)));  
        VE_log.info("th_y(measured) = " + std::to_string(thy_exp) + "\tth_y(calculated) = " + std::to_string(tfDegrees(thy_cal)));  
        VE_log.info("th_z(measured) = " + std::to_string(thz_exp) + "\tth_z(calculated) = " + std::to_string(tfDegrees(thz_cal)));  

    // 7.17 Result comparison (Refined)
        VE_log.println("\nComparison(Refined results)");
        VE_log.println("---------------------------\n");

        float norm = sqrt(pow(tx_cal,2) + pow(ty_cal,2) + pow(tz_cal,2));

        VE_log.info("t_x(measured) = " + std::to_string(tx_exp) + "\tt_x(calculated) = " + std::to_string(tx_cal/norm));  
        VE_log.info("t_y(measured) = " + std::to_string(ty_exp) + "\tt_y(calculated) = " + std::to_string(ty_cal/norm));  
        VE_log.info("t_z(measured) = " + std::to_string(tz_exp) + "\tt_z(calculated) = " + std::to_string(tz_cal/norm));  
        VE_log.info("th_x(measured) = " + std::to_string(thx_exp) + "\tth_x(calculated) = " + std::to_string(to_Degrees(thx_cal)));  
        VE_log.info("th_y(measured) = " + std::to_string(thy_exp) + "\tth_y(calculated) = " + std::to_string(to_Degrees(thy_cal)));  
        VE_log.info("th_z(measured) = " + std::to_string(thz_exp) + "\tth_z(calculated) = " + std::to_string(to_Degrees(thz_cal)));  

        VE_log.info("Essential Matrix verification complete...!");

    cv::waitKey();

    // std::cout << src_pts << std::endl;
    // std::cout << dst_pts << std::endl;
    // // std::cout << knn_matches << std::endl;
    // // std::cout << good_matches << std::endl;
    // std::cout << match_mask << std::endl;

};