//===========================================================
// Name        : variables.h
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Variables used for the ROS based visualizer
//===========================================================

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
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Geometry>

// GTSAM
#include <gtsam/geometry/Pose3.h>

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											VARIABLE DECLARATIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#ifdef DEFINE_GLOBALS
#define EXTERN
#else
#define EXTERN extern
#endif


// Publisher handles
	EXTERN ros::Publisher map_poses_pub;
	EXTERN ros::Publisher map_points_pub;
	EXTERN ros::Publisher map_lines_pub;
	EXTERN ros::Publisher map_text_pub;
	EXTERN ros::Publisher rob_pose_pub;
	EXTERN ros::Publisher initial_poses_pub;
	EXTERN ros::Publisher initial_points_pub;
	EXTERN ros::Publisher initial_lines_pub;
	EXTERN ros::Publisher initial_text_pub;
	EXTERN ros::Publisher traj_poses_pub;
	EXTERN ros::Publisher traj_points_pub;
	EXTERN ros::Publisher traj_lines_pub;
	EXTERN ros::Publisher traj_text_pub;
	EXTERN ros::Publisher optim_poses_pub;
	EXTERN ros::Publisher optim_points_pub;
	EXTERN ros::Publisher optim_lines_pub;
	EXTERN ros::Publisher optim_connections_pub;
	EXTERN ros::Publisher optim_covariance_pub;
	EXTERN ros::Publisher optim_text_pub;
	EXTERN ros::Publisher cam_poses_pub;
	EXTERN ros::Publisher cam_points_pub;
	EXTERN ros::Publisher cam_text_pub;
	EXTERN ros::Publisher visual_feedback_pub;
	EXTERN ros::Publisher pos_err_min_pub;
	EXTERN ros::Publisher pos_err_max_pub;
	EXTERN ros::Publisher pos_err_rmse_pub;
	EXTERN ros::Publisher ori_err_min_pub;
	EXTERN ros::Publisher ori_err_max_pub;
	EXTERN ros::Publisher ori_err_rmse_pub;
	// EXTERN ros::Publisher map_img_pub;
	// EXTERN ros::Publisher query_img_pub;
	EXTERN image_transport::Publisher map_img_pub;
	EXTERN image_transport::Publisher query_img_pub;
	EXTERN ros::Publisher robot_odometry_pub;

// Ros messages
//+++++++++++++

	// Map poses
	//----------
	EXTERN geometry_msgs::PoseArray poseMap_poses; // Poses
	EXTERN visualization_msgs::Marker poseMap_points; // Points
	EXTERN visualization_msgs::Marker poseMap_lines; // Lines joining the points
	EXTERN visualization_msgs::MarkerArray poseMap_text; // Pose numbers array

	// Robot poses
	//----------
	EXTERN geometry_msgs::PoseStamped poseRob_poses; // Robot poses

	// Odometer points
	//---------------
	EXTERN geometry_msgs::PoseArray initPoint_poses; // Poses	
	EXTERN visualization_msgs::Marker initPoint_points; // Points
	EXTERN visualization_msgs::Marker initPoint_lines; // Lines joining the points
	EXTERN visualization_msgs::MarkerArray initPoint_text; // Point numbers array

	// Robot trajectory
	//-----------------
	EXTERN geometry_msgs::PoseArray trajectory_poses; // Poses			
	EXTERN visualization_msgs::Marker trajectory_points; // Points
	EXTERN visualization_msgs::Marker trajectory_lines; // Lines joining the points
	EXTERN visualization_msgs::MarkerArray trajectory_text; // Point numbers array
    EXTERN geometry_msgs::TransformStamped robot_odometry_trans;
    EXTERN nav_msgs::Odometry robot_odometry;

	// Optimized poses
	//----------------
	EXTERN geometry_msgs::PoseArray poseOptim_poses; // Poses
	EXTERN visualization_msgs::Marker poseOptim_points; // Points
	EXTERN visualization_msgs::Marker poseOptim_lines; //Lines
	EXTERN visualization_msgs::Marker poseOptim_connections; // Connection lines
	EXTERN visualization_msgs::MarkerArray poseOptim_covarianace; // Connection lines
	EXTERN visualization_msgs::MarkerArray poseOptim_text; //Point numbers array

	// Camera poses
	//----------------
	EXTERN geometry_msgs::PoseArray cam_poses; // Poses
	EXTERN visualization_msgs::Marker cam_points; // Points
	EXTERN visualization_msgs::MarkerArray cam_markers; // Markers (Pyramid shape)
	EXTERN visualization_msgs::MarkerArray cam_text; //Cam numbers array

	// Visual feedback
	//----------------
	EXTERN visualization_msgs::MarkerArray visual_feedback; // Array of arrows

	// Street View panorama
	//---------------------
	EXTERN sensor_msgs::Image ImgMap;

	// Camera Image
	//-------------
	EXTERN sensor_msgs::Image Imgquery;

	// Error Statistics
	//-----------------
	EXTERN std_msgs::Float64 p_min;
	EXTERN std_msgs::Float64 p_max;
	EXTERN std_msgs::Float64 p_rmse;
	EXTERN std_msgs::Float64 o_min;
	EXTERN std_msgs::Float64 o_max;
	EXTERN std_msgs::Float64 o_rmse;