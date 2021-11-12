//============================================================================
// Name        : publishers.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : ROS node advertising the ROS topics used in the visualizer
//============================================================================

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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include "munans_localization/EssentialMatrixConstraint.h"

#include "Log.h"

Log::log P_log; //Log object

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
	// 											PUBLISHERS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

int main(int argc, char **argv){

	ros::init(argc, argv,"Publishers");
	ros::NodeHandle publishers;

	P_log.info("Publishers launched !");

	// Advertising the topics
	//+++++++++++++++++++++++
	ros::Publisher map_poses_pub = publishers.advertise<geometry_msgs::PoseArray>("map_poses", 1000);
	ros::Publisher map_points_pub = publishers.advertise<visualization_msgs::Marker>("map_points", 1000);
	ros::Publisher map_lines_pub = publishers.advertise<visualization_msgs::Marker>("map_lines", 1000);
	ros::Publisher map_text_pub = publishers.advertise<visualization_msgs::MarkerArray>("map_names", 1000);
	ros::Publisher initial_poses_pub = publishers.advertise<geometry_msgs::PoseArray>("initial_poses", 1000);
	ros::Publisher initial_points_pub = publishers.advertise<visualization_msgs::Marker>("initial_points", 1000);
	ros::Publisher initial_lines_pub = publishers.advertise<visualization_msgs::Marker>("initial_lines", 1000);
	ros::Publisher initial_text_pub = publishers.advertise<visualization_msgs::MarkerArray>("initial_names", 1000);
	ros::Publisher rob_pose_pub = publishers.advertise<geometry_msgs::PoseStamped>("robot_poses", 1000);
	ros::Publisher traj_poses_pub = publishers.advertise<geometry_msgs::PoseArray>("trajectory_poses", 1000);
	ros::Publisher traj_points_pub = publishers.advertise<visualization_msgs::Marker>("trajectory_points", 1000);
	ros::Publisher traj_lines_pub = publishers.advertise<visualization_msgs::Marker>("trajectory_lines", 1000);
	ros::Publisher traj_text_pub = publishers.advertise<visualization_msgs::MarkerArray>("trajectory_names", 1000);
	ros::Publisher robot_odometry_pub = publishers.advertise<nav_msgs::Odometry>("odom", 1000);
	ros::Publisher optim_poses_pub = publishers.advertise<geometry_msgs::PoseArray>("optim_poses", 1000);
	ros::Publisher optim_points_pub = publishers.advertise<visualization_msgs::Marker>("optim_points", 1000);
	ros::Publisher optim_lines_pub = publishers.advertise<visualization_msgs::Marker>("optim_lines", 1000);
	ros::Publisher optim_connections_pub = publishers.advertise<visualization_msgs::Marker>("connections", 1000);
	ros::Publisher optim_covariance_pub = publishers.advertise<visualization_msgs::MarkerArray>("optim_covariance", 1000);
	ros::Publisher optim_text_pub = publishers.advertise<visualization_msgs::MarkerArray>("optim_names", 1000);
	ros::Publisher cam_poses_pub = publishers.advertise<geometry_msgs::PoseArray>("cam_poses", 1000);
	ros::Publisher cam_markers_pub = publishers.advertise<visualization_msgs::MarkerArray>("cam_markers", 1000);
	ros::Publisher cam_text_pub = publishers.advertise<visualization_msgs::MarkerArray>("cam_text", 1000);
	ros::Publisher visual_feedback_pub = publishers.advertise<visualization_msgs::MarkerArray>("visual_feedback_arrows", 1000);
	ros::Publisher map_img_pub = publishers.advertise<sensor_msgs::Image>("pano_img", 1000);
	ros::Publisher query_img_pub = publishers.advertise<sensor_msgs::Image>("query_img", 1000);
	ros::Publisher pos_err_min_pub = publishers.advertise<std_msgs::Float64>("position_error/min", 1000);
	ros::Publisher pos_err_max_pub = publishers.advertise<std_msgs::Float64>("position_error/max", 1000);
	ros::Publisher pos_err_rmse_pub = publishers.advertise<std_msgs::Float64>("position_error/RMSE", 1000);
	ros::Publisher ori_err_min_pub = publishers.advertise<std_msgs::Float64>("orientation_error/min", 1000);
	ros::Publisher ori_err_max_pub = publishers.advertise<std_msgs::Float64>("orientation_error/max", 1000);
	ros::Publisher ori_err_rmse_pub = publishers.advertise<std_msgs::Float64>("orientation_error/RMSE", 1000);
	// ros::Publisher visualFB_pub = publishers.advertise<munans_localization::EssentialMatrixConstraint>("visual_feedback", 1000);

	ros::spin();

	return 0;
};