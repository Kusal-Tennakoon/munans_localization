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

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

// GTSAM
#include <gtsam/geometry/Pose3.h>

#include "variables.h" // Contains all the variables related to visualization
#include "Visualizer.h"

const int covariance_ellipse_scale = 1;

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											FUNCTION DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


void Visualizer::Initialize(ros::NodeHandle nh)
{ 
	// Map node settings
	// -----------------

	// Poses
	poseMap_poses.header.frame_id = "fixed_reference";
	poseMap_poses.header.stamp = ros::Time();

	// Points
	poseMap_points.header.frame_id = "fixed_reference";
	poseMap_points.header.stamp = ros::Time();
	poseMap_points.type = visualization_msgs::Marker::SPHERE_LIST;
	poseMap_points.action = visualization_msgs::Marker::ADD;
	poseMap_points.ns = "map_points";
	poseMap_points.id = 0;
	poseMap_points.scale.x = 0.15;
	poseMap_points.scale.y = 0.15;
	poseMap_points.scale.z = 0.15;
	poseMap_points.color.a = 1.0; // Don't forget to set the alpha!
	poseMap_points.color.r = 1.0;
	poseMap_points.color.g = 1.0;
	poseMap_points.color.b = 1.0;
	poseMap_points.pose.orientation.w = 1.0; 
	poseMap_points.lifetime = ros::Duration(0);

	// Lines
	poseMap_lines.header.frame_id = "fixed_reference";
	poseMap_lines.header.stamp = ros::Time();
	poseMap_lines.type = visualization_msgs::Marker::LINE_STRIP;
	poseMap_lines.action = visualization_msgs::Marker::ADD;
	poseMap_lines.ns = "map_lines";
	poseMap_lines.id = 0;
	poseMap_lines.scale.x = 0.1;
	poseMap_lines.color.a = 0.75; // Don't forget to set the alpha!
	poseMap_lines.color.r = 1.0;
	poseMap_lines.color.g = 1.0;
	poseMap_lines.color.b = 1.0;		
	poseMap_lines.pose.orientation.w = 1.0; 
	poseMap_lines.lifetime = ros::Duration(0);

	// Robot pose settings
	// -------------------

	// Poses
	poseRob_poses.header.frame_id = "fixed_reference";
	poseRob_poses.header.stamp = ros::Time::now();

	//	Initial points settings
	// ------------------------

	// Poses
	initPoint_poses.header.frame_id = "fixed_reference";
	initPoint_poses.header.stamp = ros::Time();

	// Points
	initPoint_points.header.frame_id = "fixed_reference";
	initPoint_points.header.stamp = ros::Time();
	initPoint_points.type = visualization_msgs::Marker::SPHERE_LIST;
	initPoint_points.action = visualization_msgs::Marker::ADD;
	initPoint_points.ns = "init_points";
	initPoint_points.id = 0;
	initPoint_points.scale.x = 0.15;
	initPoint_points.scale.y = 0.15;
	initPoint_points.scale.z = 0.15;
	initPoint_points.color.a = 1.0; // Don't forget to set the alpha!
	initPoint_points.color.r = 1.0;
	initPoint_points.color.g = 1.0;
	initPoint_points.color.b = 0.0;
	initPoint_points.pose.orientation.w = 1.0; 
	initPoint_points.lifetime = ros::Duration(0);

	// Lines
	initPoint_lines.header.frame_id = "fixed_reference";
	initPoint_lines.header.stamp = ros::Time();
	initPoint_lines.type = visualization_msgs::Marker::LINE_STRIP;
	initPoint_lines.action = visualization_msgs::Marker::ADD;
	initPoint_lines.ns = "init_lines";
	initPoint_lines.id = 0;
	initPoint_lines.scale.x = 0.05;
	initPoint_lines.color.a = 1.0; // Don't forget to set the alpha!
	initPoint_lines.color.r = 1.0;
	initPoint_lines.color.g = 1.0;
	initPoint_lines.color.b = 0.0;		
	initPoint_lines.pose.orientation.w = 1.0; 
	initPoint_lines.lifetime = ros::Duration(0);

	// Robot trajectory settings
	// -------------------------

	// Poses
	trajectory_poses.header.frame_id = "fixed_reference";
	trajectory_poses.header.stamp = ros::Time();
    robot_odometry_trans.header.frame_id = "odom";
    robot_odometry_trans.child_frame_id = "base_link";
    robot_odometry.header.frame_id = "odom";
    robot_odometry.child_frame_id = "base_link";

	// Points
	trajectory_points.header.frame_id = "fixed_reference";
	trajectory_points.header.stamp = ros::Time();
	trajectory_points.type = visualization_msgs::Marker::SPHERE_LIST;
	trajectory_points.action = visualization_msgs::Marker::ADD;
	trajectory_points.ns = "trajectory_points";
	trajectory_points.id = 0;
	trajectory_points.scale.x = 0.15;
	trajectory_points.scale.y = 0.15;
	trajectory_points.scale.z = 0.15;
	trajectory_points.color.a = 1.0; // Don't forget to set the alpha!
	trajectory_points.color.r = 0.0;
	trajectory_points.color.g = 1.0;
	trajectory_points.color.b = 1.0;
	trajectory_points.pose.orientation.w = 1.0; 
	trajectory_points.lifetime = ros::Duration(0);

	// Lines
	trajectory_lines.header.frame_id = "fixed_reference";
	trajectory_lines.header.stamp = ros::Time();
	trajectory_lines.type = visualization_msgs::Marker::LINE_STRIP;
	trajectory_lines.action = visualization_msgs::Marker::ADD;
	trajectory_lines.ns = "trajectory_lines";
	trajectory_lines.id = 0;
	trajectory_lines.scale.x = 0.05;
	trajectory_lines.color.a = 1.0; // Don't forget to set the alpha!
	trajectory_lines.color.r = 0.0;
	trajectory_lines.color.g = 1.0;
	trajectory_lines.color.b = 1.0;	
	trajectory_lines.pose.orientation.w = 1.0; 	
	trajectory_lines.lifetime = ros::Duration(0);

	// Optimized trajectory settings
	// -----------------------------

	// Poses
	poseOptim_poses.header.frame_id = "fixed_reference";
	poseOptim_poses.header.stamp = ros::Time();

	poseOptim_points.header.frame_id = "fixed_reference";
	poseOptim_points.header.stamp = ros::Time();
	poseOptim_points.type = visualization_msgs::Marker::SPHERE_LIST;
	poseOptim_points.action = visualization_msgs::Marker::ADD;
	poseOptim_points.ns = "optimized_points";
	poseOptim_points.id = 0;
	poseOptim_points.scale.x = 0.1;
	poseOptim_points.scale.y = 0.1;
	poseOptim_points.scale.z = 0.1;
	poseOptim_points.color.a = 1.0; // Don't forget to set the alpha!
	poseOptim_points.color.r = 0.0;
	poseOptim_points.color.g = 1.0;
	poseOptim_points.color.b = 0.0;
	poseOptim_points.pose.orientation.w = 1.0; 
	poseOptim_points.lifetime = ros::Duration(0);

	// Lines
	poseOptim_lines.header.frame_id = "fixed_reference";
	poseOptim_lines.header.stamp = ros::Time();
	poseOptim_lines.type = visualization_msgs::Marker::LINE_STRIP;
	poseOptim_lines.action = visualization_msgs::Marker::ADD;
	poseOptim_lines.ns = "optimized_lines";
	poseOptim_lines.id = 0;
	poseOptim_lines.scale.x = 0.05;
	poseOptim_lines.color.a = 1.0; // Don't forget to set the alpha!
	poseOptim_lines.color.r = 0.0;
	poseOptim_lines.color.g = 1.0;
	poseOptim_lines.color.b = 0.0;	
	poseOptim_lines.pose.orientation.w = 1.0; 	
	poseOptim_lines.lifetime = ros::Duration(0);

	// Connections
	poseOptim_connections.header.frame_id = "fixed_reference";
	poseOptim_connections.header.stamp = ros::Time();
	poseOptim_connections.type = visualization_msgs::Marker::LINE_LIST;
	poseOptim_connections.action = visualization_msgs::Marker::ADD;
	poseOptim_connections.ns = "connections";
	poseOptim_connections.id = 0;
	poseOptim_connections.scale.x = 0.01;
	poseOptim_connections.color.a = 1.0; // Don't forget to set the alpha!
	poseOptim_connections.color.r = 1.0;
	poseOptim_connections.color.g = 0.5;
	poseOptim_connections.color.b = 0.0;	
	poseOptim_connections.pose.orientation.w = 1.0; 	
	poseOptim_connections.lifetime = ros::Duration(0);

	map_poses_pub = nh.advertise<geometry_msgs::PoseArray>("map_poses", 1000);
	map_points_pub = nh.advertise<visualization_msgs::Marker>("map_points", 1000);
	map_lines_pub = nh.advertise<visualization_msgs::Marker>("map_lines", 1000);
	map_text_pub = nh.advertise<visualization_msgs::MarkerArray>("map_names", 1000);
	initial_poses_pub = nh.advertise<geometry_msgs::PoseArray>("initial_poses", 1000);
	initial_points_pub = nh.advertise<visualization_msgs::Marker>("initial_points", 1000);
	initial_lines_pub = nh.advertise<visualization_msgs::Marker>("initial_lines", 1000);
	initial_text_pub = nh.advertise<visualization_msgs::MarkerArray>("initial_names", 1000);
	rob_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_poses", 1000);
	traj_poses_pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_poses", 1000);
	traj_points_pub = nh.advertise<visualization_msgs::Marker>("trajectory_points", 1000);
	traj_lines_pub = nh.advertise<visualization_msgs::Marker>("trajectory_lines", 1000);
	traj_text_pub = nh.advertise<visualization_msgs::MarkerArray>("trajectory_names", 1000);
	robot_odometry_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
	optim_poses_pub = nh.advertise<geometry_msgs::PoseArray>("optim_poses", 1000);
	optim_points_pub = nh.advertise<visualization_msgs::Marker>("optim_points", 1000);
	optim_lines_pub = nh.advertise<visualization_msgs::Marker>("optim_lines", 1000);
	optim_connections_pub = nh.advertise<visualization_msgs::Marker>("connections", 1000);
	optim_covariance_pub = nh.advertise<visualization_msgs::MarkerArray>("optim_covariance", 1000);
	optim_text_pub = nh.advertise<visualization_msgs::MarkerArray>("optim_names", 1000);
	visual_feedback_pub = nh.advertise<visualization_msgs::MarkerArray>("visual_feedback_arrows", 1000);
	map_img_pub = nh.advertise<sensor_msgs::Image>("pano_img", 1000);
	query_img_pub = nh.advertise<sensor_msgs::Image>("query_img", 1000);

};

void Visualizer::computeCovarianceEllipsoid(Eigen::MatrixXd covar,Eigen::Vector3d& scale, Eigen::Quaterniond& orientation)
{

  Eigen::Matrix3d covariance;

  // covariance << covar(3,3),covar(3,4),covar(3,5),
  // 				covar(4,3),covar(4,4),covar(4,5),
  // 				covar(5,3),covar(5,4),covar(5,5);

  covariance << covar(0,0),covar(0,1),covar(0,2),
  				covar(1,0),covar(1,1),covar(1,2),
  				covar(2,0),covar(2,1),covar(2,2);

  Eigen::Vector3d eigenvalues(Eigen::Vector3d::Identity());
  Eigen::Matrix3d eigenvectors(Eigen::Matrix3d::Zero());

  // NOTE: The SelfAdjointEigenSolver only references the lower triangular part of the covariance matrix
  // FIXME: Should we use Eigen's pseudoEigenvectors() ?
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
  // Compute eigenvectors and eigenvalues
  if (eigensolver.info () == Eigen::Success)
  {
    eigenvalues = eigensolver.eigenvalues();
    eigenvectors = eigensolver.eigenvectors();
  }
  else
  {
    ROS_WARN_THROTTLE(1, "failed to compute eigen vectors/values for position. Is the covariance matrix correct?");
    eigenvalues = Eigen::Vector3d::Zero();      // Setting the scale to zero will hide it on the screen
    eigenvectors = Eigen::Matrix3d::Identity();
  }

  // // Be sure we have a right-handed orientation system
  // makeRightHanded(eigenvectors, eigenvalues);

  // Rotation matrix
  Eigen::Matrix3d rotMat ;

  rotMat << eigenvectors(0,0), eigenvectors(0,1), eigenvectors(0,2),
            eigenvectors(1,0), eigenvectors(1,1), eigenvectors(1,2),
            eigenvectors(2,0), eigenvectors(2,1), eigenvectors(2,2);

  // Define the rotation
  orientation = Eigen::Quaterniond(rotMat);

  // Define the scale. eigenvalues are the variances, so we take the sqrt to draw the standard deviation
  scale(0) = eigenvalues[0];
  scale(1) = eigenvalues[1];
  scale(2) = eigenvalues[2];
};

void Visualizer::collectMapNode(gtsam::Pose3 pose,int i)
{ 
	geometry_msgs::Pose poseMap;
	geometry_msgs::Point point;
	geometry_msgs::Point line;
	visualization_msgs::Marker node_Text;

	poseMap.orientation.x = pose.rotation().toQuaternion ().x(); 
	poseMap.orientation.y = pose.rotation().toQuaternion ().y(); 
	poseMap.orientation.z = pose.rotation().toQuaternion ().z();   
	poseMap.orientation.w = pose.rotation().toQuaternion ().w(); 
	poseMap.position.x = pose.translation().x(); 
	poseMap.position.y = pose.translation().y(); 
	poseMap.position.z = pose.translation().z(); 

	point.x = pose.translation().x(); 
	point.y = pose.translation().y(); 
	point.z = pose.translation().z(); 

	line.x = pose.translation().x(); 
	line.y = pose.translation().y(); 
	line.z = pose.translation().z(); 

	node_Text.header.frame_id = "fixed_reference";
	node_Text.header.stamp = ros::Time();
	node_Text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	node_Text.action = visualization_msgs::Marker::ADD;
	node_Text.ns = "map_text";
	node_Text.id = i-1;
	node_Text.pose.position.x = pose.translation().x(); 
	node_Text.pose.position.y = pose.translation().y(); 
	node_Text.pose.position.z = pose.translation().z()-0.5; 	
	node_Text.text = std::to_string(i);
	node_Text.scale.z = 0.5;
	node_Text.color.a = 1.0; // Don't forget to set the alpha!
	node_Text.color.r = 1.0;
	node_Text.color.g = 1.0;
	node_Text.color.b = 1.0;
	node_Text.lifetime = ros::Duration(0);

	poseMap_poses.poses.push_back(poseMap);
	poseMap_points.points.push_back(point);
	poseMap_lines.points.push_back(line);
	poseMap_text.markers.push_back(node_Text);
};

void Visualizer::publishRobotPose(gtsam::Pose3 pose,ros::Time current_time)
{ 
	// robot_odometry.header.stamp = current_time;

    // //set the position
    // robot_odometry.pose.pose.position.x = pose.translation().x();
    // robot_odometry.pose.pose.position.y = pose.translation().y();
    // robot_odometry.pose.pose.position.z = pose.translation().z();
    // robot_odometry.pose.pose.orientation.x = pose.rotation().toQuaternion ().x();
    // robot_odometry.pose.pose.orientation.y = pose.rotation().toQuaternion ().y();
    // robot_odometry.pose.pose.orientation.z = pose.rotation().toQuaternion ().z();
    // robot_odometry.pose.pose.orientation.w = pose.rotation().toQuaternion ().w();

	// robot_odometry_pub.publish(robot_odometry);

	//set the position
	poseRob_poses.header.stamp = current_time;
    poseRob_poses.pose.position.x = pose.translation().x();
    poseRob_poses.pose.position.y = pose.translation().y();
    poseRob_poses.pose.position.z = pose.translation().z();
    poseRob_poses.pose.orientation.x = pose.rotation().toQuaternion ().x();
    poseRob_poses.pose.orientation.y = pose.rotation().toQuaternion ().y();
    poseRob_poses.pose.orientation.z = pose.rotation().toQuaternion ().z();
    poseRob_poses.pose.orientation.w = pose.rotation().toQuaternion ().w();

	rob_pose_pub.publish(poseRob_poses);
};

void Visualizer::collectInitialPose(gtsam::Pose3 pose,int i)
{ 
	geometry_msgs::Pose poseInit;
	geometry_msgs::Point point;
	geometry_msgs::Point line;
	visualization_msgs::Marker node_Text;

	poseInit.orientation.x = pose.rotation().toQuaternion ().x(); 
	poseInit.orientation.y = pose.rotation().toQuaternion ().y(); 
	poseInit.orientation.z = pose.rotation().toQuaternion ().z();   
	poseInit.orientation.w = pose.rotation().toQuaternion ().w(); 
	poseInit.position.x = pose.translation().x(); 
	poseInit.position.y = pose.translation().y(); 
	poseInit.position.z = pose.translation().z(); 

	point.x = pose.translation().x(); 
	point.y = pose.translation().y(); 
	point.z = pose.translation().z(); 

	line.x = pose.translation().x(); 
	line.y = pose.translation().y(); 
	line.z = pose.translation().z(); 

	node_Text.header.frame_id = "fixed_reference";
	node_Text.header.stamp = ros::Time();
	node_Text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	node_Text.action = visualization_msgs::Marker::ADD;
	node_Text.ns = "initial_text";
	node_Text.id = i-1;
	node_Text.pose.position.x = pose.translation().x(); 
	node_Text.pose.position.y = pose.translation().y(); 
	node_Text.pose.position.z = pose.translation().z()-0.5; 	
	node_Text.text = std::to_string(i);
	node_Text.scale.z = 0.5;
	node_Text.color.a = 1.0; // Don't forget to set the alpha!
	node_Text.color.r = 1.0;
	node_Text.color.g = 1.0;
	node_Text.color.b = 1.0;
	node_Text.lifetime = ros::Duration(0);

	initPoint_poses.poses.push_back(poseInit);
	initPoint_points.points.push_back(point);
	initPoint_lines.points.push_back(line);
	initPoint_text.markers.push_back(node_Text);
};

void Visualizer::collectTrajectoryPose(gtsam::Pose3 pose,int i)
{ 
	geometry_msgs::Pose poseTraj;
	geometry_msgs::Point point;
	geometry_msgs::Point line;
	visualization_msgs::Marker node_Text;

	poseTraj.orientation.x = pose.rotation().toQuaternion ().x(); 
	poseTraj.orientation.y = pose.rotation().toQuaternion ().y(); 
	poseTraj.orientation.z = pose.rotation().toQuaternion ().z();   
	poseTraj.orientation.w = pose.rotation().toQuaternion ().w(); 
	poseTraj.position.x = pose.translation().x(); 
	poseTraj.position.y = pose.translation().y(); 
	poseTraj.position.z = pose.translation().z(); 

	point.x = pose.translation().x(); 
	point.y = pose.translation().y(); 
	point.z = pose.translation().z(); 

	line.x = pose.translation().x(); 
	line.y = pose.translation().y(); 
	line.z = pose.translation().z(); 

	node_Text.header.frame_id = "fixed_reference";
	node_Text.header.stamp = ros::Time();
	node_Text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	node_Text.action = visualization_msgs::Marker::ADD;
	node_Text.ns = "trajectory_text";
	node_Text.id = i-1;
	node_Text.pose.position.x = pose.translation().x(); 
	node_Text.pose.position.y = pose.translation().y(); 
	node_Text.pose.position.z = pose.translation().z()-0.5; 	
	node_Text.text = std::to_string(i);
	node_Text.scale.z = 0.5;
	node_Text.color.a = 1.0; // Don't forget to set the alpha!
	node_Text.color.r = 1.0;
	node_Text.color.g = 1.0;
	node_Text.color.b = 1.0;
	node_Text.lifetime = ros::Duration(0);

	trajectory_poses.poses.push_back(poseTraj);
	trajectory_points.points.push_back(point);
	trajectory_lines.points.push_back(line);
	trajectory_text.markers.push_back(node_Text);
};

void Visualizer::collectOptimizedPose(gtsam::Pose3 pose1,gtsam::Pose3 pose2,Eigen::MatrixXd covar, int i)
{ 
	geometry_msgs::Pose poseOptim;
	geometry_msgs::Point point;
	geometry_msgs::Point line;
	geometry_msgs::Point point2;
	visualization_msgs::Marker covariance_ellipse;
	visualization_msgs::Marker node_Text;

	poseOptim.orientation.x = pose1.rotation().toQuaternion ().x(); 
	poseOptim.orientation.y = pose1.rotation().toQuaternion ().y(); 
	poseOptim.orientation.z = pose1.rotation().toQuaternion ().z();   
	poseOptim.orientation.w = pose1.rotation().toQuaternion ().w(); 
	poseOptim.position.x = pose1.translation().x(); 
	poseOptim.position.y = pose1.translation().y(); 
	poseOptim.position.z = pose1.translation().z(); 

  	point.x = pose1.translation().x(); 
	point.y = pose1.translation().y(); 
	point.z = pose1.translation().z(); 

	line.x = pose1.translation().x(); 
	line.y = pose1.translation().y(); 
	line.z = pose1.translation().z(); 

	// Connection line end
	point2.x = pose2.translation().x(); 
	point2.y = pose2.translation().y(); 
	point2.z = pose2.translation().z(); 

	// Covariance ellipses

	Eigen::Vector3d scale;
	Eigen::Quaterniond orientation;

	orientation.setIdentity();

	Visualizer::computeCovarianceEllipsoid(covar,scale,orientation);

	covariance_ellipse.header.frame_id = "fixed_reference";
	covariance_ellipse.header.stamp = ros::Time();
	covariance_ellipse.type = visualization_msgs::Marker::SPHERE;
	covariance_ellipse.action = visualization_msgs::Marker::ADD;
	covariance_ellipse.ns = "covariance_ellipses";
	covariance_ellipse.id = i-1;
	covariance_ellipse.pose.orientation.x = orientation.vec().x(); 
	covariance_ellipse.pose.orientation.y = orientation.vec().y();
	covariance_ellipse.pose.orientation.z = orientation.vec().z();
	covariance_ellipse.pose.orientation.w = orientation.w();
	covariance_ellipse.pose.position.x = pose1.translation().x(); 
	covariance_ellipse.pose.position.y = pose1.translation().y(); 
	covariance_ellipse.pose.position.z = pose1.translation().z(); 
	covariance_ellipse.scale.x = covariance_ellipse_scale*scale[0];
	covariance_ellipse.scale.y = covariance_ellipse_scale*scale[1];
	covariance_ellipse.scale.z = covariance_ellipse_scale*scale[2];
	covariance_ellipse.color.a = 0.4; // Don't forget to set the alpha!
	covariance_ellipse.color.r = 0.8;
	covariance_ellipse.color.g = 0.0;
	covariance_ellipse.color.b = 0.8;	 	
	covariance_ellipse.lifetime = ros::Duration(0);

	node_Text.header.frame_id = "fixed_reference";
	node_Text.header.stamp = ros::Time();
	node_Text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	node_Text.action = visualization_msgs::Marker::ADD;
	node_Text.ns = "optimized_text";
	node_Text.id = i-1;
	node_Text.pose.position.x = pose1.translation().x(); 
	node_Text.pose.position.y = pose1.translation().y(); 
	node_Text.pose.position.z = pose1.translation().z()-0.5; 	
	node_Text.text = std::to_string(i);
	node_Text.scale.z = 0.5;
	node_Text.color.a = 1.0; // Don't forget to set the alpha!
	node_Text.color.r = 1.0;
	node_Text.color.g = 1.0;
	node_Text.color.b = 1.0;
	node_Text.lifetime = ros::Duration(0);

	poseOptim_poses.poses.push_back(poseOptim);
	poseOptim_points.points.push_back(point);
	poseOptim_lines.points.push_back(line);
	poseOptim_connections.points.push_back(point);
	poseOptim_connections.points.push_back(point2);
	poseOptim_covarianace.markers.push_back(covariance_ellipse);
	poseOptim_text.markers.push_back(node_Text);
};

void Visualizer::collectVisualFeedback(gtsam::Pose3 pose1,gtsam::Pose3 pose2,int i)
{ 
	visualization_msgs::Marker visual_arrow;
	geometry_msgs::Point start;
	geometry_msgs::Point end;

	visual_arrow.header.frame_id = "fixed_reference";
	visual_arrow.header.stamp = ros::Time();
	visual_arrow.type = visualization_msgs::Marker::ARROW;
	visual_arrow.action = visualization_msgs::Marker::ADD;
	visual_arrow.ns = "visual_feedback_arrows";
	visual_arrow.id = i-1;
	visual_arrow.scale.x = 0.1;
	visual_arrow.scale.y = 0.3;
	visual_arrow.scale.z = 0.5;
	visual_arrow.color.a = 1.0; // Don't forget to set the alpha!
	visual_arrow.color.r = 1.0;
	visual_arrow.color.g = 0.0;
	visual_arrow.color.b = 0.0;
	visual_arrow.lifetime = ros::Duration(0);

  	start.x = pose1.translation().x(); 
	start.y = pose1.translation().y(); 
	start.z = pose1.translation().z(); 

	end.x = pose2.translation().x(); 
	end.y = pose2.translation().y(); 
	end.z = pose2.translation().z(); 

	visual_arrow.points.push_back(start);
	visual_arrow.points.push_back(end);

	visual_feedback.markers.push_back(visual_arrow);
};

void Visualizer::publishMap(){

	map_poses_pub.publish(poseMap_poses);
	map_points_pub.publish(poseMap_points);
	map_lines_pub.publish(poseMap_lines);
	map_text_pub.publish(poseMap_text);
};

void Visualizer::publishInitial(){

	initial_poses_pub.publish(initPoint_poses);
	initial_points_pub.publish(initPoint_points);
	initial_lines_pub.publish(initPoint_lines);
	initial_text_pub.publish(initPoint_text);
};

void Visualizer::publishTrajectory(){

	traj_poses_pub.publish(trajectory_poses);
	traj_points_pub.publish(trajectory_points);
	traj_lines_pub.publish(trajectory_lines);
	traj_text_pub.publish(trajectory_text);
};

void Visualizer::publishOptimized(){

	optim_poses_pub.publish(poseOptim_poses);
	optim_points_pub.publish(poseOptim_points);
	optim_lines_pub.publish(poseOptim_lines);
	optim_connections_pub.publish(poseOptim_connections);
	optim_covariance_pub.publish(poseOptim_covarianace);
	optim_text_pub.publish(poseOptim_text);
};

void Visualizer::publishVisualFeedback(){

	visual_feedback_pub.publish(visual_feedback);
};

void Visualizer::publishCameraFeed(std::string img)
{ 
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(img,CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(ImgMap);
  	map_img_pub.publish(ImgMap);
};

void Visualizer::publishCaptureImage(std::string img)
{ 
    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(img,CV_LOAD_IMAGE_COLOR);
    cv_image.encoding = "bgr8";
    cv_image.toImageMsg(Imgquery);
  	query_img_pub.publish(Imgquery);
};

void Visualizer::clearOptimizedPose()
{ 
	poseOptim_poses.poses.clear(); // Poses
	poseOptim_points.points.clear() ; // Points
	poseOptim_lines.points.clear(); //Lines
	poseOptim_connections.points.clear(); // Connection lines
	poseOptim_covarianace.markers.clear(); // Connection lines
	poseOptim_text.markers.clear(); //Point numbers array

};

void Visualizer::keepVisible(){

	while (ros::ok()){
		
		Visualizer::publishMap();
		Visualizer::publishTrajectory();
		Visualizer::publishInitial();
		Visualizer::publishOptimized();
		Visualizer::publishVisualFeedback();

		sleep(2);
	};
};