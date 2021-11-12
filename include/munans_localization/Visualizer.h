//============================================================================
// Name        : Function_Tests.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Testing Functions
//============================================================================

#ifndef VISUALIZER_H
#define VISUALIZER_H

//ROS
#include <ros/ros.h>

//C++
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <vector>
#include <Eigen/Geometry>

// GTSAM
#include <gtsam/geometry/Pose3.h>

// Custom made
#include "munans.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											FUNCTION DECLARATIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

namespace Visualizer{

	void Initialize(ros::NodeHandle nh);
	void collectMapNode(gtsam::Pose3 pose,int i);
	void collectInitialPose(gtsam::Pose3 pose,int i);
	void collectOptimizedPose(gtsam::Pose3 pose1,gtsam::Pose3 pose2,Eigen::MatrixXd covar, int i);
	void collectCameraPose(gtsam::Pose3 pose, std::string text, int i);
	void collectTrajectoryPose(gtsam::Pose3 pose,int i);
	void collectVisualFeedback(gtsam::Pose3 pose1,gtsam::Pose3 pose2,int i);
	void publishCameraFeed(std::string img);
	void publishCaptureImage(std::string img);	
	void publishMap();
	void publishInitial();
	void publishTrajectory();
	void publishOptimized();
	void publishCameraPose();
	void publishVisualFeedback();
	void publishRobotPose(gtsam::Pose3 pose,ros::Time current_time);
	void computeCovarianceEllipsoid(Eigen::MatrixXd covar,Eigen::Vector3d& scale, Eigen::Quaterniond& orientation);
	void clearOptimizedPose();
	void publishErrorStats(munans::errorStats errorStats);
	void keepVisible();
}

#endif /* VISUALIZER_H */