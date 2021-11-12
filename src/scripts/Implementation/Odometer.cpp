//============================================================================
// Name        : Odometer.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : ROS node handling the odometry data from the robot
//============================================================================

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include "munans_localization/EssentialMatrixConstraint.h"

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

//Custom made
#include "munans.h"
#include "quickmath.h"
#include "Log.h"

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											CONTROL VARIABLES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

const int loop_rate = 100; //ROS loop rate (Hertz)
// int node_id = 1;
int feedback_id = 1;
double distance_thresh = 5.0;
double angle_thresh = 3.14/4;

std::vector<std::vector<int>> feedback{{2,2},{7,7},{12,12},{16,16},{22,22},{28,29},{35,35},{42,43}}; 

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log Odom_log; //Log object

ros::Subscriber odometer_sub;
ros::Subscriber amcl_sub;
ros::Subscriber visual_sub;
ros::Publisher corrected_path_pub;
ros::Publisher odometer_pub;
ros::Publisher initial_pub;
ros::Publisher amcl_pub;
ros::Publisher visual_feedback_pub;
ros::Publisher time_stamp_pub;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped last_pose_map;
geometry_msgs::PoseStamped current_pose_map;
geometry_msgs::PoseStamped amcl_pose_map;
geometry_msgs::PoseStamped odom_pose_map;
geometry_msgs::PoseStamped odom_pose_rob;
nav_msgs::Path robot_path;
std_msgs::Header time_stamp;
munans_localization::EssentialMatrixConstraint visualFB;

tf::Matrix3x3 RotMat_rob;
tf::Vector3 odom_rob;
tf::Quaternion q_rob;

double delta_x, delta_y, delta_z, delta_thx, delta_thy, delta_thz;
double roll_current, pitch_current, yaw_current, roll_last, pitch_last, yaw_last, distance;

bool amcl_publish = true;
bool image_publish = false;
bool visual_publish = false;
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void odometer_callback(const nav_msgs::OdometryConstPtr& odom){//const geometry_msgs::PoseWithCovarianceStampedConstPtr amcl, const nav_msgs::OdometryConstPtr& odom , const sensor_msgs::ImageConstPtr& image){
	
	// Time stamp
	time_stamp.stamp = odom ->header.stamp;
	time_stamp.frame_id = "map";

	// Odometer poses
	current_pose_map.header.frame_id = "map";
  	current_pose_map.pose.position.x = odom ->pose.pose.position.x;
	current_pose_map.pose.position.y = odom ->pose.pose.position.y;
	current_pose_map.pose.position.z = odom ->pose.pose.position.z;
	current_pose_map.pose.orientation.w = odom ->pose.pose.orientation.w;
	current_pose_map.pose.orientation.x = odom ->pose.pose.orientation.x;
	current_pose_map.pose.orientation.y = odom ->pose.pose.orientation.y;
	current_pose_map.pose.orientation.z = odom ->pose.pose.orientation.z;

	robot_path.header.stamp = ros::Time::now();
	robot_path.header.frame_id = "map";
	robot_path.poses.push_back(current_pose_map);

	corrected_path_pub.publish(robot_path);

	tf::Quaternion quat_odom(current_pose_map.pose.orientation.x,current_pose_map.pose.orientation.y,current_pose_map.pose.orientation.z,current_pose_map.pose.orientation.w);
	tf::Matrix3x3 RotMat_odom(quat_odom);
	RotMat_odom.getRPY(roll_current,pitch_current,yaw_current);

	tf::Quaternion quat_last(last_pose_map.pose.orientation.x,last_pose_map.pose.orientation.y,last_pose_map.pose.orientation.z,last_pose_map.pose.orientation.w);
	tf::Matrix3x3 RotMat_last(quat_last);
	RotMat_last.getRPY(roll_last,pitch_last,yaw_last);

	delta_x = current_pose_map.pose.position.x - last_pose_map.pose.position.x;
	delta_y = current_pose_map.pose.position.y - last_pose_map.pose.position.y;
	delta_z = current_pose_map.pose.position.z - last_pose_map.pose.position.z;
	delta_thx = roll_current - roll_last;
	delta_thy = pitch_current - pitch_last;
	delta_thz = yaw_current - yaw_last;

	distance = sqrt(pow(delta_x,2) + pow(delta_y,2) + pow(delta_z,2));

	if ((distance >= distance_thresh) || (delta_thz >= angle_thresh)){

		RotMat_rob = RotMat_last.transposeTimes(RotMat_odom);
		RotMat_rob.getRotation (q_rob);

		odom_rob.setX(RotMat_last.tdotx(tf::Vector3(delta_x,delta_y,delta_z)));
		odom_rob.setY(RotMat_last.tdoty(tf::Vector3(delta_x,delta_y,delta_z)));
		odom_rob.setZ(RotMat_last.tdotz(tf::Vector3(delta_x,delta_y,delta_z)));

		odom_pose_rob.header.frame_id = "map";
		odom_pose_rob.pose.position.x = odom_rob.getX();
		odom_pose_rob.pose.position.y = odom_rob.getY();
		odom_pose_rob.pose.position.z = odom_rob.getZ();
		odom_pose_rob.pose.orientation.w = q_rob.getW();
		odom_pose_rob.pose.orientation.x = q_rob.getX(); 
		odom_pose_rob.pose.orientation.y = q_rob.getY();
		odom_pose_rob.pose.orientation.z = q_rob.getZ();

		odometer_pub.publish(odom_pose_rob);
		initial_pub.publish(current_pose_map);
		time_stamp_pub.publish(time_stamp);
		amcl_publish = true;
		visual_publish = true;

		last_pose_map = current_pose_map;
	}

}

void amcl_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amcl){//const , const nav_msgs::OdometryConstPtr& odom , const sensor_msgs::ImageConstPtr& image){
	
	// // AMCL poses
	amcl_pose_map.header.frame_id = "map";
  	amcl_pose_map.pose.position.x = amcl ->pose.pose.position.x;
	amcl_pose_map.pose.position.y = amcl ->pose.pose.position.y;
	amcl_pose_map.pose.position.z = amcl ->pose.pose.position.z;
	amcl_pose_map.pose.orientation.w = amcl ->pose.pose.orientation.w;
	amcl_pose_map.pose.orientation.x = amcl ->pose.pose.orientation.x;
	amcl_pose_map.pose.orientation.y = amcl ->pose.pose.orientation.y;
	amcl_pose_map.pose.orientation.z = amcl ->pose.pose.orientation.z;

	if (amcl_publish == true){
		amcl_pub.publish(amcl_pose_map);
		amcl_publish = false;
	}
}

void visual_callback(const sensor_msgs::ImageConstPtr& image){//const , const nav_msgs::OdometryConstPtr& odom , const sensor_msgs::ImageConstPtr& image){
	
	if (visual_publish == true){
		
	// 	// Visual Feedback poses
	// 	visualFB.RobotPoseID = feedback.at(feedback_id).at(0);
	// 	visualFB.MapPoseID = feedback.at(feedback_id).at(1);
	// 	amcl_pub.publish(amcl_pose_map);
	// 	visual_publish = false;
	// 	feedback_id += 1;
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	cv_ptr ->image;
	ros::Time time = image ->header.stamp;
	cv::Mat img = cv_bridge::toCvCopy(image,"bgr8")->image;
	std::string dir = "/home/lupus/catkin_ws/src/munans_localization/src/Image_Data/poses/";
	cv::imwrite(dir + "pose" + std::to_string(feedback_id) + "(" + std::to_string(time.sec) + ").jpg",img);
	cv::imwrite(dir + "pose" + std::to_string(feedback_id) + ".jpg",img);
	visual_publish = false;
	feedback_id += 1;

	};
	
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	

int main(int argc, char **argv){

    ros::init(argc, argv,"odometer");
    ros::NodeHandle Odometer;

    corrected_path_pub = Odometer.advertise<nav_msgs::Path>("path_corrected",1000);
	odometer_pub = Odometer.advertise<geometry_msgs::PoseStamped>("factor_graph/odometer",1000);
	initial_pub = Odometer.advertise<geometry_msgs::PoseStamped>("factor_graph/accelerometer",1000);
	amcl_pub = Odometer.advertise<geometry_msgs::PoseStamped>("factor_graph/trajectory",1000);
	// visual_feedback_pub = Odometer.advertise<munans_localization::EssentialMatrixConstraint>("factor_graph/visual_feedback",1000);
	time_stamp_pub = Odometer.advertise<std_msgs::Header>("time_stamp",1000);

    ros::Rate loop_rate(loop_rate);

	amcl_pose_map.header.frame_id = "map";

	last_pose_map.header.frame_id = "map";
	last_pose_map.pose.position.x = 0.0;
	last_pose_map.pose.position.y = 0.0;
	last_pose_map.pose.position.z = 0.0;
	last_pose_map.pose.orientation.x = 0.0;
	last_pose_map.pose.orientation.y = 0.0;
	last_pose_map.pose.orientation.z = 0.0;
	last_pose_map.pose.orientation.w = 1.0;

    Odom_log.info("Odometer launched!");

  while(Odometer.ok()){

	odometer_sub = Odometer.subscribe("pioneer1/odom", 1000, odometer_callback);
	amcl_sub = Odometer.subscribe("amcl_pose", 1000, amcl_callback);
	visual_sub = Odometer.subscribe("/camera_360/image_raw", 1000,visual_callback);
    ros::spin();
	// message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amcl_sub(Odometer, "amcl_pose", 1000);
	// message_filters::Subscriber<nav_msgs::Odometry> odometer_sub(Odometer, "pioneer1/odom", 1000);
	// message_filters::Subscriber<sensor_msgs::Image> image_sub(Odometer, "/camera_360/image_raw", 1000);
	// // message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(Odometer, "camera_info", 1);
	// // message_filters::TimeSynchronizer<nav_msgs::Odometry,geometry_msgs::PoseWithCovariance , sensor_msgs::Image, sensor_msgs::CameraInfo> sync(odometer_pub, amcl_sub, image_sub, info_sub, 10);
	// message_filters::TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry, sensor_msgs::Image> sync(amcl_sub, odometer_sub, image_sub, 2000);

	// // typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry, sensor_msgs::Image> MySyncPolicy;
  	// // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  	// // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(2000), amcl_sub, odometer_sub, image_sub);
	// sync.registerCallback(&odometer_callback);

	// ros::spin();

  }
}