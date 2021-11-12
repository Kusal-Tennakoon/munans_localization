/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++	
 											HEADER FILES
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

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

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
											PARAMETER DEFINITIONS
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

Log::log Odom_log; //Log object

geometry_msgs::PoseStamped odom_pose;
ros::Subscriber odometer_sub;
ros::Publisher raw_path_pub;
// tf::TransformBroadcaster odom_broadcaster;
// geometry_msgs::TransformStamped odom_trans;
nav_msgs::Path robot_path;

int node_id = 0;
double r_x = 0.0, r_y = 0.0, r_z = 0.0, r_thx = 0.0, r_thy = 0.0, r_thz = 0.0; //Raw pose
double delta_x = 0.0, delta_y = 0.0, delta_z = 0.0, delta_thx = 0.0, delta_thy = 0.0, delta_thz = 0.0; //Raw pose
double vx = 0.0, vy = 0.0, vz = 0.0, wx = 0.0, wy = 0.0, wz = 0.0;

ros::Time current_time , last_time;


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											FUNCTION DEFINITIONS
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

void odometer_callback(const nav_msgs::Odometry::ConstPtr& msg){
	
	current_time = msg->header.stamp;
  current_time = ros::Time::now();

  // vx = msg->twist.twist.linear.x/1000;
  // vy = msg->twist.twist.linear.y/1000;
  // vz = msg->twist.twist.linear.z/1000;
  // wx = msg->twist.twist.angular.x/180*3.14;
  // wy = msg->twist.twist.angular.y/180*3.14;
  // wz = msg->twist.twist.angular.z/180*3.14;

  vx = msg->twist.twist.linear.x;
  vy = msg->twist.twist.linear.y;
  vz = msg->twist.twist.linear.z;
  wx = msg->twist.twist.angular.x;
  wy = msg->twist.twist.angular.y;
  wz = msg->twist.twist.angular.z;

  //compute odometry in a typical way given the velocities of the robot

  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(r_thy)*cos(r_thz) + vy * (sin(r_thx)*sin(r_thy)*cos(r_thz) - cos(r_thx)*sin(r_thz)) + vz * (cos(r_thx)*sin(r_thy)*cos(r_thz) + sin(r_thx)*sin(r_thz))) * dt;
  double delta_y = (vx * cos(r_thy)*sin(r_thz) + vy * (sin(r_thx)*sin(r_thy)*sin(r_thz) + cos(r_thx)*cos(r_thz)) + vz * (cos(r_thx)*sin(r_thy)*sin(r_thz) - sin(r_thx)*cos(r_thy))) * dt;
  double delta_z = (-vx * sin(r_thy) + vy * sin(r_thx)*cos(r_thy) + vz * cos(r_thx)*cos(r_thy)) * dt;
  
  double delta_thx = wx * dt;
  double delta_thy = wy * dt;
  double delta_thz = wz * dt;

  r_x += delta_x;
  r_y += delta_y;
  r_z += delta_z;
  r_thx += delta_thx;
  r_thy += delta_thy;
  r_thz += delta_thz;

  // double delta_x = (vx * cos(r_thz) - vy * sin(r_thz)) * dt;
  // double delta_y = (vx * sin(r_thz) + vy * cos(r_thz)) * dt;
  // double delta_z = 0;
  
  // double delta_thx = 0;
  // double delta_thy = 0;
  // double delta_thz = wz * dt;

  // r_x += delta_x;
  // r_y += delta_y;
  // // r_z += delta_z;
  // // r_thx += delta_thx;
  // // r_thy += delta_thy;
  // r_thz += delta_thz;


 odom_pose.header.frame_id = "map";
 odom_pose.pose.position.x = r_x;
 odom_pose.pose.position.y = r_y;
 odom_pose.pose.position.z = r_z;
 odom_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r_thx,r_thy,r_thz);

  // //since all odometry is 6DOF we'll need a quaternion created from yaw
  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(r_thx,r_thy,r_thz);

  // odom_trans.header.stamp = current_time;
  // odom_trans.header.frame_id = "odom";
  // odom_trans.child_frame_id = "base_link";

  // odom_trans.transform.translation.x = r_x;
  // odom_trans.transform.translation.y = r_y;
  // odom_trans.transform.translation.z = r_z;
  // odom_trans.transform.rotation = odom_quat;

  // //send the transform
  // odom_broadcaster.sendTransform(odom_trans);

	robot_path.header.stamp = ros::Time::now();
	robot_path.header.frame_id = "map";
	robot_path.poses.push_back(odom_pose);


  //publish the message
  raw_path_pub.publish(robot_path);

  last_time = current_time;


}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/	
// 											MAIN LOOP BEGINS HERE
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



int main(int argc, char** argv){

  ros::init(argc, argv,"odometer");
	ros::NodeHandle Odometer;

  raw_path_pub = Odometer.advertise<nav_msgs::Path>("path_raw", 1000);
  // odom_broadcaster;

	ros::Rate loop_rate(loop_rate);
	
	Odom_log.info("Odometer launched!");

  while(Odometer.ok()){

    odometer_sub = Odometer.subscribe("pioneer1/odom", 1000, odometer_callback);
    ros::spin();
    // loop_rate.sleep();

  }
}