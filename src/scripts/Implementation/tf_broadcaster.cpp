#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle tf;

  ros::Rate loop_rate(100);

  tf::TransformBroadcaster broadcaster;

  while(tf.ok()){
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.43, 0.0, 0.2175)),ros::Time::now(),"pioneer1/base_link", "pioneer1/base_laser"));
    //broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.43, 0.0, 0.2175)),ros::Time::now(),"pioneer1/base_link", "pioneer1/base_laser"));
    loop_rate.sleep();
  }
}
