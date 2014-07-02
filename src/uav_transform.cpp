#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry uav_pose;



int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;


  ros::Publisher des_pose =
       node.advertise<nav_msgs::Odometry>("uav_pose", 100);

  tf::TransformListener listener;

  ros::Rate rate(100.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/world","/vicon/CrazyFlie_1/Body",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }


    rate.sleep();
  }
  return 0;
};
