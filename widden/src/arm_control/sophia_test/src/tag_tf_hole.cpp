#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using namespace std;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_hole_pub");
  // tag0_pose.open ("world_pose.txt", std::ofstream::out | std::ofstream::trunc);
  // tag1_pose.open ("tag_pose.txt", std::ofstream::out | std::ofstream::trunc);
  // tag1_pose_in_world.open ("tag_pose_in_world.txt", std::ofstream::out | std::ofstream::trunc);

  ros::NodeHandle n;
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  tf::Quaternion q_tag9tohole;
  q_tag9tohole.setRPY(-M_PI/2,0,0);
  tf::Vector3 v_tag9tohole(0,0.075,-0.044);
  // q_tag9tohole.setRPY(0.0,M_PI/2,0);
  // tf::Vector3 v_tag9tohole(0.078,0.0,-0.043);
  tf::Transform T_tag9tohole(q_tag9tohole,v_tag9tohole);
  ros::Rate r(30);
  while(ros::ok()){
	broadcaster.sendTransform(tf::StampedTransform(T_tag9tohole,ros::Time::now(),"tag9", "hole"));
  r.sleep();
  }

  return 0;
}

// broadcaster.sendTransform(tf::StampedTransform
// 	(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),ros::Time::now(),"base_link", "base_laser"));