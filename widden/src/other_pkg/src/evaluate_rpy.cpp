#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>




using namespace std;
// global variable
bool get_tf = false;
bool get_tf_once = false;
tf::StampedTransform transform_inittocamera;
tf::Transform T_inittocamera, T_test;
// tf::Vector3 translation ;
// tf::Quaternion rotation ;
// tf::Transform T_init;

void calculate_rpy_Callback(const std_msgs::String::ConstPtr& msg){
  tf::TransformListener handposeinit_to_cameralink;
  ros::Rate rate(3.0);
  tf::Vector3 translation_inittocamera ;
  tf::Quaternion rotation_inittocamera ;

  while (ros::ok() & (!get_tf_once)){
    try{
      // handposeinit_to_cameralink.lookupTransform("/camera_link", "/handpose_init",
      //                         ros::Time(0), transform_inittocamera);
      handposeinit_to_cameralink.lookupTransform("/camera_link", "/handpose_IMU",
                              ros::Time(0), transform_inittocamera);
      translation_inittocamera = transform_inittocamera.getOrigin();
      rotation_inittocamera = transform_inittocamera.getRotation();
      T_inittocamera.setOrigin(translation_inittocamera);
      T_inittocamera.setRotation(rotation_inittocamera);


      tf::Matrix3x3 rotation_matrix(rotation_inittocamera);
      double roll, pitch, yaw;
      rotation_matrix.getRPY(roll, pitch, yaw);
      ROS_INFO("handpose init of Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
      get_tf_once = true;


      // 這邊是用來測試前*還後*
      // T_test.setIdentity();
      // tf::Transform T_rel_A_to_B = T_inittocamera * T_test;
      // tf::Matrix3x3 rotation_matrix(rotation_inittocamera);
      // double roll, pitch, yaw;
      // tf::Matrix3x3(T_rel_A_to_B.getRotation()).getRPY(roll, pitch, yaw);

      // ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);

      // get_tf_once = true;

    }
    //
    
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    rate.sleep();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "evaluate_rpy");
  ros::NodeHandle n_handpose;
  ros::Subscriber subimu = n_handpose.subscribe("start_calculate_rpy", 1, calculate_rpy_Callback);
  ros::Rate rate(3.0);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}