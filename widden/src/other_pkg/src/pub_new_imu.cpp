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
tf::StampedTransform init_transform;
// tf::Vector3 translation ;
// tf::Quaternion rotation ;
// tf::Transform T_init;

void imu_new_handpose_Callback(const sensor_msgs::Imu &msg){


    tf::TransformListener imu_init_listener;
    static tf::TransformBroadcaster imu_init_broadcaster;
    ros::Rate rate(3.0);
    tf::Vector3 translation ;
    tf::Quaternion rotation ;
    tf::Transform T_init;


    while (ros::ok() & (!get_tf) & (!get_tf_once)){
      try{
        // 好像反了
        imu_init_listener.lookupTransform("/imu_ori", "/handpose_init",
                                ros::Time(0), init_transform);
        // imu_init_listener.lookupTransform("/imu_ori", "/camera_link",
        //                         ros::Time(0), init_transform);
        translation = init_transform.getOrigin();
        rotation = init_transform.getRotation();
        T_init.setOrigin(translation);
        T_init.setRotation(rotation);
        get_tf = true;
        get_tf_once = true;
        cout<<"first pub finished"<<endl;
      }
      //
      
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      rate.sleep();
    }
    try{
      // ros::Duration duration(1.0);
      // duration.sleep();
      // T_init.setIdentity();
      imu_init_broadcaster.sendTransform(tf::StampedTransform(T_init, ros::Time::now(), "imu_ori", "handpose_IMU"));
      cout<<"pubing handpose_imu!!!!!!!"<< endl;

    }

    catch (...) {
        cout<<"pub IMU_init went wrong"<< endl;
    }
    // ros::Duration duration(1.0);
    // duration.sleep();
    // T_init.setIdentity();
    // imu_init_broadcaster.sendTransform(tf::StampedTransform(T_init, ros::Time::now(), "camera_link", "handpose_IMU"));
    // cout<<"pubing handpose_imu!!!!!!!"<< endl;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_new_imu");
  ros::NodeHandle n_handpose;
  ros::Subscriber subimu = n_handpose.subscribe("/imu/data", 1, imu_new_handpose_Callback);

  // ros::Rate rate(10.0);
  // tf::TransformListener imu_init_listener;
  // tf::TransformBroadcaster imu_init_broadcaster;
  // tf::Transform T_init;
  // while (ros::ok()){
  //   T_init.setIdentity();
  //   imu_init_broadcaster.sendTransform(tf::StampedTransform(T_init, ros::Time::now(), "camera_link", "handpose_IMU"));
  //   cout<<"pubing handpose_imu!!!!!!!"<< endl;
  //   rate.sleep();
  // }
  ros::spin();
  return 0;
}