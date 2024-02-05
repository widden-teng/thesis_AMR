#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "apriltags_ros/AprilTagDetectionArray.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

double tm_target_p[6];

void targetA_cb(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
{
    tm_target_p[0] = TargetPose->pose.position.x;
    tm_target_p[1] = TargetPose->pose.position.y;
    tm_target_p[2] = TargetPose->pose.position.z;

    tf::Quaternion q(
        TargetPose->pose.orientation.x,
        TargetPose->pose.orientation.y,
        TargetPose->pose.orientation.z,
        TargetPose->pose.orientation.w);
    tf::Matrix3x3 m(q);
    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

    ROS_INFO("targetA = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3],tm_target_p[4],tm_target_p[5]);

}

void drillup_cb(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
{
    tm_target_p[0] = TargetPose->pose.position.x;
    tm_target_p[1] = TargetPose->pose.position.y;
    tm_target_p[2] = TargetPose->pose.position.z;

    tf::Quaternion q(
        TargetPose->pose.orientation.x,
        TargetPose->pose.orientation.y,
        TargetPose->pose.orientation.z,
        TargetPose->pose.orientation.w);
    tf::Matrix3x3 m(q);
    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);

    ROS_INFO("drillup = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3],tm_target_p[4],tm_target_p[5]);

}

void drilldown_cb(const geometry_msgs::PoseStamped::ConstPtr& TargetPose)
{
    tm_target_p[0] = TargetPose->pose.position.x;
    tm_target_p[1] = TargetPose->pose.position.y;
    tm_target_p[2] = TargetPose->pose.position.z;

    tf::Quaternion q(
        TargetPose->pose.orientation.x,
        TargetPose->pose.orientation.y,
        TargetPose->pose.orientation.z,
        TargetPose->pose.orientation.w);
    tf::Matrix3x3 m(q);
    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
    m.getRPY(tm_target_p[3], tm_target_p[4], tm_target_p[5]);
    ROS_INFO("drilldown = %f %f %f %f %f %f\n",tm_target_p[0],tm_target_p[1],tm_target_p[2],tm_target_p[3],tm_target_p[4],tm_target_p[5]);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "BT300_pose_estimation");
    ros::NodeHandle node_handle;

    ROS_INFO("Subscribe /targetA");
    ros::CallbackQueue tool_pose;
    // ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/tool_pose", 10, ToolPose_Callback, ros::VoidPtr(), &tool_pose);
    ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/targetA", 10, targetA_cb, ros::VoidPtr(), &tool_pose);
    ros::Subscriber sub_toolpose = node_handle.subscribe(ops_toolpose);
    ros::AsyncSpinner async_spinner_toolpose(1, &tool_pose);
    async_spinner_toolpose.start();

    ROS_INFO("Subscribe /drillup");
    ros::CallbackQueue robot_target;
    ros::SubscribeOptions ops_robot_target = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/drillup", 10, drillup_cb, ros::VoidPtr(), &robot_target);
    ros::Subscriber sub_robot_target = node_handle.subscribe(ops_robot_target);
    ros::AsyncSpinner async_spinner_robot_target(1, &robot_target);
    async_spinner_robot_target.start();

    ROS_INFO("Subscribe /drilldown");
    ros::CallbackQueue start_move;
    ros::SubscribeOptions ops_start_move = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/drilldown", 10, drilldown_cb, ros::VoidPtr(), &start_move);
    ros::Subscriber sub_start_move = node_handle.subscribe(ops_start_move);
    ros::AsyncSpinner async_spinner_start_move(1, &start_move);
    async_spinner_start_move.start();
  
  ros::spin();
  return 0;
}