#include "ros/ros.h"    //include ros 中所有必要的header
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose.h"
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <cstdlib> // 包括 srand 和 rand 函数
#include <ctime>   // 包括 time 函数

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "ukf_testing"); //初始化ros與指定node名稱（要唯一）

    ros::NodeHandle n; //創見此node的handle（會初始化節點）


    ros::Publisher test_topic_pub = n.advertise<nav_msgs::Odometry>("test_topic", 1000);
    //於chatter(Topic)上發布type為std_msgs/String的msg
    //1000為在開始丟棄舊消息前緩衝最多1000條消息(防止發布太快）
    //讓master通知所有訂閱chatter topic的node我們將發布msg

    ros::Rate loop_rate(30);
    //根據指定的頻率(10Hz)循環,會自動紀錄上次使用Rate::sleep()時間,並休眠至正確時間

    static tf::TransformBroadcaster new_broadcaster, odom_broadcaster;

    double count = 0;
    while (ros::ok())
    {
    //ros::ok()回傳false的條件:收到Ctrl-C, 被另一個同名節點踢出網絡, ros::shutdown()被呼叫, 
    //所有NodeHandles被摧毀

    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    // double randomValue = static_cast<double>(std::rand()) ;
    int randomNumber = std::rand() % 11;

    count = count +0.05;

    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = ros::Time::now();
    odometry_msg.header.frame_id = "world";
    odometry_msg.child_frame_id = "test1";
    geometry_msgs::Pose output;
    output.position.x = 0;
    output.position.y = 0;
    output.position.z = count;
    output.orientation.w = 1;
    output.orientation.x = 0;
    output.orientation.y = 0;
    output.orientation.z = 0;
    odometry_msg.pose.pose = output;
    // 越大越相信量測值（給太小會讓predict出來的值影響現在的state, 導致現在的state被帶走, 讓之後sensor input 皆變成outliner, 無法做更新, 最終變成nan與err)
    // odom_frame 與 base_link_frame改成一樣對這邊無影響
    odometry_msg.pose.covariance = {1E-3,0, 0, 0, 0, 0,
                                0, 1E-3, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 1E-5, 0, 0,
                                0, 0, 0, 0, 1E-5, 0,
                                0, 0, 0, 0, 0, 1E-5};
    odometry_msg.twist.twist.linear.z = 0;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, count) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    new_broadcaster.sendTransform(tf::StampedTransform(transform, odometry_msg.header.stamp, "world", "test1"));
    odom_broadcaster.sendTransform(tf::StampedTransform(transform, odometry_msg.header.stamp, "world", "test_odom"));
    test_topic_pub.publish(odometry_msg);


    ROS_INFO("Position (x, y, z): %f, %f, %f", odometry_msg.pose.pose.position.x, odometry_msg.pose.pose.position.y, odometry_msg.pose.pose.position.z);
    ROS_INFO("Orientation (x, y, z, w): %f, %f, %f, %f", odometry_msg.pose.pose.orientation.x, odometry_msg.pose.pose.orientation.y, odometry_msg.pose.pose.orientation.z, odometry_msg.pose.pose.orientation.w);
    ROS_INFO("Linear Velocity (x, y, z): %f, %f, %f", odometry_msg.twist.twist.linear.x, odometry_msg.twist.twist.linear.y, odometry_msg.twist.twist.linear.z);
    ROS_INFO("Angular Velocity (x, y, z): %f, %f, %f", odometry_msg.twist.twist.angular.x, odometry_msg.twist.twist.angular.y, odometry_msg.twist.twist.angular.z);
    loop_rate.sleep();
    //配合上面的ros_rate使其休眠到特定時間(10hz)
    }


    return 0;
}