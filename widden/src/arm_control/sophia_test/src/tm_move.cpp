// picking/src/tm_DL_binpicking/src/pose_estimation.cpp
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <vector>
#include <time.h>
#include <map>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>

//TM5 Driver
#include "tm_driver/tm_print.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_communication.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_ros_node.h"
#include "tm_kinematics/tm_kin.h"

#include  <tm_msgs/FeedbackState.h>

#define D2R 0.01745329252
#define R2D 57.29577951
#define NUMBER_OF_DOFS    6
using namespace std;

double tm_end_effector_p[6];// x,y,z(m),rx,ry,rz(rad)
double tm_end_effector_j[6];
Eigen::Matrix<double,4,4> T_tm_end_effector;

void ToolPose_Callback(const geometry_msgs::PoseStamped::ConstPtr& ToolPose)
{
    tm_end_effector_p[0] = ToolPose->pose.position.x;
    tm_end_effector_p[1] = ToolPose->pose.position.y;
    tm_end_effector_p[2] = ToolPose->pose.position.z;

    tf::Quaternion q(
        ToolPose->pose.orientation.x,
        ToolPose->pose.orientation.y,
        ToolPose->pose.orientation.z,
        ToolPose->pose.orientation.w);
    tf::Matrix3x3 m(q);

    T_tm_end_effector <<   0., 0., 0., tm_end_effector_p[0],
            0., 0., 0., tm_end_effector_p[1],
            0., 0., 0., tm_end_effector_p[2],
            0., 0., 0., 1.;
    Eigen::Matrix<double,3,3> rot;
    // tf::matrixTFToEigen(m,rot);
    // T_tm_end_effector.block<3,3>(0,0) = rot.block<3,3>(0,0);
    //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
    m.getRPY(tm_end_effector_p[3], tm_end_effector_p[4], tm_end_effector_p[5]);


}

void ToolPose_Callback2(const tm_msgs::FeedbackState::ConstPtr& ToolPose)
{
    tm_end_effector_p[0] = ToolPose->tool_pose[0];
    tm_end_effector_p[1] = ToolPose->tool_pose[1];
    tm_end_effector_p[2] = ToolPose->tool_pose[2];
    tm_end_effector_p[3] = ToolPose->tool_pose[3];
    tm_end_effector_p[4] = ToolPose->tool_pose[4];
    tm_end_effector_p[5] = ToolPose->tool_pose[5];

    // tf::Quaternion q(
    //     ToolPose->pose.orientation.x,
    //     ToolPose->pose.orientation.y,
    //     ToolPose->pose.orientation.z,
    //     ToolPose->pose.orientation.w);
    // tf::Matrix3x3 m(q);

    // T_tm_end_effector <<   0., 0., 0., tm_end_effector_p[0],
    //         0., 0., 0., tm_end_effector_p[1],
    //         0., 0., 0., tm_end_effector_p[2],
    //         0., 0., 0., 1.;
    // Eigen::Matrix<double,3,3> rot;
    // // tf::matrixTFToEigen(m,rot);
    // // T_tm_end_effector.block<3,3>(0,0) = rot.block<3,3>(0,0);
    // //Roll: rx, Pitch: ry, Yaw: rz, unit: rad
    // m.getRPY(tm_end_effector_p[3], tm_end_effector_p[4], tm_end_effector_p[5]);

}

bool CheckJointLimit(double *q){
    bool valid = true;

    if(abs(q[0]) > 270*D2R)
    {
        print_warn("[WARN] the 1th joint : %lf\n",q[0] );
        valid = false;
    }
    else if(abs(q[1]) > 1.57)
    {
        print_warn("[WARN] the 2th joint : %lf\n",q[1] );
        valid = false;
    }
    else if(abs(q[2]) > 155*D2R)
    {
        print_warn("[WARN] the 3th joint : %lf\n",q[2] );
        valid = false;
    }
    else if(abs(q[3]) > 180*D2R)
    {
        print_warn("[WARN] the 4th joint : %lf\n",q[3] );
        valid = false;
    }
    else if(abs(q[4]) > 180*D2R)
    {
        print_warn("[WARN] the 5th joint : %lf\n",q[4] );
        valid = false;
    }
    else if(abs(q[5]) > 270*D2R)
    {
        print_warn("[WARN] the 6th joint : %lf\n",q[5] );
        valid = false;
    }
    else
        valid = true;

    return valid;
}

bool GetQfromInverseKinematics( double* CartesianPosition, double *q_inv)
{
    Eigen::Matrix<float,4,4> T_;
    Eigen::AngleAxisf yawAngle (CartesianPosition[5], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitchAngle  (CartesianPosition[4], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rollAngle(CartesianPosition[3], Eigen::Vector3f::UnitX());
    Eigen::Quaternion<float> q = yawAngle * pitchAngle *rollAngle;
    Eigen::Matrix<float,3,3> RotationMatrix = q.matrix();
    double *T = new double[16];


    T_ <<   0., 0., 0., CartesianPosition[0],
            0., 0., 0., CartesianPosition[1],
            0., 0., 0., CartesianPosition[2],
            0., 0., 0., 1.;

    T_.block<3,3>(0,0) = RotationMatrix.block<3,3>(0,0);

    tm_jacobian::Matrix2DoubleArray(T_,T);
    int num_sol =  tm_kinematics::inverse(T, q_inv);

    delete [] T;
    return CheckJointLimit(q_inv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_estimation");
    ros::NodeHandle node_handle;

    ROS_INFO("Subscribe /tool_position");
    ros::CallbackQueue tool_pose;
    ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>("/tool_pose", 10, ToolPose_Callback, ros::VoidPtr(), &tool_pose);
    // ros::SubscribeOptions ops_toolpose = ros::SubscribeOptions::create<tm_msgs::FeedbackState>("/feedback_states", 10, ToolPose_Callback2, ros::VoidPtr(), &tool_pose);
    
    ros::Subscriber sub_toolpose = node_handle.subscribe(ops_toolpose);
    ros::AsyncSpinner async_spinner_toolpose(1, &tool_pose);
    async_spinner_toolpose.start();

    ros::ServiceClient client = node_handle.serviceClient<tm_msgs::SetPositions>("tm_driver/set_positions");
    tm_msgs::SetPositions target_pose;

    // ros::Publisher pub_estimation;
    ros::Rate r(10);
    int valid=0;
    while(valid!=1){
    // while(ros::ok()){
        if (GetQfromInverseKinematics(tm_end_effector_p, tm_end_effector_j)){
            printf("tp=%lf %lf %lf %lf %lf %lf\n",tm_end_effector_p[0],tm_end_effector_p[1],tm_end_effector_p[2],tm_end_effector_p[3],tm_end_effector_p[4],tm_end_effector_p[5]);
            printf("j=%lf %lf %lf %lf %lf %lf\n",tm_end_effector_j[0],tm_end_effector_j[1],tm_end_effector_j[2],tm_end_effector_j[3],tm_end_effector_j[4],tm_end_effector_j[5]);
            double T[16];

            tm_kinematics::forward(tm_end_effector_j, T);
            cout << ">>>> T06" << endl;
            tm_jacobian::printMatrix(T,4,16);
        }
        std::cin>>valid;
    }
    std::vector<double> target = {-0.029867, -0.847103, 2.171225, -1.250073, 1.664053, -0.020084};
    double tp[6];
    for(int i=0;i<6;i++){
        tp[i]=tm_end_effector_p[i];
    }
    tp[0]=tp[0]-0.05;
    tp[1]=tp[1]+0.05;
    if (GetQfromInverseKinematics(tp, tm_end_effector_j)){
            printf("j=%lf %lf %lf %lf %lf %lf\n",tm_end_effector_j[0],tm_end_effector_j[1],tm_end_effector_j[2],tm_end_effector_j[3],tm_end_effector_j[4],tm_end_effector_j[5]);
            double T[16];
            tm_kinematics::forward(tm_end_effector_j, T);
            cout << ">>>> T06" << endl;
            tm_jacobian::printMatrix(T,4,16);
    }
    for(int i=0;i<6;i++){
        // target[i]=tm_end_effector_j[i];
        target[i]=tp[i];
    }
    // target_pose.request.motion_type = tm_msgs::SetPositions::Request::PTP_J;
    target_pose.request.motion_type = tm_msgs::SetPositions::Request::LINE_T;
    target_pose.request.positions=target;
    target_pose.request.velocity = 0.5;//rad/s
    target_pose.request.acc_time = 0.2;
    target_pose.request.blend_percentage = 0;
    target_pose.request.fine_goal  = false;

    if (client.call(target_pose))                             
    {
    if (target_pose.response.ok) ROS_INFO_STREAM("SetPositions to robot");
    else ROS_WARN_STREAM("SetPositions to robot , but response not yet ok ");
    }
    else
    {
    ROS_ERROR_STREAM("Error SetPositions to robot");
    return 1;
    }
    return 0;
}
